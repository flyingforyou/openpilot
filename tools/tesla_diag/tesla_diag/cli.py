"""tesla-diag command line. Offline commands never open a port; live commands say what they do."""
from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict

from tesla_diag.can import logger
from tesla_diag.profiles import Profile, ProfileError, load_profile
from tesla_diag.transport.elm327 import PROTOCOLS, DongleError, Elm327, SerialTransport
from tesla_diag.uds.client import SafetyRefusal, UdsClient, build_read_did, build_read_dtc
from tesla_diag.uds.responses import UdsError, parse_dtcs, parse_read_did


def _open(port: str, baud: int) -> Elm327:
  return Elm327(SerialTransport(port, baud))


def mask_vin(vin: str) -> str:
  return vin[:3] + '*' * max(0, len(vin) - 7) + vin[-4:] if len(vin) > 7 else '*' * len(vin)


# ---- probe -------------------------------------------------------------------------------------
def cmd_probe(a, dongle: Elm327 | None = None) -> int:
  d = dongle or _open(a.port, a.baud)
  try:
    caps = d.probe()
  except DongleError as e:
    print(f'probe failed: {type(e).__name__}: {e}', file=sys.stderr)
    return 2
  finally:
    d.close()
  print('AT-layer probe (no OBD/UDS request was sent)')
  print(f'  version      : {caps.version}')
  print(f'  description  : {caps.device or "-"}')
  print(f'  battery volt : {caps.voltage or "-"}')
  for c, st in caps.commands.items():
    print(f'  {c:<9}: {st}')
  print(f'  silent monitoring (ATCSM1): {"CONFIRMED" if caps.silent_monitoring else "NOT confirmed -> sniff will refuse"}')
  print('  note: a clone can answer OK without implementing a command; it cannot be proven from here')
  return 0 if caps.silent_monitoring else 1


# ---- sniff -------------------------------------------------------------------------------------
def cmd_sniff(a, dongle: Elm327 | None = None) -> int:
  if a.protocol not in PROTOCOLS:
    print(f'--protocol must be one of {sorted(PROTOCOLS)}: {PROTOCOLS}', file=sys.stderr)
    return 2
  desc = PROTOCOLS[a.protocol]
  meta = {'source': 'live', 'protocol': a.protocol, 'id_format': desc[1], 'bitrate_kbps': desc[2],
          'port': a.port, 'note': 'bitrate/id format were chosen by the operator, not detected'}
  d = dongle or _open(a.port, a.baud)
  frames = bad = status_n = 0
  warns: Counter = Counter()
  try:
    d.reset()
    d.setup_display()
    with logger.JsonlWriter(a.out, meta) as w:
      for p in d.monitor(a.protocol, a.seconds):
        if p.frame:
          w.write(p.frame)
          frames += 1
        elif p.status:
          status_n += 1
          warns[p.status] += 1
        elif p.garbage:
          bad += 1
  except DongleError as e:
    print(f'sniff failed: {type(e).__name__}: {e}', file=sys.stderr)
    return 2
  finally:
    d.close()
  print(f'{frames} frames -> {a.out}  (unparsed lines: {bad}, dongle messages: {dict(warns) or "none"})')
  if warns['BUFFER FULL']:
    print('WARNING: dongle buffer overflowed; frames were lost. Bluetooth + ELM327 cannot keep up with a busy bus.')
  if not frames:
    print('WARNING: nothing heard -- wrong bus/bitrate, wrong adapter wiring, or the car is asleep.')
  return 0


# ---- replay ------------------------------------------------------------------------------------
def summarise(frames, bucket: float | None = None, known: set[int] | None = None) -> dict:
  per: dict[tuple[int, bool], list] = defaultdict(list)
  for fr in frames:
    per[(fr.id, fr.extended)].append(fr)
  span = (max(f.ts for f in frames) - min(f.ts for f in frames)) if frames else 0.0
  rows = []
  for (i, ext), fs in sorted(per.items()):
    ts = [f.ts for f in fs]
    rows.append({'id': f'{i:08X}' if ext else f'{i:03X}', 'n': len(fs), 'hz': round(len(fs) / span, 1) if span else None,
                 'dlc': sorted({f.dlc for f in fs}), 'last': fs[-1].data.hex(' ').upper(),
                 'known': None if known is None else (i in known), 'first': min(ts), 'lastts': max(ts)})
  hist = None
  if bucket and frames:
    t0 = min(f.ts for f in frames)
    hist = defaultdict(Counter)
    for f in frames:
      hist[int((f.ts - t0) // bucket)][f.id_hex] += 1
  return {'frames': len(frames), 'span': round(span, 3), 'ids': rows, 'hist': hist}


def cmd_replay(a) -> int:
  try:
    meta, frames = logger.load_any(a.file)
  except (OSError, ValueError, KeyError) as e:
    print(f'cannot read {a.file}: {e}', file=sys.stderr)
    return 2
  if meta.get('SYNTHETIC'):
    print('*** SYNTHETIC FIXTURE: not a recording from any car ***')
  known = None
  if a.profile:
    known = {s.msg_id for s in load_profile(a.profile).signals}
  s = summarise(frames, a.bucket, known)
  print(f'{s["frames"]} frames over {s["span"]} s, {len(s["ids"])} ids')
  if a.summary or not (a.csv or a.candump):
    print(f'{"ID":>9} {"N":>6} {"Hz":>7}  DLC   LAST' + ('   KNOWN' if known is not None else ''))
    for r in s['ids']:
      k = '' if r['known'] is None else ('   yes' if r['known'] else '   UNKNOWN')
      print(f'{r["id"]:>9} {r["n"]:>6} {r["hz"] if r["hz"] is not None else "-":>7}  {",".join(map(str, r["dlc"])):<4}  {r["last"]}{k}')
  if s['hist'] is not None:
    for b in sorted(s['hist']):
      print(f't+{b * a.bucket:g}s: ' + ' '.join(f'{k}x{v}' for k, v in sorted(s['hist'][b].items())))
  if a.csv:
    logger.write_csv(frames, a.csv)
    print('csv ->', a.csv)
  if a.candump:
    with open(a.candump, 'w') as f:
      f.writelines(ln + '\n' for ln in logger.candump_lines(frames))
    print('candump ->', a.candump)
  return 0


# ---- decode ------------------------------------------------------------------------------------
def cmd_decode(a) -> int:
  try:
    prof = load_profile(a.profile)
    meta, frames = logger.load_any(a.file)
  except (ProfileError, OSError, ValueError) as e:
    print(e, file=sys.stderr)
    return 2
  if meta.get('SYNTHETIC'):
    print('*** SYNTHETIC FIXTURE: values below are made up ***')
  if not prof.signals:
    print('profile defines no signals', file=sys.stderr)
    return 2
  res = prof.signal_table().decode_frames(frames)
  for name, e in sorted(res.items()):
    print(f'{name:<28} last {e["last"]:>10.3f} {e["unit"]:<4} n={e["n"]:<6} min {e["min"]:.3f} max {e["max"]:.3f}  [{e["status"]}]')
  missing = [s.name for s in prof.signals if s.name not in res]
  if missing:
    print('no data for:', ', '.join(missing))
  if any(e['status'] != 'verified' for e in res.values()):
    print('note: values marked other than [verified] are layouts from public sources, not confirmed on your car;'
          ' compare with Scan My Tesla before trusting them')
  return 0


# ---- uds-read ----------------------------------------------------------------------------------
def _payload(a) -> bytes:
  if a.dtc:
    return build_read_dtc(0x02, int(a.dtc_mask, 16))
  if not a.did:
    raise SafetyRefusal('give --did HEX or --dtc')
  return build_read_did(int(a.did, 16))


def cmd_uds_read(a, dongle: Elm327 | None = None) -> int:
  try:
    prof: Profile = load_profile(a.profile)
    payload = _payload(a)
  except (ProfileError, SafetyRefusal, ValueError) as e:
    print(f'refused: {e}', file=sys.stderr)
    return 2
  client = UdsClient(prof, None, a.allow_uds_read, a.confirm_parked, a.max_requests, a.p2, a.audit_log)
  if a.dry_run:
    dr = client.dry_run(payload)
    print('DRY RUN: nothing is sent, no port is opened')
    print(f'  request : {dr.request.hex(" ").upper()}')
    print(f'  tx/rx   : {"-" if dr.tx_id is None else f"{dr.tx_id:X}"} / {"-" if dr.rx_id is None else f"{dr.rx_id:X}"}')
    print(f'  profile : {dr.note}')
    return 0
  try:
    client.preflight(payload)      # every gate runs before a port is opened
  except SafetyRefusal as e:
    print(f'refused: {e}', file=sys.stderr)
    client._audit('refused', req=payload.hex().upper(), why=str(e))
    return 3
  d = dongle or _open(a.port, a.baud)
  client.dongle = d
  try:
    d.reset()
    d.setup_display()
    resp = client.read(payload)
  except (DongleError, SafetyRefusal, UdsError) as e:
    print(f'uds-read failed: {type(e).__name__}: {e}', file=sys.stderr)
    return 2
  finally:
    d.close()
  if resp.kind == 'positive':
    if payload[0] == 0x22:
      data = parse_read_did(resp, int.from_bytes(payload[1:3], 'big'))
      text = data.decode('ascii', 'replace') if payload[1:3] == b'\xf1\x90' else ''
      if text and not a.show_vin:
        text = mask_vin(text)
      print(f'positive: DID {payload[1:3].hex().upper()} = {data.hex(" ").upper()}' + (f'  ({text})' if text else ''))
    else:
      for r in parse_dtcs(resp):
        print(f'DTC {r["dtc"]} status 0x{r["status"]:02X}')
  else:
    print(f'{resp.kind}: service {resp.sid:02X} NRC 0x{resp.nrc:02X} {resp.nrc_name}')
  return 0


def build_parser() -> argparse.ArgumentParser:
  ap = argparse.ArgumentParser(prog='tesla-diag', description=__doc__)
  sub = ap.add_subparsers(dest='cmd', required=True)

  def live(p):
    p.add_argument('--port', required=True, help='e.g. /dev/rfcomm0')
    p.add_argument('--baud', type=int, default=115200)

  p = sub.add_parser('probe', help='AT-layer check of the dongle; no vehicle request')
  live(p)
  p.set_defaults(fn=cmd_probe)
  p = sub.add_parser('sniff', help='listen-only capture; refuses without ATCSM1')
  live(p)
  p.add_argument('--protocol', required=True, help='ELM protocol 6/7/8/9; no default on purpose')
  p.add_argument('--seconds', type=float, default=60)
  p.add_argument('--out', required=True)
  p.set_defaults(fn=cmd_sniff)
  p = sub.add_parser('replay', help='summarise a capture offline')
  p.add_argument('file')
  p.add_argument('--summary', action='store_true')
  p.add_argument('--profile', help='mark ids the profile does not define as UNKNOWN')
  p.add_argument('--bucket', type=float, help='per-N-seconds id counts')
  p.add_argument('--csv')
  p.add_argument('--candump')
  p.set_defaults(fn=cmd_replay)
  p = sub.add_parser('decode', help='decode signals defined in a profile')
  p.add_argument('file')
  p.add_argument('--profile', required=True)
  p.set_defaults(fn=cmd_decode)
  p = sub.add_parser('uds-read', help='0x22 / 0x19 only; dry-run by default behaviour of every gate')
  p.add_argument('--profile', required=True)
  p.add_argument('--did')
  p.add_argument('--dtc', action='store_true', help='ReadDTCInformation reportDTCByStatusMask')
  p.add_argument('--dtc-mask', default='FF')
  p.add_argument('--dry-run', action='store_true')
  p.add_argument('--port')
  p.add_argument('--baud', type=int, default=115200)
  p.add_argument('--allow-uds-read', action='store_true')
  p.add_argument('--confirm-parked', action='store_true')
  p.add_argument('--max-requests', type=int, default=1)
  p.add_argument('--p2', type=float, default=2.0)
  p.add_argument('--audit-log', default='logs/uds_audit.jsonl')
  p.add_argument('--show-vin', action='store_true', help='do not mask the VIN in output')
  p.set_defaults(fn=cmd_uds_read)
  return ap


def main(argv=None) -> int:
  a = build_parser().parse_args(argv)
  if a.cmd == 'uds-read' and not a.dry_run and not a.port:
    print('refused: --port is required for a live read (or use --dry-run)', file=sys.stderr)
    return 2
  try:
    return a.fn(a)
  except ProfileError as e:
    print(e, file=sys.stderr)
    return 2
  except KeyboardInterrupt:
    print('cancelled', file=sys.stderr)
    return 130


if __name__ == '__main__':
  sys.exit(main())
