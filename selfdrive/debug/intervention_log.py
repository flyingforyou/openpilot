"""Record the moments a driver took the car back, with what each controller wanted.

The question this exists to answer is "when the driver overruled us, who was right -- the stock
ACC that was driving, or openpilot?". That is normally unanswerable, because you only ever see
the mistakes of whichever controller had the car. Here it is answerable for free: plannerd runs
whether or not openpilot owns longitudinal, so its plan is computed every frame and simply not
put on the bus. Logging both alongside the driver's action gives a matched pair at the one
instant where a human graded them, at no risk, from ordinary driving.

Cheap by construction: a ring buffer of small samples, written out only on a disengagement.
"""
import json
import os
import time
from collections import deque

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

EVENT_DIR = '/data/media/0/interventions'
REALDATA = '/data/media/0/realdata'
SEGMENT_SECONDS = 60.0

SAMPLE_HZ = 20
PRE_S = 10.0            # how much lead-up to keep -- enough to see the disagreement build
POST_S = 3.0            # and enough after to see what the driver did about it
MAX_EVENTS = 200        # bounded; oldest go first

PRE_N = int(PRE_S * SAMPLE_HZ)
POST_N = int(POST_S * SAMPLE_HZ)
# How far back "openpilot was driving into this" reaches. Braking cancels the stock ACC and the
# disengage can be reported before the brake edge, so one sample is not enough.
ENGAGED_LOOKBACK_N = int(1.0 * SAMPLE_HZ)


def _cause(cs, cs_prev) -> str | None:
  """Why the driver took over. Only rising edges -- a held brake is one event, not a hundred.

  steeringDisengage is the strong one on this car: handsOnLevel >= 3 or a high angle rate
  fault. With cooperative steering on, an ordinary nudge never reaches it, so seeing it means
  the driver pushed through a controller that was already yielding.
  """
  if cs.brakePressed and not cs_prev['brakePressed']:
    return 'brake'
  if cs.steeringDisengage and not cs_prev['steeringDisengage']:
    return 'steer'
  if cs.gasPressed and not cs_prev['gasPressed']:
    return 'gas'
  return None


class InterventionLog:
  def __init__(self, params: Params | None = None):
    self.params = params or Params()
    self.buf: deque = deque(maxlen=PRE_N + POST_N)
    self.prev = {'brakePressed': False, 'steeringDisengage': False, 'gasPressed': False,
                 'enabled': False}
    self.pending: dict | None = None
    self.post_left = 0
    self.last_sample = 0.0
    try:
      os.makedirs(EVENT_DIR, exist_ok=True)
    except OSError:
      cloudlog.exception("intervention_log: cannot create event dir")

  def update(self, sm) -> None:
    """Called from the state poller. Samples at a fixed rate so the window is a known duration
    regardless of how often the caller happens to run."""
    if not sm.seen['carState']:
      return

    now = time.monotonic()
    if now - self.last_sample < 1.0 / SAMPLE_HZ:
      return
    self.last_sample = now

    cs, plan, ss = sm['carState'], sm['longitudinalPlan'], sm['selfdriveState']
    lead = sm['radarState'].leadOne

    self.buf.append({
      't': round(now, 3),
      # what openpilot would have done. Shadow -- in stock-ACC mode this never reaches the bus.
      'opAccel': round(plan.aTarget, 3),
      # what the car actually did, which is the stock controller's answer when it has the car
      'aEgo': round(cs.aEgo, 3),
      'vEgo': round(cs.vEgo, 2),
      # who could see a lead, and whether it was a physical return or a model guess
      'leadStatus': bool(lead.status),
      'leadRadar': bool(lead.radar),
      'leadDRel': round(lead.dRel, 1) if lead.status else None,
      'leadVRel': round(lead.vRel, 1) if lead.status else None,
      'steeringAngle': round(cs.steeringAngleDeg, 1),
      'engaged': bool(ss.enabled),
      'brake': bool(cs.brakePressed),
      'gas': bool(cs.gasPressed),
    })

    # finish an event already in flight before looking for the next one, so a driver who brakes
    # and then swerves does not produce two half-windows of the same moment
    if self.pending is not None:
      self.post_left -= 1
      if self.post_left <= 0:
        self._write(self.pending)
        self.pending = None
      self._remember(cs, ss)
      return

    cause = _cause(cs, self.prev)
    # Only grade a takeover if openpilot was driving into it -- a brake press offroad grades
    # nobody. Deliberately "recently engaged" rather than "engaged last sample": braking cancels
    # the stock ACC, which on a pcmCruise car drops openpilot with it, and nothing orders that
    # against the brake edge. Requiring the previous sample to still be engaged loses the event
    # whenever the disengage is reported first.
    if cause is not None and self._was_engaged():
      self.pending = {
        'wallTime': time.time(),
        'cause': cause,
        'route': self.params.get("CurrentRoute") or '',
        'stockLong': bool(self.params.get("TeslaStockLong", return_default=True)),
        'atEvent': self.buf[-1],
      }
      self.post_left = POST_N

    self._remember(cs, ss)

  def _was_engaged(self) -> bool:
    """Was openpilot driving in the moment leading up to this? Looks back over the buffer
    rather than one sample, so the order of the brake edge and the disengage does not matter."""
    recent = list(self.buf)[-ENGAGED_LOOKBACK_N:]
    return any(s['engaged'] for s in recent)

  def _remember(self, cs, ss) -> None:
    self.prev = {'brakePressed': bool(cs.brakePressed),
                 'steeringDisengage': bool(cs.steeringDisengage),
                 'gasPressed': bool(cs.gasPressed),
                 'enabled': bool(ss.enabled)}

  def _write(self, meta: dict) -> None:
    meta['samples'] = list(self.buf)
    meta['disagreement'] = self._disagreement(meta['atEvent'])
    name = time.strftime('%Y-%m-%d--%H-%M-%S', time.localtime(meta['wallTime']))
    path = os.path.join(EVENT_DIR, f"{name}--{meta['cause']}.json")
    try:
      with open(path, 'w') as f:
        json.dump(meta, f, separators=(',', ':'))
      self._rotate()
      cloudlog.warning(f"intervention recorded: {meta['cause']} disagreement={meta['disagreement']}")
    except OSError:
      cloudlog.exception("intervention_log: write failed")

  @staticmethod
  def _disagreement(at: dict) -> float:
    """How far apart the two answers were at the instant the driver acted, m/s^2.

    Negative means openpilot wanted to be slower than the car actually was -- the case worth
    looking at, since that is openpilot claiming it saw something the stock controller missed.
    """
    return round(at['opAccel'] - at['aEgo'], 3)

  @staticmethod
  def _rotate() -> None:
    try:
      files = sorted(f for f in os.listdir(EVENT_DIR) if f.endswith('.json'))
      for f in files[:-MAX_EVENTS]:
        os.remove(os.path.join(EVENT_DIR, f))
    except OSError:
      pass


def list_events(base: str = EVENT_DIR) -> list[dict]:
  """Index for the events page: metadata only, never the sample windows."""
  out = []
  try:
    names = sorted((f for f in os.listdir(base) if f.endswith('.json')), reverse=True)
  except OSError:
    return out
  for name in names:
    try:
      with open(os.path.join(base, name)) as f:
        e = json.load(f)
      at = e.get('atEvent', {})
      out.append({
        'name': name[:-5],
        'wallTime': e.get('wallTime', 0),
        'cause': e.get('cause', '?'),
        'stockLong': e.get('stockLong', False),
        'disagreement': e.get('disagreement', 0.0),
        'vEgo': at.get('vEgo'),
        'opAccel': at.get('opAccel'),
        'aEgo': at.get('aEgo'),
        'leadStatus': at.get('leadStatus'),
        'leadRadar': at.get('leadRadar'),
        'leadDRel': at.get('leadDRel'),
        'route': e.get('route', ''),
      })
    except (OSError, ValueError):
      continue
  return out



def locate_segment(route: str, wall_time: float, base: str = REALDATA) -> dict | None:
  """Which segment of the route was being written when this happened.

  The event records the route but not the segment -- loggerd rolls segments on its own clock and
  nothing publishes the current index. Rather than guess from elapsed time, which drifts whenever
  a segment is cut short, this reads the segments actually on disk and picks the one whose file
  was being written at that moment. Returns the offset into it too, so the player can seek there
  instead of making you scrub a minute of video looking for the moment.
  """
  if not route or not os.path.isdir(base):
    return None

  segs = []
  for entry in os.listdir(base):
    name, _, num = entry.rpartition('--')
    if name != route or not num.isdigit():
      continue
    # rlog.zst is closed when the segment rolls, so its mtime is when the segment *ended*.
    path = os.path.join(base, entry, 'rlog.zst')
    if not os.path.isfile(path):
      continue
    try:
      segs.append((int(num), os.path.getmtime(path)))
    except OSError:
      continue
  if not segs:
    return None
  segs.sort()

  for i, (num, ended) in enumerate(segs):
    started = segs[i - 1][1] if i else ended - SEGMENT_SECONDS
    if started <= wall_time <= ended:
      return {'seg': num, 'offset': round(max(0.0, wall_time - started), 1)}

  # Past the last closed segment: the event landed in the one still being recorded, which has no
  # end time yet. Report it without an offset rather than pointing at the wrong segment.
  last_num, last_end = segs[-1]
  if wall_time > last_end:
    return {'seg': last_num + 1, 'offset': None}
  return None

def read_event(name: str, base: str = EVENT_DIR) -> dict | None:
  if '/' in name or '\\' in name or '..' in name:
    return None
  try:
    with open(os.path.join(base, f'{name}.json')) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None
