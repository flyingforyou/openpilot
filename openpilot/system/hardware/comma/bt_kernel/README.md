# Bluetooth 커널 (AGNOS 19.6 + agnos-kernel-sdm845 PR #106)

stock AGNOS 커널은 블루투스가 꺼져 있다(`CONFIG_BT=n`, `/dev/btpower` 없음). 그래서 btobdd(ELM327
동글, `/diag`·`/tdiag`·`/vlog`·`/daily`)는 manager에서 아예 켜지지 않는다(`process_config.py`의
`os.path.exists('/dev/btpower')`). 이 폴더는 블루투스를 켠 19.6 커널과, 그걸 기기에 올리는 방법이다.

## 파일

| 파일 | 내용 |
|---|---|
| `bluetooth.patch` | PR #106 diff 그대로. defconfig(BT, RFCOMM, HCI UART/QCA, `MSM_BT_POWER`)와 DT(블루투스 UART `qupv3_se6_4uart` 켜기, `hsuart1`) 3파일 |
| `boot-bt.img.xz` | 빌드 결과 (서명된 boot 이미지, xz) |
| `boot-bt.json` | 이미지 크기·sha256, 기준 AGNOS와 그 stock boot 해시 |
| `stock-19.6.config` | 19.6 기기의 `/proc/config.gz` (10.0.0.56에서 받음) |
| `bt-config.diff` | stock 대비 바뀌는 설정 — 블루투스 11개뿐. 빌드가 이걸 확인한다 |
| `build.sh` | 재현 빌드 |
| `install.py` | 기기 설치 (비활성 슬롯) |

시스템 파티션은 바꾸지 않는다. btobd는 bluez·펌웨어 파일 없이 raw HCI로 칩 ROM 상태 그대로 H4 115200에
붙는다(`btobd/hci.py`). 커널만 있으면 된다.

## 빌드 (`build.sh`)

agnos-builder 19.6(`c6a8e491ab6a`)의 `build_kernel.sh`와 같은 커널 커밋(`eccd146599`), 같은 컴파일러(gcc 8.2.1
arm-rel-8.23 — 19.6 기기의 `/proc/version`과 동일), 같은 mkbootimg 인자·커맨드라인·서명. 다른 점은 docker
없이 root 없이 돈다는 것뿐이다(python2 gcc-wrapper 우회, libssl-dev를 `apt-get download`로 풀어 씀).

PR 브랜치를 그대로 빌드하지 않는 이유: PR은 19.6보다 13커밋 앞(USB3 PHY 튜닝, UFS 등)의 master에서
갈라졌다. 그걸 올리면 블루투스 말고도 커널이 바뀐다. 그래서 19.6 커널에 PR의 3파일 diff만 얹는다.

확인한 것:
- 패치 없는 `tici_defconfig` → 19.6 기기 `/proc/config.gz`와 `CONFIG_` 줄 차이 0
- 패치 후 → 차이는 `bt-config.diff`의 블루투스 11개뿐
- boot 헤더(로드 주소, 페이지 크기, 커맨드라인)가 기기 `boot_a`(stock 19.6)와 같음. 커널 크기만 +766 KB
  (46.9 → 47.7 MB, 블루투스 코드). boot 파티션 64 MB
- `uname -r` 4.9.103 그대로 (기기에 커널 모듈은 없음 — `/lib/modules` 비어 있음)
- 실기기 (10.0.0.56, comma four, 2026-11-02): `install.py`로 slot `_a` → `_b`, 재부팅 후 `_b`로 부팅,
  `/dev/btpower`·`hci0` 생김, manager·UI·pandad 정상, btobdd `bt: up`, `/diag` 200, Wi-Fi 정상

## 설치 (`install.py`)

```sh
sudo PYTHONPATH=/data/openpilot /data/openpilot/.venv/bin/python \
  /data/openpilot/openpilot/system/hardware/comma/bt_kernel/install.py
sudo reboot
```

AGNOS 업데이트와 같은 방식이다: 지금 부팅한 슬롯은 건드리지 않고, 반대 슬롯에 stock 19.6 파티션(xbl, abl,
system ...)과 이 boot를 쓴 뒤 그 슬롯을 활성(아직 성공 표시 없음)으로 바꾼다. 새 커널이 안 뜨면 부트로더가
재시도를 다 쓰고 원래 슬롯으로 돌아간다. 뜨면 `launch_chffrplus.sh`의 `abctl --set_success`가 확정한다.

- 반대 슬롯이 19.6이 아니면 system 등을 commadist에서 받는다 — **약 1.0 GB** (system xz 1,010 MB). LTE면 비용 주의.
  이미 맞는 파티션은 건너뛴다.
- 기기 `/VERSION`이 19.6이 아니거나, 레포 `agnos.json`이 19.6이 아니면 설치하지 않는다(그때는 새 AGNOS 기준으로
  `build.sh`를 다시 돌려야 한다).
- 설치 후 확인: `ls /dev/btpower`, manager에 `btobdd`, `/diag` 동글 화면.

되돌리기: `sudo abctl --set_active <원래 슬롯 번호> && sudo reboot` (install.py가 마지막에 번호를 출력한다).

## 주의

- 나중에 레포의 AGNOS 버전이 올라가면 AGNOS 업데이트가 stock boot를 반대 슬롯에 쓰고 넘어가므로 블루투스가
  다시 없어진다. 새 버전 커널로 `build.sh`(커널 커밋·stock config 갱신)부터 다시.
- 부팅이 아예 멈춰서 재시도도 안 되는 경우는 원격으로 못 살린다. 처음 올릴 때는 기기 옆에 사람이 있을 때.
- PR #106은 comma four(`comma mici`)에서 만들어졌다. 다른 기기에서는 확인 안 됨.
