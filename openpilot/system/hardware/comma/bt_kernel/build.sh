#!/usr/bin/env bash
# Build boot-bt.img.xz: the AGNOS 19.6 kernel with commaai/agnos-kernel-sdm845 PR #106 (Bluetooth).
#
# Same compiler, defconfig, mkbootimg arguments and signing as agnos-builder's build_kernel.sh at
# AGNOS 19.6 (c6a8e491ab6a), minus the docker container, so it runs on a plain Ubuntu box without
# root. What the container provided is worked around here:
#   - python2 for scripts/gcc-wrapper.py: CC is set on the make line, bypassing the wrapper. It only
#     greps gcc's warnings, so the objects are the same.
#   - libssl-dev for the host tools sign-file / extract-cert: unpacked from `apt-get download`.
#   - gcc 10+ as host compiler: -fcommon, for dtc's shipped lexer.
#
# Usage: build.sh [workdir]   (default ~/agnos-bt-build; ~2 GB, the kernel tree is shallow)
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${1:-$HOME/agnos-bt-build}"

KERNEL_COMMIT=eccd146599f2e2f159d951092642689bede91632   # agnos-builder 19.6 submodule
BUILDER_COMMIT=c6a8e491ab6a
TOOLCHAIN_SHA256=d2451496fa23bbf94b8fc50a4b07a053ddffa70b1a7940d7b880eda0937187f7
CMDLINE='console=ttyMSM0,115200n8 isolcpus=6,7 quiet loglevel=3 earlycon=msm_geni_serial,0xA84000 androidboot.hardware=qcom androidboot.console=ttyMSM0 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 service_locator.enable=1 androidboot.selinux=permissive firmware_class.path=/lib/firmware/updates net.ifnames=0 dyndbg=""'

mkdir -p "$WORK" && cd "$WORK"
RAW=https://raw.githubusercontent.com/commaai/agnos-builder/$BUILDER_COMMIT

echo "=== toolchain ==="
if [ ! -d toolchain/aarch64-linux-gnu-gcc ]; then
  # an LFS object in agnos-builder; media.githubusercontent.com serves it without git-lfs
  curl -fL -o aarch64-linux-gnu-gcc.tar.gz \
    https://media.githubusercontent.com/media/commaai/agnos-builder/$BUILDER_COMMIT/tools/aarch64-linux-gnu-gcc.tar.gz
  echo "$TOOLCHAIN_SHA256  aarch64-linux-gnu-gcc.tar.gz" | sha256sum -c
  mkdir -p toolchain && tar -xzf aarch64-linux-gnu-gcc.tar.gz -C toolchain
fi
T="$WORK/toolchain/aarch64-linux-gnu-gcc/bin/aarch64-linux-gnu-"
[ -f mkbootimg ] || curl -fsS -o mkbootimg $RAW/tools/mkbootimg
[ -f vble-qti.key ] || curl -fsS -o vble-qti.key $RAW/vble-qti.key

echo "=== host openssl headers ==="
H="$WORK/hostdeps"
if [ ! -f "$H/root/usr/include/openssl/opensslv.h" ]; then
  mkdir -p "$H/lib" && (cd "$H" && apt-get download libssl-dev && dpkg-deb -x libssl-dev_*.deb root)
  ln -sf /usr/lib/x86_64-linux-gnu/libcrypto.so.3 "$H/lib/libcrypto.so"
  ln -sf /usr/lib/x86_64-linux-gnu/libssl.so.3 "$H/lib/libssl.so"
fi

echo "=== kernel source ==="
if [ ! -d kernel/.git ]; then
  git init -q kernel
  git -C kernel remote add origin https://github.com/commaai/agnos-kernel-sdm845.git
fi
git -C kernel fetch -q --depth 1 origin $KERNEL_COMMIT
git -C kernel checkout -q --force FETCH_HEAD
git -C kernel apply "$HERE/bluetooth.patch"

echo "=== build ==="
cd kernel
export ARCH=arm64 CROSS_COMPILE=$T KBUILD_BUILD_HOST=docker KCFLAGS=-w
# LOCALVERSION set (even empty) keeps setlocalversion from appending "+" for the git tree: uname -r
# stays 4.9.103 like stock
MAKE_ARGS=(O=../out CC=${T}gcc LD=${T}ld.bfd PYTHON=python3 LOCALVERSION=
  "HOSTCFLAGS=-fcommon -I$H/root/usr/include -I$H/root/usr/include/x86_64-linux-gnu"
  "HOSTLDFLAGS=-L$H/lib" "HOST_LOADLIBES=-L$H/lib")
make "${MAKE_ARGS[@]}" tici_defconfig
# the config must be stock 19.6 plus Bluetooth and nothing else (bt-config.diff, checked against
# /proc/config.gz of a 19.6 device)
diff <(grep -E '^CONFIG_' "$HERE/stock-19.6.config" | sort) <(grep -E '^CONFIG_' ../out/.config | sort) \
  > ../config.diff || true
diff -u "$HERE/bt-config.diff" ../config.diff
make -j"$(nproc)" "${MAKE_ARGS[@]}"
cd ..

echo "=== boot image ==="
python3 mkbootimg --kernel out/arch/arm64/boot/Image-dtb --ramdisk /dev/null --cmdline "$CMDLINE" \
  --pagesize 4096 --base 0x80000000 --kernel_offset 0x8000 --ramdisk_offset 0x8000 --tags_offset 0x100 \
  --output boot.img.nonsecure
openssl dgst -sha256 -binary boot.img.nonsecure > boot.img.sha256
openssl pkeyutl -sign -in boot.img.sha256 -inkey vble-qti.key -out boot.img.sig \
  -pkeyopt digest:sha256 -pkeyopt rsa_padding_mode:pkcs1
dd if=/dev/zero of=boot.img.sig.padded bs=2048 count=1 status=none
dd if=boot.img.sig of=boot.img.sig.padded conv=notrunc status=none
cat boot.img.nonsecure boot.img.sig.padded > boot.img

xz -9 -k -f -T0 boot.img
cp boot.img.xz "$HERE/boot-bt.img.xz"
python3 - "$HERE/boot-bt.json" boot.img <<'EOF'
import hashlib, json, os, sys
meta = json.load(open(sys.argv[1]))
data = open(sys.argv[2], 'rb').read()
meta.update(size=len(data), sha256=hashlib.sha256(data).hexdigest())
json.dump(meta, open(sys.argv[1], 'w'), indent=2)
open(sys.argv[1], 'a').write('\n')
EOF
echo "=== done: $HERE/boot-bt.img.xz ==="
