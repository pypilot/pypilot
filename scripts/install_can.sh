#!/bin/sh
# Install CAN support for an SPI CAN controller on tinypilot (piCore).
#
# piCore doesn't ship CAN kernel modules, so this packages can, can-raw,
# can-dev and the controller's driver from piCore's kernel module tarball into
# an extension, can-<kernel>.tcz, loaded at boot. The extension also brings
# can0 up at the given bitrate; it depends on iproute2, since busybox ip
# can't configure CAN links.
#
# The device tree overlay goes in config.txt on the boot partition, e.g.
#   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
#
# Run as tc. Needs internet access. Persists through tce/optional and
# onboot.lst, so no backup is needed.
set -eu

usage() {
    echo "Usage: $0 mcp251x|mcp251xfd [bitrate]"
    echo "  mcp251x   = MCP2515 boards"
    echo "  mcp251xfd = MCP2517FD/MCP2518FD boards"
    echo "  bitrate   = optional, default 250000 (NMEA 2000)"
    exit 1
}

DRIVER="${1:-}"
BITRATE="${2:-250000}"

case "$DRIVER" in
    mcp251x) DRIVER_KO="drivers/net/can/spi/mcp251x.ko" ;;
    mcp251xfd) DRIVER_KO="drivers/net/can/spi/mcp251xfd/mcp251xfd.ko" ;;
    *) usage ;;
esac

KVER="$(uname -r)"
case "$KVER" in
    *piCore*) ;;
    *)
        echo "Unexpected kernel: $KVER (this script is for piCore)"
        exit 1
        ;;
esac

# piCore 13 and 16 publish the module tarball under different names
TC_VERSION="$( (version || cat /usr/share/doc/tc/release.txt) 2>/dev/null | cut -d. -f1)"
if [ -z "$TC_VERSION" ]; then
    echo "Can't tell the piCore version"
    exit 1
fi
MIRROR="http://tinycorelinux.net/${TC_VERSION}.x"
case "$TC_VERSION" in
    13) MOD_URL="${MIRROR}/armv7/releases/RPi/src/kernel/${KVER}_modules.tar.xz" ;;
    *) MOD_URL="${MIRROR}/armhf/release/src/kernel/modules-${KVER}.tar.xz" ;;
esac

TCEDIR="/etc/sysconfig/tcedir"
OPTIONAL="${TCEDIR}/optional"
ONBOOT="${TCEDIR}/onboot.lst"
EXT="can-${KVER}"
WORK="/tmp/can-install.$$"

cleanup() {
    rm -rf "$WORK"
}
trap cleanup EXIT INT TERM
mkdir -p "$WORK"

echo "Loading tools..."
tce-load -wi squashfs-tools xz iproute2

echo "Downloading kernel modules for ${KVER}..."
wget -O "${WORK}/modules.tar.xz" "$MOD_URL"

echo "Packaging ${EXT}.tcz..."
PKG="${WORK}/pkg"
SRC="lib/modules/${KVER}/kernel"
DST="${PKG}/usr/local/lib/modules/${KVER}/kernel"
MODULES="net/can/can.ko net/can/can-raw.ko drivers/net/can/dev/can-dev.ko ${DRIVER_KO}"
mkdir -p "${WORK}/modules" "${PKG}/usr/local/tce.installed"
# the tarball's top directory differs between releases, so look the modules up
# by path, then extract only them (/tmp is in RAM)
xz -dc "${WORK}/modules.tar.xz" | tar -t > "${WORK}/contents"
members=""
for module in $MODULES; do
    member="$(grep -m 1 "${SRC}/${module}\$" "${WORK}/contents" || true)"
    if [ -z "$member" ]; then
        echo "Module ${module} not found in ${MOD_URL}"
        exit 1
    fi
    members="$members $member"
done
# shellcheck disable=SC2086 # members is a list of paths without spaces
xz -dc "${WORK}/modules.tar.xz" | tar -x -C "${WORK}/modules" $members
for module in $MODULES; do
    mkdir -p "$(dirname "${DST}/${module}")"
    cp "$(find "${WORK}/modules" -path "*/${SRC}/${module}")" "${DST}/${module}"
done

cat > "${PKG}/usr/local/tce.installed/${EXT}" <<EOF
#!/bin/sh
# Load the CAN modules and bring can0 up at ${BITRATE} bit/s.
BASE=/usr/local/lib/modules/${KVER}/kernel
for module in ${MODULES}; do
    insmod "\$BASE/\$module" 2>/dev/null
done

IP=/usr/local/sbin/ip
[ -x "\$IP" ] || IP=ip

# can0 appears once the driver binds to the overlay's device
for i in 1 2 3 4 5 6 7 8 9 10; do
    "\$IP" link show can0 >/dev/null 2>&1 && break
    sleep 0.5
done
# restart-ms recovers automatically after bus-off
"\$IP" link set can0 up type can bitrate ${BITRATE} restart-ms 100
EOF
chmod 775 "${PKG}/usr/local/tce.installed/${EXT}"

rm -f "${OPTIONAL}/${EXT}.tcz"
mksquashfs "$PKG" "${OPTIONAL}/${EXT}.tcz" -all-root -noappend >/dev/null
(cd "$OPTIONAL" && md5sum "${EXT}.tcz" > "${EXT}.tcz.md5.txt")
echo iproute2.tcz > "${OPTIONAL}/${EXT}.tcz.dep"

grep -qxF "${EXT}.tcz" "$ONBOOT" || echo "${EXT}.tcz" >> "$ONBOOT"

echo "Loading ${EXT}.tcz..."
tce-load -i "${OPTIONAL}/${EXT}.tcz"

echo
if ip link show can0 >/dev/null 2>&1; then
    ip -details link show can0
else
    echo "can0 does not exist: check the dtoverlay line in config.txt, and dmesg | grep -i mcp251"
fi
