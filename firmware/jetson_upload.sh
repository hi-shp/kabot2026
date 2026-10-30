#!/usr/bin/env bash
# Run with bash. Default is build-only; --flash backs up and verifies an UNO.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
port=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
flash=false
run_tests=false
while (($#)); do
  case "$1" in
    --flash) flash=true; shift ;;
    --build-only) flash=false; shift ;;
    --test) run_tests=true; shift ;;
    --port) test "$#" -ge 2 || exit 64; port=$2; shift 2 ;;
    -h|--help)
      echo 'bash jetson_upload.sh [--build-only | --flash] [--port DEVICE] [--test]'
      echo 'Before --flash: disconnect ESC/motor power; keep USB; stop Arduino bridge/serial monitor.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 64 ;;
  esac
done
cli=${ARDUINO_CLI:-"$HOME/.local/bin/arduino-cli"}
test -x "$cli" || { echo "Arduino CLI missing: $cli" >&2; exit 1; }
mkdir -p "$HOME/.cache/kabot-uno"
exec 9>"$HOME/.cache/kabot-uno/upload.lock"
flock -n 9 || { echo 'Another build/upload is running.' >&2; exit 1; }
check_port() {
  test -c "$port" || { echo "UNO USB unavailable: $port" >&2; exit 1; }
  if fuser "$port" >/dev/null 2>&1; then
    echo 'Port busy. Stop the Arduino bridge/launch and serial monitor before upload.' >&2
    exit 1
  fi
}
if "$flash"; then check_port; fi
build=$(mktemp -d "$HOME/.cache/kabot-uno/build.XXXXXX")
source_fingerprint=$(sha256sum "$root/kabot_actuator"/*.ino "$root/kabot_actuator"/*.h)
if "$run_tests"; then
  g++ -std=c++11 -Wall -Wextra -Werror "$root/tests/control_test.cpp" -o "$build/control_test"
  "$build/control_test"
fi
"$cli" compile --fqbn arduino:avr:uno --build-path "$build" "$root/kabot_actuator"
hex="$build/kabot_actuator.ino.hex"
test -s "$hex"
echo "BUILD_OK=$hex"
if ! "$flash"; then exit 0; fi
check_port
umask 077
mkdir -p "$HOME/code_backups"
backup=$(mktemp -d "$HOME/code_backups/arduino-preupload-$(date -u +%Y%m%dT%H%M%SZ).XXXXXX")
echo "BACKUP=$backup"
if ! avrdude -p atmega328p -c arduino -P "$port" -b 115200 -n \
    -U "flash:r:$backup/flash_read1.bin:r" -U "eeprom:r:$backup/eeprom.bin:r" >"$backup/read1.log" 2>&1; then
  cat "$backup/read1.log" >&2; exit 1
fi
if ! avrdude -p atmega328p -c arduino -P "$port" -b 115200 -n \
    -U "flash:r:$backup/flash_read2.bin:r" >"$backup/read2.log" 2>&1; then
  cat "$backup/read2.log" >&2; exit 1
fi
python3 - "$backup" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
a=(p/'flash_read1.bin').read_bytes()
b=(p/'flash_read2.bin').read_bytes()
if len(a)!=32768 or a!=b or len(set(a))<=2 or len((p/'eeprom.bin').read_bytes())!=1024:
    raise SystemExit('Backup validation failed; no flash write performed')
print('BACKUP_DOUBLE_READ_OK')
PY
test "$source_fingerprint" = "$(sha256sum "$root/kabot_actuator"/*.ino "$root/kabot_actuator"/*.h)" || {
  echo 'Source changed after compilation. Build/upload again; no flash write performed.' >&2; exit 1;
}
cp "$hex" "$backup/new_firmware.hex"
cp -a "$root/kabot_actuator" "$backup/source"
sha256sum "$backup"/*.bin "$backup/new_firmware.hex" "$backup/source"/* > "$backup/sha256.txt"
check_port
# --verify reads back the new image. Never use --force or skip verification.
"$cli" upload --fqbn arduino:avr:uno --port "$port" --input-dir "$build" --verify "$root/kabot_actuator" \
    >"$backup/upload.log" 2>&1 || { cat "$backup/upload.log" >&2; exit 1; }
cat "$backup/upload.log"
echo "UPLOAD_VERIFIED_OK backup=$backup"
