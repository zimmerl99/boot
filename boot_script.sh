#!/bin/bash

COUNTER_FILE="/home/gfrpi/boot/.crash_counter"
MAX_CRASHES=5

# Read current crash count
if [ -f "$COUNTER_FILE" ]; then
    CRASH_COUNT=$(cat "$COUNTER_FILE")
else
    CRASH_COUNT=0
fi

# Check if exceeded max crashes
if [ "$CRASH_COUNT" -ge "$MAX_CRASHES" ]; then
    echo "Ntrip client has crashed ($MAX_CRASHES) times. Check connection." >> /home/gfrpi/boot/crash.log
    exit 1
fi

# Run the Python script
python3 /home/gfrpi/boot/boot_rtk_v2.py

# the script crashed
CRASH_COUNT=$((CRASH_COUNT + 1))
echo "$CRASH_COUNT" > "$COUNTER_FILE"
echo "Crash #$CRASH_COUNT at $(date)" >> /home/gfrpi/boot/crash.log

# Reboot
sudo reboot
