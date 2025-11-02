#!/bin/bash

# Memory monitoring script for OAK-D driver debugging
echo "=== Memory Status Before Launch ==="
free -h
echo

echo "=== Swap Status ==="
swapon --show
echo

echo "=== Available Disk Space ==="
df -h /tmp
echo

echo "=== Process Memory Usage ==="
ps aux --sort=-%mem | head -10
echo

echo "=== Starting OAK-D Driver with Memory Monitoring ==="
echo "Press Ctrl+C to stop monitoring"

# Function to monitor memory during execution
monitor_memory() {
    while true; do
        echo "=== $(date) ==="
        echo "Memory:"
        free -h | grep -E "(Mem|Swap)"
        echo "Top memory users:"
        ps aux --sort=-%mem | head -5 | tail -4
        echo "---"
        sleep 5
    done
}

# Start monitoring in background
monitor_memory &
MONITOR_PID=$!

# Trap to cleanup background process
trap "kill $MONITOR_PID 2>/dev/null" EXIT

# Launch the driver
ros2 launch oakd_driver memory_safe_driver.launch.py