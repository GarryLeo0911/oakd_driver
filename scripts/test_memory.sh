#!/bin/bash

echo "=== OAK-D Driver Memory Testing Script ==="
echo "Testing progression: minimal -> simple -> full driver"
echo

echo "1. Testing minimal camera test node..."
echo "   This tests basic ROS2 functionality and memory allocation"
ros2 run oakd_driver minimal_camera_test &
MINIMAL_PID=$!
sleep 5
kill $MINIMAL_PID 2>/dev/null
echo "   Minimal test completed"
echo

echo "2. Checking current memory status..."
free -h
echo

echo "3. Testing memory allocation patterns..."
echo "   Available before test:"
free -h | grep "Mem:" | awk '{print "   Available: " $7}'

# Test memory allocation with increasing sizes
for size in 64 128 256 512 1024; do
    echo "   Testing ${size}MB allocation..."
    python3 -c "
import sys
try:
    data = bytearray(${size} * 1024 * 1024)
    print(f'   ✓ ${size}MB allocation successful')
    del data
except MemoryError:
    print(f'   ✗ ${size}MB allocation failed')
    sys.exit(1)
" || break
done

echo
echo "4. System information:"
echo "   Memory info:"
free -h | head -2
echo "   Swap info:"
swapon --show 2>/dev/null || echo "   No swap configured"
echo "   VM info:"
cat /proc/meminfo | grep -E "(MemAvailable|SwapTotal|VmallocTotal)" | head -3

echo
echo "5. USB and device info:"
lsusb | grep -i "intel\|movidius" || echo "   No Intel/Movidius devices found"

echo
echo "=== Memory Test Complete ==="
echo "If all tests passed, you can try:"
echo "  ros2 launch oakd_driver memory_safe_driver.launch.py"