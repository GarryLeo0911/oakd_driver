#!/bin/bash

# System optimization script for OAK-D driver
echo "=== OAK-D Driver System Optimization ==="

# Check if running as root for some optimizations
if [[ $EUID -eq 0 ]]; then
    echo "Running as root - applying system-level optimizations"
    
    # Increase swap if needed
    if [ ! -f /swapfile ]; then
        echo "Creating 2GB swap file..."
        fallocate -l 2G /swapfile
        chmod 600 /swapfile
        mkswap /swapfile
        swapon /swapfile
        echo "/swapfile none swap sw 0 0" >> /etc/fstab
    fi
    
    # Memory optimization settings
    echo "Applying memory optimizations..."
    echo 1 > /proc/sys/vm/overcommit_memory
    echo 50 > /proc/sys/vm/swappiness
    
    # USB optimization for OAK-D
    echo "Optimizing USB settings..."
    echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb
    
else
    echo "Not running as root - applying user-level optimizations only"
fi

# Set memory-conscious environment variables
export MALLOC_ARENA_MAX=2
export MALLOC_MMAP_THRESHOLD=65536
export MALLOC_TRIM_THRESHOLD=131072

# Clean system cache
echo "Cleaning system cache..."
sync
echo 1 > /proc/sys/vm/drop_caches 2>/dev/null || echo "Cannot drop caches (need root)"

# Check current memory status
echo
echo "=== Current Memory Status ==="
free -h
echo

echo "=== USB Device Status ==="
lsusb | grep -i "Movidius\|Intel"
echo

echo "=== Optimization Complete ==="
echo "You can now run: ros2 launch oakd_driver memory_safe_driver.launch.py"