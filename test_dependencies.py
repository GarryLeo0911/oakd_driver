#!/usr/bin/env python3

"""
Test script to verify OAK-D driver dependencies are properly installed.
"""

import sys

def test_imports():
    """Test all required imports."""
    
    print("Testing OAK-D Driver Dependencies...")
    print("=" * 50)
    
    # Test basic Python modules
    try:
        import sys
        print(f"✓ Python {sys.version}")
    except ImportError as e:
        print(f"✗ Python import failed: {e}")
        return False
    
    # Test OpenCV
    try:
        import cv2
        print(f"✓ OpenCV {cv2.__version__}")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
        return False
    
    # Test NumPy
    try:
        import numpy as np
        print(f"✓ NumPy {np.__version__}")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False
    
    # Test DepthAI
    try:
        import depthai as dai
        print(f"✓ DepthAI {dai.__version__}")
        
        # Test device detection
        devices = dai.Device.getAllAvailableDevices()
        print(f"✓ Found {len(devices)} OAK device(s)")
        
        for device in devices:
            print(f"  - Device: {device.getMxId()}")
            
    except ImportError as e:
        print(f"✗ DepthAI import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ DepthAI device detection failed: {e}")
        return False
    
    # Test ROS2 dependencies (optional)
    try:
        import rclpy
        print(f"✓ ROS2 rclpy available")
    except ImportError:
        print("! ROS2 not available (optional)")
    
    try:
        from cv_bridge import CvBridge
        print(f"✓ cv_bridge available")
    except ImportError:
        print("! cv_bridge not available (install ros-humble-cv-bridge)")
    
    print("=" * 50)
    print("✓ All core dependencies are available!")
    return True

if __name__ == "__main__":
    if test_imports():
        print("\nYou can now run the OAK-D driver:")
        print("ros2 launch oakd_driver oakd_driver.launch.py")
        sys.exit(0)
    else:
        print("\nPlease install missing dependencies:")
        print("pip install --user depthai opencv-python numpy")
        print("sudo apt install ros-humble-cv-bridge")
        sys.exit(1)