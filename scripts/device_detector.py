#!/usr/bin/env python3
"""
Device detector utility for OAK-D cameras.
Similar to the luxonis device detection utilities.
"""

import sys
import time

try:
    import depthai as dai
except ImportError:
    print("ERROR: depthai not installed. Please install with: pip install depthai")
    sys.exit(1)


def detect_devices():
    """Detect and list all available DepthAI devices"""
    print("Detecting DepthAI devices...")
    print("-" * 50)
    
    devices = dai.Device.getAllAvailableDevices()
    
    if not devices:
        print("No devices found!")
        print("\nTroubleshooting:")
        print("1. Ensure your OAK-D camera is connected via USB")
        print("2. Check if device drivers are installed")
        print("3. Try a different USB port or cable")
        print("4. For PoE cameras, check network connection")
        return
    
    print(f"Found {len(devices)} device(s):")
    print()
    
    for i, device_info in enumerate(devices):
        print(f"Device {i + 1}:")
        print(f"  Name: {device_info.name}")
        print(f"  MX ID: {device_info.getMxId()}")
        print(f"  State: {device_info.state}")
        print(f"  Protocol: {device_info.getXLinkDeviceDesc().protocol}")
        
        # Try to get additional info if device is available
        if device_info.state in [dai.XLinkDeviceState.X_LINK_UNBOOTED, dai.XLinkDeviceState.X_LINK_BOOTLOADER]:
            try:
                print("  Connecting to get device info...")
                with dai.Device(device_info) as device:
                    print(f"  Device Name: {device.getDeviceName()}")
                    print(f"  Product Name: {device.getProductName()}")
                    print(f"  Board Name: {device.getBoardName()}")
                    
                    # Get camera sensors
                    sensors = device.getCameraSensorNames()
                    if sensors:
                        print("  Camera Sensors:")
                        for socket, sensor_name in sensors.items():
                            print(f"    Socket {socket}: {sensor_name}")
                    
                    # Get USB speed if applicable
                    if device_info.getXLinkDeviceDesc().protocol != dai.XLinkProtocol.X_LINK_TCP_IP:
                        usb_speed = device.getUsbSpeed()
                        print(f"  USB Speed: {usb_speed}")
                    
                    # Get temperature info
                    try:
                        temps = device.getChipTemperature()
                        if temps:
                            print("  Temperatures:")
                            for temp_source, temp_value in temps.items():
                                print(f"    {temp_source}: {temp_value:.1f}°C")
                    except:
                        pass
                        
            except Exception as e:
                print(f"  Error getting device info: {str(e)}")
        
        print()


def test_device_connection(mx_id=None, ip=None):
    """Test connection to a specific device"""
    print("Testing device connection...")
    print("-" * 30)
    
    try:
        if mx_id:
            print(f"Connecting to device with MX ID: {mx_id}")
            devices = dai.Device.getAllAvailableDevices()
            device_info = None
            for info in devices:
                if info.getMxId() == mx_id:
                    device_info = info
                    break
            
            if not device_info:
                print(f"Device with MX ID {mx_id} not found!")
                return False
                
        elif ip:
            print(f"Connecting to device with IP: {ip}")
            device_info = dai.DeviceInfo(ip)
        else:
            print("Connecting to any available device...")
            device_info = dai.Device.getAnyAvailableDevice()
            if device_info:
                device_info = device_info[1]
            else:
                print("No devices available!")
                return False
        
        # Test connection
        with dai.Device(device_info) as device:
            print("✓ Connection successful!")
            print(f"  Device: {device.getDeviceName()}")
            print(f"  MX ID: {device.getMxId()}")
            
            # Test pipeline creation
            pipeline = dai.Pipeline()
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(640, 480)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            
            rgb_out = pipeline.create(dai.node.XLinkOut)
            rgb_out.setStreamName("rgb")
            cam_rgb.preview.link(rgb_out.input)
            
            print("✓ Pipeline creation successful!")
            
            # Start pipeline briefly
            device.startPipeline(pipeline)
            print("✓ Pipeline start successful!")
            
            # Get a few frames to verify
            q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            for i in range(5):
                frame = q.get()
                if frame:
                    print(f"✓ Frame {i+1} received")
                    break
                time.sleep(0.1)
            
            print("✓ Device test completed successfully!")
            return True
            
    except Exception as e:
        print(f"✗ Connection failed: {str(e)}")
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="DepthAI Device Detector and Tester")
    parser.add_argument("--detect", action="store_true", help="Detect all available devices")
    parser.add_argument("--test", action="store_true", help="Test device connection")
    parser.add_argument("--mx-id", type=str, help="MX ID of device to test")
    parser.add_argument("--ip", type=str, help="IP address of device to test")
    
    args = parser.parse_args()
    
    if args.detect or (not args.test):
        detect_devices()
    
    if args.test:
        print()
        test_device_connection(mx_id=args.mx_id, ip=args.ip)


if __name__ == "__main__":
    main()