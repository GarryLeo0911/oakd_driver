#!/bin/bash

echo "=== OAK-D Driver Build Verification ==="
echo "Checking for common build issues..."
echo

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "❌ Error: Not in oakd_driver directory. Please run from ~/ros_ws/src/oakd_driver"
    exit 1
fi

echo "✅ In correct directory"

# Check for problematic source files
echo
echo "🔍 Checking for missing header dependencies..."

# Look for files that might include missing headers
problematic_files=$(grep -r "RGBD\.hpp\|BasaltVIO\.hpp\|ThermalConfig\.hpp" src/ --include="*.cpp" 2>/dev/null | grep -v "//")

if [ -n "$problematic_files" ]; then
    echo "❌ Found files with missing headers:"
    echo "$problematic_files"
    echo
    echo "These files need to be excluded from CMakeLists.txt"
else
    echo "✅ No problematic header includes found"
fi

echo
echo "🔧 Attempting clean build..."
cd ~/ros_ws

# Clean previous build
echo "Cleaning previous build artifacts..."
rm -rf build/oakd_driver install/oakd_driver log/*oakd_driver* 2>/dev/null

echo "Building oakd_driver..."
colcon build --packages-select oakd_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

BUILD_RESULT=$?

echo
if [ $BUILD_RESULT -eq 0 ]; then
    echo "✅ BUILD SUCCESSFUL!"
    echo
    echo "Now you can test with:"
    echo "  source install/setup.bash"
    echo "  ros2 launch oakd_driver rtabmap_camera.launch.py"
else
    echo "❌ BUILD FAILED"
    echo
    echo "Common fixes:"
    echo "1. Check CMakeLists.txt for uncommented problematic .cpp files"
    echo "2. Make sure all missing header dependencies are commented out"
    echo "3. Check the error message above for specific issues"
fi

echo
echo "=== Build Verification Complete ==="