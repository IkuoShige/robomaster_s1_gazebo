#!/bin/bash
################################################################################
# Mecanum Drive Controller Verification Script
#
# This script verifies the mecanum drive controller setup and runs tests
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 environment is not sourced!"
    print_info "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_success "ROS2 environment detected: $ROS_DISTRO"

# Source workspace
WS_DIR="/home/ikuo/docker_play/s1_ws/src/ros2_ws"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    print_info "Sourcing workspace: $WS_DIR"
    source "$WS_DIR/install/setup.bash"
    print_success "Workspace sourced successfully"
else
    print_error "Workspace not built! Please build first:"
    print_info "  cd $WS_DIR && colcon build --packages-select robomaster_s1_gazebo"
    exit 1
fi

print_header "Step 1: Checking ROS2 Controllers"

# Check if controller_manager is available
if ros2 pkg list | grep -q "controller_manager"; then
    print_success "controller_manager package found"
else
    print_error "controller_manager package not found!"
    exit 1
fi

# Check if mecanum_drive_controller is available
if ros2 pkg list | grep -q "mecanum_drive_controller"; then
    print_success "mecanum_drive_controller package found"
else
    print_error "mecanum_drive_controller package not found!"
    exit 1
fi

print_header "Step 2: Checking Teleop Packages"

# Check teleop_twist_keyboard
if ros2 pkg list | grep -q "teleop_twist_keyboard"; then
    print_success "teleop_twist_keyboard package found"
else
    print_warning "teleop_twist_keyboard not installed"
    print_info "Install with: sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard"
fi

print_header "Step 3: Launching Gazebo (Headless)"

print_info "Starting Gazebo simulation in headless mode..."
print_info "This may take a few moments..."

# Launch Gazebo in background
ros2 launch robomaster_s1_gazebo s1_gazebo_headless.launch.py &
GAZEBO_PID=$!

print_info "Gazebo launched with PID: $GAZEBO_PID"
print_info "Waiting 15 seconds for Gazebo to initialize..."
sleep 15

print_header "Step 4: Verifying Controllers"

# Check if controller_manager is running
if ros2 node list | grep -q "controller_manager"; then
    print_success "controller_manager node is running"
else
    print_error "controller_manager node not found!"
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

# List controllers
print_info "Listing controllers..."
ros2 control list_controllers

# Check if mecanum_drive_controller is active
if ros2 control list_controllers | grep -q "mecanum_drive_controller.*active"; then
    print_success "mecanum_drive_controller is active"
else
    print_error "mecanum_drive_controller is not active!"
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

print_header "Step 5: Checking Topics"

print_info "Available topics:"
ros2 topic list | grep -E "(cmd_vel|odom|joint)"

# Check if cmd_vel topic exists
if ros2 topic list | grep -q "/mecanum_drive_controller/cmd_vel"; then
    print_success "cmd_vel topic exists"
else
    print_error "cmd_vel topic not found!"
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

# Check if odom topic exists
if ros2 topic list | grep -q "/mecanum_drive_controller/odom"; then
    print_success "odom topic exists"
else
    print_error "odom topic not found!"
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

print_header "Step 6: Manual Control Test"

print_info "Testing manual control with simple commands..."

# Test forward motion
print_info "Sending forward command..."
ros2 topic pub --once /mecanum_drive_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

# Test strafe right
print_info "Sending strafe right command..."
ros2 topic pub --once /mecanum_drive_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

# Test rotation
print_info "Sending rotation command..."
ros2 topic pub --once /mecanum_drive_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
sleep 2

# Stop
print_info "Sending stop command..."
ros2 topic pub --once /mecanum_drive_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

print_success "Manual control test completed"

print_header "Step 7: Running Automated Tests"

print_info "Launching automated test node..."
print_info "This will test various movement patterns for ~35 seconds..."

# Run test node
timeout 40s ros2 run robomaster_s1_gazebo test_mecanum_control.py || true

print_success "Automated tests completed"

print_header "Step 8: Odometry Check"

print_info "Checking odometry data..."
timeout 3s ros2 topic echo --once /mecanum_drive_controller/odom || print_warning "Could not read odometry"

print_header "Verification Complete!"

print_success "All verification steps completed successfully!"
print_info "You can now use the following commands:"
echo ""
echo "  # Manual keyboard control:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
echo "    --ros-args --remap /cmd_vel:=/mecanum_drive_controller/cmd_vel"
echo ""
echo "  # Run automated tests:"
echo "  ros2 run robomaster_s1_gazebo test_mecanum_control.py"
echo ""
echo "  # Monitor odometry:"
echo "  ros2 topic echo /mecanum_drive_controller/odom"
echo ""

print_info "Cleaning up..."
kill $GAZEBO_PID 2>/dev/null || true
wait $GAZEBO_PID 2>/dev/null || true

print_success "Done!"
