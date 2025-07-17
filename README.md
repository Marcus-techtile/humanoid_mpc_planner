# Humanoid MPC Planner

A Model Predictive Control (MPC) motion planning and control system for the humanoid robot G1. This package provides real-time motion planning and control for walking and balancing tasks.

## Features

- **MPC-based Motion Planning**: Real-time optimization for smooth motion generation
- **Walking Pattern Generation**: Dynamic gait planning with footstep optimization
- **Balance Control**: Active balance control with stability analysis
- **State Estimation**: Robust state estimation from sensor data
- **Safety Features**: Joint limit enforcement and emergency stop functionality
- **Real-time Control**: 50Hz control loop for responsive motion

## Architecture

The MPC planner consists of several key components:

### 1. HumanoidMPCPlanner (Main Controller)
- Integrates all components and manages the control loop
- Handles velocity commands and generates joint trajectories
- Publishes control outputs and status information

### 2. MPCSolver
- Solves the MPC optimization problem
- Handles cost function and constraint formulation
- Provides fallback control when optimization fails

### 3. StateEstimator
- Estimates robot state from sensor data
- Implements Kalman filtering for noise reduction
- Computes center of mass and base state

### 4. GaitPlanner
- Generates walking patterns based on velocity commands
- Plans footsteps and gait phases
- Computes ZMP and COM trajectories

### 5. BalanceController
- Maintains robot stability during motion
- Implements capture point and ZMP control
- Provides emergency balance recovery

## Installation

### Prerequisites
- ROS Melodic or Noetic
- Eigen3
- Boost
- Catkin build system

### Building
```bash
# Navigate to your workspace
cd ~/catkin_ws

# Build the package
catkin_make

# Source the workspace
source devel/setup.bash
```

## Usage

### 1. Launch the MPC Planner
```bash
# Launch the MPC planner node
roslaunch humanoid_mpc_planner mpc_planner.launch

# Launch with visualization
roslaunch humanoid_mpc_planner mpc_planner.launch use_rviz:=true

# Launch with test data
roslaunch humanoid_mpc_planner mpc_planner.launch use_joint_state_publisher:=true
```

### 2. Send Velocity Commands
```bash
# Send forward velocity command
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# Send turning command
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

### 3. Monitor Status
```bash
# Check joint commands
rostopic echo /joint_commands

# Monitor balance status
rostopic echo /balance_status

# View COM trajectory
rostopic echo /com_trajectory
```

### 4. Test with Simulation
```bash
# Run the test node to simulate robot data
rosrun humanoid_mpc_planner mpc_planner_node.py
```

## Configuration

### MPC Parameters
Edit `config/mpc_params.yaml` to adjust:

- **Control Parameters**: Control rate, prediction horizon, time step
- **Cost Weights**: Position tracking, velocity tracking, control effort
- **Joint Limits**: Position and velocity limits for each joint
- **Safety Parameters**: Maximum velocities and emergency thresholds
- **Balance Parameters**: COM height, foot dimensions, control gains
- **Gait Parameters**: Step length, width, height, and duration

### Example Configuration
```yaml
# Control parameters
control_rate: 50.0
prediction_horizon: 10
control_horizon: 5
dt: 0.02

# Cost weights
cost_weights:
  position_tracking: 1.0
  velocity_tracking: 0.5
  control_effort: 0.1
  final_state: 10.0

# Safety limits
safety:
  max_linear_velocity: 2.0
  max_angular_velocity: 2.0
  emergency_stop_threshold: 2.0
```

## Topics

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/joint_states` (sensor_msgs/JointState): Current joint states
- `/odom` (nav_msgs/Odometry): Robot odometry

### Published Topics
- `/joint_commands` (std_msgs/Float64MultiArray): Joint position commands
- `/com_trajectory` (nav_msgs/Path): Center of mass trajectory
- `/footstep_plan` (nav_msgs/Path): Planned footsteps
- `/balance_status` (std_msgs/Bool): Balance stability status

## G1 Robot Joint Configuration

The MPC planner is configured for the G1 humanoid robot with 29 degrees of freedom:

### Leg Joints (12 DOF)
- **Left Leg**: 6 joints (hip pitch/roll/yaw, knee, ankle pitch/roll)
- **Right Leg**: 6 joints (hip pitch/roll/yaw, knee, ankle pitch/roll)

### Waist Joints (3 DOF)
- **Waist**: yaw, roll, pitch

### Arm Joints (14 DOF)
- **Left Arm**: 7 joints (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
- **Right Arm**: 7 joints (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)

## Safety Features

### Joint Limit Enforcement
- Position and velocity limits for all joints
- Automatic clamping to safe values
- Warning messages for limit violations

### Emergency Stop
- Automatic stop on high velocity commands
- Timeout-based stopping when no commands received
- Graceful degradation to safe states

### Stability Monitoring
- Real-time stability margin computation
- Capture point analysis
- ZMP constraint enforcement

## Troubleshooting

### Common Issues

1. **MPC Solver Fails**
   - Check cost matrix conditioning
   - Reduce prediction horizon
   - Adjust cost weights

2. **Unstable Motion**
   - Increase balance control gains
   - Reduce step length/velocity
   - Check COM height configuration

3. **Joint Limit Violations**
   - Verify joint limit parameters
   - Check URDF joint definitions
   - Adjust velocity limits

### Debug Information
```bash
# Enable debug output
rosrun rqt_console rqt_console

# Monitor node statistics
rostopic echo /rosout
```

## Development

### Adding New Features
1. Extend the appropriate component class
2. Update the main planner integration
3. Add configuration parameters
4. Update documentation

### Testing
```bash
# Run unit tests
catkin_make run_tests_humanoid_mpc_planner

# Run integration tests
roslaunch humanoid_mpc_planner test_integration.launch
```

## Dependencies

- **ROS**: roscpp, rospy, std_msgs, geometry_msgs, sensor_msgs, nav_msgs
- **TF**: tf2, tf2_ros, tf2_geometry_msgs
- **Control**: control_msgs, trajectory_msgs, actionlib, actionlib_msgs
- **System**: Eigen3, Boost

## License

This package is licensed under the BSD License.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## Support

For issues and questions:
- Check the troubleshooting section
- Review the configuration examples
- Open an issue on the repository
- Contact the maintainers

## Acknowledgments

This MPC planner is designed for the G1 humanoid robot platform and builds upon established control theory and robotics research. 