# MPC Motion Planning System Analysis

## How the MPC SMACH System Works

The MPC (Model Predictive Control) system uses a state machine approach for robot motion planning:

### Key Components:

1. **Base Classes:**
   - `MPCPubBase`: Abstract base class handling parameter loading, odometry subscription, and timer management
   - `MPCTrajPtPub`: Handles trajectory point publishing to MPC controller
   - `MPCSinglePtPub`: Handles single point setpoint publishing

2. **Trajectory Classes (trajs.py):**
   - `BaseTraj`: Base trajectory class with common interface
   - `CircleTraj`: Circular trajectory with 1m radius, 10s period
   - Various other trajectories: LemniscateTraj, SetPointTraj, etc.

3. **SMACH State Machine:**
   - **IDLE**: User input for trajectory selection
   - **INIT**: Move to trajectory start position  
   - **TRACK**: Execute trajectory tracking
   - **HAND_CONTROL**: Manual control mode

### Data Flow:

1. Trajectory classes compute desired position/velocity/acceleration
2. `MPCTrajPtPub` publishes `MultiDOFJointTrajectory` messages to `/{robot_name}/set_ref_traj`
3. MPC controller receives trajectory and computes control commands
4. Robot executes motion while tracking errors are calculated

### Configuration Reference:
- Robot params: `/{robot_name}/controller/nmpc/*`
- Servo config: `robots/{robot_name}/config/Servo.yaml`
- Joint limits: `robots/{robot_name}/urdf/*.urdf.xacro`
