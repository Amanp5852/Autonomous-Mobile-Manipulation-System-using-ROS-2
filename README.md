# Autonomous Mobile Manipulation System
This project presents a complete autonomous mobile manipulation system developed using ROS 2 Humble and Gazebo Harmonic.

The robot autonomously detects coloured objects, navigates to their pickup locations, grasps them using a robotic manipulator, transports them to their corresponding coloured bins, and returns to continue the next task without human intervention.

The objective of this project was not only to build an autonomous robot, but also to understand the complete software architecture behind modern robotic systems by implementing the navigation, manipulation, perception and task management modules independently before integrating them into a single autonomous workflow.

# Features:
✔ Autonomous Navigation
✔ Computer Vision
✔ Mobile Manipulation
✔ Task Planning
✔ Differential Drive Robot
✔ Robotic Arm
✔ Parallel Gripper
✔ Gazebo Physics
✔ ROS2 Modular Architecture

# System Architecture:
                     Camera Node
                         │
                         ▼
                  Task Manager
                  /          \
                 /            \
        Navigation      Manipulation
                 \            /
                  \          /
                    Gazebo Robot

# Camera Node
Responsible for colour detection using OpenCV.
Publishes detected object colour to the Task Manager.

# Task Manager
Acts as the high-level decision maker.
Coordinates navigation, manipulation and mission sequencing.

# Navigation Node
Responsible for autonomous robot navigation using waypoint generation and PI heading control.

# Manipulation Node
Controls the robotic arm and gripper.
Executes calibrated pickup, transport and placement motions.

# Project Workflow
Mission Start
↓
Inspection
↓
Object Detection
↓
Navigation
↓
Pickup
↓
Navigation
↓
Placement
↓
Return Home
↓
Repeat
↓
Mission Complete

# Repository Structure

 > my_robot_description
    Contains
      URDF
      Xacro
      Robot model
      Gazebo plugins
> my_robot_bringup
    Contains
      Launch files
      Gazebo World
      RViz
      ROS-Gazebo Bridge
> my_robot_controller
    Contains
      navigation_node_v2.py
      manipulation_node.py
      camera_node.py
      task_manager_node.py
      poses.py
      location.py
      config.py

# Navigation:
  ## Autonomous Navigation
  Navigation Node is responsible for generating waypoint based trajectories between the robot's current position and the desired destination.
  The controller performs
  Goal
  ↓
  Waypoint Generation
  ↓
  Heading Correction
  ↓
  Linear Motion
  ↓
  Final Alignment

# Manipulation
  ## Robotic Manipulation
  The manipulation node executes calibrated arm motions for pickup, transport and placement.
  Motion sequence
  Move Joint0
  ↓
  Move Joint1
  ↓
  Move Joint2
  ↓
  Move Joint3
  ↓
  Close Gripper
  ↓
  Transport Pose
  ↓
  Placement
  ↓
  Return Home

# Computer Vision
  ## Computer Vision
  Image
  ↓
  HSV Threshold
  ↓
  Colour Detection
  ↓
  Detected Colour
  ↓
  Task Manager

# Task Manager
  START
  ↓
  Inspection
  ↓
  WAIT_FOR_COLOR
  ↓
  GO_TO_PICKUP
  ↓
  WAIT_FOR_NAVIGATION
  ↓
  PICKUP
  ↓
  WAIT_FOR_MANIPULATION
  ↓
  GO_TO_PLACE
  ↓
  PLACE
  ↓
  RETURN_HOME
  ↓
  MISSION_COMPLETE

# Robot Dynamics & Physics
  ## Robot Dynamics and Gazebo Physics
  Developing a reliable manipulation system required extensive tuning of Gazebo's physical parameters.
  Key optimizations included:
  ✔ Wheel-ground friction tuning
  ✔ Joint damping optimization
  ✔ Joint controller gain tuning
  ✔ Contact parameter optimization
  ✔ Gripper friction adjustment
  ✔ Velocity decay tuning
  ✔ Cube stability after placement
  ✔ Gravity compensation during manipulation
  These optimizations significantly improved grasp stability, navigation repeatability and overall simulation realism.

# Engineering Challenges & Solutions
  1. Autonomous Navigation
  **Challenge**
  Achieving repeatable and accurate navigation proved challenging using odometry-based localization alone. Small heading errors accumulated during successive movements,       causing the robot to stop with slight orientation deviations that affected the arm's alignment during object pickup.
  **Solution**
  A custom waypoint-based navigation system was developed using an orthogonal motion strategy. A PI heading controller with final orientation alignment was implemented to     improve heading accuracy, while waypoint generation ensured repeatable navigation independent of the target location.
  
  2. Robotic Manipulation
  **Challenge**
  The robotic arm initially struggled to perform reliable pick-and-place operations due to calibration errors, joint synchronization, and gripper alignment. Even small         positioning inaccuracies prevented successful grasping.
  **Solution**
  The manipulation system was redesigned as a sequential state machine controlling each joint independently. Dedicated calibrated poses were created for pickup, transport,     placement, and home configurations, allowing consistent manipulation for multiple object locations.
  
  3. Robot Dynamics & Gazebo Physics
  **Challenge**
  Simulation realism significantly affected the robot's performance. The arm oscillated under gravity, cubes slipped during grasping, and placed objects frequently rolled     away due to insufficient contact modelling.
  **Solution**
  Extensive tuning of Gazebo physics parameters was carried out, including joint damping, friction coefficients, controller gains, contact properties, restitution, and         velocity decay. These improvements resulted in more stable arm motion, reliable grasping, and realistic object interaction.
  
  4. System Integration
  **Challenge**
  Integrating perception, navigation, and manipulation into a single autonomous workflow introduced synchronization challenges between multiple ROS2 nodes. Commands needed     to be executed in the correct sequence without conflicts.
  **Solution**
  A dedicated Task Manager node was developed to coordinate the complete mission using ROS2 publishers and subscribers. Independent state machines were designed for           navigation and manipulation, while the Task Manager orchestrated communication between all subsystems through clearly defined interfaces.
  
  5. Software Architecture
  **Challenge**
  As the project grew, maintaining readability and scalability became increasingly difficult. Direct dependencies between navigation, manipulation, and perception would       have made future modifications complex.
  **Solution**
  The software was organized into modular ROS2 packages following the principle of separation of responsibilities. Each subsystem—Camera, Navigation, Manipulation, and Task   Manager—was implemented as an independent node communicating exclusively through ROS2 topics. This architecture simplified testing, debugging, and future extensibility.
  
  6. Motion Repeatability
  **Challenge**
  Accurate manipulation required the mobile robot to arrive at highly repeatable pickup and placement positions. Small navigation deviations directly impacted the success     of the grasping sequence.
  **Solution**
  The navigation strategy was redesigned to use orthogonal movement between waypoints rather than arbitrary diagonal paths. This reduced orientation uncertainty, simplified   arm calibration, and significantly improved pickup repeatability.
  
  7. Controller Tuning
  **Challenge**
  Balancing responsiveness and stability required careful tuning of both the mobile base and manipulator controllers. High controller gains produced oscillations, while       lower gains reduced positioning accuracy.
  **Solution**
  Controller gains, motion tolerances, and joint dynamics were iteratively tuned through extensive testing until a balance between stability, precision, and smooth motion     was achieved.

# Technologies Used
ROS2 Jazzy
Gazebo Harmonic
Python
OpenCV
URDF
Xacro
RViz2
Robot State Publisher
ROS-Gazebo Bridge

# Future Improvements:
□ Nav2 Integration
□ SLAM
□ AprilTag Localization
□ YOLO Object Detection
□ MoveIt Integration
□ Real Robot Deployment
