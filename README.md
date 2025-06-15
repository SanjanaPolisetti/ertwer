🧭 Multi-Map Navigation with Wormholes (TurtleBot3 + ROS)

This project demonstrates multi-map navigation for a TurtleBot3 robot in simulation using ROS and Gazebo. The robot intelligently switches between different maps (e.g., rooms) using virtual wormholes, allowing it to reach goals across disjoint environments.
🚀 Overview

    🔄 Seamless navigation across multiple maps

    🕳️ Uses wormholes as transition points between rooms

    🧠 Supports direct goal commands to a different map

    📡 Custom ROS Action Server to handle multi-map navigation

    🗃️ Wormhole data managed using an SQLite3 database

🗂️ Project Structure

catkin_ws/
├── src/
│   ├── my_turtlebot3_worlds/          # Custom world for Gazebo
│   ├── multi_map_nav/
│   │   ├── action/                    # MultiMapNav.action
│   │   ├── include/multi_map_nav/     # wormhole_db.hpp
│   │   ├── src/                       # C++ implementation
│   │   └── wormholes.db               # SQLite3 DB for wormholes
│   └── ...
├── maps/
│   ├── room1.yaml
│   ├── room2.yaml

🛠️ Setup Instructions
✅ Prerequisites

    ROS Noetic

    TurtleBot3 packages (turtlebot3_navigation, turtlebot3_slam)

    Gazebo

    sqlite3 (for database)

    Your custom package: multi_map_nav

1. Launch Custom Gazebo World

roslaunch my_turtlebot3_worlds turtlebot3_world.launch

    Ensure this world contains both room1 and room2 (or you simulate them independently).

2. Generate Maps (Optional)

Using SLAM to map each room:

# Start SLAM
roslaunch turtlebot3_slam turtlebot3_slam.launch

# After mapping
rosrun map_server map_saver -f ~/maps/room1

Repeat this for room2.
3. Launch Navigation (start in room2)

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sreejan/maps/room2.yaml

4. Run the Multi-Map Navigation Server

rosrun multi_map_nav multi_map_nav_server

5. Set Up Wormhole Database

sqlite3 ~/catkin_ws/src/multi_map_nav/wormholes.db

Inside SQLite prompt:

-- Delete any previous wormhole
DELETE FROM wormholes WHERE from_map='room2' AND to_map='room1';

-- Add a new wormhole from room2 (exit) to room1 (entry)
INSERT INTO wormholes (from_map, to_map, from_x, from_y, to_x, to_y) 
VALUES ('room2', 'room1', 5.2, 0.0, 3.0, 0.0);

    This means:

        Robot starts in room2

        Wormhole exit: (5.2, 0.0) in room2

        Wormhole entry: (3.0, 0.0) in room1

📡 Sending a Multi-Map Navigation Goal

To send a goal in another map (room1), even though the robot starts in room2, use:

rostopic pub /multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal \
'{goal: {goal_pose: {header: {frame_id: "map"}, pose: {position: {x: 6.0, y: 0.0, z: 0}, orientation: {w: 1.0}}}, target_map: "room1"}}'

🔁 What Happens:

    Robot navigates to wormhole position (5.2, 0.0) in room2

    Switches map to room1

    Relocalizes to entry position (3.0, 0.0) in room1

    Navigates to final goal (6.0, 0.0) in room1

🧠 Architecture Summary
Component	Description
multi_map_nav_server.cpp	Action server managing full navigation flow
wormhole_db.hpp	SQLite-based DB handler for wormhole queries
MultiMapNav.action	Custom action message format
📝 Example Workflow

# 1. Launch simulation world
roslaunch my_turtlebot3_worlds turtlebot3_world.launch

# 2. Launch navigation in current map (room2)
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sreejan/maps/room2.yaml

# 3. Start multi-map action server
rosrun multi_map_nav multi_map_nav_server

# 4. Send navigation goal to second map (room1)
rostopic pub /multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal ...
