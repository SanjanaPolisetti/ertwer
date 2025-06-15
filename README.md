🗺️ Multi-Map Navigation with Wormholes (TurtleBot3, ROS)

This project implements autonomous multi-map navigation using TurtleBot3 in Gazebo simulation. It enables a robot to move between different mapped environments (rooms) using "wormholes", with map transitions handled dynamically via an SQL database.
🧠 Features

    🔁 Autonomous multi-map navigation

    🌐 Map switching via SQL-defined wormholes

    📦 ROS action server to handle navigation requests across maps

    ✅ Compatible with TurtleBot3 and standard move_base navigation stack

    📡 Publishes navigation goals using a custom action interface

🧰 Requirements

    ROS Noetic

    TurtleBot3 packages

    Gazebo

    sqlite3 (for wormhole database)

    Custom package: multi_map_nav

📁 Directory Structure

catkin_ws/
├── src/
│   ├── my_turtlebot3_worlds/          # Custom world for Gazebo
│   ├── multi_map_nav/                 # This package
│   │   ├── action/                    # Contains MultiMapNav.action
│   │   ├── src/                       # Server C++ code
│   │   ├── include/multi_map_nav/     # Header for wormhole DB
│   │   └── wormholes.db               # SQLite3 wormhole database
│   └── ...
├── maps/
│   ├── room1.yaml                     # Map of Room 1
│   ├── room2.yaml                     # Map of Room 2

🛠️ Setup Instructions
1. Launch Custom World in Gazebo

roslaunch my_turtlebot3_worlds turtlebot3_world.launch

2. Generate Maps (if not already done)

For each room:

roslaunch turtlebot3_slam turtlebot3_slam.launch
# Drive robot to map the room
# Save map after mapping
rosrun map_server map_saver -f ~/maps/room1

Repeat for room2.
3. Launch Navigation with Room2 as Starting Map

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sreejan/maps/room2.yaml

4. Run the Multi-Map Navigation Server

rosrun multi_map_nav multi_map_nav_server

5. Set Up Wormhole Database

sqlite3 wormholes.db

-- View schema
.schema wormholes

-- Insert wormhole from room2 to room1
INSERT INTO wormholes (from_map, to_map, from_x, from_y, to_x, to_y) 
VALUES ('room2', 'room1', 5.2, 0.0, 3.0, 0.0);

This means:

    The robot exits room2 at (5.2, 0.0)

    It enters room1 at (3.0, 0.0)

🧪 Sending a Multi-Map Goal

You can send a goal in the second map (room1) while robot is currently in room2:

rostopic pub /multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal \
'{goal: {goal_pose: {header: {frame_id: "map"}, pose: {position: {x: 6.0, y: 0.0, z: 0}, orientation: {w: 1.0}}}, target_map: "room1"}}'

🧭 This will:

    Navigate robot to wormhole in room2 (5.2, 0.0)

    Switch map to room1

    Initialize robot at wormhole exit (3.0, 0.0)

    Navigate to final goal in room1 (6.0, 0.0)

🧱 Architecture

    MultiMapNavAction (C++): ROS Action Server that

        Accepts final pose and target map

        Reads wormhole position from SQLite

        Sends goals to move_base

        Switches map via system call

    wormhole_db.hpp: Loads wormhole coordinates from SQLite3

    MultiMapNav.action: Custom action interface
