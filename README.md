🧭 Multi-Map Navigation using Wormholes (ROS + TurtleBot3)

This project demonstrates multi-map navigation for a simulated TurtleBot3 robot in Gazebo using ROS. The robot navigates between disconnected environments (like separate rooms) by passing through virtual wormholes and automatically switching maps.
🌐 Key Features

    🔁 Navigate seamlessly across multiple maps

    🕳️ Uses wormholes to connect disjoint areas

    🧭 Accepts direct goal in another map and transitions intelligently

    📡 Implements a ROS Action Server to manage map switching and navigation

    🗃️ Stores wormhole data in an SQLite3 database

📁 Project Structure

📁 Project Structure

    my_turtlebot3_worlds/
    Contains the custom Gazebo world files for simulating the multi-room environment.

    multi_map_nav/
    Your custom ROS package for handling multi-map navigation using wormholes. This includes:

        action/
        Contains the MultiMapNav.action file used to define the custom action interface.

        include/multi_map_nav/wormhole_db.hpp
        C++ header file that defines the interface to interact with the wormhole SQLite database.

        src/multi_map_nav_server.cpp
        Main action server implementation. Handles goal processing, navigation to wormholes, and map switching.

        wormholes.db
        SQLite3 database file that stores wormhole mappings between different maps (e.g., room2 → room1).

    maps/
    Stores the SLAM-generated map files (YAML + PNG) for each room.

        room1.yaml

        room2.yaml

✅ Prerequisites

    ROS Noetic

    TurtleBot3 Packages (turtlebot3_navigation, turtlebot3_slam)

    Gazebo

    sqlite3

🧪 Simulation Setup
1. Launch Gazebo with Custom World

roslaunch my_turtlebot3_worlds turtlebot3_world.launch

Ensure the simulated world has either one combined environment or two separate rooms.
2. Generate Maps for Each Room

Use SLAM to generate maps for room1 and room2:

# Launch SLAM
roslaunch turtlebot3_slam turtlebot3_slam.launch

# After exploring the room
rosrun map_server map_saver -f ~/maps/room1

Repeat for room2 with a different name (e.g., room2.yaml).
3. Launch Navigation in Room2

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sreejan/maps/room2.yaml

This starts the robot in room2 for navigation.
4. Start Multi-Map Navigation Server

rosrun multi_map_nav multi_map_nav_server

5. Configure Wormhole Database

sqlite3 ~/catkin_ws/src/multi_map_nav/wormholes.db

Inside the SQLite prompt:

-- Remove old wormhole entry (optional)
DELETE FROM wormholes WHERE from_map='room2' AND to_map='room1';

-- Add a new wormhole from room2 to room1
INSERT INTO wormholes (from_map, to_map, from_x, from_y, to_x, to_y)
VALUES ('room2', 'room1', 5.2, 0.0, 3.0, 0.0);

    from_x, from_y: Wormhole location in room2

    to_x, to_y: Entry point in room1 after switching maps

📡 Sending a Cross-Map Navigation Goal

To send a goal in a different map (room1), use:

rostopic pub /multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal \
'{goal: {goal_pose: {header: {frame_id: "map"}, pose: {position: {x: 6.0, y: 0.0, z: 0}, orientation: {w: 1.0}}}, target_map: "room1"}}'

🔄 What Happens:

    The robot starts in room2

    It navigates to the wormhole location: (5.2, 0.0)

    The map switches to room1

    Robot spawns at the wormhole entry point: (3.0, 0.0)

    It continues to the final goal: (6.0, 0.0)

🧠 Code Overview
File	Description
multi_map_nav_server.cpp	Handles the goal, wormhole nav, and map swap
wormhole_db.hpp	SQLite3 interface to wormhole DB
MultiMapNav.action	Custom action definition
📌 Example Usage

# 1. Launch simulation world
roslaunch my_turtlebot3_worlds turtlebot3_world.launch

# 2. Launch navigation with room2 map
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sreejan/maps/room2.yaml

# 3. Start action server
rosrun multi_map_nav multi_map_nav_server

# 4. Publish goal in different map (room1)
rostopic pub /multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal ...

🔧 Future Improvements

Add service-based clean map switching

Improve relocalization after teleporting

Integrate TF map frame switching

Support bidirectional wormholes
