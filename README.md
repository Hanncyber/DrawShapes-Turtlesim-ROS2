DrawShapes-Turtlesim-ROS2






Overview

This ROS2 project allows a turtle in turtlesim to draw geometric shapes such as heart, star, and flower. It demonstrates ROS2 fundamentals, including node creation, topic communication, and handling user commands.

Features

Draw shapes: heart, star, flower

User commands: clear (reset screen), stop (terminate node)

Python-based node structure for easy extension

Requirements

ROS2 Humble

Python 3.10+

turtlesim package

Installation
# Clone the repository
git clone <your-repo-url>
cd <your-repo-folder>

# Build the workspace
colcon build

# Source the setup file
source install/setup.bash

Usage

Launch the turtlesim node:

ros2 run turtlesim turtlesim_node


Run the shape drawing node:

ros2 run draw_shapes shape_node


Enter commands when prompted:

Command	Description
heart	Draw a heart shape
star	Draw a star shape
flower	Draw a flower shape
clear	Reset the screen
stop	Stop the node
Node

shape_node: Main node that subscribes to user input and publishes commands to /turtle_commander.

Topics

/turtle_commander: Receives commands for drawing shapes and clearing/stopping.

Directory Structure
DrawShapes-Turtlesim-ROS2/
├── src/
│   └── draw_shapes/
│       └── shape_node.py
├── package.xml
├── setup.py
└── README.md

Contributing

Fork the repository

Create a branch: git checkout -b feature/<feature-name>

Commit your changes: git commit -m "Add <feature-name>"

Push and open a pull request
