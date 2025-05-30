# Maze Solver Robot using Arduino

This project is a **Maze Solving Robot** built using **Arduino**, designed to autonomously explore and solve a maze using sensor-based navigation and intelligent pathfinding.

## How It Works

The bot operates in two distinct phases:

### 1. Dry Run (Exploration Phase)
The robot starts by traversing the maze from the starting point to the goal. During this run, it identifies and records all possible paths and junctions without optimizing for speed or distance. This phase helps in building a mental map of the maze.

### 2. Actual Run (Shortest Path Execution)
After the dry run is complete, the robot processes the recorded data to calculate the shortest path. It then reruns through the maze, following the optimal route to reach the goal efficiently.

## Key Features

- **PID Control**: Implements a Proportional-Integral-Derivative control algorithm for smooth and stable motion, especially around curves and turns.
- **Autonomous Navigation**: Uses IR sensors to detect lines and junctions for intelligent decision-making.
- **Path Memory**: Stores decision data during the dry run to identify the most efficient path.
- **Arduino Based**: The bot’s core logic is developed using Arduino's C/C++ environment.

## Components Used

- Arduino Uno
- IR Sensors (for line detection)
- L298N Motor Driver
- DC Motors with wheels
- Chassis and structural parts
- Power supply module

## Learning Outcomes

Through this project, I gained practical experience in:

- Embedded system programming
- PID control tuning and implementation
- Algorithmic pathfinding in physical environments
- Hardware-software integration in autonomous robotics
