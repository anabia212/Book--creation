# Data Model: ROS 2 Nervous System Module

## Course Module Structure

### Module Entity
- **Name**: ROS 2 Nervous System Module
- **Description**: Educational module covering ROS 2 as middleware for humanoid robots
- **Chapters**: 3 (Introduction, Communication Primitives, URDF Modeling)
- **Target Audience**: AI and robotics students with basic Python knowledge

### Chapter Entity
- **ID**: Unique identifier for the chapter
- **Title**: Display title of the chapter
- **Content**: Markdown content of the chapter
- **Order**: Sequential order in the module (1-3)
- **Learning Objectives**: Specific outcomes for the chapter

### Chapter 1: ROS 2 Overview
- **ID**: ros2-overview
- **Title**: Introduction to ROS 2 and Robot Middleware
- **Content**: What ROS 2 is, why it matters for Physical AI, ROS 2 architecture, DDS-based communication, role of middleware
- **Order**: 1
- **Learning Objectives**: Understand ROS 2 as middleware, explain DDS communication, describe middleware role in humanoid robot control

### Chapter 2: ROS 2 Communication Primitives
- **ID**: ros2-communication
- **Title**: ROS 2 Nodes, Topics, Services with rclpy
- **Content**: Nodes, Topics, Services, message passing, real-time constraints, rclpy examples
- **Order**: 2
- **Learning Objectives**: Create ROS 2 nodes, implement Topics and Services, connect Python AI agents to robot controllers

### Chapter 3: URDF Modeling
- **ID**: urdf-modeling
- **Title**: Humanoid Representation with URDF
- **Content**: URDF purpose, links and joints, sensors, simulation-to-reality transfer
- **Order**: 3
- **Learning Objectives**: Create URDF models, define links and joints, explain simulation-to-reality transfer

## Navigation Structure

### Sidebar Category
- **Label**: Module 1 - ROS 2 Nervous System
- **Items**: Array of chapter references in order
- **Collapsed**: Default state for the category

### Chapter Reference
- **Type**: "doc"
- **Id**: Unique chapter identifier
- **Label**: Display name for navigation