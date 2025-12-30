# Quickstart Guide: Physical AI & Humanoid Robotics Course

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Basic knowledge of command line tools

## Installation

1. **Clone or download the course repository**
   ```bash
   git clone https://github.com/anabia212/Book--creation.git
   cd book_frontend

   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

## Running the Development Server

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Building for Production

```bash
npm run build
```

This command generates static content into the `build` directory that can be served using any static hosting service.

## Deployment

The built site can be deployed to GitHub Pages by following the Docusaurus deployment guide. The site will be accessible at `https://<username>.github.io/<repository>`.

## Course Structure

The course is organized into four comprehensive modules:

### Module 1: ROS 2 Nervous System
1. **Introduction to ROS 2 and Robot Middleware** - Understanding ROS 2 architecture and DDS-based communication
2. **ROS 2 Communication Primitives** - Nodes, Topics, Services with rclpy examples
3. **Humanoid Representation with URDF** - Modeling humanoid robots with links, joints, and sensors

### Module 2: Digital Twin (Gazebo & Unity)
1. **Physics Simulation with Gazebo** - Simulating gravity, collisions, and dynamics for humanoid behavior
2. **Digital Twin Environments with Unity** - Visual realism and human-robot interaction
3. **Sensor Simulation** - LiDAR, depth cameras, and IMUs for realistic sensor data

### Module 3: AI-Robot Brain (NVIDIA Isaac™)
1. **NVIDIA Isaac Sim & Synthetic Data** - Photorealistic simulation environments and synthetic data generation for training perception models
2. **Isaac ROS and Accelerated Perception** - Hardware-accelerated VSLAM and perception pipelines leveraging NVIDIA's GPU acceleration capabilities
3. **Nav2 for Humanoid Navigation** - Navigation stack fundamentals and path planning specifically designed for bipedal humanoid movement patterns

### Module 4: Vision-Language-Action (VLA) Systems
1. **Voice-to-Action Interfaces** - Speech-to-text using OpenAI Whisper and converting spoken commands into structured intents
2. **Cognitive Planning with LLMs** - Translating natural language goals into ROS 2 action sequences and high-level reasoning and task decomposition
3. **Capstone: The Autonomous Humanoid** - End-to-end VLA pipeline with voice command → planning → navigation → perception → manipulation

## Getting Started

Begin with the Introduction to get an overview of the course, then proceed through the modules in sequence to learn about ROS 2 as the middleware nervous system, Digital Twin technologies, and the AI-Robot Brain components for advanced robotics applications.