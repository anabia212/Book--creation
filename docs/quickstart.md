# Quickstart Guide: Physical AI & Humanoid Robotics Course

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Basic knowledge of command line tools

## Installation

1. **Clone or download the course repository**
   ```bash
   git clone <repository-url>
   cd book-creation
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

The course is organized into modules, with Module 1 covering the ROS 2 Nervous System:

1. **Introduction to ROS 2 and Robot Middleware** - Understanding ROS 2 architecture and DDS-based communication
2. **ROS 2 Communication Primitives** - Nodes, Topics, Services with rclpy examples
3. **Humanoid Representation with URDF** - Modeling humanoid robots with links, joints, and sensors

## Getting Started

Begin with the Introduction to get an overview of the course, then proceed with Module 1 to learn about ROS 2 as the middleware nervous system for humanoid robots.