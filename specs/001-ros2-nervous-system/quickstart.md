# Quickstart: ROS 2 Nervous System Module

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Basic knowledge of command line tools

## Installation

1. **Install Docusaurus** (if not already installed):
   ```bash
   npx create-docusaurus@latest book--creation-classic classic
   ```

2. **Navigate to your project directory**:
   ```bash
   cd book--creation
   ```

3. **Install additional dependencies** (if needed):
   ```bash
   npm install
   ```

## Setting up the ROS 2 Module

1. **Create the ROS 2 content directory**:
   ```bash
   mkdir -p docs/ros2
   ```

2. **Add the three chapter files**:
   - `docs/ros2/overview.md` - ROS 2 Overview
   - `docs/ros2/communication.md` - ROS 2 Communication Primitives
   - `docs/ros2/urdf.md` - Humanoid Modeling with URDF

3. **Update the sidebar configuration** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorial: [
       'intro',
       {
         type: 'category',
         label: 'Module 1 - ROS 2 Nervous System',
         items: [
           'ros2/overview',
           'ros2/communication',
           'ros2/urdf'
         ],
       },
     ],
   };
   ```

4. **Update the main configuration** in `docusaurus.config.js`:
   - Add the new sidebar category to the docs configuration
   - Ensure the docs plugin includes the new content

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