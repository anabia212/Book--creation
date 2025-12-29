# Quickstart: Digital Twin Module (Gazebo & Unity)

## Prerequisites

- Basic knowledge of ROS 2 and Python (as specified in target audience)
- Understanding of the existing course structure from Module 1 (ROS 2)
- Access to the existing Docusaurus-based course documentation

## Extending the Existing Docusaurus Setup

1. **Navigate to your project directory** (where the existing Docusaurus setup is located):
   ```bash
   cd your-course-directory
   ```

2. **Ensure dependencies are installed** (if not already done):
   ```bash
   npm install
   ```

## Adding Module 2 Content

1. **Create the Digital Twin content directory**:
   ```bash
   mkdir -p docs/digital-twin
   ```

2. **Add the three chapter files**:
   - `docs/digital-twin/physics-simulation.md` - Physics Simulation with Gazebo
   - `docs/digital-twin/unity-environments.md` - Digital Twin Environments with Unity
   - `docs/digital-twin/sensor-simulation.md` - Sensor Simulation (LiDAR, Depth Cameras, IMUs)

3. **Update the sidebar configuration** in `sidebars.js` to include the new module:
   ```javascript
   module.exports = {
     tutorialSidebar: [
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
       {
         type: 'category',
         label: 'Module 2 - Digital Twin (Gazebo & Unity)',
         items: [
           'digital-twin/physics-simulation',
           'digital-twin/unity-environments',
           'digital-twin/sensor-simulation'
         ],
       },
     ],
   };
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