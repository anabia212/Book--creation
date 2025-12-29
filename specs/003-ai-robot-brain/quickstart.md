# Quickstart: AI-Robot Brain Module (NVIDIA Isaac™)

## Prerequisites

- Understanding of ROS 2, simulation, and basic perception concepts (as specified in target audience)
- Access to the existing Docusaurus-based course documentation
- Familiarity with the existing course structure from Modules 1 and 2

## Extending the Existing Docusaurus Setup

1. **Navigate to your project directory** (where the existing Docusaurus setup is located):
   ```bash
   cd your-course-directory
   ```

2. **Ensure dependencies are installed** (if not already done):
   ```bash
   npm install
   ```

## Adding Module 3 Content

1. **Create the AI-Brain content directory**:
   ```bash
   mkdir -p docs/ai-brain
   ```

2. **Add the three chapter files**:
   - `docs/ai-brain/isaac-sim.md` - NVIDIA Isaac Sim & Synthetic Data
   - `docs/ai-brain/accelerated-perception.md` - Isaac ROS and Accelerated Perception
   - `docs/ai-brain/humanoid-navigation.md` - Nav2 for Humanoid Navigation

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
       {
         type: 'category',
         label: 'Module 3 - AI-Robot Brain (NVIDIA Isaac™)',
         items: [
           'ai-brain/isaac-sim',
           'ai-brain/accelerated-perception',
           'ai-brain/humanoid-navigation'
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