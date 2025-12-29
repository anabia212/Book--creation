// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
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
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action (VLA) Systems',
      items: [
        'vla-systems/voice-to-action',
        'vla-systems/cognitive-planning',
        'vla-systems/autonomous-humanoid'
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
