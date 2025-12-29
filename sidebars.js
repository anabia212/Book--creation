// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
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
  ],
};

module.exports = sidebars;