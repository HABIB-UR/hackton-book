// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro-to-ros2',
        'module-1/nodes-topics-services',
        'module-1/python-urdf',
      ],
    },
  ],
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/gazebo-physics-simulation',
        'module-2/unity-interaction-model',
        'module-2/sensor-simulation',
        'module-2/glossary',
      ],
    },
  ],
  module3Sidebar: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/isaac-sim',
        'module-3/isaac-ros',
        'module-3/nav2-navigation',
        'module-3/integration',
        'module-3/glossary',
        'module-3/style-guide',
      ],
    },
  ],
  module4Sidebar: [
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) for Autonomous Humanoids',
      items: [
        'module-4/voice-to-action',
        'module-4/llm-planning',
        'module-4/vision-manipulation',
        'module-4/capstone-autonomous-humanoid',
        'module-4/vla-pipeline-summary',
        'module-4/glossary',
      ],
    },
  ],
};

export default sidebars;