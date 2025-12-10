import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Quick Start',
      items: [
        'quickstart/setup-guide',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/module-1-ros2/overview',
        'modules/module-1-ros2/chapter-spec-template',
        'modules/module-1-ros2/chapter-1-1-basics',
        'modules/module-1-ros2/chapter-1-2-communication',
        'modules/module-1-ros2/chapter-1-3-nodes-topics-services',
        'modules/module-1-ros2/chapter-1-4-actions-advanced-interfaces',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-2-digital-twin/overview',
        'modules/module-2-digital-twin/chapter-2-1-gazebo-simulation',
        'modules/module-2-digital-twin/chapter-2-2-unity-visualization',
        'modules/module-2-digital-twin/chapter-2-3-sim-to-real-transfer',
        'modules/module-2-digital-twin/chapter-2-4-digital-twin-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'modules/module-3-ai-brain/overview',
        'modules/module-3-ai-brain/chapter-3-1-isaac-sim-basics',
        'modules/module-3-ai-brain/chapter-3-2-perception-pipelines',
        'modules/module-3-ai-brain/chapter-3-3-planning-algorithms',
        'modules/module-3-ai-brain/chapter-3-4-control-systems',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-4-vla/overview',
        'modules/module-4-vla/chapter-4-1-vision-language-models',
        'modules/module-4-vla/chapter-4-2-speech-recognition-whisper',
        'modules/module-4-vla/chapter-4-3-llm-integration',
        'modules/module-4-vla/chapter-4-4-vla-integration-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: The Autonomous Humanoid',
      items: [
        'capstone/overview',
        'capstone/autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        {
          type: 'category',
          label: 'Tutorial Basics',
          items: [
            'tutorial-basics/create-a-document',
            'tutorial-basics/create-a-page',
            'tutorial-basics/create-a-blog-post',
            'tutorial-basics/markdown-features',
            'tutorial-basics/congratulations',
          ],
        },
        {
          type: 'category',
          label: 'Tutorial Extras',
          items: [
            'tutorial-extras/manage-docs-versions',
            'tutorial-extras/translate-your-site',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
