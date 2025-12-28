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
    {
      type: 'category',
      label: 'My AI Robotics Textbook',
      items: [
        'chapter-1-introduction',
        'chapter-2-kinematics-dynamics',
        'chapter-3-sensing-perception',
        'chapter-4-robot-control-systems',
        'chapter-5-motion-planning-navigation',
        'chapter-6-human-robot-interaction',
        'chapter-7-learning-in-robotics',
        'chapter-8-robotics-software-ros2',
        'chapter-9-ethics-future',
        'chapter-9-healthcare',
        'chapter-10-manufacturing-logistics',
        'chapter-11-isaac-sim-humanoids',
        'chapter-12-manipulation-grasping',
        'chapter-13-embodied-ai-and-foundation-models',
        'chapter-14-physical-ai-for-real-world-robots',
        'chapter-15-final-humanoid-project',
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
