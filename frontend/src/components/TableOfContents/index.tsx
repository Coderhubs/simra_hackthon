import React, { ReactNode, useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './styles.module.css';

type Chapter = {
  id: string;
  title: string;
  link: string;
};

export default function TableOfContents(): ReactNode {
  const [chapters, setChapters] = useState<Chapter[]>([]);

  useEffect(() => {
    // In a real Docusaurus setup, you would typically get this from sidebar config
    // or other Docusaurus APIs. For this exercise, we'll hardcode a few based on the docs folder.
    const docChapters: Chapter[] = [
      { id: 'chapter-1', title: 'Chapter 1: Introduction', link: '/docs/chapter-1-introduction' },
      { id: 'chapter-2', title: 'Chapter 2: Kinematics & Dynamics', link: '/docs/chapter-2-kinematics-dynamics' },
      { id: 'chapter-3', title: 'Chapter 3: Sensing & Perception', link: '/docs/chapter-3-sensing-perception' },
      { id: 'chapter-4', title: 'Chapter 4: Robot Control Systems', link: '/docs/chapter-4-robot-control-systems' },
      { id: 'chapter-5', title: 'Chapter 5: Motion Planning & Navigation', link: '/docs/chapter-5-motion-planning-navigation' },
      { id: 'chapter-6', title: 'Chapter 6: Human-Robot Interaction', link: '/docs/chapter-6-human-robot-interaction' },
      { id: 'chapter-7', title: 'Chapter 7: Learning in Robotics', link: '/docs/chapter-7-learning-in-robotics' },
      { id: 'chapter-8', title: 'Chapter 8: Robotics Software (ROS2)', link: '/docs/chapter-8-robotics-software-ros2' },
      { id: 'chapter-9-ethics', title: 'Chapter 9: Ethics & Future', link: '/docs/chapter-9-ethics-future' },
      { id: 'chapter-10', title: 'Chapter 10: Manufacturing Logistics', link: '/docs/chapter-10-manufacturing-logistics' },
      { id: 'chapter-11', title: 'Chapter 11: Isaac Sim Humanoids', link: '/docs/chapter-11-isaac-sim-humanoids' },
      { id: 'chapter-12', title: 'Chapter 12: Manipulation & Grasping', link: '/docs/chapter-12-manipulation-grasping' },
      { id: 'chapter-13', title: 'Chapter 13: Embodied AI & Foundation Models', link: '/docs/chapter-13-embodied-ai-and-foundation-models' },
      { id: 'chapter-14', title: 'Chapter 14: Physical AI for Real-World Robots', link: '/docs/chapter-14-physical-ai-for-real-world-robots' },
      { id: 'chapter-15', title: 'Chapter 15: Final Humanoid Project', link: '/docs/chapter-15-final-humanoid-project' },
    ];
    setChapters(docChapters);
  }, []);

  return (
    <nav className={clsx('col col--3', styles.tableOfContents)}>
      <h3 className={styles.tocHeading}>Table of Contents</h3>
      <ul className={styles.tocList}>
        {chapters.map((chapter) => (
          <li key={chapter.id} className={styles.tocItem}>
            <Link to={chapter.link} className={styles.tocLink}>
              {chapter.title}
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
}
