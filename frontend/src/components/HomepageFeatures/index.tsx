import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Heading from '@theme/Heading';

import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Chapter 1: Introduction',
    description: (
      <>
        Dive into the foundational concepts of Physical AI and Humanoid Robotics.
      </>
    ),
    link: '/docs/chapter-1-introduction',
  },
  {
    title: 'Chapter 2: Kinematics & Dynamics',
    description: (
      <>
        Explore the mechanics of robot movement and forces.
      </>
    ),
    link: '/docs/chapter-2-kinematics-dynamics',
  },
  {
    title: 'Chapter 3: Sensing & Perception',
    description: (
      <>
        Understand how robots perceive their environment.
      </>
    ),
    link: '/docs/chapter-3-sensing-perception',
  },
  {
    title: 'Chapter 13: Embodied AI & Foundation Models',
    description: (
      <>
        Discover the latest in embodied intelligence and large AI models for robotics.
      </>
    ),
    link: '/docs/chapter-13-embodied-ai-and-foundation-models',
  },
];

function Feature({title, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <Link to={link} className={styles.featureLink}>
        <div className="card shadow--md">
          <div className="card__header">
            <Heading as="h3">{title}</Heading>
          </div>
          <div className="card__body">
            <p>{description}</p>
          </div>
          <div className="card__footer">
            <button className="button button--primary button--block">Read More</button>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <>
      {/* Hero Section */}
      <header className={clsx('hero hero--dark', styles.heroBanner)}>
        <div className="container">
          <div className="row">
            {/* Left Column: Text and Button */}
            <div className={clsx('col col--6', styles.heroText)}>
              <Heading as="h1" className="hero__title">
                Physical AI and Humanoid Robotics
              </Heading>
              <p className="hero__subtitle">
                Embodied Intelligence: Bridging the Digital Brain and the Physical Body
              </p>
              <div className={styles.buttons}>
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/chapter-1-introduction">
                  Start Reading the Textbook 📖
                </Link>
              </div>
            </div>

            {/* Right Column: Book Cover Image */}
            <div className={clsx('col col--6', styles.heroImageContainer)}>
              <img
                src="/img/book_cover.png"
                alt="Physical AI & Humanoid Robotics Textbook Cover"
                className={styles.bookCover}
              />
            </div>
          </div>
        </div>
      </header>

      {/* Features Section (Chapter List) */}
      <section className={styles.featuresSection}>
        <div className="container">
          <Heading as="h2" className="text--center margin-bottom--xl">
            Textbook Core Modules
          </Heading>
          <div className="row">
            {FeatureList.map((props, idx) => (
              <Feature key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>
    </>
  );
}