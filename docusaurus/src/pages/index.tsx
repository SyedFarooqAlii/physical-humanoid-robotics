import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <div className={styles.badge}>🤖 AI & Robotics</div>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              A comprehensive curriculum covering the cutting-edge intersection of artificial intelligence and humanoid robotics.
              From ROS 2 fundamentals to Vision-Language-Action models, master the technologies that will define the future.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Explore the Book 📚
              </Link>
              <Link
                className="button button--primary button--lg"
                to="/docs/modules/module-1-ros2/overview">
                Start Learning 🚀
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <div className={styles.bookCover}>
              <div className={styles.bookSpine}></div>
              <div className={styles.bookFront}>
                <div className={styles.bookTitle}>Physical AI & Humanoid Robotics</div>
                <div className={styles.bookAuthor}>By Syed Farooq Ali</div>
                <div className={styles.bookIcon}>
                  <svg width="64" height="64" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <rect x="4" y="6" width="24" height="20" rx="4" stroke="#0ea5e9" stroke-width="2" fill="none"/>
                    <circle cx="10" cy="14" r="1.5" fill="#0ea5e9"/>
                    <circle cx="22" cy="14" r="1.5" fill="#0ea5e9"/>
                    <path d="M10 20 C12 22, 20 22, 22 20" stroke="#0ea5e9" stroke-width="2" fill="none" stroke-linecap="round"/>
                    <circle cx="16" cy="8" r="1.5" fill="#0ea5e9"/>
                    <rect x="8" y="24" width="2" height="2" rx="1" fill="#0ea5e9"/>
                    <rect x="22" y="24" width="2" height="2" rx="1" fill="#0ea5e9"/>
                  </svg>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

const FeatureList = [
  {
    title: '🤖 Advanced Robotics Concepts',
    description: (
      <>
        Master cutting-edge humanoid robotics including ROS 2, Digital Twins,
        AI-Brain systems, and Vision-Language-Action models for next-generation robots.
      </>
    ),
  },
  {
    title: '🎓 Structured Curriculum',
    description: (
      <>
        Comprehensive 4-module curriculum covering everything from fundamentals
        to advanced implementation techniques in physical AI.
      </>
    ),
  },
  {
    title: '💡 Real-World Applications',
    description: (
      <>
        Hands-on projects and practical applications bridging the gap between
        theory and real-world humanoid robotics implementation.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Physical AI & Humanoid Robotics Curriculum - Advanced Robotics Education for the Future">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              {FeatureList.map((props, idx) => (
                <Feature key={idx} {...props} />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
