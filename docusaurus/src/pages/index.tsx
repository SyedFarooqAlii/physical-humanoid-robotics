import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

/* ================= HERO ================= */

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx(styles.hero)}>
      <div className="container">
        <div className={styles.heroGrid}>
          {/* LEFT */}
          <div className={styles.heroText}>
            <span className={styles.badge}>🤖 AI & Robotics Book</span>

            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>

            <p className={styles.heroSubtitle}>
              {siteConfig.tagline}
            </p>

            <p className={styles.heroDescription}>
              A modern, industry-focused book covering Physical AI and Humanoid Robotics.
              Learn ROS 2, Digital Twins, Vision-Language-Action models, and real-world robotics systems
              with a clean and structured learning path.
            </p>

            <div className={styles.heroButtons}>
              <Link className={styles.primaryBtn} to="/docs/intro">
                📘 Explore the Book
              </Link>
              <Link className={styles.secondaryBtn} to="/docs/modules/module-1-ros2/overview">
                🚀 Start Learning
              </Link>
            </div>
          </div>

          {/* RIGHT */}
          <div className={styles.heroVisual}>
            <div className={styles.book}>
              <div className={styles.bookCover}>
                <h3>Physical AI</h3>
                <p>& Humanoid Robotics</p>
                <span>By Syed Farooq Ali</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

/* ================= FEATURES ================= */

const FeatureList = [
  {
    title: 'Advanced Robotics',
    text: 'Master cutting-edge humanoid robotics including ROS 2, Digital Twins,AI-Brain systems, and Vision-Language-Action models for next-generation robots.',
    icon: '🤖',
  },
  {
    title: 'Structured Learning',
    text: 'Comprehensive 4-module curriculum covering everything from fundamentals to advanced implementation techniques in physical AI.',
    icon: '📚',
  },
  {
    title: 'Real-World Projects',
    text: 'Hands-on projects and practical applications bridging the gap between theory and real-world humanoid robotics implementation.',
    icon: '⚙️',
  },
];

function FeatureCard({ title, text, icon }) {
  return (
    <div className={styles.featureCard}>
      <span className={styles.featureIcon}>{icon}</span>
      <Heading as="h3">{title}</Heading>
      <p>{text}</p>
    </div>
  );
}

/* ================= EXTRA BOOK SECTION ================= */

function AboutBook() {
  return (
    <section className={styles.about}>
      <div className="container">
        <Heading as="h2">Why This Book?</Heading>
        <p>
          This book is designed for students, developers, and researchers who want
          to enter the future of robotics and physical AI.
          It blends theory with real-world engineering practices.
        </p>
      </div>
    </section>
  );
}

/* ================= HOME ================= */

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Modern curriculum for Physical AI and Humanoid Robotics"
    >
      <HomepageHeader />

      <main>
        <section className={styles.features}>
          <div className="container">
            <div className={styles.featureGrid}>
              {FeatureList.map((item, index) => (
                <FeatureCard key={index} {...item} />
              ))}
            </div>
          </div>
        </section>

        <AboutBook />
      </main>
    </Layout>
  );
}
