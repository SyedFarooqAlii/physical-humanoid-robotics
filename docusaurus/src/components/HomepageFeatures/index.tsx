import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
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

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
