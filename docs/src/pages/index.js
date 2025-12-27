import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro-to-ros2">
            Start Learning ROS 2 - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Educational book on ROS 2 concepts for humanoid robot control">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Introduction to ROS 2</h3>
                <img src="/img/ros2-logo.svg" alt="ROS 2 Logo" className={styles.featureSvg} />
                <p>Learn what ROS 2 is, why robots need middleware, and the core idea of distributed control.</p>
              </div>
              <div className="col col--4">
                <h3>Communication Patterns</h3>
                <img src="/images/communication/humanoid-communication.svg" alt="Humanoid Communication" className={styles.featureSvg} />
                <p>Understand how ROS 2 nodes communicate through topics, services, and actions with practical examples.</p>
              </div>
              <div className="col col--4">
                <h3>Implementation</h3>
                <img src="/images/python-urdf/python-node-structure.svg" alt="Python Node Structure" className={styles.featureSvg} />
                <p>Hands-on examples with Python (rclpy) and URDF for modeling humanoid robot structures.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}