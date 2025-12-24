// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'ROS 2 Nervous System for Humanoid Robots',
  tagline: 'Educational book on ROS 2 concepts for humanoid robot control',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Exclude node_modules from processing
          path: '.',
          exclude: ['node_modules/**', 'node_modules/**/**'],
          routeBasePath: '/', // Serve the docs at the root path
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'ROS 2 Book',
        logo: {
          alt: 'ROS 2 Logo',
          src: 'img/ros2-logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'module1Sidebar',
            position: 'left',
            label: 'Module 1',
          },
          {
            type: 'docSidebar',
            sidebarId: 'module2Sidebar',
            position: 'left',
            label: 'Module 2',
          },
          {
            type: 'docSidebar',
            sidebarId: 'module3Sidebar',
            position: 'left',
            label: 'Module 3',
          },
          {
            type: 'docSidebar',
            sidebarId: 'module4Sidebar',
            position: 'left',
            label: 'Module 4',
          },
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1: ROS 2 Concepts',
                to: '/module-1/intro-to-ros2',
              },
              {
                label: 'Module 2: Digital Twin Simulation',
                to: '/module-2/gazebo-physics-simulation',
              },
              {
                label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
                to: '/module-3/isaac-sim',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/module-4/voice-to-action',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Educational Project. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;