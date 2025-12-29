// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to type-check this file
// even if the project doesn't explicitly use TypeScript.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'Learning ROS 2 as the nervous system of humanoid robots',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://book___creation..vercel.app', // Replace with your actual Vercel URL
  baseUrl: '/', // Root path

  // GitHub pages deployment config (not needed for Vercel)
  organizationName: 'anabia212', // Your GitHub username
  projectName: 'book--creation', // Your GitHub repo name
  deploymentBranch: 'main', // Branch for deployment

  onBrokenLinks: 'throw', // Will throw error if broken links exist
  onBrokenMarkdownLinks: 'warn', // Warn for broken Markdown links

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
          sidebarPath: require.resolve('./sidebars.js'),
          // Change this to your repo for "Edit this page" links
          editUrl: 'https://github.com/anabia212/book___creation/tree/main/',
        },
        blog: false, // disable the blog plugin
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Course',
        logo: {
          alt: 'Course Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
          },
          {
            href: 'https://github.com/anabia212/book___creation',
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
                label: 'Tutorial',
                to: '/docs/intro',
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
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/anabia212/book___creation',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Anabia Asma. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
