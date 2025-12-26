import React from 'react';
import clsx from 'clsx';
import {useLocation} from '@docusaurus/router';
import {useThemeConfig} from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarLogo from '@theme/NavbarLogo';
import NavbarSearch from '@theme/NavbarSearch';
import NavbarColorModeToggle from '@theme/NavbarColorModeToggle';
import ContentVisibility from '@theme/Navbar/ContentVisibility';
import styles from './styles.module.css';
import {translate} from '@docusaurus/Translate';

function useNavbarItems() {
  // TODO temporary casting until it becomes a theme API
  const {navbar: {items}} = useThemeConfig();
  return items;
}

function NavbarItems({items}) {
  return (
    <>
      {items.map((item, i) => (
        <NavbarItem {...item} key={i} />
      ))}
    </>
  );
}

function NavbarContentLayout({left, right}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

export default function Navbar() {
  const mobileSidebar = useNavbarMobileSidebar();
  const items = useNavbarItems();
  const {leftItems, rightItems} = splitNavbarItems(items);
  const location = useLocation();

  // Add a class based on the current path for better styling
  const navbarClass = clsx(
    'navbar',
    'navbar--fixed-top',
    'shadow-sm',
    'rounded',
    'mb-3',
    'border',
    {
      'navbar-sidebar__show': mobileSidebar.shown,
    },
  );

  return (
    <nav
      ref={mobileSidebar.sidebarRef}
      className={navbarClass}
      role="banner">
      <div className="container-fluid navbar__inner">
        <NavbarContentLayout
          left={
            // TODO stop hardcoding items?
            <>
              <NavbarLogo />
              <NavbarItems items={leftItems} />
            </>
          }
          right={
            // TODO stop hardcoding items?
            <>
              <NavbarItems items={rightItems} />
              <NavbarColorModeToggle className="margin-0" />
              <NavbarSearch />
            </>
          }
        />
      </div>
      <div
        role="presentation"
        className="navbar-sidebar__backdrop"
        onClick={mobileSidebar.toggle}
      />
    </nav>
  );
}