import React, {useState, useEffect} from 'react';
import {useLocation} from '@docusaurus/router';
import {useCollapsible, Collapsible, usePrevious, findFirstCategoryLink} from '@docusaurus/theme-common';
import {isActiveSidebarItem} from '@docusaurus/theme-common/internal';
import Link from '@docusaurus/Link';
import {translate} from '@docusaurus/Translate';
import useIsBrowser from '@docusaurus/useIsBrowser';
import clsx from 'clsx';
import styles from './styles.module.css';

function DocSidebarItemsRecursive({items, ...props}) {
  const location = useLocation();
  const isBrowser = useIsBrowser();

  return (
    <ul className="menu__list">
      {items.map((item) => (
        <DocSidebarItem
          key={item.href || item.label}
          active={isActiveSidebarItem(item, location)}
          collapsed={isBrowser && !isActiveSidebarItem(item, location)}
          {...props}
          item={item}
        />
      ))}
    </ul>
  );
}

function DocSidebarItem({item, onItemClick, active, collapsed, level}) {
  const {items, href, label, type, customProps = {}} = item;
  const [isExpanded, setIsExpanded] = useState(!collapsed);
  const {
    toggleCollapsed: handleItemClick,
    collapsed: collapsedState,
  } = useCollapsible({
    // Active categories are always initialized as expanded
    initialState: () => {
      if (customProps?.sidebarItemType === 'category') {
        return !active;
      }
      return collapsed;
    },
  });

  useEffect(() => {
    setIsExpanded(!collapsedState);
  }, [collapsedState]);

  switch (type) {
    case 'category':
      return (
        <li
          className={clsx('menu__list-item', {
            'menu__list-item--collapsed': collapsedState,
          })}>
          <Link
            className={clsx('menu__link', {
              'menu__link--sublist': true,
              'menu__link--active': active,
            })}
            onClick={(e) => {
              e.preventDefault();
              handleItemClick();
            }}
            aria-label={translate(
              {
                id: 'theme.docs.sidebar.navLinkTitle',
                message: "Navigate to '{label}'",
                description:
                  'The title attribute for the link in the sidebar that goes to a doc',
              },
              {label},
            )}>
            {label}
            <svg
              className={clsx(
                'dropdown__arrow',
                'navbar__item',
                'navbar__link',
                'navbar__link--active',
                'shadow-sm',
                'rounded',
                'border',
                styles.dropdownArrow,
                {[styles.dropdownArrowExpanded]: isExpanded},
              )}
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              xmlns="http://www.w3.org/2000/svg">
              <path
                d="M5 8L12 15L19 8"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </Link>
          <Collapsible
            lazy
            className="menu__list"
            collapsed={collapsedState}>
            <DocSidebarItemsRecursive
              items={items}
              active={active}
              onItemClick={onItemClick}
              level={level + 1}
            />
          </Collapsible>
        </li>
      );

    case 'link':
    default:
      return (
        <li className="menu__list-item">
          <Link
            className={clsx('menu__link', {
              'menu__link--active': active,
              'border': true,
              'shadow-sm': true,
              'rounded': true,
            })}
            to={href}
            {...(isBrowser && {
              onClick: () => {
                onItemClick?.(item);
              },
            })}>
            {label}
          </Link>
        </li>
      );
  }
}

export default DocSidebarItemsRecursive;