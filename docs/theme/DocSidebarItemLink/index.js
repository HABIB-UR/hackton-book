import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {isActiveSidebarItem} from '@docusaurus/theme-common/internal';
import {translate} from '@docusaurus/Translate';
import useIsBrowser from '@docusaurus/useIsBrowser';
import type {Props} from '@theme/DocSidebarItem/Link';

export default function DocSidebarItemLink({
  item,
  onItemClick,
  activePath,
  level,
  ...props
}: Props): JSX.Element {
  const {href, label, className} = item;
  const isActive = isActiveSidebarItem(item, activePath);
  const isBrowser = useIsBrowser();

  return (
    <li className="menu__list-item">
      <Link
        className={clsx(
          'menu__link',
          {
            'menu__link--active': isActive,
          },
          className,
          // Enhanced visual hierarchy styling
          'rounded',
          'border',
          'ps-3',
          level > 1 ? 'ps-4' : '',
          level > 2 ? 'ps-5' : '',
        )}
        aria-current={isActive ? 'page' : undefined}
        to={href}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            onItemClick?.(item);
          }
        }}
        tabIndex={0} // Make keyboard accessible
        {...(isBrowser && {
          onClick: () => {
            onItemClick?.(item);
          },
        })}
        {...props}>
        {label}
      </Link>
    </li>
  );
}