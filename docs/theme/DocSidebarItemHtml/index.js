import React from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import type {Props} from '@theme/DocSidebarItem/Html';

export default function DocSidebarItemHtml({item, level}: Props): JSX.Element {
  const {value, className} = item;

  return (
    <li
      className={clsx(
        ThemeClassNames.docs.docSidebarItemHtml,
        `${ThemeClassNames.docs.docSidebarItemHtml}--level-${level}`,
        'menu__list-item',
        className,
      )}
      // eslint-disable-next-line react/no-danger
      dangerouslySetInnerHTML={{__html: value}}
    />
  );
}