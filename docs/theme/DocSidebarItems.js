import React, {useState} from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDocSidebarItemsExpanded, useDocSidebarItemsCollapsed} from '@docusaurus/theme-common/internal';
import DocSidebarItem from '@theme/DocSidebarItem';

export default function DocSidebarItems({items, ...props}) {
  const [expandedCategories, setExpandedCategories] = useState(new Set());

  const toggleCategory = (index) => {
    const newExpanded = new Set(expandedCategories);
    if (newExpanded.has(index)) {
      newExpanded.delete(index);
    } else {
      newExpanded.add(index);
    }
    setExpandedCategories(newExpanded);
  };

  if (items?.length === 0) {
    return null;
  }

  return (
    <ul className={clsx(ThemeClassNames.docs.docSidebarMenu, 'menu__list')}>
      {items.map((item, index) => (
        <DocSidebarItem
          key={index}
          active={props.activePath === item.href}
          level={1}
          index={index}
          item={item}
          onCollapse={() => toggleCategory(index)}
          isCollapsed={!expandedCategories.has(index)}
          {...props}
        />
      ))}
    </ul>
  );
}