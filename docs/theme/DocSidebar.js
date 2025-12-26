import React from 'react';
import {
  useScrollPosition,
  useThemeConfig,
  useAnnouncementBar,
} from '@docusaurus/theme-common';
import {
  splitSidebarItems,
  usePrevious,
} from '@docusaurus/theme-common/internal';
import {translate} from '@docusaurus/Translate';
import clsx from 'clsx';
import {
  NavbarSecondaryMenuFiller,
  ThemeClassNames,
} from '@docusaurus/theme-common';
import ComponentTypes from '@theme/ComponentTypes';
import DocSidebarItems from '@theme/DocSidebarItems';

// TODO temporary casting until it becomes a theme API
const DefaultNavbarItem = ComponentTypes.navbarItem;
const DocSidebarItemsExpanded = (props) => {
  const [expandedItem, setExpandedItem] = React.useState(undefined);

  const expand = (itemIndex) => {
    setExpandedItem(itemIndex);
  };

  const collapse = (itemIndex) => {
    setExpandedItem(expandedItem === itemIndex ? undefined : itemIndex);
  };

  return (
    <DocSidebarItems
      {...props}
      expandItem: (itemIndex) => expand(itemIndex),
      collapseItem: (itemIndex) => collapse(itemIndex),
      isItemCollapsed: (itemIndex) => expandedItem !== itemIndex,
    />
  );
};

function useAnnouncementBarPlaceholderHeight() {
  const {isActive} = useAnnouncementBar();
  const [placeholderHeight, setPlaceholderHeight] = React.useState(0);
  const placeholderRef = React.useRef(null);
  useScrollPosition(
    () => {
      if (placeholderRef.current) {
        setPlaceholderHeight(placeholderRef.current.offsetHeight);
      }
    },
    [isActive],
  );
  return {ref: placeholderRef, height: placeholderHeight};
}

export default function DocSidebar(props) {
  const {
    sidebar,
    path: currentPath,
    onCollapse: setCollapsed,
    isHidden,
  } = props;
  const [showResponsiveSidebar, setShowResponsiveSidebar] = React.useState(false);
  const {
    navbar: {hideOnScroll},
    sidebar: {autoCollapseCategories, autoExpandCategories},
  } = useThemeConfig();
  const responsiveSidebarPlaceholder = useAnnouncementBarPlaceholderHeight();
  const previousIsHidden = usePrevious(isHidden);

  React.useEffect(() => {
    if (isHidden && !previousIsHidden) {
      setShowResponsiveSidebar(false);
    }
  }, [isHidden, previousIsHidden]);

  const {left: items, right: metaItems} = splitSidebarItems(sidebar);

  return (
    <>
      <div
        className={clsx(
          ThemeClassNames.docs.docSidebarContainer,
          'menu',
          'menu--responsive',
          'border-0',
          'shadow-sm',
          'rounded',
          'p-3',
          {
            'menu--show': showResponsiveSidebar,
            'border': false,
          },
        )}
        onTransitionEnd={(e) => {
          if (e.propertyName === 'width' && !showResponsiveSidebar) {
            setCollapsed?.(true);
          }
        }}>
        <div className="menu__content">
          <div className="border-bottom pb-3 mb-3">
            <DefaultNavbarItem
              className="margin-0"
              collapsible={false}
              type="search"
            />
          </div>
          <nav
            className={clsx('menu__wrapper', {
              'menu__wrapper--show': showResponsiveSidebar,
            })}
            role="menubar"
            // Mobile sidebar uses custom scroll
            {...(!showResponsiveSidebar && responsiveSidebarPlaceholder)}>
            <DocSidebarItemsExpanded items={left} activePath={currentPath} />
          </nav>
        </div>
        {metaItems.length > 0 && (
          <div className="border-top pt-3 mt-3">
            <DocSidebarItemsExpanded items={metaItems} activePath={currentPath} />
          </div>
        )}
      </div>
      <div
        className={clsx('navbar-sidebar__backdrop', {
          'navbar-sidebar__backdrop--show': showResponsiveSidebar,
        })}
        onClick={() => {
          setShowResponsiveSidebar(false);
        }}
      />
      <NavbarSecondaryMenuFiller
        component={DefaultNavbarItem}
        props={{
          className: 'doc-sidebar__toggle',
          'aria-label': translate({
            id: 'theme.docs.sidebar.collapseButtonAriaLabel',
            message: 'Collapse sidebar',
            description: 'The ARIA label for the sidebar collapse button',
          }),
          onClick: () => {
            setShowResponsiveSidebar(!showResponsiveSidebar);
          },
          onKeyDown: (e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              e.preventDefault();
              setShowResponsiveSidebar(!showResponsiveSidebar);
            }
          },
        }}
        style={{display: 'block'}}
        order={hideOnScroll ? 2 : 1}
      />
    </>
  );
}