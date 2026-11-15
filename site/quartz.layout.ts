import { PageLayout, SharedLayout } from "./quartz/cfg"
import * as Component from "./quartz/components"

// Robotics for People: Quartz v4.5.2 layout. Copied over Quartz's default by tools/site.py prep.
const SITE = "https://markus-learning.github.io/ROBOTICS-for-PEOPLE"

export const sharedPageComponents: SharedLayout = {
  head: Component.Head(),
  header: [],
  afterBody: [],
  footer: Component.Footer({
    links: {
      "Source on GitHub": "https://github.com/MARKUS-LEARNING/ROBOTICS-for-PEOPLE",
      "Help write it": SITE + "/00_Introduction/Frontier",
      "For agents: llms.txt": SITE + "/llms.txt",
      "graph.json": SITE + "/graph.json",
    },
  }),
}

// Explorer: show "04 · Sensors and Perception" instead of "04_Sensors_and_Perception"
const explorer = Component.Explorer({
  title: "Chapters",
  folderDefaultState: "collapsed",
  mapFn: (node) => {
    if (node.isFolder) {
      node.displayName = node.displayName.replace(/^(\d+)_/, "$1 · ").replace(/_/g, " ")
    }
  },
})

export const defaultContentPageLayout: PageLayout = {
  beforeBody: [
    Component.ConditionalRender({
      component: Component.Breadcrumbs(),
      condition: (page) => page.fileData.slug !== "index",
    }),
    Component.ArticleTitle(),
    Component.ContentMeta(),
    Component.TagList(),
  ],
  left: [
    Component.PageTitle(),
    Component.MobileOnly(Component.Spacer()),
    Component.Flex({
      components: [
        { Component: Component.Search(), grow: true },
        { Component: Component.Darkmode() },
        { Component: Component.ReaderMode() },
      ],
    }),
    explorer,
  ],
  right: [
    Component.Graph(),
    Component.DesktopOnly(Component.TableOfContents()),
    Component.Backlinks(),
  ],
}

export const defaultListPageLayout: PageLayout = {
  beforeBody: [Component.Breadcrumbs(), Component.ArticleTitle(), Component.ContentMeta()],
  left: [
    Component.PageTitle(),
    Component.MobileOnly(Component.Spacer()),
    Component.Flex({
      components: [{ Component: Component.Search(), grow: true }, { Component: Component.Darkmode() }],
    }),
    explorer,
  ],
  right: [],
}
