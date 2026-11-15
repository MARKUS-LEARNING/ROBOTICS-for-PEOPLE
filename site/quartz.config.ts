import { QuartzConfig } from "./quartz/cfg"
import * as Plugin from "./quartz/plugins"

// Robotics for People: Quartz v4.5.2 configuration.
// tools/site.py prep copies this file over Quartz's default before each build; edit it here, not in the Quartz checkout.
const config: QuartzConfig = {
  configuration: {
    pageTitle: "Robotics for People",
    pageTitleSuffix: " · Robotics for People",
    enableSPA: true,
    enablePopovers: true,
    analytics: null, // no tracking
    locale: "en-US",
    baseUrl: "markus-learning.github.io/ROBOTICS-for-PEOPLE",
    ignorePatterns: ["private", "templates", ".obsidian"],
    defaultDateType: "modified",
    theme: {
      fontOrigin: "googleFonts",
      cdnCaching: true,
      typography: {
        header: "Space Grotesk",
        body: "Source Sans 3",
        code: "JetBrains Mono",
      },
      colors: {
        lightMode: {
          light: "#fbfaf7",
          lightgray: "#e6e3dc",
          gray: "#a8a49a",
          darkgray: "#3f3d38",
          dark: "#1f1e1b",
          secondary: "#b4410c",
          tertiary: "#0f766e",
          highlight: "rgba(180, 65, 12, 0.08)",
          textHighlight: "#ffd76a88",
        },
        darkMode: {
          light: "#141413",
          lightgray: "#33312d",
          gray: "#6f6b63",
          darkgray: "#d8d4cb",
          dark: "#f2efe8",
          secondary: "#f59a5b",
          tertiary: "#3fc7b4",
          highlight: "rgba(245, 154, 91, 0.10)",
          textHighlight: "#b38b0088",
        },
      },
    },
  },
  plugins: {
    transformers: [
      Plugin.FrontMatter(),
      // the content folder is a copy, so git dates would be wrong: use the notes' own date / last_reviewed
      Plugin.CreatedModifiedDate({ priority: ["frontmatter", "filesystem"] }),
      Plugin.SyntaxHighlighting({
        theme: { light: "github-light", dark: "github-dark" },
        keepBackground: false,
      }),
      Plugin.ObsidianFlavoredMarkdown({ enableInHtmlEmbed: false }),
      Plugin.GitHubFlavoredMarkdown(),
      Plugin.TableOfContents(),
      Plugin.CrawlLinks({ markdownLinkResolution: "shortest" }),
      Plugin.Description(),
      Plugin.Latex({ renderEngine: "katex" }),
    ],
    filters: [Plugin.RemoveDrafts()],
    emitters: [
      Plugin.AliasRedirects(),
      Plugin.ComponentResources(),
      Plugin.ContentPage(),
      Plugin.FolderPage(),
      Plugin.TagPage(),
      Plugin.ContentIndex({ enableSiteMap: true, enableRSS: true }),
      Plugin.Assets(),
      Plugin.Static(),
      Plugin.Favicon(),
      Plugin.NotFoundPage(),
      Plugin.CustomOgImages(), // social cards built from each note's title and description
    ],
  },
}

export default config
