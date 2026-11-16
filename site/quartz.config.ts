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
        // White page, black text, one deep-blue accent so links stay recognisable
        lightMode: {
          light: "#ffffff",
          lightgray: "#e6e6e6",
          gray: "#8c8c8c",
          darkgray: "#111111",
          dark: "#000000",
          secondary: "#0b57d0",
          tertiary: "#0842a0",
          highlight: "rgba(11, 87, 208, 0.07)",
          textHighlight: "#fff3a3",
        },
        // Available through the toggle; the site always opens in light mode (site/patches/default-light.patch)
        darkMode: {
          light: "#121212",
          lightgray: "#2e2e2e",
          gray: "#8c8c8c",
          darkgray: "#e8e8e8",
          dark: "#ffffff",
          secondary: "#8ab4f8",
          tertiary: "#a8c7fa",
          highlight: "rgba(138, 180, 248, 0.12)",
          textHighlight: "#b3a20088",
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
