// Robotics for People · Explore
// The whole textbook as a graph: notes are points, links are threads. Reads ../graph.json (tools/sitebuild.py export).
;(function () {
  "use strict"

  const $ = (id) => document.getElementById(id)
  const dark = document.documentElement.getAttribute("data-theme") === "dark"
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches
  const small = window.matchMedia("(max-width: 720px)").matches

  const THEME = dark
    ? { bg: "#121212", text: "#e8e8e8", link: "rgba(255,255,255,0.10)", related: "rgba(255,255,255,0.07)", pre: "rgba(138,180,248,0.75)", ghost: "#5a5a5a", dim: "rgba(120,120,120,0.18)", hi: "#ffffff" }
    : { bg: "#ffffff", text: "#111111", link: "rgba(0,0,0,0.10)", related: "rgba(0,0,0,0.06)", pre: "rgba(11,87,208,0.70)", ghost: "#c9c9c9", dim: "rgba(160,160,160,0.20)", hi: "#000000" }

  const GROUP_HUE = { core: 215, body: 24, domain: 150, society: 280, frontier: 335 }
  const GROUP_NAME = { core: "Core stack", body: "Bodies", domain: "Domains", society: "Society", frontier: "Frontier" }
  const LEVEL_COLOUR = { beginner: "#2e7d32", intermediate: "#1565c0", advanced: "#6a1b9a", research: "#c62828" }
  const STATUS_COLOUR = { keystone: dark ? "#ffffff" : "#000000", reviewed: "#1565c0", draft: "#f2a900", stub: "#9e9e9e" }
  const LEVELS = ["beginner", "intermediate", "advanced", "research"]
  const STATUSES = ["keystone", "reviewed", "draft", "stub"]

  const state = {
    view: small ? "2d" : "3d",
    colourBy: "group",
    levels: new Set(LEVELS),
    statuses: new Set(STATUSES),
    groups: new Set(Object.keys(GROUP_HUE)),
    subfield: "",
    type: "",
    edges: { prerequisite: true, links_to: true, related: false, wanted: false },
    selected: null,
    path: false,
  }

  let data, graph, byId, subfieldInfo, subfieldColour, chapterColour, prereqOf, nextOf, neighbours, highlight = new Set()

  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]))
  const cap = (s) => (s ? s[0].toUpperCase() + s.slice(1) : "")
  const nodeGroup = (n) => (n.ghost ? null : subfieldInfo[n.subfield[0]]?.group)

  // ------------------------------------------------------------------------------------------------ load
  fetch("../graph.json")
    .then((r) => {
      if (!r.ok) throw new Error(`graph.json: HTTP ${r.status}`)
      return r.json()
    })
    .then(init)
    .catch((e) => {
      $("loading").textContent = "Could not load the graph (" + e.message + ")."
      console.error(e)
    })

  function init(g) {
    data = g
    subfieldInfo = Object.fromEntries(g.subfields.map((s) => [s.id, s]))
    // subfield colours: each group's hue, one lightness step per subfield
    subfieldColour = {}
    for (const grp of Object.keys(GROUP_HUE)) {
      const subs = g.subfields.filter((s) => s.group === grp)
      subs.forEach((s, i) => {
        const l = subs.length > 1 ? 32 + (i / (subs.length - 1)) * 30 : 45
        subfieldColour[s.id] = `hsl(${GROUP_HUE[grp]}, 62%, ${l}%)`
      })
    }
    const chapters = [...new Set(g.nodes.map((n) => n.chapter))].sort()
    chapterColour = Object.fromEntries(chapters.map((c, i) => [c, `hsl(${Math.round((i * 360) / chapters.length)}, 58%, 45%)`]))

    const nodes = g.nodes
      .filter((n) => n.type !== "guide")
      .map((n) => ({ ...n, ghost: false, val: 1.5 + Math.sqrt(n.inbound_links || 0) * 1.6 }))
    byId = new Map(nodes.map((n) => [n.id, n]))
    const ghosts = g.missing.map((m) => ({
      id: "?" + m.name, title: m.name.replace(/_/g, " "), ghost: true, references: m.references,
      likely_subfield: m.likely_subfield, linked_from: m.linked_from, val: 0.8 + Math.sqrt(m.references),
    }))
    ghosts.forEach((n) => byId.set(n.id, n))

    const links = []
    for (const e of g.edges) if (byId.has(e.source) && byId.has(e.target)) links.push({ source: e.source, target: e.target, kind: e.kind })
    for (const m of ghosts) for (const src of m.linked_from) if (byId.has(src)) links.push({ source: src, target: m.id, kind: "wanted" })
    data.allNodes = [...nodes, ...ghosts]
    data.allLinks = links

    prereqOf = new Map(); nextOf = new Map(); neighbours = new Map()
    for (const n of nodes) {
      prereqOf.set(n.id, (n.prerequisites || []).filter((p) => byId.has(p)))
      for (const p of n.prerequisites || []) {
        if (!nextOf.has(p)) nextOf.set(p, [])
        nextOf.get(p).push(n.id)
      }
    }
    for (const l of links) {
      for (const [a, b] of [[l.source, l.target], [l.target, l.source]]) {
        if (!neighbours.has(a)) neighbours.set(a, new Set())
        neighbours.get(a).add(b)
      }
    }

    buildFilters(nodes)
    $("loading").hidden = true
    render()
    const focus = new URLSearchParams(location.search).get("focus")
    if (focus && byId.has(focus)) setTimeout(() => select(byId.get(focus), true), 1200)
  }

  // --------------------------------------------------------------------------------------------- filters
  function chip(container, value, label, colour, set) {
    const l = document.createElement("label")
    l.className = "chip"
    l.innerHTML = `<input type="checkbox" checked value="${esc(value)}">` + (colour ? `<span class="dot" style="background:${colour}"></span>` : "") + esc(label)
    l.querySelector("input").addEventListener("change", (e) => {
      e.target.checked ? set.add(value) : set.delete(value)
      refresh()
    })
    container.appendChild(l)
  }

  function buildFilters(nodes) {
    LEVELS.forEach((v) => chip($("f-level"), v, cap(v), LEVEL_COLOUR[v], state.levels))
    STATUSES.forEach((v) => chip($("f-status"), v, cap(v), STATUS_COLOUR[v], state.statuses))
    Object.keys(GROUP_HUE).forEach((v) => chip($("f-group"), v, GROUP_NAME[v], `hsl(${GROUP_HUE[v]}, 62%, 45%)`, state.groups))
    for (const s of data.subfields) {
      const o = document.createElement("option")
      o.value = s.id; o.textContent = `${s.title} (${s.notes})`
      $("f-subfield").appendChild(o)
    }
    for (const t of [...new Set(nodes.map((n) => n.type))].sort()) {
      const o = document.createElement("option")
      o.value = t; o.textContent = cap(t.replace("-", " "))
      $("f-type").appendChild(o)
    }
    const dl = $("note-list")
    for (const n of [...nodes].sort((a, b) => a.title.localeCompare(b.title))) {
      const o = document.createElement("option")
      o.value = n.title
      dl.appendChild(o)
    }
    $("f-subfield").addEventListener("change", (e) => { state.subfield = e.target.value; refresh() })
    $("f-type").addEventListener("change", (e) => { state.type = e.target.value; refresh() })
    $("colour-by").addEventListener("change", (e) => { state.colourBy = e.target.value; legend(); recolour() })
    for (const k of Object.keys(state.edges)) {
      $("e-" + k).checked = state.edges[k]
      $("e-" + k).addEventListener("change", (e) => { state.edges[k] = e.target.checked; refresh() })
    }
    $("toggle-filters").addEventListener("click", () => {
      const open = $("filters").hidden
      $("filters").hidden = !open
      $("toggle-filters").setAttribute("aria-expanded", String(open))
    })
    $("reset").addEventListener("click", () => location.assign(location.pathname))
    $("view-3d").addEventListener("click", () => switchView("3d"))
    $("view-2d").addEventListener("click", () => switchView("2d"))
    $("close").addEventListener("click", () => select(null))
    $("d-path").addEventListener("click", () => { state.path = !state.path; updateHighlight(); paintDetail() })
    document.addEventListener("keydown", (e) => { if (e.key === "Escape") select(null) })
    $("search").addEventListener("change", (e) => {
      const q = e.target.value.trim().toLowerCase()
      if (!q) return
      const hit = data.allNodes.find((n) => !n.ghost && n.title.toLowerCase() === q) ||
        data.allNodes.find((n) => !n.ghost && n.title.toLowerCase().includes(q))
      if (hit) select(hit, true)
    })
  }

  function visible(n) {
    if (n.ghost) return state.edges.wanted
    if (!state.levels.has(n.level) || !state.statuses.has(n.status)) return false
    if (!state.groups.has(nodeGroup(n))) return false
    if (state.subfield && !n.subfield.includes(state.subfield)) return false
    if (state.type && n.type !== state.type) return false
    return true
  }

  function currentData() {
    const nodes = data.allNodes.filter(visible)
    const ids = new Set(nodes.map((n) => n.id))
    const id = (x) => (typeof x === "object" ? x.id : x)
    const links = data.allLinks.filter((l) => state.edges[l.kind] && ids.has(id(l.source)) && ids.has(id(l.target)))
    // ghost notes only appear while something visible links to them
    const linked = new Set(links.flatMap((l) => [id(l.source), id(l.target)]))
    const shown = nodes.filter((n) => !n.ghost || linked.has(n.id))
    const notes = shown.filter((n) => !n.ghost).length
    const total = data.allNodes.filter((n) => !n.ghost).length
    const ghosts = shown.length - notes
    $("count").textContent = `Showing ${notes} of ${total} notes` + (ghosts ? ` and ${ghosts} still to write` : "")
    return { nodes: shown, links }
  }

  // -------------------------------------------------------------------------------------------- colours
  function baseColour(n) {
    if (n.ghost) return THEME.ghost
    switch (state.colourBy) {
      case "subfield": return subfieldColour[n.subfield[0]] || THEME.ghost
      case "level": return LEVEL_COLOUR[n.level]
      case "status": return STATUS_COLOUR[n.status]
      case "chapter": return chapterColour[n.chapter]
      default: return `hsl(${GROUP_HUE[nodeGroup(n)] ?? 0}, 62%, 45%)`
    }
  }
  // faded nodes also shrink, so a highlighted chain stands out instead of hiding behind grey spheres
  const nodeSize = (n) => (highlight.size && !highlight.has(n.id) ? Math.max(0.4, n.val * 0.25) : n.val)
  function nodeColour(n) {
    if (highlight.size && !highlight.has(n.id)) return THEME.dim
    return baseColour(n)
  }
  const lid = (x) => (typeof x === "object" ? x.id : x)
  function linkColour(l) {
    const on = highlight.size && highlight.has(lid(l.source)) && highlight.has(lid(l.target))
    if (highlight.size && !on) return "rgba(0,0,0,0)"
    if (l.kind === "prerequisite") return THEME.pre
    if (l.kind === "related" || l.kind === "wanted") return THEME.related
    return on ? THEME.pre.replace(/0\.\d+\)$/, "0.35)") : THEME.link
  }
  const linkWidth = (l) => (l.kind === "prerequisite" ? 1.2 : 0)

  function legend() {
    let items
    if (state.colourBy === "subfield") {
      items = data.subfields.filter((s) => s.notes).map((s) => [subfieldColour[s.id], s.title])
    } else if (state.colourBy === "level") items = LEVELS.map((v) => [LEVEL_COLOUR[v], cap(v)])
    else if (state.colourBy === "status") items = STATUSES.map((v) => [STATUS_COLOUR[v], cap(v)])
    else if (state.colourBy === "chapter") items = Object.entries(chapterColour).map(([c, col]) => [col, c.replace(/_/g, " ")])
    else items = Object.keys(GROUP_HUE).map((g) => [`hsl(${GROUP_HUE[g]}, 62%, 45%)`, GROUP_NAME[g]])
    if (state.edges.wanted) items.push([THEME.ghost, "Not written yet"])
    $("legend").innerHTML = `<h3>Colour: ${esc($("colour-by").selectedOptions[0].textContent)}</h3><ul>` +
      items.map(([c, t]) => `<li><span class="dot" style="background:${c}"></span>${esc(t)}</li>`).join("") +
      `</ul><h3 style="margin-top:8px">Size: how many notes link here · <span style="color:${THEME.pre}">→</span> prerequisite</h3>`
  }

  // ---------------------------------------------------------------------------------------------- render
  function tooltip(n) {
    if (n.ghost) return `<div class="graph-tooltip"><b>${esc(n.title)}</b><br>Not written yet · ${n.references} link${n.references === 1 ? "" : "s"} waiting</div>`
    return `<div class="graph-tooltip"><b>${esc(n.title)}</b><br>${cap(n.level)} · ${cap(n.status)} · ${esc(subfieldInfo[n.subfield[0]]?.title || "")}</div>`
  }

  function render() {
    const el = $("graph")
    if (graph) {
      graph._destructor && graph._destructor()
      el.innerHTML = ""
    }
    $("view-3d").setAttribute("aria-pressed", String(state.view === "3d"))
    $("view-2d").setAttribute("aria-pressed", String(state.view === "2d"))
    const d = currentData()
    if (state.view === "3d") {
      graph = ForceGraph3D({ controlType: "orbit" })(el)
        .backgroundColor(THEME.bg)
        .nodeLabel(tooltip)
        .nodeVal(nodeSize)
        .nodeColor(nodeColour)
        .nodeOpacity(0.92)
        .nodeResolution(12)
        .linkColor(linkColour)
        .linkWidth(linkWidth)
        .linkOpacity(1)
        .linkDirectionalArrowLength((l) => (l.kind === "prerequisite" ? 2.6 : 0))
        .linkDirectionalArrowRelPos(1)
        .linkDirectionalArrowColor(linkColour)
        .onNodeClick((n) => select(n, false))
        .onBackgroundClick(() => select(null))
        .cooldownTicks(220)
        .graphData(d)
      const controls = graph.controls()
      if (controls && !reducedMotion) {
        controls.autoRotate = true
        controls.autoRotateSpeed = 0.55
        el.addEventListener("pointerdown", () => { controls.autoRotate = false }, { once: true })
      }
    } else {
      graph = ForceGraph()(el)
        .backgroundColor(THEME.bg)
        .nodeLabel(tooltip)
        .nodeVal((n) => n.val)
        .nodeRelSize(3.2)
        .nodeCanvasObject(draw2d)
        .nodePointerAreaPaint((n, colour, ctx) => {
          ctx.fillStyle = colour
          ctx.beginPath(); ctx.arc(n.x, n.y, radius(n) + 2, 0, 2 * Math.PI); ctx.fill()
        })
        .linkColor(linkColour)
        .linkWidth(linkWidth)
        .linkDirectionalArrowLength((l) => (l.kind === "prerequisite" ? 4 : 0))
        .linkDirectionalArrowRelPos(1)
        .linkDirectionalArrowColor(linkColour)
        .onNodeClick((n) => select(n, false))
        .onBackgroundClick(() => select(null))
        .cooldownTicks(220)
        .graphData(d)
    }
    // frame the whole graph once the layout settles (unless the reader is already looking at a note)
    let framed = false
    graph.onEngineStop(() => {
      if (framed || state.selected) return
      framed = true
      graph.zoomToFit(600, state.view === "3d" ? 20 : 40)
    })
    legend()
  }

  const radius = (n) => Math.sqrt(n.val) * 3.2
  function draw2d(n, ctx, scale) {
    const r = radius(n)
    ctx.beginPath()
    ctx.arc(n.x, n.y, r, 0, 2 * Math.PI)
    if (n.ghost) {
      ctx.lineWidth = 1 / scale
      ctx.strokeStyle = highlight.size && !highlight.has(n.id) ? THEME.dim : THEME.ghost
      ctx.stroke()
    } else {
      ctx.fillStyle = nodeColour(n)
      ctx.fill()
      if (state.selected === n) { ctx.lineWidth = 2 / scale; ctx.strokeStyle = THEME.hi; ctx.stroke() }
    }
    const big = n.val > 9 || state.selected === n || (highlight.size && highlight.has(n.id))
    if (scale > 2.2 || (big && scale > 0.7)) {
      if (highlight.size && !highlight.has(n.id)) return
      const fs = Math.max(11 / scale, 2.2)
      ctx.font = `${n.ghost ? "italic " : ""}${fs}px "Source Sans 3", sans-serif`
      ctx.textAlign = "center"; ctx.textBaseline = "top"
      ctx.fillStyle = n.ghost ? THEME.ghost : THEME.text
      ctx.fillText(n.title, n.x, n.y + r + 1.5 / scale)
    }
  }

  let reframe
  function refresh() {
    if (!graph) return
    graph.graphData(currentData())
    legend()
    clearTimeout(reframe)
    reframe = setTimeout(() => { if (!state.selected) graph.zoomToFit(600, state.view === "3d" ? 20 : 40) }, 1200)
  }
  function recolour() {
    if (!graph) return
    graph.nodeColor && graph.nodeColor(nodeColour)
    if (state.view === "3d") graph.nodeVal(nodeSize)
    graph.linkColor(linkColour)
    graph.linkDirectionalArrowColor(linkColour)
    if (state.view === "2d") graph.nodeCanvasObject(draw2d)
  }
  function switchView(v) {
    if (state.view === v) return
    state.view = v
    render()
    if (state.selected) setTimeout(() => focusOn(state.selected), 900)
  }

  // ------------------------------------------------------------------------------------------- selection
  function ancestors(id, seen = new Set()) {
    for (const p of prereqOf.get(id) || []) if (!seen.has(p)) { seen.add(p); ancestors(p, seen) }
    return seen
  }
  function updateHighlight() {
    highlight = new Set()
    const n = state.selected
    if (n) {
      highlight.add(n.id)
      if (state.path && !n.ghost) ancestors(n.id).forEach((x) => highlight.add(x))
      else (neighbours.get(n.id) || new Set()).forEach((x) => highlight.add(x))
    }
    $("d-path").setAttribute("aria-pressed", String(state.path))
    recolour()
  }
  function focusOn(n) {
    if (!graph || n.x === undefined) return
    if (state.view === "3d") {
      const dist = 260, r = Math.hypot(n.x, n.y, n.z) || 1
      graph.cameraPosition({ x: n.x * (1 + dist / r), y: n.y * (1 + dist / r), z: n.z * (1 + dist / r) }, n, 1000)
    } else {
      graph.centerAt(n.x, n.y, 800); graph.zoom(3, 800)
    }
  }
  function select(n, fly) {
    state.selected = n
    if (!n) state.path = false
    if (n && !visible(n)) {   // make sure the chosen note is on screen
      if (!n.ghost) {
        state.levels.add(n.level); state.statuses.add(n.status); state.groups.add(nodeGroup(n)); state.subfield = ""; state.type = ""
        document.querySelectorAll(".chips input[type=checkbox]").forEach((i) => { if (!i.id) i.checked = true })
        $("f-subfield").value = ""; $("f-type").value = ""
      } else {
        state.edges.wanted = true; $("e-wanted").checked = true
      }
      refresh()
    }
    updateHighlight()
    paintDetail()
    if (n) {
      const url = new URL(location.href)
      url.searchParams.set("focus", n.ghost ? n.id.slice(1) : n.id)
      history.replaceState(null, "", url)
      if (fly) focusOn(n)
    } else {
      history.replaceState(null, "", location.pathname)
    }
  }

  function noteList(title, ids) {
    const items = ids.map((id) => byId.get(id)).filter(Boolean)
    if (!items.length) return ""
    return `<h4>${title}</h4><ul>` + items.map((m) => `<li><button type="button" data-id="${esc(m.id)}">${esc(m.title)}</button></li>`).join("") + "</ul>"
  }
  function paintDetail() {
    const n = state.selected, box = $("detail")
    if (!n) { box.hidden = true; return }
    box.hidden = false
    if (n.ghost) {
      $("d-kicker").textContent = "Not written yet"
      $("d-title").textContent = n.title
      $("d-meta").textContent = `${n.references} link${n.references === 1 ? "" : "s"} already point here` +
        (n.likely_subfield && subfieldInfo[n.likely_subfield] ? ` · likely ${subfieldInfo[n.likely_subfield].title}` : "")
      $("d-desc").textContent = "This topic is on the Frontier: the textbook's list of notes to write next. Pick it up!"
      $("d-pre").innerHTML = noteList("Waiting for it", n.linked_from)
      $("d-next").innerHTML = ""
      $("d-open").textContent = "See the Frontier"
      $("d-open").href = "../00_Introduction/Frontier"
      $("d-path").hidden = true
    } else {
      const subs = n.subfield.map((s) => subfieldInfo[s]?.title).filter(Boolean).join(", ")
      $("d-kicker").textContent = `${cap(n.type.replace("-", " "))} · ${n.chapter.replace(/^(\d+)_/, "$1 · ").replace(/_/g, " ")}`
      $("d-title").textContent = n.title
      $("d-meta").textContent = `${cap(n.level)} · ${cap(n.status)}` + (subs ? ` · ${subs}` : "") +
        (n.as_of ? ` · current as of ${n.as_of}` : "")
      $("d-desc").textContent = n.description || ""
      $("d-pre").innerHTML = noteList("Read first", prereqOf.get(n.id) || [])
      $("d-next").innerHTML = noteList("Next up", nextOf.get(n.id) || [])
      $("d-open").textContent = "Open note"
      $("d-open").href = "../" + n.slug.split("/").map(encodeURIComponent).join("/")
      $("d-path").hidden = !(prereqOf.get(n.id) || []).length
      $("d-path").textContent = state.path ? "Show neighbours" : "Path to here"
    }
    box.querySelectorAll("li button").forEach((b) => b.addEventListener("click", () => select(byId.get(b.dataset.id), true)))
  }
})()
