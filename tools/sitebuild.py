#!/usr/bin/env python3
"""Build the public site and the agent files from the vault.

  python3 tools/sitebuild.py prep --quartz DIR            copy the vault into a Quartz checkout (DIR/content), compile
                                                     Dataview queries to static Markdown, add the per-note metadata
                                                     strip and the home page, and install site/quartz.*.ts
  python3 tools/sitebuild.py check --public DIR           every internal link on every built page resolves
  python3 tools/sitebuild.py export --out DIR [--base-url URL]
                                                     write llms.txt, llms-full.txt, llms-<chapter>.txt, graph.json,
                                                     graph.schema.json and raw/<path>.md into DIR (Quartz's public/)

The vault itself is never modified: everything happens on copies.
"""
from __future__ import annotations

import argparse, collections, datetime, json, os, re, shutil, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import vault as V
import yaml

BASE_URL = 'https://markus-learning.github.io/ROBOTICS-for-PEOPLE'
REPO_URL = 'https://github.com/MARKUS-LEARNING/ROBOTICS-for-PEOPLE'
SITE_TITLE = 'Robotics for People'
TAGLINE = 'A free, open, living textbook for robotics, the field without edges. Written for people and readable by AI agents.'


# ------------------------------------------------------------------------------------------------------------ helpers
def slug(path: str) -> str:
    """Quartz's slug for a vault path (quartz/util/path.ts sluggify), without the .md extension."""
    p = re.sub(r'\.md$', '', V.nfc(path))
    segs = [re.sub(r'\s', '-', s).replace('&', '-and-').replace('%', '-percent').replace('?', '').replace('#', '') for s in p.split('/')]
    return '/'.join(segs)


def url(path: str, base: str = BASE_URL) -> str:
    return f'{base}/{urllib.parse.quote(slug(path), safe="/-_.~()!*,")}'


def raw_url(path: str, base: str = BASE_URL) -> str:
    return f'{base}/raw/{urllib.parse.quote(V.nfc(path), safe="/-_.~()!*,")}'


def split_frontmatter(raw: str):
    m = V.FRONTMATTER.match(raw)
    return (m.group(0), raw[m.end():]) if m else ('', raw)


# ---------------------------------------------------------------------------------------------- Dataview, compiled
class Query:
    """The subset of Dataview this vault uses, evaluated against the vault so the site can show static results.
    Anything outside the subset (FLATTEN, GROUP BY, lambdas) returns None and becomes a 'live in Obsidian' note."""
    KW = re.compile(r'\b(FROM|WHERE|SORT|LIMIT|GROUP BY|FLATTEN)\b', re.I)

    def __init__(self, book: V.Book):
        self.b = book; self.v = book.v
        self.notes = [n for n in self.v.notes if n.frontmatter is not None]
        self.links_to = {}
        for n in self.notes:
            s = set()
            for l in n.links:
                r = self.v.resolve(l, n)
                s.add(r.path if isinstance(r, V.Note) else V.link_key(l.target))
            self.links_to[n.path] = s
        self.inbound = self.v.backlinks()

    def target(self, wikilink: str):
        lk = V.wikilink_target(wikilink)
        if not lk:
            return None
        r = self.v.resolve(lk, None)
        return r.path if isinstance(r, V.Note) else V.link_key(lk.target)

    @staticmethod
    def split_top(expr: str, word: str):
        parts, depth, quote, cur, i = [], 0, None, '', 0
        pat = re.compile(r'\s+' + word + r'\s+', re.I)
        while i < len(expr):
            ch = expr[i]
            if quote:
                cur += ch; quote = None if ch == quote else quote; i += 1; continue
            if ch in '"\'':
                quote = ch
            elif ch in '([':
                depth += 1
            elif ch in ')]':
                depth -= 1
            if depth == 0:
                m = pat.match(expr, i)
                if m:
                    parts.append(cur); cur = ''; i = m.end(); continue
            cur += ch; i += 1
        parts.append(cur)
        return [p.strip() for p in parts]

    def field(self, n: V.Note, name: str):
        fm = n.frontmatter or {}
        name = name.strip()
        if name in ('file.link', 'file.name'):
            return f'[[{n.name}]]' if name == 'file.link' else n.name
        if name == 'file.folder':
            return os.path.dirname(n.path)
        if name == 'file.tags':
            return ['#' + t for t in n.tags]
        if name == 'tags':
            return n.tags
        return fm.get(name)

    def source(self, src: str):
        src = src.strip()
        if not src:
            return list(self.notes)
        out = {}
        for atom in self.split_top(src, 'OR'):
            atom = atom.strip()
            if atom.startswith('#'):
                t = atom[1:].lower()
                for n in self.notes:
                    if any(x.lower() == t or x.lower().startswith(t + '/') for x in n.tags):
                        out[n.path] = n
            elif re.fullmatch(r'"[^"]*"', atom):
                folder = atom[1:-1].strip('/')
                for n in self.notes:
                    if not folder or n.path.startswith(folder + '/'):
                        out[n.path] = n
            elif atom.startswith('[['):
                t = self.target(atom)
                for n in self.notes:
                    if t in self.links_to[n.path]:
                        out[n.path] = n
            else:
                return None
        return list(out.values())

    def cond(self, n: V.Note, expr: str):
        expr = expr.strip()
        ors = self.split_top(expr, 'OR')
        if len(ors) > 1:
            rs = [self.cond(n, e) for e in ors]
            return None if None in rs else any(rs)
        ands = self.split_top(expr, 'AND')
        if len(ands) > 1:
            rs = [self.cond(n, e) for e in ands]
            return None if None in rs else all(rs)
        m = re.fullmatch(r'contains\(\s*file\.outlinks\s*,\s*(\[\[[^\]]+\]\])\s*\)', expr)
        if m:
            return self.target(m.group(1)) in self.links_to[n.path]
        m = re.fullmatch(r'contains\(\s*((?:file\.)?tags)\s*,\s*"([^"]*)"\s*\)', expr)
        if m:
            vals = self.field(n, m.group(1)) or []
            return any(m.group(2).lower() in str(x).lower() for x in vals)
        m = re.fullmatch(r'length\(\s*file\.inlinks\s*\)\s*=\s*0', expr)
        if m:
            return not self.inbound.get(n.path)
        m = re.fullmatch(r'([\w.]+)\s*(!=|=)\s*"([^"]*)"', expr)
        if m:
            val = self.field(n, m.group(1))
            eq = str(val) == m.group(3) if not isinstance(val, list) else m.group(3) in [str(x) for x in val]
            return eq if m.group(2) == '=' else not eq
        if re.fullmatch(r'[\w.]+', expr):
            return bool(self.field(n, expr))
        return None

    def run(self, q: str):
        q = ' '.join(l.strip() for l in q.strip().split('\n') if l.strip() and not l.strip().startswith('//'))
        parts = self.KW.split(q)
        head, clauses = parts[0].strip(), {}
        for i in range(1, len(parts) - 1, 2):
            clauses[parts[i].upper()] = parts[i + 1].strip()
        if 'GROUP BY' in clauses or 'FLATTEN' in clauses:
            return None
        m = re.fullmatch(r'(LIST|TABLE)(\s+WITHOUT\s+ID)?\s*(.*)', head, re.I | re.S)
        if not m:
            return None
        kind, without_id, cols = m.group(1).upper(), bool(m.group(2)), m.group(3).strip()
        notes = self.source(clauses.get('FROM', ''))
        if notes is None:
            return None
        if 'WHERE' in clauses:
            keep = []
            for n in notes:
                r = self.cond(n, clauses['WHERE'])
                if r is None:
                    return None
                if r:
                    keep.append(n)
            notes = keep
        for key in reversed([s.strip() for s in clauses.get('SORT', 'file.name ASC').split(',')]):
            km = re.fullmatch(r'([\w.]+)(?:\s+(ASC|DESC))?', key, re.I)
            if not km:
                return None
            notes.sort(key=lambda n: str(self.field(n, km.group(1)) or ''), reverse=(km.group(2) or '').upper() == 'DESC')
        if 'LIMIT' in clauses:
            notes = notes[:int(clauses['LIMIT'])]
        if not notes:
            return '*No notes match this view yet.*'
        if kind == 'LIST':
            if cols:
                if not re.fullmatch(r'[\w.]+', cols):
                    return None
                return '\n'.join(f'- {self.b.link(n)}' + (f': {self.cell(self.field(n, cols))}' if self.field(n, cols) else '') for n in notes)
            return '\n'.join(f'- {self.b.link(n)}' for n in notes)
        heads, exprs = [], []
        for c in re.findall(r'(?:[^,"]|"[^"]*")+', cols) if cols else []:
            c = c.strip()
            cm = re.fullmatch(r'([\w.]+)(?:\s+AS\s+"([^"]+)")?', c, re.I)
            if not cm:
                return None
            exprs.append(cm.group(1)); heads.append(cm.group(2) or cm.group(1).replace('file.', '').replace('_', ' ').capitalize())
        cols_h = ([] if without_id else ['Note']) + heads
        rows = ['| ' + ' | '.join(cols_h) + ' |', '|' + '---|' * len(cols_h)]
        for n in notes:
            cells = ([] if without_id else [self.b.link(n, table=True)]) + [self.cell(self.field(n, e), table=True) for e in exprs]
            rows.append('| ' + ' | '.join(cells) + ' |')
        return '\n'.join(rows)

    @staticmethod
    def cell(val, table=False):
        if val is None:
            return ''
        if isinstance(val, list):
            val = ', '.join(str(x) for x in val)
        s = str(val).replace('\n', ' ')
        return s.replace('|', '\\|') if table else s


def compile_dataview(body: str, q: Query, stats: collections.Counter) -> str:
    def repl(m):
        res = q.run(m.group(2))
        if res is None:
            stats['live_only'] += 1
            return ('> [!note] Live view\n> This list is a live [Dataview](https://blacksmithgu.github.io/obsidian-dataview/) query. '
                    'Open the vault in Obsidian with Dataview enabled to see it.\n')
        stats['compiled'] += 1
        return res + '\n'
    return V.DV_BLOCK.sub(repl, body)


# ----------------------------------------------------------------------------------------------------------- prep
def unlink_missing(body: str, v: V.Vault, n: V.Note) -> tuple[str, int]:
    """On the site, a link to a note nobody has written yet would be a 404. Show its text instead; the Frontier page
    lists every one of them. Code and Dataview are left alone."""
    out, fence, count = [], None, 0
    def repl(m):
        nonlocal count
        lk = V.parse_link(m.group(2), bool(m.group(1)), 'body')
        if not lk.target or v.resolve(lk, n) is not None:
            return m.group(0)
        count += 1
        return lk.alias or (lk.target.split('/')[-1].replace('_', ' ') + (f' ({lk.heading})' if lk.heading else ''))
    for line in body.split('\n'):
        f = V.FENCE.match(line)
        if fence is None and f:
            fence = f.group(2)[0] * len(f.group(2)); out.append(line); continue
        if fence is not None:
            if f and f.group(2).startswith(fence) and not f.group(3).strip():
                fence = None
            out.append(line); continue
        pieces = re.split(r'(`+[^`]*`+)', line)
        out.append(''.join(p if p.startswith('`') else V.WIKILINK.sub(repl, p) for p in pieces))
    return '\n'.join(out), count


def meta_strip(n: V.Note, b: V.Book) -> str:
    fm = n.frontmatter
    sfs = b.sfs
    level = str(fm.get('level', '')).capitalize()
    status = {'keystone': 'Keystone entry', 'reviewed': 'Reviewed', 'draft': 'Draft', 'stub': 'Stub: help write it'}.get(fm.get('status'), '')
    subs = ' · '.join(f'[[Subfield_Atlas#{sfs[s]["title"]}|{sfs[s]["title"]}]]' for s in fm.get('subfield') or [] if s in sfs)
    lines = [f'> [!abstract] {level} · {status}' + (f' · {subs}' if subs else '')]
    pre = b.prereq.get(n.path) or []
    if pre:
        lines.append('> **Read first:** ' + ' · '.join(b.link(p) for p in pre))
    for t in fm.get('key_takeaways') or []:
        lines.append(f'> - {t}')
    return '\n'.join(lines) + '\n'


def replace_h1(body: str, block: str) -> str:
    """Quartz already prints the title above the article, so the note's own leading '# Title' is replaced by the
    metadata strip instead of being shown twice."""
    m = re.match(r'\s*# [^\n]+\n', body)
    if m:
        return '\n' + block + body[m.end():]
    return '\n' + block + '\n' + body


def home_page(b: V.Book) -> str:
    v = b.v
    miss = b.missing()
    lv = collections.Counter(n.frontmatter.get('level') for n in b.notes)
    st = collections.Counter(n.frontmatter.get('status') for n in b.notes)
    chapters = sorted({n.chapter for n in b.notes if n.chapter[:2].isdigit()})
    overview = {}
    for n in b.notes:
        if n.frontmatter.get('type') == 'overview' and n.chapter not in overview:
            overview[n.chapter] = n
    L = ['---', f'title: {SITE_TITLE}', f'description: {json.dumps(TAGLINE)}', '---', '',
         f'**{TAGLINE}**', '',
         f'{len(b.notes)} notes · {sum(n.word_count for n in b.notes):,} words · {len([k for k in b.sfs if b.by_sub.get(k)])} of {len(b.sfs)} subfields covered · '
         f'{st["keystone"]} keystone entries · {len(miss)} notes still to write', '',
         'Robotics is not one subject. It is where mathematics, mechanics, electronics, computing, biology, design and ethics meet, '
         'and it keeps reaching into places nobody planned for. This textbook maps that whole territory, from the first idea of what a robot is '
         'to the open problems at the edge of the field, and it grows every time someone adds a note.', '',
         '## Start here', '',
         '- **New to robotics?** Follow the [[Learning_Paths#Curious about robots|Curious about robots]] path.',
         '- **Studying it?** The [[Learning_Paths#Student: the core stack|core stack]] runs from maths to kinematics, sensing, estimation, planning and control.',
         '- **Building something?** The [[Learning_Paths#Builder: make a robot work|builder path]] covers actuators, electronics, ROS 2 and the labs.',
         '- **Into learning-based robotics?** [[Learning_Paths#Learning-based robotics|Machine learning to robot learning]].',
         '- **Researching?** Start at [[Learning_Paths#Researcher: the edge of the field|the edge of the field]].', '',
         '## The map', '',
         'The [[Subfield_Atlas|Subfield Atlas]] lays out every subfield, from the core stack to the domains robots work in and the frontier still being invented. '
         f'Notes are tagged by level: {lv["beginner"]} beginner, {lv["intermediate"]} intermediate, {lv["advanced"]} advanced and {lv["research"]} research.', '',
         '## Chapters', '']
    for ch in chapters:
        name = re.sub(r'^\d+_', '', ch).replace('_', ' ')
        cnt = sum(1 for n in b.notes if n.chapter == ch)
        ov = overview.get(ch)
        L.append(f'- [{ch[:2]} · {name}](<{slug(ch)}/>) ({cnt} notes)' + (f': start with {b.link(ov)}' if ov else ''))
    L += ['', '## Help write it', '',
          f'The [[Frontier]] lists what to write next: {len(miss)} topics other notes already link to, ranked by how many are waiting for them, '
          f'plus the subfields that are barely covered. Every note follows the [[Vault_Taxonomy_Guide|Vault Taxonomy Guide]]; open a pull request on '
          f'[GitHub]({REPO_URL}).', '',
          '## For AI agents', '',
          'This textbook is built to be read by agents as well as people:', '',
          f'- [`llms.txt`]({BASE_URL}/llms.txt): what is here and where to start, following the llms.txt convention',
          f'- [`llms-full.txt`]({BASE_URL}/llms-full.txt): every note as clean Markdown, with per-chapter files listed in `llms.txt`',
          f'- [`graph.json`]({BASE_URL}/graph.json): the knowledge graph (every note\'s type, level, subfields, status, prerequisites and links, plus the missing notes), '
          f'described by [`graph.schema.json`]({BASE_URL}/graph.schema.json)',
          f'- `{BASE_URL}/raw/<path>.md`: the source of any note', '',
          'Free to read, reuse and build on under the MIT License.']
    return '\n'.join(L) + '\n'


def cmd_prep(a):
    qz = os.path.abspath(a.quartz); content = os.path.join(qz, 'content')
    if os.path.isdir(content):
        shutil.rmtree(content)
    os.makedirs(content)
    for f in ('quartz.config.ts', 'quartz.layout.ts'):
        shutil.copy(os.path.join(V.ROOT, 'site', f), os.path.join(qz, f))
    v = V.Vault(); b = V.Book(v); q = Query(b); stats = collections.Counter()
    for n in v.notes:
        fm_text, body = split_frontmatter(n.raw)
        fm = dict(n.frontmatter or {})
        if fm.get('last_reviewed'):
            fm['modified'] = str(fm['last_reviewed'])
        body = compile_dataview(body, q, stats)
        body, c = unlink_missing(body, v, n); stats['unlinked'] += c
        if n.frontmatter and n.frontmatter.get('type') != 'guide' and n.path in b.prereq:
            body = replace_h1(body, meta_strip(n, b))
        else:
            body = re.sub(r'\A\s*# [^\n]+\n', '\n', body)
        out = os.path.join(content, n.path)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w', encoding='utf-8') as fh:
            fh.write('---\n' + yaml.safe_dump(json.loads(json.dumps(fm, default=str)), allow_unicode=True, sort_keys=False) + '---\n' + body)
        stats['notes'] += 1
    for att in v.attachments:
        if att.split('/')[0] in V.SKIP_DIRS or '/' not in att:        # tooling and repo files are not part of the textbook
            continue
        dst = os.path.join(content, att); os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy(os.path.join(V.ROOT, att), dst)
    with open(os.path.join(content, 'index.md'), 'w', encoding='utf-8') as fh:
        fh.write(home_page(b))
    print(f'prep: {stats["notes"]} notes -> {content}; Dataview: {stats["compiled"]} queries compiled, '
          f'{stats["live_only"]} left as live-only (GROUP BY / FLATTEN / unsupported); '
          f'{stats["unlinked"]} links to unwritten notes shown as text')


# --------------------------------------------------------------------------------------------------------- export
def plain_links(body: str, v: V.Vault, n: V.Note, base: str) -> str:
    """Wikilinks -> Markdown links to the site (or plain text when the note does not exist); Dataview removed."""
    body = V.DV_BLOCK.sub('', body)
    def repl(m):
        lk = V.parse_link(m.group(2), False, 'body')
        text = lk.alias or (lk.target.replace('_', ' ') if lk.target else (lk.heading or ''))
        r = v.resolve(lk, n)
        if isinstance(r, V.Note):
            return f'[{text}]({url(r.path, base)}' + (f'#{urllib.parse.quote(lk.heading.lower().replace(" ", "-"))}' if lk.heading else '') + ')'
        return text
    return V.WIKILINK.sub(repl, body)


def graph(b: V.Book, base: str) -> dict:
    v = b.v
    notes = [n for n in v.notes if n.frontmatter is not None]
    def rid(r):
        return r.name if isinstance(r, V.Note) else None
    nodes, edges = [], []
    for n in notes:
        fm = n.frontmatter
        rel = []
        for x in fm.get('related') or []:
            lk = V.wikilink_target(x); r = v.resolve(lk, n) if lk else None
            if isinstance(r, V.Note):
                rel.append(r.name)
        nodes.append({
            'id': n.name, 'title': V.Book.title(n), 'path': n.path, 'url': url(n.path, base), 'raw_url': raw_url(n.path, base),
            'description': fm.get('description'), 'type': fm.get('type'), 'level': fm.get('level'),
            'subfield': fm.get('subfield') or [], 'status': fm.get('status'), 'tags': n.tags,
            'prerequisites': [p.name for p in b.prereq.get(n.path, [])], 'related': rel,
            'key_takeaways': fm.get('key_takeaways') or [], 'date': str(fm.get('date')) if fm.get('date') else None,
            'last_reviewed': str(fm['last_reviewed']) if fm.get('last_reviewed') else None,
            'chapter': n.chapter, 'word_count': n.word_count, 'inbound_links': len(b.inbound.get(n.path, ())),
        })
        seen = set()
        for l in n.links:
            if l.source != 'body':
                continue
            r = v.resolve(l, n)
            if isinstance(r, V.Note) and r is not n and r.name not in seen:
                seen.add(r.name); edges.append({'source': n.name, 'target': r.name, 'kind': 'links_to'})
        for r in rel:
            edges.append({'source': n.name, 'target': r, 'kind': 'related'})
        for p in b.prereq.get(n.path, []):
            edges.append({'source': p.name, 'target': n.name, 'kind': 'prerequisite'})
    missing = []
    for t, c, src in sorted(b.missing().values(), key=lambda e: (-e[1], e[0].lower())):
        missing.append({'name': t, 'references': c, 'linked_from': sorted(s.name for s in src.values()),
                        'likely_subfield': b.likely_subfield(src.values())})
    subfields = [{'id': k, 'title': s['title'], 'group': s['group'], 'description': s['description'],
                  'notes': len(b.by_sub.get(k, []))} for k, s in b.sfs.items()]
    return {'version': 1, 'title': SITE_TITLE, 'description': TAGLINE, 'base_url': base, 'repository': REPO_URL,
            'license': 'MIT', 'generated': datetime.date.today().isoformat(),
            'counts': {'notes': len(nodes), 'edges': len(edges), 'missing': len(missing), 'subfields': len(subfields)},
            'subfields': subfields, 'nodes': nodes, 'edges': edges, 'missing': missing}


def note_text(n: V.Note, b: V.Book, base: str) -> str:
    fm = n.frontmatter
    _, body = split_frontmatter(n.raw)
    head = [f'# {V.Book.title(n)}', '', f'URL: {url(n.path, base)}',
            f'Type: {fm.get("type")} · Level: {fm.get("level")} · Status: {fm.get("status")} · Subfields: {", ".join(fm.get("subfield") or [])}']
    if b.prereq.get(n.path):
        head.append('Prerequisites: ' + ', '.join(V.Book.title(p) for p in b.prereq[n.path]))
    if fm.get('description'):
        head += ['', f'> {fm["description"]}']
    text = plain_links(body, b.v, n, base).strip()
    text = re.sub(r'\A# [^\n]*\n', '', text).strip()          # the title is already in the header
    return '\n'.join(head) + '\n\n' + text + '\n'


def cmd_export(a):
    out = os.path.abspath(a.out); base = a.base_url.rstrip('/')
    os.makedirs(out, exist_ok=True)
    v = V.Vault(); b = V.Book(v)
    notes = [n for n in v.notes if n.frontmatter is not None]
    order = {n.path: i for i, n in enumerate(b.order(b.notes))}
    by_ch = collections.defaultdict(list)
    for n in sorted(notes, key=lambda n: (n.chapter, order.get(n.path, -1), n.path)):
        by_ch[n.chapter].append(n)
    sep = '\n\n---\n\n'
    with open(os.path.join(out, 'llms-full.txt'), 'w', encoding='utf-8') as fh:
        fh.write(f'# {SITE_TITLE}: full text\n\n> {TAGLINE}\n\nSource: {REPO_URL} (MIT License). Every note, chapter by chapter, '
                 f'in reading order within each chapter.' + sep + sep.join(note_text(n, b, base) for ch in sorted(by_ch) for n in by_ch[ch]))
    chapter_files = []
    for ch in sorted(by_ch):
        if not ch[:2].isdigit():
            continue
        fn = f'llms-{ch[:2]}.txt'; chapter_files.append((fn, ch))
        with open(os.path.join(out, fn), 'w', encoding='utf-8') as fh:
            fh.write(f'# {SITE_TITLE}: {ch[3:].replace("_", " ")}\n\n> {len(by_ch[ch])} notes, in reading order.' + sep +
                     sep.join(note_text(n, b, base) for n in by_ch[ch]))
    g = graph(b, base)
    schema_src = os.path.join(HERE, 'graph.schema.json')
    with open(os.path.join(out, 'graph.json'), 'w', encoding='utf-8') as fh:
        json.dump(g, fh, ensure_ascii=False, indent=1)
    shutil.copy(schema_src, os.path.join(out, 'graph.schema.json'))
    try:
        import jsonschema
        with open(schema_src) as fh:
            jsonschema.validate(g, json.load(fh))
        valid = 'valid against graph.schema.json'
    except ImportError:
        valid = 'not validated (pip install jsonschema)'
    for n in notes:
        dst = os.path.join(out, 'raw', n.path); os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, 'w', encoding='utf-8') as fh:
            fh.write(n.raw)
    def item(n):
        return f'- [{V.Book.title(n)}]({url(n.path, base)}): {n.frontmatter.get("description") or ""}'.rstrip()
    guides = {n.name: n for n in v.notes if (n.frontmatter or {}).get('type') == 'guide'}
    keystones = [n for n in b.order(b.notes) if n.frontmatter.get('status') == 'keystone']
    overviews = [n for n in b.order(b.notes) if n.frontmatter.get('type') == 'overview']
    L = [f'# {SITE_TITLE}', '', f'> {TAGLINE}', '',
         f'{len(notes)} Markdown notes in {len(chapter_files)} chapters, from first principles to research frontiers. Every note has typed '
         'frontmatter: type, level (beginner, intermediate, advanced, research), 1-3 subfields, status (stub, draft, reviewed, keystone) '
         f'and prerequisites. Source and contributions: {REPO_URL} (MIT License).', '',
         '## Start here', '']
    for name in ('Learning_Paths', 'Subfield_Atlas', 'Frontier', 'Vault_Taxonomy_Guide', 'How_to_Use_This_Vault'):
        if name in guides:
            L.append(item(guides[name]))
    L += ['', '## Machine-readable', '',
          f'- [graph.json]({base}/graph.json): every note with its metadata, typed edges (links_to, related, prerequisite) and the missing notes; schema at {base}/graph.schema.json',
          f'- [llms-full.txt]({base}/llms-full.txt): all notes as Markdown, in reading order',
          f'- Raw source of any note: {base}/raw/<path>.md (paths are in graph.json)']
    for fn, ch in chapter_files:
        L.append(f'- [{fn}]({base}/{fn}): chapter {ch[:2]}, {ch[3:].replace("_", " ")} ({len(by_ch[ch])} notes)')
    L += ['', '## Keystone entries', ''] + [item(n) for n in keystones]
    L += ['', '## Chapter and section overviews', ''] + [item(n) for n in overviews]
    with open(os.path.join(out, 'llms.txt'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(L) + '\n')
    print(f'export: llms.txt, llms-full.txt, {len(chapter_files)} chapter files, graph.json ({g["counts"]["notes"]} nodes, '
          f'{g["counts"]["edges"]} edges, {g["counts"]["missing"]} missing; {valid}), raw/ ({len(notes)} notes) -> {out}')


def cmd_check(a):
    """Every internal link on every built page must lead to a file that exists (clean URL -> .html)."""
    pub = os.path.abspath(a.public); dead, total, pages = [], 0, 0
    for dp, _, fs in os.walk(pub):
        if os.path.relpath(dp, pub).split(os.sep)[0] == 'raw':
            continue
        for f in fs:
            if not f.endswith('.html'):
                continue
            pages += 1; p = os.path.join(dp, f)
            with open(p, encoding='utf-8') as fh:
                art = fh.read().split('<article', 1)[-1]
            for href in re.findall(r'<a href="(\.[^"#?]*)[^"]*" class="internal', art):
                total += 1
                tgt = os.path.normpath(os.path.join(dp, urllib.parse.unquote(href)))
                if not any(os.path.isfile(c) for c in (tgt, tgt + '.html', os.path.join(tgt, 'index.html'))):
                    dead.append(f'{os.path.relpath(p, pub)} -> {href}')
    for need in ('index.html', 'llms.txt', 'llms-full.txt', 'graph.json', 'graph.schema.json', 'sitemap.xml'):
        if not os.path.isfile(os.path.join(pub, need)):
            dead.append(f'missing {need}')
    print(f'check: {pages} pages, {total} internal links, {len(dead)} problems')
    for d in dead[:50]:
        print('  ' + d)
    sys.exit(1 if dead else 0)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="sitebuild.py", description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prep'); p.add_argument('--quartz', required=True); p.set_defaults(fn=cmd_prep)
    p = sub.add_parser('export'); p.add_argument('--out', required=True); p.add_argument('--base-url', default=BASE_URL)
    p.set_defaults(fn=cmd_export)
    p = sub.add_parser('check', help='every internal link in the built site resolves'); p.add_argument('--public', required=True)
    p.set_defaults(fn=cmd_check)
    a = ap.parse_args(argv); a.fn(a)


if __name__ == '__main__':
    main()
