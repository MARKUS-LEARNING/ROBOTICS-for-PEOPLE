#!/usr/bin/env python3
"""ROBOTICS-for-PEOPLE vault tooling: one portable view of the vault that matches what Obsidian sees.

It parses every note's YAML frontmatter, tags (frontmatter and inline) and wikilinks, and resolves links the way
Obsidian does (by file name, case-insensitively, with or without a folder path; links inside code are not links).
CI uses this module; locally the Obsidian CLI can cross-check it against Obsidian's own metadata cache.

Usage (from the vault root, Python 3.10+, PyYAML):
  python3 tools/vault.py stats [--tags] [--unresolved N]     counts that should match Obsidian
  python3 tools/vault.py lint [--verbose] [--enforce]        check notes against tools/schema.yml and tags.yml
  python3 tools/vault.py tags [--init-vocabulary]            tag-mapping report
  python3 tools/vault.py obsidian-check                      compare with Obsidian itself via the Obsidian CLI
"""
from __future__ import annotations

import argparse, collections, json, os, re, sys, unicodedata
from dataclasses import dataclass, field

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
TOOLS = os.path.join(ROOT, 'tools')
SKIP_DIRS = {'.git', '.obsidian', '.github', '.claude', 'tools', 'site', 'node_modules', '.venv', 'public'}
SKIP_FILES = {'README.md', 'CONTRIBUTING.md', 'CHANGELOG.md', 'CLAUDE.md', 'AGENTS.md'}   # repo docs, not vault notes

FRONTMATTER = re.compile(r'\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*(?:\r?\n|\Z)', re.S)
FENCE = re.compile(r'^([ \t]*)(`{3,}|~{3,})(.*)$')
INLINE_CODE = re.compile(r'(`+)(.+?)\1', re.S)
WIKILINK = re.compile(r'(!?)\[\[([^\[\]\n]+?)\]\]')
MDLINK = re.compile(r'(?<!!)\[[^\]\n]*\]\(<?([^)\s>]+)>?(?:\s+"[^"]*")?\)')
TAG_CHARS = r'[^\s#!"$%&\'()*+,.:;<=>?@\[\]^`{|}~\\]+'
# Obsidian inline tag: '#' at the start of a line or after whitespace, with at least one non-digit character
INLINE_TAG = re.compile(r'(?:(?<=^)|(?<=\s))#(' + TAG_CHARS + ')', re.M)
VALID_TAG = re.compile(r'^' + TAG_CHARS + r'$')


def nfc(s: str) -> str:
    return unicodedata.normalize('NFC', s)


def link_key(s: str) -> str:
    """How Obsidian matches a link target to a file: NFC, case-insensitive, no .md suffix."""
    s = nfc(s.strip())
    if s.lower().endswith('.md'):
        s = s[:-3]
    return s.lower()


@dataclass
class Link:
    target: str          # as written, without heading/block/alias
    heading: str | None
    alias: str | None
    embed: bool
    source: str          # 'body' or 'frontmatter'
    relative: bool = False   # a Markdown [text](path) link, resolved relative to the note's folder


@dataclass
class Note:
    path: str                          # vault-relative, NFC, forward slashes
    raw: str
    frontmatter: dict | None
    fm_text: str | None
    fm_error: str | None
    body: str
    links: list[Link] = field(default_factory=list)
    inline_tags: list[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return os.path.splitext(os.path.basename(self.path))[0]

    @property
    def chapter(self) -> str:
        return self.path.split('/')[0] if '/' in self.path else ''

    @property
    def fm_tags(self) -> list[str]:
        t = (self.frontmatter or {}).get('tags')
        if t is None:
            return []
        if isinstance(t, str):
            t = re.split(r'[,\s]+', t)
        return [str(x).strip().lstrip('#') for x in t if x is not None and str(x).strip()]

    @property
    def invalid_tags(self) -> list[str]:
        """Frontmatter tags Obsidian does not accept (spaces, punctuation such as '+', or digits only)."""
        return [t for t in self.fm_tags if not VALID_TAG.match(t) or t.replace('/', '').isdigit()]

    @property
    def tags(self) -> list[str]:
        """Tags as Obsidian indexes them: valid frontmatter tags plus inline tags, deduplicated case-insensitively."""
        seen, out = set(), []
        bad = set(self.invalid_tags)
        for t in [t for t in self.fm_tags if t not in bad] + self.inline_tags:
            if t.lower() not in seen:
                seen.add(t.lower()); out.append(t)
        return out

    @property
    def word_count(self) -> int:
        return len(self.body.split())


def strip_code(text: str) -> str:
    """Blank out fenced code blocks and inline code (Obsidian ignores links and tags there), keeping line structure."""
    out, fence = [], None
    for line in text.split('\n'):
        m = FENCE.match(line)
        if fence is None and m:
            fence = m.group(2)[0] * len(m.group(2)); out.append(''); continue
        if fence is not None:
            if m and m.group(2).startswith(fence) and not m.group(3).strip():
                fence = None
            out.append(''); continue
        out.append(line)
    text = '\n'.join(out)
    return INLINE_CODE.sub(lambda mm: ' ' * len(mm.group(0)), text)


def parse_link(inner: str, embed: bool, source: str) -> Link:
    alias = None
    inner = inner.replace('\\|', '|')        # escaped pipe, used for aliases inside Markdown tables
    if '|' in inner:
        inner, alias = inner.split('|', 1)
    heading = None
    if '#' in inner:
        inner, heading = inner.split('#', 1)
    return Link(target=inner.strip(), heading=heading, alias=alias, embed=embed, source=source)


def fm_links(value, out: list[Link]):
    """Wikilinks inside frontmatter values (Obsidian indexes these as links, e.g. related: ["[[X]]"])."""
    if isinstance(value, str):
        for m in WIKILINK.finditer(value):
            out.append(parse_link(m.group(2), bool(m.group(1)), 'frontmatter'))
    elif isinstance(value, list):
        for v in value:
            fm_links(v, out)
    elif isinstance(value, dict):
        for v in value.values():
            fm_links(v, out)


def load_note(path: str) -> Note:
    rel = nfc(os.path.relpath(path, ROOT).replace(os.sep, '/'))
    with open(path, encoding='utf-8') as fh:
        raw = fh.read()
    m = FRONTMATTER.match(raw)
    fm, fm_text, err, body = None, None, None, raw
    if m:
        fm_text = m.group(1); body = raw[m.end():]
        try:
            fm = yaml.safe_load(fm_text) or {}
            if not isinstance(fm, dict):
                err, fm = 'frontmatter is not a mapping', None
        except yaml.YAMLError as e:
            err = str(e).split('\n')[0]
    n = Note(path=rel, raw=raw, frontmatter=fm, fm_text=fm_text, fm_error=err, body=body)
    clean = strip_code(body)
    for lm in WIKILINK.finditer(clean):
        n.links.append(parse_link(lm.group(2), bool(lm.group(1)), 'body'))
    for lm in MDLINK.finditer(clean):
        tgt = lm.group(1)
        if '://' in tgt or tgt.startswith(('#', 'mailto:', 'tel:')):
            continue
        heading = None
        if '#' in tgt:
            tgt, heading = tgt.split('#', 1)
        n.links.append(Link(target=tgt.replace('%20', ' '), heading=heading, alias=None, embed=False, source='body', relative=True))
    if fm:
        fm_links(fm, n.links)
    for tm in INLINE_TAG.finditer(clean):
        t = tm.group(1).rstrip('/')
        if t and not t.replace('/', '').isdigit():      # Obsidian needs at least one non-digit character
            n.inline_tags.append(t)
    return n


class Vault:
    def __init__(self, root: str = ROOT):
        self.root = root
        self.notes: list[Note] = []
        self.attachments: list[str] = []
        for dp, dns, fns in os.walk(root):
            dns[:] = sorted(d for d in dns if d not in SKIP_DIRS and not d.startswith('.'))
            for fn in sorted(fns):
                p = os.path.join(dp, fn)
                if fn.endswith('.md'):
                    if dp == root and fn in SKIP_FILES:
                        continue
                    self.notes.append(load_note(p))
                elif not fn.startswith('.') and dp != root:      # root files (LICENSE) are repo files, not attachments
                    self.attachments.append(nfc(os.path.relpath(p, root).replace(os.sep, '/')))
        self.by_name: dict[str, list[Note]] = collections.defaultdict(list)
        self.by_path: dict[str, Note] = {}
        for n in self.notes:
            self.by_name[link_key(n.name)].append(n)
            self.by_path[link_key(n.path)] = n
        self.attach_keys = {a.lower() for a in self.attachments} | {os.path.basename(a).lower() for a in self.attachments}

    def resolve(self, link: Link, src: Note | None = None) -> Note | str | None:
        """The note a link points to, 'attachment' for a non-note file, or None if unresolved."""
        t = link.target
        if not t:
            return src           # [[#Heading]] links to the note itself
        if link.relative and src is not None:
            # Markdown link: a path relative to the note's folder (Obsidian also accepts a vault-root path)
            for base in (os.path.dirname(src.path), ''):
                p = nfc(os.path.normpath(os.path.join(base, t)).replace(os.sep, '/'))
                if p.startswith('..'):
                    continue
                if link_key(p) in self.by_path:
                    return self.by_path[link_key(p)]
                if p.lower() in self.attach_keys:
                    return 'attachment'
            if t.endswith('/') or not os.path.basename(t):
                return None      # a folder is not a note
        k = link_key(t)
        if '/' in k:
            n = self.by_path.get(k)
            if n:
                return n
            cands = [n for n in self.by_name.get(k.rsplit('/', 1)[1], []) if n.path.lower()[:-3].endswith(k)]
            if cands:
                return cands[0]
        else:
            cands = self.by_name.get(k)
            if cands:
                return cands[0]
        if nfc(t).lower() in self.attach_keys or os.path.basename(nfc(t)).lower() in self.attach_keys:
            return 'attachment'
        return None

    def unresolved(self) -> collections.Counter:
        c = collections.Counter()
        for n in self.notes:
            for l in n.links:
                if self.resolve(l, n) is None:
                    c[nfc(l.target)] += 1
        return c

    def backlinks(self) -> dict[str, set[str]]:
        bl: dict[str, set[str]] = collections.defaultdict(set)
        for n in self.notes:
            for l in n.links:
                r = self.resolve(l, n)
                if isinstance(r, Note) and r is not n:
                    bl[r.path].add(n.path)
        return bl

    def tag_counts(self) -> collections.Counter:
        """Notes per tag, grouped case-insensitively like Obsidian's tag pane (first-seen spelling kept)."""
        spelling, c = {}, collections.Counter()
        for n in self.notes:
            for t in n.tags:
                k = t.lower(); spelling.setdefault(k, t); c[k] += 1
        return collections.Counter({spelling[k]: v for k, v in c.items()})


# ---------------------------------------------------------------------------------------------------------- config files
def load_yaml(name: str) -> dict:
    with open(os.path.join(TOOLS, name), encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


class TagMap:
    """Maps any tag as written to its canonical form using tools/tags.yml."""

    def __init__(self):
        c = load_yaml('tags.yml')
        self.synonyms = {str(k).lower(): v for k, v in (c.get('synonyms') or {}).items()}
        self.drop = {str(t).lower() for t in c.get('drop') or []}
        self.to_type = {str(k).lower(): v for k, v in (c.get('to_type') or {}).items()}
        self.conditional = {k: re.compile(v, re.I) for k, v in (c.get('conditional') or {}).items()}
        self.vocabulary = set(c.get('vocabulary') or [])

    def canonical(self, tag: str) -> str | None:
        """Canonical tag, or None when the tag is dropped or becomes a `type:`."""
        k = tag.strip().lstrip('#').lower()
        if k in self.drop or k in self.to_type:
            return None
        if k in self.synonyms:
            return self.synonyms[k]
        k = re.sub(r'\s+', '-', k)              # 'human factors' -> 'human-factors' (spaces are not allowed in tags)
        return self.synonyms.get(k, k)


def init_vocabulary(v: Vault, tm: TagMap) -> list[str]:
    vocab = {t for n in v.notes for t0 in n.fm_tags + n.inline_tags if (t := tm.canonical(t0))}
    for sf in (load_yaml('subfields.yml').get('subfields') or {}).values():
        vocab.update(sf.get('tags') or [])
    vocab.update(tm.synonyms.values()); vocab.update(tm.conditional)
    return sorted(vocab)


def cmd_tags(a):
    v, tm = Vault(), TagMap()
    if a.init_vocabulary:
        vocab = init_vocabulary(v, tm)
        p = os.path.join(TOOLS, 'tags.yml')
        with open(p, encoding='utf-8') as fh:
            text = fh.read()
        block = 'vocabulary:\n' + ''.join(f'  - {t}\n' for t in vocab)
        text = re.sub(r'^vocabulary:.*\Z', block, text, flags=re.S | re.M)
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(text)
        print(f'wrote {len(vocab)} tags to tools/tags.yml vocabulary'); return
    before = v.tag_counts(); after = collections.Counter()
    for n in v.notes:
        for t in {tm.canonical(t) for t in n.tags} - {None}:
            after[t] += 1
    print(f'{len(before)} tags now -> {len(after)} after mapping (before conditional pruning)')


# ----------------------------------------------------------------------------------------------------------------- lint
DATE = re.compile(r'^\d{4}-\d{2}-\d{2}$')
FILENAME_OK = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_\-()+.,\'&]*$')


class Lint:
    def __init__(self):
        self.items: list[tuple[str, str, str, str]] = []   # (severity, rule, path, message)

    def add(self, sev, rule, path, msg):
        self.items.append((sev, rule, path, msg))


def wikilink_target(s) -> Link | None:
    m = WIKILINK.search(str(s)) if s is not None else None
    return parse_link(m.group(2), False, 'frontmatter') if m else None


def check_note(n: Note, v: Vault, schema: dict, tm: TagMap, subfields: set, L: Lint):
    if n.fm_text is None:
        L.add('error', 'frontmatter-missing', n.path, 'no YAML frontmatter'); return
    if n.fm_error:
        L.add('error', 'frontmatter-invalid', n.path, n.fm_error); return
    fm = n.frontmatter
    fields = schema['fields']
    for f in schema.get('removed', []):
        if f in fm:
            L.add('warning', 'field-removed', n.path, f'v1 field `{f}` should be removed')
    for f in fm:
        if f not in fields and f not in schema.get('removed', []):
            L.add('warning', 'field-unknown', n.path, f'unknown field `{f}`')
    for f, spec in fields.items():
        val = fm.get(f)
        if val in (None, '', []):
            if spec.get('required'):
                L.add('error', 'field-required', n.path, f'missing `{f}`')
            continue
        kind = spec['kind']
        if kind == 'string' and not isinstance(val, str):
            L.add('error', 'field-kind', n.path, f'`{f}` must be text')
        elif kind == 'string' and spec.get('max_length') and len(val) > spec['max_length']:
            L.add('warning', 'field-length', n.path, f'`{f}` is {len(val)} characters (max {spec["max_length"]})')
        elif kind == 'enum' and str(val) not in spec['values']:
            L.add('error', 'field-value', n.path, f'`{f}: {val}` not one of {spec["values"]}')
        elif kind == 'date' and not DATE.match(str(val)):
            L.add('error', 'field-date', n.path, f'`{f}: {val}` is not YYYY-MM-DD')
        elif kind == 'list':
            if not isinstance(val, list):
                L.add('error', 'field-kind', n.path, f'`{f}` must be a list'); continue
            if spec.get('min') and len(val) < spec['min'] or spec.get('max') and len(val) > spec['max']:
                L.add('error', 'field-count', n.path, f'`{f}` has {len(val)} items (allowed {spec.get("min", 0)}-{spec.get("max", "any")})')
            for item in val:
                it = spec.get('item')
                if it == 'tag' and str(item) not in tm.vocabulary:
                    canon = tm.canonical(str(item))
                    hint = f' (use `{canon}`)' if canon and canon != str(item) and canon in tm.vocabulary else ' (dropped or not in tools/tags.yml)'
                    L.add('error', 'tag-vocabulary', n.path, f'tag `{item}`{hint}')
                elif it == 'subfield' and str(item) not in subfields:
                    L.add('error', 'subfield-unknown', n.path, f'subfield `{item}` not in tools/subfields.yml')
                elif it == 'wikilink':
                    lk = wikilink_target(item)
                    if lk is None:
                        L.add('error', 'field-wikilink', n.path, f'`{f}` item {item!r} is not a [[wikilink]]')
                    elif v.resolve(lk, n) is None:
                        L.add('warning' if f == 'related' else 'error', f'{f}-unresolved', n.path, f'`{f}` points to missing note [[{lk.target}]]')
    for t in n.invalid_tags:
        L.add('error', 'tag-invalid', n.path, f'tag `{t}` is not a valid Obsidian tag (no spaces or punctuation like +), so Obsidian ignores it')
    if not FILENAME_OK.match(n.name):
        L.add('warning', 'filename', n.path, 'file name has spaces or punctuation other than _ - ( ) (Vault_Taxonomy_Guide.md)')


def prerequisite_cycles(v: Vault) -> list[list[str]]:
    graph = {}
    for n in v.notes:
        pre = (n.frontmatter or {}).get('prerequisites') or []
        outs = []
        for p in pre if isinstance(pre, list) else []:
            lk = wikilink_target(p); r = v.resolve(lk, n) if lk else None
            if isinstance(r, Note):
                outs.append(r.path)
        graph[n.path] = outs
    cycles, color = [], {}
    def dfs(u, stack):
        color[u] = 1; stack.append(u)
        for w in graph.get(u, []):
            if color.get(w) == 1:
                cycles.append(stack[stack.index(w):] + [w])
            elif not color.get(w):
                dfs(w, stack)
        stack.pop(); color[u] = 2
    sys.setrecursionlimit(10000)
    for u in graph:
        if not color.get(u):
            dfs(u, [])
    return cycles


def cmd_lint(a):
    v, tm = Vault(), TagMap()
    schema = load_yaml('schema.yml')
    sf_conf = load_yaml('subfields.yml').get('subfields') or {}
    L = Lint()
    if not tm.vocabulary:
        L.add('error', 'config', 'tools/tags.yml', 'vocabulary is empty (run `vault.py tags --init-vocabulary`)')
    for key, sf in sf_conf.items():
        for t in sf.get('tags') or []:
            if tm.vocabulary and t not in tm.vocabulary:
                L.add('error', 'config', 'tools/subfields.yml', f'subfield {key}: tag `{t}` not in the vocabulary')
    for n in v.notes:
        check_note(n, v, schema, tm, set(sf_conf), L)
    for k, ns in v.by_name.items():
        if len(ns) > 1:
            for n in ns:
                L.add('error', 'duplicate-name', n.path, f'another note has the same name: {[m.path for m in ns if m is not n]}')
    for cyc in prerequisite_cycles(v):
        L.add('error', 'prerequisite-cycle', cyc[0], ' -> '.join(os.path.basename(p)[:-3] for p in cyc))
    # broken-link ratchet: the number of unresolved links may only go down
    base_p = os.path.join(TOOLS, 'baseline.json')
    base = {}
    if os.path.exists(base_p):
        with open(base_p) as fh:
            base = json.load(fh)
    unres = sum(v.unresolved().values())
    if base.get('unresolved_links') is not None and unres > base['unresolved_links']:
        L.add('error', 'unresolved-ratchet', '-', f'{unres} unresolved links, up from the baseline {base["unresolved_links"]}: '
              'create the missing notes or remove the new links (tools/baseline.json)')
    if a.update_baseline:
        with open(base_p, 'w') as fh:
            json.dump({'unresolved_links': unres}, fh, indent=1)
        print(f'baseline: {unres} unresolved links')

    errors = [i for i in L.items if i[0] == 'error']; warnings = [i for i in L.items if i[0] == 'warning']
    by_rule = collections.Counter((s, r) for s, r, _, _ in L.items)
    notes_with_errors = len({p for s, _, p, _ in errors})
    print(f'lint: {len(v.notes)} notes, {len(errors)} errors in {notes_with_errors} notes, {len(warnings)} warnings, '
          f'{unres} unresolved links (baseline {base.get("unresolved_links", "none")})')
    for (s, r), c in sorted(by_rule.items(), key=lambda x: (x[0][0] != 'error', -x[1])):
        print(f'  {s:<7} {r:<22} {c:5d}')
    shown = L.items if a.verbose else [i for i in L.items if i[1] in ('config', 'duplicate-name', 'prerequisite-cycle', 'unresolved-ratchet', 'frontmatter-invalid', 'frontmatter-missing')]
    for s, r, p, m in shown[: a.limit]:
        if a.github and p not in ('-',):
            print(f'::{s} file={p},title={r}::{m}')
        else:
            print(f'  [{s}] {r}: {p}: {m}')
    if a.enforce and errors:
        sys.exit(1)


# -------------------------------------------------------------------------------------------------------- obsidian-check
def cmd_obsidian_check(a):
    """Compare this module's view of the vault with Obsidian's own metadata cache, through the Obsidian CLI
    (Obsidian 1.12+, desktop app running with this folder open as a vault, CLI enabled in Settings > General)."""
    import shutil, subprocess
    exe = a.obsidian or shutil.which('obsidian') or ('/usr/local/bin/obsidian' if os.path.exists('/usr/local/bin/obsidian') else None)
    if not exe:
        raise SystemExit('obsidian CLI not found: enable Settings > General > Command line interface in Obsidian 1.12+')
    def cli(*args):
        r = subprocess.run([exe, *args] + ([f'vault={a.vault}'] if a.vault else []), capture_output=True, text=True, timeout=120)
        if r.returncode:
            raise SystemExit(f'obsidian {" ".join(args)} failed: {r.stderr.strip() or r.stdout.strip()}')
        return r.stdout
    info = dict(l.split('\t', 1) for l in cli('vault').strip().splitlines() if '\t' in l)
    if os.path.realpath(info.get('path', '')) != os.path.realpath(ROOT):
        raise SystemExit(f'Obsidian\'s vault is {info.get("path")}, not {ROOT}; pass vault=<name> or open this folder in Obsidian')
    v = Vault()
    for f in SKIP_FILES:                       # Obsidian indexes the repo docs too
        if os.path.exists(os.path.join(ROOT, f)):
            v.notes.append(load_note(os.path.join(ROOT, f)))
    mine = collections.Counter()
    for n in v.notes:
        for l in n.links:
            if v.resolve(l, n) is None:
                mine[nfc(l.target).lower()] += 1
    obs = collections.Counter()
    for d in json.loads(cli('unresolved', 'counts', 'format=json')):
        obs[nfc(d['link']).lower()] += int(d['count'])
    my_tags = {t.lower() for n in v.notes for t in n.tags}
    obs_tags = {d['tag'].lstrip('#').lower() for d in json.loads(cli('tags', 'counts', 'format=json'))}
    problems = []
    for k in sorted(set(mine) | set(obs)):
        if mine[k] != obs[k]:
            problems.append(f'unresolved [[{k}]]: vault.py {mine[k]}, Obsidian {obs[k]}')
    for t in sorted(my_tags ^ obs_tags):
        problems.append(f'tag #{t}: only in {"vault.py" if t in my_tags else "Obsidian"}')
    print(f'Obsidian: {sum(obs.values())} unresolved links to {len(obs)} targets, {len(obs_tags)} tags')
    print(f'vault.py: {sum(mine.values())} unresolved links to {len(mine)} targets, {len(my_tags)} tags')
    for p in problems[:50]:
        print('  ' + p)
    print('MATCH' if not problems else f'{len(problems)} differences')
    sys.exit(1 if problems else 0)


# ---------------------------------------------------------------------------------------------------------------- stats
def cmd_stats(a):
    v = Vault()
    links = [(n, l) for n in v.notes for l in n.links]
    body_links = [x for x in links if x[1].source == 'body']
    unres = v.unresolved()
    tags = v.tag_counts()
    dupes = {k: [n.path for n in ns] for k, ns in v.by_name.items() if len(ns) > 1}
    bl = v.backlinks()
    s = dict(notes=len(v.notes), attachments=len(v.attachments), words=sum(n.word_count for n in v.notes),
             no_frontmatter=sum(1 for n in v.notes if n.fm_text is None),
             frontmatter_errors=sum(1 for n in v.notes if n.fm_error),
             links=len(links), body_links=len(body_links), frontmatter_links=len(links) - len(body_links),
             resolved_links=len(links) - sum(unres.values()),
             unresolved_links=sum(unres.values()), unresolved_targets=len(unres),
             tags=len(tags), tags_used_once=sum(1 for c in tags.values() if c == 1),
             orphans_no_backlinks=sum(1 for n in v.notes if n.path not in bl),
             duplicate_names=len(dupes),
             dataview_blocks=sum(len(re.findall(r'^[ \t]*```dataview', n.body, re.M)) for n in v.notes))
    if a.json:
        print(json.dumps(s, indent=1)); return
    w = max(len(k) for k in s)
    for k, val in s.items():
        print(f'{k:<{w}}  {val:,}' if isinstance(val, int) else f'{k:<{w}}  {val}')
    for n in v.notes:
        if n.fm_error:
            print(f'  frontmatter error: {n.path}: {n.fm_error}')
    for k, ps in dupes.items():
        print(f'  duplicate name {k}: {ps}')
    if a.unresolved:
        print(f'\nTop {a.unresolved} unresolved targets (refs):')
        for t, c in unres.most_common(a.unresolved):
            print(f'  {c:4d}  {t}')
    if a.tags:
        print('\nTags (notes):')
        for t, c in sorted(tags.items(), key=lambda x: (-x[1], x[0].lower())):
            print(f'  {c:4d}  {t}')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='vault.py', description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('stats', help='counts that should match Obsidian')
    p.add_argument('--json', action='store_true'); p.add_argument('--tags', action='store_true')
    p.add_argument('--unresolved', type=int, default=0, metavar='N')
    p.set_defaults(fn=cmd_stats)
    p = sub.add_parser('tags', help='tag mapping report; --init-vocabulary writes tools/tags.yml vocabulary')
    p.add_argument('--init-vocabulary', action='store_true')
    p.set_defaults(fn=cmd_tags)
    p = sub.add_parser('lint', help='check notes against tools/schema.yml, tags.yml, subfields.yml')
    p.add_argument('--enforce', action='store_true', help='exit 1 on any error (CI once the migration has landed)')
    p.add_argument('--verbose', action='store_true', help='list every finding, not just vault-level ones')
    p.add_argument('--github', action='store_true', help='print findings as GitHub Actions annotations')
    p.add_argument('--limit', type=int, default=200)
    p.add_argument('--update-baseline', action='store_true', help='record the current unresolved-link count')
    p.set_defaults(fn=cmd_lint)
    p = sub.add_parser('obsidian-check', help="compare with Obsidian's own index via the Obsidian CLI (local only)")
    p.add_argument('--obsidian', help='path to the obsidian CLI'); p.add_argument('--vault', help='Obsidian vault name')
    p.set_defaults(fn=cmd_obsidian_check)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == '__main__':
    main()
