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

import argparse, collections, json, os, re, sys, unicodedata, urllib.parse
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
        n.links.append(Link(target=urllib.parse.unquote(tgt), heading=heading, alias=None, embed=False, source='body', relative=True))
    if fm:
        fm_links(fm, n.links)
    for tm in INLINE_TAG.finditer(clean):
        t = tm.group(1).rstrip('/')
        if t and not t.replace('/', '').isdigit():      # Obsidian needs at least one non-digit character
            n.inline_tags.append(t)
    return n


class Vault:
    def __init__(self, root: str = ROOT, repo_docs: bool = False):
        """repo_docs=True also indexes README.md, CONTRIBUTING.md... as notes, as Obsidian itself does."""
        self.root = root
        self.notes: list[Note] = []
        self.attachments: list[str] = []
        for dp, dns, fns in os.walk(root):
            # hidden folders are invisible to Obsidian; tooling folders hold no notes but their files can still be linked
            dns[:] = sorted(d for d in dns if not d.startswith('.') and d not in ('node_modules', 'public'))
            tooling = os.path.relpath(dp, root).split(os.sep)[0] in SKIP_DIRS
            for fn in sorted(fns):
                p = os.path.join(dp, fn); rel = nfc(os.path.relpath(p, root).replace(os.sep, '/'))
                if fn.endswith('.md') and not tooling:
                    if dp == root and fn in SKIP_FILES and not repo_docs:
                        self.attachments.append(rel); continue
                    self.notes.append(load_note(p))
                elif not fn.startswith('.') and '.' in fn:      # Obsidian does not index files without an extension
                    self.attachments.append(rel)
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
MONTH = re.compile(r'^\d{4}-(0[1-9]|1[0-2])$')
SOURCE = re.compile(r'\((19|20)\d{2}[a-z]?\).*https?://\S+', re.S)
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
            if spec.get('required') and fm.get('type') not in (spec.get('optional_for_types') or []):
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
        elif kind == 'month' and not MONTH.match(str(val)):
            L.add('error', 'field-month', n.path, f'`{f}: {val}` is not YYYY-MM')
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
                elif it == 'source' and not SOURCE.search(str(item)):
                    L.add('error', 'source-format', n.path, f'source {str(item)[:60]!r} needs a year in parentheses and an http(s) URL')
                elif it == 'subfield' and str(item) not in subfields:
                    L.add('error', 'subfield-unknown', n.path, f'subfield `{item}` not in tools/subfields.yml')
                elif it == 'wikilink':
                    lk = wikilink_target(item)
                    if lk is None:
                        L.add('error', 'field-wikilink', n.path, f'`{f}` item {item!r} is not a [[wikilink]]')
                    elif v.resolve(lk, n) is None:
                        L.add('warning' if f == 'related' else 'error', f'{f}-unresolved', n.path, f'`{f}` points to missing note [[{lk.target}]]')
    if fm.get('as_of') and len(fm.get('sources') or []) < 3:
        L.add('error', 'sources-required', n.path, f'a note with `as_of` needs at least 3 `sources` (has {len(fm.get("sources") or [])})')
    if unclosed_fence(n.body):
        L.add('error', 'fence-unclosed', n.path, 'the note ends inside a code block that is never closed (run `vault.py fix-fences --write`)')
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
    for k, val in tm.synonyms.items():
        if val.lower() in tm.synonyms and tm.synonyms[val.lower()] != val:
            L.add('error', 'config', 'tools/tags.yml', f'synonym `{k}` -> `{val}` points to another synonym (`{val}` -> `{tm.synonyms[val.lower()]}`)')
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


# --------------------------------------------------------------------------------------------------------------- migrate
FIELD_ORDER = ['title', 'description', 'aliases', 'tags', 'type', 'level', 'subfield', 'status', 'author', 'date',
               'last_reviewed', 'as_of', 'prerequisites', 'related', 'key_takeaways', 'sources']
V1_TYPE = {'robotic concept': 'concept', 'mechanical concept': 'concept', 'concept': 'concept', 'mechanical property': 'concept',
           'physical phenomenon': 'concept', 'engineering discipline': 'concept', 'interdisciplinary field': 'concept',
           'technology': 'concept', 'system': 'concept', 'control system': 'concept', 'robot type': 'robot-type',
           'sensor': 'component', 'mechanical component': 'component', 'material': 'component', 'actuator': 'component',
           'device': 'component', 'mechanism': 'component', 'mechanical system': 'component', 'chapter overview': 'overview'}
CHAPTER_LEVEL = {'00': 'beginner', '01': 'beginner', '02': 'intermediate', '03': 'intermediate', '04': 'intermediate',
                 '05': 'intermediate', '06': 'intermediate', '07': 'intermediate', '08': 'beginner', '09': 'advanced',
                 '10': 'research', '11': 'intermediate', '12': 'beginner', '13': 'beginner'}
LEVEL_RANK = {'beginner': 0, 'intermediate': 1, 'advanced': 2, 'research': 3}
# Chapters that have had a review pass (PRs #7-#11), with the date the pass merged
REVIEWED = {'00': '2026-04-23', '01': '2026-04-25', '02': '2026-04-25', '03': '2026-04-25', '04': '2026-04-27'}
KEYSTONES = {'Kinematics', 'Dynamics', 'Kinematics_and_Dynamics', 'Inverse_Kinematics', 'Forward_Kinematics', 'Singularities',
             'Jacobian_Matrix', 'Manipulability', 'DH_Parameters', 'Product_of_Exponentials',            # PR #10
             'LIDAR', 'Camera_Systems', 'Sensors', 'IMU_Sensors', 'Localization', 'Sensor_Fusion', 'Perception',
             'Mapping', 'Computer_Vision', 'Kalman_Filter'}                                                # PR #11
METHOD_WORDS = re.compile(r'(Control|Filter|Algorithm|Planning|Planner|Learning|Estimation|SLAM|Regression|Search|Calibration|'
                          r'Fusion|Solutions|Method|Tuning|Optimization|Avoidance|Detection|Recognition|Tracking|Servoing|'
                          r'Localization|Mapping|Linearization|Decomposition|Programming|Networks?|Transformers?)(_|$)')
COMPONENT_WORDS = re.compile(r'(Motors?|Sensors?|Camera|LIDAR|IMU|Encoders?|Gears?|Gearbox|Drive|Actuators?|Joints?|Links?|Gripper|'
                             r'Screw|Microcontroller|Arduino|Battery|Batteries|Bearing|Belt|Spring|Hand|Hands|End_Effector)(_|$)')


def chapter_key(n: Note) -> str:
    return n.chapter[:2] if n.chapter[:2].isdigit() else '99'


def folder_matches(path: str, prefixes) -> int:
    best = 0
    for p in prefixes or []:
        if path.startswith(p.rstrip('/') + '/'):
            best = max(best, len(p))
    return best


def propose(n: Note, v: Vault, tm: TagMap, sfs: dict, general: list, inbound: dict) -> dict:
    """Schema v2 frontmatter proposed from the v1 frontmatter, the folder, the tags and the link graph."""
    fm = dict(n.frontmatter or {})
    out = {}
    out['title'] = fm.get('title') or n.name.replace('_', ' ')
    out['description'] = fm.get('description')
    # tags: canonical, conditional bulk tags checked against folder + text, doc-type tags become `type`
    type_from_tag, tags = None, []
    for t in n.fm_tags + n.inline_tags:
        k = t.strip().lstrip('#').lower()
        if k in tm.to_type and not type_from_tag:
            type_from_tag = tm.to_type[k]
        c = tm.canonical(t)
        if c is None or c in tags:
            continue
        if c in tm.conditional and not tm.conditional[c].search(n.path + '\n' + n.body):
            continue
        tags.append(c)
    out['tags'] = tags
    # type
    v1 = str(fm.get('type') or '').strip().lower()
    ch = chapter_key(n); name = n.name
    parent = re.sub(r'^\d+_', '', n.path.split('/')[-2]) if n.path.count('/') else ''
    if v1 in V1_TYPE:
        typ = V1_TYPE[v1]
    elif type_from_tag:
        typ = type_from_tag
    elif ch == '00':
        typ = 'reference' if 'Resources' in name else 'guide'
    elif ch == '13' or 'Resources' in name or 'Reading' in name or 'Courses' in name:
        typ = 'reference'
    elif ch == '12':
        typ = 'lab' if re.search(r'Lab', name) else 'tutorial'
    elif ch == '08':
        typ = 'robot-type'
    elif (name == parent and sum(1 for m in v.notes if m.path.startswith(os.path.dirname(n.path) + '/')) >= 3) \
            or re.search(r'(Introduction|Overview)', name) or name.replace('_', '').lower() == n.chapter[3:].replace('_', '').lower():
        typ = 'overview'
    elif ch == '07' or name in ('Python', 'C++', 'Rust', 'Git', 'Docker', 'Gazebo', 'MuJoCo'):
        typ = 'tool'
    elif COMPONENT_WORDS.search(name):
        typ = 'component'
    elif METHOD_WORDS.search(name):
        typ = 'method'
    else:
        typ = 'concept'
    out['type'] = typ
    # level
    out['level'] = CHAPTER_LEVEL.get(ch, 'intermediate')
    # subfield: the most specific folder hint first, then the subfields the tags vote for
    scores = []
    in_general = folder_matches(n.path, general) and not any(folder_matches(n.path, s.get('folders')) for s in sfs.values())
    for key, s in sfs.items():
        f = 0 if in_general else folder_matches(n.path, s.get('folders'))
        votes = len(set(tags) & set(s.get('tags') or []))
        scores.append((f, votes, key))
    by_folder = sorted([x for x in scores if x[0]], key=lambda x: (-x[0], -x[1]))
    chosen = [by_folder[0][2]] if by_folder else []
    for f, votes, key in sorted(scores, key=lambda x: -x[1]):
        if len(chosen) >= 3 or votes < (2 if chosen else 1):
            break
        if key not in chosen:
            chosen.append(key)
    if typ == 'guide':
        chosen = []                       # vault guides describe the vault, not a part of robotics
    elif not chosen:
        chosen = [{'02': 'mathematics', '03': 'kinematics-dynamics', '05': 'robot-learning', '06': 'control', '07': 'software',
                   '10': 'research-practice', '11': 'software', '12': 'software', '13': 'research-practice'}.get(ch, 'foundations')]
    out['subfield'] = chosen
    # status
    if n.word_count < 150:
        st = 'stub'
    elif name in KEYSTONES:
        st = 'keystone'
    elif ch in REVIEWED:
        st = 'reviewed'
    else:
        st = 'draft'
    out['status'] = st
    out['author'] = 'Jordan_Smith'
    d = fm.get('date')
    out['date'] = d.isoformat() if hasattr(d, 'isoformat') else (str(d).split()[0] if d else None)
    if st in ('reviewed', 'keystone') and ch in REVIEWED:
        out['last_reviewed'] = REVIEWED[ch]
    out['related'] = fm.get('related') if isinstance(fm.get('related'), list) else None
    for k in ('aliases', 'prerequisites', 'key_takeaways', 'last_reviewed'):
        if fm.get(k) and k not in out:
            out[k] = fm[k]
    # values that are already valid schema v2 win, so re-running the migration never undoes a hand edit
    fields = load_yaml('schema.yml')['fields']
    for k in ('type', 'level', 'status'):
        if str(fm.get(k)) in fields[k]['values']:
            out[k] = fm[k]
    if isinstance(fm.get('subfield'), list) and fm['subfield'] and all(s in sfs for s in fm['subfield']):
        out['subfield'] = fm['subfield']
    if isinstance(fm.get('last_reviewed'), str) or hasattr(fm.get('last_reviewed'), 'isoformat'):
        lr = fm['last_reviewed']; out['last_reviewed'] = lr.isoformat() if hasattr(lr, 'isoformat') else lr
    return out


def propose_prerequisites(props: dict, v: Vault, inbound: dict) -> None:
    """Up to 3 prerequisites per note, chosen among the notes it links to. A prerequisite comes from an earlier chapter
    at the same or an easier level, or from the same chapter at an easier level, or is the same chapter's overview.
    Every chosen edge also goes strictly backwards in one global order, so the prerequisite graph has no cycles."""
    def order(n):
        p = props[n.path]
        return (chapter_key(n), LEVEL_RANK[p['level']], 0 if p['type'] == 'overview' else 1, -len(inbound.get(n.path, ())), n.path)
    for n in v.notes:
        if n.path not in props or (n.frontmatter or {}).get('prerequisites') or props[n.path]['type'] in ('guide', 'reference'):
            continue
        me = order(n); mp = props[n.path]
        cands = {}
        for l in n.links:
            r = v.resolve(l, n)
            if not (isinstance(r, Note) and r is not n and r.path in props) or order(r) >= me:
                continue
            rp = props[r.path]; rc, nc = chapter_key(r), chapter_key(n)
            if rp['type'] in ('guide', 'reference') or rc == '00':
                continue
            # it must be about the same part of robotics (or be maths, which everything builds on)
            if not (set(rp['subfield']) & set(mp['subfield']) or 'mathematics' in rp['subfield']):
                continue
            lr, ln = LEVEL_RANK[rp['level']], LEVEL_RANK[mp['level']]
            # a same-level overview counts only when it introduces this note's own section (or the whole chapter)
            home = re.sub(r'/0?0_Introduction[^/]*$', '', os.path.dirname(r.path))
            own_section = (os.path.dirname(n.path) + '/').startswith(home + '/')
            if (rc < nc and lr <= ln) or (rc == nc and lr < ln) or \
                    (rc == nc and lr == ln and rp['type'] == 'overview' and mp['type'] != 'overview' and own_section):
                cands[r.path] = r
        best = sorted(cands.values(), key=lambda r: (-len(inbound.get(r.path, ())), r.path))[:3]
        if best:
            props[n.path]['prerequisites'] = [f'[[{r.name}]]' for r in best]


PLAIN_OK = re.compile(r'^[A-Za-z0-9(][^:#\n]*$')


def yaml_scalar(s) -> str:
    s = str(s)
    if PLAIN_OK.match(s) and not s.endswith((' ', '-')) and s.lower() not in ('yes', 'no', 'true', 'false', 'null', 'on', 'off') \
            and not re.match(r'^[\d.\-+e]+$', s) and "'" not in s[:1] and ' - ' not in s[:3]:
        return s
    return json.dumps(s, ensure_ascii=False)      # a JSON string is a valid YAML double-quoted scalar


def emit_frontmatter(d: dict) -> str:
    lines = []
    keys = [k for k in FIELD_ORDER if k in d] + [k for k in d if k not in FIELD_ORDER]
    for k in keys:
        val = d[k]
        if val is None or val == []:
            continue
        if isinstance(val, list):
            lines.append(f'{k}:')
            for item in val:
                item = str(item)
                lines.append(f'  - "{item}"' if item.startswith('[[') else f'  - {yaml_scalar(item)}')
        else:
            lines.append(f'{k}: {val if k in ("date", "last_reviewed") else yaml_scalar(val)}')
    return '\n'.join(lines)


def cmd_migrate(a):
    v, tm = Vault(), TagMap()
    sconf = load_yaml('subfields.yml'); sfs = sconf.get('subfields') or {}
    overrides = {}
    if a.overrides:
        with open(a.overrides, encoding='utf-8') as fh:
            overrides = yaml.safe_load(fh) or {}
    inbound = v.backlinks()
    props = {n.path: propose(n, v, tm, sfs, sconf.get('general_folders') or [], inbound) for n in v.notes if n.fm_text is not None}
    # human/agent corrections, keyed by vault path or note name; applied before prerequisites are chosen
    by_key = {n.name: n.path for n in v.notes}
    resolved = {}
    for key, o in overrides.items():
        path = key if key in props else by_key.get(nfc(str(key)))
        if path not in props:
            raise SystemExit(f'override for unknown note {key!r}')
        resolved[path] = o
        props[path].update({k: val for k, val in o.items() if k != 'prerequisites'})
    propose_prerequisites(props, v, inbound)
    for path, o in resolved.items():
        if 'prerequisites' in o:
            props[path]['prerequisites'] = o['prerequisites'] or None
    chapters = set(a.chapters.split(',')) if a.chapters else None
    rows, changed = [], 0
    for n in v.notes:
        if n.path not in props or (chapters and chapter_key(n) not in chapters):
            continue
        p = props[n.path]
        rows.append((n.path, p['type'], p['level'], ','.join(p['subfield']), p['status'],
                     ' '.join(x.strip('[]"') for x in p.get('prerequisites') or []), ' '.join(p['tags'])))
        if a.write:
            text = '---\n' + emit_frontmatter(p) + '\n---\n' + n.body
            back = yaml.safe_load(FRONTMATTER.match(text).group(1))
            for k, val in p.items():          # round-trip guard: what we wrote is what we meant
                want = [str(x) for x in val] if isinstance(val, list) else (str(val) if val is not None else None)
                got = back.get(k); got = [str(x) for x in got] if isinstance(got, list) else (str(got) if got is not None else None)
                if want not in (None, []) and want != got:
                    raise SystemExit(f'round-trip mismatch in {n.path} field {k}: {want!r} != {got!r}')
            if text != n.raw:
                with open(os.path.join(ROOT, n.path), 'w', encoding='utf-8') as fh:
                    fh.write(text)
                changed += 1
    if a.report:
        with open(a.report, 'w', encoding='utf-8') as fh:
            fh.write('path\ttype\tlevel\tsubfield\tstatus\tprerequisites\ttags\n')
            fh.writelines('\t'.join(r) + '\n' for r in rows)
    print(f'{len(rows)} notes proposed' + (f', {changed} files rewritten' if a.write else ' (dry run; --write to apply)'))


DV_BLOCK = re.compile(r'(^[ \t]*```dataview[^\n]*\n)(.*?)(^[ \t]*```)', re.S | re.M)


def rewrite_dataview(query: str, tm: TagMap) -> str:
    """Map tag references in a Dataview query to the v2 vocabulary. A FROM chain that relied on a dropped catch-all tag
    (#robotics was on nearly every note) becomes FROM "" so the query keeps its meaning."""
    def from_chain(m):
        tags = re.findall(r'#([\w/-]+)', m.group(2))
        if any(tm.canonical(t) is None for t in tags):
            return m.group(1) + '""'
        out = m.group(2)
        for t in tags:
            out = re.sub(r'#' + re.escape(t) + r'\b', '#' + tm.canonical(t), out)
        return m.group(1) + out
    q = re.sub(r'(\bFROM\s+)(#[\w/-]+(?:\s+OR\s+#[\w/-]+)*)', from_chain, query, flags=re.I)
    def contains_tag(m):
        c = tm.canonical(m.group(3).lstrip('#'))
        return m.group(0) if c is None else f'{m.group(1)}{m.group(2)}"{"#" if m.group(3).startswith("#") else ""}{c}")'
    return re.sub(r'(contains\(\s*)((?:file\.)?tags,\s*)"([^"]+)"\)', contains_tag, q)


def cmd_fix_dataview(a):
    v, tm = Vault(), TagMap(); changed = 0
    for n in v.notes:
        new_body = DV_BLOCK.sub(lambda m: m.group(1) + rewrite_dataview(m.group(2), tm) + m.group(3), n.body)
        if new_body != n.body:
            changed += 1
            if a.write:
                with open(os.path.join(ROOT, n.path), 'w', encoding='utf-8') as fh:
                    fh.write(n.raw[:len(n.raw) - len(n.body)] + new_body)
    print(f'{changed} notes have Dataview queries to update' + ('' if a.write else ' (dry run; --write to apply)'))


# ---------------------------------------------------------------------------------------------------------- trim-related
def trim_related_text(fm_text: str, drop: set) -> tuple[str, int]:
    """Remove `related:` items whose link target (lower-cased) is in `drop`. Works on the text, so nothing else moves."""
    out, in_related, removed = [], False, 0
    for line in fm_text.split('\n'):
        if re.match(r'^[A-Za-z_][\w-]*:', line):
            in_related = line.startswith('related:')
        m = re.match(r'^\s+-\s+"?\[\[([^\]|#]+)(?:[#|][^\]]*)?\]\]"?\s*$', line) if in_related else None
        if m and nfc(m.group(1).strip()).lower() in drop:
            removed += 1; continue
        out.append(line)
    return '\n'.join(out), removed


def stray_related_targets(v: Vault) -> set:
    """Missing notes that only a single `related:` entry in the whole vault points at: stray 'see also' links."""
    refs = collections.Counter(); from_related = collections.Counter()
    for n in v.notes:
        for l in n.links:
            if l.target and v.resolve(l, n) is None:
                k = nfc(l.target).lower(); refs[k] += 1
                if l.source == 'frontmatter':
                    from_related[k] += 1
    return {k for k, c in refs.items() if c == 1 and from_related[k] == 1}


def cmd_trim_related(a):
    v = Vault(); drop = stray_related_targets(v); total = files = 0
    for n in v.notes:
        if n.fm_text is None:
            continue
        new_fm, c = trim_related_text(n.fm_text, drop)
        if c:
            total += c; files += 1
            if a.write:
                head = n.raw[:n.raw.index(n.fm_text)]
                tail = n.raw[n.raw.index(n.fm_text) + len(n.fm_text):]
                with open(os.path.join(ROOT, n.path), 'w', encoding='utf-8') as fh:
                    fh.write(head + new_fm + tail)
    print(f'{total} stray `related:` links to notes that nothing else mentions, in {files} notes' +
          (' (removed)' if a.write else ' (dry run; --write to remove)'))


# ------------------------------------------------------------------------------------------------------------ fix-fences
def unclosed_fence(body: str) -> str | None:
    """The opening fence (e.g. '```') if the note ends inside a code block that was never closed, else None."""
    fence = None
    for line in body.split('\n'):
        m = FENCE.match(line)
        if fence is None and m:
            fence = m.group(2)[0] * len(m.group(2)); continue
        if fence is not None and m and m.group(2).startswith(fence) and not m.group(3).strip():
            fence = None
    return fence


def cmd_fix_fences(a):
    """Close code blocks left open at the end of a note. Obsidian tolerates them; GitHub, Quartz and Dataview tooling do not."""
    v = Vault(); fixed = 0
    for n in v.notes:
        f = unclosed_fence(n.body)
        if f:
            fixed += 1
            if a.write:
                with open(os.path.join(ROOT, n.path), 'w', encoding='utf-8') as fh:
                    fh.write(n.raw.rstrip('\n') + '\n' + f + '\n')
    print(f'{fixed} notes end inside an unclosed code block' + (' (closed)' if a.write else ' (dry run; --write to close them)'))


# ------------------------------------------------------------------------------------------------------------- fix-links
def near_miss_index(v: Vault) -> dict:
    """Normalized spellings -> note: 'machine learning', 'Reinforcement Learning (RL)', 'Torques' all find their note."""
    def norm(s):
        return re.sub(r'[\s\-_]+', '_', re.sub(r'\.md$', '', nfc(str(s)).lower())).strip('_')
    idx = {}
    for n in v.notes:
        fm = n.frontmatter or {}
        names = [n.name, re.sub(r'_?\([^)]*\)$', '', n.name)]
        if fm.get('title'):
            names += [str(fm['title']), re.sub(r'\s*\([^)]*\)$', '', str(fm['title']))]
        names += [str(x) for x in fm.get('aliases') or []]
        for s in names:
            idx.setdefault(norm(s), n)
    def find(target):
        base = target.split('/')[-1]
        for c in (base, re.sub(r'\s*\([^)]*\)$', '', base), re.sub(r's$', '', base), base + 's'):
            if norm(c) in idx:
                return idx[norm(c)]
        return None
    return find


def fix_links_in(text: str, fix) -> tuple[str, int]:
    """Rewrite [[target|alias]] outside fenced and inline code. `fix(target) -> new target or None`."""
    out, count, fence = [], 0, None
    def repl(m):
        nonlocal count
        embed, inner = m.group(1), m.group(2)
        esc = '\\|' in inner
        parts = inner.replace('\\|', '|').split('|', 1)
        tgt, alias = parts[0], (parts[1] if len(parts) > 1 else None)
        head = ''
        if '#' in tgt:
            tgt, h = tgt.split('#', 1); head = '#' + h
        new = fix(tgt.strip())
        if not new:
            return m.group(0)
        count += 1
        shown = alias if alias is not None else (tgt.strip() if not head else None)
        sep = '\\|' if esc else '|'
        return f'{embed}[[{new}{head}' + (f'{sep}{shown}' if shown and shown != new else '') + ']]'
    for line in text.split('\n'):
        m = FENCE.match(line)
        if fence is None and m:
            fence = m.group(2)[0] * len(m.group(2)); out.append(line); continue
        if fence is not None:
            if m and m.group(2).startswith(fence) and not m.group(3).strip():
                fence = None
            out.append(line); continue
        pieces = re.split(r'(`+[^`]*`+)', line)          # keep inline code untouched
        out.append(''.join(p if p.startswith('`') else WIKILINK.sub(repl, p) for p in pieces))
    return '\n'.join(out), count


def cmd_fix_links(a):
    """Point links at the note they obviously mean ([[Machine Learning]] -> [[Machine_Learning|Machine Learning]])."""
    v = Vault(); find = near_miss_index(v)
    def fix(target):
        lk = Link(target=target, heading=None, alias=None, embed=False, source='body')
        if not target or v.resolve(lk, None) is not None:
            return None
        n = find(target)
        return n.name if n else None
    total, files = 0, 0
    for n in v.notes:
        fm_part = n.raw[:len(n.raw) - len(n.body)]
        new_fm, c1 = re.subn(r'"\[\[([^\]|#]+)\]\]"', lambda m: f'"[[{fix(m.group(1)) or m.group(1)}]]"', fm_part)
        c1 = sum(1 for m in re.finditer(r'"\[\[([^\]|#]+)\]\]"', fm_part) if fix(m.group(1)))
        new_body, c2 = fix_links_in(n.body, fix)
        if c1 + c2:
            total += c1 + c2; files += 1
            if a.write:
                with open(os.path.join(ROOT, n.path), 'w', encoding='utf-8') as fh:
                    fh.write(new_fm + new_body)
    print(f'{total} links in {files} notes point to an existing note under another spelling' + ('' if a.write else ' (dry run; --write to apply)'))


# ----------------------------------------------------------------------------------------------------------------- pages
GEN_START = '<!-- generated by tools/vault.py pages: do not edit below this line -->'
GEN_END = '<!-- end generated -->'
GROUPS = [('core', 'Core: the stack every robot is built on'), ('body', 'Body: how robots are shaped and move'),
          ('domain', 'Domains: where robots work'), ('society', 'Society: robots among people'),
          ('frontier', 'Frontier: directions still being invented')]


class Book:
    """The vault as a textbook: notes with v2 metadata, subfields, prerequisites and one global reading order."""

    def __init__(self, v: Vault):
        self.v = v
        self.sfs = load_yaml('subfields.yml').get('subfields') or {}
        self.notes = [n for n in v.notes if n.frontmatter and n.frontmatter.get('type') != 'guide']
        # links from vault guides (including the generated pages) do not count: otherwise each rebuild would shift itself
        guides = {n.path for n in v.notes if (n.frontmatter or {}).get('type') == 'guide'}
        self.inbound = {k: {s for s in srcs if s not in guides} for k, srcs in v.backlinks().items()}
        self.by_sub = collections.defaultdict(list)
        for n in self.notes:
            for s in n.frontmatter.get('subfield') or []:
                self.by_sub[s].append(n)
        self.prereq = {}
        for n in self.notes:
            ps = []
            for p in n.frontmatter.get('prerequisites') or []:
                lk = wikilink_target(p); r = v.resolve(lk, n) if lk else None
                if isinstance(r, Note):
                    ps.append(r)
            self.prereq[n.path] = ps
        self.rank = {n.path: i for i, n in enumerate(self.order(self.notes))}

    def key(self, n):
        fm = n.frontmatter
        return (LEVEL_RANK.get(fm.get('level'), 1), 0 if fm.get('type') == 'overview' else 1, chapter_key(n),
                -len(self.inbound.get(n.path, ())), self.title(n).lower())

    def order(self, notes):
        """Prerequisites first (topological), otherwise easier, overview, earlier chapter, more linked."""
        import heapq
        S = {n.path for n in notes}; by = {n.path: n for n in notes}
        indeg = {p: 0 for p in S}; kids = collections.defaultdict(list)
        for n in notes:
            for r in self.prereq.get(n.path, []):
                if r.path in S:
                    indeg[n.path] += 1; kids[r.path].append(n.path)
        heap = [(self.key(by[p]), p) for p in S if indeg[p] == 0]; heapq.heapify(heap); out = []
        while heap:
            _, p = heapq.heappop(heap); out.append(by[p])
            for c in kids[p]:
                indeg[c] -= 1
                if indeg[c] == 0:
                    heapq.heappush(heap, (self.key(by[c]), c))
        return out

    @staticmethod
    def title(n):
        return re.sub(r'[|\[\]]', '', str((n.frontmatter or {}).get('title') or n.name.replace('_', ' ')))

    def link(self, n, table=False):
        t = self.title(n)
        if t == n.name:
            return f'[[{n.name}]]'
        return f'[[{n.name}{chr(92) if table else ""}|{t}]]'

    def missing(self):
        """Missing notes: target -> (refs, linking notes)."""
        out = {}
        for n in self.notes:
            for l in n.links:
                if self.v.resolve(l, n) is None and l.target and not l.target.endswith('/'):
                    t = nfc(l.target.split('/')[-1]); k = t.lower()
                    e = out.setdefault(k, [t, 0, {}]); e[1] += 1; e[2][n.path] = n
        return out

    def fast(self, n) -> bool:
        return any(self.sfs.get(k, {}).get('fast_moving') for k in n.frontmatter.get('subfield') or [])

    def newest_as_of(self):
        months = [str(n.frontmatter['as_of']) for n in self.notes if MONTH.match(str(n.frontmatter.get('as_of', '')))]
        return max(months) if months else None

    def behind(self):
        """Notes in fast-moving subfields with no `as_of`, or one more than 12 months older than the newest `as_of` in the
        vault. Measured against the vault itself, not today's date, so the generated pages are reproducible."""
        ref = self.newest_as_of()
        def months(m):
            y, mo = map(int, m.split('-')); return y * 12 + mo
        out = []
        for n in self.notes:
            if not self.fast(n):
                continue
            a = str(n.frontmatter.get('as_of') or '')
            if not MONTH.match(a) or (ref and months(ref) - months(a) > 12):
                out.append(n)
        return out

    def likely_subfield(self, sources):
        c = collections.Counter(s for n in sources for s in (n.frontmatter or {}).get('subfield') or [])
        return c.most_common(1)[0][0] if c else None


def page_atlas(b: Book) -> str:
    L = []
    empty = [k for k in b.sfs if not b.by_sub.get(k)]
    L.append(f'**{len(b.notes)} notes** across **{len(b.sfs)} subfields**. '
             f'{len(empty)} subfields have no notes yet; each is an open invitation (see [[Frontier]]).\n')
    for g, gtitle in GROUPS:
        keys = [k for k, s in b.sfs.items() if s.get('group') == g]
        L.append(f'## {gtitle}\n')
        L.append('| Subfield | Notes | Beginner | Intermediate | Advanced | Research |')
        L.append('|---|---:|---:|---:|---:|---:|')
        for k in keys:
            ns = b.by_sub.get(k, [])
            c = collections.Counter(n.frontmatter.get('level') for n in ns)
            L.append(f'| [[#{b.sfs[k]["title"]}]] | {len(ns)} | {c["beginner"]} | {c["intermediate"]} | {c["advanced"]} | {c["research"]} |')
        L.append('')
        for k in keys:
            s = b.sfs[k]; ns = b.order(b.by_sub.get(k, []))
            L.append(f'### {s["title"]}\n')
            L.append(f'*{s["description"]}* (`subfield: {k}`)\n')
            if not ns:
                L.append('**No notes yet.** This subfield is waiting for its first note: see [[Frontier]] for how to start one.\n')
                continue
            keys_ = [n for n in ns if n.frontmatter.get('status') == 'keystone']
            if keys_:
                L.append('**Keystones:** ' + ' · '.join(b.link(n) for n in keys_) + '\n')
            L.append('**Start here:** ' + ' → '.join(b.link(n) for n in ns[:4]) + '\n')
            L.append(f'**All {len(ns)} notes, in reading order:** ' + ' · '.join(b.link(n) for n in ns) + '\n')
    return '\n'.join(L)


def page_frontier(b: Book) -> str:
    miss = b.missing(); L = []
    stubs = [n for n in b.notes if n.frontmatter.get('status') == 'stub']
    thin = [k for k in b.sfs if len(b.by_sub.get(k, [])) < 3]
    refs = sum(e[1] for e in miss.values())
    L.append(f'**{len(miss)} notes are wanted**: existing notes link to them {refs} times, but they have not been written yet. '
             f'**{len(b.behind())} notes** in fast-moving subfields need bringing up to date. '
             f'**{len(stubs)} stub{"s" if len(stubs) != 1 else ""}** need{"s" if len(stubs) == 1 else ""} content, and **{len(thin)} subfields** have fewer than three notes.\n')
    L.append('Pick one, write it, and open a pull request. The [[Vault_Taxonomy_Guide|Vault Taxonomy Guide]] has the frontmatter template; '
             '`python3 tools/vault.py lint` checks your note before you submit.\n')
    ranked = sorted(miss.values(), key=lambda e: (-e[1], e[0].lower()))
    top = [e for e in ranked if e[1] >= 3]
    L.append('## Most wanted\n')
    L.append(f'The {len(top)} missing notes that three or more links already point at, most wanted first. '
             'The file name to create is in `code`.\n')
    L.append('| # | Wanted note | Links | Linked from | Likely subfield |')
    L.append('|---:|---|---:|---|---|')
    for i, (t, c, src) in enumerate(top, 1):
        srcs = sorted(src.values(), key=lambda n: -len(b.inbound.get(n.path, ())))
        more = f' +{len(srcs) - 3}' if len(srcs) > 3 else ''
        sf = b.likely_subfield(srcs)
        fname = re.sub(r'\s+', '_', t)
        L.append(f'| {i} | **{t.replace("_", " ")}** `{fname}.md` | {c} | ' + ', '.join(b.link(n, table=True) for n in srcs[:3]) + more +
                 f' | {b.sfs[sf]["title"] if sf in b.sfs else "—"} |')
    L.append('')
    behind = b.behind(); ref = b.newest_as_of()
    L.append('## Behind the cutting edge\n')
    L.append('Robot learning, software, humanoids, manipulation and the frontier move fast. Notes in those subfields need an `as_of` '
             'date (the month their state-of-the-art content was last brought up to date) and a list of `sources`. '
             + (f'A note is listed here if it has no `as_of`, or one more than 12 months older than the newest in the textbook ({ref}).'
                if ref else 'A note is listed here until it has an `as_of`.') + '\n')
    bys = collections.defaultdict(list)
    for n in behind:
        bys[next(k for k in n.frontmatter.get('subfield') if b.sfs.get(k, {}).get('fast_moving'))].append(n)
    for k in [k for k in b.sfs if k in bys]:
        ns = sorted(bys[k], key=lambda n: (-len(b.inbound.get(n.path, ())), b.title(n)))
        L.append(f'**{b.sfs[k]["title"]}** ({len(ns)}): ' + ' · '.join(b.link(n) for n in ns) + '\n')
    if not behind:
        L.append('*Every note in a fast-moving subfield is up to date.*\n')
    L.append('## Stubs to fill\n')
    L.append('\n'.join(f'- {b.link(n)}: {n.frontmatter.get("description", "")}' for n in stubs) or '*None right now.*')
    L.append('')
    L.append('## Subfields with fewer than three notes\n')
    L.append('Robotics is wider than any table of contents. These parts of it are barely covered yet:\n')
    for k in thin:
        s = b.sfs[k]; n = len(b.by_sub.get(k, []))
        L.append(f'- **{s["title"]}** ({n} note{"s" if n != 1 else ""}): {s["description"]}')
    L.append('\nHave an idea for a part of robotics that is not on the list at all? Propose it: add it to `tools/subfields.yml` '
             'in the same pull request as its first note.\n')
    L.append('## Drafts awaiting review, by chapter\n')
    L.append('| Chapter | Notes | Draft | Reviewed | Keystone |')
    L.append('|---|---:|---:|---:|---:|')
    byc = collections.defaultdict(collections.Counter)
    for n in b.notes:
        byc[n.chapter][n.frontmatter.get('status')] += 1
    for ch in sorted(byc):
        c = byc[ch]
        L.append(f'| {ch.replace("_", " ")} | {sum(c.values())} | {c["draft"]} | {c["reviewed"]} | {c["keystone"]} |')
    L.append('')
    L.append('## Every wanted note, by likely subfield\n')
    groups = collections.defaultdict(list)
    for t, c, src in ranked:
        groups[b.likely_subfield(src.values()) or 'unsorted'].append((t, c))
    for k in [k for k in b.sfs if k in groups] + (['unsorted'] if 'unsorted' in groups else []):
        title = b.sfs[k]['title'] if k in b.sfs else 'Unsorted'
        items = ', '.join(f'{t.replace("_", " ")} ({c})' for t, c in groups[k])
        L.append(f'**{title}** ({len(groups[k])}): {items}\n')
    return '\n'.join(L)


def page_paths(b: Book) -> str:
    L = []
    def path(notes, limit=None):
        ns = b.order(notes)[:limit] if limit else b.order(notes)
        return '\n'.join(f'{i}. {b.link(n)} · *{n.frontmatter.get("level")}*' for i, n in enumerate(ns, 1))
    def pick(subs, levels, types=None, keystone_first=False, limit=None):
        ns = [n for n in b.notes if set(n.frontmatter.get('subfield') or []) & set(subs) and n.frontmatter.get('level') in levels
              and (types is None or n.frontmatter.get('type') in types)]
        if limit:
            ns = sorted(ns, key=lambda n: (n.frontmatter.get('status') != 'keystone', n.frontmatter.get('type') != 'overview',
                                           -len(b.inbound.get(n.path, ()))))[:limit]
        return ns
    personas = [
        ('Curious about robots', 'No background needed. What robots are, how they came to be, and the ideas everything else builds on.',
         pick(['foundations', 'history-future', 'kinematics-dynamics', 'mobile-robots', 'manipulation'], ['beginner'], limit=14)),
        ('Student: the core stack', 'The sequence most robotics courses follow: maths, then how bodies move, how robots sense, estimate, plan and control.',
         pick(['mathematics', 'kinematics-dynamics', 'perception', 'estimation-mapping', 'planning', 'control'], ['beginner', 'intermediate'], limit=24)),
        ('Builder: make a robot work', 'Hardware, software and tooling: actuators, electronics, ROS 2, simulation and the labs.',
         pick(['mechanisms-actuation', 'electronics-embedded', 'software', 'systems-engineering'], ['beginner', 'intermediate'], limit=20)),
        ('Manufacturing: robots on the factory floor', 'How robots are put to work in factories: arms, work cells, controllers, welding, vision and safety.',
         pick(['industrial-automation'], ['beginner', 'intermediate', 'advanced'])),
        ('Learning-based robotics', 'From machine learning basics to deep and reinforcement learning on real robots.',
         pick(['robot-learning'], ['beginner', 'intermediate', 'advanced'], limit=18)),
        ('Researcher: the edge of the field', 'Advanced methods, open problems and the directions still being invented.',
         [n for n in b.notes if n.frontmatter.get('level') in ('advanced', 'research') or 'frontiers' in (n.frontmatter.get('subfield') or [])]),
    ]
    L.append('Each path lists notes so that **every note comes after its prerequisites**. Among notes that do not depend on each other, '
             'easier and more introductory ones come first. Paths are rebuilt from the notes\' `prerequisites`, `level` and `subfield` '
             'whenever the vault changes.\n')
    L.append('## Start here, by who you are\n')
    for name, desc, ns in personas:
        L.append(f'### {name}\n\n{desc}\n\n{path(ns)}\n')
    L.append('## Paths through each subfield\n')
    L.append('Every subfield with three or more notes, in reading order. The [[Subfield_Atlas|Subfield Atlas]] describes each subfield; '
             'the [[Frontier]] lists the ones still waiting for notes.\n')
    for k, s in b.sfs.items():
        ns = b.by_sub.get(k, [])
        if len(ns) >= 3:
            L.append(f'### {s["title"]}\n\n{path(ns)}\n')
    return '\n'.join(L)


def page_whats_new(b: Book) -> str:
    L = []
    cur = sorted([n for n in b.notes if MONTH.match(str(n.frontmatter.get('as_of', '')))],
                 key=lambda n: (str(n.frontmatter['as_of']), b.title(n).lower()), reverse=True)
    L.append(f'**{len(cur)} notes** carry an `as_of` date: their state-of-the-art content was checked against current sources that month. '
             f'**{len(b.behind())} notes** in fast-moving subfields are waiting for the same treatment (see [[Frontier]]).\n')
    L.append('## Brought up to date\n')
    if cur:
        by_m = collections.defaultdict(list)
        for n in cur:
            by_m[str(n.frontmatter['as_of'])].append(n)
        for m in sorted(by_m, reverse=True):
            L.append(f'**{m}**: ' + ' · '.join(b.link(n) for n in by_m[m]) + '\n')
    else:
        L.append('*No notes carry an `as_of` date yet.*\n')
    def recent(field, k=25):
        ns = [n for n in b.notes if DATE.match(str(n.frontmatter.get(field, '')))]
        return sorted(ns, key=lambda n: (str(n.frontmatter[field]), b.title(n).lower()), reverse=True)[:k]
    L.append('## Recently reviewed\n')
    L.append('\n'.join(f'- {n.frontmatter["last_reviewed"]}: {b.link(n)} · *{n.frontmatter.get("status")}*' for n in recent('last_reviewed')) or '*None yet.*')
    L.append('\n## Recently added\n')
    L.append('\n'.join(f'- {n.frontmatter["date"]}: {b.link(n)} · *{n.frontmatter.get("level")}*' for n in recent('date')))
    return '\n'.join(L)


PAGES = {
    '00_Introduction/Subfield_Atlas.md': (page_atlas, 'Subfield Atlas',
        'A map of robotics as a field without edges: every subfield, how many notes cover it at each level, its keystone notes and where to start.',
        ['vault-meta', 'learning-paths'], ['[[Learning_Paths]]', '[[Frontier]]', '[[Robotics_Vault_Dashboard]]'],
        'Robotics is not one subject. It is a meeting point for mathematics, mechanics, electronics, computing, biology, design and ethics, '
        'and it keeps growing into places nobody planned for. This atlas maps the part of it this textbook covers so far, '
        'grouped from the core stack every robot needs, to the bodies robots take, the places they work, the people they work with, '
        'and the frontier.\n\nEvery note names its subfields in its `subfield:` frontmatter, so this page rebuilds itself as the textbook grows. '
        'Subfields with no notes yet are listed on purpose.'),
    '00_Introduction/Frontier.md': (page_frontier, 'Frontier',
        'What this textbook is missing: notes that other notes already link to, stubs to fill, and parts of robotics that are barely covered yet.',
        ['vault-meta', 'contributing'], ['[[Subfield_Atlas]]', '[[Learning_Paths]]', '[[Vault_Taxonomy_Guide]]'],
        'A textbook about a field that is still being invented is never finished. This page is the list of what to write next, '
        'built from the textbook itself: every time a note links to a topic that has no note yet, that topic appears here, '
        'ranked by how many notes are waiting for it.\n\nIt is the best place to start contributing, for people and for AI agents alike.'),
    '00_Introduction/Whats_New.md': (page_whats_new, "What's New",
        'A living changelog: notes recently brought up to date with current sources, recently reviewed, and recently added.',
        ['vault-meta'], ['[[Frontier]]', '[[Subfield_Atlas]]', '[[Learning_Paths]]'],
        'Robotics changes month by month, and so does this textbook. This page lists what has changed recently, '
        'built from each note\'s `as_of`, `last_reviewed` and `date` fields.'),
    '00_Introduction/Learning_Paths.md': (page_paths, 'Learning Paths',
        'Reading orders through the textbook, built from each note\'s prerequisites and level: one path per kind of reader and one per subfield.',
        ['vault-meta', 'learning-paths'], ['[[Subfield_Atlas]]', '[[Frontier]]', '[[How_to_Use_This_Vault]]'],
        'You do not have to read this textbook front to back. Pick the path that matches why you are here, or a subfield you care about. '
        'Every note lists its `prerequisites`, so each path below puts the ideas you need before the ideas that use them.'),
}


def render_page(path, spec, b: Book) -> str:
    fn, title, desc, tags, related, intro = spec
    full = os.path.join(ROOT, path)
    gen = GEN_START + '\n\n' + fn(b).rstrip() + '\n\n' + GEN_END + '\n'
    if os.path.exists(full):
        with open(full, encoding='utf-8') as fh:
            cur = fh.read()
        if GEN_START in cur and GEN_END in cur:
            return cur[:cur.index(GEN_START)] + gen + cur[cur.index(GEN_END) + len(GEN_END) + 1:]
    fm = {'title': title, 'description': desc, 'tags': tags, 'type': 'guide', 'level': 'beginner', 'status': 'reviewed',
          'author': 'Jordan_Smith', 'date': '2026-11-15', 'related': related}
    return f'---\n{emit_frontmatter(fm)}\n---\n\n# {title}\n\n{intro}\n\n{gen}'


README_START = '<!-- stats: generated by tools/vault.py pages -->'
README_END = '<!-- end stats -->'


def readme_stats(b: Book) -> str:
    st = collections.Counter(n.frontmatter.get('status') for n in b.notes)
    lv = collections.Counter(n.frontmatter.get('level') for n in b.notes)
    covered = len([k for k in b.sfs if b.by_sub.get(k)])
    L = [f'**{len(b.notes)} notes** · **{sum(n.word_count for n in b.notes):,} words** · '
         f'**{covered} of {len(b.sfs)} subfields** covered · **{st["keystone"]} keystone entries** · '
         f'**{len(b.missing())} notes wanted** ([Frontier](00_Introduction/Frontier.md))', '',
         f'Levels: {lv["beginner"]} beginner · {lv["intermediate"]} intermediate · {lv["advanced"]} advanced · {lv["research"]} research. '
         f'Status: {st["keystone"]} keystone · {st["reviewed"]} reviewed · {st["draft"]} draft · {st["stub"]} stub.', '',
         '| Chapter | Notes |', '|---|---:|']
    byc = collections.Counter(n.chapter for n in b.notes)
    for ch in sorted(byc):
        if ch[:2].isdigit() and ch[:2] != '00':      # 00_Introduction holds the vault guides
            L.append(f'| [{ch[:2]} · {ch[3:].replace("_", " ")}]({urllib_quote(ch)}/) | {byc[ch]} |')
    return '\n'.join(L)


def urllib_quote(s):
    import urllib.parse
    return urllib.parse.quote(s, safe='/-_.~')


def cmd_pages(a):
    v = Vault(); b = Book(v); stale = []
    readme = os.path.join(ROOT, 'README.md')
    if os.path.exists(readme):
        with open(readme, encoding='utf-8') as fh:
            cur = fh.read()
        if README_START in cur and README_END in cur:
            new = cur[:cur.index(README_START) + len(README_START)] + '\n' + readme_stats(b) + '\n' + cur[cur.index(README_END):]
            if new != cur:
                stale.append('README.md')
                if a.write:
                    with open(readme, 'w', encoding='utf-8') as fh:
                        fh.write(new)
    for path, spec in PAGES.items():
        new = render_page(path, spec, b)
        full = os.path.join(ROOT, path)
        old = open(full, encoding='utf-8').read() if os.path.exists(full) else None
        if new != old:
            stale.append(path)
            if a.write:
                with open(full, 'w', encoding='utf-8') as fh:
                    fh.write(new)
    if a.check and stale:
        print('generated pages are out of date: ' + ', '.join(stale) + '\nrun `python3 tools/vault.py pages --write` and commit the result')
        sys.exit(1)
    print(f'{len(PAGES)} pages, {len(stale)} ' + ('updated' if a.write else 'out of date'))


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
    v = Vault(repo_docs=True)                  # Obsidian indexes the repo docs too
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
def refresh_queue(v, b):
    """Notes still to be brought up to date, most urgent first. A note is done when it has `as_of` and enough
    `sources`. Urgency: fast-moving subfield (state of the art changes yearly), draft status, how many notes link
    to it (fixing it improves the most reading paths), then shortness."""
    inbound = {k: len(s) for k, s in b.inbound.items()}
    rows = []
    for n in b.notes:
        fm = n.frontmatter
        if fm.get('as_of') and len(fm.get('sources') or []) >= 3:
            continue
        score = 4 * b.fast(n) + 2 * (fm.get('status') in ('draft', 'stub')) + min(inbound.get(n.path, 0), 20) / 5
        rows.append((round(score, 1), inbound.get(n.path, 0), n))
    rows.sort(key=lambda r: (-r[0], -r[1], r[2].word_count, r[2].path))
    return rows


def cmd_queue(a):
    v = Vault(); b = Book(v)
    rows = refresh_queue(v, b)
    if a.chapter:
        rows = [r for r in rows if r[2].chapter.startswith(a.chapter)]
    if a.json:
        print(json.dumps([dict(path=n.path, score=s, inbound=i, status=n.frontmatter.get('status'), fast=b.fast(n),
                               words=n.word_count) for s, i, n in rows], indent=1)); return
    print(f'{len(rows)} notes to refresh (done = as_of + 3 sources)')
    for s, i, n in rows[:a.limit or None]:
        print(f'{s:5.1f}  in={i:3d}  {"fast " if b.fast(n) else "     "}{n.frontmatter.get("status", ""):9}{n.word_count:6d}w  {n.path}')


TAG_WORD = re.compile(r'[a-z0-9]+')


def suggest_tags(n, vocabulary, min_hits=2, limit=6):
    """Vocabulary tags a note's text discusses but its `tags:` omit. A tag matches when all its words appear as a
    phrase (hyphens = spaces); the title and headings count triple. Suggestions only: a person accepts them."""
    text = ' '.join(TAG_WORD.findall(n.body.lower()))
    heads = ' '.join(TAG_WORD.findall(' '.join([n.name.replace('_', ' ')] + re.findall(r'^#+ (.*)$', n.body, re.M)).lower()))
    have = {t.lower() for t in n.fm_tags}
    out = []
    for t in vocabulary:
        if t in have or len(t) < 3:
            continue
        phrase = ' '.join(t.split('-'))
        pat = re.compile(r'\b' + re.escape(phrase) + r's?\b')
        hits = len(pat.findall(text)) + 3 * len(pat.findall(heads))
        if hits >= min_hits:
            out.append((hits, t))
    return [t for _, t in sorted(out, key=lambda x: (-x[0], x[1]))[:limit]]


def cmd_suggest_tags(a):
    v = Vault(); tm = TagMap()
    want = {x.removesuffix('.md') for x in a.notes}
    for n in v.notes:
        if want and n.name not in want and n.path.removesuffix('.md') not in want:
            continue
        if (n.frontmatter or {}).get('type') == 'guide':
            continue
        s = suggest_tags(n, sorted(tm.vocabulary), a.min_hits)
        if s:
            print(f'{n.path}: ' + ', '.join(s))


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
    p = sub.add_parser('migrate', help='propose (and with --write apply) schema v2 frontmatter')
    p.add_argument('--chapters', help='comma-separated chapter numbers, e.g. 00,01')
    p.add_argument('--write', action='store_true'); p.add_argument('--report', help='write proposals as TSV')
    p.add_argument('--overrides', help='YAML of {path: {field: value}} corrections')
    p.set_defaults(fn=cmd_migrate)
    p = sub.add_parser('fix-dataview', help='map tag references inside ```dataview blocks to the v2 vocabulary')
    p.add_argument('--write', action='store_true')
    p.set_defaults(fn=cmd_fix_dataview)
    p = sub.add_parser('pages', help='rebuild Subfield Atlas, Frontier and Learning Paths in 00_Introduction')
    p.add_argument('--write', action='store_true'); p.add_argument('--check', action='store_true', help='exit 1 if out of date')
    p.set_defaults(fn=cmd_pages)
    p = sub.add_parser('trim-related', help='remove related: links to missing notes that nothing else mentions')
    p.add_argument('--write', action='store_true')
    p.set_defaults(fn=cmd_trim_related)
    p = sub.add_parser('fix-fences', help='close code blocks left open at the end of a note')
    p.add_argument('--write', action='store_true')
    p.set_defaults(fn=cmd_fix_fences)
    p = sub.add_parser('fix-links', help='repoint links that misspell an existing note ([[Machine Learning]] -> Machine_Learning)')
    p.add_argument('--write', action='store_true')
    p.set_defaults(fn=cmd_fix_links)
    p = sub.add_parser('queue', help='notes still to bring up to date (as_of + sources), most urgent first')
    p.add_argument('--chapter', default=''); p.add_argument('--limit', type=int, default=40); p.add_argument('--json', action='store_true')
    p.set_defaults(fn=cmd_queue)
    p = sub.add_parser('suggest-tags', help='vocabulary tags a note discusses but does not carry (suggestions only)')
    p.add_argument('notes', nargs='*'); p.add_argument('--min-hits', type=int, default=3)
    p.set_defaults(fn=cmd_suggest_tags)
    p = sub.add_parser('obsidian-check', help="compare with Obsidian's own index via the Obsidian CLI (local only)")
    p.add_argument('--obsidian', help='path to the obsidian CLI'); p.add_argument('--vault', help='Obsidian vault name')
    p.set_defaults(fn=cmd_obsidian_check)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == '__main__':
    main()
