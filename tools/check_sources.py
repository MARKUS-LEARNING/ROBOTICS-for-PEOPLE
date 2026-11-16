#!/usr/bin/env python3
"""Check every citation in the notes' `sources:` lists.

For each citation ("Author or org (Year). Title. URL"):
  - the URL must answer (HTTP 2xx after redirects);
  - for arXiv papers, the cited title must match arXiv's own record (via the arXiv API, one batched request).

Usage:  python3 tools/check_sources.py [note names or paths ...]     (default: every note with sources)
Exit status 1 if any URL fails or any arXiv title does not match. Sites that refuse automated requests
(HTTP 401/403/429, or a timeout after retries) are reported as "blocked" and do not fail the check; read those by hand.
"""
from __future__ import annotations

import concurrent.futures, difflib, os, re, sys, time, urllib.error, urllib.parse, urllib.request
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vault as V

UA = 'robotics-for-people-source-check/1.0 (+https://github.com/MARKUS-LEARNING/ROBOTICS-for-PEOPLE)'
ARXIV = re.compile(r'arxiv\.org/(?:abs|pdf|html)/(\d{4}\.\d{4,5})(?:v\d+)?', re.I)
CITE = re.compile(r'^(?P<who>.+?)\s*\((?P<year>(?:19|20)\d{2})[a-z]?\)\.\s*(?P<title>.+?)\.?\s+(?P<url>https?://\S+)\s*$', re.S)


def norm(s: str) -> str:
    s = re.sub(r'\$[^$]*\$', '', s)                       # drop LaTeX in arXiv titles
    return re.sub(r'[^a-z0-9]+', ' ', V.nfc(s).lower()).strip()


def same_title(cited: str, real: str) -> bool:
    a, b = norm(cited), norm(real)
    return bool(a) and (a in b or b in a or difflib.SequenceMatcher(None, a, b).ratio() >= 0.85)


def fetch(url: str, tries: int = 3) -> tuple[int, str]:
    """Status and final URL. Network errors and 5xx are retried with backoff, so one slow
    server does not fail CI; a link only counts as dead if every attempt fails."""
    for attempt in range(tries):
        status, where = _fetch_once(url)
        if status and status < 500:
            return status, where
        if attempt < tries - 1:
            time.sleep(2 * (attempt + 1))
    return status, where


def _fetch_once(url: str) -> tuple[int, str]:
    req = urllib.request.Request(url, headers={'User-Agent': UA, 'Accept': 'text/html,application/pdf,*/*'})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            r.read(2048)
            return r.status, r.geturl()
    except urllib.error.HTTPError as e:
        return e.code, url
    except Exception as e:                                # DNS, timeout, TLS
        return 0, f'{type(e).__name__}: {e}'


def _arxiv_query(q: str, waits=(10, 30, 60)):
    """One arXiv API call, retried on rate limits (429), 5xx and network errors.
    Returns the parsed feed, or None if arXiv still refuses after every retry."""
    for attempt in range(len(waits) + 1):
        try:
            req = urllib.request.Request(q, headers={'User-Agent': UA})
            with urllib.request.urlopen(req, timeout=60) as r:
                return ET.fromstring(r.read())
        except urllib.error.HTTPError as e:
            if e.code != 429 and e.code < 500:
                raise
            err = f'HTTP {e.code}'
            retry_after = e.headers.get('Retry-After', '') if e.headers else ''
        except (urllib.error.URLError, TimeoutError, ET.ParseError) as e:
            err, retry_after = f'{type(e).__name__}: {e}', ''
        if attempt < len(waits):
            wait = int(retry_after) if retry_after.isdigit() else waits[attempt]
            print(f'arXiv API: {err}; retrying in {wait}s', file=sys.stderr)
            time.sleep(min(wait, 120))
    print(f'arXiv API: {err} after {len(waits) + 1} attempts', file=sys.stderr)
    return None


def arxiv_titles(ids: list[str]) -> dict | None:
    out = {}
    for i in range(0, len(ids), 50):
        q = 'https://export.arxiv.org/api/query?' + urllib.parse.urlencode({'id_list': ','.join(ids[i:i + 50]), 'max_results': 50})
        root = _arxiv_query(q)
        if root is None:
            return None                                    # arXiv unreachable: titles go unchecked
        ns = {'a': 'http://www.w3.org/2005/Atom'}
        for e in root.findall('a:entry', ns):
            aid = re.search(r'(\d{4}\.\d{4,5})', e.findtext('a:id', '', ns) or '')
            if aid:
                out[aid.group(1)] = ' '.join((e.findtext('a:title', '', ns) or '').split())
        time.sleep(3)                                      # arXiv asks for a pause between API calls
    return out


def main(argv):
    v = V.Vault()
    want = {a.removesuffix('.md') for a in argv}
    notes = [n for n in v.notes if (n.frontmatter or {}).get('sources') and (not want or n.name in want or n.path.removesuffix('.md') in want)]
    cites = []                                             # (note, citation text, parsed match or None)
    for n in notes:
        for c in n.frontmatter['sources']:
            cites.append((n, str(c), CITE.match(str(c))))
    urls = sorted({m.group('url') for _, _, m in cites if m})
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex:
        status = dict(zip(urls, ex.map(fetch, urls)))
    ids = sorted({ARXIV.search(m.group('url')).group(1) for _, _, m in cites if m and ARXIV.search(m.group('url'))})
    titles = arxiv_titles(ids) if ids else {}
    if titles is None:                                     # rate-limited: still check URLs, flag titles as unchecked
        print('::warning::arXiv API unavailable; arXiv titles were not checked this run')
    failed = blocked = 0
    for n, c, m in cites:
        if not m:
            failed += 1; print(f'FORMAT   {n.name}: {c[:90]}'); continue
        code, final = status[m.group('url')]
        if code in (401, 403, 429) or (code == 0 and 'timed out' in final):
            # Bot walls and servers that silently drop datacenter traffic: the link may be fine.
            blocked += 1; print(f'BLOCKED  {n.name}: {"timed out" if code == 0 else f"HTTP {code}"} {m.group("url")}')
        elif not 200 <= code < 400:
            failed += 1; print(f'DEAD     {n.name}: HTTP {code} {m.group("url")} {final if code == 0 else ""}')
        a = ARXIV.search(m.group('url'))
        if a and titles is not None:
            real = titles.get(a.group(1))
            if real is None:
                failed += 1; print(f'ARXIV?   {n.name}: {a.group(1)} not found in the arXiv API')
            elif not same_title(m.group('title'), real):
                failed += 1; print(f'TITLE    {n.name}: cited "{m.group("title")}" but arXiv {a.group(1)} is "{real}"')
    print(f'{len(cites)} citations in {len(notes)} notes: {len(urls)} URLs, {len(ids) if titles is not None else 0} arXiv papers checked; '
          f'{failed} problems, {blocked} blocked by the site')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
