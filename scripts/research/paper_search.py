#!/usr/bin/env python3
"""paper_search.py — 學術文獻檢索(OpenAlex + arXiv + Semantic Scholar)。

為什麼是自建腳本而不是 MCP connector:2026-08-04 查過 MCP registry 與外掛目錄,
OpenAlex / Semantic Scholar / arXiv / Consensus **都沒有可安裝的 connector**。
這三家有公開 REST API,直接打即可;Consensus 需付費帳號金鑰,沒有免費路徑。

各來源現況(實測):
    OpenAlex          免金鑰、涵蓋最廣、有被引數。附 mailto 進 polite pool 比較快。
    arXiv             免金鑰、預印本最新。⚠️ 必須用 https,http 會 301 且 curl 不跟。
    Semantic Scholar  無金鑰時**穩定 429**。設 S2_API_KEY 才會啟用(免費申請)。

用法:
    python3 paper_search.py "cooperative salvo guidance" --since 2018 --limit 25
    python3 paper_search.py "impact time control guidance" --out /tmp/lit.md
    S2_API_KEY=xxx python3 paper_search.py "..."        # 一併查 Semantic Scholar
"""
import argparse
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

MAILTO = os.environ.get("OPENALEX_MAILTO", "41030221@gm.nfu.edu.tw")
UA = f"aeroplan-studio-litsearch (mailto:{MAILTO})"


def _get(url, headers=None, retries=3):
    """帶退避的 GET。⚠️ 429 一定要退避重試 —— Semantic Scholar 無金鑰時很常觸發,
    直接放棄會讓整批檢索靜默少一個來源。"""
    for k in range(retries):
        req = urllib.request.Request(url, headers={"User-Agent": UA, **(headers or {})})
        try:
            with urllib.request.urlopen(req, timeout=40) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code in (429, 500, 502, 503) and k < retries - 1:
                time.sleep(2 ** k * 3)
                continue
            return None
        except Exception:
            if k < retries - 1:
                time.sleep(2)
                continue
            return None
    return None


def _norm(t):
    """標題正規化,用來跨來源去重。"""
    return re.sub(r"[^a-z0-9]+", " ", (t or "").lower()).strip()


def openalex(q, since, limit):
    # ⚠️ 不要用 `search=` 參數 —— 它做的是寬鬆全文比對,會把高被引但完全不相干的
    #    綜述拉到前面。實測用 "cooperative guidance impact angle" 查,第一名是
    #    〈昆蟲功能材料〉(199 引)、第二名是〈雙邊貿易〉(165 引)。
    #    改用 filter=title_and_abstract.search:,只比對標題與摘要,精準得多。
    #    多字詞用 AND 串起來,每個詞都必須出現。
    terms = " AND ".join(t for t in q.replace('"', "").split() if len(t) > 2)
    url = ("https://api.openalex.org/works?" + urllib.parse.urlencode({
        "per-page": min(limit, 50), "mailto": MAILTO,
        "filter": (f"from_publication_date:{since}-01-01,"
                   f"title_and_abstract.search:{terms}"),
        "sort": "cited_by_count:desc"}))
    raw = _get(url)
    if not raw:
        return []
    out = []
    for w in json.loads(raw).get("results", []):
        # OpenAlex 的摘要是**倒排索引**(詞→位置),要還原成文字。
        inv = w.get("abstract_inverted_index") or {}
        pos = {}
        for word, idxs in inv.items():
            for i in idxs:
                pos[i] = word
        abstract = " ".join(pos[i] for i in sorted(pos)) if pos else ""
        out.append({
            "src": "OpenAlex", "title": w.get("title") or "",
            "year": w.get("publication_year"),
            "cites": w.get("cited_by_count", 0),
            "venue": ((w.get("primary_location") or {}).get("source") or {}).get(
                "display_name") or "",
            "doi": (w.get("doi") or "").replace("https://doi.org/", ""),
            "url": w.get("doi") or w.get("id", ""),
            "abstract": abstract,
            "oa": (w.get("open_access") or {}).get("is_oa", False)})
    return out


def arxiv(q, limit):
    url = ("https://export.arxiv.org/api/query?" + urllib.parse.urlencode({
        "search_query": f"all:{q}", "max_results": min(limit, 50),
        "sortBy": "relevance", "sortOrder": "descending"}))
    raw = _get(url)
    if not raw:
        return []
    ns = {"a": "http://www.w3.org/2005/Atom"}
    out = []
    for e in ET.fromstring(raw).findall("a:entry", ns):
        g = lambda t: (e.findtext(f"a:{t}", "", ns) or "").strip()
        pub = g("published")
        out.append({
            "src": "arXiv", "title": " ".join(g("title").split()),
            "year": int(pub[:4]) if pub[:4].isdigit() else None,
            "cites": None, "venue": "arXiv (preprint)",
            "doi": e.findtext("a:doi", "", ns) or "",
            "url": g("id"), "abstract": " ".join(g("summary").split()),
            "oa": True})
    return out


def semantic_scholar(q, since, limit):
    key = os.environ.get("S2_API_KEY")
    if not key:
        return []                    # 無金鑰必 429,不要假裝查過
    url = ("https://api.semanticscholar.org/graph/v1/paper/search?"
           + urllib.parse.urlencode({
               "query": q, "limit": min(limit, 50), "year": f"{since}-",
               "fields": "title,year,citationCount,abstract,venue,"
                         "externalIds,openAccessPdf"}))
    raw = _get(url, headers={"x-api-key": key})
    if not raw:
        return []
    out = []
    for p in json.loads(raw).get("data", []):
        ext = p.get("externalIds") or {}
        out.append({
            "src": "S2", "title": p.get("title") or "", "year": p.get("year"),
            "cites": p.get("citationCount", 0), "venue": p.get("venue") or "",
            "doi": ext.get("DOI", ""),
            "url": (f"https://doi.org/{ext['DOI']}" if ext.get("DOI")
                    else f"https://semanticscholar.org/paper/{p.get('paperId','')}"),
            "abstract": p.get("abstract") or "",
            "oa": bool(p.get("openAccessPdf"))})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("query")
    ap.add_argument("--since", type=int, default=2015)
    ap.add_argument("--limit", type=int, default=25)
    ap.add_argument("--out", default=None)
    ap.add_argument("--min-cites", type=int, default=0)
    a = ap.parse_args()

    rows, seen, missing = [], set(), []
    for name, fn in [("OpenAlex", lambda: openalex(a.query, a.since, a.limit)),
                     ("arXiv", lambda: arxiv(a.query, a.limit)),
                     ("Semantic Scholar",
                      lambda: semantic_scholar(a.query, a.since, a.limit))]:
        got = fn()
        if not got:
            missing.append(name)
        for r in got:
            k = _norm(r["title"])
            if not k or k in seen:
                continue
            if r["year"] and r["year"] < a.since:
                continue
            if (r["cites"] or 0) < a.min_cites and r["src"] != "arXiv":
                continue
            seen.add(k)
            rows.append(r)

    # 排序:先被引數(None 當 -1,預印本沉底但不丟掉),再年份
    rows.sort(key=lambda r: (-(r["cites"] if r["cites"] is not None else -1),
                             -(r["year"] or 0)))

    L = [f"# 檢索:{a.query}", "",
         f"來源 OpenAlex / arXiv / Semantic Scholar;{a.since} 年以後;共 {len(rows)} 篇"]
    # ⚠️ 沒查到的來源一定要講出來,不然「少一個來源」會偽裝成「那邊沒有文獻」。
    if missing:
        L.append(f"⚠️ 無回應/未啟用的來源:{', '.join(missing)}"
                 "(Semantic Scholar 需設 S2_API_KEY,否則必定 429)")
    L += ["", "| # | 年 | 被引 | 標題 | 出處 |", "|---|---|---|---|---|"]
    for i, r in enumerate(rows, 1):
        c = "—" if r["cites"] is None else r["cites"]
        L.append(f"| {i} | {r['year'] or '—'} | {c} | [{r['title'][:95]}]({r['url']}) "
                 f"| {r['venue'][:38]} |")
    L += ["", "---", ""]
    for i, r in enumerate(rows, 1):
        L += [f"### {i}. {r['title']}",
              f"*{r['year']} · {r['venue']} · 被引 "
              f"{'—' if r['cites'] is None else r['cites']} · {r['src']}"
              f"{' · OA' if r['oa'] else ''}*", "",
              (r["abstract"][:1400] + ("…" if len(r["abstract"]) > 1400 else ""))
              if r["abstract"] else "(無摘要)", "", f"<{r['url']}>", ""]

    txt = "\n".join(L)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(txt)
        print(f"{len(rows)} 篇 → {a.out}"
              + (f"(缺:{', '.join(missing)})" if missing else ""))
    else:
        sys.stdout.write(txt)


if __name__ == "__main__":
    main()
