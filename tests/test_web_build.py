"""The pages in docs/ are self-contained and match their sources (tools/build_web.py), so
that they work on anonymous.4open.science (sandboxed, no CORS, 175 requests per 15 min)
and from the disk. Runs without Node.js: each bundle carries the digest of the module sources.
"""
import importlib.util
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
spec = importlib.util.spec_from_file_location("build_web", ROOT / "tools" / "build_web.py")
build_web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_web)


def page(name):
    return (DOCS / name).read_text(encoding="utf-8").replace("\r\n", "\n")


class WebBuild(unittest.TestCase):
    def test_bundles_match_sources(self):
        digest = build_web.sources_digest()
        for name in build_web.PAGES:
            blocks = {m[2]: m[3] for m in build_web.BLOCK.finditer(page(name))}
            entries = [path for path in blocks if path.startswith("js/pages/")]
            self.assertEqual(len(entries), 1, name)
            self.assertTrue(blocks[entries[0]].strip("\n").startswith(build_web.bundle_header(entries[0], digest).strip("\n")),
                            f"docs/{name}: the page script is out of date, run python tools/build_web.py")

    def test_inline_blocks_match_sources(self):
        for name in build_web.PAGES:
            html = page(name)
            blocks = {m[2]: m[3] for m in build_web.BLOCK.finditer(html)}
            self.assertEqual(build_web.render(html, lambda entry: blocks[entry]), html,
                             f"docs/{name} is out of date: run python tools/build_web.py")

    def test_pages_load_no_files(self):
        for name in build_web.PAGES:
            html = page(name)
            self.assertNotIn('type="module"', html, name)
            self.assertIsNone(re.search(r'<script[^>]*\ssrc=', html), name)
            self.assertIsNone(re.search(r'<link[^>]*rel="stylesheet"', html), name)
            for m in build_web.ATTRIBUTE.finditer(html):
                self.assertTrue(html[m.start():m.end()].split('"')[1].startswith("data:"), f"{name}: {m[3]}")


if __name__ == "__main__":
    unittest.main()
