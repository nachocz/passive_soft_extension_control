# Building the interactive pages

Each page in `docs/` is a single HTML file that contains its style sheets, the KaTeX fonts, its script, its data, and its figures, so that a page view is one request (the video and the music load their files only when played).

The sources of the pages are the HTML outside the blocks marked `data-inline`, `docs/css/app.css`, and the ES modules in `docs/js/lib`, `docs/js/pages`, and `docs/js/sim` (the JavaScript port of `softext/`). After editing them, `docs/data/model.json`, or `docs/img`, fill the `data-inline` blocks again (Node.js is needed; `npx` fetches esbuild):

```bash
python tools/build_web.py     # writes the style sheets, scripts, data, and figures into docs/*.html
python run.py --sandbox       # check the pages as anonymous.4open.science serves them
```

`docs/data/model.json` holds the span map identified by the paper's simulator, the scenarios S1–S3, and the previews of the overview page. Regenerate it with `python examples/export_web_data.py`, then run `python tools/build_web.py`. The figures in `docs/img` are the figures of the paper, rendered from their PDF files.

## Hosting

- GitHub Pages: select *Deploy from a branch*, branch `main`, folder `/docs`.
- Because each page is self-contained, the pages also work on hosts that serve every page in a sandboxed origin without CORS headers and limit the number of requests, such as the anonymized pages of anonymous.4open.science (175 requests per 15 minutes). That host also refuses to show a page inside another one, so there the pages open on their own instead of inside `player.html`, and the links between them carry the theme and the music.

## Tests

`tests/test_web_parity.py` runs the JavaScript port with Node.js (skipped without it) and checks that it gives the same plant, identification, closed-loop logs, and analysis quantities as the Python package. `tests/test_web_build.py` checks that the pages are self-contained and match their sources.

## Third-party files

KaTeX and three.js r160 are included in `docs/vendor/`. See `docs/vendor/three/README.md` for the one-line change to `OrbitControls.js`.
