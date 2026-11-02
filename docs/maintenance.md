# Website maintenance

## Features

The homepage lives in `index.html`; feature details live in
`pages/<feature>/index.html`. Add a unique, lowercase, hyphenated section ID.
The navbar must link to a real homepage section or existing detail page.
Use paths relative to each page so deployment under `/AROC/` works.
`styles/refresh.css` contains the current layout overrides.

## Numerical figures

Use actual MATLAB output for numerical results. Put plots and MAT data in
`data/results/`, alongside the reproduction script, source commit IDs,
parameters, solver versions, random seed, and validation outcomes. Export PNG
for inline reading and SVG for enlargement. Label conceptual diagrams as
schematics. Do not describe sampled tests as a proof for all disturbances.

The current SeDuMi substitution is documented in `data/results/README-results.txt`.
The stock AROC terminal-region implementation selects MOSEK internally. Merely
adding another solver to the MATLAB path does not change that selection.
The expanded record at `pages/reproduction/index.html` combines three isolated
execution groups. Its first six controller examples used the initial SeDuMi
environment; terminal-region and remaining cases used the renewed MOSEK
environment. Preserve each case's execution group and diagnostic warnings.
The separate MOSEK page repeats the terminal subset; do not count it twice.

## Examples

Source downloads and readable HTML listings live in `data/examples/`.
Use real current-release file names; keep the benchmark description and source
listing consistent. A source listing does not imply that its code was rerun.
Give iframes descriptive titles. Regenerate a source view when its code changes.

## Publications

The visible list is static HTML in `pages/publications/index.html`. Add only
verified references. Store a valid BibTeX file in `data/bib/`; link it together
with an available PDF and publisher/DOI page. Each paper needs one BibTeX entry.
Do not restore the old demonstration entries or Max Mustermann examples.

## Videos

Use the existing `.video` class for responsive 16:9 embeds. Include a unique
iframe title, `loading="lazy"`, and a direct watching link. Do not autoplay
sound or use a fixed 560-by-520 frame. The recorded MPC playback is implemented
in `scripts/mpc-playback.js` with data in `data/results/linear-mpc-playback.json`.

## Current release and archive

`manual/index.html` points to the current PDF, with a readable fallback link.
Historical pages belong in `pages/archive/v20XX/index.html`. Historical manuals
belong in `data/archive/manual/`; existing historical downloads are in `data/`.
Update links in the homepage, navbar, manual redirect, and archive together.
Use release documentation for release summaries. Git commit history is a
development log, not automatically a set of release notes.

## Before publishing

Check local paths and case-sensitive file names, anchor targets, image alt text,
iframe titles, desktop/mobile layouts, and the currently offered download links.
Ensure there are no placeholder publications, descriptions, or fixed CI badges.
Preserve the numerical execution records and clearly state what was actually run.

## Recorded animations

The homepage animation selector covers the 18 executed examples. Five compact
previews accompany the feature overview, and the method pages have their own
player. `scripts/result-animation.js` provides play/pause, reset, seek, speed,
and loop controls. It pauses outside the viewport or in a hidden tab and starts
paused when the visitor requests reduced motion. Static MATLAB plots remain
available if JavaScript or data loading fails.

Keep source data in `data/animations/<case-id>.json`. Include actual simulation
timestamps, projection dimensions, axis units, geometry notes, and the source
case ID. Never infer uniform times for nonuniform MPC updates. If set times
are unavailable, show those sets statically and animate only the recorded
trajectory. A result-comparison sequence must not be labelled as convergence
or physical time. Fitting-data playback must retain its validation limitation.

After numerical integration, run `tools/refine_animations.py`, then
`tools/render_animation_assets.py`. The first connects the players and refreshes
asset hashes; the second produces the standalone SVG downloads. The finalizer
does this automatically when animation data exist. `research/export_website_animations.m`
exports saved results without rerunning algorithms; see the bundle's
`ANIMATIONS.txt`. Recheck the links and run `node tools/test_result_animation.cjs`
after changing the player. A DOM/event unit test does not replace a browser
layout check on desktop and mobile.

The 2026-11-02 browser review covers 1440, 390 and 360 pixel widths. Keep
`styles/cora-theme.css` last in the stylesheet list; `apply_cora_theme.py`
handles this after animation integration. Preserve the fixed poster aspect
ratio and immediate anchor navigation: lazy-loading figures must not displace
the requested anchor. The navigation collapse has no transition so a quick
link tap closes it reliably. Long MathJax expressions scroll within their
own container rather than widening the page. Data notes stay available on
method pages, and playback controls precede the plot on every screen size.

For a local preview, serve the website folder over HTTP, for example:
`python -m http.server 8766 --bind 127.0.0.1 --directory website`.
Opening HTML directly with `file://` may block the JSON fetches. The standalone
animated SVG downloads do not need a server.
