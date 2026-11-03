# AROC website refresh

This checkout contains the local refresh of `WeiChiHuang/AROC-gh-pages-modi`,
based on commit `33e8493d46de2154ca56572eb7bcd61a604d4e61`.
It has not been published to the official AROC website.

## GitHub Pages deployment

The deployment target is `WeiChiHuang/AROC-gh-pages-modi`, with GitHub Pages
configured to publish `main` from `/` (the repository root). The preview URL is
<https://weichihuang.github.io/AROC-gh-pages-modi/>. Updating that preview
requires write access to the repository and a successful Pages deployment.

This is a static website: commit the contents of this checkout directly.
`.nojekyll` disables Jekyll processing so MATLAB code listings and generated
assets are served unchanged. No framework build or MATLAB installation is
required for hosting or browser playback. Keep relative asset paths intact so
the site works under the repository's `/AROC-gh-pages-modi/` URL prefix.
The authoring commands referenced later are run from the local parent
workspace; the generated website is independently deployable.

## Preview

From this directory run:

```powershell
python -m http.server 8766 --bind 127.0.0.1
```

Open <http://127.0.0.1:8766/>. A local HTTP server is needed for the optional
JSON-backed playback; the static text and figures also work without it.
The site uses Bootstrap from jsDelivr, MathJax on mathematical pages, and
YouTube embeds; these features need an Internet connection.

## Delivery archive

The full delivery ZIP separates `AROC-website/` (the deployable website),
`preview/` (screenshots), and `verification/` (reference checks and hashes).
Use the contents of `AROC-website/` as the website repository root, preserving
the `pages`, `data`, `styles`, and `scripts` directories. Preview the site
over HTTP before using the repository's existing publication workflow.
The downloaded AROC example/test sources retain their original GPL license;
the source license is included beside the numerical results and runners.

## What changed

- Reorganized the homepage around five capabilities, six motion-primitive
  methods, installation, reproduced examples, and videos.
- Added a missing safety-net page and two labelled workflow schematics.
- Replaced placeholder descriptions, corrected feature titles, and matched
  example source files to their benchmark descriptions.
- Updated the current manual and download references to AROC 2026.
- Removed demonstration publications and CORA-specific archive placeholders.
  Algorithm papers are now rendered as static HTML, without a CDN BibTeX parser.
- Unified navigation; corrected mobile menu behavior, anchor offsets, image
  dimensions, video proportions, and content visibility without animations.
- Added numerical results and a controlled playback of actual MPC output.

## Numerical results

See `data/results/README-results.txt` and `run-report.json` for the exact source
commits, environment, parameters, solver substitution, and validation limits.

1. Linear MPC for the double integrator: input and terminal-set checks pass.
   Uses the terminal polytope from the upstream unit test; real-time deadlines
   are disabled for this offline export.
2. Fixed zonotopic terminal region: ten sampled extreme-disturbance trajectories
   pass state/input and initial/final-set assertions.
3. Conformant synthesis for a four-vehicle platoon: three supplied measurement
   trajectories are used for fitting; 101/101 matching time points on the first
   trajectory pass full-state containment. This is not held-out validation.

PNG, SVG, MAT, and portable MATLAB reproduction scripts accompany the results.
The MPC playback draws recorded states; it does not run a new controller in JS.
The separate terminal-region result is not the terminal polytope used for MPC.

## Verification

Local file/anchor references were checked across the homepage, detail pages,
manual redirect, and source-code views. Desktop and 390-pixel mobile layouts,
mobile dropdown navigation, playback start/reset/keyboard seek, and the
publication list were inspected in the browser.

## Further scientific results

The expanded run tracks 18 representative examples across all 11 algorithm
families and the 20 script-based sections selected by upstream `runUnitTests`.
Nested function-style component tests are outside this record. See
`pages/reproduction/index.html` and `data/results/full-suite/manifest.json`
for current per-case outcomes; pending or running entries are unfinished.
The completed 2026-10-19 record has 18 executed examples and 20 passed test
sections. The SafetyNet MPC section passed on a targeted fresh-process retry
after a generated-function search-path repair in the runner. Its original
failed log, status, and retry explanation are retained in that case's record.
That original run retained algorithm constraints, sample counts, iteration
limits and assertions. Subsequent warning-resolution reruns are identified
separately in the execution record, with their patches, settings and validation.
Earlier warning-producing logs and sources remain downloadable.
This coverage does not include every benchmark script in the repository.
The unique-case record combines three isolated execution groups: the first
six controller examples from the initial process, six terminal-region
examples plus four tests from the licensed terminal group, and the remaining
six examples plus sixteen tests from the licensed remaining group. Each row
and the manifest identify its actual execution group; concurrent wall times
are not a controlled performance comparison.
CasADi 3.7.2/IPOPT is available. The initial MOSEK license returned
LICENSE_EXPIRED (1001), so SeDuMi 1.3 was available in the first process.
Its checkout's local solver selections are preserved in the initial patch.
A renewed license subsequently passed a fresh-process MOSEK solve; the
licensed checkouts retain upstream solver selections. The terminal subset is
also shown at
`pages/reproduction-mosek/index.html` and `data/results/mosek-suite/manifest.json`.
The terminal page repeats that subset of the combined record; it is not an
additional independent validation. Check unfinished and failed entries before
interpreting results. The native run's
`data/results/mosek-suite/reproduce.zip` includes its runner and setup.
`data/results/full-suite/reproduce.zip` contains the runner, exact dependency
versions, setup instructions and patch. The three initial demonstrations
above retain their separate configurations and validation records.
The current archive summaries defer to the historical manuals rather than
inventing release notes. Solver/runtime measurements are not advertised as
cross-algorithm performance comparisons.

The 2026-11-02 CORA warning repair reran the five affected terminal cases in
isolated copies. Both test sections passed; the three examples completed.
The loop now recognizes states unreachable from nonzero abstraction errors,
whose correct asymptotic order is infinite. All 104 regression fixtures retain
the same orders, and four saved terminal-region set matrices exactly match
the earlier run. The patch, CORA license and validation JSON are linked under
“Diagnostic warning fixes”. The remaining three optimizer records were replaced
on 2026-11-03 after positive solver termination and the original checks. The
linear reference uses GPS; nonlinear records use central-difference active-set
optimization of supplied logarithmic LQR weights. The original objective, weight
box, physical constraints, horizons and sampled checks are retained. Nonlinear
function, step and constraint tolerances are 1e-6; termination uses the predicted
objective-change criterion, which does not establish differentiable stationarity
or a global optimum. Exact initial/final weights, stopping diagnostics, source
patches and earlier attempts remain downloadable. The linear reference was run
on 2026-11-02; the combined record update does not count it as an extra test.
Current recorded cases have zero tracked diagnostic warnings. Case wall times
exclude prior warm-start searches and shared setup; they are not speed rankings.

## Maintenance

The animation selector at `index.html#animations` covers all 18 recorded
examples; 13 method pages and five homepage feature previews use the same
player. Play/pause, reset, seek, speed and loop controls operate on saved
data. Animation playback does not run a controller or solver in the browser.
Animations start and loop automatically on entering view, including after
scrolling back or returning to a visible tab. Choosing a new method also starts
it automatically. Explicit pause, reset and seeking remain under user control;
the system's reduced-motion preference is respected until Play is requested.
Standalone animated SVG downloads are included for offline viewing. Serve the
website over HTTP for interactive playback; opening HTML with `file://` can
block JSON loading. The original numerical record and its diagnostic warnings
remain available. Traffic planning uses four discrete snapshots; the cart
terminal-region comparison uses display order rather than physical time.
Content revisions on animation, result and record links prevent browsers from
combining new validation text with cached old data. The updated car playback is
checked against its saved MAT trajectory samples before publication.

The 2026-11-02 review also checked the actual browser at 1440×900, 390×844,
and 360×800 viewports. All 18 animations were switched and scrubbed on desktop
and mobile; 13 method pages were checked at both sizes. Navigation, playback,
pause, reset, speed, looping, data notes, long formulas and support-page overflow
were inspected. These are responsive browser checks, not physical handset tests.
The report and four actual screenshots are included in the website archive.
A targeted 2026-11-03 browser check at 1440×900 and 390×844 verified the
updated zero-warning record, source links, car autoplay and mobile overflow;
its report and four new screenshots are retained separately.

`styles/cora-theme.css` aligns the visual language with the CORA website:
Newsreader headings, Bootstrap body text, dark navigation, white figure panels
and thin separators. The animation build reapplies `tools/apply_cora_theme.py`
after integration so the shared theme and canonical introduction are preserved.
The homepage keeps two matching quick links for getting started and the
manual. One compact link groups the algorithm, example and test counts above
the left-aligned introduction; the animation navigation remains available.

See `docs/maintenance.md` for adding a feature, example, publication, video, or
archived release. Keep the documentation and reproducibility records together
with any replacement numerical figure.
