// Packs the warehouse into single HTML files that open by double-click, with no server and no
// internet: every ES module (and three.js r160 from warehouse/vendor) is inlined as a data: URL
// in an import map; the recording is inlined as base64.   node warehouse/tools/build-offline.mjs
// Output: warehouse/dist/Warehouse_Tour.html (guided presentation) and
//         warehouse/dist/Warehouse_Live.html (live engineering simulation)

import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = resolve(fileURLToPath(new URL('../../', import.meta.url)));
const VENDOR = join(ROOT, 'warehouse/vendor');
const dataUrl = (code) => 'data:text/javascript;base64,' + Buffer.from(code, 'utf8').toString('base64');
const IMPORT = /(\bimport\s*(?:[^'"()]*?\s*from\s*)?|\bexport\s*[^'"()]*?\s*from\s*)(['"])([^'"]+)\2/g;

// Every module reachable from `entry`, with relative imports rewritten to bare "@wh/<path>" names.
function graph(entry) {
  const map = {}, queue = [resolve(ROOT, entry)];
  while (queue.length) {
    const file = queue.shift(), key = '@wh/' + relative(ROOT, file).replace(/\\/g, '/');
    if (map[key]) continue;
    const code = readFileSync(file, 'utf8').replace(IMPORT, (m, pre, q, spec) => {
      if (!spec.startsWith('.')) return m; // 'three', 'three/addons/…'
      const dep = resolve(dirname(file), spec);
      queue.push(dep);
      return `${pre}${q}@wh/${relative(ROOT, dep).replace(/\\/g, '/')}${q}`;
    });
    map[key] = dataUrl(code);
  }
  return map;
}
function build(htmlFile, entry, out, extraHead = '') {
  const imports = {
    three: dataUrl(readFileSync(join(VENDOR, 'three.module.min.js'), 'utf8')),
    'three/addons/controls/OrbitControls.js': dataUrl(readFileSync(join(VENDOR, 'OrbitControls.js'), 'utf8')),
    ...graph(entry),
  };
  let html = readFileSync(join(ROOT, htmlFile), 'utf8');
  html = html.replace(/<script type="importmap">[\s\S]*?<\/script>/, () => `<script type="importmap">${JSON.stringify({ imports })}</script>`);
  html = html.replace(/<script type="module" src="[^"]+"><\/script>/, () => `${extraHead}<script type="module">import '@wh/${entry}';</script>`);
  html = html.replace('href="present.html"', 'href="Warehouse_Tour.html"').replace('href="index.html"', 'href="Warehouse_Live.html"');
  writeFileSync(out, html);
  return html.length;
}

const dist = join(ROOT, 'warehouse/dist');
mkdirSync(dist, { recursive: true });
const recFile = join(ROOT, 'warehouse/rec/clip.json.gz');
if (!existsSync(recFile)) throw new Error('no recording: run node warehouse/tools/record.mjs first');
const rec = readFileSync(recFile).toString('base64');
const a = build('warehouse/present.html', 'warehouse/js/present.js', join(dist, 'Warehouse_Tour.html'), `<script>window.__REC = "${rec}";</script>`);
const b = build('warehouse/index.html', 'warehouse/js/main.js', join(dist, 'Warehouse_Live.html'));
console.log(`Warehouse_Tour.html ${(a / 1e6).toFixed(1)} MB · Warehouse_Live.html ${(b / 1e6).toFixed(1)} MB  ->  ${dist}`);
