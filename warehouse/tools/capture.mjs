// Renders the guided tour to an MP4, frame by frame and deterministic, including every HTML
// overlay (captions, labels, panels), by driving headless Chrome / Edge over the DevTools
// protocol and piping screenshots into ffmpeg. No extra packages: Node's built-in WebSocket.
//   node warehouse/tools/capture.mjs [fps=30] [WxH=1920x1080] [seconds=all] [url]
// Default url: the offline file warehouse/dist/Warehouse_Tour.html (proves it works from file://).

import { spawn } from 'node:child_process';
import { existsSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const ROOT = resolve(fileURLToPath(new URL('../../', import.meta.url)));
const FPS = +(process.argv[2] ?? 30), [W, H] = (process.argv[3] ?? '1920x1080').split('x').map(Number), LIMIT = process.argv[4] ? +process.argv[4] : Infinity;
const page = process.argv[5] ?? pathToFileURL(join(ROOT, 'warehouse/dist/Warehouse_Tour.html')).href;
const out = join(ROOT, 'warehouse/dist', LIMIT < Infinity ? 'Warehouse_Tour_test.mp4' : 'Warehouse_Tour.mp4');
const BROWSERS = ['C:/Program Files/Google/Chrome/Application/chrome.exe', 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe', 'C:/Program Files/Microsoft/Edge/Application/msedge.exe'];
const exe = BROWSERS.find(existsSync);
if (!exe) throw new Error('no Chrome or Edge found');
const PORT = 9333;
const chrome = spawn(exe, ['--headless=new', `--remote-debugging-port=${PORT}`, `--user-data-dir=${mkdtempSync(join(tmpdir(), 'whcap-'))}`, `--window-size=${W},${H}`, '--hide-scrollbars', '--ignore-gpu-blocklist', '--enable-webgl', '--allow-file-access-from-files', 'about:blank'], { stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

let ver;
for (let i = 0; i < 60 && !ver; i++) { try { ver = await (await fetch(`http://127.0.0.1:${PORT}/json/version`)).json(); } catch { await sleep(250); } }
const target = await (await fetch(`http://127.0.0.1:${PORT}/json/new?about:blank`, { method: 'PUT' })).json();
const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((r) => (ws.onopen = r));
let seq = 0;
const pending = new Map();
ws.onmessage = (m) => { const d = JSON.parse(m.data); if (d.id && pending.has(d.id)) { pending.get(d.id)(d); pending.delete(d.id); } };
const send = (method, params = {}) => new Promise((r) => { const id = ++seq; pending.set(id, r); ws.send(JSON.stringify({ id, method, params })); });
const evaluate = async (expr) => { const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true }); if (r.result?.exceptionDetails) throw new Error(JSON.stringify(r.result.exceptionDetails)); return r.result?.result?.value; };

await send('Emulation.setDeviceMetricsOverride', { width: W, height: H, deviceScaleFactor: 1, mobile: false });
await send('Page.enable');
await send('Runtime.enable');
const url = page + (page.includes('?') ? '&' : '?') + `capture=${W}x${H}`;
await send('Page.navigate', { url });
for (let i = 0; i < 240; i++) { if (await evaluate('!!window.__ready').catch(() => false)) break; await sleep(500); }
if (!(await evaluate('!!window.__ready'))) throw new Error('page did not become ready: ' + url);
const end = Math.min(LIMIT, await evaluate('window.__tour.end')), N = Math.ceil(end * FPS);
console.log(`${exe.split('/').pop()} · ${url.slice(0, 80)}… · ${N} frames at ${FPS} fps, ${W}x${H}`);

// stills mode: STILLS=10,25.5,40 node capture.mjs 30 1280x720 -> warehouse/capture/still_<v>.jpg
if (process.env.STILLS) {
  const { mkdirSync, writeFileSync } = await import('node:fs');
  const dir = join(ROOT, 'warehouse/capture');
  mkdirSync(dir, { recursive: true });
  const list = process.env.STILLS === 'steps' ? (await evaluate('window.__tour.steps.map((s) => s.v0 + (s.v1 - s.v0) * 0.6)')) : process.env.STILLS.split(',').map(Number);
  for (const v of list) {
    await evaluate(`(window.__seek(${v}), new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))))`);
    const shot = await send('Page.captureScreenshot', { format: 'jpeg', quality: 85 });
    writeFileSync(join(dir, `still_${v.toFixed(1)}.jpg`), Buffer.from(shot.result.data, 'base64'));
  }
  console.log(JSON.stringify(await evaluate('window.__tour.steps.map((s) => [s.n, s.v0, s.caption.slice(0, 60)])')));
  ws.close(); chrome.kill(); process.exit(0);
}
const ff = spawn('ffmpeg', ['-y', '-loglevel', 'error', '-f', 'image2pipe', '-framerate', String(FPS), '-c:v', 'mjpeg', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', '-preset', 'medium', '-movflags', '+faststart', out], { stdio: ['pipe', 'inherit', 'inherit'] });
const t0 = Date.now();
for (let i = 0; i < N; i++) {
  await evaluate(`(window.__seek(${(i / FPS).toFixed(4)}), new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))))`);
  const shot = await send('Page.captureScreenshot', { format: 'jpeg', quality: 92 });
  const buf = Buffer.from(shot.result.data, 'base64');
  if (!ff.stdin.write(buf)) await new Promise((r) => ff.stdin.once('drain', r));
  if (i % (FPS * 10) === 0) console.log(`  ${(i / FPS).toFixed(0)} s / ${end.toFixed(0)} s · ${((Date.now() - t0) / 1000).toFixed(0)} s elapsed`);
}
ff.stdin.end();
await new Promise((r) => ff.on('close', r));
ws.close();
chrome.kill();
console.log(`${out} · ${N} frames · ${((Date.now() - t0) / 1000).toFixed(0)} s`);
process.exit(0);
