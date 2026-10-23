// Light / dark theme, and the colour palette read from the CSS variables, so that
// canvases, the 3D view, and HTML share one set of colours. The choice is saved
// in localStorage and, inside player.html, also kept by the player, which passes
// it to each page. Sandboxed hosts give neither; there, the links between the
// pages carry it (?theme=, js/lib/shell.js).
import { fromPage, fromPlayer, inFrame, toPage, toPlayer } from './bridge.js';

const KEY = 'softext-theme';
const listeners = new Set();
let cached = null;

function setTheme(theme) {
    if ((theme !== 'dark' && theme !== 'light') || document.documentElement.dataset.theme === theme) return;
    document.documentElement.dataset.theme = theme;
    cached = null;
    listeners.forEach((fn) => fn());
}

export function initTheme() {
    let saved = new URLSearchParams(location.search).get('theme');
    if (saved !== 'dark' && saved !== 'light') {
        saved = null;
        try { saved = localStorage.getItem(KEY); } catch { /* storage unavailable */ }
    }
    const system = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    document.documentElement.dataset.theme = saved || system;
    cached = null;
    if (inFrame()) {
        fromPlayer('theme', ({ theme }) => setTheme(theme));
        toPlayer('theme-hello');
    }
}

export function toggleTheme() {
    const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
    setTheme(next);
    try { localStorage.setItem(KEY, next); } catch { /* storage unavailable */ }
    toPlayer('theme', { theme: next });
}

// Player side: keep the theme of the pages loaded in `frame`.
export function serveTheme(frame) {
    fromPage(frame, 'theme', ({ theme }) => setTheme(theme));
    fromPage(frame, 'theme-hello', () => toPage(frame, 'theme', { theme: document.documentElement.dataset.theme }));
}

export function onThemeChange(fn) { listeners.add(fn); }

const NAMES = ['bg', 'surface', 'surface-soft', 'ink', 'ink-soft', 'muted', 'line', 'line-strong', 'accent', 'accent-strong',
    'accent-soft', 'canvas', 'grid', 'grid-strong', 'scene', 'arm', 'joint', 'rod', 'tendon1', 'tendon2', 'path', 'target',
    'obstacle', 'fulcrum', 'reference', 'axis-x', 'axis-y', 'axis-z', 'exact', 'identified', 'band', 'j1', 'j2', 'j3', 'j4',
    'j5', 'j6', 'good', 'bad', 'warn'];

// palette().roi, palette().dCol, ... (CSS names in camelCase)
export function palette() {
    if (cached) return cached;
    const style = getComputedStyle(document.documentElement);
    const p = { dark: document.documentElement.dataset.theme === 'dark' };
    for (const name of NAMES) p[name.replace(/-([a-z])/g, (_, c) => c.toUpperCase())] = style.getPropertyValue(`--${name}`).trim();
    cached = p;
    return p;
}

const toRgb = (hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
export function alpha(color, a) {
    if (!color.startsWith('#')) return color;
    const [r, g, b] = toRgb(color);
    return `rgba(${r}, ${g}, ${b}, ${a})`;
}
export function mix(a, b, t) {
    const x = toRgb(a), y = toRgb(b);
    return '#' + x.map((v, i) => Math.round(v + (y[i] - v) * t).toString(16).padStart(2, '0')).join('');
}
