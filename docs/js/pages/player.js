// player.html: one music engine for the whole visit; the pages load in the frame,
// so that moving between them does not interrupt the music. The pages reach the
// engine, the theme, and the address bar through messages (lib/bridge.js).
import { sandboxed } from '../lib/bridge.js';
import { PAGE_ADDRESS, servePage } from '../lib/frame.js';
import { createMusicEngine, serveMusic } from '../lib/music.js';
import { initTheme, serveTheme } from '../lib/theme.js';

const requested = new URLSearchParams(location.search).get('p') || '';
// Only the pages of this site are loaded in the frame.
const page = PAGE_ADDRESS.test(requested) ? requested : 'index.html';

if (sandboxed()) {
    // The host refuses to show the page in a frame: open the page itself.
    location.replace(page);
} else {
    initTheme();
    const frame = document.getElementById('app');
    serveMusic(frame, createMusicEngine());
    serveTheme(frame);
    servePage(frame);
    frame.src = page;
    frame.addEventListener('load', () => { try { frame.contentWindow.focus(); } catch { /* ignore */ } });
}
