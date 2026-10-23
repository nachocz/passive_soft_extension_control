// The pages run inside player.html, which keeps the background music playing
// while the pages change. A page opened on its own is moved into the player
// (append ?standalone to the address to prevent this); inside the player, the
// page keeps the address bar and the tab title in step with what is shown. On
// sandboxed hosts the pages run on their own (see bridge.js).
import { fromPage, inFrame, sandboxed, toPlayer } from './bridge.js';

const PLAYER = 'player.html';
export const PAGES = ['index.html', 'model.html', 'control.html', 'analysis.html'];
// Addresses that the player loads in its frame: only the pages of this site.
export const PAGE_ADDRESS = /^(index|model|control|analysis)\.html(\?[\w=&.-]*)?(#[\w-]*)?$/;

function currentPage() {
    const page = location.pathname.split('/').pop() || 'index.html';
    const params = new URLSearchParams(location.search);
    params.delete('standalone');
    const query = params.toString();
    return page + (query ? `?${query}` : '') + location.hash;
}

// True if the page is being replaced by the player (stop initializing).
export function enterPlayer() {
    if (inFrame() || sandboxed()) return false;
    if (new URLSearchParams(location.search).has('standalone')) return false;
    location.replace(`${PLAYER}?p=${encodeURIComponent(currentPage())}`);
    return true;
}

// Inside the player: send this page's address and title to the player.
export function syncPlayer() {
    toPlayer('page', { page: currentPage(), title: document.title });
}

// Player side: show the address and title of the page in the frame.
export function servePage(frame) {
    fromPage(frame, 'page', ({ page, title }) => {
        if (typeof page !== 'string' || !PAGE_ADDRESS.test(page)) return;
        if (typeof title === 'string') document.title = title;
        try {
            history.replaceState(null, '', page.startsWith('index.html') ? PLAYER : `${PLAYER}?p=${encodeURIComponent(page)}`);
        } catch { /* not allowed in a sandboxed document */ }
    });
}
