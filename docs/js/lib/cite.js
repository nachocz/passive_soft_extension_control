// Citation dialog. Author names and publication details are placeholders until
// the article appears.
import { h } from './ui.js';

const TITLE = 'Actuation and Control of Passive Tendon-Driven Soft Extensions Through Rigid Manipulator Motion';

const CITATION = `<Authors>, "${TITLE}," IEEE Transactions on Robotics, <volume>(<number>), <pages>, <year>. doi: <doi>.`;
const BIBTEX = [
    '@article{softext,',
    '  author  = {<Author 1> and <Author 2>},',
    `  title   = {${TITLE}},`,
    '  journal = {IEEE Transactions on Robotics},',
    '  year    = {<year>},',
    '  volume  = {<volume>},',
    '  number  = {<number>},',
    '  pages   = {<pages>},',
    '  doi     = {<doi>},',
    '}',
].join('\n');

async function copy(text, button) {
    try {
        await navigator.clipboard.writeText(text);
    } catch {
        const area = h('textarea', { style: 'position:fixed;opacity:0' });
        area.value = text;
        document.body.append(area);
        area.select();
        document.execCommand('copy');
        area.remove();
    }
    button.textContent = 'Copied ✓';
    setTimeout(() => { button.textContent = 'Copy'; }, 1400);
}

export function citeDialog() {
    const block = (title, text) => {
        const copyButton = h('button', { class: 'button small', type: 'button' }, 'Copy');
        copyButton.addEventListener('click', () => copy(text, copyButton));
        return h('div', { class: 'cite-block' }, h('div', { class: 'cite-block-head' }, h('span', {}, title), copyButton), h('pre', {}, text));
    };
    const close = h('button', { class: 'button', type: 'button' }, 'Close');
    const dialog = h('dialog', { class: 'cite-dialog', 'aria-labelledby': 'citeTitle' },
        h('p', { class: 'kicker' }, 'Citation'),
        h('h2', { id: 'citeTitle' }, 'Cite the paper'),
        h('p', { class: 'cite-note' }, 'Author names and publication details are placeholders until the article appears.'),
        block('Reference', CITATION),
        block('BibTeX', BIBTEX),
        h('div', { class: 'cite-actions' }, close));
    close.addEventListener('click', () => dialog.close());
    dialog.addEventListener('click', (event) => {           // a click on the backdrop closes
        const r = dialog.getBoundingClientRect();
        if (event.clientX < r.left || event.clientX > r.right || event.clientY < r.top || event.clientY > r.bottom) dialog.close();
    });
    dialog.addEventListener('close', () => dialog.remove());
    document.body.append(dialog);
    dialog.showModal();
}
