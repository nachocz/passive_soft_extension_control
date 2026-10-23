// Small DOM toolkit: element builder, controls, readouts, cards.
import { renderMath } from './math.js';

// h('div', {class: 'card', onclick: fn}, child, 'text', ...)
export function h(tag, props = {}, ...children) {
    const el = document.createElement(tag);
    for (const [key, value] of Object.entries(props || {})) {
        if (value === undefined || value === null || value === false) continue;
        if (key === 'class') el.className = value;
        else if (key === 'style') el.style.cssText = value;
        else if (key === 'html') el.innerHTML = value;
        else if (key.startsWith('on')) el.addEventListener(key.slice(2), value);
        else el.setAttribute(key, value === true ? '' : value);
    }
    for (const child of children.flat()) {
        if (child === null || child === undefined || child === false) continue;
        el.append(child instanceof Node ? child : document.createTextNode(String(child)));
    }
    return el;
}

// Inline maths inside labels: 'gain $k_I$' -> text + KaTeX span.
export function label(text) {
    const span = h('span');
    String(text).split(/(\$[^$]+\$)/).forEach((part) => {
        if (part.startsWith('$')) span.append(h('span', { class: 'tex' }, part.slice(1, -1)));
        else if (part) span.append(part);
    });
    renderMath(span);
    return span;
}

export function card(title, note = '') {
    const noteEl = h('span', { class: 'card-note' }, String(note).includes('$') ? label(note) : note);
    const body = h('div');
    const heading = title ? h('div', { class: 'card-heading' }, h('h3', {}, label(title)), noteEl) : null;
    const el = h('section', { class: 'card' }, heading, body);
    return { el, body, note: noteEl, heading };
}

export function button(text, onClick, { primary = false, small = false, title } = {}) {
    const cls = ['button', primary && 'primary', small && 'small'].filter(Boolean).join(' ');
    const el = h('button', { class: cls, type: 'button', title, onclick: onClick });
    el.append(label(text));
    return el;
}

// slider({label: 'gain $k_I$', min, max, step, value, format, log, onInput})
export function slider({ label: text, description = '', min, max, step = 0.01, value, format = (v) => v.toFixed(2),
    log = false, onInput, onChange }) {
    const toInput = (v) => (log ? Math.log10(v) : v);
    const fromInput = (v) => (log ? 10 ** v : v);
    const input = h('input', { type: 'range', min: toInput(min), max: toInput(max), step: log ? 0.001 : step, value: toInput(value) });
    const output = h('output', {}, format(value));
    const el = h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, label(text)), output),
        description ? h('p', { class: 'control-help' }, label(description)) : null, input);
    const get = () => fromInput(Number(input.value));
    input.addEventListener('input', () => { output.textContent = format(get()); onInput?.(get()); });
    input.addEventListener('change', () => onChange?.(get()));
    return {
        el, get, input,
        set(v) { input.value = toInput(v); output.textContent = format(v); },
        disable(flag) { input.disabled = flag; },
    };
}

// segmented({options: [{value, label}], value, onChange})
export function segmented({ options, value, onChange }) {
    const el = h('div', { class: 'segmented', role: 'group' });
    let current = value;
    const buttons = options.map((opt) => {
        const b = h('button', { type: 'button', 'aria-pressed': String(opt.value === value), title: opt.title });
        b.append(label(opt.label));
        b.addEventListener('click', () => { set(opt.value); onChange?.(opt.value); });
        el.append(b);
        return b;
    });
    function set(v) { current = v; buttons.forEach((b, i) => b.setAttribute('aria-pressed', String(options[i].value === v))); }
    return { el, set, get: () => current };
}

export function select({ options, value, onChange }) {
    const el = h('select', {}, options.map((opt) => (opt.group
        ? h('optgroup', { label: opt.group }, opt.options.map((o) => h('option', { value: o.value }, o.label)))
        : h('option', { value: opt.value }, opt.label))));
    el.value = value;
    el.addEventListener('change', () => onChange?.(el.value));
    return { el, get: () => el.value, set(v) { el.value = v; } };
}

export function toggle({ label: text, value = false, onChange, title }) {
    const input = h('input', { type: 'checkbox' });
    input.checked = value;
    input.addEventListener('change', () => onChange?.(input.checked));
    const el = h('label', { class: 'switch', title }, input, label(text));
    return { el, get: () => input.checked, set(v) { input.checked = v; } };
}

export function control(text, child, description = '') {
    return h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, label(text))),
        description ? h('p', { class: 'control-help' }, label(description)) : null, child);
}

// readouts([{key, label}]) -> {el, set(key, text, tone)}
export function readouts(defs) {
    const values = {};
    const el = h('div', { class: 'readouts' }, defs.map((d) => {
        values[d.key] = h('strong', {}, d.value ?? '-');
        return h('div', { class: 'readout', title: d.title }, h('span', {}, label(d.label)), values[d.key]);
    }));
    return { el, set(key, text, tone = '') { values[key].textContent = text; values[key].className = tone; } };
}

// items: [{label, color, kind: 'line' | 'dashed' | 'dotted' | 'dot' | 'band'}]
export function legend(items) {
    return h('div', { class: 'legend' }, items.map((it) => {
        const item = h('span', { class: 'legend-item' }, h('i', { class: `swatch ${it.kind === 'line' ? '' : it.kind || ''}` }), label(it.label));
        item.firstChild.style.color = it.color;
        return item;
    }));
}

export function seriesLegend(items) {
    return h('div', { class: 'series-legend' }, items.map((it) => {
        const span = h('span', {}, h('i', { class: it.dash || '' }), label(it.label));
        span.firstChild.style.color = it.color;
        return span;
    }));
}

export function toast(message) {
    const el = h('div', { class: 'toast', role: 'alert' }, message);
    document.body.append(el);
    setTimeout(() => el.remove(), 6000);
}

export const fmt = (v, digits = 2) => (v === null || v === undefined || !Number.isFinite(v) ? '–' : v.toFixed(digits));
export const clock = (t) => `${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, '0')}`;
