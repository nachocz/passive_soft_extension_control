// Equations with KaTeX (vendored, offline), in the notation of the paper.

export const MACROS = {
    '\\R': '\\mathbb{R}',
    '\\q': '\\bm q', '\\qd': '\\dot{\\bm q}', '\\qdc': '\\dot{\\bm q}_c', '\\qnom': '\\dot{\\bm q}_{\\mathrm{nom}}',
    '\\e': '\\bm e', '\\ec': '\\bm e_c', '\\tauv': '\\bm\\tau', '\\zv': '\\bm z', '\\nuv': '\\bm\\nu',
    '\\lo': '\\ell^o', '\\blo': '\\bm\\ell^o', '\\Lspan': '\\mathcal L',
    '\\Jlq': 'J_{\\ell q}', '\\Jlz': 'J_{\\ell z}', '\\Jzt': 'J_{z\\tau}', '\\JDt': 'J_{D\\tau}',
    '\\Jspan': 'J_{\\mathrm{span}}', '\\Jspanh': '\\widehat J_{\\mathrm{span}}', '\\Jtr': 'J_{\\mathrm{transport}}',
    '\\Jdef': 'J_{\\mathrm{def}}', '\\JDh': '\\widehat J_D', '\\Rbar': '\\bar R_B', '\\Bb': '{}^{B}b',
    '\\VDB': '{}^{B}\\mathcal V_{D/B}', '\\vres': 'v_{\\mathrm{res}}',
    '\\hobs': 'h_{\\mathrm{obs}}', '\\hap': 'h_{\\mathrm{ap}}', '\\hcone': 'h_{\\mathrm{cone}}', '\\hins': 'h_{\\mathrm{ins}}',
    '\\hspan': 'h_{\\mathrm{span}}', '\\rap': 'r_{\\mathrm{ap}}',
    '\\rank': '\\operatorname{rank}', '\\range': '\\operatorname{range}', '\\diag': '\\operatorname{diag}',
    '\\dist': '\\operatorname{dist}', '\\T': '\\mathsf T',
};

export function tex(source, display = false) {
    return window.katex.renderToString(source, {
        displayMode: display, macros: { ...MACROS }, throwOnError: false, strict: 'ignore',
    });
}

// Render every <span class="tex"> (inline) and <div class="tex-block"> (display) below root, once.
export function renderMath(root = document) {
    for (const el of root.querySelectorAll('.tex, .tex-block')) {
        if (el.dataset.rendered) continue;
        const ref = el.dataset.eq ? `<span class="eq-ref">(${el.dataset.eq})</span>` : '';
        el.innerHTML = tex(el.textContent, el.classList.contains('tex-block')) + ref;
        el.dataset.rendered = '1';
    }
}
