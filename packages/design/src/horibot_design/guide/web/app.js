// 작업 안내 화면 (0036). 데이터는 data/(만들기가 쓴다)에서 읽고, 확인 칸과 메모만 아티팩트 db에 쓴다. 단계를
// 눌러 보기만 해서는 아무것도 쓰지 않는다: 끝낸 단계, 진행, 이어서 할 단계는 모두 확인 칸에서 셈한다.
// db가 없으면(PC의 파일로 열거나 시험할 때) 저장만 빠지고 화면은 그대로 돈다. #selftest로 열면 모든 안내의 모든
// 단계를 넘겨 보고 결과를 페이지 끝의 selftest-report 요소에 적는다 (pytest가 읽는다).
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const app = $('app');
  const PHONE = window.matchMedia ? window.matchMedia('(max-width: 760px)') : { matches: false };
  const REDUCED = window.matchMedia ? window.matchMedia('(prefers-reduced-motion: reduce)').matches : false;
  const SELFTEST = location.hash === '#selftest';
  const LEAD_S = 0.6, MOVE_S = 1.5, TURN_S = 3.2, HOLD_S = 2.2, NOTCHES = 20;
  const KINDS = [['assembly', '조립'], ['print', '출력'], ['measure', '재기·시험'], ['wiring', '배선'], ['view', '설계 보기']];
  const PHOTO_HELP = '억지로 하지 마세요. 휴대폰 Claude 앱(Remote Control) 대화창의 카메라로 그 자리를 찍어 보내면 됩니다.';
  const PHOTO_HELP_STEP = ' 어디까지 했는지는 확인 칸으로 보이니, 어느 단계인지 따로 적지 않아도 됩니다.';

  const S = {
    catalog: null, guide: null, gid: '', cur: 0, view: 'home', speed: 1, db: null,
    checks: {}, notes: [], records: [], localNotes: [], dbState: 'off', resumeGid: '', platePart: null,
  };

  // ---------- 작은 도구 ----------
  function el(tag, cls, text) { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; }
  let toastTimer = 0;
  function toast(msg) { const t = $('toast'); t.textContent = msg; t.classList.add('on'); clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('on'), 2600); }
  function noSelection() { const s = window.getSelection ? window.getSelection() : null; return !s || String(s).length === 0; }
  function copyText(text, target) {
    const fallback = () => {
      try { const r = document.createRange(); r.selectNodeContents(target); const s = window.getSelection(); s.removeAllRanges(); s.addRange(r); } catch (e) { /* 고르기도 못 하면 그대로 */ }
      toast('글자를 골라 두었습니다. Ctrl+C로 복사하세요.');
    };
    try { navigator.clipboard.writeText(text).then(() => toast('복사했습니다: ' + text), fallback); } catch (e) { fallback(); }
  }
  function store(key, val) { try { localStorage.setItem('guide.' + key, JSON.stringify(val)); } catch (e) { /* 이 기기에 못 남겨도 화면은 된다 */ } }
  function recall(key) { try { return JSON.parse(localStorage.getItem('guide.' + key)); } catch (e) { return null; } }
  function two(n) { return (n < 10 ? '0' : '') + n; }
  function hm(iso) {
    // 날짜만 적은 기록(예: 2026-11-15)은 날짜만 보인다 (모르는 시각을 지어내지 않게)
    if (typeof iso === 'string' && /^\d{4}-\d{2}-\d{2}$/.test(iso)) return Number(iso.slice(5, 7)) + '/' + Number(iso.slice(8, 10));
    const d = new Date(iso); return isNaN(d) ? '' : (d.getMonth() + 1) + '/' + d.getDate() + ' ' + two(d.getHours()) + ':' + two(d.getMinutes());
  }
  async function getJSON(path) {
    const r = await fetch(path, { cache: 'no-cache' });
    if (!r.ok) throw new Error(path + ' 를 읽지 못했습니다 (' + r.status + ')');
    return r.json();
  }
  const PICO = {
    kit: '<path d="M2.5 5.2L8 2.4l5.5 2.8v5.6L8 13.6l-5.5-2.8z"/><path d="M2.5 5.2L8 8l5.5-2.8M8 8v5.6"/>',
    do: '<path d="M6.5 4.5h7M6.5 8h7M6.5 11.5h7"/><path d="M2.6 4.5h1.2M2.6 8h1.2M2.6 11.5h1.2"/>',
    warn: '<path d="M8 2.4l6.2 10.8H1.8z"/><path d="M8 6.6v3.1M8 11.4v.2"/>',
    check: '<circle cx="8" cy="8" r="5.9"/><path d="M5.3 8.2l1.9 1.9 3.6-3.8"/>',
    photo: '<rect x="2" y="4.6" width="12" height="8.6" rx="1.6"/><circle cx="8" cy="8.9" r="2.3"/><path d="M5.6 4.6l.9-1.8h3l.9 1.8"/>',
    note: '<path d="M10.6 2.8l2.6 2.6-7.4 7.4H3.2v-2.6z"/>',
  };

  // ---------- 나사 그림 ----------
  // 그림끼리 같은 비율로 그려 길이를 견주게 한다. 실제 크기는 아니다 (모르는 나사는 자 옆에 놓고 찍어 Claude에게 묻는다)
  const HW_PX_PER_MM = 96 / 25.4;
  function f(n) { return n.toFixed(3); }
  function svgWrap(w, h, inner, label) {
    return '<svg class="hwsvg" width="' + (w * HW_PX_PER_MM).toFixed(1) + '" height="' + (h * HW_PX_PER_MM).toFixed(1) +
      '" viewBox="0 0 ' + w.toFixed(3) + ' ' + h.toFixed(3) + '" role="img" aria-label="' + label + '">' + inner + '</svg>';
  }
  function domeHead(x0, xs, cy, dk) {
    return 'M' + f(xs) + ',' + f(cy - dk / 2) + 'C' + f(x0 + (xs - x0) * 0.25) + ',' + f(cy - dk / 2) + ' ' + f(x0) + ',' + f(cy - dk * 0.32) + ' ' + f(x0) + ',' + f(cy) +
      'C' + f(x0) + ',' + f(cy + dk * 0.32) + ' ' + f(x0 + (xs - x0) * 0.25) + ',' + f(cy + dk / 2) + ' ' + f(xs) + ',' + f(cy + dk / 2) + 'Z';
  }
  function drawScrew(L) {
    // ISO 7380 둥근머리 M3: 머리 지름 5.7, 높이 1.65
    const dk = 5.7, k = 1.65, d = 3, pad = 0.4, W = pad * 2 + k + L, H = pad * 2 + dk, cy = H / 2, x0 = pad, xs = pad + k;
    let th = '';
    for (let s = xs + 0.5; s < xs + L - 0.2; s += 0.5) th += 'M' + f(s) + ',' + f(cy - d / 2) + 'L' + f(s + 0.25) + ',' + f(cy + d / 2);
    return svgWrap(W, H, '<path class="mf" d="' + domeHead(x0, xs, cy, dk) + '"/><rect class="mf" x="' + f(xs) + '" y="' + f(cy - d / 2) + '" width="' + L + '" height="' + d + '"/><path class="th" d="' + th + '"/>', 'M3 × ' + L);
  }
  function drawCap(L) {
    // DIN 912 유두 M3: 머리 지름 5.5, 높이 3.0, 머리 끝에 육각 구멍 (2.5 mm 렌치)
    const dk = 5.5, k = 3.0, d = 3, pad = 0.4, W = pad * 2 + k + L, H = pad * 2 + dk, cy = H / 2, x0 = pad, xs = pad + k;
    let th = '';
    for (let s = xs + 0.5; s < xs + L - 0.2; s += 0.5) th += 'M' + f(s) + ',' + f(cy - d / 2) + 'L' + f(s + 0.25) + ',' + f(cy + d / 2);
    const head = '<rect class="mf" x="' + f(x0) + '" y="' + f(cy - dk / 2) + '" width="' + f(k) + '" height="' + f(dk) + '" rx="0.4"/>';
    const socket = '<rect class="hole" x="' + f(x0) + '" y="' + f(cy - 1.25) + '" width="1.2" height="2.5"/>';
    return svgWrap(W, H, head + socket + '<rect class="mf" x="' + f(xs) + '" y="' + f(cy - d / 2) + '" width="' + L + '" height="' + d + '"/><path class="th" d="' + th + '"/>', 'M3 × ' + L + ' 유두');
  }
  function drawNut() {
    const s = 5.5, R = s / Math.sqrt(3), pad = 0.4, W = 2 * R + pad * 2, H = s + pad * 2, cx = W / 2, cy = H / 2;
    let p = '';
    for (let k = 0; k < 6; k += 1) { const a = Math.PI / 3 * k; p += (k ? 'L' : 'M') + f(cx + R * Math.cos(a)) + ',' + f(cy + R * Math.sin(a)); }
    return svgWrap(W, H, '<path class="mf" d="' + p + 'Z"/><circle class="hole" cx="' + f(cx) + '" cy="' + f(cy) + '" r="1.25"/>', 'M3 너트');
  }
  function drawPin(L, d) {
    const pad = 0.4, c = 0.4, W = L + pad * 2, H = d + pad * 2;
    const p = 'M' + f(pad + c) + ',' + f(pad) + 'H' + f(pad + L - c) + 'L' + f(pad + L) + ',' + f(pad + c) + 'V' + f(pad + d - c) + 'L' + f(pad + L - c) + ',' + f(pad + d) +
      'H' + f(pad + c) + 'L' + f(pad) + ',' + f(pad + d - c) + 'V' + f(pad + c) + 'Z';
    return svgWrap(W, H, '<path class="mf" d="' + p + '"/>', '핀');
  }
  function drawTap(tot) {
    const dh = 4.0, kh = 1.3, d = 2.0, tip = 1.2, pad = 0.4, W = tot + pad * 2, H = dh + pad * 2, cy = H / 2, xs = pad + kh, xe = pad + tot;
    const shank = 'M' + f(xs) + ',' + f(cy - d / 2) + 'H' + f(xe - tip) + 'L' + f(xe) + ',' + f(cy) + 'L' + f(xe - tip) + ',' + f(cy + d / 2) + 'H' + f(xs) + 'Z';
    return svgWrap(W, H, '<path class="mf" d="' + domeHead(pad, xs, cy, dh) + '"/><path class="mf" d="' + shank + '"/>', '자가탭 나사');
  }
  function drawWasher(D, d) {
    const pad = 0.4, W = D + pad * 2, c = W / 2;
    return svgWrap(W, W, '<circle class="mf" cx="' + f(c) + '" cy="' + f(c) + '" r="' + D / 2 + '"/><circle class="hole" cx="' + f(c) + '" cy="' + f(c) + '" r="' + d / 2 + '"/>', '와셔');
  }
  function drawFor(d) {
    if (d.t === 'screw') return drawScrew(d.L);
    if (d.t === 'cap') return drawCap(d.L);
    if (d.t === 'nut') return drawNut();
    if (d.t === 'pin') return drawPin(d.L, d.d);
    if (d.t === 'tap') return drawTap(d.L);
    if (d.t === 'washer') return drawWasher(d.D, d.d);
    return '';
  }
  const ICON = {
    hex: '<path d="M6 8H30a2 2 0 0 1 2 2V25" fill="none" stroke="currentColor" stroke-width="3.4" stroke-linecap="round" stroke-linejoin="round"/>',
    driver: '<rect x="3" y="10" width="15" height="10" rx="3" fill="currentColor"/><path d="M18 15H37" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"/><path d="M37 12.5V17.5" stroke="currentColor" stroke-width="1.6"/>',
    tweezers: '<path d="M5 15L39 8M5 15L39 22" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"/>',
    knife: '<rect x="3" y="12" width="14" height="7" rx="2" fill="currentColor"/><path d="M17 12H35L41 15.5L17 19Z" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"/>',
    tape: '<circle cx="22" cy="15" r="10" fill="none" stroke="currentColor" stroke-width="4"/><circle cx="22" cy="15" r="3" fill="none" stroke="currentColor" stroke-width="1.5"/>',
    wire: '<path d="M3 20C10 6 16 6 22 15S34 24 41 10" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round"/>',
    pen: '<rect x="9" y="11" width="24" height="8" rx="2" fill="currentColor"/><path d="M33 12.5L40 15L33 17.5Z" fill="currentColor"/><rect x="4" y="11.5" width="5" height="7" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.6"/>',
    caliper: '<rect x="4" y="8" width="36" height="5" rx="1" fill="currentColor"/><path d="M8 13V24M15 13V20" stroke="currentColor" stroke-width="2.6" stroke-linecap="round"/>',
    scale: '<rect x="5" y="19" width="34" height="6" rx="2" fill="currentColor"/><path d="M9 15H35" stroke="currentColor" stroke-width="2.6" stroke-linecap="round"/><path d="M22 15V19" stroke="currentColor" stroke-width="2"/>',
    band: '<ellipse cx="22" cy="15" rx="15" ry="8" fill="none" stroke="currentColor" stroke-width="2.6"/>',
    ruler: '<rect x="3" y="9" width="38" height="11" rx="1.5" fill="none" stroke="currentColor" stroke-width="2.2"/><path d="M9 9V14M15 9V16M21 9V14M27 9V16M33 9V14" stroke="currentColor" stroke-width="1.6"/>',
    pliers: '<path d="M3 9L20 14L3 21M20 14L41 13.2M20 16L41 16.8" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/><circle cx="20" cy="15" r="2.2" fill="currentColor"/>',
    phone: '<rect x="14" y="3" width="16" height="24" rx="2.5" fill="none" stroke="currentColor" stroke-width="2.4"/><circle cx="22" cy="23" r="1.4" fill="currentColor"/>',
    book: '<path d="M6 8H20a2 2 0 0 1 2 2V24a2 2 0 0 0-2-2H6Z M38 8H24a2 2 0 0 0-2 2V24a2 2 0 0 1 2-2H38Z" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linejoin="round"/>',
  };
  const THUMBS = {};
  function itemOf(key) { return (S.guide && S.guide.items[key]) || { name: key, kind: 'tool', meta: '' }; }
  function makePic(key) {
    const it = itemOf(key);
    const box = el('div', 'pic');
    if (it.thumb) {
      const img = document.createElement('img'); img.alt = ''; img.dataset.thumb = key;
      if (THUMBS[key]) img.src = THUMBS[key];
      box.appendChild(img);
    } else if (it.icon && ICON[it.icon]) {
      box.classList.add('tool');
      box.innerHTML = '<svg class="tool-ic" viewBox="0 0 44 30" aria-hidden="true">' + ICON[it.icon] + '</svg>';
    } else if (it.draw) {
      box.classList.add('hw');
      box.innerHTML = drawFor(it.draw);
    } else {
      box.classList.add('tool');
    }
    return box;
  }
  function fileLine(name) {
    const d = el('div', 'fl');
    const fe = el('span', 'file', name);
    const b = el('button', 'copy', '복사'); b.type = 'button'; b.setAttribute('aria-label', name + ' 복사');
    b.addEventListener('click', () => copyText(name, fe));
    d.appendChild(fe); d.appendChild(b);
    return d;
  }
  // ---------- 칸: 만들기, 자리, 접기, 끌기 ----------
  // fold: keep = 접은 채로 둔다, step = 다음 단계에서 다시 펼친다 (단계마다 새 내용이라 놓치면 안 된다), 없음 = 접지 않는다
  const PANEL_DEF = {
    pHead: { ico: null, fold: null },
    pKit: { ico: 'kit', fold: 'keep' },
    pDo: { ico: 'do', fold: null },
    pWarn: { ico: 'warn', fold: 'step' },
    pCheck: { ico: 'check', fold: 'step' },
    pStuck: { ico: 'photo', fold: 'keep' },
    pNote: { ico: 'note', fold: 'keep' },
  };
  const PANEL_IDS = Object.keys(PANEL_DEF);
  const DOCKS = { l: 'dockL', r: 'dockR', b: 'dockB' };
  const DEFAULT_LAYOUT = { l: ['pHead', 'pKit'], r: ['pWarn', 'pCheck', 'pStuck', 'pNote'], b: ['pDo'] };
  const P = {};
  const GRIP = '<svg class="grip" viewBox="0 0 10 16" aria-hidden="true"><circle cx="3" cy="3" r="1.3"/><circle cx="7" cy="3" r="1.3"/><circle cx="3" cy="8" r="1.3"/><circle cx="7" cy="8" r="1.3"/><circle cx="3" cy="13" r="1.3"/><circle cx="7" cy="13" r="1.3"/></svg>';
  const CHEV = '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M2.5 4.5L6 8l3.5-3.5"/></svg>';
  let folds = recall('folds') || {};
  function buildPanels() {
    PANEL_IDS.forEach((id) => {
      const d = PANEL_DEF[id];
      const p = el('section', 'panel'); p.id = id;
      const bar = el('div', 'ph'); bar.innerHTML = GRIP;
      const pt = el('div', 'pt'); bar.appendChild(pt);
      if (d.fold) {
        const fb = el('button', 'fold-btn'); fb.type = 'button'; fb.setAttribute('aria-label', '접기 또는 펼치기'); fb.innerHTML = CHEV;
        fb.addEventListener('click', (e) => { e.stopPropagation(); toggleFold(id); });
        bar.appendChild(fb);
      }
      const body = el('div', 'pb');
      p.appendChild(bar); p.appendChild(body);
      bar.addEventListener('pointerdown', (e) => barDown(e, id));
      bar.addEventListener('click', (e) => { if (PHONE.matches && !e.target.closest('.fold-btn')) toggleFold(id); });
      P[id] = { el: p, pt: pt, body: body };
    });
  }
  function setTitle(id, text, extra) {
    const ico = PANEL_DEF[id].ico, pt = P[id].pt;
    pt.innerHTML = ico ? '<svg viewBox="0 0 16 16" aria-hidden="true">' + PICO[ico] + '</svg>' : '';
    pt.appendChild(document.createTextNode(text));
    if (extra) pt.appendChild(el('span', 'px', extra));
  }
  function validLayout(v) {
    if (!v || typeof v !== 'object') return false;
    const all = [].concat(v.l || [], v.r || [], v.b || []);
    return all.length === PANEL_IDS.length && PANEL_IDS.every((id) => all.indexOf(id) >= 0);
  }
  function applyLayout(lay) { Object.keys(DOCKS).forEach((k) => lay[k].forEach((id) => $(DOCKS[k]).appendChild(P[id].el))); }
  function readLayout() {
    const lay = {};
    Object.keys(DOCKS).forEach((k) => { lay[k] = Array.from($(DOCKS[k]).children).filter((c) => c.classList.contains('panel')).map((c) => c.id); });
    return lay;
  }
  function setFold(id, on) { P[id].el.classList.toggle('folded', on); }
  function toggleFold(id) {
    const d = PANEL_DEF[id]; if (!d.fold) return;
    const on = !P[id].el.classList.contains('folded');
    setFold(id, on);
    if (d.fold === 'keep') { folds[id] = on; store('folds', folds); }
    afterLayout();
  }
  function applyFolds() { PANEL_IDS.forEach((id) => { const d = PANEL_DEF[id]; setFold(id, d.fold === 'keep' ? !!folds[id] : false); }); }

  let drag = null;
  function barDown(e, id) {
    if (PHONE.matches || e.button !== 0 || e.target.closest('.fold-btn')) return;
    e.preventDefault();
    drag = { id: id, p: P[id].el, x0: e.clientX, y0: e.clientY, started: false };
    window.addEventListener('pointermove', dragMove);
    window.addEventListener('pointerup', dragUp);
    window.addEventListener('pointercancel', dragCancel);
  }
  function lift(e) {
    const p = drag.p, r = p.getBoundingClientRect();
    drag.started = true; drag.offX = e.clientX - r.left; drag.offY = e.clientY - r.top; drag.w = r.width; drag.h = r.height;
    drag.slot = el('div', 'slot');
    p.parentNode.insertBefore(drag.slot, p);
    sizeSlot(p.parentNode.id === 'dockB');
    p.style.width = r.width + 'px';
    p.classList.add('lifting');
    app.classList.add('dragging');
  }
  function sizeSlot(row) {
    const s = drag.slot;
    if (row) { s.style.width = Math.min(drag.w, 420) + 'px'; s.style.height = Math.min(drag.h, 150) + 'px'; }
    else { s.style.width = ''; s.style.height = drag.h + 'px'; }
  }
  function dropTarget(x, y) {
    const s = $('screen').getBoundingClientRect();
    let zone;
    if (y > s.bottom - s.height * 0.3 && x > s.left + s.width * 0.28 && x < s.right - s.width * 0.28) zone = 'b';
    else zone = x < s.left + s.width / 2 ? 'l' : 'r';
    const dock = $(DOCKS[zone]);
    const items = Array.from(dock.children).filter((c) => c !== drag.p && c !== drag.slot && c.classList.contains('panel') && !c.hidden);
    let ref = null;
    for (let k = 0; k < items.length; k += 1) {
      const r = items[k].getBoundingClientRect();
      if (zone === 'b' ? x < r.left + r.width / 2 : y < r.top + r.height / 2) { ref = items[k]; break; }
    }
    return { dock: dock, ref: ref, row: zone === 'b' };
  }
  function dragMove(e) {
    if (!drag) return;
    if (!drag.started) { if (Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) < 5) return; lift(e); }
    drag.p.style.transform = 'translate(' + (e.clientX - drag.offX).toFixed(1) + 'px,' + (e.clientY - drag.offY).toFixed(1) + 'px)';
    const t = dropTarget(e.clientX, e.clientY), s = drag.slot;
    if (s.parentNode === t.dock && (t.ref ? s.nextElementSibling === t.ref : false)) return;
    if (t.ref) t.dock.insertBefore(s, t.ref); else t.dock.appendChild(s);
    sizeSlot(t.row);
  }
  function endDrag() {
    window.removeEventListener('pointermove', dragMove);
    window.removeEventListener('pointerup', dragUp);
    window.removeEventListener('pointercancel', dragCancel);
    const d = drag; drag = null;
    if (d && d.started) { d.p.classList.remove('lifting'); d.p.style.transform = ''; d.p.style.width = ''; app.classList.remove('dragging'); }
    return d;
  }
  function dragUp() {
    const d = endDrag(); if (!d) return;
    if (!d.started) { toggleFold(d.id); return; }
    d.slot.parentNode.insertBefore(d.p, d.slot); d.slot.remove();
    store('docks', readLayout());
    afterLayout();
  }
  function dragCancel() { const d = endDrag(); if (d && d.started) d.slot.remove(); }

  // 칸이 차지한 자리. 3D는 이 바깥 가운데에 맞추고, 준비물 판은 이만큼 띄운다
  function panelInsets() {
    const ins = { l: 0, r: 0, t: 0, b: 0 };
    if (PHONE.matches || app.classList.contains('bare') || S.view !== 'step') return ins;
    const s = $('screen').getBoundingClientRect();
    const vis = (id) => Array.from($(id).children).filter((c) => c.classList.contains('panel') && !c.hidden && !c.classList.contains('lifting')).map((c) => c.getBoundingClientRect());
    const L = vis('dockL'), R = vis('dockR'), B = vis('dockB');
    if (L.length) ins.l = Math.max(0, Math.max.apply(null, L.map((r) => r.right)) - s.left + 8);
    if (R.length) ins.r = Math.max(0, s.right - Math.min.apply(null, R.map((r) => r.left)) + 8);
    if (B.length) ins.b = Math.max(0, s.bottom - Math.min.apply(null, B.map((r) => r.top)) + 8);
    return ins;
  }
  function padBoard() {
    const ins = PHONE.matches ? null : panelInsets();
    $('boardIn').style.padding = $('board').hidden || !ins ? '' : '20px ' + Math.max(24, ins.r) + 'px ' + Math.max(40, ins.b) + 'px ' + Math.max(24, ins.l) + 'px';
    const pv = $('plateView');
    pv.style.padding = pv.hidden || !ins ? '' : '16px ' + Math.max(16, ins.r) + 'px ' + Math.max(16, ins.b) + 'px ' + Math.max(16, ins.l) + 'px';
  }
  function afterLayout() { requestAnimationFrame(() => { if (G) computeInsets(); padBoard(); }); }
  function toggleBare() {
    if (PHONE.matches) return;
    const on = !app.classList.contains('bare');
    app.classList.toggle('bare', on);
    $('tBare').textContent = on ? '칸 보이기 (H)' : '칸 숨기기 (H)';
    $('tBare').setAttribute('aria-pressed', on ? 'true' : 'false');
    afterLayout();
  }
  // 이름표 끄기: 3D 위의 부품 이름표와 방향 이름표를 숨긴다 (손볼 자리 번호는 글에서 가리키므로 둔다). 이 기기에 기억한다
  function setLabels(off) {
    app.classList.toggle('nolabels', off);
    $('tLabels').textContent = off ? '이름표 켜기 (L)' : '이름표 끄기 (L)';
    $('tLabels').setAttribute('aria-pressed', off ? 'true' : 'false');
  }
  function toggleLabels() { const off = !app.classList.contains('nolabels'); setLabels(off); store('nolabels', off); }

  // ---------- 칸 채우기 ----------
  function stepRef() { return S.guide ? S.guide.steps[S.cur] : null; }
  function checksFor(stepId) { const m = marksOf(S.gid)[stepId]; return Array.isArray(m) ? m : []; }
  function renderPanels(step) {
    const g = S.guide;
    const head = step.kind === 'prep' ? '시작하기 전에' : step.kind === 'end' ? '끝' : step.kind === 'plate' ? '판 ' + step.no + ' / ' + g.work : step.no + '단계 / ' + g.work;
    setTitle('pHead', head);
    P.pHead.body.textContent = ''; P.pHead.body.appendChild(el('h1', '', step.title));
    if (step.plate) {
      P.pHead.body.appendChild(fileLine(step.plate.file));
      P.pHead.body.appendChild(el('div', 'meta', step.plate.folder + ' · ' + step.plate.preset));
    }

    setTitle('pDo', step.kind === 'end' ? '마무리' : '할 일');
    P.pDo.body.textContent = '';
    const ol = el('ol'); step.do.forEach((t) => ol.appendChild(el('li', '', t))); P.pDo.body.appendChild(ol);

    P.pWarn.el.hidden = !step.warn.length; P.pWarn.body.textContent = '';
    if (step.warn.length) { setTitle('pWarn', '주의', '· ' + step.warn.length); const ul = el('ul'); step.warn.forEach((t) => ul.appendChild(el('li', '', t))); P.pWarn.body.appendChild(ul); }

    P.pKit.el.hidden = !step.kit.length; P.pKit.body.textContent = '';
    if (step.kit.length) {
      setTitle('pKit', '이번에 쓰는 것', '· ' + step.kit.length);
      const box = el('div', 'kit');
      step.kit.forEach((row) => {
        const it = itemOf(row[0]);
        const line = el('div', 'ki');
        line.appendChild(makePic(row[0]));
        const tx = el('div');
        const nm = el('div', 'nm', it.name);
        if (row[1]) nm.appendChild(el('span', 'q', '× ' + row[1]));
        tx.appendChild(nm);
        tx.appendChild(el('div', 'meta', [row[2], it.meta].filter(Boolean).join(' · ')));
        if (it.file) tx.appendChild(fileLine(it.file));
        line.appendChild(tx); box.appendChild(line);
      });
      P.pKit.body.appendChild(box);
    }
    if (step.plate) { renderPlateParts(step); renderPlateStatus(step); } else renderChecks(step);

    P.pStuck.el.hidden = step.kind === 'end'; P.pStuck.body.textContent = '';
    setTitle('pStuck', '막히면');
    const help = el('div', 'helpbox');
    if (step.stuck) help.appendChild(el('p', '', step.stuck));
    help.appendChild(el('p', '', PHOTO_HELP + (step.check.length ? PHOTO_HELP_STEP : '')));
    P.pStuck.body.appendChild(help);

    P.pNote.body.textContent = '';
    setTitle('pNote', '이 단계 메모');
    const ta = document.createElement('textarea'); ta.id = 'noteText'; ta.placeholder = '막힌 곳, 이상한 점, 고칠 곳을 적습니다. Claude가 읽습니다.';
    const draft = recall('draft.' + S.gid + '.' + step.id); if (draft) ta.value = draft;
    ta.addEventListener('input', () => store('draft.' + S.gid + '.' + step.id, ta.value));
    P.pNote.body.appendChild(ta);
    const row = el('div', 'row');
    row.appendChild(el('span', '', S.db ? '남기면 이 단계에 저장되고 Claude가 읽습니다' : '이 화면에서는 저장 기능이 없어, 이 창을 닫으면 사라집니다'));
    const sb = el('button', 'btn', '메모 남기기'); sb.type = 'button';
    sb.addEventListener('click', () => addNote(step, ta));
    row.appendChild(sb); P.pNote.body.appendChild(row);
    const list = el('ul', 'notes-list'); list.id = 'notesList'; P.pNote.body.appendChild(list);
    renderNotes();
    applyFolds();
  }
  function renderChecks(step) {
    const hasCheck = step.check.length > 0;
    P.pCheck.el.hidden = !hasCheck; P.pCheck.body.textContent = '';
    if (!hasCheck) return;
    const title = step.kind === 'prep' ? '다 꺼냈는지 확인' : '다 하면 확인';
    const marks = checksFor(step.id);
    const count = () => step.check.filter((t, k) => checksFor(step.id)[k]).length + ' / ' + step.check.length;
    setTitle('pCheck', title, '· ' + count());
    if (step.kind === 'work') P.pCheck.body.appendChild(el('p', 'sub', '움직임의 끝 장면이 다 된 모습입니다.'));
    const ul = el('ul');
    step.check.forEach((t, k) => {
      const id = 'ck-' + step.id + '-' + k;
      const li = el('li');
      const cb = document.createElement('input'); cb.type = 'checkbox'; cb.id = id; cb.checked = !!marks[k];
      const sp = el('span', 'ck-text', t); sp.id = id + '-t'; cb.setAttribute('aria-labelledby', sp.id);
      li.classList.toggle('on', cb.checked);
      cb.addEventListener('change', () => { setCheck(step, k, cb.checked); li.classList.toggle('on', cb.checked); setTitle('pCheck', title, '· ' + count()); });
      sp.addEventListener('click', () => { if (!noSelection()) return; cb.checked = !cb.checked; cb.dispatchEvent(new Event('change')); });
      li.appendChild(cb); li.appendChild(sp); ul.appendChild(li);
    });
    P.pCheck.body.appendChild(ul);
  }
  // ---------- 판: 뽑았는지와 못 쓰는 부품 ----------
  // 사용자가 대화창에 알리면 Claude가 db의 records에 적는다. 화면은 보여 주기만 한다 (누를 칸이 없다).
  // records: {guide, step, kind: 'printed' | 'bad', part: STL 파일 이름(bad), count, note, at}
  function printedRec(gid, sid) { return S.records.find((r) => r.guide === gid && r.step === sid && r.kind === 'printed') || null; }
  function badRecs(file, gid, sid) {
    return S.records.filter((r) => r.kind === 'bad' && r.part === file && (gid == null || (r.guide === gid && r.step === sid)));
  }
  function sumCount(rs) { return rs.reduce((s, r) => s + (Number(r.count) || 1), 0); }
  function day(iso) { return hm(iso).split(' ')[0]; }
  function renderPlateParts(step) {
    const parts = step.plate.parts;
    P.pKit.el.hidden = !parts.length; P.pKit.body.textContent = '';
    const pickable = parts.some((p) => p.p != null);
    setTitle('pKit', '이 판의 부품', '· ' + parts.reduce((s, p) => s + p.count, 0) + (pickable ? ' · 누르면 3D' : ''));
    const box = el('div', 'kit');
    parts.forEach((p) => {
      const tx = el('div', 'ki plate-part');
      if (p.p != null) {
        // 누르면 3D로 이 부품을 보인다. 한 번 더 누르면 판 그림으로 돌아간다
        tx.classList.add('pick'); tx.classList.toggle('on', S.platePart === p.p);
        tx.tabIndex = 0; tx.setAttribute('role', 'button'); tx.setAttribute('aria-pressed', S.platePart === p.p ? 'true' : 'false');
        tx.addEventListener('click', (e) => { if (e.target.closest('.copy') || !noSelection()) return; selectPlatePart(step, p.p); });
        tx.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); selectPlatePart(step, p.p); } });
      }
      const nm = el('div', 'nm', p.label); nm.appendChild(el('span', 'q', '× ' + p.count));
      tx.appendChild(nm);
      if (p.height != null) tx.appendChild(el('div', 'meta', '높이 ' + p.height + ' mm'));
      tx.appendChild(fileLine(p.file));
      const bad = badRecs(p.file, S.gid, step.id);
      if (bad.length) tx.appendChild(el('div', 'pst need', '못 씀 ' + sumCount(bad) + '개' + bad.map((r) => (r.note ? ' · ' + r.note : '')).join('')));
      box.appendChild(tx);
    });
    P.pKit.body.appendChild(box);
  }
  function renderPlateStatus(step) {
    P.pCheck.el.hidden = false; P.pCheck.body.textContent = '';
    const rec = printedRec(S.gid, step.id);
    setTitle('pCheck', '상태', rec ? '· 뽑음' : '· 아직');
    if (rec) {
      P.pCheck.body.appendChild(el('p', 'pst ok', '✓ 뽑음' + (rec.at ? ' · ' + day(rec.at) : '') + (rec.note ? ' · ' + rec.note : '')));
      const bad = S.records.filter((r) => r.kind === 'bad' && r.guide === S.gid && r.step === step.id);
      if (bad.length) P.pCheck.body.appendChild(el('p', 'pst need', '못 쓰는 부품 ' + sumCount(bad) + '개 (이 판의 부품 칸에 표시)'));
    } else {
      P.pCheck.body.appendChild(el('p', 'pst wait', '아직 안 뽑았습니다.'));
    }
    P.pCheck.body.appendChild(el('p', 'sub', '다 뽑으면 대화창에 알려 주세요. 못 쓰는 부품이 있으면 무엇이 왜 그런지 같이 적어 주시면, ' +
      'Claude가 여기에 적고 모자란 부품은 다시 뽑을 판을 만듭니다.'));
  }
  // 판 화면의 무대: 기본은 판 그림, 부품을 고르면 그 부품의 3D (뽑는 방향으로 놓은 지금 설계 형상)
  function platePseudo(step, idx) {
    return { id: step.id + '-p' + idx, kind: 'plate', show: [idx], hot: [], shell: [], cut: false, ghost: false, place: [], joint: 0,
      moves: [], turn: null, spots: [], labels: [], tags: [], view: { up: [0, 0, 1], face: [0, -1, 0], el: 24, az: 32 } };
  }
  function showPlateStage(step) {
    const three = S.platePart != null && init3d();
    $('plateView').hidden = three; $('c').hidden = !three; $('tags').hidden = !three; $('tools3d').hidden = !three;
    $('tPlate').hidden = !three; $('player').hidden = true; $('doneTag').hidden = true;
    const img = $('plateImg');
    if (step.plate.image) { img.src = step.plate.image; img.alt = '판 ' + step.no + ' 배치 그림'; img.hidden = false; } else { img.removeAttribute('src'); img.hidden = true; }
    if (three) {
      if (PHONE.matches && !app.classList.contains('with3d')) { app.classList.add('with3d'); $('show3d').textContent = '3D 닫기'; }
      if (G.items.length !== S.guide.scene.length) buildItems();
      G.resize(); apply3d(platePseudo(step, S.platePart), false);
    } else if (G) G.step = null;
    afterLayout();
  }
  function selectPlatePart(step, idx) {
    S.platePart = S.platePart === idx ? null : idx;
    showPlateStage(step);
    renderPlateParts(step);
  }

  // 조립 안내의 출력 부품: 쓸 수 있는 개수와, 모자라면 뽑을 판
  function partStatus(it, need) {
    const refs = it.plates || [];
    const made = refs.reduce((s, ref) => s + (printedRec(ref[0], ref[1]) ? ref[3] : 0), 0);
    const ok = Math.max(0, made - sumCount(badRecs(it.file)));
    if (ok >= need) return { ok: true, text: '✓ 뽑음 · 판 ' + refs.filter((ref) => printedRec(ref[0], ref[1])).map((ref) => ref[2]).join(', ') };
    const next = refs.find((ref) => !printedRec(ref[0], ref[1]));
    const short = need - ok;
    if (next) return { ok: false, text: (made ? short + '개 모자람 · ' : '아직 안 뽑음 · ') + '판 ' + next[2], go: next };
    return { ok: false, text: short + '개 모자람 · 대화창에 알려 주시면 Claude가 다시 뽑을 판을 만듭니다' };
  }
  function goStep(gid, sid) { const h = '#' + gid + '--' + sid; if (location.hash === h) route(); else location.hash = h; }

  function renderNotes() {
    const list = $('notesList'); const step = stepRef();
    if (!list || !step) return;
    list.textContent = '';
    const mine = (S.db ? S.notes : S.localNotes).filter((n) => n.guide === S.gid && n.step === step.id);
    const recs = S.records.filter((r) => r.guide === S.gid && r.step === step.id);
    const all = mine.map((n) => ({ kind: 'note', at: n.at || '', n: n })).concat(recs.map((r) => ({ kind: 'rec', at: r.at || '', n: r })));
    all.sort((a, b) => String(a.at).localeCompare(String(b.at)));
    all.forEach((row) => {
      const li = el('li', 'note-item' + (row.kind === 'rec' ? ' rec' : ''));
      const meta = el('div', 'meta');
      meta.appendChild(el('span', '', (row.kind === 'rec' ? 'Claude 기록 · ' : '메모 · ') + hm(row.at)));
      if (row.kind === 'note' && S.db && row.n.id) {
        const del = el('button', 'linkbtn', '지우기'); del.type = 'button';
        del.addEventListener('click', () => deleteNote(row.n.id));
        meta.appendChild(del);
      }
      li.appendChild(meta);
      const text = row.kind === 'rec' ? [row.n.note, row.n.value != null ? row.n.value + ' ' + (row.n.unit || '') : ''].filter(Boolean).join(' · ') : row.n.text;
      if (text) li.appendChild(el('div', 'txt', text));
      if (row.kind === 'rec' && row.n.asset && /^[0-9a-f]{32}$/.test(String(row.n.asset))) {
        const img = document.createElement('img'); img.alt = '단계 기록 사진'; img.loading = 'lazy'; img.src = '/_blob/' + row.n.asset;
        li.appendChild(img);
      }
      list.appendChild(li);
    });
    const n = all.length;
    setTitle('pNote', '이 단계 메모', n ? '· ' + n : '');
  }

  // ---------- 준비물 판 ----------
  function renderBoard() {
    const inner = $('boardIn'); inner.textContent = '';
    (S.guide.board || []).forEach((g) => {
      const sec = el('section', 'bgroup');
      const h = el('h2', '', g.title);
      if (g.kind === 'hardware') h.appendChild(el('small', '', '그림끼리 같은 비율입니다 · 실제 크기는 아닙니다'));
      if (g.kind === 'printed' && g.copy) h.appendChild(el('small', '', '뽑은 파일이 있는 곳'));
      sec.appendChild(h);
      if (g.copy) sec.appendChild(fileLine(g.copy));
      const grid = el('div', 'bgrid');
      g.items.forEach((row) => {
        const it = itemOf(row[0]);
        const card = el('div', 'bi');
        card.appendChild(makePic(row[0]));
        const top = el('div', 'top');
        top.appendChild(el('span', 'nm', it.name));
        if (row[1]) top.appendChild(el('span', 'q', '× ' + row[1]));
        card.appendChild(top);
        card.appendChild(el('div', 'meta', [it.meta, row[2]].filter(Boolean).join(' · ')));
        if (it.file) card.appendChild(fileLine(it.file));
        if (it.kind === 'printed' && it.plates) {
          // 뽑았는지는 보여 주기만 한다. 조립을 막지 않는다 (이미 뽑아 둔 부품이 있을 수 있다)
          const st = partStatus(it, row[1] || 1);
          const line = el('div', 'pst ' + (st.ok ? 'ok' : 'need'));
          if (st.go) {
            const b = el('button', 'linkbtn', st.text); b.type = 'button';
            b.addEventListener('click', () => goStep(st.go[0], st.go[1]));
            line.appendChild(b);
          } else line.textContent = st.text;
          card.appendChild(line);
        }
        grid.appendChild(card);
      });
      sec.appendChild(grid);
      inner.appendChild(sec);
    });
  }

  // ---------- 진행 ----------
  // 확인 칸을 다 누른 단계가 끝낸 단계다. 판 단계(확인 칸 0)는 Claude가 뽑았다고 적으면 끝난다.
  // 단계를 눌러 보기만 해서는 진행이 바뀌지 않는다
  function marksOf(gid) { const c = S.checks[gid]; return (c && c.marks && typeof c.marks === 'object') ? c.marks : {}; }
  function stepDone(gid, id, n) {
    if (!(n > 0)) return !!printedRec(gid, id);
    const m = marksOf(gid)[id]; return Array.isArray(m) && m.length >= n && m.slice(0, n).every(Boolean);
  }
  function trackOf(gid) {
    // 끝이 정해지는 단계마다 [id, 번호, 짧은 이름, 확인 칸 수]. 연 안내는 단계 데이터에서, 아니면 안내 목록에서 읽는다
    if (S.guide && S.gid === gid) return S.guide.steps.filter((s) => s.check.length || s.kind === 'plate').map((s) => [s.id, s.no, s.short, s.check.length]);
    const row = S.catalog && S.catalog.guides.find((r) => r.id === gid);
    return (row && row.track) || [];
  }
  function lastActivity(gid) {
    let at = String((S.checks[gid] && S.checks[gid].at) || '');
    S.records.forEach((r) => { if (r.guide === gid && typeof r.at === 'string' && r.at > at) at = r.at; });
    return at;
  }
  function guideProgress(gid) {
    const tr = trackOf(gid), marks = marksOf(gid);
    const work = tr.filter((t) => t[1] !== '준비');
    const done = work.filter((t) => stepDone(gid, t[0], t[3])).length;
    const started = tr.some((t) => (t[3] > 0 ? Array.isArray(marks[t[0]]) && marks[t[0]].some(Boolean) : !!printedRec(gid, t[0])));
    const next = tr.find((t) => !stepDone(gid, t[0], t[3])) || null;
    const text = !started ? '시작 전' : !next ? '다 함' : done + ' / ' + work.length;
    return { text: text, frac: work.length ? done / work.length : 0, done: done, work: work.length, started: started, next: next };
  }
  function resumeIndex(gid) {
    // 아직 안 끝낸 첫 단계. 다 했으면 끝 화면, 확인 칸이 하나도 없으면 처음
    const steps = S.guide.steps, p = guideProgress(gid);
    if (!p.next) return p.started ? steps.length - 1 : 0;
    const i = steps.findIndex((s) => s.id === p.next[0]);
    return i < 0 ? 0 : i;
  }
  function stepName(t) { return t[1] === '준비' ? '준비물' : t[1] + '단계 · ' + t[2]; }

  // ---------- 막대, 띠, 안내 바꾸기, 처음 화면 ----------
  function renderStrip() {
    const s = $('strip'); s.textContent = '';
    S.guide.steps.forEach((st, i) => {
      const li = el('li', 'chip' + (i === S.cur ? ' cur' : '') + (stepDone(S.gid, st.id, st.check.length) ? ' done' : ''));
      li.tabIndex = 0; li.setAttribute('role', 'button');
      if (i === S.cur) li.setAttribute('aria-current', 'step');
      li.appendChild(el('span', 'n', st.no));
      li.appendChild(el('span', 't', st.short));
      li.addEventListener('click', () => { if (noSelection()) go(S.gid, i); });
      li.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); go(S.gid, i); } });
      s.appendChild(li);
    });
    const cur = s.children[S.cur]; if (cur && cur.scrollIntoView) cur.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  }
  function lastSaved() {
    let at = '';
    Object.keys(S.checks).forEach((g) => { const a = S.checks[g] && S.checks[g].at; if (typeof a === 'string' && a > at) at = a; });
    S.notes.forEach((n) => { if (typeof n.at === 'string' && n.at > at) at = n.at; });
    return at;
  }
  function renderSaved() {
    const box = $('saved'); box.textContent = '';
    box.classList.toggle('off', !S.db);
    if (!S.db) { box.appendChild(el('b', '', '저장 안 됨')); box.appendChild(document.createTextNode(' 이 화면에서는 확인 칸과 메모를 저장하지 못합니다')); return; }
    const at = lastSaved();
    if (at) { box.appendChild(el('b', '', hm(at))); box.appendChild(document.createTextNode(' 확인 칸·메모 저장')); }
    else box.appendChild(document.createTextNode('확인 칸과 메모는 이 안내에 저장되고 Claude가 읽습니다'));
  }
  function renderBar() {
    if (S.view === 'step' && S.guide) {
      const p = guideProgress(S.gid);
      $('gTitle').textContent = S.guide.title; $('gSub').textContent = S.guide.sub;
      $('prog').hidden = !p.work;
      $('progText').textContent = (S.guide.kind === 'print' ? '뽑은 판 ' : '끝낸 단계 ') + p.done + ' / ' + p.work;
      $('pbarFill').style.width = (p.frac * 100) + '%';
    } else {
      $('gTitle').textContent = '작업 안내'; $('gSub').textContent = '조립, 출력, 재기, 배선, 설계 보기';
      $('prog').hidden = true;
    }
    renderSaved();
  }
  function menuItem(k, n, p, on, act) {
    const it = el('div', 'gm-item' + (on ? ' on' : '')); it.setAttribute('role', 'menuitem'); it.tabIndex = 0;
    it.appendChild(el('span', 'k', k)); it.appendChild(el('span', 'n', n)); it.appendChild(el('span', 'p', p));
    it.addEventListener('click', () => { if (noSelection()) { closeMenu(); act(); } });
    it.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); closeMenu(); act(); } });
    return it;
  }
  function kindName(kind) { const k = KINDS.find((x) => x[0] === kind); return k ? k[1] : kind; }
  function renderMenu() {
    const m = $('gMenu'); m.textContent = '';
    m.appendChild(el('p', 'gm-lab', '안내 · 고르면 아직 안 끝낸 첫 단계로 갑니다'));
    ((S.catalog && S.catalog.guides) || []).forEach((row) => {
      const on = S.view === 'step' && row.id === S.gid;
      m.appendChild(menuItem(kindName(row.kind), row.title, guideProgress(row.id).text, on, () => go(row.id)));
    });
    m.appendChild(el('div', 'gm-sep'));
    m.appendChild(menuItem('', '모든 안내 보기 (처음 화면)', '', false, showHome));
  }
  function openMenu() { renderMenu(); $('gMenu').hidden = false; $('gBtn').setAttribute('aria-expanded', 'true'); }
  function closeMenu() { $('gMenu').hidden = true; $('gBtn').setAttribute('aria-expanded', 'false'); }
  function renderHome() {
    const kinds = $('kinds'); kinds.textContent = '';
    const rows = (S.catalog && S.catalog.guides) || [];
    KINDS.forEach(([kind, name]) => {
      const sec = el('section', 'kind');
      sec.appendChild(el('h2', '', name));
      const mine = rows.filter((r) => r.kind === kind);
      if (!mine.length) sec.appendChild(el('p', 'empty', '아직 없습니다.'));
      mine.forEach((row) => {
        const pr = guideProgress(row.id);
        const card = el('div', 'gcard'); card.tabIndex = 0; card.setAttribute('role', 'button');
        const top = el('div', 'row'); top.appendChild(el('span', 'gt', row.title)); top.appendChild(el('span', 'st', pr.text));
        card.appendChild(top);
        card.appendChild(el('span', 'gs', row.sub));
        const bar = el('div', 'mini'); const fill = el('i'); fill.style.width = (pr.frac * 100) + '%'; bar.appendChild(fill); card.appendChild(bar);
        card.addEventListener('click', () => { if (noSelection()) go(row.id); });
        card.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); go(row.id); } });
        sec.appendChild(card);
      });
      kinds.appendChild(sec);
    });
    // 이어서 하기: 확인 칸이나 판 기록이 마지막으로 바뀐 안내 가운데 아직 다 하지 않은 것
    const cand = rows.map((r) => ({ r: r, p: guideProgress(r.id), at: lastActivity(r.id) }))
      .filter((x) => x.p.started && x.p.next)
      .sort((a, b) => b.at.localeCompare(a.at));
    const top = cand[0] || null;
    S.resumeGid = top ? top.r.id : '';
    $('resume').hidden = !top;
    if (top) { $('resumeTitle').textContent = top.r.title; $('resumeStep').textContent = stepName(top.p.next); }
    $('madeAt').textContent = S.catalog ? '안내를 만든 때 ' + hm(S.catalog.made) : '';
  }

  // ---------- 3D ----------
  let G = null;
  const MESH = {};
  const COLORS = { body: 0xf1f1ee, cap: 0x2b2e33, frame: 0x3a4250, metal: 0xb7bec7, servo: 0x30343a, plug: 0xf3efe4, pin: 0xe0a030, wire: 0xd4372c };
  function b64(s) { const bin = atob(s); const out = new Uint8Array(bin.length); for (let k = 0; k < bin.length; k += 1) out[k] = bin.charCodeAt(k); return out; }
  function decodeMesh(m) {
    const T = window.THREE;
    const vb = b64(m.v), ib = b64(m.i);
    const ints = new Int16Array(vb.buffer, 0, vb.byteLength >> 1);
    const pos = new Float32Array(ints.length);
    for (let k = 0; k < ints.length; k += 1) pos[k] = ints[k] * m.q;
    const idx = m.it === 'u16' ? new Uint16Array(ib.buffer, 0, ib.byteLength >> 1) : new Uint32Array(ib.buffer, 0, ib.byteLength >> 2);
    const g = new T.BufferGeometry();
    g.setAttribute('position', new T.BufferAttribute(pos, 3));
    g.setIndex(new T.BufferAttribute(idx, 1));
    g.computeVertexNormals(); g.computeBoundingBox();
    return { g: g, e: new T.EdgesGeometry(g, 28), box: g.boundingBox.clone() };
  }
  function freeMeshes() { Object.keys(MESH).forEach((id) => { MESH[id].g.dispose(); MESH[id].e.dispose(); delete MESH[id]; }); }
  function newMat(c) {
    const T = window.THREE;
    return new T.MeshStandardMaterial({ color: COLORS[c] || 0x888888, roughness: c === 'metal' ? 0.4 : 0.7, metalness: c === 'metal' ? 0.35 : 0, side: T.DoubleSide });
  }
  function cssColor(name, fb) { const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim(); return v || fb; }
  function holdQuat(v) {
    const T = window.THREE;
    const u = new T.Vector3().fromArray(v.up).normalize();
    const fc = new T.Vector3().fromArray(v.face);
    fc.addScaledVector(u, -fc.dot(u)).normalize();
    const r = new T.Vector3().crossVectors(u, fc);
    const m = new T.Matrix4().set(r.x, r.y, r.z, 0, u.x, u.y, u.z, 0, fc.x, fc.y, fc.z, 0, 0, 0, 0, 1);
    return new T.Quaternion().setFromRotationMatrix(m);
  }
  function ease(k) { return k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2; }
  function deg(a) { return a * Math.PI / 180; }

  function init3d() {
    if (G) return true;
    const T = window.THREE;
    if (!T || !T.OrbitControls) { $('nogl').hidden = false; return false; }
    try {
      const canvas = $('c'), stage = $('stage');
      const renderer = new T.WebGLRenderer({ canvas: canvas, antialias: true, preserveDrawingBuffer: SELFTEST });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      renderer.localClippingEnabled = true;
      const scene = new T.Scene();
      const camera = new T.PerspectiveCamera(30, 1.6, 2, 6000);
      const controls = new T.OrbitControls(camera, canvas);
      controls.enableDamping = true; controls.dampingFactor = 0.12; controls.minDistance = 25; controls.maxDistance = 1500;
      scene.add(new T.HemisphereLight(0xffffff, 0x8a94a3, 0.8));
      const sun = new T.DirectionalLight(0xffffff, 0.55); sun.position.set(-160, 260, 320); scene.add(sun);
      const fill = new T.DirectionalLight(0xffffff, 0.22); fill.position.set(220, -120, -240); scene.add(fill);
      const root = new T.Group(); scene.add(root);
      const spin = new T.Group(); root.add(spin);  // 자식 쪽 부품: 관절 각만큼 출력축(y) 둘레로 돈다
      // 둘째 관절(자식 몸통 속의 관절, 예: J5 장면의 J6)의 부품: 첫째 관절과 같이 돌고, 안내 데이터의 axis2 둘레로 또 돈다
      const spin2 = new T.Group(); spin.add(spin2);
      G = {
        T: T, renderer: renderer, scene: scene, camera: camera, controls: controls, root: root, spin: spin, spin2: spin2, axis2: new T.Vector3(1, 0, 0), stage: stage, canvas: canvas, items: [],
        clip: new T.Plane(new T.Vector3(0, 0, -1), 0.01),
        arrowMat: new T.MeshBasicMaterial({ color: 0xe0560b, depthTest: false, transparent: true, opacity: 0.92 }),
        spotMat: new T.MeshBasicMaterial({ color: 0xe0560b, depthTest: false, transparent: true, opacity: 0.95 }),
        arrows: [], spots: [], tags: [], moves: [], tl: null, tween: null, clock: 0, playing: false, doneShown: false, lastNow: 0, lastPct: -1, scrubbing: false,
        vis: new Set(), hot: new Set(), shell: new Set(), cutOn: false, ghostOn: false, insets: { l: 0, r: 0, t: 0, b: 0 }, step: null, framed: false, joint: 0,
      };
      applyTheme3d();
      const resize = () => {
        const w = stage.clientWidth, h = stage.clientHeight;
        if (w < 2 || h < 2) return;
        renderer.setSize(w, h, false); camera.aspect = w / h;
        computeInsets();
      };
      G.resize = resize;
      if (window.ResizeObserver) new ResizeObserver(() => { resize(); padBoard(); }).observe(stage);
      window.addEventListener('resize', resize);
      resize();
      canvas.addEventListener('dblclick', hideAt);
      const mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
      if (mq && mq.addEventListener) mq.addEventListener('change', applyTheme3d);
      new MutationObserver(applyTheme3d).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
      if (!SELFTEST) requestAnimationFrame(frame);
      return true;
    } catch (err) {
      $('nogl').hidden = false; G = null;
      if (window.console) console.error(err);
      return false;
    }
  }
  function applyTheme3d() {
    if (!G) return;
    const T = G.T;
    G.stageColor = new T.Color(cssColor('--stage', '#e1e6ec'));
    G.edgeColor = new T.Color(cssColor('--edge', '#6f7a87'));
    G.hotColor = new T.Color(cssColor('--hot', '#e0560b'));
    G.arrowMat.color.copy(G.hotColor); G.spotMat.color.copy(G.hotColor);
    if (G.step) styleAll();
  }
  function clearItems() {
    if (!G) return;
    G.items.forEach((it) => { if (it.mesh.parent) it.mesh.parent.remove(it.mesh); it.mat.dispose(); it.emat.dispose(); if (it.flex) it.flex.g.dispose(); });
    G.items = []; G.step = null;
    clearDecor();
  }
  // 관절 각에 따라 모양이 바뀌는 납작한 선 (안내 데이터의 flex): 관절 각마다의 (x, z) 꺾은선 사이를 이어, 단면이
  // 폭 w × 두께 t인 띠로 그린다. 앞쪽 s[0] mm까지는 눕고(높이 y[0]), s[1] mm부터는 세워진다(폭이 y 방향, 높이 y[1])
  function makeFlex(fx) {
    const T = window.THREE, n = fx.p[0].length / 2;
    const g = new T.BufferGeometry();
    g.setAttribute('position', new T.BufferAttribute(new Float32Array(n * 8 * 3), 3));
    const idx = [];
    for (let i = 0; i < n - 1; i += 1) {
      for (let f = 0; f < 4; f += 1) { const a = 8 * i + 2 * f; idx.push(a, a + 8, a + 1, a + 1, a + 8, a + 9); }
    }
    g.setIndex(idx);
    const box = new T.Box3(new T.Vector3().fromArray(fx.box[0]), new T.Vector3().fromArray(fx.box[1]));
    const F = { fx: fx, g: g, n: n, xz: new Float32Array(2 * n), box: box, at: null };
    shapeFlex(F, 0);
    return F;
  }
  const FLEX_CORNERS = [[1, 1], [-1, 1], [-1, -1], [1, -1]];  // (두께 쪽, 폭 쪽) 부호
  function shapeFlex(F, a) {
    if (F.at === a) return;
    F.at = a;
    const fx = F.fx, A = fx.a, n = F.n, xz = F.xz;
    let k = 0, f = 0;
    if (a >= A[A.length - 1]) k = A.length - 1;
    else if (a > A[0]) { while (k < A.length - 2 && A[k + 1] <= a) k += 1; f = (a - A[k]) / (A[k + 1] - A[k]); }
    const p0 = fx.p[k], p1 = fx.p[Math.min(k + 1, A.length - 1)];
    for (let j = 0; j < 2 * n; j += 1) xz[j] = p0[j] + (p1[j] - p0[j]) * f;
    const pos = F.g.attributes.position.array, hw = fx.w / 2, ht = fx.t / 2;
    let s = 0;
    for (let i = 0; i < n; i += 1) {
      const i0 = Math.max(0, i - 1), i1 = Math.min(n - 1, i + 1);
      let tx = xz[2 * i1] - xz[2 * i0], tz = xz[2 * i1 + 1] - xz[2 * i0 + 1];
      const tl = Math.hypot(tx, tz) || 1; tx /= tl; tz /= tl;
      if (i > 0) s += Math.hypot(xz[2 * i] - xz[2 * i - 2], xz[2 * i + 1] - xz[2 * i - 1]);
      let e = (s - fx.s[0]) / (fx.s[1] - fx.s[0]); e = e < 0 ? 0 : e > 1 ? 1 : e; e = e * e * (3 - 2 * e);
      const c = Math.cos(e * Math.PI / 2), sn = Math.sin(e * Math.PI / 2), nx = -tz, nz = tx;
      const ax = sn * nx, ay = c, az = sn * nz;  // 두께 쪽: 누우면 y, 세우면 면 안의 법선
      const bx = c * nx, by = -sn, bz = c * nz;  // 폭 쪽
      const x = xz[2 * i], y = fx.y[0] + (fx.y[1] - fx.y[0]) * e, z = xz[2 * i + 1];
      for (let q = 0; q < 4; q += 1) {
        for (let v = 0; v < 2; v += 1) {
          const cc = FLEX_CORNERS[(q + v) % 4], o = (8 * i + 2 * q + v) * 3;
          pos[o] = x + cc[0] * ht * ax + cc[1] * hw * bx;
          pos[o + 1] = y + cc[0] * ht * ay + cc[1] * hw * by;
          pos[o + 2] = z + cc[0] * ht * az + cc[1] * hw * bz;
        }
      }
    }
    F.g.attributes.position.needsUpdate = true;
    F.g.computeVertexNormals(); F.g.computeBoundingSphere();
  }
  function setJoint(a) {
    G.spin.rotation.set(0, -deg(a), 0);
    G.items.forEach((it) => { if (it.flex) shapeFlex(it.flex, a); });
  }
  function setJoint2(b) { G.spin2.quaternion.setFromAxisAngle(G.axis2, -deg(b)); }
  function buildItems() {
    const T = G.T;
    clearItems();
    // 둘째 관절의 축: 지나는 점에 spin2를 두고, 그 부품은 그 점에서 잰 자리에 놓는다
    const ax = S.guide.axis2;
    G.spin2.position.set(0, 0, 0); G.spin2.quaternion.identity(); G.axis2.set(1, 0, 0);
    if (ax) { G.spin2.position.fromArray(ax.o); G.axis2.fromArray(ax.d).normalize(); }
    G.items = S.guide.scene.map((p, i) => {
      const flex = p.flex ? makeFlex(p.flex) : null;
      const M = flex || MESH[p.mesh];
      const mat = newMat(p.c), mesh = new T.Mesh(M.g, mat);
      const emat = new T.LineBasicMaterial({ color: 0x6f7a87 });
      if (!flex) mesh.add(new T.LineSegments(M.e, emat));
      const lv = p.m === 2 ? 2 : p.m ? 1 : 0;
      mesh.userData.i = i; (lv === 2 ? G.spin2 : lv ? G.spin : G.root).add(mesh);
      return { i: i, p: p, lv: lv, off: lv === 2 ? G.spin2.position.clone() : new T.Vector3(), mesh: mesh, mat: mat, emat: emat, hidden: false,
        t: new T.Vector3().fromArray(p.t), ctr: M.box.getCenter(new T.Vector3()), box: M.box, basePos: new T.Vector3(), baseQuat: new T.Quaternion(), flex: flex };
    });
  }
  function styleAll() {
    G.items.forEach((it) => {
      it.mesh.visible = G.vis.has(it.i) && !it.hidden;
      const hot = G.hot.has(it.i), inShell = G.shell.has(it.i);
      const cut = G.cutOn && inShell, ghost = G.ghostOn && inShell;
      const planes = cut ? [G.clip] : [];
      it.mat.clippingPlanes = planes; it.emat.clippingPlanes = planes;
      if (hot) it.mat.emissive.copy(G.hotColor).multiplyScalar(0.35); else it.mat.emissive.setHex(0x000000);
      it.emat.color.copy(hot ? G.hotColor : G.edgeColor);
      it.mat.transparent = ghost; it.mat.opacity = ghost ? 0.16 : 1; it.mat.depthWrite = !ghost;
      it.emat.transparent = ghost; it.emat.opacity = ghost ? 0.4 : 1;
      it.mat.needsUpdate = true; it.emat.needsUpdate = true;
    });
  }
  function makeArrow(a, b) {
    const T = G.T;
    const dir = new T.Vector3().subVectors(b, a); const len = dir.length(); dir.normalize();
    const headLen = Math.min(5, len * 0.4), shaftLen = Math.max(0.1, len - headLen);
    const g = new T.Group();
    const shaft = new T.Mesh(new T.CylinderGeometry(0.6, 0.6, shaftLen, 12), G.arrowMat); shaft.position.y = shaftLen / 2; shaft.renderOrder = 5;
    const head = new T.Mesh(new T.ConeGeometry(1.9, headLen, 18), G.arrowMat); head.position.y = shaftLen + headLen / 2; head.renderOrder = 5;
    g.add(shaft); g.add(head);
    g.position.copy(a); g.quaternion.setFromUnitVectors(new T.Vector3(0, 1, 0), dir);
    return g;
  }
  function clearDecor() {
    G.arrows.forEach((a) => { if (a.parent) a.parent.remove(a); a.traverse((o) => { if (o.geometry) o.geometry.dispose(); }); });
    G.arrows = [];
    G.spots.forEach((s) => { if (s.parent) s.parent.remove(s); s.geometry.dispose(); });
    G.spots = [];
    $('tags').textContent = ''; G.tags = [];
  }
  function apply3d(step, instant) {
    if (!G) return;
    const T = G.T;
    G.step = step;
    G.vis = new Set(step.show); G.hot = new Set(step.hot); G.shell = new Set(step.shell);
    G.cutOn = !!step.cut; G.ghostOn = !!step.ghost;
    const places = {}; step.place.forEach((pl) => { places[pl.p] = pl; });
    G.items.forEach((it) => {
      it.hidden = false;
      const pl = places[it.i];
      if (pl) { it.baseQuat.copy(holdQuat(pl)); it.basePos.fromArray(pl.pos).sub(it.ctr.clone().applyQuaternion(it.baseQuat)).sub(it.off); }
      else { it.baseQuat.identity(); it.basePos.copy(it.t).sub(it.off); }
      it.mesh.quaternion.copy(it.baseQuat); it.mesh.position.copy(it.basePos);
    });
    G.joint = step.joint || 0;
    setJoint(G.joint);
    setJoint2(step.joint2 || 0);
    G.moves = step.moves.map((m) => ({ idx: m.p, by: new T.Vector3().fromArray(m.by), delay: m.delay || 0 }));
    const last = G.moves.reduce((a, m) => Math.max(a, m.delay + MOVE_S), 0);
    // 돌리기: 첫째 관절(turn)을 돌린 뒤 둘째 관절(turn2)을 돌린다
    let turn = null, turn2 = null, end = last;
    if (step.turn) { turn = { a: step.turn[0], b: step.turn[1], t0: end, t1: end + TURN_S }; end = turn.t1; }
    if (step.turn2) { turn2 = { a: step.turn2[0], b: step.turn2[1], t0: end, t1: end + TURN_S }; end = turn2.t1; }
    G.tl = end > 0 ? { start: LEAD_S, end: LEAD_S + end, period: LEAD_S + end + HOLD_S, turn: turn, turn2: turn2 } : null;
    G.playing = !!G.tl && !REDUCED && !SELFTEST;
    G.clock = G.tl && (REDUCED || SELFTEST) ? G.tl.end : 0;
    G.scrubbing = false; G.lastPct = -1;
    clearDecor();
    G.moves.forEach((m) => {
      const first = G.items[m.idx[0]];
      const c = new T.Vector3();
      m.idx.forEach((i) => { const it = G.items[i]; c.add(it.ctr.clone().applyQuaternion(it.baseQuat).add(it.basePos)); });
      c.multiplyScalar(1 / Math.max(1, m.idx.length));
      const a = makeArrow(c.clone().addScaledVector(m.by, 1), c.clone().addScaledVector(m.by, 0.28));
      first.mesh.parent.add(a); G.arrows.push(a);
    });
    step.spots.forEach((sp) => {
      const it = G.items[sp.p]; if (!it) return;
      sp.at.forEach((at) => {
        const m = new T.Mesh(new T.SphereGeometry(1.6, 16, 12), G.spotMat);
        m.renderOrder = 6; m.position.fromArray(at).sub(it.t); m.userData.label = sp.label;
        it.mesh.add(m); G.spots.push(m);
      });
    });
    buildTags(step);
    styleAll();
    updateTools();
    updatePlayer();
    G.doneShown = false; $('doneTag').hidden = true;
    animate();
    reframe(instant);
  }
  function buildTags(step) {
    const T = G.T, box = $('tags');
    step.labels.forEach((l) => {
      const it = G.items[l.p]; if (!it) return;
      const e = el('div', 'tag hot', l.text); box.appendChild(e);
      const local = l.at ? new T.Vector3().fromArray(l.at).sub(it.t) : it.ctr.clone();
      G.tags.push({ el: e, at: () => it.mesh.parent.localToWorld(local.clone().applyQuaternion(it.baseQuat).add(it.basePos)), dy: -10 });
    });
    step.tags.forEach((tg) => {
      const e = el('div', 'tag dir', tg.text); box.appendChild(e);
      const v = new T.Vector3().fromArray(tg.at);
      G.tags.push({ el: e, at: () => G.root.localToWorld(v.clone()), dy: 0 });
    });
    G.spots.forEach((m) => {
      const e = el('div', 'tag spot', m.userData.label); box.appendChild(e);
      G.tags.push({ el: e, at: () => m.getWorldPosition(new T.Vector3()), dy: -8 });
    });
  }
  function placeTags() {
    const w = G.stage.clientWidth, h = G.stage.clientHeight;
    G.tags.forEach((t) => {
      const v = t.at().project(G.camera);
      if (v.z > 1 || v.z < -1) { t.el.style.visibility = 'hidden'; return; }
      t.el.style.visibility = '';
      const x = (v.x + 1) / 2 * w, y = (1 - v.y) / 2 * h + t.dy;
      t.el.style.transform = 'translate(' + x.toFixed(1) + 'px,' + y.toFixed(1) + 'px) translate(-50%,-100%)';
    });
  }
  function computeInsets() {
    if (!G) return;
    const ins = panelInsets();
    if (!PHONE.matches && G.step && G.step.kind !== 'prep') {
      const s = G.stage.getBoundingClientRect();
      const tops = ['tools3d', 'player'].map((id) => $(id)).filter((e) => !e.hidden && e.getClientRects().length).map((e) => e.getBoundingClientRect().bottom);
      if (tops.length) ins.t = Math.max(0, Math.max.apply(null, tops) - s.top + 8);
      if (s.width - ins.l - ins.r < 240) { ins.l = 0; ins.r = 0; }
      if (s.height - ins.t - ins.b < 200) { ins.t = 0; ins.b = 0; }
    }
    G.insets = ins;
    const W = Math.max(2, G.stage.clientWidth), H = Math.max(2, G.stage.clientHeight);
    if (ins.l || ins.r || ins.t || ins.b) G.camera.setViewOffset(W, H, -(ins.l - ins.r) / 2, -(ins.t - ins.b) / 2, W, H);
    else G.camera.clearViewOffset();
    G.camera.updateProjectionMatrix();
  }
  function fitView(step, q) {
    const T = G.T, box = new T.Box3(), v = new T.Vector3();
    const angles = step.turn ? [step.turn[0], step.turn[1], 0] : [step.joint || 0];
    const spins = angles.map((a) => new T.Quaternion().setFromAxisAngle(new T.Vector3(0, 1, 0), -deg(a)));
    const angles2 = step.turn2 ? [step.turn2[0], step.turn2[1], 0] : [step.joint2 || 0];
    const spins2 = angles2.map((b) => new T.Quaternion().setFromAxisAngle(G.axis2, -deg(b)));
    const still = [new T.Quaternion()];
    G.items.forEach((it) => {
      if (!G.vis.has(it.i)) return;
      const offs = [new T.Vector3()];
      G.moves.forEach((m) => { if (m.idx.indexOf(it.i) >= 0) offs.push(m.by); });
      const rots = it.lv ? spins : still, rots2 = it.lv === 2 ? spins2 : still;
      for (let k = 0; k < 8; k += 1) {
        v.set(k & 1 ? it.box.max.x : it.box.min.x, k & 2 ? it.box.max.y : it.box.min.y, k & 4 ? it.box.max.z : it.box.min.z);
        v.applyQuaternion(it.baseQuat).add(it.basePos);
        offs.forEach((o) => rots2.forEach((r2) => {
          const w = v.clone().add(o).applyQuaternion(r2).add(it.off);
          rots.forEach((r) => box.expandByPoint(w.clone().applyQuaternion(r).applyQuaternion(q)));
        }));
      }
    });
    if (box.isEmpty()) box.set(new T.Vector3(-50, -50, -50), new T.Vector3(50, 50, 50));
    const sph = box.getBoundingSphere(new T.Sphere());
    const W = Math.max(2, G.stage.clientWidth), H = Math.max(2, G.stage.clientHeight), ins = G.insets;
    const fw = Math.max(80, W - ins.l - ins.r), fh = Math.max(80, H - ins.t - ins.b);
    const tv = Math.tan(G.camera.fov * Math.PI / 360);
    const fov = Math.min(2 * Math.atan(tv * fh / H), 2 * Math.atan(tv * fw / H));
    const dist = sph.radius / Math.sin(fov / 2) * 0.9 / (step.zoom > 0 ? step.zoom : 1);
    const view = step.view || {};
    const az = deg(view.az || 0), elv = deg(view.el == null ? 20 : view.el);
    const dir = new T.Vector3(Math.sin(az) * Math.cos(elv), Math.sin(elv), Math.cos(az) * Math.cos(elv));
    const target = step.center ? new T.Vector3().fromArray(step.center).applyQuaternion(q) : sph.center.clone();
    return { target: target, pos: target.clone().addScaledVector(dir, dist) };
  }
  function reframe(instant) {
    if (!G || !G.step) return;
    computeInsets();
    const q = holdQuat(G.step.view || { up: [0, 1, 0], face: [0, 0, 1] });
    const fit = fitView(G.step, q);
    const jump = instant || REDUCED || SELFTEST || !G.framed;
    G.framed = true;
    G.tween = { from: { q: G.root.quaternion.clone(), target: G.controls.target.clone(), pos: G.camera.position.clone() },
      to: { q: q, target: fit.target, pos: fit.pos }, t0: performance.now(), dur: jump ? 0 : 650 };
    G.controls.enabled = false;
    if (jump) stepTween(performance.now());
  }
  function stepTween(now) {
    const tw = G.tween; if (!tw) return;
    const k = tw.dur ? Math.min(1, (now - tw.t0) / tw.dur) : 1, e = ease(k);
    G.root.quaternion.copy(tw.from.q).slerp(tw.to.q, e);
    G.controls.target.lerpVectors(tw.from.target, tw.to.target, e);
    G.camera.position.lerpVectors(tw.from.pos, tw.to.pos, e);
    G.camera.lookAt(G.controls.target);
    if (k >= 1) { G.tween = null; G.controls.enabled = true; }
  }

  // ---------- 움직임: 재생, 한 칸씩, 끌어 보기 ----------
  // 시계(clock)는 한 바퀴(시작 멈춤 → 끼우기와 돌리기 → 다 된 모습) 안의 초. 막대의 0~100%는 끼우기와 돌리기 구간이다
  const SVG_PLAY = '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M3 1.8l7 4.2-7 4.2z"/></svg>';
  const SVG_PAUSE = '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M2.5 1.8h2.6v8.4H2.5zM6.9 1.8h2.6v8.4H6.9z"/></svg>';
  $('pBack').innerHTML = '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M9.6 1.8L3.2 6l6.4 4.2z"/><path d="M1.6 1.8H3v8.4H1.6z"/></svg>한 칸<kbd>,</kbd>';
  $('pFwd').innerHTML = '한 칸<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M2.4 1.8L8.8 6l-6.4 4.2z"/><path d="M9 1.8h1.4v8.4H9z"/></svg><kbd>.</kbd>';
  function progress() { const tl = G.tl; return Math.max(0, Math.min(1, (G.clock - tl.start) / (tl.end - tl.start))); }
  function setProgress(p) {
    if (!G || !G.tl) return;
    G.playing = false;
    G.clock = G.tl.start + Math.max(0, Math.min(1, p)) * (G.tl.end - G.tl.start);
    updatePlayer();
  }
  function stepFrame(d) { if (!G || !G.tl) return; setProgress((Math.round(progress() * NOTCHES) + d) / NOTCHES); }
  function updatePlayer() {
    const has = !!(G && G.tl) && !$('c').hidden;
    $('player').hidden = !has;
    if (!has) return;
    $('pPlay').innerHTML = G.playing ? SVG_PAUSE + '멈추기' : SVG_PLAY + '움직이기';
    document.querySelectorAll('.speed button').forEach((b) => b.setAttribute('aria-pressed', Number(b.dataset.speed) === S.speed ? 'true' : 'false'));
    syncPlayer(true);
  }
  function syncPlayer(force) {
    if (!G || !G.tl) return;
    const p = progress(), v = Math.round(p * 1000), pc = Math.round(p * 100), s = $('pScrub');
    if (!G.scrubbing && (force || Number(s.value) !== v)) s.value = String(v);
    if (force || pc !== G.lastPct) { G.lastPct = pc; $('pPct').textContent = pc + '%'; }
  }
  $('pPlay').addEventListener('click', () => {
    if (!G || !G.tl) return;
    if (!G.playing && progress() >= 1) G.clock = 0;
    G.playing = !G.playing;
    updatePlayer();
  });
  $('pBack').addEventListener('click', () => stepFrame(-1));
  $('pFwd').addEventListener('click', () => stepFrame(1));
  $('pDone').addEventListener('click', () => setProgress(1));
  const scrub = $('pScrub');
  scrub.addEventListener('input', () => { if (!G) return; G.scrubbing = true; setProgress(Number(scrub.value) / 1000); });
  scrub.addEventListener('change', () => { if (G) G.scrubbing = false; });
  scrub.addEventListener('pointerup', () => { if (G) G.scrubbing = false; scrub.blur(); });
  document.querySelectorAll('.speed button').forEach((b) => b.addEventListener('click', () => { S.speed = Number(b.dataset.speed); updatePlayer(); }));

  function animate() {
    const tl = G.tl;
    if (!tl) { if (G.doneShown) { G.doneShown = false; $('doneTag').hidden = true; } G.arrows.forEach((a) => { a.visible = false; }); return; }
    const t = G.clock;
    G.moves.forEach((m) => {
      let k = (t - tl.start - m.delay) / MOVE_S; k = k < 0 ? 0 : k > 1 ? 1 : k;
      const e = ease(k);
      m.idx.forEach((i) => { const it = G.items[i]; it.mesh.position.copy(it.basePos).addScaledVector(m.by, 1 - e); });
    });
    if (tl.turn) {
      let k = (t - tl.start - tl.turn.t0) / (tl.turn.t1 - tl.turn.t0); k = k < 0 ? 0 : k > 1 ? 1 : k;
      setJoint(tl.turn.a + (tl.turn.b - tl.turn.a) * ease(k));
    }
    if (tl.turn2) {
      let k = (t - tl.start - tl.turn2.t0) / (tl.turn2.t1 - tl.turn2.t0); k = k < 0 ? 0 : k > 1 ? 1 : k;
      setJoint2(tl.turn2.a + (tl.turn2.b - tl.turn2.a) * ease(k));
    }
    const done = t >= tl.end - 1e-6;
    G.arrows.forEach((a) => { a.visible = !done; });
    if (done !== G.doneShown) { G.doneShown = done; $('doneTag').hidden = !done; }
    syncPlayer(false);
  }
  function renderOnce() {
    G.renderer.setClearColor(G.stageColor, 1);
    G.renderer.render(G.scene, G.camera);
    placeTags();
  }
  function frame(now) {
    requestAnimationFrame(frame);
    if (!G) return;
    const dt = G.lastNow ? Math.min(0.1, (now - G.lastNow) / 1000) : 0;
    G.lastNow = now;
    if (!G.step) return;
    const visible = G.stage.offsetParent !== null && !$('c').hidden;
    stepTween(now);
    if (!G.tween) G.controls.update();
    if (G.tl && G.playing) G.clock = (G.clock + dt * S.speed) % G.tl.period;
    animate();
    if (visible) renderOnce();
  }
  function hideAt(ev) {
    if (!G) return;
    const T = G.T, r = G.canvas.getBoundingClientRect();
    const ndc = new T.Vector2((ev.clientX - r.left) / r.width * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    const rc = new T.Raycaster(); rc.setFromCamera(ndc, G.camera);
    const hits = rc.intersectObjects(G.items.filter((it) => it.mesh.visible).map((it) => it.mesh), false);
    for (let k = 0; k < hits.length; k += 1) {
      const it = G.items[hits[k].object.userData.i];
      if (it.mat.clippingPlanes.length && G.clip.distanceToPoint(hits[k].point) < 0) continue;
      if (it.mat.transparent) continue;
      it.hidden = true; styleAll(); updateTools(); toast('숨김: ' + it.p.n);
      return;
    }
  }
  function updateTools() {
    if (!G) return;
    const hasShell = G.shell.size > 0, nHidden = G.items.filter((it) => it.hidden).length;
    $('tCut').hidden = !hasShell; $('tGhost').hidden = !hasShell;
    $('tCut').setAttribute('aria-pressed', G.cutOn ? 'true' : 'false');
    $('tGhost').setAttribute('aria-pressed', G.ghostOn ? 'true' : 'false');
    $('tUnhide').hidden = nHidden === 0; $('tUnhide').textContent = '숨긴 부품 보이기 (' + nHidden + ')';
  }
  $('tCut').addEventListener('click', () => { if (!G) return; G.cutOn = !G.cutOn; if (G.cutOn) G.ghostOn = false; styleAll(); updateTools(); });
  $('tGhost').addEventListener('click', () => { if (!G) return; G.ghostOn = !G.ghostOn; if (G.ghostOn) G.cutOn = false; styleAll(); updateTools(); });
  $('tUnhide').addEventListener('click', () => { if (!G) return; G.items.forEach((it) => { it.hidden = false; }); styleAll(); updateTools(); });
  $('tView').addEventListener('click', () => reframe(false));
  $('tPlate').addEventListener('click', () => { const st = stepRef(); if (st && st.plate) { S.platePart = null; showPlateStage(st); renderPlateParts(st); } });
  $('tBare').addEventListener('click', toggleBare);
  $('tLabels').addEventListener('click', toggleLabels);
  setLabels(recall('nolabels') === true);

  // ---------- 작은 그림 (부품 목록의 그림을 이 안내의 형상으로 그린다) ----------
  function makeThumbs() {
    const T = window.THREE; if (!T || !S.guide) return;
    let r;
    try { r = new T.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true }); } catch (e) { return; }
    const W = 256, H = 192;
    r.setPixelRatio(1); r.setSize(W, H, false); r.setClearColor(0x000000, 0);
    const scene = new T.Scene();
    scene.add(new T.HemisphereLight(0xffffff, 0x8a94a3, 0.85));
    const sun = new T.DirectionalLight(0xffffff, 0.6); sun.position.set(-160, 260, 320); scene.add(sun);
    const cam = new T.PerspectiveCamera(26, W / H, 1, 5000);
    const edgeMat = new T.LineBasicMaterial({ color: 0x59636f });
    Object.keys(S.guide.items).forEach((key) => {
      const it = S.guide.items[key]; if (!it.thumb) return;
      const g = new T.Group(), mats = [];
      it.thumb.p.forEach((i) => {
        const p = S.guide.scene[i], M = MESH[p.mesh]; if (!M) return;
        const m = newMat(p.c); mats.push(m);
        const mesh = new T.Mesh(M.g, m); mesh.add(new T.LineSegments(M.e, edgeMat)); mesh.position.fromArray(p.t); g.add(mesh);
      });
      const tv = it.thumb;
      g.quaternion.copy(holdQuat(tv)); scene.add(g);
      const sph = new T.Box3().setFromObject(g).getBoundingSphere(new T.Sphere());
      const elv = deg(tv.el == null ? 28 : tv.el), az = deg(tv.az || 0);
      const dir = new T.Vector3(Math.sin(az) * Math.cos(elv), Math.sin(elv), Math.cos(az) * Math.cos(elv));
      const fov = Math.min(cam.fov * Math.PI / 180, 2 * Math.atan(Math.tan(cam.fov * Math.PI / 360) * cam.aspect));
      cam.position.copy(sph.center).addScaledVector(dir, sph.radius / Math.sin(fov / 2) * 0.95); cam.lookAt(sph.center);
      r.render(scene, cam);
      THUMBS[key] = r.domElement.toDataURL('image/png');
      scene.remove(g); mats.forEach((m) => m.dispose());
    });
    edgeMat.dispose(); r.dispose(); if (r.forceContextLoss) r.forceContextLoss();
    document.querySelectorAll('img[data-thumb]').forEach((img) => { const u = THUMBS[img.dataset.thumb]; if (u) img.src = u; });
  }

  // ---------- 저장 (아티팩트 db) ----------
  // 문서: checks/<안내>({marks: {단계 id: [확인 칸마다 참/거짓]}, at}), notes/<자동>, records/<자동>(Claude가 쓴다).
  // 단계를 넘겨 보기만 해서는 쓰지 않는다. 한 문서에는 한 번에 하나씩 쓰고, 쓰는 동안 또 바뀌면 끝난 뒤 마지막 것만 쓴다
  const writing = {};
  function writeDoc(path, data) {
    if (!S.db) return;
    const w = writing[path] || (writing[path] = { busy: false, pending: null });
    w.pending = data;
    if (!w.busy) flushDoc(path);
  }
  async function flushDoc(path) {
    const w = writing[path]; if (!w || !w.pending) return;
    const data = w.pending; w.pending = null; w.busy = true;
    try { await S.db.doc(path).set(data); }
    catch (e) { dbFailed(e); }
    finally { w.busy = false; if (w.pending) flushDoc(path); }
  }
  function dbFailed(e) {
    const code = e && e.code;
    if (code === 'revoked' || code === 'not_granted' || code === 'capability_disabled' || code === 'invalid_argument') {
      S.db = null; S.dbState = 'off'; renderSaved();
    }
    if (code === 'quota_exceeded') toast('저장 공간이 다 찼습니다. Claude에게 알려 주세요.');
    if (window.console) console.warn('db', e);
  }
  function setCheck(step, k, on) {
    const marks = Object.assign({}, marksOf(S.gid));
    const m = (marks[step.id] || []).slice(); while (m.length < step.check.length) m.push(false);
    m[k] = on; marks[step.id] = m;
    const doc = { marks: marks, at: new Date().toISOString() };
    S.checks[S.gid] = doc;
    writeDoc('checks/' + S.gid, doc);
    renderStrip(); renderBar();
  }
  async function addNote(step, ta) {
    const text = ta.value.trim();
    if (!text) { toast('메모가 비어 있습니다.'); return; }
    const note = { guide: S.gid, step: step.id, no: step.no, text: text, at: new Date().toISOString() };
    if (!S.db) { S.localNotes.push(note); ta.value = ''; store('draft.' + S.gid + '.' + step.id, ''); renderNotes(); toast('이 화면에만 남겼습니다 (저장 기능이 없습니다).'); return; }
    try {
      await S.db.collection('notes').add(note);
      ta.value = ''; store('draft.' + S.gid + '.' + step.id, '');
      toast('메모를 남겼습니다. Claude가 읽습니다.');
    } catch (e) { dbFailed(e); toast('메모를 저장하지 못했습니다. 글은 칸에 그대로 있습니다.'); }
  }
  async function deleteNote(id) {
    if (!S.db) return;
    try { await S.db.doc('notes/' + id).delete(); } catch (e) { dbFailed(e); }
  }
  async function initDb() {
    if (SELFTEST || !window.claude || typeof window.claude.use !== 'function') { renderSaved(); return; }
    let db = null;
    try { db = await window.claude.use('db'); } catch (e) { db = null; }
    if (!db) { renderSaved(); return; }
    S.db = db; S.dbState = 'on'; renderSaved();
    const err = (e) => dbFailed(e);
    db.collection('checks').onSnapshot((snap) => {
      const out = {}; snap.docs.forEach((d) => { out[d.id] = d.data(); }); S.checks = out;
      const st = stepRef();
      if (S.view === 'step' && st && !(document.activeElement && document.activeElement.type === 'checkbox')) renderChecks(st);
      renderSaved();
      if (S.view === 'home') renderHome(); else if (S.guide) { renderStrip(); renderBar(); }
    }, err);
    db.collection('notes').onSnapshot((snap) => { S.notes = snap.docs.map((d) => Object.assign({ id: d.id }, d.data())); renderNotes(); renderSaved(); }, err);
    db.collection('records').onSnapshot((snap) => {
      S.records = snap.docs.map((d) => Object.assign({ id: d.id }, d.data()));
      renderNotes();
      // 판을 뽑았다는 기록이 오면 진행, 판 상태, 준비물의 출력 부품 상태가 바뀐다
      const st = stepRef();
      if (S.view === 'home') renderHome();
      else if (S.guide && st) {
        renderStrip(); renderBar();
        if (st.plate) { renderPlateParts(st); renderPlateStatus(st); } else if (st.kind === 'prep') renderBoard();
      }
    }, err);
  }

  // ---------- 안내 열기, 단계 보이기 ----------
  let opening = null;
  async function openGuide(gid) {
    if (S.gid === gid && S.guide) return;
    const row = S.catalog.guides.find((r) => r.id === gid);
    if (!row) throw new Error('안내 ' + gid + '가 목록에 없습니다');
    $('loading').hidden = false;
    try {
      const data = await getJSON(row.file);
      if (G) clearItems();
      freeMeshes();
      Object.keys(THUMBS).forEach((k) => delete THUMBS[k]);
      if (window.THREE) {
        const meshes = await Promise.all(data.meshes.map((id) => getJSON('data/meshes/' + id + '.json')));
        meshes.forEach((m) => { MESH[m.id] = decodeMesh(m); });
      }
      S.guide = data; S.gid = gid;
      if (init3d()) buildItems();
      if (SELFTEST) makeThumbs(); else setTimeout(makeThumbs, 60);
    } finally { $('loading').hidden = true; }
  }
  function showStep(i, opts) {
    const o = opts || {};
    const steps = S.guide.steps;
    i = Math.max(0, Math.min(steps.length - 1, i));
    S.view = 'step'; S.cur = i; S.platePart = null;
    app.dataset.view = 'step';
    $('home').hidden = true; $('screen').hidden = false; $('nav').hidden = false;
    const step = steps[i];
    app.dataset.kind = step.kind;
    renderPanels(step);
    renderStrip();
    renderBar();
    const board = step.kind === 'prep', plate = !!step.plate, flat = board || plate;
    $('board').hidden = !board; $('plateView').hidden = !plate; $('tPlate').hidden = true;
    $('c').hidden = flat; $('tags').hidden = flat; $('tools3d').hidden = flat;
    if (board) renderBoard();
    if (flat) { $('doneTag').hidden = true; $('player').hidden = true; }
    $('prevBtn').disabled = i === 0; $('nextBtn').disabled = i === steps.length - 1;
    if (!flat && (!PHONE.matches || app.classList.contains('with3d') || SELFTEST) && init3d()) apply3d(step, o.instant);
    else if (G) G.step = flat ? null : step;
    if (plate) showPlateStage(step);
    afterLayout();
  }
  function setHash(h) { try { history.replaceState(null, '', '#' + h); } catch (e) { /* 주소는 못 바꿔도 화면은 바뀐다 */ } }
  async function go(gid, index) {
    closeMenu();
    if (opening) await opening.catch(() => null);
    if (gid !== S.gid || !S.guide) {
      opening = openGuide(gid);
      try { await opening; } catch (e) { toast(String(e.message || e)); showHome(); return; } finally { opening = null; }
    }
    showStep(index == null ? resumeIndex(gid) : index);
    setHash(gid + '--' + S.guide.steps[S.cur].id);
  }
  function showHome() {
    closeMenu();
    S.view = 'home'; app.dataset.view = 'home';
    $('home').hidden = false; $('screen').hidden = true; $('nav').hidden = true;
    renderBar(); renderHome();
    setHash('home');
  }
  function route() {
    const h = decodeURIComponent((location.hash || '').slice(1));
    if (!h || h === 'home' || h === 'selftest') { showHome(); return; }
    const k = h.indexOf('--');
    const gid = k < 0 ? h : h.slice(0, k), sid = k < 0 ? '' : h.slice(k + 2);
    if (!S.catalog || !S.catalog.guides.some((r) => r.id === gid)) { showHome(); return; }
    go(gid).then(() => {
      if (!sid || !S.guide) return;
      const i = S.guide.steps.findIndex((s) => s.id === sid);
      if (i >= 0 && i !== S.cur) { showStep(i); setHash(gid + '--' + sid); }
    });
  }

  $('prevBtn').addEventListener('click', () => go(S.gid, S.cur - 1));
  $('nextBtn').addEventListener('click', () => go(S.gid, S.cur + 1));
  $('homeBtn').addEventListener('click', () => { if (S.view === 'home' && S.guide) go(S.gid, S.cur); else showHome(); });
  $('resumeBtn').addEventListener('click', () => { if (S.resumeGid) go(S.resumeGid); });
  $('gBtn').addEventListener('click', () => { if ($('gMenu').hidden) openMenu(); else closeMenu(); });
  document.addEventListener('pointerdown', (e) => { if (!$('gMenu').hidden && !e.target.closest('#gsel')) closeMenu(); });
  $('show3d').addEventListener('click', () => {
    const on = !app.classList.contains('with3d');
    app.classList.toggle('with3d', on);
    $('show3d').textContent = on ? '3D 닫기' : '3D로 보기';
    if (on && init3d() && S.guide) { if (!G.items.length) buildItems(); G.resize(); apply3d(stepRef(), true); }
  });
  $('resetDocks').addEventListener('click', () => {
    applyLayout(DEFAULT_LAYOUT); store('docks', DEFAULT_LAYOUT);
    folds = {}; store('folds', folds); applyFolds();
    if (app.classList.contains('bare')) toggleBare(); else afterLayout();
    toast('칸을 처음 자리로 돌렸습니다.');
  });
  // 키: 스페이스, → 다음 단계 / ← 이전 단계 / 쉼표(,) 마침표(.) 움직임 한 칸씩 / H 칸 숨기기 / L 이름표 끄기 / Esc 닫기
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { if (drag) dragCancel(); closeMenu(); return; }
    if (S.view !== 'step' || e.altKey || e.ctrlKey || e.metaKey || e.isComposing || !$('gMenu').hidden) return;
    const t = e.target, tag = t && t.tagName;
    if (tag === 'TEXTAREA' || (tag === 'INPUT' && t.type !== 'checkbox' && t.type !== 'range') || (t && t.isContentEditable)) return;
    if (e.key === ',' || e.key === '<') { e.preventDefault(); stepFrame(-1); return; }
    if (e.key === '.' || e.key === '>') { e.preventDefault(); stepFrame(1); return; }
    if (e.code === 'KeyH') { e.preventDefault(); toggleBare(); return; }
    if (e.code === 'KeyL') { e.preventDefault(); toggleLabels(); return; }
    const onRange = tag === 'INPUT' && t.type === 'range';
    if (e.key === ' ' || (!onRange && (e.key === 'ArrowRight' || e.key === 'PageDown'))) { e.preventDefault(); if (S.cur < S.guide.steps.length - 1) go(S.gid, S.cur + 1); }
    else if (!onRange && (e.key === 'ArrowLeft' || e.key === 'PageUp')) { e.preventDefault(); if (S.cur > 0) go(S.gid, S.cur - 1); }
  });
  if (PHONE.addEventListener) PHONE.addEventListener('change', () => { if (PHONE.matches && app.classList.contains('bare')) app.classList.remove('bare'); afterLayout(); if (G && G.step) requestAnimationFrame(() => { G.resize(); reframe(true); }); });
  window.addEventListener('hashchange', () => { if (!SELFTEST) route(); });

  // ---------- 자체 점검 (#selftest) ----------
  async function selftest() {
    const rep = { three: !!(window.THREE && window.THREE.OrbitControls), guides: [], errors: [] };
    window.addEventListener('error', (e) => rep.errors.push('error: ' + e.message));
    window.addEventListener('unhandledrejection', (e) => rep.errors.push('reject: ' + ((e.reason && e.reason.message) || e.reason)));
    try {
      for (const row of S.catalog.guides) {
        const g = { id: row.id, steps: 0, problems: [] };
        await openGuide(row.id);
        S.guide.steps.forEach((st, i) => {
          try {
            showStep(i, { instant: true });
            if (!P.pHead.body.textContent.trim()) g.problems.push(st.id + ': 단계 칸이 비었다');
            if (!P.pDo.body.querySelectorAll('li').length) g.problems.push(st.id + ': 할 일이 비었다');
            if (st.kind === 'work' && !st.kit.length) g.problems.push(st.id + ': 이번에 쓰는 것이 비었다');
            st.kit.forEach((r) => { if (!S.guide.items[r[0]]) g.problems.push(st.id + ': 부품 목록에 없는 키 ' + r[0]); });
            if (st.kind === 'prep') {
              const n = $('boardIn').querySelectorAll('.bi').length;
              if (!n) g.problems.push(st.id + ': 준비물 판이 비었다');
              const printed = (S.guide.board.find((b) => b.kind === 'printed') || { items: [] }).items.filter((row) => itemOf(row[0]).plates).length;
              if ($('boardIn').querySelectorAll('.pst').length !== printed) g.problems.push(st.id + ': 출력 부품의 판 상태가 ' + printed + '개가 아니다');
            } else if (st.plate) {
              if (st.plate.image && !$('plateImg').getAttribute('src')) g.problems.push(st.id + ': 판 그림이 없다');
              if (!st.plate.parts.length || P.pKit.el.hidden) g.problems.push(st.id + ': 이 판의 부품이 비었다');
              if (P.pCheck.el.hidden) g.problems.push(st.id + ': 판 상태 칸이 없다');
              // 부품을 누르면 그 부품 하나만 3D로 보이고, 다시 누르면 판 그림으로 돌아간다
              const pick = st.plate.parts.filter((p) => p.p != null);
              if (!pick.length) g.problems.push(st.id + ': 3D로 볼 수 있는 부품이 없다');
              if (pick.length && rep.three) {
                pick.forEach((p) => {
                  selectPlatePart(st, p.p); renderOnce();
                  const shown = G.items.filter((it) => it.mesh.visible).length;
                  if ($('c').hidden || shown !== 1) g.problems.push(st.id + ': ' + p.file + '를 눌렀는데 3D에 ' + shown + '개가 보인다');
                  if (!P.pKit.body.querySelector('.pick.on')) g.problems.push(st.id + ': 고른 부품이 칸에 표시되지 않는다');
                  selectPlatePart(st, p.p);
                  if (!$('c').hidden || $('plateView').hidden) g.problems.push(st.id + ': 다시 눌러도 판 그림으로 돌아가지 않는다');
                });
              }
            } else if (rep.three) {
              if (G.tl) { G.clock = G.tl.end; animate(); }
              renderOnce();
              const shown = G.items.filter((it) => it.mesh.visible).length;
              if (shown !== st.show.length) g.problems.push(st.id + ': 보이는 부품 ' + shown + '개, 기대 ' + st.show.length + '개');
              if (!shown) g.problems.push(st.id + ': 3D에 보이는 부품이 없다');
              const bad = G.tags.filter((t) => { const v = t.at(); return !isFinite(v.x) || !isFinite(v.y) || !isFinite(v.z); });
              if (bad.length) g.problems.push(st.id + ': 자리를 셀 수 없는 이름표 ' + bad.length + '개');
            }
          } catch (e) { g.problems.push(st.id + ': ' + ((e && e.stack) || e)); }
          g.steps += 1;
        });
        const thumbs = Object.keys(S.guide.items).filter((k) => S.guide.items[k].thumb);
        if (rep.three) thumbs.forEach((k) => { if (!THUMBS[k]) g.problems.push('작은 그림이 없다: ' + k); });
        // 진행: 모든 단계를 넘겨 봤어도 끝낸 단계가 없고, 확인 칸을 다 누른 단계만 끝난다
        const tr = trackOf(row.id);
        if (JSON.stringify(row.track || []) !== JSON.stringify(tr)) g.problems.push('안내 목록의 track이 단계 데이터와 다르다');
        const keep = S.checks[row.id], keepRec = S.records;
        delete S.checks[row.id]; S.records = [];
        renderStrip();
        if (guideProgress(row.id).started || $('strip').querySelector('.chip.done')) g.problems.push('확인 칸이나 판 기록이 없는데 진행이 있다');
        if (tr.length >= 2) {
          // 앞의 두 단계를 끝낸 것으로 꾸민다: 확인 칸은 다 누르고, 판은 Claude가 뽑았다고 적은 것처럼
          const marks = {}, recs = [];
          tr.slice(0, 2).forEach((t) => { if (t[3] > 0) marks[t[0]] = new Array(t[3]).fill(true); else recs.push({ guide: row.id, step: t[0], kind: 'printed', at: new Date().toISOString() }); });
          S.checks[row.id] = { marks: marks, at: new Date().toISOString() }; S.records = recs;
          const want = tr[2] ? S.guide.steps.findIndex((s) => s.id === tr[2][0]) : S.guide.steps.length - 1;
          if (resumeIndex(row.id) !== want) g.problems.push('이어서 할 단계가 확인 칸이나 판 기록과 맞지 않는다');
          renderStrip();
          if ($('strip').querySelectorAll('.chip.done').length !== 2) g.problems.push('끝낸 단계 표시(✓)가 확인 칸이나 판 기록과 맞지 않는다');
        }
        if (keep) S.checks[row.id] = keep; else delete S.checks[row.id];
        S.records = keepRec;
        renderStrip();
        rep.guides.push(g);
      }
    } catch (e) { rep.errors.push('selftest: ' + ((e && e.stack) || e)); }
    const pre = document.createElement('pre'); pre.id = 'selftest-report'; pre.hidden = true; pre.textContent = JSON.stringify(rep);
    document.body.appendChild(pre);
    document.title = 'selftest-done';
  }

  // ---------- 시작 ----------
  buildPanels();
  { const saved = recall('docks'); applyLayout(validLayout(saved) ? saved : DEFAULT_LAYOUT); }
  renderSaved();
  initDb();
  getJSON('data/catalog.json').then((cat) => {
    S.catalog = cat;
    if (SELFTEST) { selftest(); return; }
    renderHome();
    route();
  }).catch((e) => {
    const p = el('p', 'empty', '안내 목록을 읽지 못했습니다: ' + (e.message || e));
    $('kinds').textContent = ''; $('kinds').appendChild(p);
    if (SELFTEST) {
      const pre = document.createElement('pre'); pre.id = 'selftest-report'; pre.hidden = true;
      pre.textContent = JSON.stringify({ three: !!window.THREE, guides: [], errors: ['catalog: ' + (e.message || e)] });
      document.body.appendChild(pre); document.title = 'selftest-done';
    }
  });
})();
