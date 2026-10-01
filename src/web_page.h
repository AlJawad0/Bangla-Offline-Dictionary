// integrated_v3 -- one page, one column, both workflows.
//
// This is v1's OCR page and v2's Bangla-lookup page joined. They were on two
// tabs in the first v3; they are now one scroll, because they are one job:
// capture, pick a word, read it, look it up.
//
// Order down the page, and why:
//
//   1  capture          the file picker and the one Capture button
//   2  preview canvas   every detected word boxed; tap one to read it
//   3  recognized words each word with the Bangla the device found for it
//   4  detail & timing  v1's dump, per box and per stage
//   5  DICTIONARY       v2's search -- driven automatically by the word you
//                       just tapped, and still typeable by hand for testing
//   6  LCD preview      v2's live framebuffer and its buttons
//   7  device           v2's stats line
//   8  camera & photos  v1's gallery
//
// Nothing from either page was dropped in the merge. The two scripts still
// share no identifier and no element id, so they sit side by side below rather
// than being rewritten into each other.
#pragma once

#include <Arduino.h>

const char kWebPage[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-S3 OCR + Bangla</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif;
      --bg:#10151d; --card:#192230; --line:#304055; --fg:#eef3fb; --mut:#b9c7d8;
      --acc:#5aa9ff; --ok:#3ddc97; --amber:#f0a535; --dim:#5a6270;
      --sel:#0f1d30; --seledge:#2f7fd0; }
    body { margin: 0; background: var(--bg); color: var(--fg); }
    main { max-width: 760px; margin: auto; padding: 18px; }
    .card { background: var(--card); border: 1px solid var(--line); border-radius: 14px;
            padding: 16px; margin-bottom: 14px; }
    h1 { margin: 0 0 8px; font-size: 1.45rem; }
    h2 { font-size: 14px; color: var(--mut); margin: 0 0 8px; text-transform: uppercase;
         letter-spacing: .05em; }
    p { color: var(--mut); line-height: 1.4; margin: 8px 0 0; }
    input, button { font: inherit; }
    button.mini { margin-top: 10px; width: auto; padding: 7px 14px; border: 0; border-radius: 8px;
                  background: #2b3b50; color: #dcecff; font-size: .9rem; font-weight: 600; }
    button.mini:disabled { display: none; }
    canvas { display: block; max-width: 100%; height: auto; border-radius: 8px; background: #fff;
             touch-action: manipulation; cursor: pointer; }
    pre { white-space: pre-wrap; word-break: break-word; color: #dcecff; }
    .small { font-size: .88rem; color: #9fb0c5; }
    .live { font-size: 1.05rem; line-height: 1.7; min-height: 1.7em; }
    .live span { background: #22303f; border-radius: 6px; padding: 1px 8px; margin: 0 5px 5px 0;
                 display: inline-block; }
    .live span.pending { opacity: .4; }
    .live span b { color: var(--amber); font-weight: 600; }
    .live span i.ocr { color: var(--dim); font-style: normal; font-size: .8em; }
    /* --- camera controls and the saved-photo list --- */
    .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .row input[type=file] { flex: 1 1 120px; min-width: 0; }
    button.act { padding: 8px 14px; border: 0; border-radius: 8px; background: #2b3b50;
                 color: #dcecff; font-size: .9rem; font-weight: 600; cursor: pointer; }
    button.act:disabled { opacity: .4; cursor: default; }
    button.act.go { background: var(--acc); color: #04121f; }
    button.act.danger { background: #63262f; color: #ffdfe4; }
    .badges { margin: 6px 0 12px; font-size: .88rem; color: #9fb0c5; }
    .badges b { color: var(--fg); font-weight: 600; }
    .ok { color: #00e58b; font-weight: 700; }
    .bad { color: #ff6b6b; font-weight: 700; }
    .shots { display: flex; flex-wrap: wrap; gap: 14px; margin-top: 12px; }
    .shot { background: #101822; border: 1px solid #263448; border-radius: 10px; padding: 8px; }
    .shot img { display: block; width: 320px; max-width: 100%; height: auto; border-radius: 6px;
                background: #fff; }
    .shot .meta { display: flex; justify-content: space-between; gap: 12px; align-items: center;
                  margin-top: 6px; font-size: .82rem; color: #9fb0c5; }
    .shot a { color: #7ee8ff; }
    /* --- the dictionary half --- */
    .qrow { display: flex; gap: 8px; }
    .qrow input { flex: 1; padding: 11px 12px; border-radius: 8px; border: 1px solid var(--line);
                  background: #0c0e13; color: var(--fg); font-size: 16px; }
    button.p { padding: 11px 14px; border-radius: 8px; border: 1px solid var(--acc);
               background: var(--acc); color: #04121f; font-size: 15px; font-weight: 600;
               cursor: pointer; }
    .item { display: flex; align-items: center; gap: 10px; padding: 10px; border-radius: 8px;
            border: 1px solid var(--line); margin-top: 8px; background: #12151c; cursor: pointer }
    .item:hover { border-color: var(--acc) }
    .en { font-weight: 600; min-width: 90px }
    .bn { font-size: 20px; flex: 1 }
    .tbl { border: 1px solid var(--line); border-radius: 8px; overflow: hidden; margin-top: 10px }
    .thead, .lrow { display: grid; grid-template-columns: 26px 1fr 1fr; gap: 10px;
                    padding: 8px 10px; align-items: baseline }
    .thead { font-size: 11px; letter-spacing: .06em; border-bottom: 1px solid var(--line);
             background: #12151c }
    .thead .e { color: var(--acc) }
    .thead .b { color: var(--mut) }
    .thead .i { color: var(--dim) }
    .band { background: #16191f; color: var(--mut); font-size: 11px; letter-spacing: .08em;
            padding: 4px 10px; border-bottom: 1px solid var(--line) }
    .lrow { border-bottom: 1px solid var(--line); cursor: pointer }
    .lrow:last-child { border-bottom: 0 }
    .lrow:hover { background: #12151c }
    .lrow .i { color: var(--dim); font-size: 11px }
    .lrow .e { color: #fff; font-weight: 600 }
    .lrow .b { color: var(--amber); font-size: 19px; line-height: 1.35 }
    .lrow .m { grid-column: 2/4; color: var(--mut); font-size: 11px }
    .lrow.sel { background: var(--sel); box-shadow: inset 0 0 0 1px var(--seledge) }
    .nav { display: flex; gap: 6px; margin-top: 10px; flex-wrap: wrap }
    .nav button { flex: 1; min-width: 56px; padding: 9px 6px; font-size: 13px;
                  border-radius: 8px; border: 1px solid var(--line); background: #232833;
                  color: var(--fg); cursor: pointer }
    .tag { font-size: 11px; color: var(--mut); border: 1px solid var(--line);
           border-radius: 8px; padding: 3px 8px; flex: none; max-width: 132px;
           text-align: right; line-height: 1.35 }
    .tag.l { color: var(--ok); border-color: var(--ok) }
    .tag.f { color: var(--acc); border-color: var(--acc) }
    #lcd { width: 100%; max-width: 300px; image-rendering: pixelated;
           border: 1px solid var(--line); border-radius: 6px; display: block; background: #000;
           margin: 0 auto }
    .note { color: var(--mut); font-size: 12px; margin-top: 8px }
    .stats { color: var(--mut); font-size: 12px; font-family: ui-monospace, monospace }
    .err { color: #ff6b6b; font-size: 13px }
    /* The word the dictionary is currently answering for, so it is obvious the
       list below came from a tap on the image and not from the text box. */
    .forword { display: none; margin-top: 10px; padding: 8px 10px; border-radius: 8px;
               background: #12151c; border: 1px solid var(--seledge); font-size: .9rem }
    .forword b { color: var(--amber); font-size: 1.15rem }
  </style>
</head>
<body><main>

<!-- ============================ 1. capture ============================ -->
  <section class="card">
    <h1>ESP32-S3 detector + INT8 OCR</h1>
    <div class="row">
      <input id="file" type="file" accept="image/*">
      <button id="snap" class="act go">Capture</button>
    </div>
    <p class="small"><b>Capture</b> takes a photo on the ESP32-S3 and detects the
    words on the ESP32-S3 &mdash; the frame never leaves the board, and the same
    frame and boxes appear here and on the 3.5&Prime; panel. Or pick a photo from
    the gallery, which uploads and detects the same way. Either way,
    <b>tap any green box</b> &mdash; here or on the panel &mdash; and that word is
    recognized in about 1.7 s and looked up in the device's Bangla dictionary,
    with the matches listed below.</p>
    <div id="status" class="small">Waiting for an image.</div>
  </section>

<!-- ======================== 2. preview canvas ========================= -->
  <section class="card">
    <canvas id="preview"></canvas>
    <button id="rest" class="mini" disabled>Recognize the rest</button>
  </section>

<!-- ======================= 3. recognized words ======================== -->
  <section class="card">
    <strong>Recognized words</strong>
    <div id="live" class="live"></div>
  </section>

<!-- ======================== 4. detail & timing ======================== -->
  <section class="card"><strong>Detail and timing</strong><pre id="result">No result yet.</pre></section>

<!-- ===================== 5. dictionary (was tab 2) ==================== -->
  <section class="card" id="dict">
    <h1>Bangla lookup</h1>
    <div class="note">Matching runs in SQLite on the device's flash. Rendering happens on the
    device too. Nothing is downloaded to look up or display a word.</div>
    <div id="forword" class="forword"></div>
    <div class="qrow" style="margin-top:10px">
      <input id="q" placeholder="English word (e.g. science)" autocomplete="off">
      <button class="p" onclick="search()">Search</button>
    </div>
    <div class="note">The box above fills itself with whatever word you last tapped on
    the image, and the list appears by itself. Type in it to look a word up by hand.</div>
    <div class="note" id="qmeta"></div>
    <div id="res"></div>
    <div id="online"></div>
    <hr style="border:0;border-top:1px solid #223;margin:14px 0">
    <h2>Learned words <span id="lcount" class="note"></span></h2>
    <div class="note">Pick a CSV or TSV and send it to the device. Accepted layouts are
    <code>word,meaning</code> or <code>id,word,meaning</code>, tab or comma separated; a
    quoted meaning may contain commas. Every row is checked against the 12k dictionary
    <em>and</em> the words already learned, so only genuinely new pairs are stored.</div>
    <div class="qrow" style="margin-top:10px">
      <input id="impf" type="file" accept=".csv,.tsv,.txt,text/csv,text/plain">
      <button class="p" onclick="importWords()">Import</button>
      <button onclick="location.href='/api/learned.tsv'" title="download learned.tsv">Export</button>
    </div>
    <pre id="impout" class="note" style="white-space:pre-wrap"></pre>
  </section>

<!-- ======================== 6. LCD preview ============================ -->
  <section class="card">
    <h2>LCD preview &mdash; rendered by the ESP32</h2>
    <img id="lcd" src="/api/lcd" alt="device framebuffer">
    <div class="stats" id="lmeta"></div>
    <div class="nav">
      <button onclick="nav('up')" title="older words">&#9650;</button>
      <button onclick="nav('down')" title="newer words">&#9660;</button>
      <button onclick="nav('new')" title="jump to the newest">&#8659; NEW</button>
      <button onclick="nav('capture')" title="append the current pair to the history">&#9673; CAPTURE</button>
      <button onclick="nav('clear')" title="clear the preview, keep the words">&#10226; CLEAR</button>
      <button onclick="nav('hist')" title="history page">&#9776; HIST</button>
      <button onclick="nav('back')" title="back to the capture page">&#8592; BACK</button>
      <button onclick="nav('clearhist')" title="delete every word">&#10005; CLEAR HISTORY</button>
    </div>
    <div class="nav">
      <button onclick="nav('img')" title="the captured frame with its word boxes">&#9635; IMAGE</button>
      <button onclick="nav('readall')" title="recognize every green box, on the device">&#9654; READ ALL</button>
    </div>
    <div class="note">This is the actual 320&times;480 portrait framebuffer the panel receives,
    drawn by the device's own shaper and glyph atlas, in palette B ("nocturne") from
    ui_mockup_3p5.html. The buttons above post the same actions the panel's own touch
    buttons do, so the two cannot disagree. Oldest word at the top, newest appended at
    the bottom. CLEAR wipes the preview only; CLEAR HISTORY deletes the words.
    IMAGE puts the captured frame full screen with a box around every word, and tapping
    a box on the panel recognizes it and looks the word up &mdash; the same session this
    page is selecting from.</div>
  </section>

<!-- =========================== 7. device ============================== -->
  <section class="card">
    <h2>Device</h2>
    <div class="stats" id="stats">loading...</div>
  </section>

<!-- ====================== 8. camera & photos ========================== -->
  <section class="card">
    <strong>Camera and saved photos</strong>
    <div id="caminfo" class="badges">Checking the camera and flash storage&hellip;</div>
    <button id="clear" class="act danger" disabled>Clear</button>
    <div id="shots" class="shots"></div>
  </section>

</main>
<script>
/* =====================================================================
   THE OCR HALF -- v1's script, unchanged except where a line is marked v3.
   ===================================================================== */
const file = document.getElementById('file');
const restBtn = document.getElementById('rest');
const status = document.getElementById('status');
const canvas = document.getElementById('preview');
const live = document.getElementById('live');
const result = document.getElementById('result');
const ctx = canvas.getContext('2d', {willReadFrequently: true});

let bitmap = null;      // decoded photo, redrawn under the boxes every repaint
let gray = null;        // the exact bytes POSTed to the device
let detection = null;   // last /detect response
let boxes = [];         // detection.words
let texts = new Map();  // index -> the raw string the recognizer returned
let matched = new Map();// v3r2: index -> the dictionary word it MATCHED
let bangla = new Map(); // v3: index -> that matched word's Bangla
let queue = [];         // taps waiting for a request
let inflight = new Set();   // indices in the request currently streaming
let draining = false;
let detecting = false;
let autoQuery = '';     // v3: the word the dictionary should answer for next

const isPending = i => inflight.has(i) || queue.includes(i);
const greenBoxes = () => boxes.filter(w => w.green);
const remaining = () => greenBoxes().filter(w => !texts.has(w.index) && !isPending(w.index));

async function prepare(chosen) {
  if (bitmap && bitmap.close) bitmap.close();
  bitmap = await createImageBitmap(chosen, {imageOrientation: 'from-image'});
  const scale = Math.min(1, 480 / bitmap.width, 640 / bitmap.height);
  canvas.width = Math.max(1, Math.round(bitmap.width * scale));
  canvas.height = Math.max(1, Math.round(bitmap.height * scale));
  ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
  const rgba = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
  gray = new Uint8Array(canvas.width * canvas.height);
  for (let src = 0, dst = 0; dst < gray.length; src += 4, ++dst) {
    gray[dst] = (77 * rgba[src] + 150 * rgba[src + 1] + 29 * rgba[src + 2]) >> 8;
  }
  detection = null; boxes = []; texts.clear(); matched.clear(); bangla.clear();
  queue = []; inflight.clear();
  live.textContent = ''; result.textContent = 'No result yet.';
  updateRest();
}

// v3: a guard for the device-capture path. The device's boxes are in ITS frame's
// coordinates, so the canvas has to be exactly that size or every box is drawn
// and hit-tested in the wrong place. prepare()'s own rule already produces
// 480x360 from an SVGA frame, so this is normally a no-op -- but it stops a
// camera resolution change from silently misaligning everything.
function fitCanvasTo(w, h) {
  if (!bitmap || (canvas.width === w && canvas.height === h)) return;
  canvas.width = w;
  canvas.height = h;
  ctx.drawImage(bitmap, 0, 0, w, h);
}

// Every box is drawn so the segmentation stays visible. Only green boxes react
// to a tap: a truncated string is not safe for a dictionary lookup and a
// punctuation box decodes to noise, so neither is worth 1.7 s of inference.
function draw() {
  if (!bitmap) return;
  ctx.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
  for (const word of boxes) {
    const [x1, y1, x2, y2] = word.box;
    const w = x2 - x1 + 1, h = y2 - y1 + 1;
    const done = texts.has(word.index);
    const pending = isPending(word.index);
    ctx.save();
    if (!word.green) {
      ctx.strokeStyle = word.edge ? '#ffd43b' : '#7b8a9e';
      ctx.lineWidth = 1;
    } else if (done) {
      ctx.strokeStyle = '#7ee8ff';
      ctx.lineWidth = 2;
    } else if (pending) {
      ctx.strokeStyle = '#ffb454';
      ctx.lineWidth = 3;
      ctx.setLineDash([5, 4]);
      ctx.fillStyle = 'rgba(255,180,84,0.18)';
      if (word.quad) { quadPath(word.quad); ctx.fill(); } else ctx.fillRect(x1, y1, w, h);
    } else {
      ctx.strokeStyle = '#00e58b';
      ctx.lineWidth = 2;
    }
    // A line-detector box follows its text line: draw the tilted quad it really is.
    if (word.quad) { quadPath(word.quad); ctx.stroke(); } else ctx.strokeRect(x1, y1, w, h);
    ctx.restore();
    ctx.fillStyle = !word.green ? (word.edge ? '#ffd43b' : '#7b8a9e')
      : done ? '#7ee8ff' : pending ? '#ffb454' : '#00e58b';
    ctx.font = word.green ? 'bold 12px system-ui' : '11px system-ui';
    // v3r2: the label is the word that was MATCHED, because that is the one
    // whose Bangla gets filed. The raw OCR string is in the chips below.
    const label = done ? (matched.get(word.index) || texts.get(word.index) || '(empty)')
                       : pending ? '...' : String(word.index);
    ctx.fillText(label, x1 + 2, Math.max(12, y1 - 3));
  }
}

function quadPath(q) {
  ctx.beginPath();
  ctx.moveTo(q[0], q[1]); ctx.lineTo(q[2], q[3]); ctx.lineTo(q[4], q[5]); ctx.lineTo(q[6], q[7]);
  ctx.closePath();
}

function canvasPoint(event) {
  const rect = canvas.getBoundingClientRect();
  const src = event.changedTouches ? event.changedTouches[0] : event;
  return [(src.clientX - rect.left) * canvas.width / rect.width,
          (src.clientY - rect.top) * canvas.height / rect.height];
}

// Smallest containing green box wins, so a word inside a larger merged box is
// still reachable.
function boxAt(cx, cy) {
  let best = null, bestArea = Infinity;
  for (const word of boxes) {
    if (!word.green) continue;
    const [x1, y1, x2, y2] = word.box;
    if (cx < x1 || cx > x2 || cy < y1 || cy > y2) continue;
    const area = (x2 - x1 + 1) * (y2 - y1 + 1);
    if (area < bestArea) { best = word; bestArea = area; }
  }
  return best;
}

function onTap(event) {
  if (!boxes.length || detecting) return;
  event.preventDefault();
  const [cx, cy] = canvasPoint(event);
  const word = boxAt(cx, cy);
  if (!word) return;
  // v3: a box that was already read does not need 1.7 s of inference again --
  // re-running is deterministic. Send its word to the dictionary instead, which
  // is what tapping it is now for.
  if (texts.has(word.index)) {
    askDictionary(texts.get(word.index));
    return;
  }
  if (isPending(word.index)) return;
  enqueue([word.index]);
}
canvas.addEventListener('click', onTap);
canvas.addEventListener('touchend', onTap, {passive: false});

restBtn.addEventListener('click', () => enqueue(remaining().map(w => w.index)));

function updateRest() {
  const n = remaining().length;
  restBtn.disabled = n === 0;
  restBtn.textContent = `Recognize the rest (${n}, ~${(n * 1.7).toFixed(0)} s)`;
}

function renderLive() {
  live.innerHTML = '';
  for (const word of greenBoxes()) {
    if (!texts.has(word.index) && !isPending(word.index)) continue;
    const span = document.createElement('span');
    if (texts.has(word.index)) {
      // v3r2: MATCHED word first, then its Bangla -- the two describe each
      // other. The recognizer's own string follows in small type when it
      // differs, which is what makes "Nowhere (Enowiedes)" legible instead of
      // looking like the dictionary is broken.
      const ocr = texts.get(word.index) || '(empty)';
      const en = matched.get(word.index) || ocr;
      span.textContent = en;
      const bn = bangla.get(word.index);
      if (bn) {
        span.appendChild(document.createTextNode(' → '));
        const b = document.createElement('b');
        b.textContent = bn;
        span.appendChild(b);
      }
      if (ocr && ocr.toLowerCase() !== en.toLowerCase()) {
        const o = document.createElement('i');
        o.className = 'ocr';
        o.textContent = ' ocr: ' + ocr;
        span.appendChild(o);
      }
      // v3: and tapping it re-runs the dictionary for that word, no inference.
      span.style.cursor = 'pointer';
      span.onclick = () => askDictionary(texts.get(word.index));
    } else { span.textContent = '…'; span.className = 'pending'; }
    live.appendChild(span);
  }
}

function renderSummary() {
  if (!detection) return;
  const d = detection;
  const lines = [
    `Image: ${d.width} x ${d.height}${d.source === 'device' ? '  (detected on the ESP32)' : ''}`,
    `Boxes: ${d.box_count} -> green ${d.recognizable}, cut off ${d.truncated}, punctuation ${d.punctuation}`,
    `Char height: ${d.char_height}; threshold C: ${d.threshold_c}; contrast: ${d.contrast}`,
    `Background radius: ${d.bg_radius}; skew: ${d.skew_deg} deg; gap-split: ${d.split}; rules removed: ${d.rules_removed}`,
    `Detection: ${d.detection_ms} ms`,
  ];
  for (const word of boxes) {
    if (!texts.has(word.index)) continue;
    const notes = [];
    if (word.flags & 32) notes.push('split');
    if (word.flags & 16) notes.push('pad-clipped');
    if (word.shrunk) notes.push('shrunk to fit');
    const note = notes.length ? ` (${notes.join(',')})` : '';
    // v3: the dictionary hit rides along on the same line.
    const bn = bangla.get(word.index);
    const hit = bn ? `  -> ${word.dict_en || ''} ${bn} [${word.dict_src || ''}]` : '';
    lines.push(`${String(word.index).padStart(2)}: ocr "${texts.get(word.index) || ''}"${note}`
      + `   [${word.fit}, span ${word.crop_span}, ${word.invoke_ms} ms]${hit}`);
  }
  const sentence = boxes.filter(w => texts.has(w.index))
                        .map(w => texts.get(w.index)).filter(Boolean).join(' ');
  // (the transcription is the RAW OCR, not the dictionary's corrections)
  if (sentence) lines.push('', `As text: ${sentence}`);
  result.textContent = lines.join('\n');
}

function repaint() { draw(); renderLive(); renderSummary(); updateRest(); }

async function detect() {
  detecting = true;
  status.textContent = 'Uploading and detecting on the ESP32-S3...';
  try {
    const form = new FormData();
    form.append('image', new Blob([gray], {type: 'application/octet-stream'}), 'page.gray');
    const response = await fetch(`/detect?w=${canvas.width}&h=${canvas.height}`,
                                {method: 'POST', body: form});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    detection = data; boxes = data.words;
    repaint();
    status.textContent = `${data.box_count} boxes in ${data.detection_ms} ms, `
      + `${data.recognizable} complete words. Tap one here or on the panel.`;
    refreshLcdTimed();   // v3: the panel got the same frame and the same boxes
  } catch (error) {
    status.textContent = `Detection failed: ${error.message}`;
  } finally {
    detecting = false;
  }
}

// Taps go into a queue drained by a single in-flight request, so a burst of
// taps never opens two connections to the device - it serves one at a time -
// and no tap is dropped while a word is being recognized.
function enqueue(indices) {
  let added = false;
  for (const index of indices) {
    if (texts.has(index) || isPending(index)) continue;
    queue.push(index);
    added = true;
  }
  if (!added) return;
  repaint();
  drain();
}

async function drain() {
  if (draining || !detection) return;
  draining = true;
  autoQuery = '';
  try {
    while (queue.length) {
      const batch = queue.splice(0, queue.length).sort((a, b) => a - b);
      inflight = new Set(batch);
      repaint();
      await recognize(batch);
      inflight.clear();
      repaint();
    }
  } finally {
    draining = false;
    repaint();
    refreshLcdTimed();   // v3: the panel has the same words now
    stats();
    // v3: the word that was just read goes to the dictionary by itself. Once
    // per drain, not once per word: a SQLite query blocks the device for
    // ~72-188 ms, and firing seventeen of them behind "Recognize the rest"
    // would cost three seconds to show one list.
    if (autoQuery) askDictionary(autoQuery);
  }
}

// Reads the chunked NDJSON stream and paints each word the moment it arrives,
// rather than waiting for the whole batch. One line per finished inference.
async function recognize(batch) {
  const started = performance.now();
  let count = 0;
  try {
    const response = await fetch(`/recognize?s=${detection.session}&b=${batch.join(',')}`);
    if (!response.ok) {
      const data = await response.json().catch(() => ({}));
      throw new Error(data.error || `HTTP ${response.status}`);
    }
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    for (;;) {
      const {done, value} = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, {stream: true});
      let nl;
      while ((nl = buffer.indexOf('\n')) >= 0) {
        const raw = buffer.slice(0, nl).trim();
        buffer = buffer.slice(nl + 1);
        if (!raw) continue;
        const event = JSON.parse(raw);
        if (event.type === 'begin') continue;
        if (event.type === 'done') {
          status.textContent = `${event.recognized} word${event.recognized === 1 ? '' : 's'} `
            + `in ${(event.total_ms / 1000).toFixed(1)} s`
            + (event.aborted ? ' (stopped early)' : '') + '. Tap another box.';
          continue;
        }
        const word = boxes[event.index];
        if (word) Object.assign(word, event);
        texts.set(event.index, event.text);
        if (event.dict_en) matched.set(event.index, event.dict_en);   // v3r2
        if (event.bn) bangla.set(event.index, event.bn);   // v3
        if (event.text) autoQuery = event.text;            // v3
        inflight.delete(event.index);
        ++count;
        repaint();
        status.textContent = `${event.dict_en || event.text || '(empty)'}`
          + (event.bn ? ` → ${event.bn}` : '')
          + (event.dict_en && event.text && event.dict_en.toLowerCase() !== event.text.toLowerCase()
             ? `  (ocr: ${event.text})` : '')
          + `   (${count}/${batch.length}, ${((performance.now() - started) / 1000).toFixed(1)} s)`;
      }
    }
  } catch (error) {
    status.textContent = `Recognition failed: ${error.message}`;
  }
}

file.addEventListener('change', async () => {
  if (!file.files.length) return;
  try {
    await prepare(file.files[0]);
    await detect();          // no button: picking a photo is the whole gesture
  } catch (error) {
    status.textContent = `Image error: ${error.message}`;
  }
});

// --- Camera and flash photo store ------------------------------------------
const snapBtn = document.getElementById('snap');
const clearBtn = document.getElementById('clear');
const caminfo = document.getElementById('caminfo');
const shots = document.getElementById('shots');

let cam = null;       // last /caminfo response
let camBusy = false;  // a capture or a clear is in flight

function renderCam() {
  if (!cam) {
    caminfo.textContent = 'Camera status unavailable.';
    snapBtn.disabled = true;
    clearBtn.disabled = true;
    return;
  }
  const mark = on => on ? '<span class="ok">OK</span>' : '<span class="bad">X</span>';
  caminfo.innerHTML = `<b>Wi-Fi</b> ${mark(cam.wifi)} &nbsp;|&nbsp; <b>Cam</b> ${mark(cam.camera)}`
    + ` &nbsp;|&nbsp; <b>Flash</b> ${mark(cam.fs)} &nbsp;|&nbsp; ${cam.ip}`
    + (cam.fs ? ` &nbsp;|&nbsp; ${cam.fs_used_kb} of ${cam.fs_total_kb} KB used` : '');
  snapBtn.disabled = camBusy || !cam.camera || !cam.fs;
  clearBtn.disabled = camBusy || !cam.fs || !cam.photos.length;

  shots.innerHTML = '';
  if (!cam.photos.length) {
    const empty = document.createElement('p');
    empty.className = 'small';
    empty.textContent = cam.fs ? 'No photos saved yet.'
      : 'Flash storage is unavailable, so photos cannot be stored.';
    shots.appendChild(empty);
    return;
  }
  for (const photo of cam.photos) {
    const card = document.createElement('div');
    card.className = 'shot';
    const img = document.createElement('img');
    // Slots are overwritten in place, so the capture number is what stops the
    // browser showing a cached older photo under the same filename.
    img.src = `${photo.path}?v=${photo.gen}`;
    img.alt = photo.name;
    const meta = document.createElement('div');
    meta.className = 'meta';
    const label = document.createElement('span');
    label.textContent = `${photo.name} - ${(photo.bytes / 1024).toFixed(0)} KB`;
    const link = document.createElement('a');
    link.href = photo.path;
    link.download = photo.name;
    link.textContent = 'Download';
    meta.append(label, link);
    card.append(img, meta);
    shots.appendChild(card);
  }
}

async function refreshCam() {
  try {
    const response = await fetch('/caminfo');
    cam = await response.json();
  } catch (error) {
    cam = null;
  }
  renderCam();
}

// v3: ONE capture button, and the whole pipeline runs on the ESP32.
//
// v1 had two steps here: /snap stored a JPEG, then the browser downloaded it,
// decoded it, downscaled it and POSTed the grayscale back to /detect. The
// device can do all of that itself now, so /snap?detect=1 answers with the
// finished detection in one round trip and the frame never leaves the board.
// The JPEG is still pulled afterwards, but only so there is something to draw
// the boxes on -- it is not what was detected.
snapBtn.addEventListener('click', async () => {
  camBusy = true;
  renderCam();
  detecting = true;
  status.textContent = 'Capturing and detecting on the ESP32-S3...';
  try {
    const response = await fetch('/snap?detect=1');
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    camBusy = false;
    await refreshCam();
    const blob = await (await fetch(`${data.path}?v=${data.gen}`)).blob();
    await prepare(blob);              // clears the old detection and its words
    fitCanvasTo(data.width, data.height);
    detection = data;
    boxes = data.words;
    repaint();
    status.textContent = `${data.box_count} boxes in ${data.detection_ms} ms, `
      + `${data.recognizable} complete words. Tap one here or on the panel.`;
    refreshLcdTimed();
    stats();
  } catch (error) {
    camBusy = false;
    renderCam();
    status.textContent = `Capture failed: ${error.message}`;
  } finally {
    detecting = false;
  }
});

clearBtn.addEventListener('click', async () => {
  if (!confirm('Delete all photos? This cannot be undone.')) return;
  camBusy = true;
  renderCam();
  try {
    const response = await fetch('/clear');
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    status.textContent = `Cleared ${data.deleted} photo${data.deleted === 1 ? '' : 's'}.`;
  } catch (error) {
    status.textContent = `Clear failed: ${error.message}`;
  }
  camBusy = false;
  await refreshCam();
});

refreshCam();

/* =====================================================================
   THE DICTIONARY HALF -- v2's script, unchanged except where marked v3.
   ===================================================================== */
const $ = s => document.querySelector(s);
let lastQuery = '';

function item(en, bn, src, meta) {
  const d = document.createElement('div');
  d.className = 'item';
  const cls = src === 'learned' ? ' l' : (src === 'db/fts5' ? ' f' : '');
  d.innerHTML = '<span class="en"></span><span class="bn"></span>' +
                '<span class="tag' + cls + '"></span>';
  d.children[0].textContent = en;
  d.children[1].textContent = bn;
  d.children[2].textContent = meta || src;
  d.onclick = () => show(en, bn, src);
  return d;
}

// One row of the panel's table: index, ENGLISH, বাংলা, then how it matched.
function lrow(i, en, bn, src, meta, sel) {
  const d = document.createElement('div');
  d.className = 'lrow' + (sel ? ' sel' : '');
  d.innerHTML = '<span class="i"></span><span class="e"></span><span class="b"></span>' +
                '<span class="m"></span>';
  d.children[0].textContent = i;
  d.children[1].textContent = en;
  d.children[2].textContent = bn;
  d.children[3].textContent = meta || src;
  d.onclick = () => show(en, bn, src);
  return d;
}

async function nav(a) {
  await fetch('/api/nav?a=' + encodeURIComponent(a));
  await refreshLcdTimed();
  stats();
}

// Reload the preview and resolve once the image has actually decoded, so the
// number reported is the latency you can see, not just the request.
function refreshLcdTimed() {
  return new Promise(res => {
    const img = $('#lcd'), t0 = performance.now();
    img.onload = img.onerror = () => res(performance.now() - t0);
    img.src = '/api/lcd?t=' + Date.now();
  });
}

async function show(en, bn, src) {
  document.querySelectorAll('.lrow').forEach(r =>
    r.classList.toggle('sel', r.children[1].textContent === en &&
                              r.children[2].textContent === bn));
  const t0 = performance.now();
  $('#lmeta').textContent = 'rendering on device...';
  const j = await (await fetch('/api/show?en=' + encodeURIComponent(en) +
              '&bn=' + encodeURIComponent(bn) + '&src=' + encodeURIComponent(src))).json();
  const tShow = performance.now() - t0;
  const frameMs = await refreshLcdTimed();
  // Device-side numbers for the frame it just sent.
  let s = {};
  try { s = await (await fetch('/api/stats')).json(); } catch (e) {}
  const kb = s.lcd_bytes ? (s.lcd_bytes / 1024).toFixed(0) + 'K' : '?';
  $('#lmeta').textContent =
    'paint ' + ((j.paint_us || 0) / 1000).toFixed(1) + ' ms · frame ' + kb + ' in ' +
    frameMs.toFixed(0) + ' ms (device send ' + ((s.lcd_us || 0) / 1000).toFixed(0) +
    ' ms) · show round-trip ' + tShow.toFixed(0) + ' ms';
}

// v3: `silent` is set only by the automatic lookup. It suppresses the
// render-the-top-hit step, because when the word came from a tap on the image
// the device has ALREADY rendered it -- and /api/show would drag the panel off
// the frame the user is still picking words out of.
async function search(silent) {
  const auto = (silent === true);
  const q = $('#q').value.trim();
  if (!q) return;
  lastQuery = q;
  $('#res').innerHTML = '<div class="note">searching device...</div>';
  $('#qmeta').textContent = '';
  $('#online').innerHTML = '';
  const r = await fetch('/api/search?q=' + encodeURIComponent(q));
  const list = await r.json();
  const us = r.headers.get('X-Query-Us'), nc = r.headers.get('X-Candidates');
  const dbup = r.headers.get('X-Db') === '1';
  $('#qmeta').textContent = dbup
    ? ('SQLite: ' + nc + ' candidates in ' + (us / 1000).toFixed(1) + ' ms (FTS5+BM25 merged with a ' +
       'full-table fuzzy scan)')
    : 'No database in flash — upload data/dictionary.db with tools/upload_db.py';
  $('#res').innerHTML = '';
  if (!list.length) {
    $('#res').innerHTML = '<div class="note">No match in the dictionary.</div>';
  } else {
    const t = document.createElement('div');
    t.className = 'tbl';
    t.innerHTML = '<div class="thead"><span class="i">1-' + list.length + '</span>' +
                  '<span class="e">ENGLISH</span><span class="b">বাংলা</span></div>' +
                  '<div class="band">TODAY</div>';
    list.forEach((e, i) => {
      let m = e.via + ' · ' + e.glyphs + 'g · ' + e.w + 'px';
      if (e.fuzzy !== undefined && e.via !== 'LEARNED') m += ' · fuzzy ' + e.fuzzy.toFixed(3);
      if (e.bm25 !== undefined) m += ' · bm25 ' + e.bm25.toFixed(2);
      t.appendChild(lrow(i + 1, e.en, e.bn, e.src, m, i === 0));
    });
    $('#res').appendChild(t);
    if (!auto) show(list[0].en, list[0].bn, list[0].src);   // top hit -> the LCD
  }
  const btn = document.createElement('button');
  btn.textContent = 'Search online for "' + q + '"';
  btn.className = 'act';
  btn.style.marginTop = '10px';
  btn.onclick = () => online(q);
  $('#online').appendChild(btn);
}

// v3: the join, on the page side. A word read off the image goes into the search
// box and the list appears by itself -- the standalone dictionary's behaviour,
// with the word supplied by the OCR instead of typed.
async function askDictionary(word) {
  const clean = (word || '').replace(/^[^0-9A-Za-z]+/, '').replace(/[^0-9A-Za-z]+$/, '');
  if (!clean) return;
  const fw = document.getElementById('forword');
  fw.style.display = 'block';
  fw.innerHTML = '';
  fw.appendChild(document.createTextNode('read off the image: '));
  const b = document.createElement('b');
  b.textContent = clean;
  fw.appendChild(b);
  $('#q').value = clean;
  await search(true);
  document.getElementById('dict').scrollIntoView({behavior: 'smooth', block: 'start'});
}

async function online(q) {
  $('#online').innerHTML = '<div class="note">asking translation service (via this phone)...</div>';
  try {
    const u = 'https://api.mymemory.translated.net/get?q=' +
              encodeURIComponent(q) + '&langpair=en|bn';
    const r = await fetch(u);
    const j = await r.json();
    const seen = new Set(); const out = [];
    const push = t => { t = (t || '').trim();
      if (t && /[ঀ-৿]/.test(t) && !seen.has(t)) { seen.add(t); out.push(t); } };
    push(j.responseData && j.responseData.translatedText);
    (j.matches || []).forEach(m => push(m.translation));
    if (!out.length) { $('#online').innerHTML = '<div class="err">No Bangla result found.</div>'; return; }
    $('#online').innerHTML = '<h2>Online results &mdash; pick one to store</h2>';
    out.slice(0, 8).forEach(t => {
      const d = item(q, t, 'online', 'tap to save');
      d.onclick = () => save(q, t);
      $('#online').appendChild(d);
    });
  } catch (e) {
    $('#online').innerHTML = '<div class="err">Online lookup failed: ' + e.message +
      '<br>(the phone needs internet; the ESP32 itself does not)</div>';
  }
}

async function save(en, bn) {
  const r = await fetch('/api/add?en=' + encodeURIComponent(en) + '&bn=' + encodeURIComponent(bn));
  const j = await r.json();
  $('#online').innerHTML = '<div class="note">saved &mdash; rendered on device as ' +
    j.glyphs + ' glyphs, ' + j.w + 'px wide, with no extra download.</div>';
  await refreshLcdTimed();
  stats();
  search();
}


// v3r6: send the picked file to the device as a multipart upload. The device
// streams it to a temp file and parses it there, so a big list does not have to
// fit in anyone's RAM on the way.
async function importWords() {
  const f = document.querySelector('#impf').files[0];
  const out = document.querySelector('#impout');
  if (!f) { out.textContent = 'Pick a .csv or .tsv file first.'; return; }
  out.textContent = 'Sending ' + f.name + ' (' + f.size + ' bytes)...';
  const fd = new FormData();
  fd.append('file', f, f.name);
  try {
    const r = await fetch('/api/import', { method: 'POST', body: fd });
    const j = await r.json();
    if (!j.ok) { out.textContent = 'Failed: ' + (j.err || r.status); return; }
    out.textContent =
      'Rows read        ' + j.received + '\n' +
      'Added            ' + j.added + '\n' +
      'Already in dict  ' + j.in_dictionary + '\n' +
      'Already learned  ' + j.already_learned + '\n' +
      'Rejected         ' + j.rejected + '\n' +
      (j.no_room ? 'No room          ' + j.no_room + '\n' : '') +
      '\nLearned now     ' + j.learned + ' / ' + j.capacity +
      '   (' + j.ms + ' ms)';
    stats();
  } catch (e) {
    out.textContent = 'Failed: ' + e;
  }
}

async function stats() {
  const j = await (await fetch('/api/stats')).json();
  const lc = document.querySelector('#lcount');
  if (lc) lc.textContent = j.learned + ' / ' + (j.learned_cap || 2000);
  $('#stats').textContent =
    'panel ' + j.fbw + 'x' + j.fbh + ' · ' + j.hist + ' words, top ' + j.top +
    (j.showhist ? ' (history)' : (j.showimg ? ' (image)' : ' (capture)')) +
    ' · sqlite ' + (j.db ? j.dbrows + ' rows in flash' : 'MISSING') +
    ' · learned ' + j.learned +
    ' · atlas ' + j.glyphs + ' glyphs @' + j.ppem + 'px · ' +
    j.clusters + ' clusters · ' + j.anchors + ' anchors' +
    ' · ocr ' + (j.ocr ? 'ready' : 'FAILED') + ' · cam ' + (j.cam ? 'ok' : 'FAILED') +
    ' · frame ' + j.frame_w + 'x' + j.frame_h + ' ' + j.frame_boxes + ' boxes/' +
    j.frame_green + ' green, ' + j.frame_left + ' unread · heap ' + (j.heap / 1024 | 0) +
    'K · psram ' + (j.psram / 1024 | 0) + 'K';
}

$('#q').addEventListener('keydown', e => { if (e.key === 'Enter') search(); });
stats();
</script></body></html>
)HTML";
