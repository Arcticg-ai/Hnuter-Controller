'use strict';

const palette = {
  red: '#d1495b',
  blue: '#1769aa',
  green: '#238636',
  amber: '#b26a00',
  purple: '#7b4ab5',
  gray1: '#66717b',
  gray2: '#9099a1',
  gray3: '#434b52',
  gray4: '#b3bac0',
};

const query = new URLSearchParams(window.location.search);
const token = query.get('token') || '';
const tokenQuery = token ? `?token=${encodeURIComponent(token)}` : '';
const samples = [];
let config = null;
let eventSource = null;
let paused = false;
let historySeconds = 30;
let latestMode = {armed: false, posctl: false, offboard: false};
let drawPending = false;
let receivedSinceRate = 0;
let rateStarted = performance.now();

const $ = (selector) => document.querySelector(selector);

function finite(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

function format(value, digits = 2) {
  return finite(value) ? `${value >= 0 ? '+' : ''}${value.toFixed(digits)}` : '--';
}

async function api(path, options = {}) {
  const separator = path.includes('?') ? '&' : '?';
  const url = token ? `${path}${separator}token=${encodeURIComponent(token)}` : path;
  const headers = {'Content-Type': 'application/json', ...(options.headers || {})};
  if (token) headers['X-Hnuter-Token'] = token;
  const response = await fetch(url, {...options, headers, cache: 'no-store'});
  let body = {};
  try { body = await response.json(); } catch (_) { body = {error: response.statusText}; }
  if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
  return body;
}

function valueAt(sample, path) {
  let value = sample;
  for (const key of path) {
    if (value == null) return null;
    value = value[key];
  }
  return value;
}

class CanvasChart {
  constructor(canvas, series, options = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.series = series;
    this.fixedRange = options.fixedRange || null;
    this.minimumSpan = options.minimumSpan || 1;
    this.margin = options.margin ?? 0.08;
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(320, Math.floor(rect.width * ratio));
    const height = Math.max(100, Math.floor(rect.height * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    return {width: rect.width, height: rect.height};
  }

  yRange(visible) {
    if (this.fixedRange) return this.fixedRange;
    const values = [];
    for (const sample of visible) {
      for (const line of this.series) {
        const value = valueAt(sample, line.path);
        if (finite(value)) values.push(value);
      }
    }
    if (!values.length) return [-1, 1];
    let low = Math.min(...values);
    let high = Math.max(...values);
    const span = Math.max(high - low, this.minimumSpan);
    const center = (high + low) / 2;
    low = center - span / 2;
    high = center + span / 2;
    const padding = span * this.margin;
    return [low - padding, high + padding];
  }

  draw(allSamples) {
    const {width, height} = this.resize();
    const ctx = this.ctx;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, width, height);

    const left = 55;
    const right = 12;
    const top = 27;
    const bottom = 25;
    const plotWidth = Math.max(width - left - right, 10);
    const plotHeight = Math.max(height - top - bottom, 10);
    const latestT = allSamples.length ? allSamples[allSamples.length - 1].t : 0;
    const startT = latestT - historySeconds;
    const visible = allSamples.filter((sample) => sample.t >= startT);
    const [yLow, yHigh] = this.yRange(visible);

    ctx.lineWidth = 1;
    ctx.font = '11px Segoe UI, Arial, sans-serif';
    ctx.textBaseline = 'middle';
    for (let index = 0; index <= 4; index += 1) {
      const fraction = index / 4;
      const y = top + fraction * plotHeight;
      const value = yHigh - fraction * (yHigh - yLow);
      ctx.strokeStyle = '#e3e7ea';
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + plotWidth, y);
      ctx.stroke();
      ctx.fillStyle = '#66717b';
      ctx.textAlign = 'right';
      ctx.fillText(value.toFixed(Math.abs(value) < 0.1 ? 3 : 1), left - 7, y);
    }
    for (let index = 0; index <= 5; index += 1) {
      const fraction = index / 5;
      const x = left + fraction * plotWidth;
      const secondsAgo = -historySeconds + fraction * historySeconds;
      ctx.strokeStyle = '#eef0f2';
      ctx.beginPath();
      ctx.moveTo(x, top);
      ctx.lineTo(x, top + plotHeight);
      ctx.stroke();
      ctx.fillStyle = '#66717b';
      ctx.textAlign = 'center';
      ctx.fillText(`${secondsAgo.toFixed(0)}s`, x, top + plotHeight + 14);
    }

    const xOf = (t) => left + ((t - startT) / historySeconds) * plotWidth;
    const yOf = (value) => top + ((yHigh - value) / (yHigh - yLow)) * plotHeight;
    for (const line of this.series) {
      ctx.strokeStyle = line.color;
      ctx.lineWidth = line.width || 1.5;
      ctx.setLineDash(line.dash || []);
      ctx.beginPath();
      let active = false;
      for (const sample of visible) {
        const value = valueAt(sample, line.path);
        if (!finite(value)) {
          active = false;
          continue;
        }
        const x = xOf(sample.t);
        const y = yOf(value);
        if (!active) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        active = true;
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);

    let legendX = left;
    ctx.textAlign = 'left';
    for (const line of this.series) {
      ctx.strokeStyle = line.color;
      ctx.lineWidth = line.width || 1.5;
      ctx.setLineDash(line.dash || []);
      ctx.beginPath();
      ctx.moveTo(legendX, 13);
      ctx.lineTo(legendX + 17, 13);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#3b454e';
      ctx.fillText(line.label, legendX + 22, 13);
      legendX += 29 + ctx.measureText(line.label).width;
    }
  }
}

const charts = [
  new CanvasChart($('#attitude-chart'), [
    {label: 'roll', path: ['attitude', 0], color: palette.red},
    {label: 'roll sp', path: ['setpoint', 0], color: palette.red, dash: [5, 4]},
    {label: 'pitch', path: ['attitude', 1], color: palette.blue},
    {label: 'pitch sp', path: ['setpoint', 1], color: palette.blue, dash: [5, 4]},
    {label: 'yaw', path: ['attitude', 2], color: palette.green},
    {label: 'yaw sp', path: ['setpoint', 2], color: palette.green, dash: [5, 4]},
  ], {minimumSpan: 5}),
  new CanvasChart($('#position-chart'), [
    {label: 'N', path: ['position', 0], color: palette.red},
    {label: 'N sp', path: ['position_setpoint', 0], color: palette.red, dash: [5, 4]},
    {label: 'E', path: ['position', 1], color: palette.blue},
    {label: 'E sp', path: ['position_setpoint', 1], color: palette.blue, dash: [5, 4]},
    {label: 'D', path: ['position', 2], color: palette.green},
    {label: 'D sp', path: ['position_setpoint', 2], color: palette.green, dash: [5, 4]},
  ], {minimumSpan: 1}),
  new CanvasChart($('#error-chart'), [
    {label: 'roll', path: ['error', 0], color: palette.red},
    {label: 'pitch', path: ['error', 1], color: palette.blue},
    {label: 'yaw', path: ['error', 2], color: palette.green},
  ], {minimumSpan: 2}),
  new CanvasChart($('#position-error-chart'), [
    {label: 'N', path: ['position_error', 0], color: palette.red},
    {label: 'E', path: ['position_error', 1], color: palette.blue},
    {label: 'D', path: ['position_error', 2], color: palette.green},
  ], {minimumSpan: 0.2}),
  new CanvasChart($('#torque-chart'), [
    {label: 'Tx', path: ['torque', 0], color: palette.red},
    {label: 'Ty', path: ['torque', 1], color: palette.blue},
    {label: 'Tz', path: ['torque', 2], color: palette.green},
  ], {minimumSpan: 0.02}),
  new CanvasChart($('#motor-chart'), [
    {label: 'M1', path: ['motors', 0], color: palette.gray1},
    {label: 'M2', path: ['motors', 1], color: palette.gray2},
    {label: 'M3', path: ['motors', 2], color: palette.gray3},
    {label: 'M4', path: ['motors', 3], color: palette.gray4},
    {label: 'M5', path: ['motors', 4], color: palette.purple, width: 2.2},
  ], {fixedRange: [-1.05, 1.05]}),
];

function scheduleDraw() {
  if (drawPending) return;
  drawPending = true;
  requestAnimationFrame(() => {
    drawPending = false;
    for (const chart of charts) chart.draw(samples);
  });
}

function setStatus(element, text, state) {
  element.textContent = text;
  element.className = `status ${state}`;
}

function updateLiveState(payload) {
  const data = payload.telemetry;
  latestMode = data.mode;
  const allAges = Object.values(data.age);
  const liveTopics = allAges.filter((age) => finite(age) && age < 1.0).length;
  const ddsHealthy = liveTopics === allAges.length;
  const ddsState = ddsHealthy ? 'good' : liveTopics >= 4 ? 'neutral' : 'bad';
  setStatus(
    $('#dds-status'),
    ddsHealthy ? 'DDS live' : `DDS ${liveTopics}/${allAges.length}`,
    ddsState,
  );
  setStatus($('#mav-status'), payload.mavlink ? 'MAVLink connected' : 'MAVLink offline', payload.mavlink ? 'good' : 'bad');
  const modeText = data.mode.armed
    ? `Armed | ${data.mode.offboard ? 'Offboard' : data.mode.posctl ? 'Position' : 'Other'}`
    : 'Disarmed';
  setStatus($('#mode-status'), modeText, data.mode.armed ? 'armed' : 'neutral');
  $('#endpoint').textContent = payload.endpoint || 'MAVLink endpoint not connected';
  $('#roll-value').textContent = `${format(data.attitude[0])} / ${format(data.setpoint[0])} deg`;
  $('#pitch-value').textContent = `${format(data.attitude[1])} / ${format(data.setpoint[1])} deg`;
  $('#yaw-value').textContent = `${format(data.attitude[2])} / ${format(data.setpoint[2])} deg`;
  $('#position-value').textContent = data.position.map((value) => format(value, 2)).join(' ');
  $('#position-error-value').textContent = data.position_error.map((value) => format(value, 2)).join(' ');
  $('#torque-value').textContent = data.torque.map((value) => format(value, 3)).join(' ');
  $('#motor5-value').textContent = format(data.motors[4], 3);
}

function acceptTelemetry(payload) {
  updateLiveState(payload);
  receivedSinceRate += 1;
  const now = performance.now();
  if (now - rateStarted >= 1500) {
    const rate = receivedSinceRate * 1000 / (now - rateStarted);
    $('#sample-rate').textContent = `${rate.toFixed(1)} Hz`;
    rateStarted = now;
    receivedSinceRate = 0;
  }
  if (paused) return;
  samples.push(payload.telemetry);
  const cutoff = payload.telemetry.t - Math.max(historySeconds, 120) - 5;
  while (samples.length && samples[0].t < cutoff) samples.shift();
  scheduleDraw();
}

function connectEvents() {
  if (eventSource) eventSource.close();
  setStatus($('#dds-status'), 'DDS connecting', 'neutral');
  eventSource = new EventSource(`/api/events${tokenQuery}`);
  eventSource.addEventListener('telemetry', (event) => {
    try { acceptTelemetry(JSON.parse(event.data)); } catch (error) { console.error(error); }
  });
  eventSource.onerror = () => setStatus($('#dds-status'), 'DDS reconnecting', 'bad');
}

function showMessage(text, kind = '') {
  const element = $('#parameter-message');
  element.textContent = text;
  element.className = `message ${kind}`;
}

function renderParameterGroup() {
  const groupName = $('#group-select').value;
  const group = config.groups[groupName];
  const list = $('#parameter-list');
  list.replaceChildren();
  const warning = $('#group-warning');
  if (groupName === 'Vehicle model' || groupName === 'Allocator') {
    warning.textContent = 'These values change the vehicle model or allocation. Verify the airframe before applying.';
    warning.classList.remove('hidden');
  } else {
    warning.classList.add('hidden');
  }
  for (const [name, cfg] of Object.entries(group)) {
    const row = document.createElement('div');
    row.className = 'parameter-row';
    row.dataset.name = name;
    row.innerHTML = `
      <div class="parameter-name">
        <span>${name}</span><span class="confirmed">not read</span>
      </div>
      <div class="parameter-controls">
        <input class="range" type="range" min="${cfg.min}" max="${cfg.max}" step="${cfg.step}" value="${cfg.default}" aria-label="${name}">
        <input class="number" type="number" min="${cfg.min}" max="${cfg.max}" step="${cfg.step}" value="${cfg.default}" aria-label="${name} value">
        <button class="apply" type="button">Apply</button>
      </div>`;
    const range = row.querySelector('.range');
    const number = row.querySelector('.number');
    range.addEventListener('input', () => { number.value = range.value; });
    number.addEventListener('input', () => { range.value = number.value; });
    row.querySelector('.apply').addEventListener('click', () => applyParameter(row));
    list.appendChild(row);
  }
  loadParameters();
}

async function loadParameters() {
  const group = $('#group-select').value;
  const button = $('#read-params');
  button.disabled = true;
  showMessage(`Reading ${group} from PX4...`);
  try {
    const response = await api(`/api/params?group=${encodeURIComponent(group)}`);
    for (const [name, value] of Object.entries(response.values)) {
      const row = document.querySelector(`.parameter-row[data-name="${name}"]`);
      if (!row || !finite(value)) continue;
      row.querySelector('.range').value = value;
      row.querySelector('.number').value = value;
      row.querySelector('.confirmed').textContent = `PX4 ${Number(value).toPrecision(6)}`;
    }
    if (response.missing.length) showMessage(`Missing: ${response.missing.join(', ')}`, 'error');
    else showMessage(`${Object.keys(response.values).length} parameters read`, 'success');
  } catch (error) {
    showMessage(error.message, 'error');
  } finally {
    button.disabled = false;
  }
}

async function applyParameter(row) {
  const name = row.dataset.name;
  const value = Number(row.querySelector('.number').value);
  const button = row.querySelector('.apply');
  button.disabled = true;
  showMessage(`Applying ${name}...`);
  try {
    const response = await api('/api/params/set', {
      method: 'POST', body: JSON.stringify({name, value}),
    });
    row.querySelector('.range').value = response.confirmed;
    row.querySelector('.number').value = response.confirmed;
    row.querySelector('.confirmed').textContent = `PX4 ${Number(response.confirmed).toPrecision(6)}`;
    showMessage(`${name} confirmed by PX4`, 'success');
  } catch (error) {
    showMessage(`${name}: ${error.message}`, 'error');
  } finally {
    button.disabled = false;
  }
}

async function initialize() {
  try {
    config = await api('/api/config');
    const select = $('#group-select');
    for (const name of Object.keys(config.groups)) {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      select.appendChild(option);
    }
    select.addEventListener('change', renderParameterGroup);
    renderParameterGroup();
    connectEvents();
  } catch (error) {
    showMessage(error.message, 'error');
    setStatus($('#dds-status'), 'Server unavailable', 'bad');
  }
}

$('#history').addEventListener('change', (event) => {
  historySeconds = Number(event.target.value);
  scheduleDraw();
});
$('#pause').addEventListener('click', (event) => {
  paused = !paused;
  event.target.textContent = paused ? 'Resume' : 'Pause';
});
$('#clear').addEventListener('click', () => { samples.length = 0; scheduleDraw(); });
$('#read-params').addEventListener('click', loadParameters);
$('#save-params').addEventListener('click', async () => {
  const warning = latestMode.armed ? 'The vehicle is ARMED. ' : '';
  if (!window.confirm(`${warning}Permanently save current PX4 parameters?`)) return;
  try {
    await api('/api/params/save', {method: 'POST', body: '{}'});
    showMessage('PX4 parameter save requested', 'success');
  } catch (error) { showMessage(error.message, 'error'); }
});
$('#reconnect').addEventListener('click', async () => {
  showMessage('Reconnecting MAVLink...');
  try {
    const response = await api('/api/mavlink/reconnect', {method: 'POST', body: '{}'});
    showMessage(`Connected via ${response.endpoint}`, 'success');
    loadParameters();
  } catch (error) { showMessage(error.message, 'error'); }
});
window.addEventListener('resize', scheduleDraw);
window.addEventListener('beforeunload', () => { if (eventSource) eventSource.close(); });

initialize();
scheduleDraw();
