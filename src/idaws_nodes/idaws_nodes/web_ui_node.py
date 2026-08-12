"""
web_ui_node
───────────
Flask tabanlı hafif web arayüzü. Telemetri izleme, otonomi kontrolü,
use_yolo parametresi değiştirme, canlı kamera (MJPEG) ve LiDAR görselleştirme.

Kamera kaynağı (video_source parametresi):
  * "ros"       → sensor_msgs/Image topic'inden abone olunur (Gazebo/SITL ve
                  gerçek araçta ROS'a köprülenmiş kamera için önerilen yol).
                  YOLO açıksa yolo_vision_node'un çizim yapılmış (annotated)
                  görüntüsü tercih edilir, yoksa ham kamera görüntüsü gösterilir.
  * "gstreamer" → udpsrc port=gstreamer_port ! RTP/H264 (sahadaki alıcı script'i)
  * "device"    → cv2.VideoCapture(camera_index)

LiDAR: /scan (sensor_msgs/LaserScan) ve lidar_logger_node'un ürettiği kümeler
(idaws_msgs/ClusterArray) ile lokal cost map (nav_msgs/OccupancyGrid) abone
olunur; tarayıcıda canvas üzerinde kuş bakışı çizilir (/api/lidar ve
/api/costmap JSON endpoint'leri üzerinden).
"""

import json
import math
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import NavSatFix, Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from idaws_msgs.msg import ClusterArray

from flask import Flask, render_template_string, jsonify, request, Response


WEB_TEMPLATE = """
<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>IDAWS Kontrol Paneli</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', monospace; background: #0d1117; color: #c9d1d9; padding: 20px; }
  h1 { color: #58a6ff; margin-bottom: 20px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px; }
  .card { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 16px; }
  .card h2 { color: #79c0ff; font-size: 14px; margin-bottom: 10px; text-transform: uppercase; }
  .val { font-size: 22px; font-weight: bold; color: #f0f6fc; }
  .small { font-size: 13px; color: #8b949e; }
  button { padding: 10px 20px; border: none; border-radius: 6px; cursor: pointer;
           font-size: 14px; font-weight: 600; margin: 4px; transition: 0.2s; }
  .btn-start { background: #238636; color: #fff; }
  .btn-stop  { background: #da3633; color: #fff; }
  .btn-toggle { background: #1f6feb; color: #fff; }
  button:hover { opacity: 0.85; }
  #status { margin-top: 10px; padding: 8px; background: #0d1117; border-radius: 4px; }
  .sensors { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px;
             margin-bottom: 16px; }
  .cam-card img, .lidar-card canvas, .map-card canvas {
      width: 100%; display: block; border-radius: 6px;
      background: #000; border: 1px solid #30363d; }
  .badge { float: right; font-size: 11px; padding: 2px 8px; border-radius: 10px;
           background: #21262d; color: #8b949e; text-transform: none; }
  .badge.live { background: #238636; color: #fff; }
  .badge.dead { background: #da3633; color: #fff; }
  .dpad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px;
          max-width: 280px; margin-top: 12px; }
  .btn-dir { background: #21262d; color: #c9d1d9; border: 1px solid #30363d;
             padding: 12px 4px; font-size: 12px; line-height: 1.4; margin: 0;
             user-select: none; touch-action: none; }
  .btn-dir:hover { background: #30363d; }
  .btn-dir.active { background: #1f6feb; color: #fff; }
  .btn-estop { background: #da3633; color: #fff; margin: 0; font-size: 13px; }
</style>
</head>
<body>
<h1>IDAWS Kontrol Paneli</h1>

<div class="sensors">
  <div class="card cam-card">
    <h2>Kamera <span class="badge" id="cam-badge">--</span></h2>
    <img id="cam-img" src="/video_feed" alt="kamera goruntusu">
  </div>
  <div class="card lidar-card">
    <h2>LiDAR + Kumeleme <span class="badge" id="lidar-badge">--</span></h2>
    <canvas id="lidar-canvas" width="480" height="480"></canvas>
    <div class="small" id="lidar-info" style="margin-top:8px;">--</div>
  </div>
  <div class="card map-card">
    <h2>Lokal Cost Map <span class="badge" id="map-badge">--</span></h2>
    <canvas id="map-canvas" width="480" height="480"></canvas>
    <div class="small" id="map-info" style="margin-top:8px;">--</div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>GPS</h2>
    <div class="val" id="gps">--</div>
  </div>
  <div class="card">
    <h2>Heading / Groundspeed</h2>
    <div class="val" id="heading">--</div>
    <div class="small" id="speed">--</div>
  </div>
  <div class="card">
    <h2>Gorev Fazi</h2>
    <div class="val" id="phase">--</div>
  </div>
  <div class="card">
    <h2>Heartbeat</h2>
    <div class="val" id="hb">--</div>
  </div>
  <div class="card">
    <h2>Kontrol</h2>
    <button class="btn-start" onclick="cmd('start')">Baslat</button>
    <button class="btn-stop"  onclick="cmd('stop')">Durdur</button>
    <button class="btn-toggle" onclick="toggleYolo()">YOLO Toggle</button>
    <div id="status"></div>
  </div>

  <div class="card">
    <h2>Otopilot</h2>
    <button class="btn-start" onclick="setArm(true)">ARM</button>
    <button class="btn-stop"  onclick="setArm(false)">DISARM</button>
    <div style="margin-top:8px;">
      <button class="btn-toggle" onclick="setMode('MANUAL')">MANUAL</button>
      <button class="btn-toggle" onclick="setMode('GUIDED')">GUIDED</button>
      <button class="btn-toggle" onclick="setMode('HOLD')">HOLD</button>
    </div>
  </div>
</div>

<div class="grid" style="margin-top:16px;">
  <div class="card manual-card">
    <h2>Manuel Kumanda</h2>
    <div class="small">Butona basili tuttugun sure boyunca komut gider; birakinca durur.
      Klavye: W/A/S/D veya yon tuslari, bosluk = acil dur.</div>
    <div class="dpad">
      <div></div>
      <button class="btn-dir" data-lin="1"  data-ang="0">&#9650;<br>Ileri</button>
      <div></div>
      <!-- Isaret ROS geleneginde: +angular.z = CCW = SOLA. collision_avoidance
           hedef acisini atan2(y, x) ile uretirken ayni gelenegi kullaniyor;
           panel ters isaret kullanirsa manuel kumanda ile otonomi birbirinin
           aynasi olur ve dumen yonu hangi kaynagin surdugune gore degisir. -->
      <button class="btn-dir" data-lin="0"  data-ang="1">&#9664;<br>Sol</button>
      <button class="btn-estop" onclick="stopManual()">DUR</button>
      <button class="btn-dir" data-lin="0"  data-ang="-1">&#9654;<br>Sag</button>
      <div></div>
      <button class="btn-dir" data-lin="-1" data-ang="0">&#9660;<br>Geri</button>
      <div></div>
    </div>
    <div style="margin-top:12px;">
      <label class="small">Gaz: <span id="throttle-val">0.50</span></label>
      <input type="range" id="throttle" min="0.1" max="1.0" step="0.05" value="0.5"
             style="width:100%;" oninput="document.getElementById('throttle-val').textContent=(+this.value).toFixed(2)">
    </div>
    <div class="small" id="manual-status" style="margin-top:8px;">bekleme</div>
  </div>

  <div class="card">
    <h2>Parkur Secimi (SCR_USER1)</h2>
    <div class="small">Otopilottaki SCR_USER1 parametresini yazar — sahadaki gercek
      akisin aynisi. Jetson degeri geri okuyup faza cevirir.</div>
    <div style="margin-top:10px;">
      <button class="btn-stop"   onclick="setParkur(0)">0 &middot; IDLE</button>
      <button class="btn-start"  onclick="setParkur(5)">5 &middot; GERCEK</button>
    </div>
    <div style="margin-top:6px;">
      <button class="btn-toggle" onclick="setParkur(10)">10 &middot; Parkur 1</button>
      <button class="btn-toggle" onclick="setParkur(15)">15 &middot; Parkur 2</button>
      <button class="btn-toggle" onclick="setParkur(20)">20 &middot; Parkur 3</button>
    </div>
    <div style="margin-top:12px;">
      <label class="small">Parkur 3 renk kodu (SCR_USER2):</label>
      <input type="number" id="color-val" value="1" step="1" min="0"
             style="width:70px; padding:6px; background:#0d1117; color:#c9d1d9;
                    border:1px solid #30363d; border-radius:4px;">
      <button class="btn-toggle" onclick="setColor()">Yaz</button>
    </div>
    <div class="small" id="param-status" style="margin-top:10px;">
      SCR_USER1: -- | SCR_USER2: --
    </div>
  </div>
</div>

<script>
// lidar_logger_node'daki CLUSTER_COLORS paletinin RGB karsiligi
const CLUSTER_COLORS = [
  'rgb(80,220,80)', 'rgb(255,160,80)', 'rgb(60,180,255)', 'rgb(255,120,200)',
  'rgb(230,230,60)', 'rgb(160,120,255)', 'rgb(180,255,140)', 'rgb(255,180,180)',
];

function setBadge(id, ok, text) {
  const el = document.getElementById(id);
  el.textContent = text;
  el.className = 'badge ' + (ok ? 'live' : 'dead');
}

async function refresh() {
  try {
    const r = await fetch('/api/telemetry');
    const d = await r.json();
    document.getElementById('gps').textContent =
      d.lat.toFixed(6) + ', ' + d.lon.toFixed(6) + ' | Alt: ' + d.alt.toFixed(1) + 'm';
    document.getElementById('heading').textContent = d.heading.toFixed(0) + ' deg';
    document.getElementById('speed').textContent = d.groundspeed.toFixed(1) + ' m/s';
    document.getElementById('phase').textContent = d.phase;
    document.getElementById('hb').textContent = d.heartbeat;
    setBadge('cam-badge', d.camera_ok, d.camera_source);
  } catch(e) {}
}

// ─────────────── LiDAR kus bakisi cizim ───────────────
function drawLidar(d) {
  const cv = document.getElementById('lidar-canvas');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  const cx = W / 2, cy = H / 2;
  const maxR = d.range_max > 0 ? d.range_max : 12.0;
  const scale = (Math.min(W, H) / 2 - 14) / maxR;

  ctx.fillStyle = '#010409';
  ctx.fillRect(0, 0, W, H);

  // Mesafe halkalari
  ctx.strokeStyle = '#21262d';
  ctx.fillStyle = '#484f58';
  ctx.font = '10px monospace';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i++) {
    const r = maxR * i / 4;
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(r.toFixed(1) + 'm', cx + 3, cy - r * scale + 11);
  }
  // Eksenler
  ctx.beginPath();
  ctx.moveTo(cx, 0); ctx.lineTo(cx, H);
  ctx.moveTo(0, cy); ctx.lineTo(W, cy);
  ctx.stroke();

  // Guvenlik / tehlike cemberleri
  const circles = [[d.safety_radius, '#d29922'], [d.danger_radius, '#da3633']];
  for (const [rad, color] of circles) {
    if (rad > 0 && rad < maxR) {
      ctx.strokeStyle = color;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.arc(cx, cy, rad * scale, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // Isin noktalari — arac burnu yukari (+x ileri)
  for (let i = 0; i < d.ranges.length; i++) {
    const r = d.ranges[i];
    if (r === null || r <= 0 || r > maxR) continue;
    const a = d.angle_min + i * d.angle_increment;
    const px = cx - r * Math.sin(a) * scale;   // sol = +y
    const py = cy - r * Math.cos(a) * scale;   // yukari = +x
    ctx.fillStyle = r < d.danger_radius ? '#f85149'
                  : r < d.safety_radius ? '#d29922' : '#3fb950';
    ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
  }

  // Kumeler — lidar_logger_node ile ayni renk paleti (kaydedilen mp4 ile eslesir)
  ctx.font = '11px monospace';
  for (const c of (d.clusters || [])) {
    const col = CLUSTER_COLORS[c.id % CLUSTER_COLORS.length];
    const px = cx - c.y * scale;
    const py = cy - c.x * scale;
    const rad = Math.max(6, (c.width / 2 + 0.2) * scale);
    ctx.strokeStyle = col;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(px, py, rad, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillStyle = col;
    ctx.fillText('K' + c.id + ' ' + c.range.toFixed(1) + 'm', px + rad + 3, py + 4);
  }
  ctx.lineWidth = 1;

  // Arac ikonu (burun yukari)
  ctx.fillStyle = '#58a6ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 10);
  ctx.lineTo(cx - 6, cy + 7);
  ctx.lineTo(cx + 6, cy + 7);
  ctx.closePath();
  ctx.fill();
}

// ─────────────── Cost map cizim ───────────────
function costColor(c) {
  // 3 duraklı rampa: koyu mor → turuncu → sarı
  const t = c / 100;
  const stops = [[40, 12, 60], [240, 110, 20], [255, 240, 150]];
  const k = t < 0.5 ? 0 : 1;
  const f = t < 0.5 ? t * 2 : (t - 0.5) * 2;
  const a = stops[k], b = stops[k + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}

function drawCostmap(d) {
  const cv = document.getElementById('map-canvas');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;

  ctx.fillStyle = '#010409';
  ctx.fillRect(0, 0, W, H);
  if (!d.ok || d.width === 0) return;

  const off = document.createElement('canvas');
  off.width = d.width; off.height = d.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(d.width, d.height);

  // Grid satiri = +x (ileri), sutunu = +y (sol). Ekranda ileri yukari,
  // sol solda olsun diye her iki eksen de ters cevrilir.
  for (let py = 0; py < d.height; py++) {
    const srcRow = d.height - 1 - py;
    for (let px = 0; px < d.width; px++) {
      const c = d.data[srcRow * d.width + (d.width - 1 - px)];
      const o = (py * d.width + px) * 4;
      if (c <= 0) {
        img.data[o] = 10; img.data[o+1] = 12; img.data[o+2] = 16;
      } else {
        const rgb = costColor(c);
        img.data[o] = rgb[0]; img.data[o+1] = rgb[1]; img.data[o+2] = rgb[2];
      }
      img.data[o+3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, 0, 0, W, H);

  // Arac ikonu
  const cx = W / 2, cy = H / 2;
  ctx.fillStyle = '#58a6ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 10);
  ctx.lineTo(cx - 6, cy + 7);
  ctx.lineTo(cx + 6, cy + 7);
  ctx.closePath();
  ctx.fill();
}

async function refreshCostmap() {
  try {
    const r = await fetch('/api/costmap');
    const d = await r.json();
    setBadge('map-badge', d.ok, d.ok ? d.width + 'x' + d.height : 'veri yok');
    drawCostmap(d);
    document.getElementById('map-info').textContent = d.ok
      ? d.size_m.toFixed(0) + 'x' + d.size_m.toFixed(0) + ' m | '
        + d.resolution.toFixed(2) + ' m/hucre (goruntuleme)'
      : 'perception/costmap yayini yok (lidar_logger_node calisiyor mu?)';
  } catch(e) {}
}

async function refreshLidar() {
  try {
    const r = await fetch('/api/lidar');
    const d = await r.json();
    setBadge('lidar-badge', d.ok, d.ok ? d.point_count + ' isin' : 'veri yok');
    drawLidar(d);
    document.getElementById('lidar-info').textContent = d.ok
      ? 'En yakin engel: ' + d.min_range.toFixed(2) + ' m @ ' + d.min_angle_deg.toFixed(0) + ' deg'
        + ' | Kume: ' + (d.clusters || []).length
        + ' | Menzil: ' + d.range_max.toFixed(1) + ' m'
      : '/scan topic\\'inden veri gelmiyor (gz bridge calisiyor mu?)';
  } catch(e) {}
}

async function cmd(action) {
  const r = await fetch('/api/command', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({action})
  });
  const d = await r.json();
  document.getElementById('status').textContent = d.result;
}
async function toggleYolo() {
  const r = await fetch('/api/toggle_yolo', {method:'POST'});
  const d = await r.json();
  document.getElementById('status').textContent = 'YOLO: ' + d.use_yolo;
}

// ─────────────── Otopilot ───────────────
async function post(url, body) {
  const r = await fetch(url, {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(body || {})
  });
  return r.json();
}
async function setArm(on) {
  const d = await post('/api/arm', {arm: on});
  document.getElementById('status').textContent = d.result;
}
async function setMode(m) {
  const d = await post('/api/mode', {mode: m});
  document.getElementById('status').textContent = d.result;
}
async function setParkur(v) {
  const d = await post('/api/user_param', {name: 'SCR_USER1', value: v});
  document.getElementById('status').textContent = d.result;
}
async function setColor() {
  const v = parseFloat(document.getElementById('color-val').value);
  const d = await post('/api/user_param', {name: 'SCR_USER2', value: v});
  document.getElementById('status').textContent = d.result;
}
async function refreshParams() {
  try {
    const r = await fetch('/api/user_params');
    const d = await r.json();
    const f = (k) => (d[k] === undefined ? '--' : (+d[k]).toFixed(0));
    document.getElementById('param-status').textContent =
      'SCR_USER1: ' + f('SCR_USER1') + ' | SCR_USER2: ' + f('SCR_USER2');
  } catch(e) {}
}

// ─────────────── Manuel kumanda ───────────────
// Basili tutulan yon + gaz degeri 5 Hz ile gonderilir. Sunucu tarafinda
// deadman var: akis kesilirse motorlar otomatik durur.
let manualVec = {lin: 0, ang: 0};
let manualTimer = null;

function throttle() { return parseFloat(document.getElementById('throttle').value); }

function sendManual() {
  const t = throttle();
  post('/api/manual', {linear: manualVec.lin * t, angular: manualVec.ang * t});
}

function startManual(lin, ang, el) {
  manualVec = {lin: lin, ang: ang};
  if (el) el.classList.add('active');
  document.getElementById('manual-status').textContent =
    'komut: ileri=' + (lin * throttle()).toFixed(2) + '  donus=' + (ang * throttle()).toFixed(2);
  sendManual();
  if (manualTimer === null) manualTimer = setInterval(sendManual, 200);
}

function stopManual() {
  manualVec = {lin: 0, ang: 0};
  if (manualTimer !== null) { clearInterval(manualTimer); manualTimer = null; }
  document.querySelectorAll('.btn-dir').forEach(b => b.classList.remove('active'));
  post('/api/manual', {linear: 0, angular: 0});
  document.getElementById('manual-status').textContent = 'durduruldu';
}

document.querySelectorAll('.btn-dir').forEach(btn => {
  const lin = parseFloat(btn.dataset.lin), ang = parseFloat(btn.dataset.ang);
  const down = (e) => { e.preventDefault(); startManual(lin, ang, btn); };
  btn.addEventListener('mousedown', down);
  btn.addEventListener('touchstart', down, {passive: false});
  btn.addEventListener('mouseup', stopManual);
  btn.addEventListener('mouseleave', stopManual);
  btn.addEventListener('touchend', stopManual);
});

const KEYS = {
  'w': [1, 0], 'arrowup': [1, 0], 's': [-1, 0], 'arrowdown': [-1, 0],
  'a': [0, 1], 'arrowleft': [0, 1], 'd': [0, -1], 'arrowright': [0, -1],
};
let heldKey = null;
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); stopManual(); return; }
  if (!KEYS[k] || heldKey === k) return;
  e.preventDefault();
  heldKey = k;
  startManual(KEYS[k][0], KEYS[k][1], null);
});
document.addEventListener('keyup', (e) => {
  if (e.key.toLowerCase() === heldKey) { heldKey = null; stopManual(); }
});
window.addEventListener('blur', stopManual);

setInterval(refresh, 1000);
setInterval(refreshLidar, 200);
setInterval(refreshCostmap, 500);
setInterval(refreshParams, 1000);
refresh();
refreshLidar();
refreshCostmap();
refreshParams();
</script>
</body>
</html>
"""


class WebUiNode(Node):

    # Bir kaynağın "canlı" sayılması için son verinin azami yaşı (sn)
    FRESH_SEC = 2.0

    def __init__(self):
        super().__init__('web_ui_node')

        self.declare_parameter('web_port', 5000)
        self._port = self.get_parameter('web_port').value

        # ---------- Kamera parametreleri ----------
        # video_source: "ros" (ROS Image topic — sim ve köprülenmiş gerçek kamera),
        # "gstreamer" (udpsrc port=gstreamer_port ! RTP/H264) veya
        # "device" (cv2.VideoCapture(camera_index)).
        self.declare_parameter('video_source', 'ros')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', 'vision/image_annotated')
        self.declare_parameter('gstreamer_port', 5400)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('stream_fps', 15.0)
        self._video_source = self.get_parameter('video_source').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._annotated_topic = self.get_parameter('annotated_topic').value
        self._gst_port = self.get_parameter('gstreamer_port').value
        self._camera_idx = self.get_parameter('camera_index').value
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._min_frame_dt = 1.0 / max(1.0, float(self.get_parameter('stream_fps').value))

        # ---------- LiDAR parametreleri ----------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cluster_topic', 'perception/lidar_clusters')
        self.declare_parameter('costmap_topic', 'perception/costmap')
        self.declare_parameter('lidar_max_points', 360)
        self.declare_parameter('costmap_max_cells', 120)
        self.declare_parameter('safety_radius', 2.0)
        self.declare_parameter('danger_radius', 1.0)
        self._scan_topic = self.get_parameter('scan_topic').value
        self._cluster_topic = self.get_parameter('cluster_topic').value
        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._lidar_max_points = int(self.get_parameter('lidar_max_points').value)
        self._costmap_max_cells = int(self.get_parameter('costmap_max_cells').value)
        self._safety_r = float(self.get_parameter('safety_radius').value)
        self._danger_r = float(self.get_parameter('danger_radius').value)

        # ---------- Telemetri cache ----------
        self._telem = {
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'heading': 0.0, 'groundspeed': 0.0,
            'phase': 'IDLE', 'heartbeat': '--',
        }
        self._yolo_enabled = False
        self._user_params = {}

        # ---------- Manuel kumanda durumu ----------
        self._manual_last_cmd = 0.0
        self._manual_active = False

        # ---------- Kamera frame cache ----------
        self._frame_lock = threading.Lock()
        self._latest_jpeg = None
        self._frame_seq = 0            # her yeni karede artar (MJPEG için)
        self._last_frame_time = 0.0    # kaynaktan gelen son karenin zamanı
        self._last_encode_time = 0.0   # hız sınırlama için
        self._last_annotated_time = 0.0
        self._active_source = self._video_source

        # ---------- LiDAR cache ----------
        self._scan_lock = threading.Lock()
        self._scan = None
        self._scan_time = 0.0
        self._clusters = []
        self._clusters_time = 0.0
        self._costmap = None
        self._costmap_time = 0.0

        # ---------- Subscriber'lar ----------
        self.create_subscription(NavSatFix, 'telemetry/gps', self._cb_gps, 10)
        self.create_subscription(Float64, 'telemetry/heading', self._cb_heading, 10)
        self.create_subscription(Float64, 'telemetry/groundspeed', self._cb_gs, 10)
        self.create_subscription(String, 'mission/phase', self._cb_phase, 10)
        self.create_subscription(String, 'telemetry/heartbeat_status', self._cb_hb, 10)

        # Sensör verisi best-effort QoS ile gelir (gz bridge de böyle yayınlar)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, self._scan_topic, self._cb_scan, sensor_qos)
        self.create_subscription(ClusterArray, self._cluster_topic, self._cb_clusters, 10)
        self.create_subscription(OccupancyGrid, self._costmap_topic, self._cb_costmap, 1)

        if self._video_source == 'ros':
            self.create_subscription(Image, self._camera_topic, self._cb_raw_image, sensor_qos)
            self.create_subscription(
                Image, self._annotated_topic, self._cb_annotated_image, sensor_qos
            )
            self.get_logger().info(
                f'Kamera kaynağı: ROS topic — ham: {self._camera_topic}, '
                f'işlenmiş: {self._annotated_topic}'
            )

        self.create_subscription(String, 'telemetry/user_params', self._cb_user_params, 10)
        # Etkin (hıza bağlı) güvenlik yarıçapları — çizimde parametre yerine
        # collision_avoidance_node'un gerçekte kullandığı değerler gösterilir.
        self.create_subscription(
            Float64, 'collision/safety_radius', self._cb_safety_radius, 10)
        self.create_subscription(
            Float64, 'collision/danger_radius', self._cb_danger_radius, 10)

        # ---------- Publisher'lar ----------
        self.pub_start = self.create_publisher(Bool, 'mission/start', 10)
        self.pub_stop = self.create_publisher(Bool, 'mission/stop', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.pub_arm = self.create_publisher(Bool, 'cmd/arm', 10)
        self.pub_mode = self.create_publisher(String, 'cmd/mode', 10)
        self.pub_set_param = self.create_publisher(String, 'cmd/set_user_param', 10)

        # Manuel kumanda ölü adam anahtarı: tarayıcı komut göndermeyi bırakırsa
        # (sekme kapandı, ağ koptu) motorlar kilitli kalmasın diye sıfırlanır.
        self.create_timer(0.2, self._manual_deadman)

        # ---------- SetParameters client ----------
        self._set_param_client = self.create_client(
            SetParameters, '/yolo_vision_node/set_parameters'
        )

        # ---------- Flask ----------
        self._app = Flask(__name__)
        self._setup_routes()

        flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        flask_thread.start()

        if self._video_source in ('gstreamer', 'device'):
            camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
            camera_thread.start()

        self.get_logger().info(f'web_ui_node başlatıldı — http://0.0.0.0:{self._port}')

    # ───────────────────── Telemetri Callback'leri ─────────────────────

    def _cb_gps(self, msg: NavSatFix):
        self._telem['lat'] = msg.latitude
        self._telem['lon'] = msg.longitude
        self._telem['alt'] = msg.altitude

    def _cb_heading(self, msg: Float64):
        self._telem['heading'] = msg.data

    def _cb_gs(self, msg: Float64):
        self._telem['groundspeed'] = msg.data

    def _cb_phase(self, msg: String):
        self._telem['phase'] = msg.data

    def _cb_hb(self, msg: String):
        self._telem['heartbeat'] = msg.data

    def _cb_user_params(self, msg: String):
        try:
            self._user_params = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_safety_radius(self, msg: Float64):
        self._safety_r = float(msg.data)

    def _cb_danger_radius(self, msg: Float64):
        self._danger_r = float(msg.data)

    # ───────────────────── Manuel Kumanda ─────────────────────

    MANUAL_TIMEOUT_SEC = 0.8

    def _publish_manual(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = max(-1.0, min(1.0, float(linear)))
        cmd.angular.z = max(-1.0, min(1.0, float(angular)))
        self.pub_cmd_vel.publish(cmd)
        self._manual_last_cmd = time.time()
        self._manual_active = not (cmd.linear.x == 0.0 and cmd.angular.z == 0.0)

    def _manual_deadman(self):
        """Komut akışı kesilirse bir kez sıfır hız yayınlayıp motorları durdurur."""
        if not self._manual_active:
            return
        if time.time() - self._manual_last_cmd > self.MANUAL_TIMEOUT_SEC:
            self._manual_active = False
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().warn('Manuel komut akışı kesildi — motorlar durduruldu.')

    # ───────────────────── LiDAR ─────────────────────

    def _cb_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        increment = msg.angle_increment

        # Tarayıcıya gönderilecek nokta sayısını sınırla (JSON boyutu / CPU)
        step = max(1, int(math.ceil(len(ranges) / float(self._lidar_max_points))))
        if step > 1:
            ranges = ranges[::step]
            increment *= step

        # Geçersiz (inf/nan/menzil dışı) ölçümleri JSON'da null olarak göster
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        clean = [round(float(r), 3) if v else None for r, v in zip(ranges, valid)]

        if np.any(valid):
            idx = int(np.argmin(np.where(valid, ranges, np.inf)))
            min_range = float(ranges[idx])
            min_angle = math.degrees(msg.angle_min + idx * increment)
        else:
            min_range, min_angle = 0.0, 0.0

        with self._scan_lock:
            self._scan = {
                'ok': True,
                'angle_min': float(msg.angle_min),
                'angle_increment': float(increment),
                'range_max': float(msg.range_max),
                'ranges': clean,
                'point_count': int(np.count_nonzero(valid)),
                'min_range': min_range,
                'min_angle_deg': min_angle,
                'safety_radius': self._safety_r,
                'danger_radius': self._danger_r,
            }
            self._scan_time = time.time()

    def _cb_clusters(self, msg: ClusterArray):
        clusters = [{
            'id': int(c.id),
            'x': round(float(c.center_x), 3),
            'y': round(float(c.center_y), 3),
            'range': round(float(c.range), 2),
            'width': round(float(c.width), 2),
            'points': int(c.point_count),
        } for c in msg.clusters]
        with self._scan_lock:
            self._clusters = clusters
            self._clusters_time = time.time()

    def _cb_costmap(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)

        # Tarayıcıya gönderilecek hücre sayısını sınırla; küçültürken maksimum
        # alınır ki tek hücrelik engeller kaybolmasın.
        step = max(1, int(math.ceil(max(w, h) / float(self._costmap_max_cells))))
        if step > 1:
            trim_h, trim_w = (h // step) * step, (w // step) * step
            grid = grid[:trim_h, :trim_w].reshape(
                trim_h // step, step, trim_w // step, step
            ).max(axis=(1, 3))
            h, w = grid.shape

        with self._scan_lock:
            self._costmap = {
                'ok': True,
                'width': int(w),
                'height': int(h),
                'resolution': round(float(msg.info.resolution) * step, 4),
                'size_m': round(float(msg.info.resolution) * step * w, 2),
                'data': np.clip(grid, 0, 100).astype(np.uint8).flatten().tolist(),
            }
            self._costmap_time = time.time()

    def _empty_costmap(self):
        return {'ok': False, 'width': 0, 'height': 0, 'resolution': 0.0,
                'size_m': 0.0, 'data': []}

    def _empty_scan(self):
        return {
            'ok': False,
            'angle_min': -math.pi,
            'angle_increment': 2 * math.pi / 360.0,
            'range_max': 12.0,
            'ranges': [],
            'point_count': 0,
            'min_range': 0.0,
            'min_angle_deg': 0.0,
            'safety_radius': self._safety_r,
            'danger_radius': self._danger_r,
            'clusters': [],
        }

    # ───────────────────── Kamera: ROS Image topic ─────────────────────

    def _cb_annotated_image(self, msg: Image):
        """YOLO çizimli görüntü — varsa her zaman ham görüntüye tercih edilir."""
        self._last_annotated_time = time.time()
        self._store_frame(msg, source='ros:annotated')

    def _cb_raw_image(self, msg: Image):
        # İşlenmiş görüntü akıyorsa ham kareyi yok say.
        if time.time() - self._last_annotated_time < self.FRESH_SEC:
            return
        self._store_frame(msg, source='ros:raw')

    def _store_frame(self, msg: Image, source: str):
        now = time.time()
        if now - self._last_encode_time < self._min_frame_dt:
            return  # yayın hızını sınırla — gereksiz JPEG encode CPU yakar
        self._last_encode_time = now

        frame = self._imgmsg_to_bgr(msg)
        if frame is None:
            return
        self._publish_jpeg(frame, source)

    @staticmethod
    def _imgmsg_to_bgr(msg: Image):
        """sensor_msgs/Image → BGR numpy dizisi (cv_bridge bağımlılığı olmadan)."""
        import cv2

        enc = msg.encoding.lower()
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
        if channels is None:
            return None

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            return None
        img = buf[:expected].reshape(msg.height, msg.step)
        img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)

        if enc == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == 'mono8':
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    # ───────────────────── Kamera: GStreamer / cihaz ─────────────────────

    def _camera_loop(self):
        import cv2

        if self._video_source == 'gstreamer':
            pipeline = (
                f'udpsrc port={self._gst_port} ! '
                'application/x-rtp, payload=96 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! '
                'videoconvert ! appsink drop=true sync=false'
            )
            self.get_logger().info(f'Kamera açılıyor (GStreamer, port={self._gst_port})...')
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self.get_logger().info(f'Kamera açılıyor (device, index={self._camera_idx})...')
            cap = cv2.VideoCapture(self._camera_idx)

        if not cap.isOpened():
            self.get_logger().error('Kamera açılamadı — /video_feed görüntü vermeyecek.')
            return

        self.get_logger().info('Kamera akışı başladı.')
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            self._publish_jpeg(frame, self._video_source)

        cap.release()

    # ───────────────────── JPEG cache & MJPEG akışı ─────────────────────

    def _publish_jpeg(self, frame, source: str):
        import cv2

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            return
        with self._frame_lock:
            self._latest_jpeg = buf.tobytes()
            self._frame_seq += 1
            self._last_frame_time = time.time()
            self._active_source = source

    def _placeholder_jpeg(self):
        """Kaynak yokken tarayıcıya gösterilecek 'sinyal yok' karesi."""
        import cv2

        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, 'KAMERA SINYALI YOK', (110, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (60, 60, 200), 2)
        cv2.putText(img, f'kaynak: {self._video_source}', (110, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        ok, buf = cv2.imencode('.jpg', img)
        return buf.tobytes() if ok else b''

    def _camera_alive(self) -> bool:
        return (time.time() - self._last_frame_time) < self.FRESH_SEC

    def _mjpeg_generator(self):
        last_seq = -1
        placeholder_sent = False
        while True:
            with self._frame_lock:
                frame = self._latest_jpeg
                seq = self._frame_seq
                alive = (time.time() - self._last_frame_time) < self.FRESH_SEC

            if frame is not None and alive and seq != last_seq:
                last_seq = seq
                placeholder_sent = False
                payload = frame
            elif not alive and not placeholder_sent:
                placeholder_sent = True
                last_seq = -1
                payload = self._placeholder_jpeg()
            else:
                time.sleep(0.03)
                continue

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(payload)).encode() + b'\r\n\r\n'
                + payload + b'\r\n'
            )
            time.sleep(0.03)

    # ───────────────────── Flask Routes ─────────────────────

    def _setup_routes(self):
        app = self._app

        @app.route('/')
        def index():
            return render_template_string(WEB_TEMPLATE)

        @app.route('/api/telemetry')
        def api_telem():
            data = dict(self._telem)
            data['camera_ok'] = self._camera_alive()
            data['camera_source'] = self._active_source if self._camera_alive() else 'yayin yok'
            return jsonify(data)

        @app.route('/api/lidar')
        def api_lidar():
            now = time.time()
            with self._scan_lock:
                scan = self._scan
                fresh = (now - self._scan_time) < self.FRESH_SEC
                clusters = (
                    self._clusters if (now - self._clusters_time) < self.FRESH_SEC else []
                )
            if scan is None or not fresh:
                return jsonify(self._empty_scan())
            payload = dict(scan)
            payload['clusters'] = clusters
            return jsonify(payload)

        @app.route('/api/costmap')
        def api_costmap():
            with self._scan_lock:
                costmap = self._costmap
                fresh = (time.time() - self._costmap_time) < self.FRESH_SEC
            if costmap is None or not fresh:
                return jsonify(self._empty_costmap())
            return jsonify(costmap)

        @app.route('/video_feed')
        def video_feed():
            return Response(
                self._mjpeg_generator(),
                mimetype='multipart/x-mixed-replace; boundary=frame',
            )

        @app.route('/api/command', methods=['POST'])
        def api_command():
            data = request.get_json(silent=True) or {}
            action = data.get('action', '')
            if action == 'start':
                msg = Bool()
                msg.data = True
                self.pub_start.publish(msg)
                return jsonify({'result': 'Görev başlatıldı'})
            elif action == 'stop':
                msg = Bool()
                msg.data = True
                self.pub_stop.publish(msg)
                return jsonify({'result': 'Görev durduruldu'})
            return jsonify({'result': f'Bilinmeyen komut: {action}'})

        @app.route('/api/manual', methods=['POST'])
        def api_manual():
            """Manuel sürüş: {"linear": -1..1, "angular": -1..1}. Tarayıcı basılı
            tuş boyunca tekrar tekrar gönderir; akış kesilirse deadman durdurur."""
            data = request.get_json(silent=True) or {}
            try:
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
            except (TypeError, ValueError):
                return jsonify({'result': 'gecersiz komut'}), 400
            self._publish_manual(linear, angular)
            return jsonify({'linear': linear, 'angular': angular})

        @app.route('/api/arm', methods=['POST'])
        def api_arm():
            data = request.get_json(silent=True) or {}
            arm = bool(data.get('arm', False))
            msg = Bool()
            msg.data = arm
            self.pub_arm.publish(msg)
            return jsonify({'result': 'ARM komutu gonderildi' if arm else 'DISARM komutu gonderildi'})

        @app.route('/api/mode', methods=['POST'])
        def api_mode():
            data = request.get_json(silent=True) or {}
            mode = str(data.get('mode', '')).strip()
            if not mode:
                return jsonify({'result': 'mod bos'}), 400
            msg = String()
            msg.data = mode
            self.pub_mode.publish(msg)
            return jsonify({'result': f'Mod komutu: {mode}'})

        @app.route('/api/user_param', methods=['POST'])
        def api_user_param():
            """Parkur seçimi: otopilottaki SCR_USER parametresini yazar. Gerçek
            akışla birebir aynı yol — değer otopiluta gider, oradan geri okunur."""
            data = request.get_json(silent=True) or {}
            name = str(data.get('name', '')).strip()
            if not name:
                return jsonify({'result': 'parametre adi bos'}), 400
            try:
                value = float(data.get('value'))
            except (TypeError, ValueError):
                return jsonify({'result': 'deger sayisal degil'}), 400
            msg = String()
            msg.data = json.dumps({'name': name, 'value': value})
            self.pub_set_param.publish(msg)
            return jsonify({'result': f'{name} = {value:g} yazildi'})

        @app.route('/api/user_params')
        def api_user_params():
            return jsonify(self._user_params)

        @app.route('/api/toggle_yolo', methods=['POST'])
        def api_toggle_yolo():
            self._yolo_enabled = not self._yolo_enabled
            self._set_yolo_param(self._yolo_enabled)
            return jsonify({'use_yolo': self._yolo_enabled})

    def _set_yolo_param(self, value: bool):
        if not self._set_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('yolo_vision_node set_parameters servisi bulunamadı.')
            return
        req = SetParameters.Request()
        p = ParameterMsg()
        p.name = 'use_yolo'
        p.value = ParameterValue()
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = value
        req.parameters = [p]
        self._set_param_client.call_async(req)

    def _run_flask(self):
        self._app.run(host='0.0.0.0', port=self._port, threaded=True)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebUiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT'te rclpy kendi handler'ıyla context'i zaten kapatmış olabilir;
        # ikinci kez çağırmak RCLError fırlatır.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
