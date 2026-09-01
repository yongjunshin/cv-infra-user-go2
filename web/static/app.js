/* app.js — 로봇 조종실 (Go2 patrol controller, localhost only).
 *
 * ── vendored dependency ────────────────────────────────────────────────────────────
 *   roslib.min.js
 *     version : roslib 1.4.1   (the newest 1.x on npm; 'latest' is 2.1.0)
 *     source  : https://cdn.jsdelivr.net/npm/roslib@1.4.1/build/roslib.min.js
 *     sha256  : 3c510df2bc04be9d5b07efafb01eb5f473d8a5092d93292351a715ca60daf9d6
 *     bytes   : 66315
 *
 * ── why the /patrol goal is NOT sent through roslib ────────────────────────────────
 *   Checked, not assumed: this build exposes NO `ROSLIB.Action`. Its exported names are
 *   only the ROS 1 actionlib trio (ActionClient / ActionListener / SimpleActionServer),
 *   which drives goal/feedback/result *topics* — a ROS 2 action server has none of those.
 *   The bundle does not contain the string "send_action_goal" at all.
 *
 *   So /patrol is driven with the rosbridge v2 raw action ops, written straight onto the
 *   SAME websocket roslib already owns (`ros.callOnConnection` to send, a second
 *   'message' listener on `ros.socket` to receive). rosbridge_suite 2.7.0 — the version
 *   pinned in web/Dockerfile — registers `send_action_goal`, `cancel_action_goal`,
 *   and emits `action_feedback` / `action_result` (verified in its
 *   rosbridge_library/capabilities/send_action_goal.py).
 *
 *   One sharp edge that shapes the code below: rosbridge looks a running goal up by the
 *   `id` the client chose at send time (`client_handler_list[cid]`). A cancel with no id,
 *   or a different id, is silently ignored. So `patrolId` is generated once per goal and
 *   reused verbatim by the 중지 button.
 *
 * Everything the user reads is plain Korean. No topic names, no frames, no coordinates.
 */

'use strict';

/* ── what we talk to ───────────────────────────────────────────────────────────────── */

var ROSBRIDGE_URL = 'ws://localhost:9090';
var RECONNECT_MS  = 2000;

var TELEOP_TOPIC  = '/cmd_vel_teleop';
var PATROL_ACTION = '/patrol';
var PATROL_TYPE   = 'go2_msgs/action/Patrol';

/* Manual drive. The locomotion policy has a low-speed dead zone, so there is no speed
 * slider anywhere in this UI — one fixed speed while a button is held, zero on release. */
var LINEAR_MPS  = 0.4;
var ANGULAR_RPS = 0.8;   /* +z = 왼쪽(반시계), ROS convention */
var TELEOP_HZ   = 10;

/* Camera frame size the detector works in. naturalWidth wins when the stream is up;
 * these are only the fallback for the first frames. */
var IMG_W = 640, IMG_H = 480;
var DET_STALE_MS = 2000;

/* Map georeferencing — hardcoded from robot_sw/src/go2_bringup/maps/
 * carter_warehouse_navigation.yaml (resolution 0.05, origin [-11.975, -17.975, 0.0])
 * and the PNG header (480 x 776). The map never changes during a mission, so the page
 * renders the baked-in image instead of subscribing to anything.
 * map_server convention: `origin` is the image's BOTTOM-LEFT pixel -> flip Y. */
var MAP_RES = 0.05, MAP_ORIGIN_X = -11.975, MAP_ORIGIN_Y = -17.975;
var MAP_W = 480, MAP_H = 776;

var DRAW_HZ = 5;

/* ── plain-Korean vocabulary ───────────────────────────────────────────────────────── */

var TARGET_KO = { chair: '의자', person: '사람' };

var STATE_KO = {
  PROBE:       '카메라를 점검하고 있어요',
  SEARCHING:   '주변을 살피는 중이에요',
  APPROACHING: '발견! 다가가는 중이에요',
  HOLDING:     '맞는지 확인하는 중이에요'
};

var CONNECT_FAIL = '로봇에 연결할 수 없어요 — 로봇 앱이 켜져 있는지 확인해 주세요';

/* action_msgs/GoalStatus */
var STATUS_CANCELED = 5;

/* ── DOM ───────────────────────────────────────────────────────────────────────────── */

var $ = function (id) { return document.getElementById(id); };

var connBox     = $('conn'),        connText   = $('conn-text'),  banner     = $('banner');
var patrolStart = $('patrol-start'), patrolStop = $('patrol-stop');
var statusLine  = $('patrol-status'), statusDetail = $('patrol-detail');
var showDet     = $('show-det');
var camImg      = $('cam'),  camOverlay = $('cam-overlay'), camMsg = $('cam-msg');
var mapCanvas   = $('map-canvas'), mapMsg = $('map-msg');

/* ── state ─────────────────────────────────────────────────────────────────────────── */

var ros = null;
var connected = false;
var reconnectTimer = null;

var cmdVel = null;                       /* ROSLIB.Topic, advertised once per connection */
var detSub = null, scanSub = null, tfSub = null;

var driveKey = null, driveTimer = null;

var target = 'chair';
var patrolId = null;                     /* non-null == a goal is in flight */
var patrolTarget = null;

var detections = [], lastDetAt = 0;
var latestScan = null;
var tfMapOdom = null, tfOdomBase = null;

var activeTab = 'camera';

var mapImage = null, mapReady = false;   /* map.png pre-composited over white */

/* ══ connection ════════════════════════════════════════════════════════════════════ */

/* One Ros object for the life of the page, reconnected in place. Building a fresh one
 * every 2 s would pile up a dead EventEmitter + socket per retry while the robot app is
 * down, which can be hours. */
function connect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = null;

  if (!ros) {
    ros = new ROSLIB.Ros({});                        /* no url -> no auto-connect */
    ros.on('connection', onConnected);
    ros.on('close', onDisconnected);
    /* roslib emits 'error' for a refused socket too; with no listener it would throw. */
    ros.on('error', function () { onDisconnected(); });
  }
  ros.connect(ROSBRIDGE_URL);
}

function onConnected() {
  connected = true;
  connBox.className = 'conn is-on';
  connText.textContent = '연결됨';
  banner.hidden = true;

  hookRawOps(ros.socket);

  cmdVel = new ROSLIB.Topic({
    ros: ros, name: TELEOP_TOPIC, messageType: 'geometry_msgs/msg/Twist'
  });
  cmdVel.advertise();

  /* /scan and /tf feed the 지도 tab. Throttled where it is safe to drop frames — but
   * NOT on /tf: the map→odom correction from AMCL only arrives every few seconds, and a
   * throttle window would happily be the thing that eats it. */
  scanSub = new ROSLIB.Topic({
    ros: ros, name: '/scan', messageType: 'sensor_msgs/msg/LaserScan', throttle_rate: 200
  });
  scanSub.subscribe(function (m) { latestScan = m; });

  tfSub = new ROSLIB.Topic({
    ros: ros, name: '/tf', messageType: 'tf2_msgs/msg/TFMessage'
  });
  tfSub.subscribe(onTf);

  syncDetectionSub();
  refreshPatrolButtons();
}

function onDisconnected() {
  if (connected) {
    stopDrive();                          /* never leave the robot walking on a drop */
  }
  connected = false;
  connBox.className = 'conn is-off';
  connText.textContent = '연결 안 됨';
  banner.textContent = CONNECT_FAIL;
  banner.hidden = false;

  cmdVel = null; detSub = null; scanSub = null; tfSub = null;
  latestScan = null; tfMapOdom = null; tfOdomBase = null;
  detections = [];

  if (patrolId) {
    patrolId = null;
    setStatus('순찰이 중단됐어요 — 로봇과 연결이 끊겼어요.', 'is-sad');
  }
  refreshPatrolButtons();

  if (!reconnectTimer) {
    reconnectTimer = setTimeout(connect, RECONNECT_MS);
  }
}

/* ══ raw rosbridge action ops ══════════════════════════════════════════════════════ */

/* roslib 1.4.1's dispatcher knows nothing about `action_feedback` / `action_result`, so
 * we read them off the socket ourselves. addEventListener runs ALONGSIDE the `onmessage`
 * roslib assigned — it does not replace it, so ordinary topics keep working. */
function hookRawOps(socket) {
  if (!socket || socket.__go2Hooked) { return; }
  socket.__go2Hooked = true;
  if (typeof socket.addEventListener === 'function') {
    socket.addEventListener('message', onRawMessage);
  } else {
    var prev = socket.onmessage;
    socket.onmessage = function (ev) {
      onRawMessage(ev);
      if (prev) { prev.call(socket, ev); }
    };
  }
}

function onRawMessage(event) {
  if (typeof event.data !== 'string') { return; }   /* rosbridge speaks JSON text here */
  var msg;
  try { msg = JSON.parse(event.data); } catch (e) { return; }

  if (!patrolId || msg.id !== patrolId) { return; }
  if (msg.op === 'action_feedback')   { onPatrolFeedback(msg.values); }
  else if (msg.op === 'action_result') { onPatrolResult(msg); }
}

/* ══ 수동 조작 ═════════════════════════════════════════════════════════════════════ */

var DRIVE = {
  forward: [ LINEAR_MPS, 0],
  back:    [-LINEAR_MPS, 0],
  left:    [0,  ANGULAR_RPS],
  right:   [0, -ANGULAR_RPS]
};

function twist(lx, az) {
  return new ROSLIB.Message({
    linear:  { x: lx, y: 0, z: 0 },
    angular: { x: 0,  y: 0, z: az }
  });
}

function startDrive(key) {
  if (!DRIVE[key] || driveKey === key || !connected) { return; }
  stopDrive();
  driveKey = key;
  markKey(key, true);
  publishDrive();                                   /* now, not 100 ms from now */
  driveTimer = setInterval(publishDrive, 1000 / TELEOP_HZ);
}

function publishDrive() {
  if (!cmdVel || !driveKey) { return; }
  var v = DRIVE[driveKey];
  cmdVel.publish(twist(v[0], v[1]));
}

function stopDrive() {
  if (!driveKey) { return; }
  clearInterval(driveTimer);
  driveTimer = null;
  markKey(driveKey, false);
  driveKey = null;
  if (cmdVel) { cmdVel.publish(twist(0, 0)); }      /* exactly one zero on release */
}

function markKey(key, on) {
  var btn = document.querySelector('[data-drive="' + key + '"]');
  if (btn) { btn.classList.toggle('is-pressed', on); }
}

Array.prototype.forEach.call(document.querySelectorAll('[data-drive]'), function (btn) {
  var key = btn.getAttribute('data-drive');
  btn.addEventListener('pointerdown', function (e) {
    e.preventDefault();
    startDrive(key);
  });
  ['pointerup', 'pointercancel', 'pointerleave'].forEach(function (ev) {
    btn.addEventListener(ev, function () {
      if (driveKey === key) { stopDrive(); }
    });
  });
  btn.addEventListener('contextmenu', function (e) { e.preventDefault(); });
});

var WASD = { KeyW: 'forward', KeyS: 'back', KeyA: 'left', KeyD: 'right' };

window.addEventListener('keydown', function (e) {
  if (e.repeat || e.metaKey || e.ctrlKey || e.altKey) { return; }
  var key = WASD[e.code];
  if (!key) { return; }
  e.preventDefault();
  startDrive(key);
});

window.addEventListener('keyup', function (e) {
  var key = WASD[e.code];
  if (key && driveKey === key) { stopDrive(); }
});

/* Losing focus (alt-tab, minimise) never delivers a keyup — stop instead of running on. */
window.addEventListener('blur', stopDrive);
document.addEventListener('visibilitychange', function () {
  if (document.hidden) { stopDrive(); }
});

/* ══ 순찰 ══════════════════════════════════════════════════════════════════════════ */

Array.prototype.forEach.call(document.querySelectorAll('[data-target]'), function (chip) {
  chip.addEventListener('click', function () {
    if (patrolId) { return; }                        /* not mid-mission */
    target = chip.getAttribute('data-target');
    Array.prototype.forEach.call(document.querySelectorAll('[data-target]'), function (c) {
      c.classList.toggle('is-on', c === chip);
    });
  });
});

patrolStart.addEventListener('click', function () {
  if (!connected || patrolId) { return; }

  patrolId = 'patrol_' + Date.now();                 /* cancel MUST reuse this exact id */
  patrolTarget = target;

  ros.callOnConnection({
    op: 'send_action_goal',
    id: patrolId,
    action: PATROL_ACTION,
    action_type: PATROL_TYPE,
    args: { target_class: patrolTarget },
    feedback: true
  });

  setStatus(TARGET_KO[patrolTarget] + '을(를) 찾으러 출발했어요.', 'is-busy');
  statusDetail.hidden = true;
  refreshPatrolButtons();
});

patrolStop.addEventListener('click', function () {
  if (!patrolId) { return; }
  ros.callOnConnection({ op: 'cancel_action_goal', id: patrolId, action: PATROL_ACTION });
  setStatus('순찰을 중지하는 중이에요…', 'is-busy');
});

function onPatrolFeedback(values) {
  if (!values) { return; }
  setStatus(STATE_KO[values.state] || '순찰 중이에요', 'is-busy');
}

function onPatrolResult(msg) {
  var name = TARGET_KO[patrolTarget] || patrolTarget || '';
  patrolId = null;
  patrolTarget = null;
  refreshPatrolButtons();

  /* result:false == rosbridge could not even run the goal (no such action server, bad
   * type, rejected goal). `values` is the exception text, not a Patrol result. */
  if (msg.result === false) {
    setStatus('순찰을 마치지 못했어요:', 'is-sad');
    showDetail(typeof msg.values === 'string' ? msg.values : '');
    return;
  }

  var v = msg.values || {};
  if (v.found) {
    setStatus('임무 완수 — ' + name + '을(를) 찾았어요!', 'is-happy');
    statusDetail.hidden = true;
    return;
  }

  if (msg.status === STATUS_CANCELED) {
    setStatus('순찰을 중지했어요.', 'is-sad');
  } else {
    setStatus(v.message ? '순찰을 마치지 못했어요:' : '순찰을 마치지 못했어요.', 'is-sad');
  }
  showDetail(v.message || '');
}

function setStatus(text, cls) {
  statusLine.className = 'status ' + (cls || '');
  statusLine.textContent = text;
}

/* The robot's own wording, verbatim and small — it is the only place raw text is shown,
 * and it is the one thing a helper on the phone will ask the user to read out. */
function showDetail(text) {
  if (!text) { statusDetail.hidden = true; return; }
  statusDetail.textContent = text;
  statusDetail.hidden = false;
}

function refreshPatrolButtons() {
  var running = !!patrolId;
  patrolStart.disabled = running || !connected;
  patrolStop.disabled = !running;
  Array.prototype.forEach.call(document.querySelectorAll('[data-target]'), function (c) {
    c.disabled = running;
  });
}

/* ══ tabs ══════════════════════════════════════════════════════════════════════════ */

Array.prototype.forEach.call(document.querySelectorAll('[data-tab]'), function (tab) {
  tab.addEventListener('click', function () {
    activeTab = tab.getAttribute('data-tab');
    Array.prototype.forEach.call(document.querySelectorAll('[data-tab]'), function (t) {
      t.classList.toggle('is-on', t === tab);
    });
    $('view-camera').hidden = activeTab !== 'camera';
    $('view-map').hidden    = activeTab !== 'map';
  });
});

/* ══ 카메라 tab ════════════════════════════════════════════════════════════════════ */

showDet.addEventListener('change', function () {
  syncDetectionSub();
  if (!showDet.checked) { detections = []; }
});

function syncDetectionSub() {
  if (!connected) { detSub = null; return; }
  if (showDet.checked && !detSub) {
    detSub = new ROSLIB.Topic({
      ros: ros, name: '/detections',
      messageType: 'vision_msgs/msg/Detection2DArray', throttle_rate: 200
    });
    detSub.subscribe(function (m) {
      detections = m.detections || [];
      lastDetAt = Date.now();
    });
  } else if (!showDet.checked && detSub) {
    detSub.unsubscribe();
    detSub = null;
  }
}

/* The MJPEG stream is a plain <img>. If the robot app is not up yet the browser fires
 * 'error' once and gives up forever, so re-arm it on a slow timer — the user should not
 * have to know that reloading the page is a thing. */
var camRetry = 0;
camImg.addEventListener('load',  function () { camMsg.hidden = true; });
camImg.addEventListener('error', function () {
  camMsg.hidden = false;
  setTimeout(function () {
    camImg.src = camImg.src.split('&_=')[0] + '&_=' + (++camRetry);
  }, 5000);
});

function drawDetections() {
  var c = fitCanvas(camOverlay);
  if (!c) { return; }
  c.ctx.clearRect(0, 0, c.w, c.h);

  if (!showDet.checked || !detections.length) { return; }
  if (Date.now() - lastDetAt > DET_STALE_MS) { return; }

  /* Reproduce CSS `object-fit: contain` so a box lands on the pixels it describes. */
  var iw = camImg.naturalWidth  || IMG_W;
  var ih = camImg.naturalHeight || IMG_H;
  var s  = Math.min(c.w / iw, c.h / ih);
  var ox = (c.w - iw * s) / 2, oy = (c.h - ih * s) / 2;

  var ctx = c.ctx;
  ctx.font = '700 14px "Noto Sans KR", system-ui, sans-serif';
  ctx.textBaseline = 'middle';

  for (var i = 0; i < detections.length; i++) {
    var d = detections[i];
    if (!d || !d.bbox || !d.bbox.center) { continue; }

    var bw = d.bbox.size_x * s, bh = d.bbox.size_y * s;
    var bx = ox + (d.bbox.center.position.x - d.bbox.size_x / 2) * s;
    var by = oy + (d.bbox.center.position.y - d.bbox.size_y / 2) * s;

    ctx.lineWidth = 3;
    ctx.strokeStyle = '#22d36b';
    ctx.strokeRect(bx, by, bw, bh);

    var hyp = (d.results && d.results[0]) ? d.results[0].hypothesis : null;
    /* class_id is the class name; Detection2D.id carries the same name as a fallback. */
    var raw = (hyp && hyp.class_id) || d.id || '';
    var label = (TARGET_KO[raw] || raw || '무언가');
    if (hyp && typeof hyp.score === 'number') {
      label += '  ' + Math.round(hyp.score * 100) + '%';
    }

    var tw = ctx.measureText(label).width;
    var chipH = 22;
    var chipY = by - chipH - 2 >= 0 ? by - chipH - 2 : by + 2;   /* flip inside if clipped */
    ctx.fillStyle = '#22d36b';
    ctx.fillRect(bx, chipY, tw + 14, chipH);
    ctx.fillStyle = '#06240f';
    ctx.fillText(label, bx + 7, chipY + chipH / 2 + 1);
  }
}

/* ══ 지도 tab ══════════════════════════════════════════════════════════════════════ */

/* map.png is RGBA; compositing it over white once means transparency reads as free
 * space and the walls stay dark, exactly like map_server's trinary view. */
(function loadMap() {
  var img = new Image();
  img.onload = function () {
    var off = document.createElement('canvas');
    off.width = MAP_W; off.height = MAP_H;
    var octx = off.getContext('2d');
    octx.fillStyle = '#ffffff';
    octx.fillRect(0, 0, MAP_W, MAP_H);
    octx.drawImage(img, 0, 0, MAP_W, MAP_H);
    mapImage = off;
    mapReady = true;
  };
  img.src = 'map.png';
})();

function worldToPx(wx, wy) {
  return [ (wx - MAP_ORIGIN_X) / MAP_RES, MAP_H - (wy - MAP_ORIGIN_Y) / MAP_RES ];
}

function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function onTf(m) {
  var list = m.transforms || [];
  for (var i = 0; i < list.length; i++) {
    var t = list[i];
    var parent = String(t.header.frame_id).replace(/^\//, '');
    var child  = String(t.child_frame_id).replace(/^\//, '');
    var flat = {
      x: t.transform.translation.x,
      y: t.transform.translation.y,
      yaw: yawOf(t.transform.rotation)
    };
    if (parent === 'map' && child === 'odom')            { tfMapOdom = flat; }
    else if (parent === 'odom' && child === 'base_link') { tfOdomBase = flat; }
  }
}

/* map→base_link = (map→odom) ∘ (odom→base_link), 2D only. Before AMCL's first
 * correction arrives map and odom are coincident, which is what Nav2 assumes too. */
function robotPose() {
  if (!tfOdomBase) { return null; }
  var mo = tfMapOdom || { x: 0, y: 0, yaw: 0 };
  var b = tfOdomBase;
  var c = Math.cos(mo.yaw), s = Math.sin(mo.yaw);
  return {
    x: mo.x + c * b.x - s * b.y,
    y: mo.y + s * b.x + c * b.y,
    yaw: mo.yaw + b.yaw
  };
}

function drawMapView() {
  var c = fitCanvas(mapCanvas);
  if (!c) { return; }
  var ctx = c.ctx;

  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, c.w, c.h);
  if (!mapReady) { return; }

  /* whole map always visible, centred */
  var s  = Math.min(c.w / MAP_W, c.h / MAP_H);
  var ox = (c.w - MAP_W * s) / 2, oy = (c.h - MAP_H * s) / 2;
  ctx.drawImage(mapImage, ox, oy, MAP_W * s, MAP_H * s);

  var pose = robotPose();
  mapMsg.hidden = !!pose || !connected;
  if (!pose) { return; }

  /* live lidar: polar in the lidar frame ≈ base_link (the mount is a pure +z offset),
   * then rigid-transformed into the map frame by the robot pose. */
  if (latestScan && latestScan.ranges) {
    var r = latestScan.ranges;
    var cy = Math.cos(pose.yaw), sy = Math.sin(pose.yaw);
    ctx.fillStyle = '#ef6c1a';
    for (var i = 0; i < r.length; i++) {
      var d = r[i];
      if (typeof d !== 'number' || !isFinite(d)) { continue; }   /* Infinity / NaN */
      if (d < latestScan.range_min || d > latestScan.range_max) { continue; }
      var a = latestScan.angle_min + i * latestScan.angle_increment;
      var bx = d * Math.cos(a), by = d * Math.sin(a);
      var p = worldToPx(pose.x + cy * bx - sy * by, pose.y + sy * bx + cy * by);
      ctx.fillRect(ox + p[0] * s - 1.25, oy + p[1] * s - 1.25, 2.5, 2.5);
    }
  }

  /* the robot, at a fixed on-screen size so it stays findable at any zoom.
   * Canvas Y grows downward while world Y grows upward -> rotate by -yaw. */
  var rp = worldToPx(pose.x, pose.y);
  ctx.save();
  ctx.translate(ox + rp[0] * s, oy + rp[1] * s);
  ctx.rotate(-pose.yaw);
  ctx.beginPath();
  ctx.moveTo(15, 0); ctx.lineTo(-9, 9); ctx.lineTo(-4, 0); ctx.lineTo(-9, -9);
  ctx.closePath();
  ctx.fillStyle = '#16a34a';
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#04270f';
  ctx.stroke();
  ctx.restore();
}

/* ══ draw loop ═════════════════════════════════════════════════════════════════════ */

/* Backing store in device pixels (crisp on HiDPI), drawing units in CSS pixels.
 * Returns null while the canvas is inside a hidden tab and has no box. */
function fitCanvas(cv) {
  var rect = cv.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) { return null; }
  var dpr = window.devicePixelRatio || 1;
  var w = Math.round(rect.width * dpr), h = Math.round(rect.height * dpr);
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  var ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  return { ctx: ctx, w: rect.width, h: rect.height };
}

setInterval(function () {
  if (activeTab === 'camera') { drawDetections(); }
  else                        { drawMapView(); }
}, 1000 / DRAW_HZ);

/* ══ go ════════════════════════════════════════════════════════════════════════════ */

if (typeof ROSLIB === 'undefined') {
  banner.textContent = CONNECT_FAIL;
  banner.hidden = false;
} else {
  refreshPatrolButtons();
  connect();
}
