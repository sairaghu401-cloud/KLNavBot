/**
 * KLNavBot — Dashboard Script
 * Handles Firebase real-time sync, map rendering,
 * OTP verification, manual controls, and robot animation.
 *
 * Author : KLNavBot Team
 * Version: 1.0.0
 */

'use strict';

// ============================================================
// 1. FIREBASE CONFIGURATION
// ============================================================
const FIREBASE_CONFIG = {
  apiKey:            "AIzaSyD2C_IbPssaeiSZGz69iuG5QBQb3Lk24bI",
  authDomain:        "klu-robot-tracker.firebaseapp.com",
  databaseURL:       "https://klu-robot-tracker-default-rtdb.firebaseio.com/",
  projectId:         "klu-robot-tracker"
};

firebase.initializeApp(FIREBASE_CONFIG);
const db   = firebase.database();
const auth = firebase.auth();

// Sign in anonymously so Firebase rules can validate uid-based access
auth.signInAnonymously().catch(err => console.warn("Auth error:", err));

// ============================================================
// 2. CAMPUS LOCATION DATA  [longitude, latitude]
// ============================================================
const CAMPUS_LOCATIONS = {
  'canteen':       { name: 'Main Canteen',         coords: [80.62423437677018, 16.441589404127136] },
  'c-block':       { name: 'C Block',               coords: [80.6222172981291,  16.440340583595606] },
  's-block':       { name: 'S Block',               coords: [80.6215211613485,  16.443536461341907] },
  'rnd-block':     { name: 'R&D Block',             coords: [80.62364799551604, 16.440581535802988] },
  'm-block':       { name: 'M Block',               coords: [80.62271974542571, 16.440781648425897] },
  'fed-block':     { name: 'FED Block',             coords: [80.62159988406823, 16.44129213890485]  },
  'l-block':       { name: 'L Block (Library)',     coords: [80.62225562037058, 16.441823047549892] },
  'arts-block':    { name: 'Arts Block',            coords: [80.62302206542779, 16.441635187741856] },
  'kgh-hostel':    { name: 'KGH Boys Hostel',       coords: [80.6210607959365,  16.44302252269531]  },
  'kl-hostel':     { name: 'KL Girls Hostel',       coords: [80.62488282365112, 16.441124698166586] },
  'sports-complex':{ name: 'Sports Complex',        coords: [80.62351173864059, 16.442635743364043] }
};

// ORS Routing API key
const ORS_API_KEY = "eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6IjlmN2NhNzFmMGVkNDQ2ZTNhYjM0MzE2NDA4YjkxNjhjIiwiaCI6Im11cm11cjY0In0=";

// ============================================================
// 3. MAP INITIALISATION (OpenLayers)
// ============================================================
let map, markerLayer, routeLayer;
let liveRobotFeature, simulatedRobotFeature;

function initMap() {
  map = new ol.Map({
    target: 'map',
    layers: [ new ol.layer.Tile({ source: new ol.source.OSM() }) ],
    view: new ol.View({
      center: ol.proj.fromLonLat(CAMPUS_LOCATIONS['canteen'].coords),
      zoom: 16.5
    })
  });

  markerLayer = new ol.layer.Vector({ source: new ol.source.Vector() });
  map.addLayer(markerLayer);
}

/** Add a static pin marker to the map */
function addMarker(coord, color, label) {
  const feature = new ol.Feature({
    geometry: new ol.geom.Point(ol.proj.fromLonLat(coord))
  });
  feature.setStyle(new ol.style.Style({
    image: new ol.style.Circle({
      radius: 9,
      fill:   new ol.style.Fill({ color }),
      stroke: new ol.style.Stroke({ color: '#fff', width: 3 })
    }),
    text: new ol.style.Text({
      text: label, offsetY: -22, font: 'bold 12px sans-serif',
      fill: new ol.style.Fill({ color: '#1f2937' }),
      stroke: new ol.style.Stroke({ color: 'white', width: 4 }),
      backgroundFill: new ol.style.Fill({ color: 'rgba(255,255,255,0.9)' }),
      padding: [2, 4, 2, 4]
    })
  }));
  markerLayer.getSource().addFeature(feature);
}

/** Create the animated robot marker (red dot + 🤖 emoji) */
function createRobotMarker(coord) {
  const feature = new ol.Feature({
    geometry: new ol.geom.Point(ol.proj.fromLonLat(coord))
  });
  feature.setStyle([
    new ol.style.Style({ image: new ol.style.Circle({ radius: 18, fill: new ol.style.Fill({ color: 'rgba(239,68,68,0.18)' }), stroke: new ol.style.Stroke({ color: '#ef4444', width: 2 }) }) }),
    new ol.style.Style({ image: new ol.style.Circle({ radius: 10,  fill: new ol.style.Fill({ color: '#ef4444' }), stroke: new ol.style.Stroke({ color: '#fff', width: 3 }) }) }),
    new ol.style.Style({ text: new ol.style.Text({ text: '🤖', font: '24px sans-serif', offsetY: -30 }) })
  ]);
  return feature;
}

// ============================================================
// 4. ROUTING  (OpenRouteService)
// ============================================================
let routeCoordinates = [];        // current outbound path
let currentStart  = 'canteen';
let currentTarget = 'c-block';

/** Fetch a walking route from ORS and draw it on the map */
async function loadRoute() {
  const startLoc  = CAMPUS_LOCATIONS[currentStart];
  const targetLoc = CAMPUS_LOCATIONS[currentTarget];

  if (!startLoc || !targetLoc) return;
  if (currentStart === currentTarget) {
    alert('Start and target must be different!');
    return;
  }

  // Reset UI
  stopAnimation();
  markerLayer.getSource().clear();
  if (routeLayer) map.removeLayer(routeLayer);
  document.getElementById('routeBanner').innerHTML = `<strong>Loading…</strong><span>Fetching campus path</span>`;
  setStatus('loading', 'Loading…');
  document.getElementById('startBtn').disabled = true;

  addMarker(startLoc.coords,  '#10b981', 'Start');
  addMarker(targetLoc.coords, '#3b82f6', 'Target');

  try {
    const data = await fetchRoute([startLoc.coords, targetLoc.coords]);
    routeCoordinates = data.features[0].geometry.coordinates;
    const seg = data.features[0].properties.segments[0];

    drawRouteLine(data, '#ef4444');
    fitRoute();

    // Simulated robot marker (for dashboard demo)
    simulatedRobotFeature = createRobotMarker(startLoc.coords);
    markerLayer.getSource().addFeature(simulatedRobotFeature);

    animState.totalDistance = seg.distance;
    animState.distanceTraveled = 0;
    animState.battery = 85;

    const eta = Math.round(seg.duration / 60);
    document.getElementById('routeBanner').innerHTML =
      `<strong>🚀 ${startLoc.name} → ${targetLoc.name}</strong><span>${Math.round(seg.distance)}m · ETA ${eta} min</span>`;

    document.getElementById('deliveryRoute').textContent  = `${startLoc.name} → ${targetLoc.name}`;
    document.getElementById('deliveryPackage').textContent = `Package: #KLB${Math.floor(1000 + Math.random() * 9000)}`;

    setStatus('idle', 'Ready');
    document.getElementById('startBtn').disabled  = false;
    document.getElementById('pauseBtn').disabled  = true;

  } catch (err) {
    console.error('Route fetch failed:', err);
    document.getElementById('routeBanner').innerHTML = `<strong>❌ Route Error</strong><span>${err.message}</span>`;
    setStatus('error', 'Error');
  }
}

/** Call ORS directions API */
async function fetchRoute(coordPairs) {
  const res = await fetch(
    "https://api.openrouteservice.org/v2/directions/foot-walking/geojson",
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'Authorization': ORS_API_KEY },
      body: JSON.stringify({ coordinates: coordPairs })
    }
  );
  if (!res.ok) throw new Error(`ORS ${res.status}`);
  return res.json();
}

/** Draw the route GeoJSON as a coloured line */
function drawRouteLine(geojson, color) {
  if (routeLayer) map.removeLayer(routeLayer);
  const features = new ol.format.GeoJSON().readFeatures(geojson, { featureProjection: 'EPSG:3857' });
  routeLayer = new ol.layer.Vector({
    source: new ol.source.Vector({ features }),
    style:  new ol.style.Style({ stroke: new ol.style.Stroke({ color, width: 7, lineCap: 'round', lineJoin: 'round' }) })
  });
  map.addLayer(routeLayer);
}

function fitRoute() {
  if (!routeLayer) return;
  map.getView().fit(routeLayer.getSource().getExtent(), { padding: [80,80,80,80], duration: 800 });
}

// ============================================================
// 5. ROBOT ANIMATION (simulated movement along route)
// ============================================================
const animState = {
  isRunning:    false,
  isPaused:     false,
  isReturning:  false,
  deliveryDone: false,
  frameId:      null,
  currentIndex: 0,
  progress:     0.0,
  battery:      85,
  distanceTraveled: 0,
  totalDistance: 0,
  returnCoords: []
};

function animateRobot() {
  if (!animState.isRunning || animState.isPaused) return;

  const coords = animState.isReturning ? animState.returnCoords : routeCoordinates;
  if (!coords || coords.length < 2) return;

  if (animState.currentIndex >= coords.length - 1) {
    if (!animState.isReturning) {
      triggerDeliveryCompletion();
    } else {
      onRouteComplete();
    }
    return;
  }

  const a = coords[animState.currentIndex];
  const b = coords[animState.currentIndex + 1];

  // Speed: ~0.02 progress steps per frame ≈ 1.2 m/s visual
  animState.progress += 0.022;

  if (animState.progress >= 1) {
    animState.progress = 0;
    animState.currentIndex++;
  }

  const lon = a[0] + (b[0] - a[0]) * animState.progress;
  const lat = a[1] + (b[1] - a[1]) * animState.progress;

  simulatedRobotFeature.getGeometry().setCoordinates(ol.proj.fromLonLat([lon, lat]));

  // Stats update
  const dLon = b[0] - a[0], dLat = b[1] - a[1];
  const segM  = Math.sqrt(dLon * dLon + dLat * dLat) * 111320;
  animState.distanceTraveled += segM * 0.022;
  animState.battery = Math.max(0, animState.battery - 0.0008);

  document.getElementById('speed').textContent    = '1.2 m/s';
  document.getElementById('distance').textContent = Math.round(animState.distanceTraveled) + ' m';
  document.getElementById('battery').textContent  = animState.battery.toFixed(1) + '%';

  const pct = Math.min(100, Math.round(
    (animState.currentIndex + animState.progress) / coords.length * 100
  ));
  setStatus('active', animState.isReturning ? `Returning ${pct}%` : `En Route ${pct}%`);

  animState.frameId = requestAnimationFrame(animateRobot);
}

function stopAnimation() {
  if (animState.frameId) cancelAnimationFrame(animState.frameId);
  animState.isRunning = false;
  animState.frameId   = null;
}

function resetAnimState() {
  stopAnimation();
  Object.assign(animState, {
    isPaused: false, isReturning: false, deliveryDone: false,
    currentIndex: 0, progress: 0, battery: 85,
    distanceTraveled: 0, totalDistance: 0, returnCoords: []
  });
}

// ============================================================
// 6. OTP SYSTEM
// ============================================================
let generatedOTP = null;
let otpTimeoutId = null;

function generateOTP() {
  return String(Math.floor(1000 + Math.random() * 9000));
}

/** Called when simulated robot reaches the target */
function triggerDeliveryCompletion() {
  animState.isRunning  = false;
  animState.deliveryDone = true;
  document.getElementById('speed').textContent = '0 m/s';

  generatedOTP = generateOTP();

  // Push OTP to Firebase
  db.ref('deliveryOTP').set({
    otp:       generatedOTP,
    delivered: true,
    verified:  false,
    createdAt: Date.now()
  });

  setStatus('paused', '⏳ Awaiting OTP');
  document.getElementById('otpSection').style.display = 'block';
  document.getElementById('otpInput').focus();

  // Auto-return if OTP not entered in 30 seconds
  otpTimeoutId = setTimeout(() => {
    hideOTPSection();
    setStatus('error', 'OTP timeout — returning');
    setTimeout(beginReturnJourney, 1500);
  }, 30000);
}

function verifyOTP() {
  const input = document.getElementById('otpInput').value.trim();
  const msgEl = document.getElementById('otpMessage');

  if (input.length !== 4) {
    msgEl.textContent = 'Please enter a 4-digit code.';
    msgEl.className   = 'otp-msg error';
    return;
  }

  if (input === generatedOTP) {
    msgEl.textContent = '✅ Verified! Robot returning…';
    msgEl.className   = 'otp-msg success';
    clearTimeout(otpTimeoutId);

    db.ref('deliveryOTP').update({ verified: true, verifiedAt: Date.now() });

    setTimeout(() => {
      hideOTPSection();
      beginReturnJourney();
    }, 1800);
  } else {
    msgEl.textContent = '❌ Incorrect code. Try again.';
    msgEl.className   = 'otp-msg error';
    document.getElementById('otpInput').value = '';
    document.getElementById('otpInput').focus();
  }
}

function hideOTPSection() {
  document.getElementById('otpSection').style.display = 'none';
  document.getElementById('otpInput').value  = '';
  document.getElementById('otpMessage').textContent = '';
}

// ============================================================
// 7. RETURN JOURNEY
// ============================================================
async function beginReturnJourney() {
  const startLoc  = CAMPUS_LOCATIONS[currentStart];
  const targetLoc = CAMPUS_LOCATIONS[currentTarget];

  setStatus('active', 'Loading return route…');

  try {
    const data = await fetchRoute([targetLoc.coords, startLoc.coords]);
    animState.returnCoords  = data.features[0].geometry.coordinates;

    drawRouteLine(data, '#f59e0b');    // amber for return
    document.getElementById('deliveryRoute').textContent = `Returning → ${startLoc.name}`;

    animState.isReturning   = true;
    animState.isRunning     = true;
    animState.isPaused      = false;
    animState.currentIndex  = 0;
    animState.progress      = 0;

    animateRobot();
  } catch (err) {
    console.error('Return route error:', err);
    onRouteComplete();
  }
}

function onRouteComplete() {
  stopAnimation();
  setStatus('active', '✅ Mission Complete!');
  document.getElementById('speed').textContent = '0 m/s';
  document.getElementById('startBtn').disabled = true;
  document.getElementById('pauseBtn').disabled = true;
  document.getElementById('deliveryRoute').textContent = 'Delivery complete';
}

// ============================================================
// 8. FIREBASE REAL-TIME LISTENERS
// ============================================================

/** Listen for live GPS coordinates pushed by ESP32 */
function listenForLiveGPS() {
  db.ref('robotGPS').on('value', snapshot => {
    const data = snapshot.val();
    if (!data || data.lat == null || data.lon == null) return;

    const pos = ol.proj.fromLonLat([data.lon, data.lat]);

    // Update live marker (separate from simulated)
    if (!liveRobotFeature) {
      liveRobotFeature = createRobotMarker([data.lon, data.lat]);
      markerLayer.getSource().addFeature(liveRobotFeature);
    } else {
      liveRobotFeature.getGeometry().setCoordinates(pos);
    }

    document.getElementById('gpsCoords').textContent =
      `${data.lat.toFixed(5)}, ${data.lon.toFixed(5)}`;
  });
}

/** Listen for robot status (obstacle, state) from ESP32 */
function listenForRobotStatus() {
  db.ref('robotStatus').on('value', snapshot => {
    const data = snapshot.val();
    if (!data) return;

    const alertEl = document.getElementById('obstacleAlert');
    const stateEl = document.getElementById('robotStateText');

    if (data.obstacle) {
      alertEl.classList.add('show');
      stateEl.textContent = '🚧 OBSTACLE';
      stateEl.style.color = '#ef4444';
      // Disable forward/left/right when obstacle is detected
      ['btn-forward','btn-left','btn-right'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = true;
      });
    } else {
      alertEl.classList.remove('show');
      stateEl.textContent = data.state || 'Idle';
      stateEl.style.color = '#6b7280';
      ['btn-forward','btn-left','btn-right'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = false;
      });
    }
  });
}

// ============================================================
// 9. MANUAL CONTROL (send commands to ESP32 via Firebase)
// ============================================================
function sendCommand(cmd) {
  db.ref('robotCommands').update({
    move:      cmd,
    timestamp: Date.now()
  }).catch(err => console.error('Command error:', err));

  // Visual feedback: briefly highlight the pressed button
  const btnId = `btn-${cmd}`;
  const el = document.getElementById(btnId);
  if (el) {
    el.style.filter = 'brightness(1.3)';
    setTimeout(() => { el.style.filter = ''; }, 200);
  }
}

// ============================================================
// 10. UI HELPERS
// ============================================================
function setStatus(type, text) {
  const badge = document.getElementById('statusBadge');
  badge.textContent = text;
  badge.className   = `badge badge-${type}`;
}

function swapLocations() {
  const s = document.getElementById('startLocation');
  const t = document.getElementById('targetLocation');
  const tmp = s.value;
  s.value = t.value;
  t.value = tmp;
  currentStart  = s.value;
  currentTarget = t.value;
  resetAnimState();
  loadRoute();
}

// ============================================================
// 11. BUTTON EVENT LISTENERS
// ============================================================
document.getElementById('startLocation').addEventListener('change', e => {
  currentStart = e.target.value;
  resetAnimState();
  loadRoute();
});

document.getElementById('targetLocation').addEventListener('change', e => {
  currentTarget = e.target.value;
  resetAnimState();
  loadRoute();
});

document.getElementById('startBtn').addEventListener('click', () => {
  if (animState.isRunning) return;
  animState.isRunning = true;
  animState.isPaused  = false;
  setStatus('active', 'En Route 0%');
  document.getElementById('startBtn').disabled = true;
  document.getElementById('pauseBtn').disabled = false;
  animateRobot();
});

document.getElementById('pauseBtn').addEventListener('click', () => {
  if (!animState.isRunning) return;
  animState.isPaused = !animState.isPaused;

  if (animState.isPaused) {
    setStatus('paused', 'Paused');
    document.getElementById('pauseBtn').textContent = 'Resume';
    document.getElementById('speed').textContent    = '0 m/s';
  } else {
    setStatus('active', 'Resumed');
    document.getElementById('pauseBtn').textContent = 'Pause';
    animateRobot();
  }
});

document.getElementById('resetBtn').addEventListener('click', () => {
  clearTimeout(otpTimeoutId);
  generatedOTP = null;
  hideOTPSection();
  resetAnimState();

  document.getElementById('speed').textContent    = '0 m/s';
  document.getElementById('distance').textContent = '0 m';
  document.getElementById('battery').textContent  = '85%';
  document.getElementById('pauseBtn').textContent  = 'Pause';
  document.getElementById('startBtn').disabled    = false;
  document.getElementById('pauseBtn').disabled    = true;

  loadRoute();
});

// Allow Enter key for OTP input
document.getElementById('otpInput').addEventListener('keydown', e => {
  if (e.key === 'Enter') verifyOTP();
  if (!/[0-9]/.test(e.key) && !['Backspace','Tab','ArrowLeft','ArrowRight','Delete'].includes(e.key)) {
    e.preventDefault();
  }
});

// ============================================================
// 12. INITIALISE
// ============================================================
initMap();
listenForLiveGPS();
listenForRobotStatus();
loadRoute();
