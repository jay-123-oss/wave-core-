/**
 * Wave-Core Avionics 3D Volumetric Radar & Reflex Simulator
 * Interactive Digital Twin rendering 48-beam spherical mesh, tilt compensation,
 * wind disturbance observation, and sub-6µs safety reflex overrides.
 */

(function () {
  'use strict';

  // Canvas & Context
  const canvas = document.getElementById('reflexCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // Simulator State
  const state = {
    mode: 'wavecore', // 'wavecore' or 'unprotected'
    scenario: 'wall', // 'wall', 'pole', 'dive', 'wind'
    isPlannerFrozen: false,
    hasCrashed: false,
    overrideActive: false,
    
    // Physics & Attitude
    droneX: 180,
    droneY: 260,
    baseAltitude: 260,
    altitude: 4.2, // meters
    pitchDeg: -25.0, // cruising tilt
    rollDeg: 2.1,
    forwardSpeed: 3.0, // m/s
    verticalSpeed: 0.0, // m/s
    windForce: 3.6, // Newtons
    windEstimate: 0.0, // Observer convergence
    
    // Obstacles
    obstacleX: 620,
    obstacleY: 260,
    obstacleType: 'wall', // 'wall', 'pole', 'ground'
    obstacleDist: 4.8, // meters
    
    // Rotor Animation
    rotorAngle: 0,
    particles: [],
    windParticles: [],
    crashParticles: [],
    
    // Time
    lastTime: performance.now(),
    cycleTimeUs: 2.85,
  };

  // DOM Elements
  const simModeBadge = document.getElementById('simModeBadge');
  const canvasStatusText = document.getElementById('canvasStatusText');
  const canvasArbiterText = document.getElementById('canvasArbiterText');
  const canvasAlertBanner = document.getElementById('canvasAlertBanner');
  const canvasAlertMsg = document.getElementById('canvasAlertMsg');
  const telAttitude = document.getElementById('telAttitude');
  const telWind = document.getElementById('telWind');
  const telVx = document.getElementById('telVx');
  const telVz = document.getElementById('telVz');
  const telMargin = document.getElementById('telMargin');
  const mavlinkHexStream = document.getElementById('mavlinkHexStream');
  const modeDescription = document.getElementById('modeDescription');

  // Controls
  const btnEnableWaveCore = document.getElementById('btnEnableWaveCore');
  const btnDisableWaveCore = document.getElementById('btnDisableWaveCore');
  const sliderSpeed = document.getElementById('sliderSpeed');
  const sliderPitch = document.getElementById('sliderPitch');
  const sliderWind = document.getElementById('sliderWind');
  const valSpeed = document.getElementById('valSpeed');
  const valPitch = document.getElementById('valPitch');
  const valWind = document.getElementById('valWind');
  const btnSimulateFreeze = document.getElementById('btnSimulateFreeze');

  // Scenario Buttons
  const scenarioBtns = document.querySelectorAll('.btn-scenario');

  // Initialize Wind Particles
  for (let i = 0; i < 40; i++) {
    state.windParticles.push({
      x: Math.random() * canvas.width,
      y: Math.random() * canvas.height,
      speed: 2 + Math.random() * 4,
      length: 15 + Math.random() * 25,
      alpha: 0.1 + Math.random() * 0.25
    });
  }

  // Event Listeners for Controls
  if (btnEnableWaveCore) {
    btnEnableWaveCore.addEventListener('click', () => setMode('wavecore'));
  }
  if (btnDisableWaveCore) {
    btnDisableWaveCore.addEventListener('click', () => setMode('unprotected'));
  }

  function setMode(newMode) {
    state.mode = newMode;
    state.hasCrashed = false;
    state.overrideActive = false;
    state.droneX = 180;
    state.droneY = state.baseAltitude;
    state.isPlannerFrozen = false;

    if (newMode === 'wavecore') {
      btnEnableWaveCore.classList.add('active');
      btnDisableWaveCore.classList.remove('active');
      simModeBadge.textContent = 'MODE: WAVE-CORE PROTECTED';
      simModeBadge.style.color = '#00f2fe';
      simModeBadge.style.borderColor = 'rgba(0,242,254,0.4)';
      modeDescription.textContent = 'Wave-Core transparently monitors sensor streams and overrides in 2.85 µs if a hazard is detected.';
    } else {
      btnDisableWaveCore.classList.add('active');
      btnEnableWaveCore.classList.remove('active');
      simModeBadge.textContent = 'MODE: UNPROTECTED (LEGACY PLANNER)';
      simModeBadge.style.color = '#ff3366';
      simModeBadge.style.borderColor = 'rgba(255,51,102,0.4)';
      modeDescription.textContent = 'WARNING: Wave-Core disabled. If planner delays or freezes, the drone will impact obstacles directly!';
    }
  }

  scenarioBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      scenarioBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      setScenario(btn.dataset.scenario);
    });
  });

  function setScenario(scen) {
    state.scenario = scen;
    state.hasCrashed = false;
    state.overrideActive = false;
    state.droneX = 180;
    state.droneY = state.baseAltitude;
    state.isPlannerFrozen = false;
    
    if (scen === 'wall') {
      state.obstacleType = 'wall';
      state.obstacleX = 640;
      state.obstacleY = 260;
    } else if (scen === 'pole') {
      state.obstacleType = 'pole';
      state.obstacleX = 600;
      state.obstacleY = 250;
    } else if (scen === 'dive') {
      state.obstacleType = 'ground';
      state.obstacleX = 580;
      state.obstacleY = 410;
      state.verticalSpeed = -2.5; // High speed downward dive
    } else if (scen === 'wind') {
      state.obstacleType = 'wall';
      state.obstacleX = 640;
      state.obstacleY = 260;
      state.windForce = 4.5;
      sliderWind.value = 4.5;
      valWind.textContent = '4.5 N';
    }
  }

  // Slider Listeners
  if (sliderSpeed) {
    sliderSpeed.addEventListener('input', (e) => {
      state.forwardSpeed = parseFloat(e.target.value);
      valSpeed.textContent = state.forwardSpeed.toFixed(1) + ' m/s';
    });
  }
  if (sliderPitch) {
    sliderPitch.addEventListener('input', (e) => {
      state.pitchDeg = parseFloat(e.target.value);
      valPitch.textContent = state.pitchDeg.toFixed(1) + '°';
    });
  }
  if (sliderWind) {
    sliderWind.addEventListener('input', (e) => {
      state.windForce = parseFloat(e.target.value);
      valWind.textContent = state.windForce.toFixed(1) + ' N';
    });
  }

  // Freeze Button
  if (btnSimulateFreeze) {
    btnSimulateFreeze.addEventListener('click', () => {
      state.isPlannerFrozen = true;
      state.droneX = 220;
      state.droneY = state.baseAltitude;
      state.hasCrashed = false;
      state.overrideActive = false;
      
      canvasAlertMsg.textContent = 'PLANNER CRASH SIMULATED: NODE HUNG AT 3.0 m/s!';
      canvasAlertBanner.classList.add('active');
      setTimeout(() => {
        if (!state.overrideActive && !state.hasCrashed) {
          canvasAlertBanner.classList.remove('active');
        }
      }, 2500);
    });
  }

  // Main Simulation Loop
  function animate(now) {
    const dt = Math.min((now - state.lastTime) / 1000, 0.05);
    state.lastTime = now;

    updatePhysics(dt);
    drawScene();

    requestAnimationFrame(animate);
  }

  // Physics & Reflex Evaluation
  function updatePhysics(dt) {
    state.rotorAngle += 0.45;

    // Wind observer convergence
    state.windEstimate += (state.windForce - state.windEstimate) * 0.08;

    if (state.hasCrashed) {
      // Update crash explosion particles
      state.crashParticles.forEach(p => {
        p.x += p.vx;
        p.y += p.vy;
        p.vy += 0.15;
        p.alpha -= 0.015;
      });
      state.crashParticles = state.crashParticles.filter(p => p.alpha > 0);
      return;
    }

    // Distance to obstacle in meters (80 pixels = 1.0 meter)
    const pxToMeters = 1 / 80;
    const dx = (state.obstacleX - state.droneX) * pxToMeters;
    const dy = (state.obstacleY - state.droneY) * pxToMeters;
    state.obstacleDist = Math.max(0, Math.sqrt(dx * dx + dy * dy) - 0.5);

    // Reflex Arbiter Trigger Check
    const hazardThreshold = 2.4; // 2.4 meters safety envelope
    const isImminentCollision = (state.obstacleDist < hazardThreshold) && (dx > 0);

    if (isImminentCollision) {
      if (state.mode === 'wavecore') {
        // Wave-Core Safety Arbiter Takes Over in 2.85 µs!
        state.overrideActive = true;
        state.cycleTimeUs = 2.85;

        // Emergency Climb & Braking Action
        if (state.scenario === 'dive') {
          state.verticalSpeed = 6.0; // Urgent pull-up climb
        } else {
          state.forwardSpeed = Math.max(0, state.forwardSpeed - 6.0 * dt); // Hard brake
          state.verticalSpeed = 4.5; // Climb over obstacle
        }

        // Apply Climb Movement
        state.droneY -= state.verticalSpeed * 80 * dt;
        state.droneX += state.forwardSpeed * 40 * dt;

        // Display Alert Banner
        canvasAlertMsg.textContent = '⚡ WAVE-CORE REFLEX OVERRIDE (2.85 µs): CLIMB +6.0 m/s & BRAKE APPLIED';
        canvasAlertBanner.classList.add('active');

        // MAVLink Hex packet update (SET_POSITION_TARGET_LOCAL_NED)
        if (mavlinkHexStream) {
          mavlinkHexStream.textContent = 'FD 35 00 00 01 01 01 54 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 C0 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 00 A2 4F';
        }

      } else {
        // Unprotected Mode: Traditional Planner Latency (>40,000 µs)
        // Drone moves unchecked into obstacle
        state.droneX += state.forwardSpeed * 80 * dt;
        if (state.scenario === 'dive') {
          state.droneY -= state.verticalSpeed * 80 * dt;
        }

        if (state.obstacleDist <= 0.2) {
          // CATASTROPHIC IMPACT!
          triggerCrash();
        }
      }
    } else {
      // Normal Clear Path Flight
      state.overrideActive = false;
      canvasAlertBanner.classList.remove('active');

      // Forward motion
      state.droneX += state.forwardSpeed * 65 * dt;

      // Reset drone when it leaves right edge
      if (state.droneX > canvas.width + 60) {
        state.droneX = 80;
        state.droneY = state.baseAltitude;
        state.verticalSpeed = (state.scenario === 'dive') ? -2.5 : 0.0;
      }
    }

    // Telemetry updates
    updateTelemetryUI();
  }

  function triggerCrash() {
    state.hasCrashed = true;
    state.overrideActive = false;
    canvasAlertMsg.textContent = '💥 CRASH DETECTED: PLANNER FAILED TO REACT IN TIME (42ms LAG)';
    canvasAlertBanner.classList.add('active');

    // Spawn explosion particles
    state.crashParticles = [];
    for (let i = 0; i < 60; i++) {
      const angle = Math.random() * Math.PI * 2;
      const speed = 2 + Math.random() * 8;
      state.crashParticles.push({
        x: state.droneX,
        y: state.droneY,
        vx: Math.cos(angle) * speed,
        vy: Math.sin(angle) * speed,
        color: Math.random() > 0.4 ? '#ff3366' : '#ffd200',
        size: 3 + Math.random() * 5,
        alpha: 1.0
      });
    }

    setTimeout(() => {
      // Auto respawn after 3 seconds
      if (state.hasCrashed) {
        state.hasCrashed = false;
        state.droneX = 140;
        state.droneY = state.baseAltitude;
        canvasAlertBanner.classList.remove('active');
      }
    }, 3200);
  }

  function updateTelemetryUI() {
    if (telAttitude) {
      telAttitude.textContent = `${state.pitchDeg.toFixed(1)}° / ${state.rollDeg.toFixed(1)}°`;
    }
    if (telWind) {
      telWind.textContent = `${state.windEstimate.toFixed(2)} N`;
    }
    if (telVx) {
      telVx.textContent = `${state.forwardSpeed.toFixed(2)} m/s`;
    }
    if (telVz) {
      telVz.textContent = `${state.verticalSpeed.toFixed(2)} m/s`;
    }
    if (telMargin) {
      telMargin.textContent = `${state.obstacleDist.toFixed(2)} m (${state.obstacleDist < 2.0 ? 'HAZARD' : 'SAFE'})`;
      telMargin.className = state.obstacleDist < 2.0 ? 'tel-val text-danger' : 'tel-val text-emerald';
    }

    if (canvasStatusText) {
      if (state.hasCrashed) {
        canvasStatusText.textContent = 'CRASHED (IMPACT)';
        canvasStatusText.className = 'hud-m-val text-danger';
      } else if (state.overrideActive) {
        canvasStatusText.textContent = 'REFLEX OVERRIDE ACTIVE';
        canvasStatusText.className = 'hud-m-val text-danger';
      } else {
        canvasStatusText.textContent = 'ARMED & MONITORING';
        canvasStatusText.className = 'hud-m-val text-emerald';
      }
    }

    if (canvasArbiterText) {
      if (state.overrideActive) {
        canvasArbiterText.textContent = 'INTERVENING (2.85 µs)';
        canvasArbiterText.className = 'hud-m-val text-emerald';
      } else {
        canvasArbiterText.textContent = 'STANDBY (0% OVERHEAD)';
        canvasArbiterText.className = 'hud-m-val';
      }
    }
  }

  // Canvas Drawing Routine
  function drawScene() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Draw High-Tech Grid & Facility Background
    drawGrid();

    // 2. Draw Wind Disturbance Stream
    drawWindStream();

    // 3. Draw Terrain / Ground Floor
    drawTerrain();

    // 4. Draw Obstacle (Wall, Pole, or Cable)
    drawObstacle();

    // 5. Draw 48-Beam Spherical Volumetric Mesh (The Core Innovation)
    if (!state.hasCrashed) {
      drawVolumetricMesh();
    }

    // 6. Draw Quadcopter Drone Body
    if (!state.hasCrashed) {
      drawDrone();
    } else {
      drawCrashExplosion();
    }
  }

  function drawGrid() {
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    const step = 40;
    for (let x = 0; x < canvas.width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }
  }

  function drawWindStream() {
    ctx.lineWidth = 1.5;
    state.windParticles.forEach(p => {
      p.x -= p.speed * (state.windForce / 3.0);
      if (p.x < 0) p.x = canvas.width;

      ctx.strokeStyle = `rgba(217, 119, 6, ${p.alpha * (state.windForce / 3.5)})`;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(p.x - p.length, p.y);
      ctx.stroke();
    });
  }

  function drawTerrain() {
    const groundY = 440;
    // Ground Gradient
    const grad = ctx.createLinearGradient(0, groundY, 0, canvas.height);
    grad.addColorStop(0, 'rgba(2, 132, 199, 0.12)');
    grad.addColorStop(1, 'rgba(2, 132, 199, 0.02)');

    ctx.fillStyle = grad;
    ctx.fillRect(0, groundY, canvas.width, canvas.height - groundY);

    ctx.strokeStyle = '#0284c7';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, groundY);
    ctx.lineTo(canvas.width, groundY);
    ctx.stroke();

    // Distance Marks
    ctx.fillStyle = '#64748b';
    ctx.font = '11px JetBrains Mono';
    for (let x = 100; x < canvas.width; x += 160) {
      const meters = ((x - 100) / 80).toFixed(1);
      ctx.fillText(`${meters}m`, x, groundY + 18);
    }
  }

  function drawObstacle() {
    if (state.scenario === 'wall') {
      // Concrete Wall
      const w = 45;
      const h = 260;
      const topY = state.obstacleY - h / 2;

      ctx.fillStyle = 'rgba(225, 29, 72, 0.12)';
      ctx.fillRect(state.obstacleX, topY, w, h);

      ctx.strokeStyle = '#e11d48';
      ctx.lineWidth = 2.5;
      ctx.strokeRect(state.obstacleX, topY, w, h);

      // Warning Stripes
      ctx.strokeStyle = 'rgba(217, 119, 6, 0.6)';
      ctx.lineWidth = 2;
      for (let y = topY + 20; y < topY + h; y += 30) {
        ctx.beginPath();
        ctx.moveTo(state.obstacleX, y);
        ctx.lineTo(state.obstacleX + w, y - 15);
        ctx.stroke();
      }

      ctx.fillStyle = '#be123c';
      ctx.font = 'bold 11px JetBrains Mono';
      ctx.fillText('CONCRETE HAZARD', state.obstacleX - 15, topY - 10);

    } else if (state.scenario === 'pole') {
      // High Voltage Power Cable & Thin Pole
      ctx.strokeStyle = '#d97706';
      ctx.lineWidth = 6;
      ctx.beginPath();
      ctx.moveTo(state.obstacleX, 100);
      ctx.lineTo(state.obstacleX, 440);
      ctx.stroke();

      // Power Lines
      ctx.strokeStyle = 'rgba(217, 119, 6, 0.8)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 160);
      ctx.lineTo(canvas.width, 160);
      ctx.stroke();

      ctx.fillStyle = '#b45309';
      ctx.font = 'bold 11px JetBrains Mono';
      ctx.fillText('POWER CABLE (THIN HAZARD)', state.obstacleX - 70, 140);
    }
  }

  // 48-Beam Spherical Volumetric Mesh
  function drawVolumetricMesh() {
    const x = state.droneX;
    const y = state.droneY;
    const radius = 110;
    const tiltRad = (state.pitchDeg * Math.PI) / 180;

    // Boundary Ring
    ctx.strokeStyle = state.overrideActive ? 'rgba(225, 29, 72, 0.5)' : 'rgba(2, 132, 199, 0.35)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.stroke();

    // 1. Ceiling Beams (8 channels)
    for (let i = 0; i < 8; i++) {
      const angle = -Math.PI / 2 + (i - 3.5) * 0.18;
      drawBeam(x, y, angle, radius * 0.9, 'rgba(124, 58, 237, 0.7)', 'ceiling');
    }

    // 2. Planar Beams (32 channels - tilt-compensated)
    for (let i = 0; i < 32; i++) {
      const angle = (i / 32) * Math.PI * 2 + (tiltRad * 0.25);
      const isForwardSector = (angle > -Math.PI / 4 && angle < Math.PI / 4);
      const isHazardBeam = isForwardSector && (state.obstacleDist < 2.4);

      const color = isHazardBeam 
        ? '#e11d48' 
        : (isForwardSector ? '#059669' : 'rgba(2, 132, 199, 0.65)');

      drawBeam(x, y, angle, isHazardBeam ? Math.min(radius, state.obstacleDist * 80) : radius, color, 'planar');
    }

    // 3. Ground Beams (8 channels)
    for (let i = 0; i < 8; i++) {
      const angle = Math.PI / 2 + (i - 3.5) * 0.18;
      drawBeam(x, y, angle, radius * 0.85, 'rgba(217, 119, 6, 0.75)', 'ground');
    }
  }

  function drawBeam(ox, oy, angle, len, strokeStyle, type) {
    const ex = ox + Math.cos(angle) * len;
    const ey = oy + Math.sin(angle) * len;

    ctx.strokeStyle = strokeStyle;
    ctx.lineWidth = strokeStyle.includes('e11d48') ? 2.5 : 1.5;
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ex, ey);
    ctx.stroke();

    // Endpoint dot
    ctx.fillStyle = strokeStyle;
    ctx.beginPath();
    ctx.arc(ex, ey, strokeStyle.includes('e11d48') ? 3.5 : 2, 0, Math.PI * 2);
    ctx.fill();
  }

  // Draw High-Tech Drone (Light Theme Crisp Styling)
  function drawDrone() {
    ctx.save();
    ctx.translate(state.droneX, state.droneY);
    ctx.rotate((state.pitchDeg * Math.PI) / 180);

    // Arms
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(-36, -6);
    ctx.lineTo(36, -6);
    ctx.stroke();

    // Center Pod Body
    ctx.fillStyle = '#ffffff';
    ctx.strokeStyle = state.overrideActive ? '#e11d48' : '#0284c7';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.roundRect(-18, -12, 36, 18, 6);
    ctx.fill();
    ctx.stroke();

    // Camera Lens
    ctx.fillStyle = '#0284c7';
    ctx.beginPath();
    ctx.arc(14, -3, 3, 0, Math.PI * 2);
    ctx.fill();

    // Landing Skids
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-16, 6);
    ctx.lineTo(-12, 16);
    ctx.lineTo(12, 16);
    ctx.lineTo(16, 6);
    ctx.stroke();

    // Rotating Rotors
    drawRotor(-36, -8);
    drawRotor(36, -8);

    // Emergency Climb Vector Thrust Arrow
    if (state.overrideActive) {
      ctx.strokeStyle = '#059669';
      ctx.fillStyle = '#059669';
      ctx.lineWidth = 3.5;
      ctx.beginPath();
      ctx.moveTo(0, -18);
      ctx.lineTo(0, -58);
      ctx.stroke();

      // Arrow head
      ctx.beginPath();
      ctx.moveTo(-7, -46);
      ctx.lineTo(0, -62);
      ctx.lineTo(7, -46);
      ctx.fill();

      ctx.font = 'bold 11px JetBrains Mono';
      ctx.fillText('+6.0 m/s CLIMB', 12, -45);
    }

    ctx.restore();
  }

  function drawRotor(rx, ry) {
    const rotorWidth = 24 * Math.cos(state.rotorAngle);
    ctx.strokeStyle = 'rgba(15, 23, 42, 0.75)';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(rx - rotorWidth, ry);
    ctx.lineTo(rx + rotorWidth, ry);
    ctx.stroke();

    ctx.fillStyle = '#0284c7';
    ctx.beginPath();
    ctx.arc(rx, ry, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawCrashExplosion() {
    state.crashParticles.forEach(p => {
      ctx.fillStyle = p.color;
      ctx.globalAlpha = p.alpha;
      ctx.beginPath();
      ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.globalAlpha = 1.0;
  }

  // Start Animation Loop
  requestAnimationFrame(animate);

})();
