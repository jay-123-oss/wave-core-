/**
 * Wave-Core Avionics Website Application Logic
 * Manages language tabs, code copying, report viewer, and navigation
 */

(function () {
  'use strict';

  // --- 1. Multi-Language Code Switcher ---
  const langTabs = document.querySelectorAll('.tab-btn');
  const codePanels = document.querySelectorAll('.code-panel');
  const btnCopyCode = document.getElementById('btnCopyCode');
  const copyBtnText = document.getElementById('copyBtnText');

  let activeLang = 'python';

  langTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      langTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');

      activeLang = tab.dataset.lang;

      codePanels.forEach(panel => {
        panel.classList.remove('active');
        if (panel.id === `panel-${activeLang}`) {
          panel.classList.add('active');
        }
      });
    });
  });

  // Copy Snippet functionality
  if (btnCopyCode) {
    btnCopyCode.addEventListener('click', async () => {
      const activePanel = document.getElementById(`panel-${activeLang}`);
      if (!activePanel) return;

      const codeText = activePanel.textContent;
      try {
        await navigator.clipboard.writeText(codeText);
        copyBtnText.textContent = 'Copied to Clipboard! ✓';
        btnCopyCode.style.borderColor = '#00f5a0';
        btnCopyCode.style.color = '#00f5a0';

        setTimeout(() => {
          copyBtnText.textContent = 'Copy 5 Lines';
          btnCopyCode.style.borderColor = '';
          btnCopyCode.style.color = '';
        }, 2200);
      } catch (err) {
        console.error('Failed to copy code: ', err);
      }
    });
  }

  // --- 2. Technical Audit Reports Explorer (All 8 Reports & Papers) ---
  const repTabs = document.querySelectorAll('.rep-tab');
  const reportPreContent = document.getElementById('reportPreContent');
  const viewerFilePath = document.getElementById('viewerFilePath');
  const viewerReportTitle = document.getElementById('viewerReportTitle');
  const btnDownloadCurrentReport = document.getElementById('btnDownloadCurrentReport');
  const btnCopyReport = document.getElementById('btnCopyReport');

  const reportMeta = {
    unified: {
      path: 'data/UNIFIED_DRONE_AUTOPILOT_REPORT.txt',
      file: 'UNIFIED_DRONE_AUTOPILOT_REPORT.txt',
      title: 'Unified 3D Drone Autopilot Integration Report (Options 1+2+3 Converged)'
    },
    montecarlo: {
      path: 'data/MONTE_CARLO_1000_TEST_REPORT.txt',
      file: 'MONTE_CARLO_1000_TEST_REPORT.txt',
      title: '1,000-Scenario Monte Carlo Stress Test Report (0 Collisions, 100% Pass)'
    },
    realitygap: {
      path: 'data/ACTUATOR_REALITY_GAP_1000_REPORT.txt',
      file: 'ACTUATOR_REALITY_GAP_1000_REPORT.txt',
      title: 'Actuator Reality Gap & Motor Dynamics Validation Report'
    },
    baremetal: {
      path: 'data/EMBEDDED_MCU_BAREMETAL_REPORT.txt',
      file: 'EMBEDDED_MCU_BAREMETAL_REPORT.txt',
      title: 'Bare-Metal ARM Cortex-M4 Microcontroller Benchmark Report ($5 MCU)'
    },
    ros2: {
      path: 'data/ROS2_UNIVERSAL_INTERFACE_REPORT.txt',
      file: 'ROS2_UNIVERSAL_INTERFACE_REPORT.txt',
      title: 'ROS2 Universal Interface & Zero-Copy C-ABI Integration Report'
    },
    drone3d: {
      path: 'data/DRONE_3D_VOLUMETRIC_ENGINE_REPORT.txt',
      file: 'DRONE_3D_VOLUMETRIC_ENGINE_REPORT.txt',
      title: '3D Volumetric 6-DOF Flight Engine & 48-Beam Spherical Mesh Report'
    },
    realworld: {
      path: 'data/REAL_WORLD_DATASET_1000_REPORT.txt',
      file: 'REAL_WORLD_DATASET_1000_REPORT.txt',
      title: 'Real-World Robotics Sensor Dataset 1,000 Evaluation Report'
    },
    hil: {
      path: 'data/HIL_PX4_GAZEBO_CLOSED_LOOP_REPORT.txt',
      file: 'HIL_PX4_GAZEBO_CLOSED_LOOP_REPORT.txt',
      title: 'PX4 Autopilot + Gazebo Harmonic Closed-Loop HIL Benchmark Report'
    }
  };

  let currentReportKey = 'unified';

  function displayReport(key) {
    currentReportKey = key;
    const meta = reportMeta[key] || { path: key, file: key + '.txt', title: key };
    
    if (viewerFilePath) viewerFilePath.textContent = meta.path;
    if (viewerReportTitle) viewerReportTitle.textContent = meta.title;

    if (window.WAVE_CORE_REPORTS && window.WAVE_CORE_REPORTS[key]) {
      const dataObj = window.WAVE_CORE_REPORTS[key];
      reportPreContent.textContent = typeof dataObj === 'object' ? dataObj.content : dataObj;
    } else {
      // Fallback: try fetching from data/
      fetch(`data/${meta.file}`)
        .then(res => res.text())
        .then(text => {
          reportPreContent.textContent = text;
        })
        .catch(() => {
          reportPreContent.textContent = 'Unable to load report text. Please inspect the raw file in the repository.';
        });
    }
  }

  repTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      repTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      displayReport(tab.dataset.report);
    });
  });

  // Initial load
  displayReport('unified');

  // Download currently selected report
  if (btnDownloadCurrentReport) {
    btnDownloadCurrentReport.addEventListener('click', () => {
      const content = reportPreContent.textContent;
      const meta = reportMeta[currentReportKey] || { file: 'report.txt' };
      const blob = new Blob([content], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = meta.file;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    });
  }

  // Copy raw report text
  if (btnCopyReport) {
    btnCopyReport.addEventListener('click', async () => {
      const content = reportPreContent.textContent;
      try {
        await navigator.clipboard.writeText(content);
        const originalText = btnCopyReport.textContent;
        btnCopyReport.textContent = 'Copied! ✓';
        btnCopyReport.style.color = '#059669';
        setTimeout(() => {
          btnCopyReport.textContent = originalText;
          btnCopyReport.style.color = '';
        }, 2000);
      } catch (err) {
        console.error('Failed to copy: ', err);
      }
    });
  }

  // --- 3. Hero CTA Scroll Handlers ---
  const btnHeroReports = document.getElementById('btnHeroReports');
  if (btnHeroReports) {
    btnHeroReports.addEventListener('click', (e) => {
      e.preventDefault();
      document.getElementById('reports')?.scrollIntoView({ behavior: 'smooth' });
    });
  }

  const btnHeroThesis = document.getElementById('btnHeroThesis');
  if (btnHeroThesis) {
    btnHeroThesis.addEventListener('click', (e) => {
      e.preventDefault();
      document.getElementById('investor')?.scrollIntoView({ behavior: 'smooth' });
    });
  }

  // --- 4. Mobile Menu Toggle ---
  const mobileToggle = document.getElementById('mobileToggle');
  const navMenu = document.getElementById('navMenu');

  if (mobileToggle && navMenu) {
    mobileToggle.addEventListener('click', () => {
      navMenu.classList.toggle('open');
      mobileToggle.classList.toggle('active');
    });

    document.querySelectorAll('.nav-link').forEach(link => {
      link.addEventListener('click', () => {
        navMenu.classList.remove('open');
        mobileToggle.classList.remove('active');
      });
    });
  }

  // --- 5. Sticky Navbar, Scroll Progress & Back-to-Top ---
  const navbar = document.getElementById('navbar');
  const scrollProgress = document.getElementById('scrollProgress');
  const backToTop = document.getElementById('backToTop');

  function onScroll() {
    const y = window.scrollY || document.documentElement.scrollTop;

    if (navbar) {
      navbar.classList.toggle('scrolled', y > 40);
    }

    if (scrollProgress) {
      const docH = document.documentElement.scrollHeight - window.innerHeight;
      const pct = docH > 0 ? Math.min((y / docH) * 100, 100) : 0;
      scrollProgress.style.width = pct + '%';
    }

    if (backToTop) {
      backToTop.classList.toggle('visible', y > 600);
    }
  }

  window.addEventListener('scroll', onScroll, { passive: true });
  window.addEventListener('resize', onScroll);
  onScroll();

  if (backToTop) {
    backToTop.addEventListener('click', () => {
      window.scrollTo({ top: 0, behavior: 'smooth' });
    });
  }

  // --- 6. Active Nav Link Scrollspy ---
  const navLinks = Array.from(document.querySelectorAll('.nav-link'));
  const spySections = navLinks
    .map(link => {
      const id = (link.getAttribute('href') || '').trim();
      return id.startsWith('#') && id.length > 1 ? document.querySelector(id) : null;
    })
    .filter(Boolean);

  if ('IntersectionObserver' in window && spySections.length) {
    const spyObserver = new IntersectionObserver(
      entries => {
        entries.forEach(entry => {
          if (!entry.isIntersecting) return;
          const id = '#' + entry.target.id;
          navLinks.forEach(l => l.classList.toggle('active', l.getAttribute('href') === id));
        });
      },
      { rootMargin: '-45% 0px -50% 0px', threshold: 0 }
    );
    spySections.forEach(sec => spyObserver.observe(sec));
  }

  // --- 7. Premium Scroll Reveal Animations ---
  const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  if (!prefersReducedMotion && 'IntersectionObserver' in window) {
    const revealTargets = document.querySelectorAll(
      '.section-header, .metric-card, .hero-pill, .hero-title, .hero-description, .hero-cta-group, .hero-visual, ' +
      '.sim-display-card, .sim-control-card, .compare-card, .code-showcase-box, .pillar-card, ' +
      '.benchmark-chart-card, .benchmark-table-card, .report-explorer-card, .hw-card, .download-banner, ' +
      '.chart-bar-group, .footer-brand, .footer-col'
    );

    // Stagger siblings that share a parent for a cascading feel
    const seenParents = new Map();
    revealTargets.forEach(el => {
      const key = el.parentElement;
      const idx = seenParents.get(key) || 0;
      seenParents.set(key, idx + 1);
      el.classList.add('reveal');
      el.style.setProperty('--reveal-delay', Math.min(idx * 90, 360) + 'ms');
    });

    document.documentElement.classList.add('js-motion');

    const revealObserver = new IntersectionObserver(
      (entries, obs) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            entry.target.classList.add('is-visible');
            obs.unobserve(entry.target);
          }
        });
      },
      { rootMargin: '0px 0px -8% 0px', threshold: 0.08 }
    );

    revealTargets.forEach(el => revealObserver.observe(el));

    // Fallback: very fast scrolling can skip IntersectionObserver samples,
    // so reconcile visibility on every scroll/resize frame. This guarantees
    // content is never left invisible, regardless of scroll speed.
    let revealQueued = false;
    const revealInView = () => {
      revealQueued = false;
      document.querySelectorAll('.reveal:not(.is-visible)').forEach(el => {
        if (el.getBoundingClientRect().top < window.innerHeight * 0.94) {
          el.classList.add('is-visible');
        }
      });
    };
    const queueRevealCheck = () => {
      if (revealQueued) return;
      revealQueued = true;
      requestAnimationFrame(revealInView);
    };
    window.addEventListener('scroll', queueRevealCheck, { passive: true });
    window.addEventListener('resize', queueRevealCheck);
    window.addEventListener('load', queueRevealCheck);

    // Safety net: if anything is still hidden after 2.5s (e.g. observer quirk), show it
    setTimeout(() => {
      document.querySelectorAll('.reveal:not(.is-visible)').forEach(el => {
        const r = el.getBoundingClientRect();
        if (r.top < window.innerHeight) el.classList.add('is-visible');
      });
    }, 2500);
  }

  // --- 8. R&D Frontier Domains Filtering ---
  const rdFilterBtns = document.querySelectorAll('.rd-filter-btn');
  const rdCards = document.querySelectorAll('.rd-card');

  if (rdFilterBtns.length && rdCards.length) {
    rdFilterBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        rdFilterBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        const filter = btn.dataset.filter;

        rdCards.forEach(card => {
          if (filter === 'all' || card.dataset.domain === filter) {
            card.style.display = 'flex';
            card.style.opacity = '1';
            card.style.transform = 'translateY(0)';
          } else {
            card.style.display = 'none';
          }
        });
      });
    });
  }

  // --- 9. Hero Visual Showcase Switcher ---
  const heroVisualTabs = document.querySelectorAll('.hero-v-tab');
  const heroDroneImg = document.getElementById('heroDroneImg');
  const heroHudTag = document.getElementById('heroHudTag');
  const heroHudVal = document.getElementById('heroHudVal');

  if (heroVisualTabs.length && heroDroneImg) {
    heroVisualTabs.forEach(tab => {
      tab.addEventListener('click', () => {
        heroVisualTabs.forEach(t => t.classList.remove('active'));
        tab.classList.add('active');

        const mode = tab.dataset.mode;
        if (mode === 'executive') {
          heroDroneImg.src = 'assets/executive_airbag_hud.jpg';
          heroDroneImg.alt = '3D Executive Airbag HUD with Barrier Detected Mesh Wall';
          if (heroHudTag) heroHudTag.textContent = '3D EXECUTIVE AIRBAG HUD // BARRIER DETECTED';
          if (heroHudVal) heroHudVal.textContent = '2.85 µs REFLEX · NO-STD RUST';
        } else {
          heroDroneImg.src = 'assets/hero_drone_light.jpg';
          heroDroneImg.alt = 'Wave-Core 3D Volumetric Radar Shield Around Drone';
          if (heroHudTag) heroHudTag.textContent = '3D VOLUMETRIC RADAR SHIELD';
          if (heroHudVal) heroHudVal.textContent = '48 CHANNELS · 200 Hz IMU';
        }
      });
    });
  }

  // --- 10. HIL Verification Suite Video & Telemetry Switcher ---
  const vTabBtns = document.querySelectorAll('.v-tab-btn');
  const vVideoPlayer = document.getElementById('vVideoPlayer');
  const vMissionTitle = document.getElementById('vMissionTitle');
  const vMissionDesc = document.getElementById('vMissionDesc');
  const vSpeedVal = document.getElementById('vSpeedVal');
  const vHazardVal = document.getElementById('vHazardVal');
  const vClearanceVal = document.getElementById('vClearanceVal');
  const vLatencyVal = document.getElementById('vLatencyVal');
  const vReflexVal = document.getElementById('vReflexVal');

  const missionData = {
    full3d: {
      video: 'videos/wave_core_3d_simulation_demo.mp4',
      title: '3D Real-Time Multi-View Flight Simulator (4 Conditions Tested)',
      desc: 'Pure 3D physics-driven simulation with 3 synchronized views (Chase Cam, Forward Sensor FPV, Side Ortho Clearance Ruler). Rigorously demonstrates autonomous reflex actions across Concrete Blast Wall (climb vault), 2cm High-Voltage Cable (45° bank slalom), Severe Downdraft Vortex (1.85m altitude clamp), and Rising Incline Terrain.',
      speed: '8.0 m/s – 12.0 m/s Dynamic Cruise',
      hazard: 'Multi-Condition (Wall, 2cm Cable, Vortex, Incline)',
      clearance: '2.01m Min Safe Clearance Preserved',
      latency: '2.85 µs (Deterministic C-ABI)',
      reflex: 'Autonomous Multi-Axis Aerospace Reflex Suite'
    },
    m1: {
      video: 'videos/m1_wall_abort.mp4',
      title: 'Mission 1: Blast Wall Collision Abort (8.0 m/s Cruise)',
      desc: 'Quadcopter cruising at 8.0 m/s encounters an emergency blast wall barrier at 4.8m. Evaluated via C-ABI at 2.85µs with zero memory allocation, triggering dynamic climb vault (+4.5 m/s) and active retro-braking.',
      speed: '8.0 m/s (28.8 km/h)',
      hazard: 'Reinforced Blast Barrier (4.8m trigger)',
      clearance: '2.01 m Remaining',
      latency: '2.85 µs (Deterministic)',
      reflex: 'Climb Vault +4.5 m/s & Full Retro-Brake'
    },
    m2: {
      video: 'videos/m2_ground_dive.mp4',
      title: 'Mission 2: Steep Ground Dive Recovery & Vortex Rejection',
      desc: 'High-speed negative vertical descent with severe crosswind disturbance. Wave-Core Disturbance Observer detects downward vortex and dynamically clamps the ground floor buffer, commanding a positive +6.0 m/s emergency flare.',
      speed: '6.5 m/s (Descent -4.2 m/s)',
      hazard: 'Downwash Vortex & Ground Impact Floor',
      clearance: '1.24 m Safe Floor',
      latency: '2.84 µs (Deterministic)',
      reflex: 'Emergency Flare +6.0 m/s & Ground Clamp'
    },
    m3: {
      video: 'videos/m3_wire_slalom.mp4',
      title: 'Mission 3: 2cm High-Voltage Thin Wire Snag Prevention',
      desc: 'A 2cm power transmission wire crosses the flight path—undetected by traditional 2D planar LiDAR and ignored by 40ms Nav2 planners. Wave-Core 48-beam persistence filter registers the obstruction and commands an evasive slalom climb.',
      speed: '7.2 m/s Forward Cruise',
      hazard: '2cm High-Voltage Wire Strand',
      clearance: '0.94 m Dynamic Margin',
      latency: '2.87 µs (Deterministic)',
      reflex: 'Evasive Slalom S-Curve & Climb +3.8 m/s'
    }
  };

  if (vTabBtns.length && vVideoPlayer) {
    vTabBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        vTabBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        const mKey = btn.dataset.mission;
        const d = missionData[mKey];
        if (!d) return;

        vVideoPlayer.src = d.video;
        vVideoPlayer.load();
        vVideoPlayer.play().catch(() => {});

        if (vMissionTitle) vMissionTitle.textContent = d.title;
        if (vMissionDesc) vMissionDesc.textContent = d.desc;
        if (vSpeedVal) vSpeedVal.textContent = d.speed;
        if (vHazardVal) vHazardVal.textContent = d.hazard;
        if (vClearanceVal) vClearanceVal.textContent = d.clearance;
        if (vLatencyVal) vLatencyVal.textContent = d.latency;
        if (vReflexVal) vReflexVal.textContent = d.reflex;
      });
    });
  }

  // --- 11. Wave-Studio CLI Terminal Switcher ---
  const termTabBtns = document.querySelectorAll('.term-tab-btn');
  const termBodyCode = document.getElementById('termBodyCode');

  const cliSnippets = {
    tune: `# 1. Optimize distance and velocity trigger envelopes
$ wave-studio tune --profile high-speed-recon \\
  --max-speed 15.0 --d-trigger 3.5 \\
  --clearance-floor 0.8 --output config.toml

✓ Optimizing parameters for high-speed-recon (15.0 m/s)...
✓ Synthesizing 48-beam distance thresholds...
✓ Generated config.toml: Invariant safety guaranteed down to 2.85µs.`,

    replay: `# 2. Dry-run flight telemetry playback through Wave-Core
$ wave-studio replay --telemetry artifacts/hil_telemetry.csv \\
  --hz 200 --assert-zero-breach

[00:00.000] Stream started: 100 frames ingested (48-beam).
[00:00.480] Hazard detected at 4.2m! Distance envelope breached.
[00:00.480] OVERRIDE: MAVLink #84 dispatched in 2.85 µs.
✓ Replay completed: 0 invariant violations across 100 frames.`,

    check: `# 3. Validate C-ABI struct alignment and memory constraints
$ wave-studio check --schema c-abi --abi-version 1.0

Checking struct WavePointcloud48 (192 bytes)... OK
Checking struct WaveDroneAttitude (24 bytes)... OK
Checking deterministic stack footprint: 3.8 KB (< 4 KB target)... OK
✓ All 54 invariant tests PASSED. C-ABI 100% compliant.`
  };

  if (termTabBtns.length && termBodyCode) {
    termTabBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        termTabBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        const cmd = btn.dataset.cmd;
        if (cliSnippets[cmd]) {
          termBodyCode.textContent = cliSnippets[cmd];
        }
      });
    });
  }

  // --- 12. Investor & OEM Intake Form Flow ---
  const intakeToggleBtns = document.querySelectorAll('.intake-toggle-btn');
  const intakeTitle = document.getElementById('intakeTitle');
  const intakeSub = document.getElementById('intakeSub');
  const intakeSubmitBtn = document.getElementById('intakeSubmitBtn');
  const intakeOrgLabel = document.getElementById('intakeOrgLabel');
  const intakeFleetLabel = document.getElementById('intakeFleetLabel');
  const intakeForm = document.getElementById('intakeForm');
  const intakeSuccessCard = document.getElementById('intakeSuccessCard');
  const generatedTokenVal = document.getElementById('generatedTokenVal');

  let currentFlow = 'oem'; // 'oem' or 'investor'

  if (intakeToggleBtns.length) {
    intakeToggleBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        intakeToggleBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        currentFlow = btn.dataset.flow;
        if (currentFlow === 'oem') {
          if (intakeTitle) intakeTitle.textContent = 'Request 30-Day Physical Bench Integration SDK';
          if (intakeSub) intakeSub.textContent = 'For Tier-1 Drone OEMs, Autopilot Engineers & System Integrators.';
          if (intakeOrgLabel) intakeOrgLabel.textContent = 'Company / Drone OEM Name';
          if (intakeFleetLabel) intakeFleetLabel.textContent = 'Autopilot & Companion Hardware (e.g., PX4 / Pixhawk 6X / Jetson)';
          if (intakeSubmitBtn) intakeSubmitBtn.textContent = 'Request OEM Hardware SDK Access';
        } else {
          if (intakeTitle) intakeTitle.textContent = 'Request DeepTech Data Room Access';
          if (intakeSub) intakeSub.textContent = 'For Institutional Funds, Defense Angels & Strategic Corporate Investors.';
          if (intakeOrgLabel) intakeOrgLabel.textContent = 'Venture Capital / Investment Fund / Angel Syndicate';
          if (intakeFleetLabel) intakeFleetLabel.textContent = 'Target Allocation / iSAFE Check Size & Jurisdiction';
          if (intakeSubmitBtn) intakeSubmitBtn.textContent = 'Request Verified Data Room Access (iSAFE / HIL Logs)';
        }
      });
    });
  }

  if (intakeForm) {
    intakeForm.addEventListener('submit', (e) => {
      e.preventDefault();

      const randomHex = Array.from({ length: 8 }, () => Math.floor(Math.random() * 16).toString(16)).join('').toUpperCase();
      const prefix = currentFlow === 'oem' ? 'WAVE-OEM' : 'WAVE-INV';
      const token = `${prefix}-${randomHex}-2026`;

      if (generatedTokenVal) generatedTokenVal.textContent = token;
      if (intakeForm) intakeForm.style.display = 'none';
      if (intakeSuccessCard) intakeSuccessCard.style.display = 'block';
    });
  }

  console.log('Wave-Core Avionics & Wave-OS Platform Suite initialized. Deterministic 2.85µs C-ABI active.');
})();

