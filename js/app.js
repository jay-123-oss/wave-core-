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
      path: '/home/jaydeep/Documents/AI core/Robotics/UNIFIED_DRONE_AUTOPILOT_REPORT.txt',
      file: 'UNIFIED_DRONE_AUTOPILOT_REPORT.txt',
      title: 'Unified 3D Drone Autopilot Integration Report (Options 1+2+3 Converged)'
    },
    montecarlo: {
      path: '/home/jaydeep/Documents/AI core/Robotics/MONTE_CARLO_1000_TEST_REPORT.txt',
      file: 'MONTE_CARLO_1000_TEST_REPORT.txt',
      title: '1,000-Scenario Monte Carlo Stress Test Report (0 Collisions, 100% Pass)'
    },
    realitygap: {
      path: '/home/jaydeep/Documents/AI core/Robotics/ACTUATOR_REALITY_GAP_1000_REPORT.txt',
      file: 'ACTUATOR_REALITY_GAP_1000_REPORT.txt',
      title: 'Actuator Reality Gap & Motor Dynamics Validation Report'
    },
    baremetal: {
      path: '/home/jaydeep/Documents/AI core/Robotics/EMBEDDED_MCU_BAREMETAL_REPORT.txt',
      file: 'EMBEDDED_MCU_BAREMETAL_REPORT.txt',
      title: 'Bare-Metal ARM Cortex-M4 Microcontroller Benchmark Report ($5 MCU)'
    },
    ros2: {
      path: '/home/jaydeep/Documents/AI core/Robotics/ROS2_UNIVERSAL_INTERFACE_REPORT.txt',
      file: 'ROS2_UNIVERSAL_INTERFACE_REPORT.txt',
      title: 'ROS2 Universal Interface & Zero-Copy C-ABI Integration Report'
    },
    drone3d: {
      path: '/home/jaydeep/Documents/AI core/Robotics/DRONE_3D_VOLUMETRIC_ENGINE_REPORT.txt',
      file: 'DRONE_3D_VOLUMETRIC_ENGINE_REPORT.txt',
      title: '3D Volumetric 6-DOF Flight Engine & 48-Beam Spherical Mesh Report'
    },
    realworld: {
      path: '/home/jaydeep/Documents/AI core/Robotics/REAL_WORLD_DATASET_1000_REPORT.txt',
      file: 'REAL_WORLD_DATASET_1000_REPORT.txt',
      title: 'Real-World Robotics Sensor Dataset 1,000 Evaluation Report'
    },
    paper: {
      path: '/home/jaydeep/Documents/AI core/Robotics/RESEARCH_PAPER.md',
      file: 'RESEARCH_PAPER.md',
      title: 'Formal Scientific Research Paper: Hyperdimensional Neuromorphic Reflexes'
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

  console.log('Wave-Core Avionics Engine v2.4 initialized. All systems ready.');
})();
