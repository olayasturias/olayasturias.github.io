// Home-page timeline: entries show label + title; the entry whose title has most recently
// crossed the middle of the viewport opens to show its description and image.
// Markup: _includes/home_timeline.liquid. Styles: assets/css/main.scss.
// Without JS, or with prefers-reduced-motion, every entry stays open (the rail still fills).
//
// Panel heights are driven by critically damped springs rather than CSS transitions, so
// an entry that reverses mid-motion keeps its velocity instead of restarting a fixed-length
// animation, and the scroll correction that keeps the open entry steady runs on the same
// frame as the height change it corrects.
(() => {
  const timeline = document.querySelector(".home-timeline");
  if (!timeline) return;

  const items = Array.from(timeline.querySelectorAll(".tl-item"));
  if (!items.length) return;

  // Scroll progress on the rail (replaces the page-top progress bar on the home page):
  // the fill runs down to the viewport midline, and dots above it read as passed.
  // Kept under reduced motion too: it tracks the scroll 1:1, like the bar it replaces.
  const track = timeline.querySelector(".tl-track");
  const rail = timeline.querySelector(".tl-rail");
  const updateRail = () => {
    if (!track || !rail) return;
    const midline = window.innerHeight / 2;
    const railRect = rail.getBoundingClientRect();
    const fill = Math.min(1, Math.max(0, (midline - railRect.top) / railRect.height));
    rail.style.setProperty("--tl-fill", fill.toFixed(4));
    for (const item of items) {
      const title = item.querySelector(".tl-title") || item;
      item.classList.toggle("is-passed", title.getBoundingClientRect().top <= midline);
    }
  };
  let railTicking = false;
  const requestRail = () => {
    if (!railTicking) {
      railTicking = true;
      requestAnimationFrame(() => {
        railTicking = false;
        updateRail();
      });
    }
  };
  window.addEventListener("scroll", requestRail, { passive: true });
  window.addEventListener("resize", requestRail);
  updateRail();

  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  if (reducedMotion.matches) return;

  // Spring: damping ratio 1.0 (no overshoot), response 0.4 s.
  const RESPONSE_S = 0.4;
  const STIFFNESS = (2 * Math.PI / RESPONSE_S) ** 2;
  const DAMPING = 2 * Math.sqrt(STIFFNESS);
  const MAX_STEP_S = 1 / 120;

  // Scrolling back up, the open entry closes only once its title is this far below
  // the midline, so it doesn't flicker open/closed right at the line.
  const HYSTERESIS_PX = 32;
  // While flinging faster than this, entries flying past don't open; the one that
  // ends up at the midline opens once the scroll slows down.
  const FLING_PX_PER_MS = 2.5;

  timeline.classList.add("tl-js");

  const states = items.map((item) => {
    const detail = item.querySelector(".tl-detail");
    const inner = detail && detail.querySelector(".tl-detail-inner");
    return { item, detail, inner, title: item.querySelector(".tl-title") || item, x: 0, v: 0, open: false };
  });

  let active = null;
  let anchor = null; // title kept steady on screen while heights change
  let running = false;
  let lastFrame = 0;

  const naturalHeight = (s) => (s.inner ? s.inner.offsetHeight : 0);
  const docTop = (el) => el.getBoundingClientRect().top + window.scrollY;

  const render = (s, target) => {
    if (!s.detail) return;
    s.detail.style.height = `${Math.max(0, s.x)}px`;
    s.item.style.setProperty("--tl-p", target > 0 ? Math.min(1, Math.max(0, s.x / target)).toFixed(3) : "0");
  };

  const frame = (now) => {
    const dt = Math.min((now - lastFrame) / 1000, 1 / 30);
    lastFrame = now;
    const anchorBefore = anchor ? docTop(anchor) : 0;

    let moving = false;
    for (const s of states) {
      if (!s.detail) continue;
      // Re-measured every frame: lazy-loaded images change the open height.
      const target = s.open ? naturalHeight(s) : 0;
      const full = naturalHeight(s);
      if (Math.abs(s.x - target) < 0.5 && Math.abs(s.v) < 4) {
        if (s.x !== target) {
          s.x = target;
          s.v = 0;
          render(s, full);
        }
        continue;
      }
      for (let left = dt; left > 0; left -= MAX_STEP_S) {
        const h = Math.min(MAX_STEP_S, left);
        s.v += (-STIFFNESS * (s.x - target) - DAMPING * s.v) * h;
        s.x += s.v * h;
      }
      render(s, full);
      moving = true;
    }

    // Heights above the anchor changed this frame; cancel the shift so the entry being
    // read stays still. "instant" overrides the page's smooth scroll-behavior.
    if (anchor) {
      const shift = docTop(anchor) - anchorBefore;
      if (Math.abs(shift) > 0.1) {
        window.scrollBy({ top: shift, behavior: "instant" });
        // Not the reader's scroll: marine snow must not react to it (effects.js).
        if (window.siteEffects) window.siteEffects.compensateScroll(shift);
      }
    }

    updateRail();
    if (moving) {
      requestAnimationFrame(frame);
    } else {
      running = false;
      anchor = null;
    }
  };

  const run = () => {
    if (running) return;
    running = true;
    lastFrame = performance.now();
    requestAnimationFrame(frame);
  };

  const activate = (item) => {
    if (item === active) return;
    for (const s of states) {
      s.open = s.item === item;
      s.item.classList.toggle("is-active", s.open);
    }
    active = item;
    anchor = item ? states[items.indexOf(item)].title : null;
    run();
  };

  // Scroll velocity, smoothed, to tell a fling from reading.
  let lastY = window.scrollY;
  let lastT = performance.now();
  let velocity = 0;
  let settleTimer = 0;
  let ticking = false;

  const titleTop = (item) => states[items.indexOf(item)].title.getBoundingClientRect().top;

  const update = () => {
    ticking = false;
    const now = performance.now();
    const dy = window.scrollY - lastY;
    const dtMs = Math.max(now - lastT, 1);
    velocity = 0.6 * velocity + 0.4 * (dy / dtMs);
    lastY = window.scrollY;
    lastT = now;

    const rect = timeline.getBoundingClientRect();
    if (rect.bottom < 0 || rect.top > window.innerHeight) return;

    if (Math.abs(velocity) > FLING_PX_PER_MS) {
      clearTimeout(settleTimer);
      settleTimer = setTimeout(() => {
        velocity = 0;
        update();
      }, 120);
      return;
    }

    const midline = window.innerHeight / 2;
    let target = null;
    for (const item of items) {
      if (titleTop(item) <= midline) target = item;
    }

    const movingBack = active && items.indexOf(target) < items.indexOf(active);
    if (movingBack && titleTop(active) <= midline + HYSTERESIS_PX) return;
    activate(target);
  };

  const requestUpdate = () => {
    if (!ticking) {
      ticking = true;
      requestAnimationFrame(update);
    }
  };

  // Keyboard users: tabbing into an entry opens it.
  timeline.addEventListener("focusin", (event) => {
    const item = event.target.closest(".tl-item");
    if (item) activate(item);
  });

  window.addEventListener("scroll", requestUpdate, { passive: true });
  window.addEventListener("resize", () => {
    run();
    requestUpdate();
  });
  reducedMotion.addEventListener("change", () => window.location.reload());

  for (const s of states) render(s, 0);
  update();
})();
