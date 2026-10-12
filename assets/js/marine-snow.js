// Marine snow overlay above the beams background: three layers of particles drifting upward
// at different speeds, with a spring-smoothed parallax that follows the mouse.
//
// Scroll couples into the drift: scrolling down adds to the upward motion in proportion to
// the scroll speed; scrolling up subtracts from it, and a fast enough upward scroll reverses
// it, so the reader sinks past the snow. Faster (nearer) layers couple more strongly. The
// scroll speed is smoothed by a critically damped spring, so the snow eases back to its
// base drift rather than stopping dead when scrolling stops.
// Vanilla port of the 21st.dev "stars" React component. Container: #marine-snow in
// _layouts/default.liquid; drift keyframes and colours in assets/css/main.scss.
//
// Differences from the original:
// - Particles are placed only where they can be seen (the original put half of them at
//   negative x, off-screen), at the same on-screen density.
// - Box-shadows carry no colour, so they take the layer's CSS `color` and follow the theme.
// - Parallax runs only for hover-capable pointers and not under prefers-reduced-motion,
//   and its spring loop stops once settled instead of running every frame.
// - Drift is driven here rather than by CSS keyframes, so scroll can speed it up or
//   reverse it.
(() => {
  const root = document.getElementById("marine-snow");
  if (!root) return;

  const TILE_W = 2560; // wide enough for large monitors; the tile repeats vertically only
  const TILE_H = 2000;
  // [count, size px, base drift px/s, scroll coupling]: the original's 1000/400/200
  // particles over a 4000 px square, scaled to the visible tile; drift speeds match its
  // 50/100/150 s loops over 2000 px. Coupling: px of extra drift per px scrolled.
  const LAYERS = [
    [640, 1, 40, 0.6],
    [256, 2, 20, 0.4],
    [128, 3, 13.3, 0.25],
  ];
  // Scroll-velocity smoothing spring: damping ratio 1, response 0.35 s.
  const SCROLL_STIFFNESS = (2 * Math.PI / 0.35) ** 2;
  const SCROLL_DAMPING = 2 * Math.sqrt(SCROLL_STIFFNESS);
  const FACTOR = 0.05; // parallax offset per px of pointer distance from the centre
  const STIFFNESS = 50; // spring, as in the original (mass 1)
  const DAMPING = 20;

  const shadows = (count) => {
    const list = [];
    for (let i = 0; i < count; i++) {
      list.push(`${Math.floor(Math.random() * TILE_W)}px ${Math.floor(Math.random() * TILE_H)}px`);
    }
    return list.join(", ");
  };

  const parallax = document.createElement("div");
  parallax.className = "snow-parallax";
  const layers = [];
  for (const [count, size, speed, coupling] of LAYERS) {
    const layer = document.createElement("div");
    layer.className = "snow-layer";
    layers.push({ el: layer, speed, coupling, y: -Math.random() * TILE_H });
    const boxShadow = shadows(count);
    // Two copies, one tile apart, so the drift loops seamlessly in either direction.
    for (const top of [0, TILE_H]) {
      const dots = document.createElement("div");
      dots.className = "snow-dots";
      dots.style.cssText = `top:${top}px;width:${size}px;height:${size}px;box-shadow:${boxShadow}`;
      layer.appendChild(dots);
    }
    parallax.appendChild(layer);
  }
  root.appendChild(parallax);

  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  const finePointer = window.matchMedia("(hover: hover) and (pointer: fine)");

  const spring = { x: 0, y: 0, vx: 0, vy: 0, tx: 0, ty: 0 };
  let running = false;
  let last = 0;

  const frame = (now) => {
    const dt = Math.min((now - last) / 1000, 1 / 30);
    last = now;
    for (const [p, v, t] of [["x", "vx", "tx"], ["y", "vy", "ty"]]) {
      spring[v] += (-STIFFNESS * (spring[p] - spring[t]) - DAMPING * spring[v]) * dt;
      spring[p] += spring[v] * dt;
    }
    parallax.style.transform = `translate3d(${spring.x.toFixed(2)}px, ${spring.y.toFixed(2)}px, 0)`;
    const settled =
      Math.abs(spring.x - spring.tx) < 0.05 &&
      Math.abs(spring.y - spring.ty) < 0.05 &&
      Math.abs(spring.vx) < 0.05 &&
      Math.abs(spring.vy) < 0.05;
    if (settled) {
      running = false;
    } else {
      requestAnimationFrame(frame);
    }
  };

  window.addEventListener(
    "pointermove",
    (event) => {
      if (reducedMotion.matches || !finePointer.matches || event.pointerType !== "mouse") return;
      spring.tx = -(event.clientX - window.innerWidth / 2) * FACTOR;
      spring.ty = -(event.clientY - window.innerHeight / 2) * FACTOR;
      if (!running) {
        running = true;
        last = performance.now();
        requestAnimationFrame(frame);
      }
    },
    { passive: true },
  );

  // --- Drift, coupled to scroll ----------------------------------------------------------
  const wrap = (y) => ((y % TILE_H) - TILE_H) % TILE_H; // keep in (-TILE_H, 0]
  const place = () => {
    for (const l of layers) l.el.style.transform = `translate3d(0, ${l.y.toFixed(2)}px, 0)`;
  };

  let lastScrollY = window.scrollY;
  let scrollV = 0; // smoothed scroll velocity, px/s (positive = scrolling down)
  let scrollA = 0; // its rate of change, for the smoothing spring
  let driftLast = 0;
  let driftId = 0;

  const drift = (now) => {
    const dt = Math.min((now - driftLast) / 1000, 1 / 20);
    driftLast = now;
    if (dt > 0) {
      const measured = (window.scrollY - lastScrollY) / dt;
      lastScrollY = window.scrollY;
      // Spring the smoothed velocity towards the measured one (sub-stepped for stability).
      for (let left = dt; left > 0; left -= 1 / 240) {
        const h = Math.min(1 / 240, left);
        scrollA += (-SCROLL_STIFFNESS * (scrollV - measured) - SCROLL_DAMPING * scrollA) * h;
        scrollV += scrollA * h;
      }
      for (const l of layers) {
        // Upward is negative y: base drift plus a share of the scroll velocity.
        l.y = wrap(l.y - (l.speed + l.coupling * scrollV) * dt);
      }
      place();
    }
    driftId = requestAnimationFrame(drift);
  };

  const startDrift = () => {
    cancelAnimationFrame(driftId);
    place();
    if (reducedMotion.matches) return; // still particles under reduced motion
    lastScrollY = window.scrollY;
    scrollV = 0;
    scrollA = 0;
    driftLast = performance.now();
    driftId = requestAnimationFrame(drift);
  };

  // Shared with spotlight.js, which draws extra lit particles under its beam that must
  // drift exactly with these layers.
  window.marineSnow = {
    tileWidth: TILE_W,
    tileHeight: TILE_H,
    offset: (i) => (layers[i] ? layers[i].y : 0),
    parallax: () => [spring.x, spring.y],
  };

  reducedMotion.addEventListener("change", startDrift);
  startDrift();
})();
