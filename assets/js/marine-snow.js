// Marine snow overlay above the beams background: three layers of particles drifting upward
// at different speeds, with a spring-smoothed parallax that follows the mouse.
//
// Scroll couples into the drift: scrolling down adds to the upward motion in proportion to
// the scroll speed; scrolling up subtracts from it, and a fast enough upward scroll reverses
// it, so the reader sinks past the snow. Faster (nearer) layers couple more strongly. The
// scroll speed is smoothed by a critically damped spring, so the snow eases back to its
// base drift rather than stopping dead when scrolling stops. Scrolls made by the home
// timeline to keep its open entry still are not the reader's, and are ignored
// (window.siteEffects, assets/js/effects.js).
//
// Vanilla port of the 21st.dev "stars" React component, drawn on one viewport-sized canvas
// inside #marine-snow (_layouts/default.liquid); colours in assets/css/main.scss.
//
// Differences from the original:
// - One canvas instead of three 2560 x 4000 px box-shadow layers, so graphics memory is a
//   single screen's worth rather than ~100 MB.
// - Particles are placed only where they can be seen, at the original on-screen density,
//   and take the container's CSS `color`, so they follow the theme.
// - Parallax runs only for a mouse and not under prefers-reduced-motion, on a critically
//   damped spring like the rest of the site.
// - The loop stops while the effects are switched off or hidden.
(() => {
  const root = document.getElementById("marine-snow");
  if (!root) return;
  const canvas = document.createElement("canvas");
  canvas.setAttribute("aria-hidden", "true");
  root.appendChild(canvas);
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  const TILE_W = 2560; // particle field width; it repeats vertically only
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
  // Mouse parallax: offset per px of pointer distance from the centre, on a critically
  // damped spring with a 0.5 s response.
  const FACTOR = 0.05;
  const PARALLAX_STIFFNESS = (2 * Math.PI / 0.5) ** 2;
  const PARALLAX_DAMPING = 2 * Math.sqrt(PARALLAX_STIFFNESS);

  const layers = LAYERS.map(([count, size, speed, coupling]) => {
    const xs = new Float32Array(count);
    const ys = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      xs[i] = Math.random() * TILE_W;
      ys[i] = Math.random() * TILE_H;
    }
    return { xs, ys, size, speed, coupling, y: -Math.random() * TILE_H };
  });

  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  const finePointer = window.matchMedia("(hover: hover) and (pointer: fine)");
  const hiddenByPreference = window.matchMedia("(prefers-reduced-transparency: reduce), (prefers-contrast: more)");
  const effectsOn = () => !window.siteEffects || window.siteEffects.enabled();
  const visible = () => effectsOn() && !hiddenByPreference.matches;

  let width = 0;
  let height = 0;
  let dpr = 1;
  let colour = "#fff";

  const resize = () => {
    width = window.innerWidth;
    height = window.innerHeight;
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.ceil(width * dpr);
    canvas.height = Math.ceil(height * dpr);
  };

  const readColour = () => {
    colour = getComputedStyle(root).color;
  };

  const parallax = { x: 0, y: 0, vx: 0, vy: 0, tx: 0, ty: 0 };
  const wrap = (y) => ((y % TILE_H) - TILE_H) % TILE_H; // keep in (-TILE_H, 0]

  const draw = () => {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = colour;
    for (const layer of layers) {
      const r = layer.size / 2;
      for (let i = 0; i < layer.xs.length; i++) {
        const x = layer.xs[i] + parallax.x;
        if (x < -r || x > width + r) continue;
        const y = ((((layer.ys[i] + layer.y) % TILE_H) + TILE_H) % TILE_H) + parallax.y;
        if (y < -r || y > height + r) continue;
        if (layer.size === 1) {
          ctx.fillRect(x, y, 1, 1);
        } else {
          ctx.beginPath();
          ctx.arc(x, y, r, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }
  };

  const step = (value, velocity, goal, k, c, dt) => {
    let p = value;
    let v = velocity;
    for (let left = dt; left > 0; left -= 1 / 240) {
      const h = Math.min(1 / 240, left);
      v += (-k * (p - goal) - c * v) * h;
      p += v * h;
    }
    return [p, v];
  };

  let lastScrollY = window.scrollY;
  let scrollV = 0; // smoothed scroll velocity, px/s (positive = scrolling down)
  let scrollA = 0; // its rate of change, for the smoothing spring
  let last = 0;
  let frameId = 0;

  const frame = (now) => {
    const dt = Math.min((now - last) / 1000, 1 / 20);
    last = now;
    if (dt > 0) {
      // The reader's scroll only: drop what the timeline scrolled to hold its entry still.
      const programmatic = window.siteEffects ? window.siteEffects.takeCompensation() : 0;
      const measured = (window.scrollY - lastScrollY - programmatic) / dt;
      lastScrollY = window.scrollY;
      [scrollV, scrollA] = step(scrollV, scrollA, measured, SCROLL_STIFFNESS, SCROLL_DAMPING, dt);
      for (const l of layers) {
        // Upward is negative y: base drift plus a share of the scroll velocity.
        l.y = wrap(l.y - (l.speed + l.coupling * scrollV) * dt);
      }
      [parallax.x, parallax.vx] = step(parallax.x, parallax.vx, parallax.tx, PARALLAX_STIFFNESS, PARALLAX_DAMPING, dt);
      [parallax.y, parallax.vy] = step(parallax.y, parallax.vy, parallax.ty, PARALLAX_STIFFNESS, PARALLAX_DAMPING, dt);
      draw();
    }
    frameId = requestAnimationFrame(frame);
  };

  const start = () => {
    cancelAnimationFrame(frameId);
    frameId = 0;
    if (!visible()) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      return;
    }
    readColour();
    if (reducedMotion.matches) {
      draw(); // still particles
      return;
    }
    lastScrollY = window.scrollY;
    scrollV = 0;
    scrollA = 0;
    last = performance.now();
    frameId = requestAnimationFrame(frame);
  };

  window.addEventListener(
    "pointermove",
    (event) => {
      if (reducedMotion.matches || !finePointer.matches || event.pointerType !== "mouse") return;
      parallax.tx = -(event.clientX - window.innerWidth / 2) * FACTOR;
      parallax.ty = -(event.clientY - window.innerHeight / 2) * FACTOR;
    },
    { passive: true },
  );
  window.addEventListener("resize", () => {
    resize();
    if (reducedMotion.matches && visible()) draw();
  });
  // Particle colour follows the theme.
  new MutationObserver(() => {
    readColour();
    if (reducedMotion.matches && visible()) draw();
  }).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

  // Shared with spotlight.js, which draws extra lit particles under its beam that must
  // drift exactly with these layers.
  window.marineSnow = {
    tileWidth: TILE_W,
    tileHeight: TILE_H,
    offset: (i) => (layers[i] ? layers[i].y : 0),
    parallax: () => [parallax.x, parallax.y],
  };

  reducedMotion.addEventListener("change", start);
  hiddenByPreference.addEventListener("change", start);
  if (window.siteEffects) window.siteEffects.onChange(start);

  resize();
  start();
})();
