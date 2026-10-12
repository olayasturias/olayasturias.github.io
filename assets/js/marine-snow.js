// Marine snow overlay above the beams background: three layers of particles drifting at
// different speeds, with a spring-smoothed parallax that follows the mouse.
// Vanilla port of the 21st.dev "stars" React component. Container: #marine-snow in
// _layouts/default.liquid; drift keyframes and colours in assets/css/main.scss.
//
// Differences from the original:
// - Particles are placed only where they can be seen (the original put half of them at
//   negative x, off-screen), at the same on-screen density.
// - Box-shadows carry no colour, so they take the layer's CSS `color` and follow the theme.
// - Parallax runs only for hover-capable pointers and not under prefers-reduced-motion,
//   and its spring loop stops once settled instead of running every frame.
(() => {
  const root = document.getElementById("marine-snow");
  if (!root) return;

  const TILE_W = 2560; // wide enough for large monitors; the tile repeats vertically only
  const TILE_H = 2000;
  // [count, size px, drift duration multiplier]: the original's 1000/400/200 particles over a
  // 4000 px square, scaled to the visible tile.
  const LAYERS = [
    [640, 1, 1],
    [256, 2, 2],
    [128, 3, 3],
  ];
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
  for (const [count, size, multiplier] of LAYERS) {
    const layer = document.createElement("div");
    layer.className = "snow-layer";
    layer.style.setProperty("--snow-duration-scale", multiplier);
    const boxShadow = shadows(count);
    // Two copies, one tile apart, so the upward drift loops seamlessly.
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
})();
