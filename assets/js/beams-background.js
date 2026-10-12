// Animated beams background: soft, blurred light beams drifting upward behind the page.
// Vanilla port of the 21st.dev "beams-background" React component. Canvas: #beams-background
// in _layouts/default.liquid; styles in assets/css/main.scss.
//
// Differences from the original, for a background that runs on every page:
// - Drawn at quarter resolution and scaled up by the browser; the beams are blurred anyway,
//   so this looks the same at a fraction of the cost (no full-screen CSS/backdrop blurs).
// - Beam motion is time-based, so it runs at the same speed on 60 Hz and 120 Hz displays.
// - The overlay's 10 s opacity loop is dropped (slow full-screen oscillation is a
//   vestibular trigger); prefers-reduced-motion gets a single still frame.
// - Hues follow the site palette (blue to purple) and adapt to light/dark mode.
(() => {
  const canvas = document.getElementById("beams-background");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  const SCALE = 0.25; // internal resolution relative to the viewport
  const BEAM_COUNT = 30;
  const HUE_MIN = 205; // palette blue
  const HUE_RANGE = 80; // ...to purple
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

  let width = 0; // CSS pixels
  let height = 0;
  let beams = [];
  let frameId = 0;
  let last = 0;

  const isDark = () => document.documentElement.getAttribute("data-theme") === "dark";

  const createBeam = () => ({
    x: Math.random() * width * 1.5 - width * 0.25,
    y: Math.random() * height * 1.5 - height * 0.25,
    width: 30 + Math.random() * 60,
    length: height * 2.5,
    angle: -35 + Math.random() * 10,
    speed: 0.6 + Math.random() * 1.2, // px per 60 Hz frame
    opacity: 0.12 + Math.random() * 0.16,
    hue: HUE_MIN + Math.random() * HUE_RANGE,
    pulse: Math.random() * Math.PI * 2,
    pulseSpeed: 0.02 + Math.random() * 0.03,
  });

  // Recycle a beam below the viewport, spread over three columns.
  const resetBeam = (beam, index) => {
    const spacing = width / 3;
    beam.y = height + 100;
    beam.x = (index % 3) * spacing + spacing / 2 + (Math.random() - 0.5) * spacing * 0.5;
    beam.width = 100 + Math.random() * 100;
    beam.speed = 0.5 + Math.random() * 0.4;
    beam.hue = HUE_MIN + (index * HUE_RANGE) / beams.length;
    beam.opacity = 0.2 + Math.random() * 0.1;
  };

  const resize = () => {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = Math.ceil(width * SCALE);
    canvas.height = Math.ceil(height * SCALE);
    ctx.setTransform(SCALE, 0, 0, SCALE, 0, 0); // draw in CSS pixels
    beams = Array.from({ length: BEAM_COUNT }, createBeam);
  };

  const drawBeam = (beam, dark) => {
    const alpha = beam.opacity * (0.8 + Math.sin(beam.pulse) * 0.2);
    const colour = dark ? `${beam.hue}, 85%, 65%` : `${beam.hue}, 75%, 60%`;
    const gradient = ctx.createLinearGradient(0, 0, 0, beam.length);
    gradient.addColorStop(0, `hsla(${colour}, 0)`);
    gradient.addColorStop(0.1, `hsla(${colour}, ${alpha * 0.5})`);
    gradient.addColorStop(0.4, `hsla(${colour}, ${alpha})`);
    gradient.addColorStop(0.6, `hsla(${colour}, ${alpha})`);
    gradient.addColorStop(0.9, `hsla(${colour}, ${alpha * 0.5})`);
    gradient.addColorStop(1, `hsla(${colour}, 0)`);

    ctx.save();
    ctx.translate(beam.x, beam.y);
    ctx.rotate((beam.angle * Math.PI) / 180);
    ctx.fillStyle = gradient;
    ctx.fillRect(-beam.width / 2, 0, beam.width, beam.length);
    ctx.restore();
  };

  const draw = () => {
    const dark = isDark();
    ctx.clearRect(0, 0, width, height);
    // Blur radius in canvas pixels: ~35 px at full resolution.
    ctx.filter = `blur(${35 * SCALE}px)`;
    beams.forEach((beam) => drawBeam(beam, dark));
    ctx.filter = "none";
  };

  const step = (now) => {
    const frames = Math.min((now - last) / (1000 / 60), 4); // in 60 Hz frames, capped after tab switches
    last = now;
    beams.forEach((beam, index) => {
      beam.y -= beam.speed * frames;
      beam.pulse += beam.pulseSpeed * frames;
      if (beam.y + beam.length < -100) resetBeam(beam, index);
    });
    draw();
    frameId = requestAnimationFrame(step);
  };

  const start = () => {
    cancelAnimationFrame(frameId);
    if (reducedMotion.matches) {
      draw();
      return;
    }
    last = performance.now();
    frameId = requestAnimationFrame(step);
  };

  resize();
  start();

  window.addEventListener("resize", () => {
    resize();
    if (reducedMotion.matches) draw();
  });
  reducedMotion.addEventListener("change", start);
  // Redraw a still frame immediately when the theme toggles under reduced motion.
  new MutationObserver(() => reducedMotion.matches && draw()).observe(document.documentElement, {
    attributes: true,
    attributeFilter: ["data-theme"],
  });
})();
