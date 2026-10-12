// Spotlight: dims the page and shines a soft beam from the cursor onto the content block
// nearest the middle of the viewport, so the reader's attention lands on it. The light is
// a circular pool on the block plus a cone from the cursor that ends in the pool's
// semicircle; with the cursor near the vertical middle of the window, only the pool shows.
// Light styling follows the 21st.dev "particle-hero" spotlight (blurred conic beams in
// rgba(124,145,182)). Canvas: #spotlight in _layouts/default.liquid; styles in main.scss.
//
// Motion: the beam's source tracks the mouse 1:1. When the lit block changes, the lit
// rectangle springs from where it was drawn to the new block (critically damped), while
// still following scroll exactly. Without a mouse, the light comes from the top centre.
(() => {
  const canvas = document.getElementById("spotlight");
  const main = document.querySelector('[role="main"]');
  if (!canvas || !main) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  // Second layer for the light itself, blended with `screen` (CSS) so it can only
  // brighten: on a dark page the background lifts while light text stays crisp.
  const lightCanvas = document.createElement("canvas");
  lightCanvas.className = "spotlight-light";
  lightCanvas.setAttribute("aria-hidden", "true");
  canvas.after(lightCanvas);
  const lctx = lightCanvas.getContext("2d");
  // Third layer: marine snow caught in the light. Extra particles (so the snow reads
  // denser there), each a bright core in a Gaussian halo, masked by the light's own
  // shape and brightness. Also screen-blended. Mask kept at the light's resolution.
  const snowCanvas = document.createElement("canvas");
  snowCanvas.className = "spotlight-snow";
  snowCanvas.setAttribute("aria-hidden", "true");
  lightCanvas.after(snowCanvas);
  const sctx = snowCanvas.getContext("2d");
  const maskCanvas = document.createElement("canvas");
  const mctx = maskCanvas.getContext("2d");

  const SCALE = 0.5; // internal resolution; every edge is blurred anyway
  const PAD = 14; // px of light around the lit block
  const GLOW_FADE_MS = 600; // matches the .is-spotlit text-shadow transition in main.scss
  const SWITCH_MARGIN = 40; // px a new block must be closer to the midline to take over
  const LIGHT = "124, 145, 182"; // reference beam colour
  const RESPONSE_S = 0.45; // lit-rectangle spring, damping ratio 1
  const STIFFNESS = (2 * Math.PI / RESPONSE_S) ** 2;
  const DAMPING = 2 * Math.sqrt(STIFFNESS);
  const SOURCE_RESPONSE_S = 0.6; // beam source returning to the top when the mouse leaves
  const SOURCE_STIFFNESS = (2 * Math.PI / SOURCE_RESPONSE_S) ** 2;
  const SOURCE_DAMPING = 2 * Math.sqrt(SOURCE_STIFFNESS);

  // Blocks that can be lit. Nested matches are dropped in favour of the outermost one,
  // so a timeline entry is lit as a whole rather than its paragraph.
  const CANDIDATES = [
    ".post-header",
    ".profile-intro-text > p",
    ".profile-intro-photo",
    ".tl-intro",
    ".tl-item",
    "ol.bibliography > li",
    ".post-list > li",
    ".card",
    "p",
    "h1",
    "h2",
    "h3",
    "h4",
    "figure",
    "pre",
    "blockquote",
    "table",
    "iframe",
    // Lists only directly in a post body: the timeline and bibliography are lists too,
    // and their entries should be lit one by one.
    "article > ul",
    "article > ol",
  ].join(", ");

  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

  let width = 0;
  let height = 0;
  let candidates = [];
  let target = null;
  // Drawn rectangle = target's live rect + offset; the offset springs to zero.
  const offset = { x: 0, y: 0, w: 0, h: 0, vx: 0, vy: 0, vw: 0, vh: 0 };
  let drawn = null;
  // Beam source.
  const source = { x: 0, y: 0, vx: 0, vy: 0, tx: 0, ty: 0, mouse: false };
  let last = 0;
  let lastKey = "";

  const resize = () => {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = lightCanvas.width = maskCanvas.width = Math.ceil(width * SCALE);
    canvas.height = lightCanvas.height = maskCanvas.height = Math.ceil(height * SCALE);
    snowCanvas.width = width;
    snowCanvas.height = height;
    if (!source.mouse) {
      source.tx = width / 2;
      source.ty = -60;
    }
  };

  const collect = () => {
    const all = Array.from(main.querySelectorAll(CANDIDATES));
    const set = new Set(all);
    candidates = all.filter((el) => {
      for (let p = el.parentElement; p && p !== main; p = p.parentElement) {
        if (set.has(p)) return false;
      }
      return true;
    });
  };

  // Distance from the viewport midline to the block: 0 when the midline crosses it.
  const distance = (rect) => {
    const mid = height / 2;
    if (rect.top <= mid && rect.bottom >= mid) return 0;
    return Math.min(Math.abs(rect.top - mid), Math.abs(rect.bottom - mid));
  };

  const pick = () => {
    let best = null;
    let bestD = Infinity;
    for (const el of candidates) {
      const r = el.getBoundingClientRect();
      if (r.height < 4 || r.width < 4 || r.bottom < 0 || r.top > height) continue;
      const d = distance(r);
      if (d < bestD) {
        best = el;
        bestD = d;
      }
    }
    if (!best || best === target) return;
    if (target) {
      const current = target.getBoundingClientRect();
      if (current.bottom > 0 && current.top < height && distance(current) - bestD < SWITCH_MARGIN) return;
    }
    retarget(best);
  };

  const retarget = (el) => {
    const r = el.getBoundingClientRect();
    if (drawn && !reducedMotion.matches) {
      // Start the new rectangle where the old one is drawn: no jump, velocity kept.
      offset.x = drawn.x - (r.left - PAD);
      offset.y = drawn.y - (r.top - PAD);
      offset.w = drawn.w - (r.width + PAD * 2);
      offset.h = drawn.h - (r.height + PAD * 2);
    } else {
      Object.assign(offset, { x: 0, y: 0, w: 0, h: 0, vx: 0, vy: 0, vw: 0, vh: 0 });
    }
    // Text in the lit block glows (CSS .is-spotlit). The previous block keeps a
    // transition-only class briefly so its glow fades out instead of snapping off.
    if (target) {
      const previous = target;
      previous.classList.remove("is-spotlit");
      previous.classList.add("was-spotlit");
      setTimeout(() => previous.classList.remove("was-spotlit"), GLOW_FADE_MS);
    }
    el.classList.remove("was-spotlit");
    el.classList.add("is-spotlit");
    target = el;
  };

  const stepSpring = (pos, vel, goal, k, c, dt) => {
    let p = pos;
    let v = vel;
    for (let left = dt; left > 0; left -= 1 / 240) {
      const h = Math.min(1 / 240, left);
      v += (-k * (p - goal) - c * v) * h;
      p += v * h;
    }
    return [p, v];
  };

  const smoothstep = (t) => {
    const x = Math.min(1, Math.max(0, t));
    return x * x * (3 - 2 * x);
  };

  // Radial gradient whose alpha follows a smoothstep curve from `inner` to `outer`
  // (many stops, so the falloff has no visible bands or edge).
  const smoothRadial = (cx, cy, r0, r1, peak, colour = "0,0,0") => {
    const g = ctx.createRadialGradient(cx, cy, 0, cx, cy, r1);
    const STOPS = 16;
    for (let i = 0; i <= STOPS; i++) {
      const t = i / STOPS;
      const r = t * r1;
      const a = r <= r0 ? peak : peak * (1 - smoothstep((r - r0) / (r1 - r0)));
      g.addColorStop(t, `rgba(${colour}, ${a.toFixed(4)})`);
    }
    return g;
  };

  // Pool of light: a circle centred on the lit block, sized to cover its width.
  const pool = (rect) => {
    const r = Math.min(Math.max(rect.w, rect.h) / 2, Math.min(width, height) * 0.48);
    return { x: rect.x + rect.w / 2, y: rect.y + rect.h / 2, r: Math.max(r, 90) };
  };

  // Cone weight: 0 in a narrow band around the vertical middle of the window (circle
  // only), 1 beyond it. Rather than fading the cone (which would detach it from the
  // circle), the cone's apex slides from the cursor into the circle's centre.
  const CIRCLE_BAND = 0.005; // circle only within ±0.5% of the window height (±4 px at 800)
  const BLEND_BAND = 0.01; // cone fully out to the cursor by ±1.5%
  const apexFor = (circle) => {
    const offMiddle = Math.abs(source.y - height / 2) / height;
    const w = smoothstep((offMiddle - CIRCLE_BAND) / BLEND_BAND);
    return { x: circle.x + (source.x - circle.x) * w, y: circle.y + (source.y - circle.y) * w };
  };

  // One outline for the whole light: the two tangent lines from the apex and the far arc
  // of the circle (the convex hull of apex and circle). Filled once, there is no seam
  // between cone and circle however close the apex is. Apex inside the circle: the circle.
  const hullPath = (c, apex, circle) => {
    const dx = circle.x - apex.x;
    const dy = circle.y - apex.y;
    const d = Math.hypot(dx, dy);
    c.beginPath();
    if (d <= circle.r + 0.5) {
      c.arc(circle.x, circle.y, circle.r, 0, Math.PI * 2);
      return d;
    }
    const theta = Math.atan2(dy, dx);
    const half = Math.asin(circle.r / d);
    const start = theta - half - Math.PI / 2; // tangent points, seen from the centre
    const end = theta + half + Math.PI / 2;
    c.moveTo(apex.x, apex.y);
    c.lineTo(circle.x + circle.r * Math.cos(start), circle.y + circle.r * Math.sin(start));
    c.arc(circle.x, circle.y, circle.r, start, end); // around the far side
    c.closePath();
    return d;
  };

  // Gradient along the light: dimmer at the apex, full from where it meets the circle.
  const alongLight = (c, apex, d, circle, near, far, colour) => {
    const reach = d + circle.r;
    const g = c.createRadialGradient(apex.x, apex.y, 0, apex.x, apex.y, Math.max(reach, 1));
    const meet = Math.max(0, Math.min(1, (d - circle.r) / reach));
    g.addColorStop(0, `rgba(${colour}, ${near.toFixed(3)})`);
    g.addColorStop(meet, `rgba(${colour}, ${far.toFixed(3)})`);
    g.addColorStop(1, `rgba(${colour}, ${far.toFixed(3)})`);
    return g;
  };

  // --- Lit marine snow ------------------------------------------------------------------
  // Same three layers as marine-snow.js (sizes 1/2/3 px), but denser, so more specks show
  // in the light than in the dim. Positions live in the shared 2560 x 2000 tile.
  const LIT_LAYERS = [
    [2200, 1],
    [800, 2],
    [320, 3],
  ];
  let litBounds = null;
  let litLayers = null;

  // Bright core in a Gaussian halo, pre-rendered once per s
  // Kept small and soft: the lit snow should read as texture, not compete with the text.
  const SPRITE_RADIUS = (size) => Math.ceil(2 + size * 1.8); // halo radius, px
  const HALO_ALPHA = 0.22;
  const CORE_ALPHA = 0.2;
  const sprite = (size) => {
    const R = SPRITE_RADIUS(size);
    const c = document.createElement("canvas");
    c.width = c.height = R * 2;
    const g = c.getContext("2d");
    const grad = g.createRadialGradient(R, R, 0, R, R, R);
    for (let i = 0; i <= 12; i++) {
      const t = i / 12;
      // Gaussian, sigma = R/3, plus a tight core (half the particle size).
      const halo = Math.exp(-((t * 3) ** 2) / 2) * HALO_ALPHA;
      const core = t < (size * 0.5) / R ? CORE_ALPHA : 0;
      grad.addColorStop(t, `rgba(225, 236, 255, ${Math.min(1, halo + core).toFixed(3)})`);
    }
    g.fillStyle = grad;
    g.fillRect(0, 0, R * 2, R * 2);
    return { canvas: c, R };
  };

  const ensureLit = () => {
    if (litLayers || !window.marineSnow) return litLayers;
    const { tileWidth, tileHeight } = window.marineSnow;
    litLayers = LIT_LAYERS.map(([count, size]) => {
      const xs = new Float32Array(count);
      const ys = new Float32Array(count);
      for (let i = 0; i < count; i++) {
        xs[i] = Math.random() * tileWidth;
        ys[i] = Math.random() * tileHeight;
      }
      return { xs, ys, sprite: sprite(size) };
    });
    return litLayers;
  };

  const drawSnow = () => {
    sctx.setTransform(1, 0, 0, 1, 0, 0);
    sctx.globalCompositeOperation = "source-over";
    sctx.clearRect(0, 0, width, height);
    const snow = window.marineSnow;
    if (!litBounds || !snow || !ensureLit()) return;
    const tileH = snow.tileHeight;
    const [px, py] = snow.parallax();
    const b = litBounds;
    litLayers.forEach((layer, index) => {
      const offset = snow.offset(index);
      const { canvas: img, R } = layer.sprite;
      for (let i = 0; i < layer.xs.length; i++) {
        const x = layer.xs[i] + px;
        if (x < b.left || x > b.right) continue;
        const y = ((((layer.ys[i] + offset) % tileH) + tileH) % tileH) + py;
        if (y < b.top || y > b.bottom) continue;
        sctx.drawImage(img, x - R, y - R);
      }
    });
    // Keep only what the light reaches, in proportion to how bright it is there.
    sctx.globalCompositeOperation = "destination-in";
    sctx.drawImage(maskCanvas, 0, 0, width, height);
    sctx.globalCompositeOperation = "source-over";
  };

  const draw = () => {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    ctx.setTransform(SCALE, 0, 0, SCALE, 0, 0);
    lctx.setTransform(SCALE, 0, 0, SCALE, 0, 0);
    for (const c of [ctx, lctx]) {
      c.globalCompositeOperation = "source-over";
      c.filter = "none";
      c.clearRect(0, 0, width, height);
    }

    // 1. Dim everything.
    ctx.fillStyle = dark ? "rgba(5, 6, 15, 0.55)" : "rgba(14, 9, 21, 0.42)";
    ctx.fillRect(0, 0, width, height);
    if (!drawn) {
      litBounds = null;
      return;
    }

    const circle = pool(drawn);
    const apex = apexFor(circle);
    const blur = `blur(${28 * SCALE}px)`;

    // 2. Light = dimming removed (never added on top, which would grey the dimming):
    //    the cone-and-circle outline at ~60%, then the circle's centre fully, smoothly.
    ctx.globalCompositeOperation = "destination-out";
    ctx.filter = blur;
    const d = hullPath(ctx, apex, circle);
    ctx.fillStyle = alongLight(ctx, apex, d, circle, 0.4, 0.6, "0,0,0");
    ctx.fill();
    ctx.filter = "none";
    ctx.fillStyle = smoothRadial(circle.x, circle.y, circle.r * 0.4, circle.r * 1.1, 1);
    ctx.fillRect(circle.x - circle.r * 1.1, circle.y - circle.r * 1.1, circle.r * 2.2, circle.r * 2.2);
    if (source.mouse) {
      ctx.fillStyle = smoothRadial(source.x, source.y, 0, 40, 0.8);
      ctx.fillRect(source.x - 40, source.y - 40, 80, 80);
    }
    ctx.globalCompositeOperation = "source-over";

    // 3. The light itself, on the screen-blended layer (it can only brighten): the same
    //    outline as a beam, plus a glow in the circle. Stronger in dark mode.
    const beamPeak = dark ? 0.4 : 0.22;
    const poolPeak = dark ? 0.32 : 0.16;
    lctx.filter = blur;
    hullPath(lctx, apex, circle);
    lctx.fillStyle = alongLight(lctx, apex, d, circle, beamPeak * 0.5, beamPeak, LIGHT);
    lctx.fill();
    lctx.filter = "none";
    const glow = lctx.createRadialGradient(circle.x, circle.y, 0, circle.x, circle.y, circle.r * 1.15);
    for (let i = 0; i <= 16; i++) {
      const t = i / 16;
      glow.addColorStop(t, `rgba(${LIGHT}, ${(poolPeak * (1 - smoothstep((t - 0.25) / 0.75))).toFixed(4)})`);
    }
    lctx.fillStyle = glow;
    lctx.fillRect(circle.x - circle.r * 1.15, circle.y - circle.r * 1.15, circle.r * 2.3, circle.r * 2.3);

    // 4. Mask for the lit snow: the light's shape at full strength in the circle.
    mctx.setTransform(SCALE, 0, 0, SCALE, 0, 0);
    mctx.clearRect(0, 0, width, height);
    mctx.filter = blur;
    hullPath(mctx, apex, circle);
    mctx.fillStyle = alongLight(mctx, apex, d, circle, 0.45, 0.8, "255,255,255");
    mctx.fill();
    mctx.filter = "none";
    mctx.fillStyle = smoothRadial(circle.x, circle.y, circle.r * 0.3, circle.r * 1.1, 1, "255,255,255");
    mctx.fillRect(circle.x - circle.r * 1.1, circle.y - circle.r * 1.1, circle.r * 2.2, circle.r * 2.2);
    const reach = circle.r * 1.2;
    litBounds = {
      left: Math.min(apex.x, circle.x - reach) - 30,
      right: Math.max(apex.x, circle.x + reach) + 30,
      top: Math.min(apex.y, circle.y - reach) - 30,
      bottom: Math.max(apex.y, circle.y + reach) + 30,
    };
  };

  const frame = (now) => {
    const dt = Math.min((now - last) / 1000, 1 / 20);
    last = now;
    const instant = reducedMotion.matches;

    // Lit rectangle: live rect of the target plus a decaying offset.
    if (target && target.isConnected) {
      const r = target.getBoundingClientRect();
      for (const [p, v] of [["x", "vx"], ["y", "vy"], ["w", "vw"], ["h", "vh"]]) {
        if (instant) {
          offset[p] = 0;
          offset[v] = 0;
        } else {
          [offset[p], offset[v]] = stepSpring(offset[p], offset[v], 0, STIFFNESS, DAMPING, dt);
        }
      }
      drawn = {
        x: r.left - PAD + offset.x,
        y: r.top - PAD + offset.y,
        w: Math.max(0, r.width + PAD * 2 + offset.w),
        h: Math.max(0, r.height + PAD * 2 + offset.h),
      };
    }

    // Source: 1:1 with the mouse; springs back to the top centre without one.
    if (source.mouse || instant) {
      source.x = source.tx;
      source.y = source.ty;
      source.vx = 0;
      source.vy = 0;
    } else {
      [source.x, source.vx] = stepSpring(source.x, source.vx, source.tx, SOURCE_STIFFNESS, SOURCE_DAMPING, dt);
      [source.y, source.vy] = stepSpring(source.y, source.vy, source.ty, SOURCE_STIFFNESS, SOURCE_DAMPING, dt);
    }

    // Redraw only when something visible changed.
    const key = drawn
      ? `${drawn.x.toFixed(1)},${drawn.y.toFixed(1)},${drawn.w.toFixed(1)},${drawn.h.toFixed(1)},${source.x.toFixed(1)},${source.y.toFixed(1)},${width},${height},${document.documentElement.getAttribute("data-theme")}`
      : `none,${width},${height}`;
    if (key !== lastKey) {
      lastKey = key;
      draw();
    }
    drawSnow(); // every frame: the snow keeps drifting even when the light is still
    requestAnimationFrame(frame);
  };

  // Re-pick the lit block on scroll, and periodically for layout changes (e.g. timeline
  // entries opening) that move blocks without a scroll.
  let picking = false;
  const requestPick = () => {
    if (!picking) {
      picking = true;
      requestAnimationFrame(() => {
        picking = false;
        pick();
      });
    }
  };

  window.addEventListener("scroll", requestPick, { passive: true });
  window.addEventListener("resize", () => {
    resize();
    collect();
    requestPick();
  });
  window.addEventListener(
    "pointermove",
    (event) => {
      if (event.pointerType !== "mouse") return;
      source.mouse = true;
      source.tx = event.clientX;
      source.ty = event.clientY;
    },
    { passive: true },
  );
  document.documentElement.addEventListener("mouseleave", () => {
    source.mouse = false;
    source.tx = width / 2;
    source.ty = -60;
  });
  setInterval(requestPick, 300);
  new MutationObserver(() => {
    collect();
    requestPick();
  }).observe(main, { childList: true, subtree: true });

  resize();
  source.x = source.tx;
  source.y = source.ty;
  collect();
  pick();
  last = performance.now();
  requestAnimationFrame(frame);
})();
