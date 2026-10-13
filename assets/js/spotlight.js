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

// =========================================================================================
//  SETTINGS - everything worth tuning is here. Save the file and the dev server reloads it.
//  Opacities run from 0 (invisible / no effect) to 1 (fully opaque / full effect).
//  Related settings elsewhere: the dim marine snow itself is in marine-snow.js, and the
//  text glow colours and fade are in main.scss (search for "is-spotlit").
// =========================================================================================
const SPOTLIGHT_SETTINGS = {
  // --- Size and shape of the light ---------------------------------------------------------
  // The circle of light is sized to cover the width of the lit block, then scaled by this.
  CIRCLE_SIZE: .7, // 1 = covers the block; 0.7 = smaller pool; 1.3 = larger pool
  CIRCLE_MIN_RADIUS: 90, // px; small blocks (a heading) never get a smaller circle than this
  CIRCLE_MAX_RADIUS: 0.48, // largest radius, as a fraction of the window's shorter side
  // Width of the beam where it meets the circle, relative to the circle. 1 = the beam's
  // edges touch the circle's sides; 0.5 = a beam half as wide that still flows into it.
  BEAM_WIDTH: 1.0,
  LIGHT_PADDING: 14, // px of extra space around the lit block before sizing the circle
  EDGE_SOFTNESS: 28, // px of blur on the beam's edges; higher = softer, hazier outline

  // --- How strongly the page is dimmed and the light cuts through ---------------------------
  DIM_LIGHT_MODE: 0.42, // darkness outside the light in light mode
  DIM_DARK_MODE: 0.55, // darkness outside the light in dark mode
  // How much of the dimming the beam removes: at the cursor end and where it meets the
  // circle (1 = beam as bright as the page with no dimming at all).
  BEAM_CLEAR_AT_CURSOR: 0.4,
  BEAM_CLEAR_AT_CIRCLE: 0.6,
  // The circle is fully clear in its middle, then fades out smoothly. Both are fractions
  // of the circle's radius: fully lit out to CIRCLE_CORE, gone by CIRCLE_FALLOFF.
  CIRCLE_CORE: 0.4,
  CIRCLE_FALLOFF: 1.1,
  CURSOR_SPOT_RADIUS: 40, // px; small bright spot at the cursor, where the light comes from
  CURSOR_SPOT_STRENGTH: 0.8, // how much of the dimming the spot removes

  // --- Colour and opacity of the light itself (the cool glow added on top) -----------------
  LIGHT_COLOUR: "124, 145, 182", // r, g, b of the beam (the reference's steel blue)
  // Opacity of the beam's glow where it meets the circle; the cursor end gets
  // BEAM_GLOW_AT_CURSOR times this. Dark mode needs more to be visible.
  BEAM_GLOW_LIGHT_MODE: 0.22,
  BEAM_GLOW_DARK_MODE: 0.4,
  BEAM_GLOW_AT_CURSOR: 0.5,
  // Opacity of the glow in the middle of the circle, and how far it spreads (x radius).
  CIRCLE_GLOW_LIGHT_MODE: 0.16,
  CIRCLE_GLOW_DARK_MODE: 0.32,
  CIRCLE_GLOW_SIZE: 1.15,

  // --- Marine snow caught in the light -----------------------------------------------------
  // Extra particles drawn only inside the light, drifting with the regular snow.
  // Particles per 2560 x 2000 px field for the 1, 2 and 3 px layers; higher = denser.
  SNOW_COUNT: [2200, 800, 320],
  // Size: each particle is a bright core inside a soft (Gaussian) halo. Halo radius in px
  // = SNOW_HALO_BASE + particle size x SNOW_HALO_PER_SIZE.
  SNOW_HALO_BASE: 2,
  SNOW_HALO_PER_SIZE: 1.8,
  SNOW_CORE_SIZE: 0.5, // core diameter as a fraction of the particle size
  // Transparency: opacity of the halo at its centre, and extra opacity of the core.
  SNOW_HALO_OPACITY: 0.22,
  SNOW_CORE_OPACITY: 0.2,
  SNOW_COLOUR: "225, 236, 255", // r, g, b
  // How visible the snow is along the beam (cursor end, circle end) and in the circle.
  SNOW_IN_BEAM_AT_CURSOR: 0.45,
  SNOW_IN_BEAM_AT_CIRCLE: 0.8,
  SNOW_IN_CIRCLE: 1,
  SNOW_CIRCLE_CORE: 0.3, // fraction of the circle's radius where snow is at full strength

  // --- Photos ------------------------------------------------------------------------------
  // How much of the glow and lit snow stays over images, videos and embeds, so they keep
  // their own colours. 0 = none over photos; 1 = same as everywhere else.
  PHOTO_HALO_KEEP: 0.35,

  // --- Motion ------------------------------------------------------------------------------
  // Springs: seconds to (nearly) settle; lower = snappier. None of them bounce.
  MOVE_RESPONSE_S: 0.45, // the light moving from one block to the next
  CATCH_RESPONSE_S: 0.15, // the light catching up with the cursor when it re-enters the window
  RETURN_RESPONSE_S: 0.6, // the light gliding back to the top centre when the cursor leaves
  // px a new block must be closer to the middle of the window than the lit one before the
  // light moves to it; higher = the light stays put longer.
  SWITCH_MARGIN: 40,
  // The cone shrinks into the circle when the cursor is near the vertical middle of the
  // window: circle only within +/- CIRCLE_ONLY_BAND of the window height, full cone by
  // +/- (CIRCLE_ONLY_BAND + CONE_BLEND_BAND).
  CIRCLE_ONLY_BAND: 0.005,
  CONE_BLEND_BAND: 0.01,
  GLOW_FADE_MS: 600, // text glow fade; keep equal to the .is-spotlit transition in main.scss

  // --- Quality -----------------------------------------------------------------------------
  RESOLUTION: 0.5, // drawing resolution of the light (1 = full); lower = faster, softer
};
// =========================================================================================

(() => {
  const S = SPOTLIGHT_SETTINGS;
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

  const SCALE = S.RESOLUTION;
  const PAD = S.LIGHT_PADDING;
  // Critically damped springs (damping ratio 1) from a response time in seconds.
  const spring = (response) => {
    const k = (2 * Math.PI / response) ** 2;
    return { k, c: 2 * Math.sqrt(k) };
  };
  const MOVE = spring(S.MOVE_RESPONSE_S);
  const CATCH = spring(S.CATCH_RESPONSE_S);
  const RETURN = spring(S.RETURN_RESPONSE_S);

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
  // catching: the cursor came back; glide from the light's current position to it, then
  // follow 1:1, rather than snapping across the screen in one frame.
  const source = { x: 0, y: 0, vx: 0, vy: 0, tx: 0, ty: 0, mouse: false, catching: false };
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

  // On the home page the timeline already decides which entry is "at the centre" (the
  // open one); the light follows that decision so the two never disagree.
  const timelineActive = () => {
    const item = main.querySelector(".tl-js .tl-item.is-active");
    if (!item) return null;
    const r = item.getBoundingClientRect();
    return r.bottom > height * 0.2 && r.top < height * 0.8 ? item : null;
  };

  const pick = () => {
    if (!frameId) return; // effects off or hidden
    const open = timelineActive();
    if (open) {
      if (open !== target) retarget(open);
      return;
    }
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
      if (current.bottom > 0 && current.top < height && distance(current) - bestD < S.SWITCH_MARGIN) return;
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
      setTimeout(() => previous.classList.remove("was-spotlit"), S.GLOW_FADE_MS);
    }
    el.classList.remove("was-spotlit");
    el.classList.add("is-spotlit");
    target = el;
  };

  const stepSpring = (pos, vel, goal, { k, c }, dt) => {
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
    const cover = (Math.max(rect.w, rect.h) / 2) * S.CIRCLE_SIZE;
    const r = Math.min(cover, Math.min(width, height) * S.CIRCLE_MAX_RADIUS);
    return { x: rect.x + rect.w / 2, y: rect.y + rect.h / 2, r: Math.max(r, S.CIRCLE_MIN_RADIUS) };
  };

  // Cone weight: 0 in a narrow band around the vertical middle of the window (circle
  // only), 1 beyond it. Rather than fading the cone (which would detach it from the
  // circle), the cone's apex slides from the cursor into the circle's centre.
  const apexFor = (circle) => {
    const offMiddle = Math.abs(source.y - height / 2) / height;
    const w = smoothstep((offMiddle - S.CIRCLE_ONLY_BAND) / S.CONE_BLEND_BAND);
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

  // Gradient along the light: from `near` at the apex to `far` where it meets the circle.
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
  let litBounds = null;
  let photos = []; // on-screen image rects under the light, refreshed with each draw()

  // The part of an element actually on screen: clipped by every ancestor that hides its
  // overflow (e.g. a closed timeline entry, whose image still has its full size but sits
  // in a zero-height panel). Null when nothing shows.
  const visiblePart = (el, rect) => {
    let left = rect.left;
    let top = rect.top;
    let right = rect.right;
    let bottom = rect.bottom;
    for (let p = el.parentElement; p && p !== document.body; p = p.parentElement) {
      const style = getComputedStyle(p);
      if (style.display === "none" || style.visibility === "hidden") return null;
      if (style.overflowX === "visible" && style.overflowY === "visible") continue;
      const c = p.getBoundingClientRect();
      left = Math.max(left, c.left);
      top = Math.max(top, c.top);
      right = Math.min(right, c.right);
      bottom = Math.min(bottom, c.bottom);
      if (right <= left || bottom <= top) return null;
    }
    return { left, top, right, bottom, width: right - left, height: bottom - top };
  };

  // Visible images that the light reaches, with their corner radius (the round profile
  // photo stays round).
  const photoRects = () => {
    if (!litBounds) return [];
    const list = [];
    for (const el of main.querySelectorAll("img, video, iframe")) {
      const full = el.getBoundingClientRect();
      const r = visiblePart(el, full);
      if (!r || r.width < 8 || r.height < 8) continue;
      if (r.right < litBounds.left || r.left > litBounds.right || r.bottom < litBounds.top || r.top > litBounds.bottom)
        continue;
      const radius = getComputedStyle(el).borderTopLeftRadius;
      const rad = radius.endsWith("%")
        ? (parseFloat(radius) / 100) * Math.min(full.width, full.height)
        : parseFloat(radius) || 0;
      list.push({ x: r.left, y: r.top, w: r.width, h: r.height, r: Math.min(rad, r.width / 2, r.height / 2) });
    }
    return list;
  };

  // Take most of the halo back out over the images, with softened edges.
  const clearPhotos = (c, blurPx) => {
    if (!photos.length) return;
    c.save();
    c.globalCompositeOperation = "destination-out";
    c.filter = blurPx ? `blur(${blurPx}px)` : "none";
    c.fillStyle = `rgba(0, 0, 0, ${1 - S.PHOTO_HALO_KEEP})`;
    for (const ph of photos) {
      c.beginPath();
      c.roundRect(ph.x, ph.y, ph.w, ph.h, ph.r);
      c.fill();
    }
    c.restore();
  };
  let litLayers = null;

  // Bright core in a Gaussian halo, pre-rendered once per particle size.
  const sprite = (size) => {
    const R = Math.ceil(S.SNOW_HALO_BASE + size * S.SNOW_HALO_PER_SIZE);
    const c = document.createElement("canvas");
    c.width = c.height = R * 2;
    const g = c.getContext("2d");
    const grad = g.createRadialGradient(R, R, 0, R, R, R);
    for (let i = 0; i <= 12; i++) {
      const t = i / 12;
      // Gaussian halo (sigma = R/3) plus a tight core.
      const halo = Math.exp(-((t * 3) ** 2) / 2) * S.SNOW_HALO_OPACITY;
      const core = t < (size * S.SNOW_CORE_SIZE) / R ? S.SNOW_CORE_OPACITY : 0;
      grad.addColorStop(t, `rgba(${S.SNOW_COLOUR}, ${Math.min(1, halo + core).toFixed(3)})`);
    }
    g.fillStyle = grad;
    g.fillRect(0, 0, R * 2, R * 2);
    return { canvas: c, R };
  };

  const ensureLit = () => {
    if (litLayers || !window.marineSnow) return litLayers;
    const { tileWidth, tileHeight } = window.marineSnow;
    litLayers = S.SNOW_COUNT.map((count, index) => {
      const xs = new Float32Array(count);
      const ys = new Float32Array(count);
      for (let i = 0; i < count; i++) {
        xs[i] = Math.random() * tileWidth;
        ys[i] = Math.random() * tileHeight;
      }
      return { xs, ys, sprite: sprite(index + 1) };
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
    clearPhotos(sctx, 2);
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
    ctx.fillStyle = dark ? `rgba(5, 6, 15, ${S.DIM_DARK_MODE})` : `rgba(14, 9, 21, ${S.DIM_LIGHT_MODE})`;
    ctx.fillRect(0, 0, width, height);
    if (!drawn) {
      litBounds = null;
      return;
    }

    const circle = pool(drawn);
    const apex = apexFor(circle);
    // The beam's outline uses a circle scaled by BEAM_WIDTH; it still flows into the pool.
    const beam = { x: circle.x, y: circle.y, r: circle.r * S.BEAM_WIDTH };
    const blur = `blur(${S.EDGE_SOFTNESS * SCALE}px)`;

    // 2. Light = dimming removed (never added on top, which would grey the dimming):
    //    the cone-and-circle outline, then the circle's centre fully, smoothly.
    ctx.globalCompositeOperation = "destination-out";
    ctx.filter = blur;
    const d = hullPath(ctx, apex, beam);
    ctx.fillStyle = alongLight(ctx, apex, d, beam, S.BEAM_CLEAR_AT_CURSOR, S.BEAM_CLEAR_AT_CIRCLE, "0,0,0");
    ctx.fill();
    ctx.filter = "none";
    const fall = circle.r * S.CIRCLE_FALLOFF;
    ctx.fillStyle = smoothRadial(circle.x, circle.y, circle.r * S.CIRCLE_CORE, fall, 1);
    ctx.fillRect(circle.x - fall, circle.y - fall, fall * 2, fall * 2);
    if (source.mouse) {
      const sr = S.CURSOR_SPOT_RADIUS;
      ctx.fillStyle = smoothRadial(source.x, source.y, 0, sr, S.CURSOR_SPOT_STRENGTH);
      ctx.fillRect(source.x - sr, source.y - sr, sr * 2, sr * 2);
    }
    ctx.globalCompositeOperation = "source-over";

    // 3. The light itself, on the screen-blended layer (it can only brighten): the same
    //    outline as a beam, plus a glow in the circle. Stronger in dark mode.
    const beamPeak = dark ? S.BEAM_GLOW_DARK_MODE : S.BEAM_GLOW_LIGHT_MODE;
    const poolPeak = dark ? S.CIRCLE_GLOW_DARK_MODE : S.CIRCLE_GLOW_LIGHT_MODE;
    lctx.filter = blur;
    hullPath(lctx, apex, beam);
    lctx.fillStyle = alongLight(lctx, apex, d, beam, beamPeak * S.BEAM_GLOW_AT_CURSOR, beamPeak, S.LIGHT_COLOUR);
    lctx.fill();
    lctx.filter = "none";
    const glowR = circle.r * S.CIRCLE_GLOW_SIZE;
    const glow = lctx.createRadialGradient(circle.x, circle.y, 0, circle.x, circle.y, glowR);
    for (let i = 0; i <= 16; i++) {
      const t = i / 16;
      glow.addColorStop(t, `rgba(${S.LIGHT_COLOUR}, ${(poolPeak * (1 - smoothstep((t - 0.25) / 0.75))).toFixed(4)})`);
    }
    lctx.fillStyle = glow;
    lctx.fillRect(circle.x - glowR, circle.y - glowR, glowR * 2, glowR * 2);

    // 4. Mask for the lit snow: the light's shape, strongest in the circle.
    mctx.setTransform(SCALE, 0, 0, SCALE, 0, 0);
    mctx.clearRect(0, 0, width, height);
    mctx.filter = blur;
    hullPath(mctx, apex, beam);
    mctx.fillStyle = alongLight(mctx, apex, d, beam, S.SNOW_IN_BEAM_AT_CURSOR, S.SNOW_IN_BEAM_AT_CIRCLE, "255,255,255");
    mctx.fill();
    mctx.filter = "none";
    mctx.fillStyle = smoothRadial(circle.x, circle.y, circle.r * S.SNOW_CIRCLE_CORE, fall, S.SNOW_IN_CIRCLE, "255,255,255");
    mctx.fillRect(circle.x - fall, circle.y - fall, fall * 2, fall * 2);
    const reach = Math.max(fall, glowR) * 1.05;
    litBounds = {
      left: Math.min(apex.x, circle.x - reach) - 30,
      right: Math.max(apex.x, circle.x + reach) + 30,
      top: Math.min(apex.y, circle.y - reach) - 30,
      bottom: Math.max(apex.y, circle.y + reach) + 30,
    };

    // 5. Weaker halo over photos.
    photos = photoRects();
    clearPhotos(lctx, 4 * SCALE);
  };

  // Off while the effects are switched off (effects.js) or hidden by high contrast.
  const hiddenByPreference = window.matchMedia("(prefers-contrast: more)");
  const visible = () => (!window.siteEffects || window.siteEffects.enabled()) && !hiddenByPreference.matches;
  let frameId = 0;

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
          [offset[p], offset[v]] = stepSpring(offset[p], offset[v], 0, MOVE, dt);
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
    if (instant || (source.mouse && !source.catching)) {
      source.x = source.tx;
      source.y = source.ty;
      source.vx = 0;
      source.vy = 0;
    } else if (source.catching) {
      [source.x, source.vx] = stepSpring(source.x, source.vx, source.tx, CATCH, dt);
      [source.y, source.vy] = stepSpring(source.y, source.vy, source.ty, CATCH, dt);
      if (Math.hypot(source.x - source.tx, source.y - source.ty) < 1) source.catching = false;
    } else {
      [source.x, source.vx] = stepSpring(source.x, source.vx, source.tx, RETURN, dt);
      [source.y, source.vy] = stepSpring(source.y, source.vy, source.ty, RETURN, dt);
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
    frameId = requestAnimationFrame(frame);
  };

  const start = () => {
    cancelAnimationFrame(frameId);
    frameId = 0;
    if (!visible()) {
      // Nothing lit: drop the text glow and clear every layer.
      if (target) target.classList.remove("is-spotlit");
      target = null;
      drawn = null;
      litBounds = null;
      lastKey = "";
      for (const c of [canvas, lightCanvas, snowCanvas]) c.getContext("2d").clearRect(0, 0, c.width, c.height);
      return;
    }
    last = performance.now();
    frameId = requestAnimationFrame(frame);
    collect();
    pick();
  };

  // Re-pick the lit block on scroll and on layout changes.
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
      if (!source.mouse) source.catching = true;
      source.mouse = true;
      source.tx = event.clientX;
      source.ty = event.clientY;
    },
    { passive: true },
  );
  document.documentElement.addEventListener("mouseleave", () => {
    source.mouse = false;
    source.catching = false;
    source.tx = width / 2;
    source.ty = -60;
  });
  // Layout changes that move blocks without a scroll (timeline entries opening, images
  // loading) change the main column's size.
  new ResizeObserver(requestPick).observe(main);
  new MutationObserver(() => {
    collect();
    requestPick();
  }).observe(main, { childList: true, subtree: true });

  hiddenByPreference.addEventListener("change", start);
  if (window.siteEffects) window.siteEffects.onChange(start);

  resize();
  source.x = source.tx;
  source.y = source.ty;
  start();
})();
