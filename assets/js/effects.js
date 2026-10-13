// Shared state for the decorative effects (beams background, marine snow, spotlight).
//
// - A navbar toggle turns them on or off; the choice is remembered per browser. While off,
//   <html data-effects="off"> hides the effect canvases (main.scss) and each effect stops
//   its animation loop.
// - Scroll compensation: the home timeline scrolls the page itself to keep the open entry
//   still while panels resize. It reports those scrolls here so marine snow can ignore
//   them instead of reading them as the reader scrolling.
//
// Loaded before the effect scripts in _layouts/default.liquid.
(() => {
  const STORAGE_KEY = "site-effects";
  const root = document.documentElement;
  const listeners = new Set();

  const read = () => {
    try {
      return window.localStorage.getItem(STORAGE_KEY) !== "off";
    } catch {
      return true;
    }
  };
  const write = (on) => {
    try {
      window.localStorage.setItem(STORAGE_KEY, on ? "on" : "off");
    } catch {
      // Storage unavailable (private mode, blocked): the choice lasts for this page only.
    }
  };

  let enabled = read();
  let compensation = 0;

  // Written out as "data-effects" (not dataset.effects) so the deploy's PurgeCSS step
  // sees the attribute and keeps the html[data-effects="off"] rules.
  const apply = () => {
    if (enabled) {
      root.removeAttribute("data-effects");
    } else {
      root.setAttribute("data-effects", "off");
    }
  };
  apply();

  window.siteEffects = {
    enabled: () => enabled,
    onChange: (fn) => listeners.add(fn),
    // Called by the timeline with each programmatic scroll, in px.
    compensateScroll: (px) => {
      compensation += px;
    },
    // Called by marine snow once per frame: the programmatic share of this frame's scroll.
    takeCompensation: () => {
      const px = compensation;
      compensation = 0;
      return px;
    },
  };

  // --- Navbar toggle, next to the theme switch ---------------------------------------------
  const themeItem = document.querySelector(".navbar .toggle-container");
  if (!themeItem) return;

  const item = document.createElement("li");
  item.className = "toggle-container effects-toggle-container";
  const button = document.createElement("button");
  button.id = "effects-toggle";
  button.type = "button";
  button.innerHTML = '<i class="fa-solid fa-lightbulb" aria-hidden="true"></i>';
  item.appendChild(button);
  themeItem.before(item);

  const label = () => {
    const text = enabled ? "Turn off light and background effects" : "Turn on light and background effects";
    button.title = text;
    button.setAttribute("aria-label", text);
    button.setAttribute("aria-pressed", String(enabled));
  };
  label();

  button.addEventListener("click", () => {
    enabled = !enabled;
    write(enabled);
    apply();
    label();
    listeners.forEach((fn) => fn(enabled));
  });
})();
