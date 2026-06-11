---
layout: post
published: false # draft — excluded from the built site
title: "ICRA 2026: a robot-platform map"
date: 2026-06-10 10:00:00 +0200
description: An interactive map of all 3,028 ICRA 2026 papers organized by the kind of robot — aerial, ground, marine, humanoid, manipulator, and more — that each work is tested or deployed on. Node size is the number of papers; edges link platforms that appear together. Search by title or author, click a platform to list its papers, click a title to read the abstract.
tags: robotics ICRA visualization
categories: robotics
---

A companion to the [topic map]({% post_url 2026-06-10-icra-2026-topic-map %}):
the same **3,028** ICRA 2026 papers, but organized by the **kind of robot /
environment** each work is actually tested or deployed on — aerial (UAVs),
ground (wheeled, autonomous vehicles, legged), marine/underwater (AUVs, ROVs),
humanoids, manipulator arms and hands, soft/medical/wearable robots, and so on.
Papers with no physical robot (pure simulation, datasets, theory) land in
**Simulation**.

Each node is a platform, sized by how many papers use it; an edge means two
platforms appear together in the same paper (e.g. aerial + ground for
heterogeneous teams, or arm + hand for dexterous manipulation).

**How to use it:** search by **title or author** — the graph lights up the
platforms those papers use and the panel lists every match. Click a highlighted
platform to **refine** to it. With the box empty, click any platform to browse
its papers, and click a title to read the abstract.

<link href="https://fonts.googleapis.com/css2?family=Source+Serif+4:ital,opsz,wght@0,8..60,400;0,8..60,600;1,8..60,400&display=swap" rel="stylesheet" />

<style>
  /* Light theme (default) */
  .tm-app {
    --tm-bg:#f4f6fc; --tm-surface:#ffffff; --tm-surface-2:#eceff7;
    --tm-border:#d7ddec; --tm-border-soft:#e7ebf5;
    --tm-text:#16203a; --tm-muted:#5a637f; --tm-faint:#9aa3bd;
    --tm-accent:#bd3fd8; --tm-accent-dim:#e6cdee; --tm-radius:12px;
    --tm-grad-a:rgba(189,63,216,.10); --tm-grad-b:rgba(88,196,221,.12); --tm-fade:rgba(244,246,252,0);
    --tm-shadow:0 14px 44px rgba(40,30,60,.14);
    background:
      radial-gradient(1000px 480px at 74% -14%, var(--tm-grad-a) 0%, var(--tm-fade) 60%),
      radial-gradient(820px 460px at 6% 112%, var(--tm-grad-b) 0%, var(--tm-fade) 55%),
      var(--tm-bg);
    color:var(--tm-text); border:1px solid var(--tm-border); border-radius:16px;
    padding:1.5rem clamp(1rem,3vw,1.9rem) 1.7rem; margin:1.5rem 0;
    box-shadow:var(--tm-shadow);
    font-family:"Source Serif 4",Charter,Georgia,Cambria,"Times New Roman",serif; letter-spacing:.1px;
  }
  /* Dark theme */
  html[data-theme="dark"] .tm-app {
    --tm-bg:#0a0c16; --tm-surface:#11152a; --tm-surface-2:#171c34;
    --tm-border:#222a45; --tm-border-soft:#1a2038;
    --tm-text:#e8eefb; --tm-muted:#8b95b5; --tm-faint:#5a637f;
    --tm-accent:#58C4DD; --tm-accent-dim:#234a5a;
    --tm-grad-a:#16213f; --tm-grad-b:#122033; --tm-fade:rgba(10,12,22,0);
    --tm-shadow:0 14px 44px rgba(0,0,0,.5);
  }
  .tm-app * { box-sizing:border-box; }
  .tm-app .tm-eyebrow { font-size:.72rem; font-weight:600; letter-spacing:.2em; text-transform:uppercase; color:var(--tm-accent); }
  .tm-app h2.tm-h { margin:.3rem 0 .4rem; font-family:"Source Serif 4",Charter,Georgia,serif; font-size:1.7rem; font-weight:600; color:var(--tm-text); border:0; padding:0; }
  .tm-app .tm-sub { margin:0; color:var(--tm-muted); font-size:.92rem; max-width:72ch; line-height:1.5; }

  .tm-app #tm-controls { display:flex; gap:.6rem; align-items:center; margin:1.1rem 0 .6rem; flex-wrap:wrap; }
  .tm-app #tm-search { flex:1; min-width:200px; padding:.55rem .9rem; font-size:.92rem; color:var(--tm-text); background:var(--tm-surface); border:1px solid var(--tm-border); border-radius:10px; outline:none; transition:border-color .15s, box-shadow .15s; }
  .tm-app #tm-search::placeholder { color:var(--tm-faint); }
  .tm-app #tm-search:focus { border-color:var(--tm-accent-dim); box-shadow:0 0 0 3px rgba(123,183,255,.12); }
  .tm-app #tm-reset { padding:.55rem 1rem; font-size:.9rem; font-weight:500; color:var(--tm-text); cursor:pointer; background:var(--tm-surface); border:1px solid var(--tm-border); border-radius:10px; transition:background .15s, border-color .15s; }
  .tm-app #tm-reset:hover { background:var(--tm-surface-2); border-color:var(--tm-accent-dim); }

  .tm-app #tm-legend { display:flex; flex-wrap:wrap; gap:.5rem .9rem; margin:.2rem 0 1rem; font-size:.78rem; color:var(--tm-muted); }
  .tm-app #tm-legend .tm-leg { display:inline-flex; align-items:center; gap:.4rem; }
  .tm-app #tm-legend i { width:11px; height:11px; border-radius:3px; display:inline-block; box-shadow:0 0 0 1px rgba(0,0,0,.4); }

  .tm-app #tm-wrap { display:flex; gap:1rem; align-items:stretch; flex-wrap:wrap; }
  .tm-app #tm-graph { flex:2 1 520px; height:min(70vh,700px); min-width:320px; background:radial-gradient(900px 480px at 30% 0%, #0f141d 0%, #0a0d12 70%), var(--tm-bg); border:1px solid var(--tm-border); border-radius:var(--tm-radius); }
  .tm-app #tm-panel { flex:1 1 320px; max-height:min(70vh,700px); overflow:auto; padding:.25rem .35rem .5rem; background:var(--tm-surface); border:1px solid var(--tm-border); border-radius:var(--tm-radius); }
  .tm-app #tm-panel::-webkit-scrollbar { width:10px; }
  .tm-app #tm-panel::-webkit-scrollbar-thumb { background:#2a3342; border-radius:8px; border:2px solid var(--tm-surface); }

  .tm-app .tm-topic-head { position:sticky; top:0; background:linear-gradient(var(--tm-surface),rgba(17,22,31,.85)); backdrop-filter:blur(4px); border-left:4px solid #888; padding:.6rem .8rem; margin:0 0 .4rem; z-index:1; }
  .tm-app .tm-topic-head h3 { margin:0 0 .15rem; font-family:"Source Serif 4",Charter,Georgia,serif; font-size:1.12rem; font-weight:600; color:var(--tm-text); }
  .tm-app .tm-topic-head .tm-meta { font-size:.74rem; letter-spacing:.04em; text-transform:uppercase; color:var(--tm-muted); }
  .tm-app ol.tm-papers { list-style:none; padding:0 .5rem; margin:0; }
  .tm-app li.tm-paper { padding:.55rem .2rem; border-bottom:1px solid var(--tm-border-soft); }
  .tm-app li.tm-paper:last-child { border-bottom:0; }
  .tm-app .tm-badge { display:inline-flex; align-items:center; justify-content:center; width:1.35em; height:1.35em; font-size:.62rem; font-weight:700; border-radius:5px; margin-right:.5rem; color:#0b0e14; vertical-align:middle; }
  .tm-app .tm-badge.tm-P { background:var(--tm-accent); } .tm-app .tm-badge.tm-W { background:#FC6255; }
  .tm-app .tm-title { cursor:pointer; font-size:.9rem; line-height:1.4; color:var(--tm-text); transition:color .12s; }
  .tm-app li.tm-paper:hover .tm-title { color:var(--tm-accent); }
  .tm-app .tm-abstract { font-size:.84rem; line-height:1.55; margin:.5rem 0 .2rem 1.85em; padding:.6rem .8rem; color:#cdd6e2; background:var(--tm-surface-2); border:1px solid var(--tm-border-soft); border-radius:9px; }
  .tm-app .tm-abstract .tm-authors { font-size:.8rem; font-style:italic; color:var(--tm-muted); margin-bottom:.4rem; }
  .tm-app .tm-abstract .tm-links { margin-top:.5rem; font-size:.8rem; }
  .tm-app .tm-abstract a { color:var(--tm-accent); text-decoration:none; }
  .tm-app .tm-abstract a:hover { text-decoration:underline; }
  .tm-app .tm-hint { color:var(--tm-muted); font-size:.9rem; line-height:1.5; padding:1.2rem 1rem; }
  .tm-app .tm-error { color:#ff7b86; font-size:.9rem; padding:1rem; }
</style>

<div class="tm-app">
  <div class="tm-eyebrow">ICRA 2026 · Proceedings &amp; Workshops</div>
  <h2 class="tm-h">Robot-Platform Map</h2>
  <p class="tm-sub">3,028 papers organized by the robot platform each work is tested or deployed on. Search by title or author, or click a platform to list its papers.</p>
  <div id="tm-controls">
    <input id="tm-search" type="search" placeholder="Search papers — title or author…" />
    <button id="tm-reset" type="button">Reset</button>
  </div>
  <div id="tm-legend"></div>
  <div id="tm-wrap">
    <div id="tm-graph"></div>
    <div id="tm-panel"></div>
  </div>
</div>

<script src="https://unpkg.com/layout-base@2.0.1/layout-base.js"></script>
<script src="https://unpkg.com/cose-base@2.2.0/cose-base.js"></script>
<script src="https://unpkg.com/cytoscape@3.30.2/dist/cytoscape.min.js"></script>
<script src="https://unpkg.com/cytoscape-fcose@2.2.0/cytoscape-fcose.js"></script>
<script>
  window.TOPIC_DATA_BASE = "{{ '/assets/json/' | relative_url }}";
  window.TOPIC_GRAPH_FILE = "platform_graph.json";
</script>
<script src="{{ '/assets/js/topic-graph.js' | relative_url }}"></script>
