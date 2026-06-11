/* ICRA 2026 Topic Map — interactive topic graph.
 *
 * Self-contained Cytoscape.js widget. Expects, already loaded on the page:
 *   - cytoscape            (https://unpkg.com/cytoscape)
 *   - cytoscapeFcose       (https://unpkg.com/cytoscape-fcose) + its dep layout-base/cose-base
 * Data is fetched from window.TOPIC_DATA_BASE (default "./"):
 *   - topic_graph.json   {nodes:[{id,label,group,count,paperIds}], edges:[{source,target,weight}], meta}
 *   - topic_papers.json  {id:{title,abstract,authors,source,workshop,code,url,primary,topics}}
 *
 * Markup it expects on the page:
 *   <div id="tm-graph"></div>           graph canvas
 *   <div id="tm-panel"></div>           side panel (titles / abstracts)
 *   <input id="tm-search">              optional filter box
 *   <button id="tm-reset">              optional reset button
 */
(function () {
  "use strict";

  var BASE = (window.TOPIC_DATA_BASE || "./").replace(/\/?$/, "/");
  var GRAPH_FILE = window.TOPIC_GRAPH_FILE || "topic_graph.json";
  var PAPERS_FILE = window.TOPIC_PAPERS_FILE || "topic_papers.json";
  // Node background icons live in assets/img/ (sibling of the json dir).
  var ICON_BASE = (window.TOPIC_ICON_BASE || BASE.replace(/json\/?$/, "img/")).replace(/\/?$/, "/");

  // Coarse super-category -> colour. 3Blue1Brown / Manim palette.
  var GROUP_COLORS = {
    "Estimation & Mapping": "#58C4DD",   // BLUE
    "Perception": "#5CD0B3",             // TEAL
    "Learning": "#F0AC5F",               // GOLD
    "Manipulation": "#FC6255",           // RED
    "Locomotion & Platforms": "#83C167", // GREEN
    "Planning & Control": "#E8C547",     // YELLOW
    "Applications": "#CF8DE5",           // PURPLE
    "Methods & Tooling": "#E07A9B",      // MAROON/PINK
  };
  var DEFAULT_COLOR = "#9CDCEB";
  var ACCENT = "#58C4DD"; // 3b1b signature blue

  function esc(s) {
    return (s == null ? "" : String(s)).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function el(id) { return document.getElementById(id); }

  Promise.all([
    fetch(BASE + GRAPH_FILE).then(function (r) { return r.json(); }),
    fetch(BASE + PAPERS_FILE).then(function (r) { return r.json(); }),
  ]).then(function (res) {
    init(res[0], res[1]);
  }).catch(function (e) {
    var p = el("tm-panel");
    if (p) p.innerHTML = '<p class="tm-error">Failed to load topic data: ' + esc(e.message) + "</p>";
    // eslint-disable-next-line no-console
    console.error("topic-graph: data load failed", e);
  });

  function init(graph, papers) {
    if (window.cytoscapeFcose) { window.cytoscapeFcose(window.cytoscape); }

    var counts = graph.nodes.map(function (n) { return n.count; });
    var maxCount = Math.max.apply(null, counts);
    var maxWeight = Math.max.apply(null, graph.edges.map(function (e) { return e.weight; }).concat([1]));

    var elements = [];
    graph.nodes.forEach(function (n) {
      elements.push({ data: {
        id: n.id, label: n.label, group: n.group, count: n.count,
        paperIds: n.paperIds,
        color: n.color || GROUP_COLORS[n.group] || DEFAULT_COLOR,
        // node diameter 22..96 px, scaled by sqrt(count)
        size: 22 + 74 * Math.sqrt(n.count / maxCount),
        iconUrl: n.icon ? ICON_BASE + n.icon : "",
      }});
    });
    graph.edges.forEach(function (e) {
      elements.push({ data: {
        id: e.source + "__" + e.target, source: e.source, target: e.target,
        weight: e.weight, width: 1 + 7 * (e.weight / maxWeight),
      }});
    });

    // Topic map: labels centered inside the node (light text, dark outline).
    // Platform map: labels below the node, coloured to match, no outline.
    var labelsInside = !(graph.meta && graph.meta.kind === "platforms");
    var labColor = labelsInside ? "#eef4ff" : "data(color)";
    var labValign = labelsInside ? "center" : "bottom";
    var labMargin = labelsInside ? 0 : 5;
    var labMaxW = labelsInside ? "data(size)" : 110;
    var labOutline = labelsInside ? 2.6 : 0;

    var cy = window.cytoscape({
      container: el("tm-graph"),
      elements: elements,
      wheelSensitivity: 0.2,
      style: [
        { selector: "node", style: {
          "background-color": "data(color)",
          "background-opacity": 0.96,
          "width": "data(size)", "height": "data(size)",
          "label": "data(label)",
          "font-size": 11, "font-weight": 600,
          "font-family": "\"Source Serif 4\", Charter, Georgia, serif",
          "color": labColor,
          "text-valign": labValign, "text-halign": "center", "text-margin-y": labMargin,
          "text-wrap": "wrap", "text-max-width": labMaxW,
          "text-outline-color": "#0a0c16", "text-outline-width": labOutline,
          // circular ring (no box); transparent until highlighted
          "border-width": 3, "border-color": "data(color)", "border-opacity": 0,
          "transition-property": "opacity border-opacity border-width",
          "transition-duration": "150ms",
        }},
        { selector: "node[iconUrl != '']", style: {
          "background-image": "data(iconUrl)",
          "background-fit": "contain",
          "background-clip": "node",
          "background-image-opacity": 0.85,
          "background-image-containment": "over",
        }},
        { selector: "edge", style: {
          "width": "data(width)", "line-color": "#39507a",
          "curve-style": "haystack", "haystack-radius": 0, "opacity": 0.5,
        }},
        { selector: "node:selected", style: {
          "border-width": 5, "border-color": ACCENT, "border-opacity": 1,
        }},
        { selector: "node.hl", style: {
          "border-width": 3, "border-color": ACCENT, "border-opacity": 0.85,
        }},
        { selector: "edge.hl", style: { "line-color": ACCENT, "opacity": 0.9 } },
        { selector: ".faded", style: { "opacity": 0.06, "text-opacity": 0.04 } },
        { selector: ".hl", style: { "opacity": 1, "text-opacity": 1 } },
      ],
      layout: window.cytoscapeFcose
        ? { name: "fcose", quality: "proof", nodeRepulsion: 9000,
            idealEdgeLength: function (e) { return 140 / Math.sqrt(e.data("weight")); },
            animate: false, padding: 30 }
        : { name: "cose", animate: false, padding: 30 },
    });

    // --- interactions -------------------------------------------------------
    var panel = el("tm-panel");
    var info = {};  // topic id -> {label, group, color, count}
    graph.nodes.forEach(function (n) {
      info[n.id] = { label: n.label, group: n.group, count: n.count,
                     color: GROUP_COLORS[n.group] || DEFAULT_COLOR };
    });

    // Search state: a query restricts which papers are listed/clickable.
    var matchSet = null;   // Set<paperId> matching the query, or null when no query
    var matchQuery = "";

    function paperMatches(p, q) {
      if (p.title && p.title.toLowerCase().indexOf(q) !== -1) return true;
      if (p.authors) {
        for (var i = 0; i < p.authors.length; i++) {
          if (String(p.authors[i]).toLowerCase().indexOf(q) !== -1) return true;
        }
      }
      return false;
    }

    function renderPapers(ids, headHtml) {
      ids = ids.slice().sort(function (a, b) {
        return (papers[a] ? papers[a].title : a).localeCompare(papers[b] ? papers[b].title : b);
      });
      var html = headHtml + '<ol class="tm-papers">';
      if (!ids.length) {
        html += '<li class="tm-paper"><span class="tm-hint">No papers.</span></li>';
      }
      ids.forEach(function (pid) {
        var p = papers[pid] || { title: pid, source: "" };
        var badge = p.source === "workshop" ? "W" : "P";
        html += '<li class="tm-paper" data-pid="' + esc(pid) + '">' +
          '<span class="tm-badge tm-' + badge + '">' + badge + "</span>" +
          '<span class="tm-title">' + esc(p.title) + "</span>" +
          '<div class="tm-abstract" hidden></div></li>';
      });
      html += "</ol>";
      panel.innerHTML = html;
      panel.querySelectorAll(".tm-paper[data-pid]").forEach(function (li) {
        li.querySelector(".tm-title").addEventListener("click", function () {
          var pid = li.getAttribute("data-pid");
          var box = li.querySelector(".tm-abstract");
          if (!box.hidden) { box.hidden = true; return; }
          var p = papers[pid] || {};
          var meta = [];
          if (p.authors && p.authors.length) meta.push(esc(p.authors.slice(0, 8).join(", ")));
          if (p.source === "proceedings" && pid.indexOf("pp:") === 0) {
            meta.push("Paper ID " + esc(pid.slice(3)));
          }
          if (p.workshop) meta.push("Workshop: " + esc(p.workshop));
          var links = [];
          if (p.url) links.push('<a href="' + esc(p.url) + '" target="_blank" rel="noopener">source</a>');
          if (p.code) links.push('<a href="' + esc(p.code) + '" target="_blank" rel="noopener">code</a>');
          box.innerHTML =
            (meta.length ? '<div class="tm-authors">' + meta.join(" — ") + "</div>" : "") +
            "<p>" + esc(p.abstract || "(no abstract)") + "</p>" +
            (links.length ? '<div class="tm-links">' + links.join(" · ") + "</div>" : "");
          box.hidden = false;
        });
      });
    }

    function topicHead(tid, ids) {
      var t = info[tid] || { label: tid, group: "", color: DEFAULT_COLOR };
      var sub = esc(t.group) + " · " + ids.length + " papers";
      if (matchSet) sub += ' matching “' + esc(matchQuery) + '”';
      return '<div class="tm-topic-head" style="border-color:' + esc(t.color) + '">' +
        "<h3>" + esc(t.label) + "</h3><span class=\"tm-meta\">" + sub + "</span></div>";
    }

    // Show a topic's papers (intersected with the active search, if any).
    function showTopic(tid) {
      var ids = (info[tid] ? (graphPaperIds[tid] || []) : []).slice();
      if (matchSet) ids = ids.filter(function (p) { return matchSet.has(p); });
      renderPapers(ids, topicHead(tid, ids));
    }

    // paperIds per topic, kept off the cy nodes for convenience
    var graphPaperIds = {};
    graph.nodes.forEach(function (n) { graphPaperIds[n.id] = n.paperIds || []; });

    function showSearchResults() {
      var ids = [];
      matchSet.forEach(function (pid) { ids.push(pid); });
      var head = '<div class="tm-topic-head" style="border-color:' + ACCENT + '">' +
        "<h3>" + ids.length + ' result' + (ids.length === 1 ? "" : "s") + "</h3>" +
        '<span class="tm-meta">matching “' + esc(matchQuery) +
        '” · click a highlighted topic to refine</span></div>';
      renderPapers(ids, head);
    }

    function highlightTopics(topicSet) {
      cy.batch(function () {
        cy.elements().addClass("faded").removeClass("hl");
        cy.nodes().unselect();
        cy.nodes().forEach(function (n) {
          if (topicSet.has(n.id())) { n.removeClass("faded").addClass("hl"); }
        });
        cy.edges().forEach(function (e) {
          if (topicSet.has(e.source().id()) && topicSet.has(e.target().id())) {
            e.removeClass("faded").addClass("hl");
          }
        });
      });
    }

    function clearSearch() {
      matchSet = null; matchQuery = "";
      cy.elements().removeClass("faded hl");
      cy.nodes().unselect();
      showIntro();
    }

    // --- node click: list papers (refines within a search) ------------------
    cy.on("tap", "node", function (evt) {
      var node = evt.target;
      if (matchSet) {
        cy.nodes().unselect(); node.select();   // keep search highlight, mark focus
      } else {
        cy.batch(function () {
          cy.elements().addClass("faded").removeClass("hl");
          node.removeClass("faded").addClass("hl");
          node.neighborhood().removeClass("faded").addClass("hl");
        });
      }
      showTopic(node.id());
    });
    cy.on("tap", function (evt) {
      if (evt.target === cy && !matchSet) { cy.elements().removeClass("faded hl"); }
    });

    // --- search -------------------------------------------------------------
    var search = el("tm-search");
    if (search) {
      search.addEventListener("input", function () {
        var q = search.value.trim().toLowerCase();
        if (!q) { clearSearch(); return; }
        matchQuery = q;
        matchSet = new Set();
        var topicsHit = new Set();
        Object.keys(papers).forEach(function (pid) {
          if (paperMatches(papers[pid], q)) {
            matchSet.add(pid);
            (papers[pid].topics || []).forEach(function (t) { topicsHit.add(t); });
          }
        });
        highlightTopics(topicsHit);
        showSearchResults();
      });
    }
    var reset = el("tm-reset");
    if (reset) {
      reset.addEventListener("click", function () {
        if (search) search.value = "";
        clearSearch();
        cy.fit(undefined, 30);
      });
    }

    // --- legend -------------------------------------------------------------
    var legend = el("tm-legend");
    if (legend) {
      var groups = (graph.meta && graph.meta.groups) || [];
      if (!groups.length) {  // fallback: derive from nodes
        var seen = {};
        graph.nodes.forEach(function (n) {
          if (!seen[n.group]) { seen[n.group] = 1; groups.push({ group: n.group, color: n.color || GROUP_COLORS[n.group] || DEFAULT_COLOR }); }
        });
      }
      legend.innerHTML = groups.map(function (g) {
        return '<span class="tm-leg"><i style="background:' + esc(g.color) + '"></i>' + esc(g.group) + "</span>";
      }).join("");
    }

    function showIntro() {
      panel.innerHTML = '<p class="tm-hint">Search paper titles &amp; authors above, or click a topic. ' +
        graph.meta.papers + ' papers across ' + graph.nodes.length +
        ' topics. Click a paper title to read its abstract.</p>';
    }
    showIntro();
  }
})();
