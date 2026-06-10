# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Personal academic website for Olaya Álvarez-Tuñón, built with **Jekyll** on the **al-folio** theme (v1, gem-based). Published at https://olayasturias.github.io/ via GitHub Pages. Content: robotics / underwater-SLAM research — about page, BibTeX-driven publications, structured CV, projects, blog.

al-folio v1 ships as **theme gems** (`al_folio_core`, `al_icons`, `al_folio_cv`, …) — there are **no local `_layouts/`, `_includes/`, or `_sass/`**; those come from the gems. To override a theme file, copy it from the gem into a local path of the same name.

## Commands

Native Windows builds are not supported (al-folio needs imagemagick, jekyll-scholar, Node-based execjs). **Use Docker for local preview:**

```powershell
docker compose up        # bundle install + serve at http://localhost:8080 (livereload :35729)
```

`bin/entry_point.sh` runs `bundle check`/`bundle install` inside the container, then `jekyll serve`. The container regenerates `Gemfile.lock` if it is untracked. Editing `_config.yml` triggers a full Jekyll restart (watched by inotify); other files hot-reload.

## Deploy

- **GitHub Actions**, not native Pages. `.github/workflows/deploy.yml` builds on push to `master`/`main`, purges unused CSS, and deploys `_site/` to the **`gh-pages`** branch via `JamesIves/github-pages-deploy-action`.
- The repo's Pages source must be set to the **`gh-pages` branch** (Settings → Pages). A plain `master` push no longer serves the site directly — the Action does.
- `_site/` is gitignored.

## Architecture / where content lives

- **`_config.yml`** — site config. Identity via `first_name`/`last_name` (`title: blank` → uses full name). `url: https://olayasturias.github.io`, `baseurl: ""` (user page, not project page). `theme: al_folio_core`. `scholar:` block configures jekyll-scholar (name variants for citation matching). `giscus.repo` is set; `repo_id`/`category_id` must be filled from giscus.app. `imagemagick.enabled: true` generates responsive WebP from `assets/img/`.
- **`_pages/`** — `about.md` (`layout: about`, `permalink: /` — the home page; profile photo + selected papers + news + latest posts), `publications.md` (renders `{% bibliography %}`), `cv.md`, `projects.md`, `blog.md`, `news.md`, `404.md`.
- **`_bibliography/papers.bib`** — all publications as BibTeX. al-folio fields per entry: `abbr`, `abstract`, `doi`/`arxiv`, `html` (paper link), `code` (repo URL), `preview` (image in `assets/img/publication_preview/`), `selected={true}` (shows on the about page). Edit this file to add papers — the publications page is generated.
- **`_data/socials.yml`** — contact + social usernames (email, github, linkedin, orcid, bluesky, `cv_pdf`). Rendered by jekyll-socials. Add `scholar_userid` to enable the Scholar icon.
- **CV**: `cv_format: jsonresume` in `_pages/cv.md` renders **`assets/json/resume.json`** (JSON Resume schema, pulled in via `jekyll_get_json`). `cv_pdf: /assets/pdf/cv.pdf` is the download button. (al-folio also supports `rendercv`; we use jsonresume + a pre-made PDF.) The legacy `_data/cv.yml` is unused.
- **`_projects/`** — project pages (`layout: page`, `category`, `importance`, `img` thumbnail). `_pages/projects.md` groups by `display_categories` (currently `[creative]`). Images embedded via `{% include figure.liquid path="..." %}`.
- **`_news/`** — short inline announcements (`inline: true`) shown on the about page.
- **`_posts/`** — blog. al-folio front matter: `layout: post`, `title`, `date`, `description`, `tags` (space-separated → array), optional `thumbnail`. (Migrated from Mediumish: old `cover`→`thumbnail`, `summary`→`description`, `categories`→`tags`; `author`/`featured` dropped.) `featured: true` would surface a post in a featured row on `/blog/`.
- **`assets/`** — `img/` (theme + `prof_pic.jpg` + `publication_preview/`), `images/` (legacy post/portfolio images, kept so post bodies resolve), `pdf/cv.pdf`, `json/resume.json`. Post bodies use plain markdown/`<img>` against `assets/images/...`.

## Notes

- `_data/cv.yml`, `_data/coauthors.yml`, `_data/venues.yml` etc. are al-folio data files; only the ones referenced by enabled features matter.
- jemoji is enabled, so `:emoji:` shorthand renders in posts.
- Permalinks changed from the old Mediumish scheme to al-folio's `/blog/:year/:title/`.
