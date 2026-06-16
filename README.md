# olayasturias.github.io

Personal academic website for Olaya Álvarez-Tuñón, built with [Jekyll](https://jekyllrb.com/) on the [al-folio](https://github.com/alshedivat/al-folio) theme. Published at <https://olayasturias.github.io/> via GitHub Pages.

## Run locally

Native Windows builds are not supported (al-folio needs imagemagick, jekyll-scholar, and Node-based execjs). Use Docker for local preview.

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and Docker Compose

### Serve the site

```bash
docker compose up
```

This runs `bin/entry_point.sh` inside the container (`bundle install` + `jekyll serve`), then serves at:

- Site: <http://localhost:8081>
- LiveReload: port `35729`

Stop with `Ctrl+C`, or `docker compose down` to remove the container.

### Notes

- Editing `_config.yml` triggers a full Jekyll restart; other files hot-reload.
- The container regenerates `Gemfile.lock` if it is untracked.
- A lighter variant is available via `docker compose -f docker-compose-slim.yml up`.

## Deploy

Deployment is automated by GitHub Actions (`.github/workflows/deploy.yml`): on push to `master`/`main` it builds the site, purges unused CSS, and publishes `_site/` to the `gh-pages` branch. The repo's Pages source is set to the `gh-pages` branch.
