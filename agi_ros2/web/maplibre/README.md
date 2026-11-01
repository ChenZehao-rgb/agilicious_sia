# Local basemap renderer

Vendored upstream distribution files, unmodified:

- MapLibre GL JS 5.13.0: https://unpkg.com/maplibre-gl@5.13.0/dist/
  (`maplibre-gl.js`, `maplibre-gl.css`, `LICENSE-maplibre.txt`).
- MapLibre GL Leaflet 0.1.3:
  https://unpkg.com/@maplibre/maplibre-gl-leaflet@0.1.3/
  (`leaflet-maplibre-gl.js`, `LICENSE-leaflet-adapter`).

MapLibre renders only the vector basemap. Existing Leaflet telemetry/goal layers
remain in use. Version >= 5.11 is required for local-font labels with `glyphs`
omitted: https://maplibre.org/maplibre-style-spec/glyphs/ . System fonts must
contain the desired scripts (e.g. Chinese). No CDN or font server is used by the
browser. Distribution source maps are not required at runtime.

Styles and sprites are fetched from OpenFreeMap on first online use and copied
into each offline package. The Liberty style is adapted to use local fonts and
vector layers only (no remote Natural Earth raster background).
Attribution is preserved through the Leaflet adapter.

Provider and offline download terms: https://openfreemap.org/tos/ .
