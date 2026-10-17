# Download cache

Runtime cache for **Nominatim** geocode responses and **Overpass** road JSON used by **Szukaj miejsca…** in the GUI.

| Subdir | Contents |
|--------|----------|
| `geocode/` | Nominatim search JSON (keyed by normalized query hash) |
| `overpass/` | Overpass road-network JSON (keyed by bbox + query version) |

- Reused automatically unless the user checks **Wymuś ponowne pobranie**.
- Large payloads are gitignored; this README is kept so the folder exists in the tree.
- Automated tests use `tests/output/cache_stage3/` with a fake HTTP transport — they never hit the public network.

Attribution for cached map data: © OpenStreetMap contributors (ODbL). Geocoding: Nominatim. See [docs/OSM_IMPORT.md](../../docs/OSM_IMPORT.md).
