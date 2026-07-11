# Jackal navigation overlay

`jackal_nav.usda` is the Git-tracked, project-owned override layer. It deactivates
the legacy sensor frames and authors ROS-compatible mechanical and optical frames.

NVIDIA binary assets are intentionally excluded from Git. Import and verify them:

```bash
python3 isaac_sim/tools/import_assets.py
python3 isaac_sim/tools/import_assets.py --check
```

The importer preserves the original relative `configuration/` dependency below
`source/`. Do not reference the schema layer directly; it has no `defaultPrim`.
