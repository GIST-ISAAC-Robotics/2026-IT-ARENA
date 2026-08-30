# Supplied track audit

Audit date: 2026-08-30

## Provenance

- Imported from `C:\Users\Jinhyeong\Documents\카카오톡 받은 파일\it_arena_track_final.zip`.
- Preserved repository copy: `assets/track/original/it_arena_track_final.zip`.
- SHA-256: `DE448BA10C614E0F635D44B2F36BAB29EBF455C323DE442562DD01A8296758E4`.
- Extracted files are under `assets/track/source/it_arena_track/`.
- Do not edit the preserved archive. Runtime fixes belong in ROS/Gazebo package directories and must be traceable.

## Observed `output_final` facts

The actual `output_final/scene.json`, CSVs, image, and SDF describe:

- lap length: 46.6329 m;
- main track width: 0.35 m;
- two branches, each 0.12 m wide;
- branch spans: 13.2-16.8 m and 32.897-36.397 m along the main path;
- scene's assumed vehicle: 0.15 m x 0.10 m;
- six grid slots in two staggered columns;
- wall height: 0.30 m and thickness: 0.05 m;
- one 0.01 m-high speed bump in the observed output;
- signal gantry height: 1.20 m; lamp centers at approximately 1.08 m;
- ArUco dictionary: `DICT_4X4_50`, marker size 0.10 m;
- present marker IDs: 0, 20, 30, and 45;
- general minimum centerline radius: 0.298695 m;
- `min_radius_general_ok: false` in the generated verification record.

## Conflicts with the accompanying README

The README instead describes an older/different result, including:

- 38.11 m lap length;
- 0.15 m wall height;
- multiple bump groups;
- marker ID 10 and fake IDs 7, 23, 33;
- general minimum radius 0.45 m and a PASS;
- `track_gen.py` as the single source of truth.

The current `scene.json` calls the course a `track_editor.html` user-designed course, and the actual output names / values do not match several README claims. Therefore neither the README nor `track_gen.py` may be treated as authoritative for the event until the organizer confirms the final version.

## Runtime issues to fix in a derived copy

1. All red, yellow, and green lamp visuals are emissive simultaneously.
2. `traffic_light.py` broadcasts UDP state but does not change the Gazebo visuals.
3. ArUco texture paths in `world.sdf` use `../aruco/...` even though the images are in the same output directory's `aruco/` child. The accompanying `<script>` material elements also omit the required `<name>`, so strict SDF validation fails. The derived runtime world corrects the path and removes the malformed legacy script element while retaining the PBR texture.
4. The world contains no vehicle or competition logic, which is expected but must be supplied by this repository.
5. The general minimum-radius verification failure must be reconciled with the actual steering geometry.

The reproducible derived world is built by `scripts/build_runtime_world.py`. In addition to the ArUco material repairs, it merges repetitive static track links while retaining semantic traffic-light and marker links. The supplied output contains 1,090 links; the derived world reduces this to 10 without changing the individual geometry poses.

## Rule-impact observations

- A 0.12 m branch cannot accommodate a 0.15 m-wide vehicle. Even the scene's 0.10 m-wide assumed vehicle has only 0.01 m nominal clearance on each side.
- A 0.12 m-wide provisional vehicle has zero theoretical branch clearance. The branch must be considered unavailable until either its width or the actual vehicle width is confirmed.
- A high traffic signal and a low, road-facing D435i can be mutually incompatible. Visibility must be checked from all six grid slots.
- A 0.30 m wall is high relative to a 0.20 m-long car and can dominate LiDAR/depth observations; confirm whether the physical track will match it.

## Source hierarchy until organizer confirmation

1. Preserved ZIP hash for provenance.
2. `output_final/scene.json` and `world.sdf` for reproducing the supplied actual output.
3. CSV/map/image files for cross-checking geometry.
4. README only as historical intent, not current numeric truth.
