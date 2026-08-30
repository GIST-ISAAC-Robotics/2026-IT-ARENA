# IT ARENA project handoff rules

Before changing this repository, read these files in order:

1. `docs/PROJECT_CONTEXT.md`
2. the newest file in `docs/activity/`
3. any relevant record in `docs/decisions/`
4. `docs/track/TRACK_AUDIT.md` when touching the course or simulator world

Keep these records current:

- Update `docs/PROJECT_CONTEXT.md` when confirmed facts, decisions, blockers, or next actions change.
- Append material work to the current date's file in `docs/activity/`. Record outcomes and evidence, not a transcript of commands.
- Add an ADR under `docs/decisions/` for choices that would be costly to reverse.
- Do not edit `assets/track/original/it_arena_track_final.zip`. It is the byte-for-byte source archive.
- Treat `assets/track/source/it_arena_track/output_final/scene.json` and `world.sdf` as the observed current outputs, not the accompanying README. Their known inconsistencies are documented in the track audit.
- Hardware dimensions in `src/arena_description/config/vehicle.yaml` are provisional until explicitly marked confirmed.
- Keep simulated and real vehicle-facing ROS topics compatible. Simulation-only ground truth must never be consumed by competition autonomy nodes.

The repeatable track + parameterized Ackermann + D435i/encoder interface baseline was verified on 2026-08-30. The next milestone is ground-truth centerline following with curvature-based speed control; do not skip directly to opaque end-to-end racing behavior.
