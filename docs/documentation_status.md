# Documentation status

Reviewed for the current V6 handoff on 2026-08-29.

## Current documents

| Document | Role |
| --- | --- |
| [`../README.md`](../README.md) | Current entry links and shortest wrapper commands. |
| [`CURRENT_STATE.md`](CURRENT_STATE.md) | Module3 runtime boundary and pointer to the authoritative Integration handoff. |
| [`RUNBOOK.md`](RUNBOOK.md) | Clean-shell setup, current indoor/outdoor operation, NAS, and cleanup. |
| [`interfaces.md`](interfaces.md) | Current ownership, topics, TF, reset, and control contracts. |
| [`repository_index.md`](repository_index.md) | Current implementation/config/asset lookup. |
| [`handoff/EXPERIMENT_LEDGER.md`](handoff/EXPERIMENT_LEDGER.md) | Historical engineering ledger; entries retain their original evidence scope. |

## Fact priority

For current behavior, use this order:

1. the exact Module3/Integration/Module2 runtime tuple in Integration
   `docs/CURRENT_STATE.md`, referenced by [`CURRENT_STATE.md`](CURRENT_STATE.md);
2. current wrappers and launch/config implementation;
3. [`interfaces.md`](interfaces.md) and [`RUNBOOK.md`](RUNBOOK.md);
4. focused tests and current-run evidence;
5. the historical ledger, reports, and campaign documents.

Historical Rivermark, Attempt21-32, 4x20, and V6 milestone entries remain in
the ledger for their original evidence scope. They are not current commands or
proof that the V6 baseline has completed outdoor, appearance, or formal
qualification. No V6 micro-handoff files remain.

## Generated-report storage

The generated `docs/reports/` tree at source HEAD
`09c3ae80a5766ccf37fd244421e4c5f50afe7e91` is stored at
`/mnt/nas_home/Bio_Nav_Data/experiments/visualizations/module3_repo_generated_09c3ae80a5766ccf37fd244421e4c5f50afe7e91/docs/reports/`:
9 files, 512198 bytes, with per-file `cmp` PASS. The move does not promote its
evidence classification. `docs/report_assets/`, `docs/videos/`, and
`docs/figures/` remain tracked because repository callers still reference them.

## Maintenance boundary

- Runtime command changes update `README.md` and `RUNBOOK.md`.
- Topic, TF, reset, or authority changes update `interfaces.md`.
- Scene, map, spawn, GVG, or Nav2 profile changes update
  `repository_index.md` and `CURRENT_STATE.md`.
- Evidence claims state their layer: focused/build, live engineering, pilot,
  or formal qualification.
- A result from another repository may be linked at its current-state page;
  do not duplicate detailed metrics into Module3.
