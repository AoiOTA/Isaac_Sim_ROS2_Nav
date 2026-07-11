# Contributing

This repository is developed in acceptance-gated phases. Keep every change
small enough to review, reproduce, and revert independently.

## Development workflow

1. Read the relevant phase and acceptance criteria in `plan.md`.
2. Create or update implementation code, configuration, and tests together.
3. Run the narrowest relevant tests, followed by the applicable workspace
   checks documented in `docs/development.md`.
4. Review `git diff` and `git status`; stage only the intended files.
5. Commit only after the phase-level behavior has objective verification.

Do not mix unrelated refactors, generated data, formatting sweeps, or asset
imports into a feature commit. A behavior-changing commit must include its
configuration and tests. Update documentation in the same commit when the
operator workflow or public interface changes.

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/):

```text
<type>(<optional-scope>): <imperative summary>

<optional rationale and verification notes>
```

Supported types include `feat`, `fix`, `test`, `docs`, `refactor`, `perf`,
`build`, `ci`, and `chore`. Keep the subject concise and describe one coherent
change. Use `!` and a `BREAKING CHANGE:` footer only for intentional interface
breaks.

Examples:

```text
feat(stage): validate a single PhysicsScene during composition
test(scan): cover lidar height projection boundaries
docs(experiments): document reproducible run metadata
```

Where useful, include the verification command and result in the commit body;
do not commit terminal logs merely as evidence.

## Repository data and assets

Never commit:

- ROS `build/`, `install/`, or `log/` trees;
- Python caches, virtual environments, Isaac Sim logs, or crash dumps;
- rosbags, generated trajectories, batch metrics, reports, or run directories;
- the external Isaac Sim warehouse environment or another machine's absolute
  asset tree;
- credentials, tokens, local DDS profiles, or machine-specific settings.

The official warehouse remains an external runtime dependency. Commit only
project-authored USD layers and assets deliberately vendored for the robot,
with their provenance documented.

Small deterministic test data belongs in a `fixtures/` directory under the
relevant data or test directory. Generated occupancy maps and pose graphs are
ignored by default. If a curated map is required for reproducibility, track it
with Git LFS or publish it to the project artifact store; record its checksum,
format, source scenario, and retrieval instructions. Do not bypass the ignore
rules to place large binary maps in ordinary Git history.

See `data/README.md` for the data lifecycle and `docs/development.md` for build
and test commands.

## Documentation maintenance

Keep the operator documentation synchronized with the code:

- update `docs/user_manual.md` when a command, startup sequence, required
  argument, expected result, or troubleshooting procedure changes;
- update `docs/repository_index.md` whenever a tracked file is added, removed,
  renamed, or changes responsibility;
- keep `README.md` as the short GitHub entrypoint and link detailed procedures
  instead of duplicating them there;
- keep runtime claims in `docs/verification.md` evidence-backed and separate
  smoke results from statistical acceptance.

Before a documentation handoff, compare `git ls-files` with the backticked file
rows in `docs/repository_index.md` so every tracked file remains indexed.

## Pull-request and handoff checklist

- The diff addresses one phase or one clearly bounded prerequisite.
- Code, configuration, tests, and operator documentation agree.
- Relevant pytest and/or colcon tests pass in the documented environment.
- Simulator-only checks include the Isaac Sim version and observed result.
- Topic, frame, QoS, and TF ownership changes are called out explicitly.
- Generated data and external warehouse assets are absent from the diff.
- New versioned fixtures are small, deterministic, and documented.
