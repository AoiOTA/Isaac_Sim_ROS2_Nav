# Final Bio Navigation development

- Develop only in the `final_bio_navigation` worktree and branch created from
  this repository's `main`. Never read, enter, diff, or search the
  `complete-cognitive-navigation` branch or worktree.
- Module3 exclusively owns estimated odometry, localization TF, physical maps
  and graph legality, Route Server, Nav2, collision safety, and `/cmd_vel`.
  Module2 inputs are bounded additive guidance and must fail open when stale or
  unhealthy.
- Use a fresh explorer, coder, and reviewer for each development stage. Only
  the designated coder may write; explorers and reviewers remain read-only.
- Optimize for a fast research loop: make small reversible changes and run
  focused unit/launch-contract tests before any simulator campaign.
- Ordinary Git history, handoff notes, and experiment directories are enough.
  Do not add SHA256, receipt, sealed-evidence, or formal-provenance workflows.
- `/ground_truth/*` is evaluator/diagnostic data only. Online route tracking
  consumes `/odom` plus TF.
