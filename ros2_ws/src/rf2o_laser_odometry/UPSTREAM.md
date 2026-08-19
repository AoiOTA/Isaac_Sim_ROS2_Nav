# Vendored upstream

- Repository: https://github.com/MAPIRlab/rf2o_laser_odometry
- Commit: `b38c68e46387b98845ecbfeb6660292f967a00d3`
- Imported: 2026-08-20
- License: GPLv3; see `LICENSE`.

Local patches keep the algorithm intact while making the ROS 2 package usable
on Jazzy: package-format/dependency fixes, a single ROS node, TF readiness
before initialization, scan-derived monotonic timestamps, topic-only odometry,
and explicit conservative covariance parameters. The vendored source is built
locally and is never downloaded at launch time.
