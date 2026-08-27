#!/usr/bin/python3
"""Wait for one exact std_srvs/srv/Empty service using rclpy discovery."""

from __future__ import annotations

import argparse
import os
import sys

import rclpy
from std_srvs.srv import Empty


def _positive_timeout(value: str) -> float:
    timeout = float(value)
    if timeout <= 0.0:
        raise argparse.ArgumentTypeError("timeout must be positive")
    return timeout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", required=True)
    parser.add_argument("--timeout", required=True, type=_positive_timeout)
    args = parser.parse_args()

    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    node = None
    rclpy.init(args=None)
    try:
        node = rclpy.create_node(
            f"bio_nav_wait_for_empty_service_{os.getpid()}",
            enable_rosout=False,
            start_parameter_services=False,
        )
        client = node.create_client(Empty, args.service)
        if client.wait_for_service(timeout_sec=args.timeout):
            print(
                f"ready: {args.service} [std_srvs/srv/Empty] "
                f"on ROS_DOMAIN_ID={domain_id}"
            )
            return 0
        print(
            f"not ready after {args.timeout:g}s: {args.service} "
            f"[std_srvs/srv/Empty] on ROS_DOMAIN_ID={domain_id}",
            file=sys.stderr,
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
