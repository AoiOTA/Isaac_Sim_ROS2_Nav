"""Order lifecycle shutdown while ROS contexts and executors are still valid."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time
from typing import Sequence


@dataclass(frozen=True)
class ShutdownStep:
    """One lifecycle service transition in the required shutdown order."""

    label: str
    service: str
    kind: str
    command: int


def shutdown_steps(operation: str) -> tuple[ShutdownStep, ...]:
    """Return strict managed-node ordering for one bringup operation."""

    normalized = operation.strip().lower()
    if normalized == "navigation":
        return (
            ShutdownStep(
                "navigation lifecycle manager",
                "/lifecycle_manager_navigation/manage_nodes",
                "manager",
                4,
            ),
            ShutdownStep(
                "localization lifecycle manager",
                "/lifecycle_manager_localization/manage_nodes",
                "manager",
                4,
            ),
        )
    if normalized == "localization":
        return (
            ShutdownStep(
                "localization lifecycle manager",
                "/lifecycle_manager_localization/manage_nodes",
                "manager",
                4,
            ),
        )
    if normalized in {"mapping", "incremental_mapping"}:
        return (
            ShutdownStep(
                "slam_toolbox deactivate",
                "/slam_toolbox/change_state",
                "lifecycle",
                4,
            ),
            ShutdownStep(
                "slam_toolbox cleanup",
                "/slam_toolbox/change_state",
                "lifecycle",
                2,
            ),
            ShutdownStep(
                "slam_toolbox shutdown",
                "/slam_toolbox/change_state",
                "lifecycle",
                5,
            ),
        )
    raise ValueError(f"unsupported operation for ordered shutdown: {operation!r}")


def _call_step(
    node,
    executor,
    step: ShutdownStep,
    timeout_s: float,
) -> tuple[bool, str]:
    if step.kind == "manager":
        from nav2_msgs.srv import ManageLifecycleNodes

        service_type = ManageLifecycleNodes
        request = ManageLifecycleNodes.Request()
        request.command = step.command
    else:
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState

        service_type = ChangeState
        request = ChangeState.Request()
        request.transition.id = step.command
        request.transition.label = {
            Transition.TRANSITION_DEACTIVATE: "deactivate",
            Transition.TRANSITION_CLEANUP: "cleanup",
            Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: "shutdown",
        }.get(step.command, "")

    client = node.create_client(service_type, step.service)
    try:
        deadline = time.monotonic() + timeout_s
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"service unavailable after {timeout_s:.1f}s"
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            return False, f"step deadline exhausted after {timeout_s:.1f}s"
        future = client.call_async(request)
        import rclpy

        rclpy.spin_until_future_complete(
            node,
            future,
            executor=executor,
            timeout_sec=remaining_s,
        )
        if not future.done():
            future.cancel()
            return False, f"request timed out within {timeout_s:.1f}s step budget"
        try:
            response = future.result()
        except Exception as exc:
            return False, f"{type(exc).__name__}: {exc}"
        if response is None or not getattr(response, "success", False):
            return False, "service rejected transition"
        return True, "complete"
    finally:
        node.destroy_client(client)


def run_ordered_shutdown(operation: str, *, timeout_s: float = 20.0) -> bool:
    """Execute all available shutdown transitions in process-safe order."""

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node("isaac_nav_ordered_shutdown", context=context)
    executor = SingleThreadedExecutor(context=context)
    overall = True
    deadline = time.monotonic() + timeout_s
    try:
        for step in shutdown_steps(operation):
            started = time.monotonic()
            remaining_s = deadline - started
            if remaining_s <= 0.0:
                print(
                    f"ordered shutdown: {step.label}: WARN "
                    f"(global {timeout_s:.1f}s deadline exhausted)",
                    file=sys.stderr,
                    flush=True,
                )
                overall = False
                break
            success, detail = _call_step(
                node, executor, step, remaining_s
            )
            elapsed = time.monotonic() - started
            stream = sys.stdout if success else sys.stderr
            print(
                f"ordered shutdown: {step.label}: "
                f"{'PASS' if success else 'WARN'} ({elapsed:.3f}s; {detail})",
                file=stream,
                flush=True,
            )
            overall = success and overall
        return overall
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if context.ok():
            context.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "operation",
        choices=("mapping", "incremental_mapping", "localization", "navigation"),
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    if arguments.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")
    return 0 if run_ordered_shutdown(
        arguments.operation, timeout_s=arguments.timeout
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
