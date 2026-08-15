"""Build the frozen Attempt30/A21 physical multi-route engineering bundle."""

from __future__ import annotations

import argparse
import json

from .benchmark_environment import generate_benchmark_bundle


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--defaults", required=True)
    parser.add_argument("--asset-root", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser


def main(argv=None) -> None:
    arguments = build_parser().parse_args(argv)
    summary = generate_benchmark_bundle(
        arguments.layout,
        arguments.defaults,
        arguments.asset_root,
        arguments.output_dir,
    )
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
