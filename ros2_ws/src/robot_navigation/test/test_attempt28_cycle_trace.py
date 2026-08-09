#!/usr/bin/env python3
"""Prove the Attempt28 preload tracer preserves returned Twist values."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> None:
    probe, original, tracer = map(Path, sys.argv[1:4])
    for local_scope in (False, True):
        base_environment = dict(os.environ)
        if local_scope:
            base_environment['BIO_NAV_TRACE_TEST_LOCAL'] = '1'
        baseline = subprocess.check_output(
            (probe, original), text=True, env=base_environment)
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / 'cycles.jsonl'
            environment = dict(base_environment)
            environment['LD_PRELOAD'] = str(tracer)
            environment['BIO_NAV_ATTEMPT28_CYCLE_TRACE'] = str(trace)
            instrumented = subprocess.check_output(
                (probe, original), text=True, env=environment)
            if baseline != instrumented:
                raise SystemExit(
                    f'Twist changed: baseline={baseline!r} traced={instrumented!r}')
            rows = [json.loads(line) for line in trace.read_text().splitlines()]
            assert [row['event'] for row in rows] == [
                'compute_start', 'compute_end']
            assert all(
                row['schema'] == 'bio_nav_attempt28_controller_cycle_v1'
                for row in rows)
            assert rows[0]['cycle_id'] == rows[1]['cycle_id']
            assert rows[0]['steady_ns'] <= rows[1]['steady_ns']
            assert rows[1]['exception_flag'] is False


if __name__ == '__main__':
    main()
