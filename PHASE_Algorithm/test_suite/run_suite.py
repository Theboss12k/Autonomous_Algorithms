#!/usr/bin/env python3
"""Run the PHASE test suite on one version from ../versions.

    python test_suite/run_suite.py V1.2                  # deterministic + 10 random per family
    python test_suite/run_suite.py V1.2 --random 0       # deterministic cases only
    python test_suite/run_suite.py V1.1 --cases 02_head_on,16_grazing

Any option after the version is passed to the suite unchanged (see
``phase_autonomous_test_suite_altitude_locked.py --help``). Output goes to
``test_suite/output/<version>/`` (git-ignored) unless you pass ``--output``.
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
VERSIONS = HERE.parent / "versions"
SUITE = HERE / "phase_autonomous_test_suite_altitude_locked.py"


def main() -> int:
    available = sorted(p.name for p in VERSIONS.iterdir() if (p / "phase_altitude_locked.py").exists())
    if len(sys.argv) < 2 or sys.argv[1] not in available:
        print(__doc__)
        print("versions:", ", ".join(available))
        return 2
    version, extra = sys.argv[1], sys.argv[2:]
    cmd = [sys.executable, str(SUITE), "--phase-file", str(VERSIONS / version / "phase_altitude_locked.py")]
    if "--output" not in extra:
        cmd += ["--output", str(HERE / "output" / version)]
    return subprocess.call(cmd + extra)


if __name__ == "__main__":
    sys.exit(main())
