#!/usr/bin/env python3
"""Check that the Windows MAVExplorer bundle contains MagFit's SciPy imports."""

import importlib.util
import subprocess
import sys
from pathlib import Path

from scipy import optimize


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: check_mavexplorer_bundle.py MAVExplorer.exe")

    exe = Path(sys.argv[1]).resolve(strict=True)

    # This passes in regular Python even when the old PyInstaller bundle fails.
    result = optimize.least_squares(lambda x: x - 2, [0.0])
    if abs(result.x[0] - 2) > 1e-6:
        raise RuntimeError("SciPy optimize failed in regular Python")

    # SciPy 1.18 moved array_api_compat from _lib to _external.
    candidates = (
        "scipy._external.array_api_compat.numpy.fft",
        "scipy._lib.array_api_compat.numpy.fft",
    )
    fft_module = None
    for candidate in candidates:
        try:
            if importlib.util.find_spec(candidate) is not None:
                fft_module = candidate
                break
        except ModuleNotFoundError:
            pass
    if fft_module is None:
        raise RuntimeError("Could not locate SciPy's array_api_compat FFT module")

    archive = subprocess.run(
        [sys.executable, "-m", "PyInstaller.utils.cliutils.archive_viewer",
         "--brief", "--recursive", str(exe)],
        capture_output=True, text=True, check=True,
    )
    bundled_modules = {line.strip() for line in archive.stdout.splitlines()}
    missing = {"scipy.optimize", fft_module} - bundled_modules
    if missing:
        raise RuntimeError(f"MAVExplorer bundle is missing: {sorted(missing)}")

    startup = subprocess.run([str(exe), "--version"], capture_output=True, text=True)
    if startup.returncode not in (0, 1) or "MAVExplorer Version:" not in startup.stdout:
        raise RuntimeError(
            f"MAVExplorer did not start: {startup.returncode}\n"
            f"{startup.stdout}\n{startup.stderr}"
        )

    print(f"MAVExplorer startup and bundled {fft_module} verified")


if __name__ == "__main__":
    main()
