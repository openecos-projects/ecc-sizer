#!/usr/bin/env python3
"""Check Sizer sources using the actual CMake compile database, without editing files."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def compile_database(build_dir):
    entries = json.loads((build_dir / "compile_commands.json").read_text())
    selected = {}
    for entry in entries:
        source = Path(entry["file"])
        if not source.is_absolute():
            source = Path(entry["directory"]) / source
        source = source.resolve()
        if not source.is_relative_to(ROOT / "src") or source.suffix not in (".cpp", ".cu"):
            continue
        if str(source) in selected:
            continue  # The STA test target compiles several implementation files again.
        args = entry.get("arguments") or shlex.split(entry["command"])
        if source.suffix == ".cu":
            # Translate NVCC-only flags for Clang's CUDA parser. Keep actual
            # defines/includes and C++17; this does not change the NVCC build.
            expanded = []
            iterator = iter(args[1:])
            for arg in iterator:
                if arg == "--options-file":
                    response = Path(entry["directory"]) / next(iterator)
                    expanded.extend(shlex.split(response.read_text()))
                else:
                    expanded.append(arg)
            flags = []
            iterator = iter(expanded)
            for arg in iterator:
                if arg in ("-I", "-isystem", "-D", "-U"):
                    flags.extend((arg, next(iterator)))
                elif arg.startswith(("-I", "-D", "-U", "-std=")):
                    flags.append(arg)
            cuda = Path(args[0]).resolve().parent.parent
            args = ["clang++", *flags, f"--cuda-path={cuda}",
                    "--cuda-gpu-arch=sm_80", "--no-cuda-version-check",
                    "-Wno-unknown-cuda-version", "-c", str(source)]
        selected[str(source)] = {"directory": entry["directory"],
                                 "file": str(source), "arguments": args}
    if not selected:
        raise ValueError("No Sizer translation units found in compile_commands.json")
    return list(selected.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build")
    parser.add_argument("--tool", choices=("clang-tidy", "clangd"), default="clang-tidy")
    parser.add_argument("--executable", help="Tool path, e.g. /usr/lib/llvm-18/bin/clang-tidy")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    tool = shutil.which(args.executable or args.tool)
    if not tool:
        parser.error(f"{args.executable or args.tool} not found")
    if args.tool == "clang-tidy":
        config = subprocess.run(
            [tool, "--verify-config", "--config-file=" + str(ROOT / ".clang-tidy")],
            cwd=ROOT, capture_output=True, text=True)
        if config.returncode:
            parser.error(config.stdout + config.stderr)
    try:
        entries = compile_database(args.build_dir.resolve())
    except (OSError, ValueError) as error:
        parser.error(str(error))
    with tempfile.TemporaryDirectory(prefix="sizer-lint-") as directory:
        database = Path(directory)
        (database / "compile_commands.json").write_text(json.dumps(entries))

        def check(entry):
            if args.tool == "clang-tidy":
                command = [tool, entry["file"], "-p", str(database),
                           "--warnings-as-errors=*",
                           "--header-filter=" + re.escape(str(ROOT / "src"))
                           + r"/(actions/|algorithms/|repair/|[^/]+\.(h|cpp)$)"]
            else:
                # Check-mode's refactoring-tweak probes can fail on valid break/
                # continue statements. Disable those probes, not diagnostics.
                command = [tool, "--check=" + entry["file"],
                           "--compile-commands-dir=" + str(database), "--tweaks="]
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            return entry["file"], result

        failed = 0
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            for source, result in pool.map(check, entries):
                name = Path(source).relative_to(ROOT)
                print(f"{'FAIL' if result.returncode else 'PASS'} {name}", flush=True)
                if result.returncode:
                    failed += 1
                    print(result.stdout + result.stderr, flush=True)
        print(f"{len(entries) - failed}/{len(entries)} translation units passed")
        return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
