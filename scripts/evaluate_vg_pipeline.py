#!/usr/bin/env python3
"""Evaluate VG and native repair on identical inputs, including stage snapshots."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--equiv-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sizer', type=Path, required=True)
    parser.add_argument('--openroad', type=Path, required=True)
    parser.add_argument('--budgets', type=int, nargs='+', default=[64, 512, 0],
                        help='VG buffer budgets for combined flows; 0 means unlimited')
    args = parser.parse_args()
    if any(b < 0 for b in args.budgets) or len(set(args.budgets)) != len(args.budgets):
        parser.error('budgets must be distinct nonnegative integers')
    args.output.mkdir(parents=True, exist_ok=True)
    runner = Path(__file__).with_name('evaluate_fanout.py').resolve()
    variants = [('native', ['--repair-setup']),
                ('vg', ['--repair-vg', '--repair-set', 'vg.max_buffers=0'])]
    variants += [(f'pipeline_{b or "unlimited"}',
                  ['--repair-vg', '--repair-setup', '--repair-set', f'vg.max_buffers={b}'])
                 for b in args.budgets]
    manifest = dict(commands={}, completed=[], budgets=args.budgets,
                    note='Sequential runs; stage metrics are independently reloaded placement estimates.')
    for path in [args.sizer, args.openroad]:
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for data in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(data)
        manifest[str(path.resolve())] = h.hexdigest()
    combined = []
    for name, flags in variants:
        command = [sys.executable, str(runner), '--benchmark', str(args.benchmark.resolve()),
                   '--equiv-dir', str(args.equiv_dir.resolve()),
                   '--output', str((args.output/name).resolve()),
                   '--sizer', str(args.sizer.resolve()), '--openroad', str(args.openroad.resolve()),
                   '--stage-metrics', '--repair-set', 'power.enabled=false'] + flags
        manifest['commands'][name] = command
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(f'Running {name}', flush=True)
        with (args.output/f'{name}.log').open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        rows = json.loads((args.output/name/'summary.json').read_text())
        for row in rows:
            row['variant'] = name
        combined.extend(rows)
        manifest['completed'].append(name)
        (args.output/'summary.json').write_text(json.dumps(combined, indent=2)+'\n')
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(f'Completed {name}', flush=True)


if __name__ == '__main__':
    main()
