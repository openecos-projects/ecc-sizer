#!/usr/bin/env python3
"""Repeat equal whole-net-buffer-budget VG CPU/CUDA and GBI CUDA experiments.

Each run reloads the original placed design. No design/setup/power repair is run.
Raw logs, fingerprints, binaries, commands, output hashes and timing ranges are
retained. This measures placement-estimated PPA and end-to-end process time;
solve_ms also includes host tree preparation/traceback, not just GPU kernels.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def capture(command):
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    return dict(command=command, status=result.returncode, output=result.stdout)


def spread(values):
    return dict(median=statistics.median(values), min=min(values), max=max(values))


def aggregate(records):
    groups = {}
    for record in records:
        groups.setdefault((record['design'], record['algorithm'], record['budget']), []).append(record)
    output = []
    for (design, algorithm, budget), runs in groups.items():
        metrics = {key: spread([r['metrics'][key] for r in runs])
                   for key in runs[0]['metrics'] if key != 'repair_ms'}
        output.append(dict(design=design, algorithm=algorithm, budget=budget,
                           repeats=len(runs), metrics=metrics,
                           ppa_identical=all(v['min'] == v['max'] for v in metrics.values()),
                           output_hashes_identical=len({json.dumps(r['output_sha256'], sort_keys=True) for r in runs}) == 1,
                           sizer_seconds=spread([r['sizer_seconds'] for r in runs]),
                           solve_ms=spread([r['topology']['solve_ms'] for r in runs]),
                           topology={k: spread([r['topology'][k] for r in runs])
                                     for k in runs[0]['topology']},
                           runs=[r['directory'] for r in runs]))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--equiv-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sizer', type=Path, required=True)
    parser.add_argument('--openroad', type=Path, required=True)
    parser.add_argument('--dont-use-list', type=Path,
                        help='same explicit master exclusion list for all algorithms')
    parser.add_argument('--designs', nargs='+', default=['aes_cipher_top', 'jpeg_encoder'])
    parser.add_argument('--budgets', nargs='+', type=int, default=[64, 0])
    parser.add_argument('--algorithms', nargs='+', choices=['vg-cpu', 'vg-cuda', 'gbi-cuda'],
                        default=['vg-cpu', 'vg-cuda', 'gbi-cuda'])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--gpu', default='0')
    parser.add_argument('--sta-threads', type=int, default=32)
    parser.add_argument('--vg-memory-mb', type=int, default=4096)
    parser.add_argument('--resume', action='store_true',
                        help='continue validated cases with identical binaries, inputs and settings')
    args = parser.parse_args()
    if args.repeats < 1 or args.sta_threads < 1 or args.vg_memory_mb < 1 or any(b < 0 for b in args.budgets):
        parser.error('invalid repetition/thread/memory/budget value')
    if not args.resume and args.output.exists() and any(args.output.iterdir()):
        parser.error('--output must be empty to preserve earlier evidence')
    if args.resume and not (args.output / 'comparison.json').is_file():
        parser.error('--resume requires an existing comparison.json')
    args.output.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[1]
    source_files = sorted(p for folder in ['src', 'scripts', 'tests']
                          for p in (repo / folder).rglob('*')
                          if p.is_file() and '__pycache__' not in p.parts)
    source_files += [repo / 'CMakeLists.txt']
    manifest = dict(policy='whole-net plans, collection order, skip oversized, zero unlimited',
                    args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items() if k != 'resume'},
                    git_head=capture(['git', 'rev-parse', 'HEAD']),
                    git_status=capture(['git', 'status', '--short']),
                    source_sha256={str(p.relative_to(repo)): sha(p) for p in source_files},
                    binaries={str(p.resolve()): sha(p) for p in [args.sizer, args.openroad]},
                    platform=platform.platform(), cpu=capture(['lscpu']),
                    gpu=capture(['nvidia-smi']), start_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                    commands=[])
    cache = args.sizer.resolve().parents[1] / 'CMakeCache.txt'
    if args.dont_use_list:
        manifest['dont_use_list'] = dict(path=str(args.dont_use_list.resolve()),
                                        sha256=sha(args.dont_use_list),
                                        masters=args.dont_use_list.read_text().splitlines())
    records, baselines = [], {}
    if args.resume:
        previous = json.loads((args.output / 'manifest.json').read_text())
        for key in ['args', 'binaries', 'dont_use_list']:
            if manifest.get(key) != previous.get(key):
                parser.error(f'cannot resume: {key} changed')
        changed = [key for key in manifest['source_sha256'].keys() | previous['source_sha256'].keys()
                   if manifest['source_sha256'].get(key) != previous['source_sha256'].get(key)]
        # Version orchestration updates separately; never mix solver/evaluator changes.
        if set(changed) - {'scripts/evaluate_buffer_algorithms.py'}:
            parser.error(f'cannot resume: experiment sources changed: {changed}')
        previous.setdefault('resumptions', []).append(dict(
            utc=manifest['start_utc'], gpu=manifest['gpu'],
            orchestrator_sha256=sha(__file__), changed_sources=changed))
        manifest = previous
        saved = json.loads((args.output / 'comparison.json').read_text())
        records, baselines = saved['runs'], saved['baselines']
        for record in records:
            for ext, expected in record['output_sha256'].items():
                if sha(Path(record['directory']) / 'sizer' / f'repaired.{ext}') != expected:
                    parser.error('cannot resume: a validated output changed')
    if cache.is_file() and not args.resume:
        (args.output / 'CMakeCache.txt').write_bytes(cache.read_bytes())
    orchestrators = args.output / 'orchestrators'
    orchestrators.mkdir(exist_ok=True)
    (orchestrators / f'{sha(__file__)}.py').write_bytes(Path(__file__).read_bytes())
    env = dict(os.environ, CUDA_VISIBLE_DEVICES=args.gpu)
    cases = [(a, b) for b in args.budgets for a in args.algorithms]
    for repeat in range(args.repeats):
        order = cases[repeat % len(cases):] + cases[:repeat % len(cases)]
        for algorithm, budget in order:
            completed = {r['design'] for r in records if
                         (r['repeat'], r['algorithm'], r['budget']) == (repeat + 1, algorithm, budget)}
            if completed == set(args.designs):
                continue
            if completed:
                raise RuntimeError('partially validated case in comparison.json')
            dest = args.output.resolve() / f'repeat-{repeat + 1}' / f'{algorithm}-{budget}'
            if dest.exists():
                index = 1
                while dest.with_name(f'{dest.name}-invalid-{index}').exists():
                    index += 1
                archived = dest.with_name(f'{dest.name}-invalid-{index}')
                dest.rename(archived)
                manifest.setdefault('excluded_runs', []).append(dict(
                    directory=str(archived), reason='case did not complete validation; not included in statistics'))
            dest.mkdir(parents=True)
            command = [sys.executable, str(repo / 'scripts/evaluate_fanout.py'),
                       '--benchmark', str(args.benchmark.resolve()), '--equiv-dir', str(args.equiv_dir.resolve()),
                       '--sizer', str(args.sizer.resolve()), '--openroad', str(args.openroad.resolve()),
                       '--output', str(dest), '--designs', *args.designs,
                       '--repair-gbi' if algorithm == 'gbi-cuda' else '--repair-vg']
            if args.dont_use_list:
                command += ['--dont-use-list', str(args.dont_use_list.resolve())]
            settings = {'power.enabled': 'false', 'runtime.sta_threads': args.sta_threads,
                        'vg.max_buffers': budget, 'gbi.max_buffers': budget,
                        'vg.backend': 'cpu' if algorithm == 'vg-cpu' else 'cuda',
                        'vg.cuda_max_memory_mb': args.vg_memory_mb,
                        'vg.max_nets': 0, 'gbi.max_nets': 0,
                        'vg.require_drv_reduction': 'true', 'gbi.require_drv_reduction': 'true'}
            for key, value in settings.items():
                command += ['--repair-set', f'{key}={value}']
            manifest['commands'].append(dict(repeat=repeat + 1, algorithm=algorithm, budget=budget,
                                             command=command, load=os.getloadavg(),
                                             gpu=capture(['nvidia-smi', '--query-gpu=index,utilization.gpu,memory.used', '--format=csv'])))
            (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            print(f'Running repeat {repeat + 1}/{args.repeats}: {algorithm} budget={budget}', flush=True)
            with (dest / 'runner.log').open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            for row in json.loads((dest / 'summary.json').read_text()):
                name, metrics = row['design'], row['metrics']
                if not all(math.isfinite(v) for v in metrics.values()):
                    raise RuntimeError(f'{dest}/{name}: nonfinite metrics')
                for key, expected in [('unplaced', 0), ('supply_pin_errors', 0), ('placement_checked', 1)]:
                    if metrics[key] != expected:
                        raise RuntimeError(f'{dest}/{name}: {key} failed')
                quality = {k: v for k, v in metrics.items() if k != 'repair_ms'}
                baseline = baselines.setdefault(name, dict(fingerprint=row['input_fingerprint'], metrics=quality))
                if row['input_fingerprint'] != baseline['fingerprint']:
                    raise RuntimeError(f'{dest}/{name}: inputs differ')
                if row['mode'] == 'baseline':
                    if quality != baseline['metrics']:
                        raise RuntimeError(f'{dest}/{name}: independently evaluated baselines differ')
                    continue
                topo = row['topology']
                if row['contest_structural_check'] != 'passed':
                    raise RuntimeError('structural check failed')
                if metrics['inserted_sizer_buffers'] != topo['inserted_buffers']:
                    raise RuntimeError(f'{dest}/{name}: insertion count mismatch')
                with (dest / name / 'sizer/node.csv').open() as stream:
                    inserted_masters = Counter(node['Master'] for node in csv.DictReader(stream)
                                               if node['Name'].startswith(('sizer_vg_buf', 'sizer_gbi_buf')))
                if sum(inserted_masters.values()) != topo['inserted_buffers']:
                    raise RuntimeError(f'{dest}/{name}: exported instances differ from insertion count')
                if set(inserted_masters) & set(row['excluded_masters']):
                    raise RuntimeError(f'{dest}/{name}: inserted an excluded master')
                if budget and topo['attempted_buffers'] > budget:
                    raise RuntimeError(f'{dest}/{name}: buffer budget exceeded')
                for key in ['setup_wns_ns', 'setup_tns_ns', 'hold_wns_ns', 'hold_tns_ns']:
                    if metrics[key] + 1e-5 < baseline['metrics'][key]:
                        raise RuntimeError(f'{dest}/{name}: reloaded {key} regressed')
                for key in ['slew_violations', 'cap_violations', 'fanout_violations']:
                    if metrics[key] > baseline['metrics'][key]:
                        raise RuntimeError(f'{dest}/{name}: reloaded {key} regressed')
                log = (dest / name / 'sizer/sizer.log').read_text()
                backend = 'cpu' if algorithm == 'vg-cpu' else 'cuda'
                if f'backend_used={backend}' not in log or ('cuda_fallback="' in log and 'cuda_fallback=""' not in log):
                    raise RuntimeError(f'{dest}/{name}: unexpected backend/fallback')
                row.update(algorithm=algorithm, budget=budget, repeat=repeat + 1,
                           inserted_masters=dict(inserted_masters),
                           directory=str(dest / name), output_sha256={ext: sha(dest / name / 'sizer' / f'repaired.{ext}') for ext in ['def', 'v']})
                records.append(row)
                print(f'  {name}: buffers={topo["inserted_buffers"]:.0f}, WNS={metrics["setup_wns_ns"]:.6f}, total={row["sizer_seconds"]:.3f}s', flush=True)
            comparison = dict(baselines=baselines, aggregate=aggregate(records), runs=records)
            (args.output / 'comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
    manifest['end_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Completed: {args.output}/comparison.json', flush=True)


if __name__ == '__main__':
    main()
