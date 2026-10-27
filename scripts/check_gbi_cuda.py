#!/usr/bin/env python3
"""Check CPU/CUDA GBI parity from evaluate_fanout.py result directories."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(directory):
    return {r['design']: r for r in json.loads((directory / 'summary.json').read_text())
            if r['mode'] == 'sizer'}


def check(cpu_dir, cuda_dir):
    cpu, cuda = load(cpu_dir), load(cuda_dir)
    if not cpu or cpu.keys() != cuda.keys():
        raise ValueError('CPU/CUDA design sets differ or are empty')
    comparisons = []
    tolerances = dict(setup_wns_ns=1e-6, setup_tns_ns=1e-6,
                      hold_wns_ns=1e-6, hold_tns_ns=1e-6, area_um2=1e-6,
                      total_power_w=1e-9, leakage_power_w=1e-9,
                      internal_power_w=1e-9, switching_power_w=1e-9,
                      instances=0, nets=0, inserted_sizer_buffers=0,
                      slew_violations=0, cap_violations=0, fanout_violations=0)
    for name, c in cpu.items():
        g = cuda[name]
        if c['input_fingerprint'] != g['input_fingerprint']:
            raise ValueError(f'{name}: input DEF/SDC/platform differ')
        configs = []
        for row in (c, g):
            configs.append({k: v for k, v in row['effective_repair_config'].items()
                            if k not in ('gbi.backend', 'gbi.cuda_max_memory_mb')})
            if row.get('contest_structural_check') != 'passed':
                raise ValueError(f'{name}: missing structural check')
            for metric in ('unplaced', 'supply_pin_errors'):
                if row['metrics'][metric] != 0:
                    raise ValueError(f'{name}: {metric} is nonzero')
            if row['metrics']['placement_checked'] != 1:
                raise ValueError(f'{name}: placement not checked')
        if configs[0] != configs[1]:
            raise ValueError(f'{name}: repair settings differ')
        for directory, backend in ((cpu_dir, 'cpu'), (cuda_dir, 'cuda')):
            log = (directory / name / 'sizer' / 'sizer.log').read_text()
            matches = re.findall(r'Sizer GBI repair: .*?backend_used=(\w+) cuda_fallback="([^"]*)"', log)
            if matches != [(backend, '')]:
                raise ValueError(f'{name}: expected {backend} without fallback, got {matches}')
        for metric, tolerance in tolerances.items():
            if not math.isclose(c['metrics'][metric], g['metrics'][metric],
                                rel_tol=0, abs_tol=tolerance):
                raise ValueError(f'{name}: {metric} differs')
        for key in ('nets_collected', 'plans_collected', 'attempted_buffers', 'inserted_buffers', 'committed'):
            if c['gbi_topology'][key] != g['gbi_topology'][key]:
                raise ValueError(f'{name}: {key} differs')
        hashes = {}
        for filename in ('repaired.v', 'repaired.def'):
            ch = sha256(cpu_dir / name / 'sizer' / filename)
            gh = sha256(cuda_dir / name / 'sizer' / filename)
            if ch != gh:
                raise ValueError(f'{name}: {filename} differs byte for byte')
            hashes[filename] = ch
        comparisons.append(dict(
            design=name, parity='passed', input_fingerprint=c['input_fingerprint'],
            output_sha256=hashes, metrics=g['metrics'],
            inserted_buffers=g['gbi_topology']['inserted_buffers'],
            cpu_solve_ms=c['gbi_topology']['solve_ms'],
            cuda_solve_ms=g['gbi_topology']['solve_ms'],
            solve_speedup=c['gbi_topology']['solve_ms']/g['gbi_topology']['solve_ms'],
            cpu_seconds=c['sizer_seconds'], cuda_seconds=g['sizer_seconds'],
            total_speedup=c['sizer_seconds']/g['sizer_seconds']))
    return comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cpu', type=Path)
    parser.add_argument('cuda', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = check(args.cpu, args.cuda)
    text = json.dumps(results, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
