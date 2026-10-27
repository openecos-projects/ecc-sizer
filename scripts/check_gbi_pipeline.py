#!/usr/bin/env python3
"""Check stage handoff, input parity and physical invariants of GBI comparisons."""
import argparse
import json
import math
from pathlib import Path


def check(directory):
    rows = json.loads((directory/'summary.json').read_text())
    errors = []
    manifest = json.loads((directory/'manifest.json').read_text())
    variants = ['native', 'gbi'] + [f'pipeline_{b or "unlimited"}' for b in manifest['budgets']]
    if manifest['completed'] != variants:
        errors.append('comparison suite is incomplete')
    expected_rows = {(variant, design, mode) for variant in variants
                     for design in ['aes_cipher_top', 'jpeg_encoder']
                     for mode in ['baseline', 'sizer']}
    if {(r['variant'], r['design'], r['mode']) for r in rows} != expected_rows:
        errors.append('missing or unexpected comparison rows')
    reference = {}
    index = {}
    metrics_to_match = ['setup_wns_ns', 'setup_tns_ns', 'area_um2', 'total_power_w',
                        'instances', 'nets', 'slew_violations', 'cap_violations',
                        'fanout_violations', 'hold_wns_ns', 'hold_tns_ns']

    def close(a, b):
        return math.isclose(a, b, abs_tol=1e-6, rel_tol=1e-6)

    def equal_metrics(a, b, label):
        for key in metrics_to_match:
            if not close(a[key], b[key]):
                errors.append(f'{label}: {key} differs ({a[key]} vs {b[key]})')

    for row in rows:
        key = (row['variant'], row['design'], row['mode'])
        if key in index:
            errors.append(f'duplicate row {key}')
        index[key] = row
        if row['mode']=='baseline':
            if row['design'] in reference:
                base = reference[row['design']]
                if base['input_fingerprint']!=row['input_fingerprint']:
                    errors.append(f'{key}: input fingerprint differs')
                equal_metrics(base['metrics'], row['metrics'], str(key))
            else:
                reference[row['design']] = row
    for row in rows:
        if row['mode']!='sizer':
            continue
        label = f"{row['variant']}/{row['design']}"
        base = reference[row['design']]
        if base['input_fingerprint']!=row['input_fingerprint']:
            errors.append(f'{label}: input fingerprint differs')
        stages = row['stage_metrics']
        expected = ['input'] + (['gbi'] if row['repair_gbi'] else [])
        expected += (['design'] if row['repair_design'] else [])
        expected += (['setup'] if row['repair_setup'] else [])
        if list(stages)!=expected:
            errors.append(f'{label}: missing or out-of-order stages')
        equal_metrics(base['metrics'], stages['input']['metrics'], label+'/input')
        equal_metrics(row['metrics'], stages[expected[-1]]['metrics'], label+'/final')
        checks = [('final', row)] + list(stages.items())
        for stage, data in checks:
            m = data['metrics']
            if not all(math.isfinite(v) for v in m.values()):
                errors.append(f'{label}/{stage}: nonfinite metric')
            if m['placement_checked']!=1 or m['unplaced']!=0 or m['supply_pin_errors']!=0:
                errors.append(f'{label}/{stage}: placement/supply failure')
            if data.get('contest_structural_check')!='passed':
                errors.append(f'{label}/{stage}: structural checker missing')
        before = stages['input']['metrics']
        for stage in expected[1:]:
            after = stages[stage]['metrics']
            for key in ['setup_wns_ns', 'setup_tns_ns', 'hold_wns_ns', 'hold_tns_ns']:
                if after[key]<before[key] and not close(after[key], before[key]):
                    errors.append(f'{label}/{stage}: {key} regressed')
            for key in ['slew_violations', 'cap_violations', 'fanout_violations']:
                if after[key]>before[key]:
                    errors.append(f'{label}/{stage}: {key} increased')
            before = after
        if row['repair_gbi']:
            n = row['gbi_topology']['inserted_buffers']
            budget = int(row['effective_repair_config']['gbi.max_buffers'])
            if budget and n>budget:
                errors.append(f'{label}: GBI budget exceeded')
            if n!=stages['gbi']['metrics']['instances']-stages['input']['metrics']['instances']:
                errors.append(f'{label}: GBI inserted count differs from netlist')
    # Unlimited standalone GBI and the GBI prefix of the combined flow must
    # produce the same state when all settings and input are identical.
    for design in reference:
        standalone = index.get(('gbi', design, 'sizer'))
        combined = index.get(('pipeline_unlimited', design, 'sizer'))
        if standalone and combined:
            equal_metrics(standalone['metrics'], combined['stage_metrics']['gbi']['metrics'],
                          design+'/standalone-vs-prefix')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    errors = check(args.directory)
    for error in errors:
        print(error)
    print('FAILED' if errors else 'PASSED: input parity, stage handoff, budgets, timing/DRV, placement/supply and structural checks')
    raise SystemExit(bool(errors))


if __name__=='__main__':
    main()
