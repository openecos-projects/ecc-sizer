#!/usr/bin/env python3
"""Evaluate ECC/ICS55 placement snapshots with the shared Sizer benchmark runner."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys

import benchmark_ispd26 as bench

ROOT = bench.ROOT
PROFILES = ['baseline', 'drv-setup', 'vg', 'gobi', 'path']


def discover(dataset):
    cases = {}
    digests = {}
    for snapshot in sorted(dataset.glob('*/snapshot_manifest.json')):
        manifest = json.loads(snapshot.read_text())
        for name, record in sorted(manifest['designs'].items()):
            directory = snapshot.parent / 'workspaces' / name / 'artifacts/ecc'
            source = directory / 'post_pl/design.def'
            metadata = directory / 'post_rt/stage_metadata.json'
            config = json.loads(metadata.read_text())['effective_flow_config']
            files = [directory / 'post_pl' / filename for filename in record['files']]
            files += [metadata, Path(config['tlef_path'])]
            files += [Path(p) for key in ('lef_paths', 'lib_paths', 'macro_lef_paths', 'macro_lib_paths')
                      for p in config.get(key, [])]
            for path in files:
                if path not in digests:
                    digests[path] = bench.digest(path)
            hashes = {str(p): digests[p] for p in files}
            for filename, expected in record['files'].items():
                if hashes[str(directory / 'post_pl' / filename)] != expected:
                    raise ValueError(f'Input differs from snapshot: {source.parent / filename}')
            key = f'{snapshot.parent.name}__{name}'
            cases[key] = dict(batch=snapshot.parent.name, name=name, source=str(source),
                              metadata=str(metadata), hashes=hashes, config=config)
    if not cases:
        raise ValueError(f'No ECC snapshots in {dataset}')
    return cases


def design_loader(cases):
    def load(_benchmark, design, input_def, threads, _dont_use):
        case = cases[design]
        config = case['config']
        lines = [f'read_lef {bench.tcl(config["tlef_path"])}']
        lines += [f'read_lef {bench.tcl(p)}' for key in ('lef_paths', 'macro_lef_paths') for p in config.get(key, [])]
        lines += [f'read_liberty {bench.tcl(p)}' for key in ('lib_paths', 'macro_lib_paths') for p in config.get(key, [])]
        lines += [f'read_def {bench.tcl(input_def)}',
                  f'read_sdc {bench.tcl(Path(case["source"]).with_name("constraints.sdc"))}',
                  'set_ideal_network [all_clocks]', f'set_thread_count {threads}',
                  'set_wire_rc -signal -layer MET2', 'set_wire_rc -clock -layer MET2',
                  'set benchmark_forbidden_masters [dict create]']
        patterns = config['ecc']['dont_use']
        buffers = config['ecc']['buffers']
        if not 1 <= len(buffers) <= 32:
            raise ValueError(f'Expected 1..32 configured buffer types: {design}')
        lines += ['set benchmark_dont_use [list ' + ' '.join(bench.tcl(p) for p in patterns) + ']',
                  'set benchmark_buffers [list ' + ' '.join(bench.tcl(p) for p in buffers) + ']',
                  'foreach cell [get_lib_cells *] {',
                  '    set name [$cell name]',
                  '    set forbidden [expr {[$cell is_buffer] && $name ni $benchmark_buffers}]',
                  '    foreach pattern $benchmark_dont_use {',
                  '        if {[string match $pattern $name]} { set forbidden 1 }',
                  '    }',
                  '    if {$forbidden} {',
                  '        set_dont_use $cell',
                  '        dict set benchmark_forbidden_masters $name 1',
                  '    }',
                  '}',
                  'foreach name $benchmark_buffers {',
                  '    set cells [get_lib_cells -quiet $name]',
                  '    if {[llength $cells] != 1 || ![[lindex $cells 0] is_buffer] || [dict exists $benchmark_forbidden_masters $name]} {',
                  '        error "Invalid configured buffer: $name"',
                  '    }',
                  '}']
        # Prepare supply only for the source input. Independent output evaluation
        # must detect missing connections rather than repair them before checking.
        if Path(input_def) == Path(case['source']):
            lines += ['add_global_connection -net VDD -inst_pattern .* -pin_pattern ^VDD$ -power',
                      'add_global_connection -net VSS -inst_pattern .* -pin_pattern ^VSS$ -ground',
                      'global_connect']
        return '\n'.join(lines) + '\n'
    return load


def reports(output, manifest, update_docs, show=True):
    designs = {key: f'{case["batch"]}/{case["name"]}' for key, case in manifest['cases'].items()}
    bench.write_reports(output, show=show, designs=designs)
    records = bench.load_results(output)
    expected = {(d, p) for d in designs for p in manifest['profiles']}
    complete = set(records) == expected
    passed = sum(r['ok'] for r in records.values())
    if update_docs:
        doc = ROOT/'docs/benchmark.md'
        text = doc.read_text()
        date = manifest['date']
        lines = [f'## ECC / ICS55（{date}）', '',
                 f'本轮评测 {len(designs)} 个 post-placement 快照，分别运行初始、drv+setup、VG、GOBI、Path。已完成 {len(records)}/{len(expected)} 项，独立验证通过 {passed} 项。', '',
                 f'采用各快照元数据指定的 TT 1.2 V / 25°C Liberty 与 cworst 技术 LEF；使用 MET2 的 LEF RC 做 placement 寄生估计，不是布线后提取或签核结果。保留原 SDC、理想时钟和工艺 dont-use；最大 fanout 为 32，STA 使用 {manifest["identity"]["threads"]} 线程。Path 使用与本页 ASAP7 评测相同的阶段末验证、增量分析及续搜策略，其余流程采用默认策略；不启用 VT 后缀映射。各流程统一使用工艺元数据指定的 BUFX8H7L、BUFX12H7L、BUFX16H7L、BUFX20H7L 四种 buffer，以满足当前 GOBI 最多 32 种 buffer 的限制。', '']
        for batch in sorted({c['batch'] for c in manifest['cases'].values()}):
            names = {key: c['name'] for key, c in manifest['cases'].items() if c['batch'] == batch}
            lines += [f'### {batch}', '', bench.comparison_markdown(records, names).strip(), '']
            # Keep reviewed conclusions when refreshing the generated data tables.
            start_marker, end_marker = f'<!-- analysis:{batch} -->', f'<!-- /analysis:{batch} -->'
            if start_marker in text and end_marker in text:
                start = text.index(start_marker)
                end = text.index(end_marker, start) + len(end_marker)
                lines += [text[start:end], '']
        failures = [f'{d}/{p}' for (d, p), r in records.items() if not r['ok']]
        if failures:
            lines += ['未通过项（不作为有效结果）：' + '、'.join(failures) + '。', '']
        relative = os.path.relpath(output, ROOT / 'docs')
        lines += [f'原始记录：[{output.name}]({relative}/)。GOBI 的完整指标见结果目录的 `tables.md`。', '',
                  '复现命令：', '', '```sh',
                  f'python3 scripts/benchmark_ecc.py --output results/{output.name} --update-docs', '```']
        block = '<!-- ecc-ics55-benchmark -->\n' + '\n'.join(lines) + '\n<!-- /ecc-ics55-benchmark -->'
        if '<!-- ecc-ics55-benchmark -->' in text:
            start = text.index('<!-- ecc-ics55-benchmark -->')
            end = text.index('<!-- /ecc-ics55-benchmark -->', start) + len('<!-- /ecc-ics55-benchmark -->')
            text = text[:start] + block + text[end:]
        else:
            text = text.rstrip() + '\n\n' + block + '\n'
        doc.write_text(text)
    return complete and passed == len(expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, default=ROOT/'benchmark/ecc_ics55_inputs')
    parser.add_argument('--sizer', type=Path, default=ROOT/'build/src/Sizer')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--batches', nargs='+')
    parser.add_argument('--designs', nargs='+')
    parser.add_argument('--profiles', nargs='+', choices=PROFILES, default=PROFILES)
    parser.add_argument('--threads', type=int, default=32)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--report-only', action='store_true')
    parser.add_argument('--update-docs', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.report_only:
        manifest = json.loads((args.output/'manifest.json').read_text())
        return not reports(args.output, manifest, args.update_docs)
    if args.threads < 1 or not 0 < args.timeout < float('inf'):
        parser.error('threads and timeout must be positive and finite')
    args.benchmark, args.sizer = args.benchmark.resolve(), args.sizer.resolve()
    cases = discover(args.benchmark)
    for selected, available, label in ((args.batches, {c['batch'] for c in cases.values()}, 'batches'),
                                       (args.designs, {c['name'] for c in cases.values()}, 'designs')):
        if selected and set(selected) - available:
            parser.error(f'Unknown {label}: {set(selected) - available}')
    cases = {k: c for k, c in cases.items() if (not args.batches or c['batch'] in args.batches)
             and (not args.designs or c['name'] in args.designs)}
    if not cases:
        parser.error('No matching cases')
    # Keep the SDC fanout limit and repair constraint consistent.
    args.set, args.dont_use = ['common.max_fanout=32'], []
    identity = dict(binary_sha256=bench.digest(args.sizer), runner_sha256=bench.digest(Path(__file__)),
                    shared_runner_sha256=bench.digest(Path(bench.__file__)), threads=args.threads,
                    benchmark=str(args.benchmark), overrides=args.set,
                    cuda_visible_devices=os.environ.get('CUDA_VISIBLE_DEVICES', 'all'))
    manifest = dict(identity=identity, date=datetime.now().strftime('%Y-%m-%d'),
                    git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    cases=cases, profiles={p: bench.PROFILES[p] for p in args.profiles})
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_file = args.output/'manifest.json'
    if manifest_file.exists() and json.loads(manifest_file.read_text()) != manifest:
        raise ValueError('Use a new output directory for a different experiment')
    manifest_file.write_text(json.dumps(manifest, indent=2) + '\n')
    (args.output/'source.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT))
    for script in (Path(__file__), Path(bench.__file__)):
        (args.output/script.name).write_bytes(script.read_bytes())
    results = []
    def status(phase, current=None):
        temporary = args.output/'status.tmp'
        temporary.write_text(json.dumps(dict(phase=phase, current=current,
            completed=len(results), passed=sum(r['ok'] for r in results), total=len(cases)*len(args.profiles),
            updated_utc=datetime.now(timezone.utc).isoformat()), indent=2) + '\n')
        temporary.replace(args.output/'status.json')
    load = design_loader(cases)
    for design, case in cases.items():
        for profile in args.profiles:
            status('running', f'{design}/{profile}')
            result = bench.run_case(args, design, profile, identity, input_def=Path(case['source']), load_design=load)
            if result is not None:
                results.append(result)
                (args.output/'results.json').write_text(json.dumps(results, indent=2) + '\n')
        if not args.dry_run:
            reports(args.output, manifest, args.update_docs, show=False)
    if args.dry_run:
        status('dry-run')
        return 0
    success = reports(args.output, manifest, args.update_docs)
    status('complete' if success else 'completed_with_failures')
    return not success


if __name__ == '__main__':
    raise SystemExit(main())
