#!/usr/bin/env python3
"""Print current ISPD26 result.json records and generate Markdown report tables."""
import argparse
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import unicodedata

DESIGNS = {
    "aes_cipher_top": "AES",
    "aes_cipher_top_v2": "AES v2",
    "jpeg_encoder": "JPEG",
    "jpeg_encoder_v2": "JPEG v2",
    "ariane": "Ariane",
    "ariane_v2": "Ariane v2",
}
LABELS = {"baseline": "初始", "drv-setup": "Native", "vg": "VG CPU",
          "gobi": "GOBI CUDA", "path": "Path", "path-drv-setup": "Path→DRV→Setup"}
DRV = [f"{name}_violations" for name in ("slew", "cap", "fanout")]


def display_width(value):
    return sum(0 if unicodedata.combining(c) else
               2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in value)


def print_table(header, groups):
    rows = [header] + [row for group in groups for row in group]
    widths = [max(display_width(row[i]) for row in rows) for i in range(len(header))]

    def border(left, middle, right, fill):
        return left + middle.join(fill * (width + 2) for width in widths) + right

    def line(row):
        cells = []
        for i, (value, width) in enumerate(zip(row, widths)):
            padding = " " * (width - display_width(value))
            cells.append(" " + (value + padding if i < 2 else padding + value) + " ")
        return "│" + "│".join(cells) + "│"

    print(border("┌", "┬", "┐", "─"))
    print(line(header))
    print(border("╞", "╪", "╡", "═"))
    for index, group in enumerate(groups):
        if index:
            print(border("├", "┼", "┤", "─"))
        for row in group:
            print(line(row))
    print(border("└", "┴", "┘", "─"))


def improvement(before, after, key, digits=2):
    initial, final = before[key], after[key]
    if key in ("setup_wns_ns", "setup_tns_ns"):
        initial, final = abs(min(initial, 0)), abs(min(final, 0))
    if initial == 0:
        return "—"
    percent = 100 * (initial - final) / initial
    if abs(percent) < 0.5 * 10 ** -digits:
        percent = 0
    return f"{percent:.{digits}f}%"


def load_results(directory):
    records = {}
    for path in sorted(directory.glob("*/*/result.json")):
        record = json.loads(path.read_text())
        key = record["design"], record["profile"]
        if key in records:
            raise ValueError(f"Duplicate result: {key}")
        records[key] = record
    if not records:
        raise ValueError(f"No */*/result.json in {directory}")
    identities = [r["identity"] for r in records.values()]
    if any(identity != identities[0] for identity in identities):
        raise ValueError("Results use different binaries or benchmark settings")
    return records


def valid(records, design, profile):
    record = records.get((design, profile))
    return record if record and record.get("ok") else None


def metric_values(metrics):
    return [f"{metrics['setup_wns_ns']:.4f}", f"{metrics['setup_tns_ns']:.2f}",
            " / ".join(str(int(metrics[k])) for k in DRV),
            f"{metrics['area_um2']:.2f}", f"{metrics['total_power_w'] * 1000:.3f}"]


def stage_seconds(record):
    stages = record.get("stages", {})
    return sum(stage["seconds"] for stage in stages.values()) if stages else None


def detail_table(records, profiles, title, path_gain=False):
    print("\n" + title)
    header = ["设计", "状态", "WNS (ns)", "TNS (ns)", "电气违例 S / C / F",
              "面积 (µm²)", "功耗 (mW)", "新增 buffer"]
    solver = profiles == ["vg", "gobi"]
    if solver:
        header.append("求解耗时 (s)")
    header.extend(["算法阶段 (s)", "完整进程 (s)"])
    groups = []
    for design, name in DESIGNS.items():
        initial = valid(records, design, "baseline")
        available = [p for p in profiles if (design, p) in records]
        if not available:
            continue
        group = []
        if initial:
            group.append([name, "初始"] + metric_values(initial["metrics"]) + ["0"] +
                         (["—"] if solver else []) + ["—", "—"])
        for profile in available:
            record = valid(records, design, profile)
            label = "完成" if path_gain else LABELS[profile]
            if not record:
                group.append([name if not group else "", label + " 失败"] + ["—"] * (len(header) - 2))
                continue
            row = [name if not group else "", label] + metric_values(record["metrics"])
            row += [str(int(record["metrics"]["inserted_buffers"]))]
            total = f"{record['sizer']['seconds']:.2f}"
            if solver:
                solve = record.get("solver", {}).get("solve_seconds")
                text = f"{solve:.3f}" if solve is not None else "—"
                cpu = valid(records, design, "vg")
                if profile == "gobi" and cpu:
                    cpu_solve = cpu.get("solver", {}).get("solve_seconds")
                    if solve and cpu_solve is not None:
                        text += f" ({cpu_solve / solve:.2f}×)"
                    if record['sizer']['seconds'] > 0:
                        total += f" ({cpu['sizer']['seconds'] / record['sizer']['seconds']:.2f}×)"
                row.append(text)
            stage = stage_seconds(record)
            group.append(row + [f"{stage:.3f}" if stage is not None else "—", total])
        final = valid(records, design, "path")
        if path_gain and initial and final:
            before, after = initial["metrics"], final["metrics"]
            group.append(["", "提升百分比", improvement(before, after, "setup_wns_ns"),
                          improvement(before, after, "setup_tns_ns"),
                          " / ".join(improvement(before, after, key) for key in DRV),
                          improvement(before, after, "area_um2", 4),
                          improvement(before, after, "total_power_w", 4), "—", "—", "—"])
        groups.append(group)
    print_table(header, groups)


def comparison_markdown(records):
    rows = ["| 设计 | 初始输入 | Native | VG | Path |", "|---|---|---|---|---|"]
    for design, name in DESIGNS.items():
        if not any(key[0] == design for key in records):
            continue
        initial = valid(records, design, "baseline")
        vg = valid(records, design, "vg")
        cells = [name]
        for profile in ("baseline", "drv-setup", "vg", "path"):
            record = valid(records, design, profile)
            if not record:
                cells.append("失败" if (design, profile) in records else "未评测")
                continue
            metrics = record["metrics"]
            lines = []
            for key, label, digits in (("setup_wns_ns", "WNS", 6), ("setup_tns_ns", "TNS", 3)):
                value = f"{label} {metrics[key]:.{digits}f} ns"
                if profile != "baseline" and initial:
                    value += f" ({improvement(initial['metrics'], metrics, key)})"
                lines.append(value)
            stage = stage_seconds(record)
            times = (["时间 —"] if profile == "baseline" else
                     ([f"算法时间 {stage:.2f} s"] if stage is not None else []) +
                     [f"进程时间 {record['sizer']['seconds']:.2f} s"])
            buffers = f"buffer {int(metrics['inserted_buffers'])}"
            if profile == "path" and vg:
                buffers += f" ({int(metrics['inserted_buffers'] - vg['metrics']['inserted_buffers']):+d})"
            cells.append("<br>".join(lines + times + [buffers]))
        rows.append("| " + " | ".join(cells) + " |")
    return "\n".join(rows) + "\n"


def write_reports(directory, show=False):
    records = load_results(directory)
    sections = []
    for profiles, title, path_gain in [
        (["vg", "gobi"], "GPU 加速缓冲器插入：初始 / VG CPU / GOBI CUDA", False),
        (["path"], "基于路径的缓冲器插入：初始 / 完成 / 提升百分比", True),
        (["drv-setup"], "DRV + Setup（Native）", False),
    ]:
        with redirect_stdout(io.StringIO()) as stream:
            detail_table(records, profiles, title, path_gain)
        sections.append(stream.getvalue())
    console = "\n".join(sections)
    (directory / "tables.txt").write_text(console)
    markdown = comparison_markdown(records)
    (directory / "comparison.md").write_text(markdown)
    (directory / "tables.md").write_text("\n\n".join(
        section.strip().split("\n", 1)[0] + "\n\n```text\n" + section.strip().split("\n", 1)[1] + "\n```"
        for section in sections) + "\n")
    if show:
        print(console)
    return markdown


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, help="benchmark_ispd26.py output directory")
    parser.add_argument("--markdown", action="store_true", help="Print the Native/VG/Path comparison")
    args = parser.parse_args()
    try:
        markdown = write_reports(args.results, show=not args.markdown)
        if args.markdown:
            print(markdown, end="")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"读取评测结果失败：{error}\n")


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.stdout = None
