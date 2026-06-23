#!/usr/bin/env python3
"""Static G2 project doctor.

This tool inspects the repository as files on disk only. It reads source text,
checks metadata, and writes a Markdown report. It never imports project modules
and never starts robot, ROS, driver, controller, or GDK runtime code.
"""

import argparse
import ast
import datetime
import os
from pathlib import Path
import re
import sys


SKIP_DIRS = {".git", "__pycache__"}
SKIP_SUFFIXES = {".pyc"}
LOG_SUFFIXES = [".log", ".jsonl", ".json"]
STRUCTURE_TARGETS = [
    "logs/",
    "tools/",
    "reports/",
    "overlays/",
    "rack_hybrid_docking_package/",
    "handoff/",
    ".ai_agents/",
    "AGENTS.md",
    "CLAUDE.md",
    "G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md",
]
VIEWER_FILES = [
    "g2_head_av_viewer.py",
    "g2_head_tunnel_viewer.py",
    "g2_head_webrtc_viewer.py",
]
SAFETY_WORDS = ["estop", "e_stop", "torque", "velocity", "limit", "safety"]
REPORT_NOTICE = "本报告为静态分析, 未运行任何机器人脚本"


def parse_args(argv):
    default_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="Generate a static G2 project diagnostic report."
    )
    parser.add_argument(
        "--root",
        default=str(default_root),
        help="Repository root. Defaults to the parent directory of tools/.",
    )
    parser.add_argument(
        "--output",
        default="reports/g2_project_doctor_report.md",
        help="Markdown report path. Relative paths are resolved under --root.",
    )
    return parser.parse_args(argv)


def resolve_output(root, output):
    output_path = Path(output)
    if output_path.is_absolute():
        return output_path
    return root / output_path


def rel_path(root, path):
    return str(path.relative_to(root)).replace(os.sep, "/")


def iter_project_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(name for name in dirnames if name not in SKIP_DIRS)
        current_dir = Path(dirpath)
        for filename in sorted(filenames):
            path = current_dir / filename
            if path.suffix in SKIP_SUFFIXES:
                continue
            yield path


def read_text(path):
    return path.read_text(encoding="utf-8", errors="replace")


def parse_python(path):
    try:
        tree = ast.parse(read_text(path), filename=str(path))
    except SyntaxError as err:
        return False, f"{err.msg} (line {err.lineno})", None
    except OSError as err:
        return False, str(err), None
    return True, "OK", tree


def format_size(num_bytes):
    units = ["B", "KiB", "MiB", "GiB"]
    value = float(num_bytes)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            if unit == "B":
                return f"{int(value)} {unit}"
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{num_bytes} B"


def format_time(timestamp):
    if timestamp is None:
        return "N/A"
    return datetime.datetime.fromtimestamp(timestamp).astimezone().isoformat(
        timespec="seconds"
    )


def status_icon(exists):
    return "✅ 存在" if exists else "❌ 缺失"


def structure_rows(root):
    rows = []
    for target in STRUCTURE_TARGETS:
        clean_target = target.rstrip("/")
        path = root / clean_target
        rows.append((target, path.exists()))
    return rows


def collect_logs(root):
    logs_dir = root / "logs"
    summary = {}
    zero_size_files = []
    if not logs_dir.exists():
        for suffix in LOG_SUFFIXES:
            summary[suffix] = {"count": 0, "size": 0, "latest": None}
        return logs_dir, summary, zero_size_files

    for suffix in LOG_SUFFIXES:
        files = [
            path
            for path in logs_dir.rglob(f"*{suffix}")
            if path.is_file() and "__pycache__" not in path.parts
        ]
        total_size = 0
        latest = None
        for path in files:
            stat = path.stat()
            total_size += stat.st_size
            if latest is None or stat.st_mtime > latest:
                latest = stat.st_mtime
            if stat.st_size == 0:
                zero_size_files.append(path)
        summary[suffix] = {
            "count": len(files),
            "size": total_size,
            "latest": latest,
        }
    return logs_dir, summary, sorted(set(zero_size_files))


def collect_viewers(root):
    rows = []
    for name in VIEWER_FILES:
        path = root / name
        if not path.exists():
            rows.append(
                {
                    "name": name,
                    "exists": False,
                    "size": "N/A",
                    "syntax": "❌ 缺失",
                }
            )
            continue
        ok, message, _tree = parse_python(path)
        rows.append(
            {
                "name": name,
                "exists": True,
                "size": format_size(path.stat().st_size),
                "syntax": "✅ 语法 OK" if ok else f"❌ 语法错: {message}",
            }
        )
    return rows


def collect_vla_world(root):
    matched = []
    fixture_files = []
    fixtures_dir = root / ".ai_agents" / "fixtures"
    for path in iter_project_files(root):
        if not path.is_file():
            continue
        rel = rel_path(root, path)
        lowered = rel.lower()
        if "vla" in lowered or "world" in lowered:
            matched.append(path)
    if fixtures_dir.exists():
        fixture_files = sorted(
            path
            for path in fixtures_dir.rglob("*.jsonl")
            if path.is_file() and "__pycache__" not in path.parts
        )
    return sorted(set(matched)), fixture_files


def main_guard_test(test_node):
    if not isinstance(test_node, ast.Compare):
        return False
    if len(test_node.ops) != 1 or not isinstance(test_node.ops[0], ast.Eq):
        return False
    if len(test_node.comparators) != 1:
        return False

    left = test_node.left
    right = test_node.comparators[0]

    return (
        is_name_dunder(left)
        and is_main_string(right)
        or is_main_string(left)
        and is_name_dunder(right)
    )


def is_name_dunder(node):
    return isinstance(node, ast.Name) and node.id == "__name__"


def is_main_string(node):
    return isinstance(node, ast.Constant) and node.value == "__main__"


def has_main_guard(tree):
    if tree is None:
        return None
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and main_guard_test(node.test):
            return True
    return False


def collect_motion_scripts(root):
    patterns = [
        "move_*.py",
        "offset_*.py",
        "industrial_*.py",
        "rack_hybrid_docking_package/run_*.py",
        "rack_hybrid_docking_package/rack_*.py",
    ]
    paths = []
    for pattern in patterns:
        paths.extend(path for path in root.glob(pattern) if path.is_file())

    rows = []
    for path in sorted(set(paths)):
        ok, message, tree = parse_python(path)
        main_guard = has_main_guard(tree)
        if main_guard is True:
            main_status = "✅ has __main__ guard"
        elif main_guard is False:
            main_status = "⚠️ no __main__ guard"
        else:
            main_status = f"❌ parse failed: {message}"
        rows.append(
            {
                "path": rel_path(root, path),
                "main": main_status,
                "mark": "MOTION ENTRYPOINT — DO NOT RUN",
            }
        )
    return rows


def collect_safety_files(root):
    rows = []
    word_re = re.compile("|".join(re.escape(word) for word in SAFETY_WORDS), re.I)
    for path in iter_project_files(root):
        if path.suffix != ".py":
            continue
        hits = set()
        hit_lines = 0
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                for line in handle:
                    found = word_re.findall(line)
                    if found:
                        hit_lines += 1
                        hits.update(word.lower() for word in found)
        except OSError:
            continue
        if hits:
            rows.append(
                {
                    "path": rel_path(root, path),
                    "words": ", ".join(sorted(hits)),
                    "lines": hit_lines,
                    "mark": "PROTECTED — DO NOT MODIFY",
                }
            )
    return rows


def warning_and_error_counts(
    structure,
    log_zero_files,
    viewers,
    motion_rows,
):
    warnings = 0
    errors = 0
    errors += sum(1 for _target, exists in structure if not exists)
    warnings += len(log_zero_files)
    errors += sum(1 for row in viewers if row["syntax"].startswith("❌"))
    warnings += sum(1 for row in motion_rows if row["main"].startswith("⚠️"))
    errors += sum(1 for row in motion_rows if row["main"].startswith("❌"))
    return warnings, errors


def append_list(lines, paths, root):
    if not paths:
        lines.append("- 无")
        return
    for path in paths:
        lines.append(f"- `{rel_path(root, path)}` ({format_size(path.stat().st_size)})")


def build_report(root):
    structure = structure_rows(root)
    logs_dir, log_summary, log_zero_files = collect_logs(root)
    viewers = collect_viewers(root)
    vla_world_files, fixture_files = collect_vla_world(root)
    motion_rows = collect_motion_scripts(root)
    safety_rows = collect_safety_files(root)
    warnings, errors = warning_and_error_counts(
        structure, log_zero_files, viewers, motion_rows
    )
    if errors:
        overall = "有 ❌"
    elif warnings:
        overall = "有 ⚠️"
    else:
        overall = "OK"

    lines = [
        "# G2 项目状态静态诊断报告",
        "",
        f"- 生成时间: {datetime.datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"- 声明: {REPORT_NOTICE}",
        f"- 仓库根目录: `{root}`",
        "",
        "## 1. 项目结构概览",
        "",
    ]

    for target, exists in structure:
        lines.append(f"- {status_icon(exists)} `{target}`")

    lines.extend(["", "## 2. 日志检查 (logs/)", ""])
    if logs_dir.exists():
        lines.append(f"- ✅ 存在 `logs/`")
    else:
        lines.append("- ❌ 缺失 `logs/`")
    for suffix in LOG_SUFFIXES:
        item = log_summary[suffix]
        lines.append(
            "- "
            f"`*{suffix}`: count={item['count']}, "
            f"total_size={format_size(item['size'])}, "
            f"latest_mtime={format_time(item['latest'])}"
        )
    lines.append("- ⚠️ 零字节 / 空文件:")
    append_list(lines, log_zero_files, root)

    lines.extend(["", "## 3. Viewer 文件检查", ""])
    for row in viewers:
        exists_text = "✅ 存在" if row["exists"] else "❌ 缺失"
        lines.append(
            f"- `{row['name']}`: {exists_text}, size={row['size']}, {row['syntax']}"
        )

    lines.extend(["", "## 4. VLA / World Model 文件检查", ""])
    lines.append(f"- 文件名含 `vla` 或 `world` 的文件数量: {len(vla_world_files)}")
    append_list(lines, vla_world_files, root)
    lines.append(f"- `.ai_agents/fixtures/` 下 `*.jsonl` 夹具数量: {len(fixture_files)}")
    append_list(lines, fixture_files, root)

    lines.extend(["", "## 5. 机器人运动脚本入口 (只列出, 禁止运行)", ""])
    lines.append(
        "**醒目提示: 以下脚本禁止由本工具或 Codex 运行；本章节只做源码文本清点。**"
    )
    if not motion_rows:
        lines.append("- ❌ 未找到匹配的运动脚本入口")
    else:
        for row in motion_rows:
            lines.append(f"- `{row['path']}`: {row['main']} | {row['mark']}")

    lines.extend(["", "## 6. 安全相关文件 (受保护, 只读)", ""])
    if not safety_rows:
        lines.append("- 无关键字命中文件")
    else:
        for row in safety_rows:
            lines.append(
                f"- `{row['path']}`: keywords={row['words']}, "
                f"matched_lines={row['lines']} | {row['mark']}"
            )

    lines.extend(["", "## 7. 诊断结论 / 摘要", ""])
    lines.append(f"- 整体状态: {overall}")
    lines.append(f"- 项目结构检查项: {len(structure)}")
    lines.append(
        "- 日志文件计数: "
        + ", ".join(
            f"*{suffix}={log_summary[suffix]['count']}" for suffix in LOG_SUFFIXES
        )
    )
    lines.append(f"- Viewer 文件数: {len(viewers)}")
    lines.append(f"- VLA / World Model 文件数: {len(vla_world_files)}")
    lines.append(f"- VLA dry-run JSONL 夹具数: {len(fixture_files)}")
    lines.append(f"- 机器人运动脚本入口数: {len(motion_rows)}")
    lines.append(f"- 安全关键字命中文件数: {len(safety_rows)}")
    lines.append(f"- Warning 数: {warnings}")
    lines.append(f"- Error 数: {errors}")
    lines.append(f"- 安全声明: {REPORT_NOTICE}")
    lines.append("- 未导入项目运动、docking、controller 模块；未启动任何运行时。")
    lines.append("")
    return "\n".join(lines)


def main(argv=None):
    args = parse_args(argv)
    root = Path(args.root).resolve()
    output_path = resolve_output(root, args.output)
    try:
        report = build_report(root)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(report, encoding="utf-8")
    except OSError as err:
        print(f"g2_project_doctor: failed to write report: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
