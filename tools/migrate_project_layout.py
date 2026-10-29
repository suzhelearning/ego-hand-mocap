#!/usr/bin/env python3
"""Inventory or apply the copy-only legacy layout migration."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from joint_capture.migration import apply_migration_plan, build_migration_plan
from joint_capture.paths import checked_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--migration-id", default="layout-20261029-v1")
    parser.add_argument("--apply", action="store_true", help="复制并校验数据；默认只生成清单")
    args = parser.parse_args()
    root = args.root.resolve()
    checked_id(args.migration_id, "migration-id")
    output = root / "reports" / "migrations" / args.migration_id
    output.mkdir(parents=True, exist_ok=True)
    plan = build_migration_plan(root, args.migration_id)
    (output / "inventory.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    (output / "plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    if not args.apply:
        print(json.dumps({"migration_id": args.migration_id, "files": len(plan["entries"]), "bytes": plan["total_bytes"], "plan": str(output / "plan.json")}, ensure_ascii=False, indent=2))
        return 0
    result = apply_migration_plan(root, plan)
    print(json.dumps({"migration_id": args.migration_id, "copied": sum(x["status"] == "copied" for x in result["applied"]), "already_identical": sum(x["status"] == "already_identical" for x in result["applied"]), "conflicts": len(result["conflicts"]), "report": str(output / "results.json")}, ensure_ascii=False, indent=2))
    return 0 if not result["conflicts"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
