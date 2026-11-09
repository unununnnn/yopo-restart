"""python -m yopo_l1 export|import|report …"""

from __future__ import annotations

import sys


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print("用法：python -m yopo_l1 export|import|report …\n子命令的 -h 看各自参数。")
        return 0 if argv and argv[0] in ("-h", "--help") else 1
    cmd, rest = argv[0], argv[1:]
    if cmd == "export":
        from .export import main as run
    elif cmd == "import":
        from .importer import main as run
    elif cmd == "report":
        from .report import main as run
    else:
        print(f"未知子命令 {cmd}", file=sys.stderr)
        return 1
    return run(rest)


if __name__ == "__main__":
    raise SystemExit(main())
