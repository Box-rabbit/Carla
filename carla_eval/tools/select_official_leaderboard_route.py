"""Write a one-route Leaderboard 2.0 XML without changing the source file."""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="routes/dongfeng_leaderboard_2.0.xml")
    parser.add_argument("--route-id", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    source_root = ET.parse(args.input).getroot()
    matches = [
        route for route in source_root.findall("route")
        if route.get("id") == args.route_id
    ]
    if len(matches) != 1:
        raise SystemExit(
            f"expected exactly one route id {args.route_id!r}, found {len(matches)}"
        )

    output_root = ET.Element("routes")
    output_root.append(copy.deepcopy(matches[0]))
    try:
        ET.indent(output_root, space="  ")
    except AttributeError:
        pass

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        + ET.tostring(output_root, encoding="unicode")
        + "\n",
        encoding="utf-8",
    )
    print(f"selected route {args.route_id} -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
