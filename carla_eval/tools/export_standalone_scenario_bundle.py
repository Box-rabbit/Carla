"""Export one scenario into a standalone delivery bundle.

The bundle is intentionally file-oriented:
- one route XML
- one scenario YAML
- one annotation YAML/JSON
- one voice-match YAML filtered to the target scenario
- one manifest YAML

This is useful for scenarios that currently live inside shared benchmark files
and need to be delivered as self-contained packages.
"""

from __future__ import annotations

import argparse
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


DEFAULT_ROUTES = Path("routes/dongfeng_benchmark.xml")
DEFAULT_SCENARIOS = Path("configs/scenario_annotations/dongfeng_benchmark.yaml")
DEFAULT_VOICE_MATCHES = Path("configs/lmdrive/route_audio_matches.yaml")
DEFAULT_OUTPUT_ROOT = Path("scenario_bundles")


def _load_structured_file(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        if path.suffix.lower() == ".json":
            return json.load(f)
        return yaml.safe_load(f) or {}


def _find_annotation(
    annotations_data: Dict[str, Any],
    scenario_id: str,
) -> Tuple[str, Dict[str, Any]]:
    target = str(scenario_id)
    for town_block in annotations_data.get("available_scenarios", []):
        for town, scenario_list in town_block.items():
            for item in scenario_list:
                item_scenario_id = str(item.get("scenario_id", item.get("route_id", "")))
                item_route_id = str(item.get("route_id", item_scenario_id))
                if target in {item_scenario_id, item_route_id}:
                    return str(town), item
    raise KeyError(f"Scenario not found in annotations: {scenario_id}")


def _find_route_element(routes_file: Path, route_id: str) -> ET.Element:
    root = ET.parse(routes_file).getroot()
    for route_elem in root.findall("route"):
        if str(route_elem.get("id", "")) == str(route_id):
            return copy.deepcopy(route_elem)
    raise KeyError(f"Route id not found in routes file: {route_id}")


def _write_xml_document(route_elem: ET.Element, output_path: Path) -> None:
    root = ET.Element("routes")
    root.append(copy.deepcopy(route_elem))
    ET.indent(root, space="  ")
    xml_text = ET.tostring(root, encoding="unicode")
    output_path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n' + xml_text + "\n",
        encoding="utf-8",
    )


def _detect_category(config_path: Path, cfg: Dict[str, Any], route_elem: ET.Element) -> str:
    category = str(cfg.get("category", "")).strip()
    if category:
        return category

    xml_category = str(route_elem.get("category", "")).strip()
    if xml_category:
        return xml_category

    parent_name = config_path.parent.name.strip()
    return parent_name or "uncategorized"


def _candidate_route_file(category: str, scenario_id: str) -> Path:
    return Path("routes") / category / f"{scenario_id}.xml"


def _write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")


def _route_mode(cfg: Dict[str, Any]) -> str:
    return str(cfg.get("route", {}).get("mode", "xml")).strip() or "xml"


def _build_exported_scenario_config(
    cfg: Dict[str, Any],
    *,
    route_id: str,
    route_xml_rel: str,
) -> Dict[str, Any]:
    exported = copy.deepcopy(cfg)
    route_cfg = exported.setdefault("route", {})
    source_mode = str(route_cfg.get("mode", "")).strip()

    if source_mode in {"carla_lane_trace", "carla_runtime_planner"}:
        route_cfg["source_mode"] = source_mode
        if "lane_trace" in route_cfg:
            route_cfg["source_lane_trace"] = copy.deepcopy(route_cfg["lane_trace"])
            route_cfg.pop("lane_trace", None)
        if "planner" in route_cfg:
            route_cfg["source_planner"] = copy.deepcopy(route_cfg["planner"])
            route_cfg.pop("planner", None)
        route_cfg.pop("mode", None)

    route_cfg["route_file"] = route_xml_rel
    route_cfg["route_id"] = route_id
    return exported


def _collect_voice_matches(
    voice_matches_data: Dict[str, Any],
    scenario_id: str,
    route_id: str,
    route_file_rel: str,
    scenario_config_rel: str,
    annotations_rel: str,
) -> Dict[str, Any]:
    matches = []
    for item in voice_matches_data.get("matches", []):
        item_route_id = str(item.get("route_id", ""))
        item_scenario_id = str(item.get("scenario_id", item_route_id))
        if scenario_id not in {item_route_id, item_scenario_id} and route_id not in {
            item_route_id,
            item_scenario_id,
        }:
            continue

        copied = copy.deepcopy(item)
        copied["route_file"] = route_file_rel
        copied["scenario_config"] = scenario_config_rel
        matches.append(copied)

    payload = {
        "generated_from": str(DEFAULT_VOICE_MATCHES),
        "scenario_annotations_file": annotations_rel,
        "prefer_route": route_id,
        "prefer_route_hard": True,
        "matches": matches,
    }
    if "voice_lead_distance_m" in voice_matches_data:
        payload["voice_lead_distance_m"] = voice_matches_data["voice_lead_distance_m"]
    return payload


def _build_annotation_bundle(
    town: str,
    annotation_item: Dict[str, Any],
    scenario_config_rel: str,
) -> Dict[str, Any]:
    exported = copy.deepcopy(annotation_item)
    exported["config_path"] = scenario_config_rel
    return {
        "description": "Standalone scenario annotation bundle exported from the shared benchmark file.",
        "available_scenarios": [
            {
                town: [exported],
            }
        ],
    }


def _build_manifest(
    *,
    scenario_id: str,
    route_id: str,
    town: str,
    category: str,
    config_path: Path,
    cfg: Dict[str, Any],
    route_elem: ET.Element,
    route_file_rel: str,
    scenario_config_rel: str,
    annotations_rel: str,
    voice_matches_rel: str,
    annotations_format: str,
    source_routes_file: Path,
    source_annotations_file: Path,
    source_voice_matches_file: Path,
) -> Dict[str, Any]:
    route_mode = _route_mode(cfg)
    waypoint_count = len(route_elem.findall("waypoint"))
    runtime = cfg.get("runtime", {})
    route_cfg = cfg.get("route", {})

    notes = []
    if route_mode in {"carla_lane_trace", "carla_runtime_planner"}:
        notes.append(
            "This scenario still uses runtime route generation in its source scenario YAML."
        )
        notes.append(
            "The exported XML is a standalone route snapshot and should be replaced with a dense, validated route before external delivery."
        )
    if waypoint_count <= 2:
        notes.append(
            "The exported route XML is sparse; validate it against CARLA GlobalRoutePlanner before using it as the only route source."
        )

    manifest: Dict[str, Any] = {
        "scenario_id": scenario_id,
        "route_id": route_id,
        "town": town,
        "category": category,
        "carla_version": runtime.get("carla_version"),
        "source": {
            "routes_file": str(source_routes_file),
            "annotations_file": str(source_annotations_file),
            "voice_matches_file": str(source_voice_matches_file),
            "scenario_config": str(config_path),
        },
        "files": {
            "route_xml": route_file_rel,
            "scenario_config": scenario_config_rel,
            "annotations": annotations_rel,
            "voice_matches": voice_matches_rel,
        },
        "route": {
            "source_mode": route_mode,
            "exported_mode": "route_xml",
            "waypoint_count": waypoint_count,
            "town": route_elem.get("town", town),
            "category": route_elem.get("category", category),
            "target_length_m": route_cfg.get("lane_trace", {}).get("target_length_m"),
        },
        "annotations": {
            "format": annotations_format,
        },
    }
    if notes:
        manifest["notes"] = notes
    return manifest


def export_bundle(
    *,
    scenario_id: str,
    routes_file: Path,
    scenarios_file: Path,
    voice_matches_file: Path,
    output_root: Path,
    annotations_format: str,
) -> Path:
    annotations_data = _load_structured_file(scenarios_file)
    voice_matches_data = _load_structured_file(voice_matches_file)
    town, annotation_item = _find_annotation(annotations_data, scenario_id)

    route_id = str(annotation_item.get("route_id", annotation_item.get("scenario_id", scenario_id)))
    config_path = Path(annotation_item["config_path"])
    cfg = _load_structured_file(config_path)
    route_elem = _find_route_element(routes_file, route_id)
    category = _detect_category(config_path, cfg, route_elem)

    bundle_root = output_root / scenario_id
    config_out_dir = bundle_root / "configs"
    route_out_dir = bundle_root / "routes"
    config_out_dir.mkdir(parents=True, exist_ok=True)
    route_out_dir.mkdir(parents=True, exist_ok=True)

    scenario_config_rel = f"configs/{scenario_id}.yaml"
    route_xml_rel = f"routes/{scenario_id}.xml"
    annotations_name = f"{scenario_id}.annotations.{annotations_format}"
    annotations_rel = f"configs/{annotations_name}"
    voice_matches_name = f"route_audio_matches_{scenario_id}.yaml"
    voice_matches_rel = f"configs/{voice_matches_name}"

    exported_cfg = _build_exported_scenario_config(
        cfg,
        route_id=route_id,
        route_xml_rel=route_xml_rel,
    )
    _write_yaml(bundle_root / scenario_config_rel, exported_cfg)
    _write_xml_document(route_elem, bundle_root / route_xml_rel)

    annotation_bundle = _build_annotation_bundle(
        town=town,
        annotation_item=annotation_item,
        scenario_config_rel=scenario_config_rel,
    )
    if annotations_format == "json":
        _write_json(bundle_root / annotations_rel, annotation_bundle)
    else:
        _write_yaml(bundle_root / annotations_rel, annotation_bundle)

    voice_bundle = _collect_voice_matches(
        voice_matches_data=voice_matches_data,
        scenario_id=scenario_id,
        route_id=route_id,
        route_file_rel=route_xml_rel,
        scenario_config_rel=scenario_config_rel,
        annotations_rel=annotations_rel,
    )
    _write_yaml(bundle_root / voice_matches_rel, voice_bundle)

    manifest = _build_manifest(
        scenario_id=scenario_id,
        route_id=route_id,
        town=town,
        category=category,
        config_path=config_path,
        cfg=cfg,
        route_elem=route_elem,
        route_file_rel=route_xml_rel,
        scenario_config_rel=scenario_config_rel,
        annotations_rel=annotations_rel,
        voice_matches_rel=voice_matches_rel,
        annotations_format=annotations_format,
        source_routes_file=routes_file,
        source_annotations_file=scenarios_file,
        source_voice_matches_file=voice_matches_file,
    )
    _write_yaml(bundle_root / "configs/manifest.yaml", manifest)

    canonical_route_file = _candidate_route_file(category, scenario_id)
    if not canonical_route_file.exists():
        canonical_route_file.parent.mkdir(parents=True, exist_ok=True)
        _write_xml_document(route_elem, canonical_route_file)

    return bundle_root


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Export one scenario into a standalone XML/YAML bundle.")
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--routes", default=str(DEFAULT_ROUTES))
    parser.add_argument("--scenarios", default=str(DEFAULT_SCENARIOS))
    parser.add_argument("--voice-match-config", default=str(DEFAULT_VOICE_MATCHES))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--annotations-format", choices=("yaml", "json"), default="yaml")
    args = parser.parse_args(argv)

    bundle_root = export_bundle(
        scenario_id=str(args.scenario_id),
        routes_file=Path(args.routes),
        scenarios_file=Path(args.scenarios),
        voice_matches_file=Path(args.voice_match_config),
        output_root=Path(args.output_root),
        annotations_format=str(args.annotations_format),
    )
    print(bundle_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
