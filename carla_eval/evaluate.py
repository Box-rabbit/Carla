"""离线评测入口：frames.jsonl -> events.json -> evaluation_report.json。"""

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_eval.metrics.event_detector import EventDetector, load_frames
from carla_eval.metrics.report_generator import ReportGenerator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario_config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    frames = load_frames(Path(args.frames))
    detector = EventDetector.from_yaml(Path(args.scenario_config))
    events = detector.detect(frames)
    detector.save(events, Path(args.output_dir) / "events.json")

    generator = ReportGenerator.from_yaml(Path(args.scenario_config))
    report = generator.generate(frames, events)
    generator.save(report, Path(args.output_dir))


if __name__ == "__main__":
    main()
