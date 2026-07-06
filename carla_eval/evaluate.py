"""离线评测入口：frames.jsonl -> events.json -> evaluation_report.json。"""
from pathlib import Path
import argparse
from metrics.logger import read_jsonl
from metrics.event_detector import EventDetector
from metrics.report_generator import ReportGenerator

def main():
    p=argparse.ArgumentParser(); p.add_argument('--scenario_config', required=True); p.add_argument('--frames', required=True); p.add_argument('--output_dir', required=True); a=p.parse_args()
    frames=read_jsonl(Path(a.frames))
    events=EventDetector.from_yaml(Path(a.scenario_config)).detect(frames)
    gen=ReportGenerator.from_yaml(Path(a.scenario_config))
    gen.save(gen.generate(frames, events), Path(a.output_dir))
if __name__=='__main__': main()
