"""场景运行入口骨架。后续接入 LMDrive / CARLA Leaderboard evaluation。"""
from pathlib import Path
import argparse, json, time

def load_yaml(path):
    import yaml
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output_dir', default='logs')
    args=parser.parse_args()
    cfg=load_yaml(args.config)
    sid=cfg.get('scenario_id', Path(args.config).stem)
    out=Path(args.output_dir)/sid; out.mkdir(parents=True, exist_ok=True)
    meta={'scenario_id':sid,'status':'placeholder','message':'Integrate with LMDrive evaluation before real execution.','timestamp':time.time()}
    (out/'run_meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(meta, ensure_ascii=False, indent=2))
if __name__=='__main__': main()
