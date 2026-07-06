from pathlib import Path
from typing import Any, Dict, Iterable, List
import csv, json
class FrameLogger:
    def __init__(self, output_dir):
        self.output_dir=Path(output_dir); self.output_dir.mkdir(parents=True, exist_ok=True)
        self._fp=(self.output_dir/'frames.jsonl').open('a', encoding='utf-8')
    def log_frame(self, record: Dict[str, Any]):
        self._fp.write(json.dumps(record, ensure_ascii=False)+'\n')
    def close(self): self._fp.close()
def read_jsonl(path):
    path=Path(path); out=[]
    if not path.exists(): return out
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip(): out.append(json.loads(line))
    return out
def write_csv(records: Iterable[Dict[str, Any]], path):
    records=list(records); path=Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    if not records: path.write_text('', encoding='utf-8'); return
    keys=sorted({k for r in records for k in r})
    with path.open('w', newline='', encoding='utf-8') as f:
        writer=csv.DictWriter(f, fieldnames=keys); writer.writeheader(); writer.writerows(records)
