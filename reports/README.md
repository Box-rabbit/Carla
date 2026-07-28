# Reports Guide

`reports/` contains compact, versioned evaluation summaries. Raw frame logs
are runtime artifacts under `logs/` and are intentionally excluded from Git.

Directory layout:

- `pdf_delivery/`: reports for the three PDF delivery scenarios, S11/S12/S13.

Historical short-scenario reports were moved to
`archive/legacy_short_scenarios/reports/legacy_baselines/`.

Generate a report from a run log:

```bash
python carla_eval/evaluate.py \
  --scenario_config configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml \
  --frames logs/complex_obstacle/S12_complex_obstacle_scene2_8km/frames.jsonl \
  --output_dir reports/pdf_delivery/S12_complex_obstacle_scene2_8km
```

Reports are evidence for the exact log used to generate them. They do not
replace repeated-trial statistics or real voice-model latency measurements.
