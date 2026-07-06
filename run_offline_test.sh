set -e

echo "[1/6] Validate config"
python carla_eval/validate_config.py \
  --config configs/scenarios/S01_keep_lane_speed.yaml

echo "[2/6] Generate fake success log"
python carla_eval/generate_fake_success_log.py

echo "[3/6] Detect success events"
python carla_eval/metrics/event_detector.py \
  --config configs/scenarios/S01_keep_lane_speed.yaml \
  --frames logs/S01_keep_lane_speed/frames.jsonl \
  --output logs/S01_keep_lane_speed/events.json

echo "[4/6] Generate success report"
python carla_eval/metrics/report_generator.py \
  --config configs/scenarios/S01_keep_lane_speed.yaml \
  --frames logs/S01_keep_lane_speed/frames.jsonl \
  --events logs/S01_keep_lane_speed/events.json \
  --output_dir reports/S01_keep_lane_speed

echo "[5/6] Generate fake collision log"
python carla_eval/generate_fake_collision_log.py

echo "[6/6] Detect collision events and generate failure report"
python carla_eval/metrics/event_detector.py \
  --config configs/scenarios/S01_keep_lane_speed.yaml \
  --frames logs/S01_keep_lane_speed_collision/frames.jsonl \
  --output logs/S01_keep_lane_speed_collision/events.json

python carla_eval/metrics/report_generator.py \
  --config configs/scenarios/S01_keep_lane_speed.yaml \
  --frames logs/S01_keep_lane_speed_collision/frames.jsonl \
  --events logs/S01_keep_lane_speed_collision/events.json \
  --output_dir reports/S01_keep_lane_speed_collision

echo "[DONE] Offline evaluation test finished."
echo "Success report: reports/S01_keep_lane_speed/evaluation_report.json"
echo "Failure report: reports/S01_keep_lane_speed_collision/evaluation_report.json"
