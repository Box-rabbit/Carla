PYTHON ?= python

.PHONY: list validate-pdf validate-s11 validate-s12 validate-s13 \
        build-lmdrive-routes match-audio bundle-s11 bundle-s12 bundle-s13 \
        report-s11 report-s12 report-s13

list:
	$(PYTHON) carla_eval/run_benchmark.py --list

validate-pdf: validate-s11 validate-s12 validate-s13

validate-s11:
	$(PYTHON) carla_eval/validate_config.py --config configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml

validate-s12:
	$(PYTHON) carla_eval/validate_config.py --config configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml

validate-s13:
	$(PYTHON) carla_eval/validate_config.py --config configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml

build-lmdrive-routes:
	$(PYTHON) carla_eval/tools/build_lmdrive_adapted_routes.py

match-audio:
	$(PYTHON) carla_eval/tools/match_route_audio.py

bundle-s11:
	$(PYTHON) carla_eval/tools/export_standalone_scenario_bundle.py --scenario-id S11_basic_control_scene1_5km --annotations-format json

bundle-s12:
	$(PYTHON) carla_eval/tools/export_standalone_scenario_bundle.py --scenario-id S12_complex_obstacle_scene2_8km

bundle-s13:
	$(PYTHON) carla_eval/tools/export_standalone_scenario_bundle.py --scenario-id S13_extreme_emergency_scene3_6km

report-s11:
	$(PYTHON) carla_eval/evaluate.py --scenario_config configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml --frames logs/basic_control/S11_basic_control_scene1_5km/frames.jsonl --output_dir reports/pdf_delivery/S11_basic_control_scene1_5km

report-s12:
	$(PYTHON) carla_eval/evaluate.py --scenario_config configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml --frames logs/complex_obstacle/S12_complex_obstacle_scene2_8km/frames.jsonl --output_dir reports/pdf_delivery/S12_complex_obstacle_scene2_8km

report-s13:
	$(PYTHON) carla_eval/evaluate.py --scenario_config configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml --frames logs/emergency_response/S13_extreme_emergency_scene3_6km/frames.jsonl --output_dir reports/pdf_delivery/S13_extreme_emergency_scene3_6km
