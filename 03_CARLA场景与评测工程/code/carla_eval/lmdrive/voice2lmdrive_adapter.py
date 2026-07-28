import json
import shlex
import subprocess
import time
from pathlib import Path

from .trigger_runtime import resolve_config_relative_path


class Voice2LMDriveAdapter:
    def __init__(self, lmdrive_cfg):
        self.lmdrive_cfg = lmdrive_cfg
        self.backend_cfg = lmdrive_cfg.get("voice_backend", {})

    def _normalize_output(self, payload, fallback_text):
        intents = payload.get("intents") or payload.get("recognized_intents") or []
        if isinstance(intents, str):
            intents = [intents]

        return {
            "recognized_text": payload.get("recognized_text") or payload.get("text") or fallback_text,
            "recognized_intents": intents,
            "target_speed_max_kmh": payload.get("target_speed_max_kmh"),
            "no_collision": payload.get("no_collision"),
            "raw_output": payload,
        }

    def _run_command_backend(self, command, audio_path, scenario_id, fallback_text):
        rendered = []
        for part in command:
            rendered.append(
                part.format(
                    audio_path=str(audio_path),
                    scenario_id=scenario_id,
                )
            )

        proc = subprocess.run(
            rendered,
            capture_output=True,
            text=True,
            check=False,
        )

        if proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or f"command exited with code {proc.returncode}")

        stdout = proc.stdout.strip()
        if not stdout:
            raise RuntimeError("empty Voice2LMDrive stdout")

        try:
            payload = json.loads(stdout)
        except json.JSONDecodeError:
            payload = {
                "recognized_text": stdout,
                "recognized_intents": [],
            }

        return self._normalize_output(payload, fallback_text)

    def _run_mock_backend(self, trigger_cfg, fallback_text):
        expected = trigger_cfg.get("expected", {})
        payload = {
            "recognized_text": fallback_text,
            "recognized_intents": list(expected.get("intents", [])),
            "target_speed_max_kmh": expected.get("target_speed_max_kmh"),
            "no_collision": expected.get("no_collision"),
            "backend_note": "mock backend used because no external Voice2LMDrive command is configured",
        }
        return self._normalize_output(payload, fallback_text)

    def run(self, scenario_id, trigger_cfg, instruction_text):
        started = time.perf_counter()
        trigger = trigger_cfg.get("trigger", {})
        input_mode = trigger.get("input_mode", "wav")
        audio_path = resolve_config_relative_path(trigger_cfg, trigger.get("audio_path"))
        fallback_text = instruction_text or scenario_id

        result = {
            "backend": self.backend_cfg.get("name", "Voice2LMDrive"),
            "backend_mode": self.backend_cfg.get("execution_mode", "mock"),
            "input_mode": input_mode,
            "audio_path": str(audio_path) if audio_path is not None else trigger.get("audio_path"),
            "status": "ok",
            "error": None,
        }

        model_started = time.perf_counter()
        try:
            command = self.backend_cfg.get("command")
            if isinstance(command, str):
                command = shlex.split(command)

            if command:
                normalized = self._run_command_backend(command, audio_path, scenario_id, fallback_text)
                result["backend_mode"] = "command"
            else:
                normalized = self._run_mock_backend(trigger_cfg, fallback_text)
        except Exception as exc:  # pragma: no cover - defensive fallback
            result["status"] = "error"
            result["error"] = str(exc)
            normalized = self._run_mock_backend(trigger_cfg, fallback_text)
            result["backend_mode"] = "mock_fallback_after_error"

        model_ended = time.perf_counter()
        total_ended = time.perf_counter()

        result.update(normalized)
        result["asr_latency_ms"] = 0.0
        result["parser_latency_ms"] = 0.0
        result["model_latency_ms"] = (model_ended - model_started) * 1000.0
        result["end_to_end_latency_ms"] = (total_ended - started) * 1000.0
        return result
