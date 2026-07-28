"""Fixed-position voice instruction overlay.

Uses tkinter from the Python standard library to avoid extra runtime
dependencies. The overlay is a fixed desktop window, not a CARLA world-space
debug label, so the text does not move with the ego vehicle.
"""

from typing import Any, Dict, Optional


class FixedVoiceOverlay:
    def __init__(self, title: str = "Voice Input", geometry: str = "980x300+40+40"):
        self._tk = None
        self._root = None
        self._labels = {}
        self.available = False

        try:
            import tkinter as tk

            self._tk = tk
            root = tk.Tk()
            root.title(title)
            root.geometry(geometry)
            root.configure(bg="#050505")
            root.attributes("-topmost", True)

            title_label = tk.Label(
                root,
                text="语音输入 / Voice Command",
                font=("Noto Sans CJK SC", 28, "bold"),
                fg="#ffd84d",
                bg="#050505",
                anchor="w",
            )
            title_label.pack(fill="x", padx=24, pady=(18, 8))

            for key, font_size, color in (
                ("text", 30, "#ffffff"),
                ("intent", 24, "#7fffd4"),
                ("event", 24, "#ffb86c"),
                ("progress", 22, "#cfcfcf"),
            ):
                label = tk.Label(
                    root,
                    text="",
                    font=("Noto Sans CJK SC", font_size, "bold"),
                    fg=color,
                    bg="#050505",
                    anchor="w",
                    justify="left",
                    wraplength=920,
                )
                label.pack(fill="x", padx=24, pady=4)
                self._labels[key] = label

            self._root = root
            self.available = True
            self.update(None, 0.0)
        except Exception as exc:
            print(f"[VOICE_OVERLAY] disabled: {exc}")
            self.available = False

    def update(self, event: Optional[Dict[str, Any]], route_progress_m: float) -> None:
        if not self.available:
            return

        if event is None:
            values = {
                "text": "等待语音触发...",
                "intent": "intent: -",
                "event": "event: -",
                "progress": f"route_progress: {route_progress_m:.1f} m",
            }
        else:
            voice = event.get("voice", {})
            expected = event.get("expected", {})
            text = voice.get("input_text") or voice.get("normalized_text") or voice.get("instruction") or "-"
            intents = voice.get("recognized_intents") or expected.get("recognized_intents") or expected.get("intents") or []
            values = {
                "text": f"语音：{text}",
                "intent": "intent: " + ", ".join(str(item) for item in intents),
                "event": f"route/event: {event.get('route_id')} :: {event.get('event_id')}",
                "progress": (
                    f"trigger: {event.get('trigger', {}).get('distance_m')} m"
                    f"    current: {route_progress_m:.1f} m"
                ),
            }

        try:
            for key, value in values.items():
                self._labels[key].configure(text=value)

            self._root.update_idletasks()
            self._root.update()
        except Exception as exc:
            print(f"[VOICE_OVERLAY] disabled after update error: {exc}")
            self.available = False

    def destroy(self) -> None:
        if not self.available:
            return
        try:
            self._root.destroy()
        except Exception:
            pass
        self.available = False
