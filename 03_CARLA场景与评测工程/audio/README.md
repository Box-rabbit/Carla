# Audio Assets Guide

Voice assets are organized as `audio/<scenario>/<recording-date>/`.
Each recording has a matching `.wav` input and `.json` recognition/intent
metadata file with the same basename.

The authoritative index is `manifest.yaml`. Regenerate route-event matches
after adding or replacing recordings:

```bash
python code/carla_eval/tools/match_route_audio.py
```

`triggers/route_audio_matches.yaml` stores repository-relative POSIX
paths only. It does not retain the original recorder workstation path.
