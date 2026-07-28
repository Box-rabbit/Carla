"""Package bootstrap for Dongfeng CARLA evaluation."""

from pathlib import Path
import os
import sys


def _bootstrap_carla_pythonapi() -> None:
    """Best-effort: add a matching CARLA egg to sys.path if not already importable."""
    try:
        import carla  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    major = sys.version_info.major
    minor = sys.version_info.minor
    py_tag = f"py{major}.{minor}"

    candidates = []

    env_egg = os.environ.get("CARLA_PYTHONAPI_EGG")
    if env_egg:
        candidates.append(Path(env_egg))

    env_root = os.environ.get("CARLA_ROOT")
    if env_root:
        candidates.append(
            Path(env_root)
            / "PythonAPI/carla/dist"
            / f"carla-0.9.10-{py_tag}-linux-x86_64.egg"
        )

    default_roots = [
        Path("/data/hdt_workspace/CARLA_0.9.10.1"),
        Path.home() / "CARLA_0.9.10.1",
    ]
    for root in default_roots:
        candidates.append(
            root / "PythonAPI/carla/dist" / f"carla-0.9.10-{py_tag}-linux-x86_64.egg"
        )

    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            break


_bootstrap_carla_pythonapi()
