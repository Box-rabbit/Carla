"""
RouteIndexer: iterate over all routes in an XML file with repetitions and
resume support (LMDrive-style).
"""

import json
from pathlib import Path
from typing import List, Optional

from .route_parser import RouteParser, RouteScenarioConfiguration


class RouteIndexer:
    """
    Iterate over a route XML file producing RouteScenarioConfiguration objects.

    Supports:
    - repetitions: run each route N times (different random seeds)
    - resume:      skip already-completed routes from a checkpoint JSON
    """

    def __init__(
        self,
        routes_file: Path,
        scenarios_file: Optional[Path] = None,
        repetitions: int = 1,
    ):
        self._routes_file = Path(routes_file)
        self._scenarios_file = Path(scenarios_file) if scenarios_file else None
        self._repetitions = max(1, repetitions)

        raw = RouteParser.parse_routes_file(self._routes_file, self._scenarios_file)
        # Expand by repetitions: each config gets a seed = rep_index
        self._configs: List[RouteScenarioConfiguration] = []
        for rep in range(self._repetitions):
            for cfg in raw:
                import copy
                c = copy.copy(cfg)
                c.extra = dict(c.extra)
                c.extra["repetition"] = rep
                c.extra["random_seed"] = rep
                if rep > 0:
                    c.name = f"{cfg.name}_rep{rep}"
                self._configs.append(c)

        self._index = 0
        self.total = len(self._configs)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def peek(self) -> bool:
        """Return True if there are remaining routes."""
        return self._index < self.total

    def next(self) -> RouteScenarioConfiguration:
        """Return the next config and advance the index."""
        if not self.peek():
            raise StopIteration("No more routes.")
        cfg = self._configs[self._index]
        self._index += 1
        return cfg

    def __iter__(self):
        while self.peek():
            yield self.next()

    # ------------------------------------------------------------------
    # Checkpoint / Resume
    # ------------------------------------------------------------------

    def resume(self, checkpoint_path: Path) -> int:
        """
        Load progress from a checkpoint JSON and fast-forward the index.

        Returns the number of routes skipped.
        """
        checkpoint_path = Path(checkpoint_path)
        if not checkpoint_path.exists():
            return 0

        with checkpoint_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        progress = data.get("_checkpoint", {}).get("progress", [0, self.total])
        completed = int(progress[0]) if isinstance(progress, list) else 0
        completed = min(completed, self.total)
        skipped = completed - self._index
        self._index = completed
        return max(0, skipped)

    def save_state(self, checkpoint_path: Path, extra_data: Optional[dict] = None):
        """Persist current progress to a checkpoint JSON."""
        checkpoint_path = Path(checkpoint_path)
        checkpoint_path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            "_checkpoint": {
                "progress": [self._index, self.total],
                "routes_file": str(self._routes_file),
                "repetitions": self._repetitions,
            },
        }
        if extra_data:
            payload.update(extra_data)

        with checkpoint_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def current_index(self) -> int:
        return self._index

    def remaining(self) -> int:
        return self.total - self._index

    def __repr__(self) -> str:
        return (
            f"RouteIndexer(routes={self._routes_file.name}, "
            f"total={self.total}, index={self._index}, "
            f"repetitions={self._repetitions})"
        )
