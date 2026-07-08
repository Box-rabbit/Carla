import math
import carla


def yaw_diff(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)


client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
carla_map = world.get_map()
spawn_points = carla_map.get_spawn_points()

candidates = []

for idx, sp in enumerate(spawn_points):
    try:
        wp = carla_map.get_waypoint(
            sp.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

        yaws = [wp.transform.rotation.yaw]
        cur = wp

        ok = True
        for _ in range(12):
            nxt = cur.next(10.0)
            if not nxt:
                ok = False
                break
            cur = nxt[0]
            yaws.append(cur.transform.rotation.yaw)

        if not ok:
            continue

        total_yaw_change = 0.0
        max_step_yaw = 0.0

        for a, b in zip(yaws[:-1], yaws[1:]):
            d = yaw_diff(a, b)
            total_yaw_change += d
            max_step_yaw = max(max_step_yaw, d)

        score = total_yaw_change + 2.0 * max_step_yaw

        candidates.append({
            "idx": idx,
            "score": score,
            "total_yaw_change": total_yaw_change,
            "max_step_yaw": max_step_yaw,
            "x": sp.location.x,
            "y": sp.location.y,
            "z": sp.location.z,
            "yaw": sp.rotation.yaw,
        })

    except Exception:
        continue

candidates.sort(key=lambda x: x["score"])

print(f"Total spawn points: {len(spawn_points)}")
print("Top straight spawn candidates:")
for c in candidates[:30]:
    print(
        f"idx={c['idx']:3d} "
        f"score={c['score']:.2f} "
        f"total_yaw={c['total_yaw_change']:.2f} "
        f"max_step_yaw={c['max_step_yaw']:.2f} "
        f"loc=({c['x']:.1f},{c['y']:.1f},{c['z']:.1f}) "
        f"yaw={c['yaw']:.1f}"
    )
