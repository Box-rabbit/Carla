import carla


def is_valid_left_lane(wp, left_wp):
    if left_wp is None:
        return False

    if left_wp.lane_type != carla.LaneType.Driving:
        return False

    # 避免选到对向车道：要求 lane_id 同号
    if wp.lane_id * left_wp.lane_id <= 0:
        return False

    return True


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

        left_wp = wp.get_left_lane()

        if not is_valid_left_lane(wp, left_wp):
            continue

        # 向前检查一段距离，避免刚变道就到路口或断路
        cur_wp = wp
        cur_left_wp = left_wp
        ok = True

        for _ in range(8):
            next_wps = cur_wp.next(10.0)
            next_left_wps = cur_left_wp.next(10.0)

            if not next_wps or not next_left_wps:
                ok = False
                break

            cur_wp = next_wps[0]
            cur_left_wp = next_left_wps[0]

        if not ok:
            continue

        candidates.append({
            "idx": idx,
            "x": sp.location.x,
            "y": sp.location.y,
            "z": sp.location.z,
            "yaw": sp.rotation.yaw,
            "road_id": wp.road_id,
            "lane_id": wp.lane_id,
            "target_lane_id": left_wp.lane_id,
        })

    except Exception:
        continue

print(f"Total spawn points: {len(spawn_points)}")
print(f"Left-lane-change candidates: {len(candidates)}")

for c in candidates[:40]:
    print(
        f"idx={c['idx']:3d} "
        f"road={c['road_id']} "
        f"lane={c['lane_id']} -> target_lane={c['target_lane_id']} "
        f"loc=({c['x']:.1f},{c['y']:.1f},{c['z']:.1f}) "
        f"yaw={c['yaw']:.1f}"
    )
