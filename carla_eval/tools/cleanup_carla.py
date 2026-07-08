import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()

actors = []
actors.extend(world.get_actors().filter("vehicle.*"))
actors.extend(world.get_actors().filter("walker.*"))
actors.extend(world.get_actors().filter("sensor.*"))

print(f"[INFO] actors to destroy: {len(actors)}")

for actor in actors:
    try:
        actor.destroy()
    except Exception as e:
        print("[WARN] failed to destroy", actor.id, e)

print("[OK] cleanup finished")
