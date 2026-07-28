import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()

print("[OK] Connected to CARLA")
print("Map:", world.get_map().name)
print("Actors:", len(world.get_actors()))
