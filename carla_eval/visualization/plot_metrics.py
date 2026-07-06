def plot_speed_curve(frames, output_path):
    import matplotlib.pyplot as plt
    times=[r.get('timestamp', i) for i,r in enumerate(frames)]; speeds=[r.get('ego_speed_kmh',0) for r in frames]
    plt.figure(); plt.plot(times, speeds); plt.xlabel('Time (s)'); plt.ylabel('Speed (km/h)'); plt.title('Speed Curve'); plt.grid(True); plt.savefig(output_path, dpi=200, bbox_inches='tight'); plt.close()
def plot_latency_curve(frames, output_path):
    import matplotlib.pyplot as plt
    times=[r.get('timestamp', i) for i,r in enumerate(frames)]; lat=[r.get('end_to_end_latency_ms',0) for r in frames]
    plt.figure(); plt.plot(times, lat); plt.xlabel('Time (s)'); plt.ylabel('End-to-end latency (ms)'); plt.title('Latency Curve'); plt.grid(True); plt.savefig(output_path, dpi=200, bbox_inches='tight'); plt.close()
