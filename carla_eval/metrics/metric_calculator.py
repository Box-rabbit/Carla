def mean(vals):
    vals=[v for v in vals if v is not None]
    return sum(vals)/len(vals) if vals else None
def count_events(events, name): return sum(1 for e in events if e.get('event')==name)
def compute_basic_metrics(frames, events):
    collision_count=count_events(events,'collision')
    violation_count=sum(1 for r in frames if r.get('lane_invasion') or r.get('red_light_violation') or r.get('route_deviation'))
    lat=[float(r['end_to_end_latency_ms']) for r in frames if r.get('end_to_end_latency_ms') is not None]
    success=any(e.get('event')=='task_success' for e in events) and collision_count==0
    response_times = [
        float(e.get('response_time_s')) * 1000.0
        for e in events
        if e.get('response_time_s') is not None
    ]
    return {
        'success':success,
        'collision_count':collision_count,
        'violation_count':violation_count,
        'mean_end_to_end_latency_ms':mean(lat),
        'mean_response_latency_ms':mean(response_times),
    }
