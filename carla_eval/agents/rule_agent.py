class RuleAgent:
    """
    最小规则 Agent，用来代替 LMDrive。
    当前只做一件事：控制车辆加速到目标速度附近。
    """

    def __init__(self, target_speed_kmh=60.0):
        self.target_speed_kmh = float(target_speed_kmh)

    def run_step(self, observation, instruction=None):
        speed_kmh = float(observation.get("speed_kmh", 0.0))

        if speed_kmh < self.target_speed_kmh - 5:
            throttle = 0.55
            brake = 0.0
        elif speed_kmh > self.target_speed_kmh + 5:
            throttle = 0.0
            brake = 0.25
        else:
            throttle = 0.25
            brake = 0.0

        control = {
            "steer": 0.0,
            "throttle": throttle,
            "brake": brake,
        }

        debug_info = {
            "agent_type": "RuleAgent",
            "asr_latency_ms": 0,
            "parser_latency_ms": 0,
            "model_latency_ms": 1,
        }

        return control, debug_info
