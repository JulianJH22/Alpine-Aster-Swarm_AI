def reward(previous_distance, current_distance, invalid, success, cfg):
    return (cfg.progress_gain * (previous_distance - current_distance)
            - cfg.step_penalty - (cfg.invalid_penalty if invalid else 0.0)
            + (cfg.success_bonus if success else 0.0))
