from __future__ import annotations


class RewardBuilder:
    def compute(
        self,
        catch_score: int,
        collision: bool,
        distance_before: float | None,
        distance_after: float | None,
        front_obstacle_proximity: float,
    ) -> tuple[float, dict[str, float]]:
        reward = -0.01
        components = {"time_penalty": -0.01}

        if catch_score > 0:
            reward += catch_score
            components["catch_reward"] = float(catch_score)

        if collision:
            reward -= 2.0
            components["collision_penalty"] = -2.0

        if distance_before is not None and distance_after is not None:
            delta = distance_before - distance_after
            if delta > 0.0:
                shaping = 0.05 * delta
            else:
                shaping = 0.05 * delta
            reward += shaping
            components["distance_shaping"] = shaping

        if front_obstacle_proximity > 0.8:
            proximity_penalty = -0.1 * front_obstacle_proximity
            reward += proximity_penalty
            components["front_proximity_penalty"] = proximity_penalty

        return reward, components
