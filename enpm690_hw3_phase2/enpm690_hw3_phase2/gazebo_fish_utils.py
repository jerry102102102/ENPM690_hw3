from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from simulation_interfaces.msg import EntityFilters, EntityState, Resource
from simulation_interfaces.srv import GetEntities, SetEntityState, SpawnEntity


def fish_model_paths() -> dict[str, Path]:
    share = Path(get_package_share_directory("enpm690_hw3_phase2"))
    return {
        "tuna": share / "models" / "tuna_simple" / "model.sdf",
        "sardine": share / "models" / "sardine_simple" / "model.sdf",
        "seaweed": share / "models" / "seaweed_simple" / "model.sdf",
    }


def make_get_entities_request() -> GetEntities.Request:
    request = GetEntities.Request()
    request.filters = EntityFilters()
    request.filters.filter = ""
    return request


def make_spawn_entity_request(entity_name: str, model_path: Path, x: float, y: float, z: float = 0.15) -> SpawnEntity.Request:
    request = SpawnEntity.Request()
    request.name = entity_name
    request.allow_renaming = False
    request.entity_resource = Resource(uri=model_path.as_uri(), resource_string="")
    request.initial_pose.header.frame_id = "world"
    request.initial_pose.pose.position.x = x
    request.initial_pose.pose.position.y = y
    request.initial_pose.pose.position.z = z
    request.initial_pose.pose.orientation.w = 1.0
    return request


def make_set_entity_state_request(entity_name: str, x: float, y: float, active: bool, z_active: float = 0.15) -> SetEntityState.Request:
    request = SetEntityState.Request()
    request.entity = entity_name
    request.state = EntityState()
    request.state.header.frame_id = "world"
    request.state.pose.position.x = x
    request.state.pose.position.y = y
    request.state.pose.position.z = z_active if active else -2.0
    request.state.pose.orientation.w = 1.0
    request.state.twist.linear.x = 0.0
    request.state.twist.linear.y = 0.0
    request.state.twist.linear.z = 0.0
    request.state.twist.angular.x = 0.0
    request.state.twist.angular.y = 0.0
    request.state.twist.angular.z = 0.0
    request.set_pose = True
    request.set_twist = True
    request.set_acceleration = False
    return request
