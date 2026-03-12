from __future__ import annotations

import math
from pathlib import Path

from simulation_interfaces.msg import EntityState, Resource
from simulation_interfaces.srv import DeleteEntity, SetEntityState, SpawnEntity


def make_spawn_entity_request(entity_name: str, model_path: Path, x: float, y: float, z: float = 0.12) -> SpawnEntity.Request:
    request = SpawnEntity.Request()
    request.name = entity_name
    if hasattr(request, "allow_renaming"):
        request.allow_renaming = False

    resource = Resource(uri=model_path.as_uri(), resource_string=model_path.read_text())
    if hasattr(request, "entity_resource"):
        request.entity_resource = resource
    elif hasattr(request, "resource"):
        request.resource = resource
    elif hasattr(request, "uri"):
        request.uri = model_path.as_uri()
    else:
        raise AttributeError("SpawnEntity request does not expose a supported resource field")

    if hasattr(request, "entity_namespace"):
        request.entity_namespace = ""

    request.initial_pose.header.frame_id = "world"
    request.initial_pose.pose.position.x = x
    request.initial_pose.pose.position.y = y
    request.initial_pose.pose.position.z = z
    request.initial_pose.pose.orientation.w = 1.0
    return request


def make_delete_entity_request(entity_name: str) -> DeleteEntity.Request:
    request = DeleteEntity.Request()
    if hasattr(request, "entity"):
        request.entity = entity_name
    elif hasattr(request, "name"):
        request.name = entity_name
    else:
        raise AttributeError("DeleteEntity request does not expose a supported name field")
    return request


def make_set_entity_state_request(entity_name: str, x: float, y: float, heading: float, z: float = 0.16) -> SetEntityState.Request:
    request = SetEntityState.Request()
    if hasattr(request, "entity"):
        request.entity = entity_name
    elif hasattr(request, "name"):
        request.name = entity_name
    else:
        raise AttributeError("SetEntityState request does not expose a supported entity field")

    request.state = EntityState()
    request.state.header.frame_id = "world"
    request.state.pose.position.x = x
    request.state.pose.position.y = y
    request.state.pose.position.z = z
    request.state.pose.orientation.z = math.sin(heading / 2.0)
    request.state.pose.orientation.w = math.cos(heading / 2.0)
    request.state.twist.linear.x = 0.0
    request.state.twist.linear.y = 0.0
    request.state.twist.linear.z = 0.0
    request.state.twist.angular.x = 0.0
    request.state.twist.angular.y = 0.0
    request.state.twist.angular.z = 0.0
    if hasattr(request, "set_pose"):
        request.set_pose = True
    if hasattr(request, "set_twist"):
        request.set_twist = True
    if hasattr(request, "set_acceleration"):
        request.set_acceleration = False
    return request
