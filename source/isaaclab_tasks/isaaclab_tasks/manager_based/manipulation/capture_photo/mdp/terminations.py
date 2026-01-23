# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

#modification for capture photo
def is_grasped(env: ManagerBasedRLEnv,
               robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """
    Termination triggers immediately after gripper closes then opens.
    Assumes _grasped_once is properly reset at episode start via reset().
    """

    robot: Articulation = env.scene[robot_cfg.name]

    if not hasattr(env, "_grasped_once"):
        env._grasped_once = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    # -------------------------------
    # PARALLEL JAW GRIPPER
    # -------------------------------
    if hasattr(env.cfg, "gripper_joint_names"):
        joint_ids, _ = robot.find_joints(env.cfg.gripper_joint_names)

        fingers_closed = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
        for j in joint_ids:
            fingers_closed &= (
                torch.abs(
                    robot.data.joint_pos[:, j]
                    - torch.tensor(env.cfg.gripper_open_val, dtype=torch.float32).to(env.device)
                ) > env.cfg.gripper_threshold
            )

        fingers_open = ~fingers_closed

        env._grasped_once |= fingers_closed

        terminated = env._grasped_once & fingers_open
        return terminated

    # -------------------------------
    # SUCTION GRIPPER
    # -------------------------------
    if hasattr(env.scene, "surface_grippers") and len(env.scene.surface_grippers) > 0:
        sg = env.scene.surface_grippers["surface_gripper"]
        suction_closed = (sg.state == 1).view(-1)
        suction_open = (sg.state == -1).view(-1)

        env._grasped_once |= suction_closed
        terminated = env._grasped_once & suction_open
        return terminated

    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)