# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_joint_pos_env_cfg:TrossenCubeCapturePhotoEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Instance-Randomize-Trossen-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": (
            f"{__name__}.capture_photo_joint_pos_instance_randomize_env_cfg:TrossenCubeCapturePhotoInstanceRandomizeEnvCfg"
        ),
    },
    disable_env_checker=True,
)


##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_ik_rel_env_cfg:TrossenCubeCapturePhotoEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_low_dim.json",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Rel-Visuomotor-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_ik_rel_visuomotor_env_cfg:TrossenCubeCapturePhotoVisuomotorEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_image_200.json",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Rel-Visuomotor-Cosmos-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": (
            f"{__name__}.capture_photo_ik_rel_visuomotor_cosmos_env_cfg:TrossenCubeCapturePhotoVisuomotorCosmosEnvCfg"
        ),
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_image_cosmos.json",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_ik_abs_env_cfg:TrossenCubeCapturePhotoEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_low_dim.json",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Instance-Randomize-Trossen-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": (
            f"{__name__}.capture_photo_ik_rel_instance_randomize_env_cfg:TrossenCubeCapturePhotoInstanceRandomizeEnvCfg"
        ),
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Rel-Blueprint-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_ik_rel_blueprint_env_cfg:TrossenCubeCapturePhotoBlueprintEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Trossen-IK-Rel-Skillgen-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.capture_photo_ik_rel_env_cfg_skillgen:TrossenCubeCapturePhotoSkillgenEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_low_dim.json",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-CapturePhoto-Cube-Bin-Trossen-IK-Rel-Mimic-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bin_capture_photo_ik_rel_env_cfg:TrossenBinCapturePhotoEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_low_dim.json",
    },
    disable_env_checker=True,
)
