# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Trossen Arm.

This configuration uses the Trossen USD file and aligns joint names
with the USD and URDF.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

TROSSEN_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"https://nucleus.fortableau.com/omni/web3/omniverse://nucleus.fortableau.com:443/Isaaclab_Meta_Assets/trossen/wxai_follower.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_0": 0.0,
            "joint_1": 1.57,
            "joint_2": 1.57,
            "joint_3": -1.57,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "left_carriage_joint": 0.0,
            "right_carriage_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[r"joint_[0-5]$"],  # matches all joints
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=80.0,
            damping=4.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*carriage_joint"],  # gripper only
            effort_limit_sim=500.0,
            velocity_limit_sim=1.0,
            stiffness=2000,
            damping=100,
        ),
    },
)
"""Configuration of Trossen arm using implicit actuator models."""


TROSSEN_HIGH_PD_CFG = TROSSEN_CFG.copy()
TROSSEN_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
TROSSEN_HIGH_PD_CFG.actuators["arm"].stiffness = 400.0
TROSSEN_HIGH_PD_CFG.actuators["arm"].damping = 80.0
TROSSEN_HIGH_PD_CFG.actuators["gripper"].stiffness = 400.0
TROSSEN_HIGH_PD_CFG.actuators["gripper"].damping = 80.0
"""Configuration of trossen robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
