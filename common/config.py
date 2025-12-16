#!/usr/bin/env python3
"""Camera configuration for vision inspection pipeline

All measurements in millimeters (mm) unless otherwise specified.
"""

import numpy as np
from pathlib import Path

# ============================================================================
# Project Paths
# ============================================================================
PROJECT_ROOT = Path(__file__).parent.parent
DATA_ROOT = PROJECT_ROOT / "data"

# ============================================================================
# Camera Specifications
# ============================================================================

# Camera Field of View (mm)
CAMERA_FOV_WIDTH_MM = 41.0
CAMERA_FOV_HEIGHT_MM = 30.0

# Working distance (mm) - distance from camera to surface
CAMERA_WORKING_DISTANCE_MM = 110.0

# Overlap ratio between adjacent viewpoints (0.5 = 50% overlap)
CAMERA_OVERLAP_RATIO = 0.5


# ============================================================================
# World Configuration (Isaac Sim coordinates, meters)
# ============================================================================

# Target object configuration
TARGET_OBJECT = {
    "name": "target_object",
    "position": np.array([0.00, 1.09 + 0.13, 0.8 - 0.8], dtype=np.float64),
    "rotation": np.array([0.7071, 0.0, 0.0, 0.7071], dtype=np.float64),  # quaternion: w, x, y, z
}

# Table cuboid configuration
TABLE = {
    "name": "table",
    "position": np.array([0.0, 1.09 + 0.05, 0.365 - 0.8], dtype=np.float64),
    "dimensions": np.array([1.0, 0.6, 0.5], dtype=np.float64),
}

# Wall (Fence) cuboid configuration - 4 walls surrounding the workspace
WALLS = [
    {
        "name": "wall_front",
        "position": np.array([0.0, 1.5, 0.5], dtype=np.float64),
        "dimensions": np.array([2.2, 0.1, 1.0], dtype=np.float64),
    },
    {
        "name": "wall_back",
        "position": np.array([0.0, -1.0, 0.5], dtype=np.float64),
        "dimensions": np.array([2.2, 0.1, 1.0], dtype=np.float64),
    },
    {
        "name": "wall_left",
        "position": np.array([-1.0, 0.0, 0.5], dtype=np.float64),
        "dimensions": np.array([0.1, 2.2, 1.0], dtype=np.float64),
    },
    {
        "name": "wall_right",
        "position": np.array([1.0, 0.0, 0.5], dtype=np.float64),
        "dimensions": np.array([0.1, 2.2, 1.0], dtype=np.float64),
    },
]

# Robot mount (base) configuration
ROBOT_MOUNT = {
    "name": "robot_mount",
    "position": np.array([0.0, 0.0, -0.25], dtype=np.float64),
    "dimensions": np.array([0.3, 0.3, 0.5], dtype=np.float64),
}


# ============================================================================
# Robot Configuration
# ============================================================================


DEFAULT_ROBOT_CONFIG = "ur20_with_camera.yml"
DEFAULT_URDF_PATH = "/curobo/src/curobo/content/assets/robot/ur_description/ur20_with_camera.urdf"

# Tool offset: distance from tool0/wrist3 to camera_optical_frame (meters)
# End-Effector로부터 카메라 초점까지의 실제 거리로 바뀌어야 합니ㅏ다.
TOOL_TO_CAMERA_OPTICAL_OFFSET_M = 0.234


# ============================================================================
# IK Solver Parameters
# ============================================================================

IK_NUM_SEEDS = 32

# ============================================================================
# GTSP optimization defaults (not in config)
# ============================================================================
DEFAULT_KNN = 10
DEFAULT_LAMBDA_ROT = 1.0

# ============================================================================
# Collision Checking Parameters
# ============================================================================

COLLISION_MARGIN = 0.0
COLLISION_ADAPTIVE_MAX_JOINT_STEP_DEG = 0.05
COLLISION_INTERP_EXCLUDE_LAST_JOINT = True


# ============================================================================
# Replanning Parameters
# ============================================================================

REPLAN_ENABLED = True
REPLAN_MAX_ATTEMPTS = 60
REPLAN_TIMEOUT = 10.0  # seconds
REPLAN_INTERP_DT = 0.02
REPLAN_TRAJOPT_TSTEPS = 32


# ============================================================================
# Object-Based Data Path Helpers
# ============================================================================

def get_mesh_path(object_name: str, filename: str = None, mesh_type: str = "target") -> Path:
    """
    Get path to object mesh file

    Args:
        object_name: Name of the object (e.g., "glass", "phone")
        filename: Explicit mesh filename (overrides mesh_type if provided)
        mesh_type: Type of mesh file (default: "target")
            - "source": source.obj (full multi-material mesh for collision checking)
            - "target": target.ply (inspection surface for viewpoint sampling)

    Returns:
        Path to mesh file: data/{object_name}/mesh/{filename}

    Examples:
        >>> get_mesh_path("glass")  # Default: target mesh
        PosixPath('data/glass/mesh/target.ply')  # or target.obj if .ply doesn't exist

        >>> get_mesh_path("glass", mesh_type="source")  # Full mesh for collision
        PosixPath('data/glass/mesh/source.obj')

        >>> get_mesh_path("glass", filename="custom.obj")  # Explicit filename
        PosixPath('data/glass/mesh/custom.obj')
    """
    if filename is None:
        # Auto-determine filename based on mesh_type
        if mesh_type == "source":
            filename = "source.obj"
        elif mesh_type == "target":
            # Try target.ply first (preferred for inspection), fallback to target.obj
            target_ply = DATA_ROOT / object_name / "mesh" / "target.ply"
            if target_ply.exists():
                return target_ply
            filename = "target.obj"
        else:
            raise ValueError(f"Invalid mesh_type: '{mesh_type}'. Must be 'source' or 'target'")

    return DATA_ROOT / object_name / "mesh" / filename


def get_viewpoint_path(object_name: str, num_viewpoints: int, filename: str = "viewpoints.h5") -> Path:
    """
    Get path to viewpoints file

    Args:
        object_name: Name of the object (e.g., "glass")
        num_viewpoints: Number of viewpoints
        filename: Filename (default: "viewpoints.h5")

    Returns:
        Path to viewpoints: data/{object_name}/viewpoint/{num_viewpoints}/{filename}

    Example:
        >>> get_viewpoint_path("glass", 500)
        PosixPath('data/glass/viewpoint/500/viewpoints.h5')
    """
    return DATA_ROOT / object_name / "viewpoint" / str(num_viewpoints) / filename


def get_ik_path(object_name: str, num_viewpoints: int, filename: str = "ik_solutions.h5") -> Path:
    """
    Get path to IK solutions file

    Args:
        object_name: Name of the object (e.g., "glass")
        num_viewpoints: Number of viewpoints
        filename: Filename (default: "ik_solutions.h5")

    Returns:
        Path to IK solutions: data/{object_name}/ik/{num_viewpoints}/{filename}

    Example:
        >>> get_ik_path("glass", 500)
        PosixPath('data/glass/ik/500/ik_solutions.h5')
    """
    return DATA_ROOT / object_name / "ik" / str(num_viewpoints) / filename


def get_trajectory_path(object_name: str, num_viewpoints: int, filename: str = "gtsp.csv") -> Path:
    """
    Get path to trajectory file

    Args:
        object_name: Name of the object (e.g., "glass")
        num_viewpoints: Number of viewpoints
        filename: Filename (default: "gtsp.csv", can also be "gtsp_final.csv")

    Returns:
        Path to trajectory: data/{object_name}/trajectory/{num_viewpoints}/{filename}

    Example:
        >>> get_trajectory_path("glass", 500)
        PosixPath('data/glass/trajectory/500/gtsp.csv')
        >>> get_trajectory_path("glass", 500, "gtsp_final.csv")
        PosixPath('data/glass/trajectory/500/gtsp_final.csv')
    """
    return DATA_ROOT / object_name / "trajectory" / str(num_viewpoints) / filename
