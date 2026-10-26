# Third-party notices

This repository combines original integration work with upstream open-source projects. Each third-party component remains subject to its own license and copyright notices.

| Component | Upstream | License / notice |
| --- | --- | --- |
| OCS2 ROS 2 | https://github.com/legubiao/ocs2_ros2, pinned at `cd23b0c3fd131aa3eed12d77371c7ac9a5a38828` | Not vendored by this repository. Source-file notices and licenses in the fetched repository apply. |
| quadruped_ros2_control | https://github.com/legubiao/quadruped_ros2_control | Apache-2.0; retained in `verified_humble_ws/src/quadruped_ros2_control/LICENSE`. |
| legged_control-derived controller code | https://github.com/qiayuanl/legged_control | BSD-3-Clause; retained in `verified_humble_ws/src/quadruped_ros2_control/LICENSES/legged_control/LICENSE`. |
| qpOASES | https://github.com/coin-or/qpOASES, pinned at `9e40af7d170f440b7887fc4f9cf162f3f3ae24e8` | LGPL-2.1; fetched at build time and dynamically linked. |
| mujoco_ros2_control base implementation | Package metadata credits Sangtaek Lee | MIT; local modifications preserve the package metadata and MIT notice. |
| MuJoCo | https://github.com/google-deepmind/mujoco | Apache-2.0; installed separately and not redistributed here. |
| Unitree Go2 URDF and DAE meshes | https://github.com/unitreerobotics/unitree_ros | BSD-3-Clause, copyright Unitree Robotics; retained in `go2_description/LICENSE`. |
| Unitree Go2 MJCF visual assets | https://github.com/legubiao/unitree_mujoco, source revision `ace942311ffef188d6ce15fc6728a1aafceeba63` | BSD-3-Clause, copyright Unitree Robotics; retained in `verified_go2_mujoco/LICENSES/unitree_mujoco/LICENSE`. |

The OCS2 and qpOASES source trees are downloaded during setup and are intentionally excluded from this Git repository. Generated CppAD libraries are also excluded because they are machine-generated build artifacts.
