#pragma once

#include <mujoco/mujoco.h>

namespace mujoco_ros2_control {
// BODY object velocity is centered at the COM; its local frame is the
// inertial principal-axis frame, not the link frame used by Odometry.
inline void bodyOriginLocalVelocity(const mjModel *model, const mjData *data,
                                    int body, mjtNum result[6]) {
  mjtNum world[6], offset[3], cross[3], origin[3];
  mj_objectVelocity(model, data, mjOBJ_BODY, body, world, 0);
  mju_sub3(offset, data->xipos + 3 * body, data->xpos + 3 * body);
  mju_cross(cross, world, offset);
  mju_sub3(origin, world + 3, cross);
  mju_mulMatTVec(result, data->xmat + 9 * body, world, 3, 3);
  mju_mulMatTVec(result + 3, data->xmat + 9 * body, origin, 3, 3);
}
} // namespace mujoco_ros2_control
