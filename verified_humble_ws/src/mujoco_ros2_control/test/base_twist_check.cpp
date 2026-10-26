#include "mujoco_ros2_control/base_twist.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  char error[1024];
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) { std::cerr << error << '\n'; return 1; }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  const int body = mj_name2id(model.get(), mjOBJ_BODY, "base_link");
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  if (body < 0 || home < 0) return 1;
  double max_error = 0, finite_error = 0, old_error = 0;
  for (int trial = 0; trial < 4; ++trial) {
    mj_resetDataKeyframe(model.get(), data.get(), home);
    data->qpos[3] = 1;
    for (int j = 0; j < 3; ++j) data->qpos[4+j] = .07 * trial * (j+1);
    mju_normalize4(data->qpos + 3);
    for (int j = 0; j < model->nv; ++j) data->qvel[j] = -.5 + .06*j;
    mj_forward(model.get(), data.get());
    mjtNum local[6], old[6], rotation[9], position[3];
    mujoco_ros2_control::bodyOriginLocalVelocity(model.get(), data.get(), body, local);
    mj_objectVelocity(model.get(), data.get(), mjOBJ_BODY, body, old, 1);
    mju_copy(rotation, data->xmat + 9*body, 9);
    mju_copy3(position, data->xpos + 3*body);
    std::vector<mjtNum> jp(3*model->nv), jr(3*model->nv);
    mj_jacBody(model.get(), data.get(), jp.data(), jr.data(), body);
    mjtNum v[3], w[3], expected_v[3], expected_w[3];
    mju_mulMatVec(v, jp.data(), data->qvel, 3, model->nv);
    mju_mulMatVec(w, jr.data(), data->qvel, 3, model->nv);
    mju_mulMatTVec(expected_v, rotation, v, 3, 3);
    mju_mulMatTVec(expected_w, rotation, w, 3, 3);
    for (int j = 0; j < 3; ++j) {
      max_error = std::max(max_error, std::abs(local[j] - expected_w[j]));
      max_error = std::max(max_error, std::abs(local[j+3] - expected_v[j]));
      old_error = std::max(old_error, std::abs(old[j+3] - expected_v[j]));
    }
    constexpr double dt = 1e-7;
    mj_integratePos(model.get(), data->qpos, data->qvel, dt);
    mj_forward(model.get(), data.get());
    mjtNum delta[3], finite[3];
    mju_sub3(delta, data->xpos + 3*body, position);
    mju_mulMatTVec(finite, rotation, delta, 3, 3);
    for (int j = 0; j < 3; ++j)
      finite_error = std::max(finite_error, std::abs(finite[j]/dt - local[j+3]));
  }
  std::cout << "Jacobian error=" << max_error << " finite difference error="
            << finite_error << " old interface error=" << old_error << '\n';
  return max_error < 1e-12 && finite_error < 1e-7 && old_error > .1 ? 0 : 1;
}
