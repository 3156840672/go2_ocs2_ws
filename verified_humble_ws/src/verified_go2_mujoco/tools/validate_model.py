#!/usr/bin/env python3
"""Compare generated MuJoCo FK and mass matrices against the verified URDF."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin


def main():
    root = Path(__file__).resolve().parents[1]
    model = pin.buildModelFromUrdf(str(root/'config/robot.urdf'), pin.JointModelFreeFlyer())
    data = model.createData()
    sim = mujoco.MjModel.from_xml_path(str(root/'mjcf/scene.xml'))
    state = mujoco.MjData(sim)
    foot_error = mass_error = 0.
    for seed in range(10):
        rng = np.random.default_rng(seed)
        q = pin.neutral(model)
        q[2] = .36
        for j in range(2, model.njoints):
            q[model.idx_qs[j]] = {'hip': 0., 'thigh': .67, 'calf': -1.3}[
                model.names[j].split('_')[1]] + rng.uniform(-.08, .08)
        state.qpos[:3], state.qpos[3:7] = q[:3], q[[6, 3, 4, 5]]
        for j in range(2, model.njoints):
            idx = mujoco.mj_name2id(sim, mujoco.mjtObj.mjOBJ_JOINT, model.names[j])
            state.qpos[sim.jnt_qposadr[idx]] = q[model.idx_qs[j]]
        mujoco.mj_forward(sim, state)
        pin.framesForwardKinematics(model, data, q)
        for leg in ('FL', 'FR', 'RL', 'RR'):
            idx = mujoco.mj_name2id(sim, mujoco.mjtObj.mjOBJ_GEOM, leg)
            expected = data.oMf[model.getFrameId(leg+'_foot')].translation
            foot_error = max(foot_error, float(np.max(np.abs(state.geom_xpos[idx]-expected))))
        matrix = np.zeros((sim.nv, sim.nv))
        try:  # MuJoCo 3.6+ Python API.
            mujoco.mj_fullM(sim, state, matrix)
        except TypeError:  # MuJoCo 3.3 Python API.
            mujoco.mj_fullM(sim, matrix, state.qM)
        # MuJoCo adds configured rotor armature to the generalized inertia diagonal.
        # Remove it when comparing the rigid-body model imported from URDF.
        rigid_body_matrix = matrix - np.diag(sim.dof_armature)
        mass_error = max(
            mass_error, float(np.max(np.abs(rigid_body_matrix-pin.crba(model, data, q)))))
    # MuJoCo diagonalizes imported full inertias numerically. A 1e-6 absolute
    # tolerance allows that roundoff (observed 4.81e-8), not model mismatch.
    report = {'samples': 10, 'robot_mass_kg': float(sim.body_mass.sum()),
              'maximum_armature_kg_m2': float(np.max(sim.dof_armature)),
              'worst_foot_position_error_m': foot_error, 'worst_mass_matrix_error': mass_error,
              'mass_matrix_tolerance': 1e-6, 'passed': foot_error < 1e-9 and mass_error < 1e-6}
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
