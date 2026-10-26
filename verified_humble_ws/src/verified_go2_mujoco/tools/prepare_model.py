#!/usr/bin/env python3
"""Generate MuJoCo physics from the verified controller's expanded URDF.

Existing MJCF meshes are used for visuals only. Bodies, inertias, joints,
collision shapes and IMU placement come from config/robot.urdf.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]


def numbers(values):
    return ' '.join(f'{float(x):.17g}' for x in values)


def quat(rotation):
    q = pin.Quaternion(rotation).coeffs()
    return numbers(q[[3, 0, 1, 2]])


def main():
    urdf = ROOT / 'config/robot.urdf'
    model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
    links = ET.parse(urdf).getroot()
    old = ET.parse(ROOT / 'mjcf/go2.xml').getroot()
    root = ET.Element('mujoco', model='verified_go2')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='assets', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.001', cone='elliptic', impratio='100')
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'geom', friction='0.4', condim='1', margin='0.001')
    ET.SubElement(defaults, 'joint', armature='0.01', frictionloss='0.2')
    visuals = ET.SubElement(defaults, 'default', {'class': 'visual'})
    ET.SubElement(visuals, 'geom', type='mesh', contype='0', conaffinity='0', group='2')
    root.append(copy.deepcopy(old.find('asset')))
    world = ET.SubElement(root, 'worldbody')
    bodies = {}
    for j in range(1, model.njoints):
        name = 'base_link' if j == 1 else model.names[j].removesuffix('_joint')
        parent = world if j == 1 else bodies[int(model.parents[j])]
        placement = model.jointPlacements[j]
        body = ET.SubElement(parent, 'body', name=name, pos=numbers(placement.translation),
                             quat=quat(placement.rotation))
        bodies[j] = body
        if j == 1:
            ET.SubElement(body, 'freejoint', name='root')
        else:
            joint = links.find(f"joint[@name='{model.names[j]}']")
            limit = joint.find('limit')
            ET.SubElement(body, 'joint', name=model.names[j], type='hinge',
                          axis=joint.find('axis').get('xyz'),
                          range=f"{limit.get('lower')} {limit.get('upper')}", damping='0.1')
        inertia = model.inertias[j]
        matrix = inertia.inertia
        ET.SubElement(body, 'inertial', mass=str(inertia.mass), pos=numbers(inertia.lever),
                      fullinertia=numbers([matrix[0, 0], matrix[1, 1], matrix[2, 2],
                                          matrix[0, 1], matrix[0, 2], matrix[1, 2]]))
        old_body = old.find(f".//body[@name='{name}']")
        if old_body is not None:
            for geom in old_body.findall('geom'):
                if geom.get('class') == 'visual':
                    body.append(copy.deepcopy(geom))
    for link in links.findall('link'):
        frame = model.frames[model.getFrameId(link.get('name'))]
        for i, collision in enumerate(link.findall('collision')):
            origin = collision.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            placement = frame.placement * pin.SE3(pin.rpy.rpyToMatrix(rpy), xyz)
            geometry = list(collision.find('geometry'))[0]
            name = link.get('name')[:2] if link.get('name').endswith('_foot') else f"{link.get('name')}_{i}"
            args = dict(name=name, pos=numbers(placement.translation), quat=quat(placement.rotation), group='3')
            if link.get('name').endswith('_foot'):
                args.update(priority='1', condim='6', friction='0.4 0.02 0.01')
            if geometry.tag == 'box':
                args.update(type='box', size=numbers(np.fromstring(geometry.get('size'), sep=' ')/2))
            elif geometry.tag == 'sphere':
                args.update(type='sphere', size=geometry.get('radius'))
            elif geometry.tag == 'cylinder':
                args.update(type='cylinder', size=f"{geometry.get('radius')} {float(geometry.get('length'))/2}")
            else:
                raise ValueError(f'Unsupported collision geometry: {geometry.tag}')
            ET.SubElement(bodies[int(frame.parentJoint)], 'geom', **args)
    imu = model.frames[model.getFrameId('imu_link')]
    ET.SubElement(bodies[int(imu.parentJoint)], 'site', name='imu_sensor',
                  pos=numbers(imu.placement.translation), quat=quat(imu.placement.rotation), size='.001')
    sensors = ET.SubElement(root, 'sensor')
    for kind, suffix in [('framequat', 'quat'), ('gyro', 'gyro'), ('accelerometer', 'accel')]:
        args = {'name': f'imu_sensor_{suffix}'}
        args.update({'objtype': 'site', 'objname': 'imu_sensor'} if kind == 'framequat' else {'site': 'imu_sensor'})
        ET.SubElement(sensors, kind, **args)
    q = pin.neutral(model)
    for j in range(2, model.njoints):
        q[model.idx_qs[j]] = {'hip': 0., 'thigh': .67, 'calf': -1.3}[model.names[j].split('_')[1]]
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    q[2] = .023 - min(data.oMf[model.getFrameId(f'{leg}_foot')].translation[2]
                      for leg in ('FL', 'FR', 'RL', 'RR'))
    key = np.concatenate((q[:3], q[6:7], q[3:6], q[7:]))
    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='home', qpos=numbers(key))
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(ROOT / 'mjcf/go2.xml', encoding='unicode')
    print(f'Generated {model.njoints-2} joint model from verified URDF; mass={sum(i.mass for i in model.inertias):.6f} kg')


if __name__ == '__main__':
    main()
