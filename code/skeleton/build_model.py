"""Rebuild three self-contained MJCF variants from the archived model and PMX hand landmarks.

Run from the project root: python code/skeleton/build_model.py.
The original PMX is not needed: hand_reference.json stores the source landmarks.
"""
from pathlib import Path
import copy
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

BASE = Path(__file__).resolve().parents[2] / 'models' / 'Venti_skeleton_updated'
ARCHIVE = BASE / 'backups' / 'before_hand_revision_20261108'
REFERENCE = json.loads((BASE / 'hand_reference.json').read_text(encoding='utf-8'))


def vec(value):
    return np.fromstring(value, sep=' ')


def fmt(values):
    return ' '.join(f'{float(x):.10g}' for x in values)


def unit(value):
    return value / np.linalg.norm(value)


def translated(element, offset):
    result = copy.deepcopy(element)
    if 'fromto' in result.attrib:
        result.set('fromto', fmt((vec(result.get('fromto')).reshape(2, 3) + offset).ravel()))
    else:
        result.set('pos', fmt(vec(result.get('pos', '0 0 0')) + offset))
    return result


def basis_quat(x, y, z):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([x, y, z]).ravel())
    return fmt(quat)


def joint_range(name):
    if name.startswith('waist_'):
        return {'roll': (-25, 25), 'pitch': (-35, 40), 'yaw': (-60, 60)}[name.split('_')[-1]]
    if name.startswith('neck_'):
        return (-70, 70) if name.endswith('yaw') else (-40, 40)
    if 'shoulder' in name:
        return (-100, 100) if name.endswith('roll') else (-120, 120)
    if name.endswith('elbow'):
        return (0, 145)
    if 'wrist' in name:
        return (-65, 65) if name.endswith('pitch') else (-80, 80)
    if 'hip' in name:
        return {'roll': (-40, 40), 'pitch': (-105, 45), 'yaw': (-45, 45)}[name.split('_')[-1]]
    if name.endswith('knee'):
        return (0, 145)
    return (-40, 30) if name.endswith('pitch') else (-25, 25)


def add_hands(root, metadata):
    for side, hand_ref in REFERENCE['hands'].items():
        hand = root.find(f".//body[@name='{side}_hand']")
        wrist = np.array(hand_ref['wrist']['world_position_m'])
        chains = {name: np.array([p['world_position_m'] for p in points])
                  for name, points in hand_ref['fingers'].items()}
        knuckles = np.array([chains[n][0] for n in ('index', 'middle', 'ring', 'little')])
        long_axis = unit(knuckles.mean(axis=0) - wrist)
        normal = unit(np.cross(knuckles[0] - knuckles[-1], long_axis))
        if normal[2] > 0:
            normal = -normal
        # A right-handed palm frame, with +Y toward the fingers and +Z palmar.
        width_axis = unit(np.cross(long_axis, normal))
        normal = unit(np.cross(width_axis, long_axis))
        local_knuckles = (knuckles - wrist) @ np.column_stack([width_axis, long_axis, normal])
        half_width = (np.ptp(local_knuckles[:, 0]) / 2) + .005
        palm_length = float(np.mean(local_knuckles[:, 1]))
        palm_center = (width_axis * np.mean(local_knuckles[:, 0]) + long_axis * palm_length / 2)
        # The former ellipsoid covered the fingers as well as the palm; replace it.
        for geom in list(hand.findall('geom')):
            hand.remove(geom)
        ET.SubElement(hand, 'geom', name=f'{side}_palm_collision', type='box',
                      size=fmt([half_width, palm_length / 2 - .003, .009]),
                      pos=fmt(palm_center), quat=basis_quat(width_axis, long_axis, normal),
                      mass='.12', **{'class': 'collision'})
        ET.SubElement(hand, 'site', name=f'{side}_palm_center', pos=fmt(palm_center),
                      size='.003', group='4', rgba='0.2 0.9 0.3 1')
        hand.find(f"joint[@name='{side}_wrist_pitch']").set('axis', fmt(width_axis))
        hand.find(f"joint[@name='{side}_wrist_roll']").set('axis', fmt(long_axis))
        metadata['palm_frames'][side] = {'long_axis': long_axis.tolist(),
                                         'palmar_normal': normal.tolist(),
                                         'width_axis': width_axis.tolist()}
        for finger, points in chains.items():
            parent = hand
            previous = wrist
            radii = {'thumb': .006, 'index': .0055, 'middle': .0058,
                     'ring': .0053, 'little': .0045}
            for k in range(3):
                body_name = f'{side}_{finger}_{("proximal", "middle", "distal")[k]}'
                body = ET.SubElement(parent, 'body', name=body_name, pos=fmt(points[k] - previous))
                body.append(ET.Comment(f' PMX bone {hand_ref["fingers"][finger][k]["pmx_index"]}; source position retained. '))
                direction = unit(points[k + 1] - points[k])
                flex_axis = unit(np.cross(direction, normal))
                if finger == 'thumb':
                    if k == 0:
                        # Oppose toward the index/middle fingers, in a second plane.
                        toward = unit(knuckles[:2].mean(axis=0) - points[k])
                        opposition_axis = unit(np.cross(direction, toward))
                        ET.SubElement(body, 'joint', name=f'{side}_thumb_cmc_opposition',
                                      axis=fmt(opposition_axis), range='-15 90', **{'class': 'finger'})
                    label = ('cmc_flex', 'mcp_flex', 'ip_flex')[k]
                    limits = ('-20 55', '0 80', '0 80')[k]
                else:
                    if k == 0:
                        ET.SubElement(body, 'joint', name=f'{side}_{finger}_mcp_spread',
                                      axis=fmt(normal), range='-15 15', **{'class': 'finger'})
                    label = ('mcp_flex', 'pip_flex', 'dip_flex')[k]
                    limits = ('0 90', '0 105', '0 80')[k]
                ET.SubElement(body, 'joint', name=f'{side}_{finger}_{label}',
                              axis=fmt(flex_axis), range=limits, **{'class': 'finger'})
                radius = radii[finger] * (1, .86, .74)[k]
                length = np.linalg.norm(points[k + 1] - points[k])
                # Rounded capsule ends meet exactly at source joint/tip landmarks.
                start, end = direction * radius, direction * (length - radius)
                mass = 700 * (np.pi * radius ** 2 * (length - 2 * radius) + 4 / 3 * np.pi * radius ** 3)
                ET.SubElement(body, 'geom', name=body_name + '_collision', type='capsule',
                              fromto=fmt(np.r_[start, end]), size=fmt([radius]),
                              mass=f'{mass:.10g}', friction='1.0 0.01 0.001', **{'class': 'collision'})
                if k == 2:
                    ET.SubElement(body, 'site', name=f'{side}_{finger}_tip',
                                  pos=fmt(points[k + 1] - points[k]), size='.003',
                                  group='4', rgba='0.2 0.9 0.3 1')
                parent, previous = body, points[k]


def build():
    root = ET.parse(ARCHIVE / 'venti_skeleton_plain.xml').getroot()
    root.set('model', 'Venti_modeling_plain')
    root.insert(0, ET.Comment(' Modeling stage: fixed pelvis, zero gravity, no actuators. Nominal virtual masses; not hardware calibration. '))
    root.insert(1, ET.Comment(' Group 1: physical envelopes. Group 2: optional cosmetics. Group 4 sites: palm centers and fingertips. '))
    compiler = root.find('compiler')
    compiler.set('inertiafromgeom', 'true')
    compiler.set('inertiagrouprange', '1 1')
    default = root.find('default')
    default.find('joint').set('limited', 'true')
    default.find('geom').set('density', '0')
    default.find("default[@class='visual']/geom").set('group', '2')
    ET.SubElement(ET.SubElement(default, 'default', {'class': 'finger'}), 'joint',
                  damping='.005', armature='.00001', frictionloss='.0002', limited='true')
    world = root.find('worldbody')
    metadata = {'merged_bodies': {}, 'palm_frames': {}, 'modeling_only': True,
                'renamed_bodies_after_merging': {'neck': 'head'},
                'reference': '../Venti_MJCF/venti.xml', 'original_body_count': 19,
                'notes': ['PMX hand joint positions retained; hinge axes and limits are design choices.',
                          'All 29 original joint names retained; wrist axes corrected to palm frame.',
                          'Fixed base, zero gravity, robot self-contact disabled, no actuators.',
                          'Masses are nominal virtual-body estimates; inertias inferred from physical primitives.']}
    # These four nodes are welded adapters; removing them does not remove a DOF.
    for name in ('chest', 'head', 'right_clavicle', 'left_clavicle'):
        body = root.find(f".//body[@name='{name}']")
        parent = next(p for p in world.iter() if body in list(p))
        offset = vec(body.get('pos', '0 0 0'))
        metadata['merged_bodies'][name] = {'into': parent.get('name'), 'translation_m': offset.tolist()}
        ET.SubElement(parent, 'site', name=name + '_reference', pos=fmt(offset), size='.002',
                      group='4', rgba='0.7 0.3 0.8 1')
        for child in body:
            if child.tag != 'inertial':
                parent.append(translated(child, offset))
        parent.remove(body)
    # Head motion occurs at the neck pivot. Use a functional name for the merged rigid link.
    head = root.find(".//body[@name='neck']")
    head.set('name', 'head')
    for body in world.iter('body'):
        for inertia in list(body.findall('inertial')):
            body.remove(inertia)
        for joint in body.findall('joint'):
            joint.set('range', fmt(joint_range(joint.get('name'))))
    masses = {'pelvis_collision': 3.5, 'waist_collision': 2, 'chest_collision': 6,
              'head_collision': 2.5, 'neck_collision': .15}
    for side in ('right', 'left'):
        masses.update({f'{side}_{part}_collision': mass for part, mass in
                       [('upper_arm', .65), ('forearm', .45), ('thigh', 2.2), ('shin', 1.1), ('foot', .5), ('ankle', .1)]})
    for geom in world.iter('geom'):
        if geom.get('name') in masses:
            geom.set('mass', str(masses[geom.get('name')]))
    add_hands(root, metadata)
    # Joint count/order grew: create keyframes from names rather than hand-written qpos indices.
    joints = list(world.iter('joint'))
    keyframes = ET.SubElement(root, 'keyframe')
    poses = {'open': {}, 'relaxed': {}, 'fist': {}, 'point': {}}
    for joint in joints:
        name = joint.get('name')
        if not any(f'_{finger}_' in name for finger in ('thumb', 'index', 'middle', 'ring', 'little')):
            continue
        if name.endswith('spread'):
            continue
        if 'thumb' in name:
            relaxed = 5 if name.endswith('opposition') else 10
            closed = {'cmc_opposition': 44.5, 'cmc_flex': 9.6, 'mcp_flex': 29.7, 'ip_flex': 67}[
                name.split('_thumb_')[1]]
        else:
            relaxed = 12 if name.endswith('mcp_flex') else 18
            closed = 65 if name.endswith('mcp_flex') else (85 if name.endswith('pip_flex') else 55)
        poses['relaxed'][name] = relaxed
        poses['fist'][name] = closed
        poses['point'][name] = 0 if '_index_' in name else closed
    for name, pose in poses.items():
        ET.SubElement(keyframes, 'key', name=name,
                      qpos=fmt(np.deg2rad([pose.get(j.get('name'), 0) for j in joints])))
    metadata['joint_names_in_qpos_order'] = [j.get('name') for j in joints]
    metadata['robot_body_count'] = len(list(world.iter('body')))
    metadata['joint_count'] = len(joints)
    write(root, BASE / 'venti_skeleton_plain.xml')
    # Cosmetic variants share exactly the same kinematics, collision and inertial geometry.
    old = ET.parse(ARCHIVE / 'venti_skeleton.xml').getroot()
    for gray in (False, True):
        variant = copy.deepcopy(root)
        variant.set('model', 'Venti_modeling_untextured' if gray else 'Venti_modeling_color')
        collision = variant.find("default/default[@class='collision']/geom")
        collision.set('group', '3')
        collision.set('rgba', '0.2 0.65 0.95 0.35')
        variant.find('compiler').set('inertiagrouprange', '3 3')
        if not gray:
            variant.find("default/default[@class='visual']/geom").attrib.pop('rgba', None)
            asset = ET.Element('asset')
            for mat in old.findall('asset/material'):
                asset.append(copy.deepcopy(mat))
            variant.insert(list(variant).index(variant.find('worldbody')), asset)
        for body in variant.findall('.//worldbody//body'):
            for physical in list(body.findall('geom')):
                if physical.get('class') != 'collision':
                    continue
                cosmetic = copy.deepcopy(physical)
                cosmetic.set('name', physical.get('name').replace('_collision', '_visual'))
                cosmetic.set('class', 'visual')
                cosmetic.set('mass', '0')
                cosmetic.set('rgba', '0.64 0.67 0.70 1')
                if not gray:
                    name = body.get('name')
                    material = ('skin' if any(s in name for s in ['hand', 'thumb', 'index', 'middle', 'ring', 'little', 'head'])
                                else 'green_dark' if name == 'pelvis' or name.endswith('foot')
                                else 'white' if any(s in name for s in ['thigh', 'shin']) else 'ivory')
                    cosmetic.set('material', material)
                    cosmetic.attrib.pop('rgba', None)
                body.append(cosmetic)
        # A few nonphysical identity cues replace the previous 127 decorative geoms.
        merged_head = variant.find(".//body[@name='head']")
        offset = vec(old.find(".//body[@name='head']").get('pos'))
        for geom in old.findall(".//body[@name='head']/geom"):
            name = geom.get('name', '')
            if name in ('hair_back', 'beret_band', 'beret_crown', 'feather_0', 'feather_1', 'feather_2') or name.startswith(('braid_', 'eye_white_', 'eye_iris_', 'eye_pupil_')):
                g = translated(geom, offset)
                if gray:
                    g.attrib.pop('material', None)
                    g.set('rgba', '0.64 0.67 0.70 1')
                merged_head.append(g)
        torso = variant.find(".//body[@name='waist']")
        for sign in (-1, 1):
            attrs = {'class': 'visual', 'type': 'box', 'size': '.008 .055 .14',
                     'pos': fmt([-.072, sign * .06, .12]), 'euler': fmt([sign * 8, -8, 0]),
                     'name': f'cape_panel_{sign}'}
            attrs.update({'rgba': '0.64 0.67 0.70 1'} if gray else {'material': 'green'})
            ET.SubElement(torso, 'geom', attrs)
        write(variant, BASE / ('venti_skeleton_untextured.xml' if gray else 'venti_skeleton.xml'))
    (BASE / 'model_revision.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Built 3 variants: {metadata["robot_body_count"]} bodies, {len(joints)} joints.')


def write(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    build()
