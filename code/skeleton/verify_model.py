"""Validate revised geometry/kinematics and render actual MuJoCo previews.

Usage from project root: python code/skeleton/verify_model.py
Requires mujoco==3.17.0, numpy, Pillow. Does not change model XML files.
"""
from pathlib import Path
import json
import itertools
import argparse

import mujoco
import numpy as np
from PIL import Image, ImageDraw

BASE = Path(__file__).resolve().parents[2] / 'models' / 'Venti_skeleton_updated'
REF = json.loads((BASE / 'hand_reference.json').read_text(encoding='utf-8'))
REV = json.loads((BASE / 'model_revision.json').read_text(encoding='utf-8'))
FILES = ['venti_skeleton_plain.xml', 'venti_skeleton.xml', 'venti_skeleton_untextured.xml']


def load(path):
    m = mujoco.MjModel.from_xml_path(str(path.resolve()))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def assert_clean(m, d):
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert not any(w.number for w in d.warning)


def render(m, d, path, lookat, distance, azimuth, elevation, size=(900, 800)):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    option.sitegroup[:] = 0
    with mujoco.Renderer(m, height=size[1], width=size[0]) as renderer:
        renderer.update_scene(d, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE,
                        help='Directory for generated reports and previews; default is model directory.')
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    preview = output / 'previews_revision'
    preview.mkdir(exist_ok=True)
    original, original_data = load(BASE.parent / 'Venti_MJCF' / 'venti.xml')
    baseline, _ = load(BASE / 'backups/before_hand_revision_20261108/venti_skeleton_plain.xml')
    report = {'mujoco_version': mujoco.__version__, 'models': [], 'limitations': [
        'Fixed pelvis, zero gravity, no actuators: not a balance/locomotion or grasp controller test.',
        'Robot self-contact disabled; environment contact enabled.',
        'Joint ranges, palmar directions and nominal masses are virtual-body design choices.',
        'Existing Venti_MJCF meshes/skins still have rigid hands; new fingers are primitive geometry only.']}
    canonical = None
    for filename in FILES:
        m, d = load(BASE / filename)
        assert m.nbody == 46 and m.nq == m.nv == 69 and m.nu == 0
        assert m.njnt == 69 and m.jnt_limited.all()
        assert (m.body_mass[1:] > 0).all() and (m.body_inertia[1:] > 0).all()
        assert (m.body_inertia[:, 0] + m.body_inertia[:, 1] >= m.body_inertia[:, 2] - 1e-12).all()
        assert np.array_equal(m.opt.gravity, np.zeros(3))
        # Original source landmarks and all major joint anchors must remain aligned.
        anchor_errors = []
        for i in range(original.njnt):
            name = mujoco.mj_id2name(original, mujoco.mjtObj.mjOBJ_JOINT, i)
            anchor_errors.append(np.max(np.abs(d.xanchor[m.joint(name).id] - original_data.xanchor[i])))
        assert max(anchor_errors) < 2e-8
        hand_errors = []
        for side, reference in REF['hands'].items():
            for finger, points in reference['fingers'].items():
                for k, segment in enumerate(('proximal', 'middle', 'distal')):
                    actual = d.body(f'{side}_{finger}_{segment}').xpos
                    hand_errors.append(np.max(np.abs(actual - points[k]['world_position_m'])))
                actual = d.site(f'{side}_{finger}_tip').xpos
                hand_errors.append(np.max(np.abs(actual - points[3]['world_position_m'])))
        assert max(hand_errors) < 2e-8
        # All moving links have an effective axis, including finger spread/opposition.
        for i in range(m.njnt):
            mujoco.mj_resetData(m, d)
            d.qpos[m.jnt_qposadr[i]] = .1
            mujoco.mj_forward(m, d)
            bid = m.jnt_bodyid[i]
            assert np.linalg.norm(d.xmat[bid].reshape(3, 3) - np.eye(3)) > .05
        # Check positive finger flexion initially bends toward the selected palmar side.
        for side in ('left', 'right'):
            n = np.array(REV['palm_frames'][side]['palmar_normal'])
            for finger in ('thumb', 'index', 'middle', 'ring', 'little'):
                mujoco.mj_resetData(m, d)
                mujoco.mj_forward(m, d)
                start = d.site(f'{side}_{finger}_tip').xpos.copy()
                name = f'{side}_{finger}_' + ('cmc_flex' if finger == 'thumb' else 'mcp_flex')
                d.joint(name).qpos[0] = .01
                mujoco.mj_forward(m, d)
                assert np.dot(d.site(f'{side}_{finger}_tip').xpos - start, n) > 0
        pose_results = {}
        for k in range(m.nkey):
            mujoco.mj_resetDataKeyframe(m, d, k)
            mujoco.mj_forward(m, d)
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, k)
            assert (d.qpos >= m.jnt_range[:, 0] - 1e-9).all()
            assert (d.qpos <= m.jnt_range[:, 1] + 1e-9).all()
            # Finger roots intentionally join the palm; the other phalanges must stay outside.
            for side in ('left', 'right'):
                palm = m.geom(f'{side}_palm_collision').id
                for finger in ('thumb', 'index', 'middle', 'ring', 'little'):
                    for segment in ('middle', 'distal'):
                        geom = m.geom(f'{side}_{finger}_{segment}_collision').id
                        assert mujoco.mj_geomDistance(m, d, palm, geom, 1, None) >= -1e-6
                for a, b in itertools.combinations(('thumb', 'index', 'middle', 'ring', 'little'), 2):
                    for sa, sb in itertools.product(('proximal', 'middle', 'distal'), repeat=2):
                        ga = m.geom(f'{side}_{a}_{sa}_collision').id
                        gb = m.geom(f'{side}_{b}_{sb}_collision').id
                        assert mujoco.mj_geomDistance(m, d, ga, gb, 1, None) >= -1e-6
            for _ in range(1000):
                mujoco.mj_step(m, d)
            assert_clean(m, d)
            pose_results[name] = {'steps': 1000, 'warnings': 0,
                                  'max_drift_rad': float(np.max(np.abs(d.qpos - m.key_qpos[k])))}
        # Nonzero velocities exercise inertia and damping rather than just a motionless rest pose.
        mujoco.mj_resetDataKeyframe(m, d, m.key('relaxed').id)
        d.qvel[:] = np.linspace(-.03, .03, m.nv)
        for _ in range(1000):
            mujoco.mj_step(m, d)
        assert_clean(m, d)
        # Compare all physically relevant arrays across cosmetic variants.
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
        arrays = [m.body_pos, m.body_mass, m.body_ipos, m.body_inertia,
                  m.jnt_axis, m.jnt_range, m.dof_damping, m.dof_armature]
        if canonical is None:
            canonical = (names, [a.copy() for a in arrays])
        else:
            assert names == canonical[0]
            for actual, expected in zip(arrays, canonical[1]):
                np.testing.assert_allclose(actual, expected, atol=1e-12)
        mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)
        initial_contacts = d.ncon
        assert initial_contacts == 0
        # A 1-mm floor overlap must only hit the two sole boxes.
        floor_model, floor_data = load(BASE / filename)
        floor_model.body_pos[floor_model.body('pelvis').id, 2] -= .001
        mujoco.mj_forward(floor_model, floor_data)
        floor_contacts = [(mujoco.mj_id2name(floor_model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)),
                           mujoco.mj_id2name(floor_model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)))
                          for c in floor_data.contact]
        assert len(floor_contacts) >= 2
        assert all(set(pair) <= {'ground', 'left_foot_collision', 'right_foot_collision'} for pair in floor_contacts)
        # Verify a newly introduced distal finger can contact an external object.
        spec = mujoco.MjSpec.from_file(str((BASE / filename).resolve()))
        center = d.geom('left_index_distal_collision').xpos.copy()
        spec.worldbody.add_geom(name='finger_probe', type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                pos=center, size=[.004, 0, 0], contype=2, conaffinity=1)
        probe_m = spec.compile()
        probe_d = mujoco.MjData(probe_m)
        mujoco.mj_forward(probe_m, probe_d)
        probe_contacts = [tuple(mujoco.mj_id2name(probe_m, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                                for g in (c.geom1, c.geom2)) for c in probe_d.contact]
        assert any(set(pair) == {'finger_probe', 'left_index_distal_collision'} for pair in probe_contacts)
        report['models'].append({'file': filename, 'robot_bodies': m.nbody - 1, 'joints': m.njnt,
            'finger_bodies': 30, 'finger_dofs': 40, 'geoms': m.ngeom,
            'nominal_mass_kg': float(m.body_mass.sum()), 'initial_contacts': initial_contacts,
            'max_original_joint_anchor_error_m': float(max(anchor_errors)),
            'max_pmx_finger_landmark_error_m': float(max(hand_errors)),
            'individual_joint_motion_checks': 69, 'keyframe_checks': pose_results,
            'keyframes_no_middle_distal_palm_penetration': True,
            'keyframes_no_interfinger_penetration': True,
            'nonzero_velocity_1000_steps_no_warning': True,
            'floor_probe_contacts': floor_contacts, 'finger_probe_contacts': probe_contacts})
        stem = filename.removesuffix('.xml')
        render(m, d, preview / f'{stem}_front.png', [0, 0, .75], 2.6, 180, -7)
        if filename == FILES[0]:
            for azimuth, label in [(0, 'back'), (90, 'side'), (145, 'threequarter')]:
                render(m, d, preview / f'body_{label}.png', [0, 0, .75], 2.6, azimuth, -7)
            for pose in ('open', 'relaxed', 'fist', 'point'):
                mujoco.mj_resetDataKeyframe(m, d, m.key(pose).id)
                mujoco.mj_forward(m, d)
                center = d.body('left_hand').xpos + [.005, .055, -.045]
                render(m, d, preview / f'left_hand_{pose}.png', center, .25, 100, 40, (800, 700))
                center = d.body('right_hand').xpos + [.005, -.055, -.045]
                render(m, d, preview / f'right_hand_{pose}.png', center, .25, 260, 40, (800, 700))
    report['physical_arrays_match_across_variants'] = True
    report['previous_plain_geoms'] = baseline.ngeom
    (output / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    # Replace the stale 29-DOF untextured report as well.
    (output / 'validation_untextured.json').write_text(json.dumps(
        [r for r in report['models'] if r['file'] != 'venti_skeleton.xml'], ensure_ascii=False, indent=2), encoding='utf-8')
    canvas = Image.new('RGB', (1600, 760), 'white')
    draw = ImageDraw.Draw(canvas)
    for i, label in enumerate(('open', 'fist')):
        canvas.paste(Image.open(preview / f'left_hand_{label}.png'), (i * 800, 50))
        draw.text((i * 800 + 20, 15), f'LEFT HAND: {label.upper()}', fill='black')
    canvas.save(preview / 'hand_comparison.png')
    print(json.dumps({r['file']: {'bodies': r['robot_bodies'], 'joints': r['joints'],
                                 'landmark_error_m': r['max_pmx_finger_landmark_error_m']}
                      for r in report['models']}, indent=2))
    print('All checks passed. Renders:', preview)


if __name__ == '__main__':
    main()
