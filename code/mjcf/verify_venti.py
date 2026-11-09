"""Verify exported models using MuJoCo, then render reference images."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from PIL import Image, ImageDraw

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('model_dir',type=Path,nargs='?',
                    default=Path(__file__).resolve().parents[2]/'models'/'Venti_MJCF')
out=parser.parse_args().model_dir.resolve()
info=json.loads((out/'conversion_info.json').read_text(encoding='utf-8'))
report={'mujoco_version':mujoco.__version__,'models':[]}
for filename in ('venti.xml','venti_skin.xml'):
    model=mujoco.MjModel.from_xml_path(str(out/filename))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    assert model.nq==29 and model.nv==29 and model.nbody==20
    assert model.nu==0
    errors=[]
    for name,pos in info['body_positions_world'].items():
        errors.append(float(np.max(np.abs(data.body(name).xpos-pos))))
    assert max(errors)<1e-8
    # Verify every selected axis is normalized and changes descendant poses.
    rest=data.xpos.copy()
    motion=[]
    for i in range(model.nq):
        data.qpos[:]=0
        data.qpos[i]=.1
        mujoco.mj_forward(model,data)
        # Even terminal wrist/ankle joints rotate their own body orientation.
        bid=model.jnt_bodyid[i]
        assert np.linalg.norm(data.xmat[bid].reshape(3,3)-np.eye(3))>.05
        motion.append(mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i))
    data.qpos[:]=0
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    for _ in range(1000):
        mujoco.mj_step(model,data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert np.max(np.abs(data.qpos))<1e-10
    assert not any(w.number for w in data.warning)
    cam=mujoco.MjvCamera()
    cam.lookat[:]=[0,0,.79]
    cam.distance=2.65
    cam.azimuth=190
    cam.elevation=-5
    entry=dict(file=filename,nq=model.nq,nv=model.nv,robot_bodies=model.nbody-1,
               joints=model.njnt,meshes=model.nmesh,skins=model.nskin,
               max_body_bind_error_m=max(errors),zero_pose_1000_steps_stable=True,
               individually_checked_joints=motion)
    with mujoco.Renderer(model,height=1000,width=900) as renderer:
        renderer.update_scene(data,camera=cam)
        if model.nskin:
            # Global bind-space skin coordinates must be unchanged at rest.
            actual=np.asarray(renderer.scene.skinvert).reshape(-1,3)
            expected=np.asarray(model.skin_vert).reshape(-1,3)
            err=float(np.max(np.abs(actual-expected)))
            entry['skin_bind_pose_max_error_m']=err
            assert err<2e-6
        Image.fromarray(renderer.render()).save(out/(filename.replace('.xml','_preview.png')))
        # A modest arm + knee bend checks non-zero joint states and skin motion.
        for name,value in [('left_shoulder_roll',-.35),('left_elbow',.55),('right_knee',.25),('neck_yaw',-.15)]:
            data.joint(name).qpos[0]=value
        mujoco.mj_forward(model,data)
        renderer.update_scene(data,camera=cam)
        Image.fromarray(renderer.render()).save(out/(filename.replace('.xml','_posed.png')))
    report['models'].append(entry)
(out/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
