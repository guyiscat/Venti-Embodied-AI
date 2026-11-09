"""Convert this Venti PMX into a basic MuJoCo robot and a skinned visual variant.

Dependencies: numpy, Pillow. MuJoCo is needed only to load the resulting XML.
No Blender, MMD or PMX plug-ins are required.
"""
from pathlib import Path
from collections import defaultdict
import argparse
import csv
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from pmx_reader import PMXReader


# Body frames have identity orientation in the source rest pose.
# Joint axes use the resulting robot coordinates: X forward, Y left, Z up.
# Joint zero is the PMX A-pose. These are modeling choices, not recovered hardware.
BODY_SPECS = [
    ('pelvis', 13, None, []),
    ('waist', 11, 'pelvis', [('waist_roll', (1,0,0)), ('waist_pitch', (0,1,0)), ('waist_yaw', (0,0,1))]),
    ('chest', 12, 'waist', []),
    ('neck', 14, 'chest', [('neck_yaw', (0,0,1)), ('neck_pitch', (0,1,0))]),
    ('head', 15, 'neck', []),
    ('right_clavicle', 17, 'chest', []),
    ('right_upper_arm', 19, 'right_clavicle', [('right_shoulder_roll', (1,0,0)), ('right_shoulder_pitch', (0,1,0)), ('right_shoulder_yaw', (0,0,1))]),
    ('right_forearm', 24, 'right_upper_arm', [('right_elbow', (0,0,1))]),
    ('right_hand', 31, 'right_forearm', [('right_wrist_pitch', (0,1,0)), ('right_wrist_roll', (1,0,0))]),
    ('left_clavicle', 35, 'chest', []),
    ('left_upper_arm', 37, 'left_clavicle', [('left_shoulder_roll', (1,0,0)), ('left_shoulder_pitch', (0,1,0)), ('left_shoulder_yaw', (0,0,1))]),
    ('left_forearm', 42, 'left_upper_arm', [('left_elbow', (0,0,1))]),
    ('left_hand', 49, 'left_forearm', [('left_wrist_pitch', (0,1,0)), ('left_wrist_roll', (1,0,0))]),
    ('right_thigh', 58, 'pelvis', [('right_hip_roll', (1,0,0)), ('right_hip_pitch', (0,1,0)), ('right_hip_yaw', (0,0,1))]),
    ('right_shin', 59, 'right_thigh', [('right_knee', (0,1,0))]),
    ('right_foot', 60, 'right_shin', [('right_ankle_pitch', (0,1,0)), ('right_ankle_roll', (1,0,0))]),
    ('left_thigh', 62, 'pelvis', [('left_hip_roll', (1,0,0)), ('left_hip_pitch', (0,1,0)), ('left_hip_yaw', (0,0,1))]),
    ('left_shin', 63, 'left_thigh', [('left_knee', (0,1,0))]),
    ('left_foot', 64, 'left_shin', [('left_ankle_pitch', (0,1,0)), ('left_ankle_roll', (1,0,0))]),
]


def fmt(values):
    return ' '.join(f'{float(x):.9g}' for x in values)


def save_xml(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def convert(source, output, scale=.08):
    p = PMXReader(source)
    output = Path(output)
    meshdir = output/'assets'/'meshes'
    texdir = output/'assets'/'textures'
    skindir = output/'assets'/'skins'
    for d in (meshdir, texdir, skindir):
        d.mkdir(parents=True, exist_ok=True)

    expected = {13:'下半身', 11:'上半身', 12:'上半身2', 14:'首', 15:'頭',
                19:'右腕', 24:'右ひじ', 31:'右手首', 37:'左腕', 42:'左ひじ',
                49:'左手首', 58:'右足', 59:'右ひざ', 60:'右足首',
                62:'左足', 63:'左ひざ', 64:'左足首'}
    for i, name in expected.items():
        if p.bones[i]['name'] != name:
            raise ValueError('This converter is specific to the supplied Venti rig; bone names changed.')

    # PMX left-handed Y-up -> robot right-handed Z-up. This is a reflection,
    # so triangle winding is reversed below. PMX faces agree with PMX normals.
    def transform(array):
        a = np.asarray(array, dtype=float)
        return a[..., [2,0,1]] * np.array([-1,1,1])
    positions = transform([v['pos'] for v in p.vertices]) * scale
    normals = transform([v['normal'] for v in p.vertices])
    uv = np.asarray([v['uv'] for v in p.vertices])
    uv[:,1] = 1-uv[:,1]
    # Put the lowest source vertex exactly on the visual floor.
    offset = np.array([0,0,-positions[:,2].min()])
    positions += offset
    body_positions = {n: transform(p.bones[i]['pos'])*scale + offset
                      for n,i,_,_ in BODY_SPECS}
    selected = {i:n for n,i,_,_ in BODY_SPECS}
    override = {
        18:'right_upper_arm', 36:'left_upper_arm',
        25:'right_forearm', 26:'right_forearm', 43:'left_forearm', 44:'left_forearm',
        68:'right_thigh',69:'right_shin',70:'right_foot',71:'right_foot',
        72:'left_thigh',73:'left_shin',74:'left_foot',75:'left_foot',
        66:'right_foot',67:'left_foot', 345:'chest',350:'chest',
    }
    def resolve(i):
        visited = set()
        while i >= 0 and i not in visited:
            if i in override:
                return override[i]
            if i in selected:
                return selected[i]
            visited.add(i)
            i=p.bones[i]['parent']
        return 'pelvis'
    bone_map = {i:resolve(i) for i in range(len(p.bones))}
    names = [x[0] for x in BODY_SPECS]
    index = {n:i for i,n in enumerate(names)}
    weights = np.zeros((len(p.vertices),len(names)))
    for i,v in enumerate(p.vertices):
        for bone,w in zip(v['bones'],v['weights']):
            if w > 0 and bone >= 0:
                weights[i,index[bone_map[bone]]] += w
        if weights[i].sum() == 0:
            weights[i,0]=1
    weights /= weights.sum(axis=1, keepdims=True)

    # Base color textures only: MMD toon/sphere maps and expression morphs
    # do not have a direct equivalent in a basic MJCF visual model.
    texture_ids = sorted({m['texture'] for m in p.materials if m['texture']>=0})
    for i in texture_ids:
        im=Image.open(p.path.parent / p.textures[i])
        im.convert('RGB').save(texdir/f'tex_{i:02d}.png')

    fragments=[]
    skin_assets=[]
    material_info=[]
    start=0
    removed_degenerate=0
    for mi,m in enumerate(p.materials):
        faces=np.asarray(p.faces[start:start+m['face_count']],dtype=np.int32)[:,[0,2,1]]
        start+=m['face_count']
        # Hidden facial-expression helper panels sit inside the head in rest pose.
        # Exclude them; normal eyes/face remain fully represented.
        if m['name']=='表情':
            continue
        cross=np.cross(positions[faces[:,1]]-positions[faces[:,0]],positions[faces[:,2]]-positions[faces[:,0]])
        valid=np.linalg.norm(cross,axis=1)>1e-14
        removed_degenerate+=int((~valid).sum())
        faces=faces[valid]
        material_info.append(dict(id=mi, source_name=m['name'], texture=m['texture'],faces=len(faces)))
        owners=np.argmax(weights[faces].sum(axis=1),axis=1)
        for bi,name in enumerate(names):
            ff=faces[owners==bi]
            if not len(ff):
                continue
            ids,inverse=np.unique(ff,return_inverse=True)
            local=positions[ids]-body_positions[name]
            ff=inverse.reshape(-1,3)
            stem=f'{name}_m{mi:02d}'
            with (meshdir/f'{stem}.obj').open('w',encoding='ascii',newline='\n') as out:
                out.write('# Venti PMX visual mesh, local body coordinates in meters.\n')
                for v in local:out.write('v '+fmt(v)+'\n')
                for t in uv[ids]:out.write('vt '+fmt(t)+'\n')
                for n in normals[ids]:out.write('vn '+fmt(n)+'\n')
                for f in ff+1:
                    out.write('f '+' '.join(f'{v}/{v}/{v}' for v in f)+'\n')
            fragments.append(dict(name=stem,body=name,material=mi,vertices=len(ids),faces=len(ff)))

        # Optional smooth visual skin preserves the PMX blend weights after
        # auxiliary bones have been merged into the simplified robot links.
        ids,inverse=np.unique(faces,return_inverse=True)
        ff=inverse.reshape(-1,3)
        skinname=f'skin_m{mi:02d}'
        relevant=np.flatnonzero(weights[ids].sum(axis=0)>1e-8)
        with (skindir/f'{skinname}.skn').open('wb') as out:
            out.write(struct.pack('<4i',len(ids),len(ids),len(ff),len(relevant)))
            out.write(np.asarray(positions[ids],dtype='<f4').tobytes())
            # MuJoCo SKN texture coordinates use the image-top convention,
            # unlike the Wavefront OBJ UVs exported above.
            skin_uv=uv[ids].copy()
            skin_uv[:,1]=1-skin_uv[:,1]
            out.write(np.asarray(skin_uv,dtype='<f4').tobytes())
            out.write(np.asarray(ff,dtype='<i4').tobytes())
            for bi in relevant:
                name=names[bi]
                out.write(name.encode('ascii').ljust(40,b'\0'))
                out.write(struct.pack('<7f',*body_positions[name],1,0,0,0))
                vi=np.flatnonzero(weights[ids,bi]>0)
                out.write(struct.pack('<i',len(vi)))
                out.write(np.asarray(vi,dtype='<i4').tobytes())
                out.write(np.asarray(weights[ids[vi],bi],dtype='<f4').tobytes())
        skin_assets.append(dict(name=skinname,material=mi,vertices=len(ids),faces=len(ff)))

    joint_order=[]
    for skin in (False,True):
        root=ET.Element('mujoco',model='Venti_basic_skin' if skin else 'Venti_basic')
        root.append(ET.Comment(' PMX-derived visual / kinematic template. Physics are placeholders. '))
        root.append(ET.Comment(' Fixed base, gravity off, no contacts, no actuators. Joint zero = original A-pose. '))
        ET.SubElement(root,'compiler',angle='degree',autolimits='true',inertiafromgeom='false',
                      meshdir='assets/meshes',texturedir='assets/textures',fusestatic='false')
        ET.SubElement(root,'option',timestep='0.002',gravity='0 0 0')
        ET.SubElement(root,'statistic',center='0 0 0.77',extent='1.8')
        visual=ET.SubElement(root,'visual')
        ET.SubElement(visual,'global',azimuth='190',elevation='-8',offwidth='1000',offheight='1000')
        ET.SubElement(visual,'headlight',ambient='0.55 0.55 0.55',diffuse='0.5 0.5 0.5',specular='0 0 0')
        ET.SubElement(visual,'rgba',haze='0.92 0.94 0.96 1')
        default=ET.SubElement(root,'default')
        ET.SubElement(default,'joint',type='hinge',limited='false',damping='1',armature='0.01')
        ET.SubElement(default,'geom',contype='0',conaffinity='0',density='0',group='2')
        asset=ET.SubElement(root,'asset')
        ET.SubElement(asset,'texture',name='sky',type='skybox',builtin='gradient',
                      rgb1='0.82 0.87 0.92',rgb2='0.96 0.97 0.98',width='512',height='3072')
        for i in texture_ids:
            ET.SubElement(asset,'texture',name=f'tex_{i:02d}',type='2d',file=f'tex_{i:02d}.png')
        for m in material_info:
            attrib=dict(name=f'mat_{m["id"]:02d}',rgba='1 1 1 1',specular='0.05',shininess='0.05')
            if m['texture']>=0:attrib['texture']=f'tex_{m["texture"]:02d}'
            ET.SubElement(asset,'material',**attrib)
        if not skin:
            for f in fragments:
                ET.SubElement(asset,'mesh',name=f['name'],file=f'{f["name"]}.obj',inertia='shell')
        world=ET.SubElement(root,'worldbody')
        ET.SubElement(world,'light',name='key_light',pos='3 -3 4',dir='-3 3 -4',diffuse='0.55 0.55 0.55')
        ET.SubElement(world,'geom',name='visual_floor',type='plane',size='3 3 0.05',rgba='0.88 0.9 0.92 1',group='0')
        bodies={}
        this_joints=[]
        for name,bone,parent,joints in BODY_SPECS:
            parent_pos=body_positions[parent] if parent else np.zeros(3)
            be=ET.SubElement(bodies[parent] if parent else world,'body',name=name,
                             pos=fmt(body_positions[name]-parent_pos))
            be.append(ET.Comment(f' PMX bone {bone}: {p.bones[bone]["name"]} '))
            if not parent:
                be.append(ET.Comment(' Fixed root. Add <freejoint name="root"/> here for a floating base. '))
            # These satisfy the loader and allow editing joint positions only.
            # They are explicitly unrelated to a real robot mass distribution.
            ET.SubElement(be,'inertial',pos='0 0 0',mass='1',diaginertia='0.01 0.01 0.01')
            for jn,axis in joints:
                if 'elbow' in jn:
                    # Elbow flexion normal to the rest-pose arm direction and
                    # forward vector. This accounts for the source A-pose.
                    wrist=body_positions['left_hand' if name.startswith('left') else 'right_hand']
                    forearm=wrist-body_positions[name]
                    axis=np.cross(forearm,np.array([1.,0.,0.]))
                    axis=axis/np.linalg.norm(axis)
                ET.SubElement(be,'joint',name=jn,axis=fmt(axis))
                this_joints.append(dict(name=jn,body=name,axis=list(map(float,axis))))
            if not skin:
                for f in fragments:
                    if f['body']==name:
                        ET.SubElement(be,'geom',name=f['name'],type='mesh',mesh=f['name'],material=f'mat_{f["material"]:02d}')
            bodies[name]=be
        if skin:
            deformable=ET.SubElement(root,'deformable')
            for s in skin_assets:
                ET.SubElement(deformable,'skin',name=s['name'],file=f'../skins/{s["name"]}.skn',
                              material=f'mat_{s["material"]:02d}',group='2')
        save_xml(root,output/('venti_skin.xml' if skin else 'venti.xml'))
        joint_order=this_joints

    with (output/'bone_map.csv').open('w',encoding='utf-8-sig',newline='') as out:
        writer=csv.writer(out)
        writer.writerow(['pmx_bone_index','pmx_bone_name','mjcf_body'])
        writer.writerows((i,b['name'],bone_map[i]) for i,b in enumerate(p.bones))
    metadata=dict(source=str(p.path),source_model=p.name,source_vertices=len(p.vertices),source_faces=len(p.faces),
                  source_bones=len(p.bones),robot_bodies=len(BODY_SPECS),hinge_joints=len(joint_order),
                  meters_per_pmx_unit=scale,visual_height_m=float(np.ptp(positions[:,2])),
                  coordinates='X forward, Y left, Z up; PMX (x,y,z) -> (-z,x,y)*scale + floor offset',
                  floor_offset_m=offset.tolist(),source_materials=material_info,
                  body_positions_world={n:pos.tolist() for n,pos in body_positions.items()},
                  joints_in_qpos_order=joint_order,visual_meshes=len(fragments),visual_skins=len(skin_assets),
                  excluded_expression_faces=442,degenerate_faces_removed=removed_degenerate,
                  placeholder_mass_per_body_kg=1,placeholder_diagonal_inertia_kg_m2=[.01,.01,.01])
    (output/'conversion_info.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    rules=p.path.parent/'使用规则.txt'
    if rules.exists():
        (output/'source_model_rules.txt').write_text(rules.read_text(encoding='gb18030'),encoding='utf-8')
    print(json.dumps(metadata,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--scale',type=float,default=.08,help='Meters per PMX unit, default 0.08')
    args=parser.parse_args()
    convert(args.source,args.output,args.scale)
