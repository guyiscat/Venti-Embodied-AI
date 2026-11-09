"""Minimal PMX 2.0/2.1 reader for geometry, materials and skeleton."""
import struct
from pathlib import Path


class PMXReader:
    def __init__(self, path):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        self.offset = 0
        if self.take(4) != b'PMX ':
            raise ValueError('Not a PMX file')
        self.version = self.unpack('f')
        header = self.take(self.unpack('B'))
        self.encoding = 'utf-16-le' if header[0] == 0 else 'utf-8'
        self.extra_uv = header[1]
        self.sizes = dict(zip(('vertex', 'texture', 'material', 'bone', 'morph', 'rigid'), header[2:8]))
        self.name, self.name_en, self.comment, self.comment_en = [self.string() for _ in range(4)]
        self.vertices = []
        for _ in range(self.unpack('i')):
            pos = self.vec(3)
            normal = self.vec(3)
            uv = self.vec(2)
            self.take(16 * self.extra_uv)
            deform = self.unpack('B')
            if deform == 0:
                bones, weights = [self.index('bone')], [1.0]
            elif deform in (1, 3):
                bones = [self.index('bone') for _ in range(2)]
                w = self.unpack('f')
                weights = [w, 1.0 - w]
                if deform == 3:
                    self.take(36)
            elif deform in (2, 4):
                bones = [self.index('bone') for _ in range(4)]
                weights = list(self.vec(4))
            else:
                raise ValueError(f'Unsupported deform: {deform}')
            edge = self.unpack('f')
            self.vertices.append(dict(pos=pos, normal=normal, uv=uv, bones=bones, weights=weights))
        self.faces = [tuple(self.index('vertex', unsigned=True) for _ in range(3))
                      for _ in range(self.unpack('i') // 3)]
        self.textures = [self.string() for _ in range(self.unpack('i'))]
        self.materials = []
        for _ in range(self.unpack('i')):
            name, en = self.string(), self.string()
            diffuse = self.vec(4)
            specular = self.vec(3)
            shininess = self.unpack('f')
            ambient = self.vec(3)
            flags = self.unpack('B')
            self.take(20)
            texture = self.index('texture')
            sphere_texture = self.index('texture')
            sphere_mode = self.unpack('B')
            shared_toon = self.unpack('B')
            toon = self.unpack('B') if shared_toon else self.index('texture')
            memo = self.string()
            face_count = self.unpack('i') // 3
            self.materials.append(dict(name=name, name_en=en, diffuse=diffuse, specular=specular,
                                       shininess=shininess, ambient=ambient, flags=flags,
                                       texture=texture, sphere_texture=sphere_texture,
                                       sphere_mode=sphere_mode, toon=toon, memo=memo,
                                       face_count=face_count))
        self.bones = []
        for _ in range(self.unpack('i')):
            name, en = self.string(), self.string()
            pos = self.vec(3)
            parent = self.index('bone')
            layer = self.unpack('i')
            flags = self.unpack('H')
            tail = self.index('bone') if flags & 1 else self.vec(3)
            inherit = None
            if flags & 0x300:
                inherit = (self.index('bone'), self.unpack('f'))
            if flags & 0x400:
                self.take(12)
            if flags & 0x800:
                self.take(24)
            if flags & 0x2000:
                self.take(4)
            ik = None
            if flags & 0x20:
                target = self.index('bone')
                loops, limit = self.unpack('i'), self.unpack('f')
                links = []
                for _ in range(self.unpack('i')):
                    bone, limited = self.index('bone'), self.unpack('B')
                    limits = (self.vec(3), self.vec(3)) if limited else None
                    links.append((bone, limits))
                ik = dict(target=target, loops=loops, limit=limit, links=links)
            self.bones.append(dict(name=name, name_en=en, pos=pos, parent=parent, layer=layer,
                                   flags=flags, tail=tail, inherit=inherit, ik=ik))

    def take(self, n):
        result = self.data[self.offset:self.offset+n]
        if len(result) != n:
            raise EOFError(f'Unexpected EOF at {self.offset}')
        self.offset += n
        return result

    def unpack(self, fmt):
        values = struct.unpack('<' + fmt, self.take(struct.calcsize('<' + fmt)))
        return values[0] if len(values) == 1 else values

    def vec(self, n):
        return self.unpack('f' * n)

    def index(self, kind, unsigned=False):
        return self.unpack({1: 'B' if unsigned else 'b', 2: 'H' if unsigned else 'h',
                            4: 'I' if unsigned else 'i'}[self.sizes[kind]])

    def string(self):
        return self.take(self.unpack('i')).decode(self.encoding, errors='replace')


if __name__ == '__main__':
    import json
    import sys
    p = PMXReader(sys.argv[1])
    print(json.dumps(dict(version=p.version, name=p.name, name_en=p.name_en,
                         comment=p.comment, comment_en=p.comment_en,
                         vertex_count=len(p.vertices), face_count=len(p.faces),
                         textures=p.textures, materials=p.materials,
                         bones=[dict(index=i, **b) for i,b in enumerate(p.bones)]),
                     ensure_ascii=False, indent=2))
