import os
from collections import defaultdict
from math import isnan

def walklevel(input_dir, depth=1, is_folder=False):
    stuff = os.path.abspath(os.path.expanduser(os.path.expandvars(input_dir)))
    output = []
    for root, dirs, files in os.walk(stuff):
        if root[len(stuff):].count(os.sep) == depth:
            if is_folder:
                output.append(root)
            else:
                for f in files:
                    output.append(os.path.join(root, f))
    return output


def save_obj_mesh_filterNAN(verts, faces, output_path):
    indexMap = defaultdict(list)
    file = open(output_path, 'w')

    idx = 0
    for i, v in enumerate(verts):
        if isnan(v[0]) or isnan(v[1]) or isnan(v[2]):
            indexMap[i] = -1
        else:
            idx += 1
            indexMap[i] = idx
            file.write('v %.4f %.4f %.4f\n' % (v[0], v[1], v[2]))

    for i, f in enumerate(faces):
        vid0 = f[0]
        vid1 = f[1]
        vid2 = f[2]
        newVid0 = indexMap[vid0]
        newVid1 = indexMap[vid1]
        newVid2 = indexMap[vid2]
        if newVid0 == -1 or newVid1 == -1 or newVid2 == -1:
            continue
        file.write('f %d %d %d\n' % (newVid0, newVid1, newVid2))
    file.close()


def load_filelist(path):
    fopen = open(path, 'r', encoding='utf-8')
    lines = fopen.readlines()
    filelist = []
    linecount = 0
    for line in lines:
        filelist.append(line.strip('\n'))
        linecount = linecount + 1
    fopen.close()
    return filelist
