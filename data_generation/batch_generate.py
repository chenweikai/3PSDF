import subprocess
import os
import json
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description='Computes 3PSDF field from a given mesh and generates raw sampling data for network training.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("geo_dir", type=str, help='Folder containing input meshes that are ready to be batch processed.')
parser.add_argument("sdf_dir", type=str, help='Folder for the generated `.sdf` files (raw data samples for 3PSDF training).')
parser.add_argument("obj_dir", type=str, help='Folder for the reconstructed `.obj` files (reconstructed meshed from the output 3PSDF field).')
parser.add_argument("ply_dir", type=str, help='Folder for the sampling points encoded in `.ply` format.')
parser.add_argument("octree_depth", type=int, default=9, help='The depth of the octree that is used to generate sampling points. The larger the depth, the more accurate is the 3PSDF reconstruction - 6 (64^3), 7 (128^3), 8 (256^3) and 9 (512^3).')
parser.add_argument('--writeSDF', action='store_true',  help='Flag of whether to generate `.sdf` file that encodes the raw data samples for 3PSDF learning.')
parser.add_argument('--writeOBJ', action='store_true', help='Flag of whether to reconstruct the generated 3PSDF field to a mesh for debug purposes.')
parser.add_argument('--writePLY', action='store_true', help='Flag of whether to generate `.ply` file that encodes the sampling points used in the `.sdf` training data.')
parser.add_argument("--status_path", type=str, default="batch_generate_status.json", help='Filename for the status file to track the exit status of the data generation process. (1 is the successful exit status)')
args = parser.parse_args()

if os.path.isfile(args.status_path):
    with open(args.status_path, "r") as f:
        status_dict = json.load(f)
else:
    status_dict = {}

obj_paths = list(Path(args.geo_dir).rglob('*.obj'))
num_objs = len(obj_paths)
for i, obj_path in enumerate(obj_paths):
    fname = obj_path.stem
    print(f"{i + 1}/{num_objs} {fname} ", flush=True, end="")
    if fname in status_dict:
        if status_dict[fname] == 1:
            print("exists, skipping")
            continue
            
    subfolder = os.path.relpath(obj_path.parent, args.geo_dir)
    sdf_dir = os.path.join(args.sdf_dir, subfolder)
    if not os.path.isdir(sdf_dir) and args.writeSDF:
        os.makedirs(sdf_dir)
    obj_dir = os.path.join(args.obj_dir, subfolder)
    if not os.path.isdir(obj_dir) and args.writeOBJ:
        os.makedirs(obj_dir)
    ply_dir = os.path.join(args.ply_dir, subfolder)
    if not os.path.isdir(ply_dir) and args.writePLY:
        os.makedirs(ply_dir)

    status = subprocess.call(
        [
            "./build/gen_3psdf_samples",
            str(obj_path),
            os.path.join(sdf_dir, f"{fname}.sdf"),
            os.path.join(obj_dir, f"{fname}.obj"),
            os.path.join(ply_dir, f"{fname}.ply"),
            str(args.octree_depth),
            str(1) if args.writeSDF is True else str(0),
            str(1) if args.writeOBJ is True else str(0),
            str(1) if args.writePLY is True else str(0),
        ],
        stdout=subprocess.DEVNULL
    )
    print("OK" if status == 1 else "FAILED")
    status_dict[fname] = status
    with open(args.status_path, "w") as f:
        json.dump(status_dict, f, indent=4)
