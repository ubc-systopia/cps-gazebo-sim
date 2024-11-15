"""
Resaves a COLLADA (.dae) file using Blender, applying canonical transformations.

### Prerequisites:
Blender 2.8 or above installed
"""

import sys
import os
import bpy

argv = sys.argv

if "--" not in argv or len(argv) < 5:
    print("Usage: blender -b -P blender_resave.py -- <input_path> <output_path>")
    sys.exit(1)

input_file = argv[argv.index("--") + 1]
output_file = argv[argv.index("--") + 2]

if not os.path.isfile(input_file):
    print(f"Error: Input file '{input_file}' does not exist.")
    sys.exit(1)

output_dir = os.path.dirname(output_file)
if not os.path.isdir(output_dir):
    print(f"Error: Output directory '{output_dir}' does not exist.")
    sys.exit(1)

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.collada_import(filepath=input_file)
bpy.ops.object.select_all(action="SELECT")
# reset the object’s local transform values to canonical form
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
bpy.ops.wm.collada_export(filepath=output_file)
print("Resave completed successfully.")
