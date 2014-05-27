 #!/usr/bin/env python
 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2013-, Stefano Michieletto
 #
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   # Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   # Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   # Neither the name of the copyright holder(s) nor the names of its
 #     contributors may be used to endorse or promote products derived
 #     from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 #  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 #  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 #  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 #  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 #  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 #  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 #  POSSIBILITY OF SUCH DAMAGE.
 #
 # io_export_separate.py
 # Authors: Vincent Rabaud [vincent.rabaud@gmail.com]
 #          Séverin Lemaignan [severin.lemaignan@epfl.ch]
 #              Roberto Bortoletto [roberto.bortoletto@dei.unipd.it]
 #              Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 #

# This script parses the nao-v4.blend file from Aldebaran robotics, parses it
# into several .dae visual files and .stl collision files to use with a URDF. 
# It also decimates the collision meshes to help with faster collision checking

import bpy, sys
import os
import roslib.packages
import argparse
import collections

#parser = argparse.ArgumentParser(description='Convert meshes to lower res ones.')
#parser.add_argument('mesh', type=str, help='Path of the mesh to read', nargs='?', default=os.environ.get('mesh', ''))
#parser.add_argument('visual_dir', type=str, help='Directory of visual meshes', nargs='?', default=os.environ.get('visual_dir', ''))
#parser.add_argument('collision_dir', type=str, help='Directory of collision meshes', nargs='?', default=os.environ.get('collision_dir', ''))

#args = parser.parse_args()


args=collections.namedtuple('Options', ['mesh', 'visual_dir', 'collision_dir'])
args.mesh=os.environ.get('mesh', '')
args.visual_dir=os.environ.get('visual_dir', '')
args.collision_dir=os.environ.get('collision_dir', '')


# load the input mesh
if args.mesh.endswith('.obj'):
    print('Processing mesh %s' % args.mesh)

    # gather list of items of interest.
    candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]

    # select them only.
    for object_name in candidate_list:
        bpy.data.objects[object_name].select = True

    # remove all selected.
    bpy.ops.object.delete()

    bpy.ops.import_scene.obj(filepath=args.mesh)

    for ob in bpy.data.objects:
        ob.name=os.path.splitext(os.path.split(args.mesh)[-1])[0]

# Get the folder where the visual meshes will be saved
visual_mesh_dir = args.visual_dir
if visual_mesh_dir:
    print("Exporting visual meshes to <%s>." % visual_mesh_dir)
    if not os.path.isdir(visual_mesh_dir):
        os.makedirs(visual_mesh_dir)
else:
    print('Not creating a mesh for visualization')

# Get the folder where the collision meshes will be saved
#collision_mesh_dir = os.path.join(roslib.packages.get_pkg_dir('nao_description'), 'mesh', 'collision', 'stl')
collision_mesh_dir = args.collision_dir
print("Exporting collision meshes to <%s>." % collision_mesh_dir)
if not os.path.isdir(collision_mesh_dir):
    os.makedirs(collision_mesh_dir)

# Keep a copy of user selection
bpy.ops.object.select_by_type(type="MESH")
sel_obs = bpy.context.selected_objects[:]

for ob in bpy.data.objects:

    # Skip non-mesh objects
    if ob.type != 'MESH':
        continue

    # Clear selection
    bpy.ops.object.select_all(action="DESELECT")

    # Select single object
    ob.hide = False
    ob.select = True

    # Export single object to visual mesh (DAE)
    #bpy.context.scene.collada_export(filepath=os.path.join(visual_mesh_dir, ob.name + ".dae"), selected = True)
    if visual_mesh_dir:
        bpy.ops.export_mesh.ply(filepath=os.path.join(visual_mesh_dir, ob.name + ".ply"))

    # decimate the mesh
    bpy.context.scene.objects.active = ob
    bpy.ops.object.modifier_add(type='DECIMATE')
    mod = ob.modifiers[0]
    # TODO: can an automatic parameter be chosen ? 0.5 seems to work.
    # It needs check with MoveIt! though to see if the modfied meshes create collisions
    # It could also only be done only for some parts (like Head/Torso)
    mod.ratio = 0.5

    # Export single object to collision mesh (STL)
    bpy.ops.export_mesh.ply(filepath=os.path.join(collision_mesh_dir, ob.name + ".ply"))
    # stl files seem to be rotated by blender so we're not using them
    #bpy.ops.export_mesh.stl(filepath=os.path.join(collision_mesh_dir, ob.name + ".stl"))

    # remove the modifier to get back to normal once done
    bpy.ops.object.modifier_remove(modifier=mod.name)

# Restore user selection 
bpy.ops.object.select_all(action="DESELECT")
for ob in sel_obs:
    ob.select = True
bpy.context.scene.objects.active = ob

print("%s meshes exported." % len(sel_obs))

bpy.ops.wm.quit_blender()
