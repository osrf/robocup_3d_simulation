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
 # io_export_collision.py
 # Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# This script import one by one each collada file in a folder, 
# decimate the meshes and export them as stl files
# This is an Aldebaran specific scripts which suppose that you have the package nao_meshes installed

import bpy
import os
import sys

sys.path.append("/usr/lib/pymodules/python2.7")

import roslib.packages

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

# Get the folder where the visual meshes will be saved
visual_mesh_dir = os.path.join(roslib.packages.get_pkg_dir(str(argv[0])), 'meshes',argv[1],'visual')
# Get the folder where the collision meshes will be saved
collision_mesh_dir = os.path.join(roslib.packages.get_pkg_dir(str(argv[0])), 'meshes',argv[1],'collision')
print("Exporting collsion meshes to <%s>." % collision_mesh_dir)
if not os.path.isdir(collision_mesh_dir):
    os.makedirs(collision_mesh_dir)

scene = bpy.context.scene
#delete all existing objects in the blender file
for ob in scene.objects:
    if ob.type == 'MESH' or ob.name == "Camera":
        ob.select = True
    else: 
        ob.select = False

bpy.ops.object.delete()

file_list = sorted(os.listdir(visual_mesh_dir))
for file in file_list:
# import the visual meshes one by one
    if file.endswith('.dae') == True:
        print(str(visual_mesh_dir + '/' + file))
        bpy.ops.wm.collada_import(filepath= str(visual_mesh_dir + '/' + file))

        # decimate them
        bpy.ops.object.modifier_add(type='DECIMATE')
        mod = bpy.context.scene.objects.active.modifiers[0]
        mod.ratio = 0.1

        # apply decimation 
        bpy.ops.object.modifier_apply(apply_as='DATA')
        # export them
        bpy.ops.export_mesh.stl(filepath=os.path.join(collision_mesh_dir, file[0:file.find('.dae')] + ".stl"))
        #bpy.ops.wm.collada_export(filepath=os.path.join(collision_mesh_dir, file), selected = True)
        # delete them
        bpy.ops.object.delete()

bpy.ops.wm.quit_blender()
