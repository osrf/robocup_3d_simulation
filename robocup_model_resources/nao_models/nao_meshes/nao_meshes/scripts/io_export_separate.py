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
 #	        Roberto Bortoletto [roberto.bortoletto@dei.unipd.it]
 # 	        Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 #

# This script parses the nao-v4.blend file from Aldebaran robotics, parses it
# into several .dae visual files and .stl collsion files to use with a URDF. 
# It also decimates the collision meshes to help with faster collision checking

import bpy
import os
import sys

sys.path.append("/usr/lib/pymodules/python2.7")

import roslib.packages

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

# Get the folder where the visual meshes will be saved

visual_mesh_dir = os.path.join(roslib.packages.get_pkg_dir(str(argv[0])), 'meshes',argv[1],'visual')

if not os.path.isdir(visual_mesh_dir):
    os.makedirs(visual_mesh_dir)

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
    if(ob.name.find('WristYaw') == -1):
    	bpy.context.scene.collada_export(filepath=os.path.join(visual_mesh_dir, ob.name + ".dae"), selected = True, include_uv_textures=True)
    else:
        bpy.context.scene.collada_export(filepath=os.path.join(visual_mesh_dir, ob.name[1:] + ".dae"), selected = True, include_uv_textures=True)

# Restore user selection 
bpy.ops.object.select_all(action="DESELECT")
for ob in sel_obs:
    ob.select = True
bpy.context.scene.objects.active = ob

print("%s meshes exported." % len(sel_obs))

bpy.ops.wm.quit_blender()
