#!/usr/bin/env python
#
# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This file takes the official binary Ogre meshes from the folder
# share/alrobotmodel/meshes/nao from simulator-sdk-1.14.5-linux64.tar.gz at
# https://community.aldebaran-robotics.com/resources/file/1.14.5/simulator-sdk-1.14.5-linux64.tar.gz
# changes them to .xml Ogre and then to .obj. It also creates
# a simpler version of the meshes for collision checking

import xml.etree.ElementTree as ET
import os
import re
import urllib2
from subprocess import Popen

NAO_FOLDER = 'nao'

class MeshConverter:
    # where we have all the data from ogre_xml_path
    OUT_FOLDER = 'meshes'
    ASSIMP_EXEC = '/home/vrabaud/software/assimp/build/tools/assimp_cmd/assimp'

    def __init__(self, nao_type):
        self.collision_folder = os.path.join(self.OUT_FOLDER, nao_type, 'collision')
        if not os.path.exists(self.collision_folder):
            os.makedirs(self.collision_folder)
        self.visual_folder = os.path.join(self.OUT_FOLDER, nao_type, 'visual')
        if not os.path.exists(self.visual_folder):
            os.makedirs(self.visual_folder)

    def process(self, mesh_relative_path):
        paths = os.path.split(mesh_relative_path)
        ogre_xml_path = os.path.join(os.getcwd(), mesh_relative_path + '.xml')
        # First convert the mesh to .xml Ogre
        a = Popen('OgreXMLConverter %s %s' % (mesh_relative_path, ogre_xml_path), shell=True)
        a.wait()
        # Convert to an obj
        obj_path = os.path.join(os.getcwd(), self.visual_folder, os.path.splitext(paths[-1])[0] + '.obj')
        a = Popen('%s export %s %s' % (self.ASSIMP_EXEC, ogre_xml_path, obj_path), shell=True)
        a.wait()
        # simplify the mesh for collision
        new_env = os.environ.copy()
        new_env['collision_dir']=self.collision_folder
        new_env['mesh']=obj_path
        a = Popen('blender -b -P ./scripts/split_blender.py', shell=True, env=new_env)
        a.wait()
        # clean temporary files
        os.remove(ogre_xml_path)
        

def main():
#    for nao_type in [ 'V32', 'V32T2', 'V33T14', 'V40', 'V40T2', 'V32T14', 'V33', 'V33T2', 'V40T14' ]:
    for nao_type in [ 'V40' ]:
        path = os.path.join(NAO_FOLDER, nao_type)
        mesh_converter = MeshConverter(nao_type)
        for file in next(os.walk(path))[2]:
            if file.endswith('.mesh'):
                mesh_converter.process(os.path.join(path, file))
        # Deal with some meshes independently
        mesh_converter.process(os.path.join(NAO_FOLDER, 'Phalax.mesh'))
        mesh_converter.process(os.path.join(NAO_FOLDER, 'PhalaxEnd.mesh'))
                

if __name__ == '__main__':
    main()
