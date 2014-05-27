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

# This file is actually unusedbut is here for the record

import xml.etree.ElementTree as ET
import os
import re
import urllib2

def main():
    if not os.path.exists('NaoV4H21.rsi2'):
        url = 'https://raw.github.com/bhuman/BHuman2013/master/Config/Scenes/NaoV4H21.rsi2'
        f = urllib2.urlopen(url)
        local_file = open(os.path.basename(url), "wb")
        local_file.write(f.read())
    root = ET.parse('NaoV4H21.rsi2').getroot()

    # Go over the different meshes
    def re_search(obj):
        return not bool(re.compile(r'[^\-0-9.]').search(obj))

    vertices = {}
    for mesh in root.findall('Vertices'):
        vertices[mesh.attrib['name']] = mesh.text.split()

    # Define the triangles that form the meshes
    triangles = {}
    for appearance in root.findall('ComplexAppearance'):
        # get the mes name
        mesh_name = appearance.attrib['name']
        vertices_set = appearance.find('Vertices').attrib['ref']
        triangles_local = appearance.find('Triangles')
        if triangles_local is not None:
            triangles[mesh_name] = (vertices_set, triangles_local.text.split())
        else:
            vertices.pop(mesh_name, None)

    # save the meshes one by one
    for mesh_name in triangles:
        file = open('%s.obj' % mesh_name, 'w')
        i = 0
        vertices_set = vertices[triangles[mesh_name][0]]
        while i < len(vertices_set):
            file.write('v')
            j = 0
            while j < 3:
                if re_search(vertices_set[i]):
                    file.write(' ' + vertices_set[i])
                    j += 1
                i += 1
                if i >= len(vertices_set):
                    break
            file.write('\n')

        i = 0
        triangles_set = triangles[mesh_name][1]
        while i < len(triangles_set):
            file.write('f')
            j = 0
            while j < 3:
                if re_search(triangles_set[i]):
                    file.write(' %i' % (int(triangles_set[i])+1))
                    j += 1
                i += 1
                if i >= len(triangles_set):
                    break
            file.write('\n')

    file.close()

if __name__ == '__main__':
    main()
