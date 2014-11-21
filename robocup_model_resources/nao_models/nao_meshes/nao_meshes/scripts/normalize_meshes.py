#! /usr/bin/env python

 #  Copyright (c) 2013-, Mikael Arguedas 
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

# This script open a blender file, export all meshes to collada, decimate them with a factor 10 and then export the decimeted meshes as stl files (collision files)
# This is an Aldebaran specific scripts which suppose that you have the package nao_meshes installed

import argparse
from xml.dom.minidom import parse, parseString
import os

parser = argparse.ArgumentParser(usage='convert offset of all dae files in a directory')
parser.add_argument('-i','--input', default=None, help='inputDirectory')

args = parser.parse_args()
if args.input is None:
    if not os.path.isdir(args.input):
        directory = os.getcwd()
    else:
        directory = args.input
else:
    directory = args.input

file_list = sorted(os.listdir(directory))
for file in file_list:
    if file.endswith('.dae'):
        dom = parse(directory + '/' + file)
        for node in dom.getElementsByTagName('matrix'):
            if node.nodeName == 'matrix':
                node.firstChild.nodeValue = "0.01 0 0 0 0 0.01 0 0 0 0 0.01 0 0 0 0 1"
        for node in dom.getElementsByTagName('init_from'):
            if node.firstChild.nodeValue.startswith('/'):
                node.firstChild.nodeValue= '../../../texture/' +str(node.firstChild.nodeValue[node.firstChild.nodeValue.rfind('/')+1:])
        print 'processing ' + file
        f = open(directory+ '/' + file,'w')
        f.write(dom.toxml())
        f.close()


