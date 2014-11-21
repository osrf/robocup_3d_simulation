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

import os
import argparse
import subprocess

parser = argparse.ArgumentParser(usage='Export meshes and convert them')
parser.add_argument('-b','--blenderdir', default='/usr/bin', help='location of your blender directory')
parser.add_argument('-s', '--scriptpath', default=None, help='location of the scripts')
parser.add_argument('-f','--file', default='nao-v4.blend',help='blender file to process')

args = parser.parse_args()

if args.file.startswith('nao'):
    robot='nao'
    version='V40'
elif args.file.startswith('juliette'):
    robot='juliette'
    version = 'V1'
elif args.file.startswith('pepper'):
    robot='pepper'
elif args.file.startswith('romeo'):
    robot='romeo'
package = str(robot) + '_meshes'
emptyfile = 'untitled.blend'
cmd= 'rospack find ' +str(package)
pathmeshes = subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)[:-1]

if pathmeshes != '':
    os.system(str(args.blenderdir)+'/blender --background '+str(args.blenderdir)+'/'+str(args.file)+' -P '+str(args.scriptpath)+'/io_export_separate.py -- '+str(package)+' '+str(version))
    os.system(str(args.scriptpath)+'/normalize_meshes.py -i'+str(pathmeshes)+'/meshes/'+str(version)+'/visual')
    os.system(str(args.blenderdir)+'/blender --background -P '+str(args.scriptpath)+'/io_export_collision.py -- '+str(package)+' '+str(version))

else:
    print 'no mesh package found'

