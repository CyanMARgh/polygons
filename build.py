#!/usr/bin/python3
import os

sources = [
	["main.cpp", "demos.h"],
	["demo0.cpp", "demos.h", "polygon.h", "plot.h", "transforms.h"],
	["demo1.cpp", "demos.h", "polygon.h", "plot.h", "transforms.h"],
	["demo2.cpp", "demos.h", "tree.h"],
	["demo3.cpp", "demos.h", "primitives.h", "polygon.h", "transforms.h", "plot.h"],
	["demo4.cpp", "demos.h", "polygon.h", "transforms.h", "plot.h", "primitives.h"],
	["hull.cpp", "polygon.h", "primitives.h"],
	["mass.cpp", "polygon.h"],
	["plot.cpp", "plot.h", "polygon.h", "primitives.h"],
	["polygon.cpp", "polygon.h", "primitives.h"],
	["primitives.cpp", "primitives.h", "polygon.h"],
	["transforms.cpp", "transforms.h"],
	["tree.cpp", "tree.h", "utils.h"],
	["utils.cpp", "utils.h", "transforms.h"]
]

flags = "-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3"
compiler = "g++"
build_folder = "build/"
tmp_folder = "tmp/"
folder_marker = 'i'
mf_name=".mfpy.make"

def get_obj_name(source_name):
	return tmp_folder + source_name.replace('/', '_').replace(".cpp", ".o")
def target(file):
	source_name = file[0]
	obj_name = get_obj_name(source_name)
	result  = obj_name + ": "
	for s in file: result += s + ' '
	result += tmp_folder + folder_marker + "\n\t" + compiler + " -c " + source_name + " -o " + obj_name + " $(FLAGS)\n"
	return result
def get_makefile_string():
	result = "FLAGS=" + flags + "\nall: " + build_folder + "app\nbuild/app:"
	objs = ""
	for e in sources: objs += ' ' + get_obj_name(e[0])
	result += objs + "\n\t" + compiler + objs + " -o " + build_folder + "app $(FLAGS)\n";
	for e in sources: result += target(e)
	result += tmp_folder + "i:\n\tmkdir tmp -p\n\tmkdir build -p\n\ttouch " + tmp_folder + "i\n" 	
	return result
def build:
	with open(mf_name, 'w') as f: f.write(get_makefile_string())
	os.system("make -f " + mf_name)


build()