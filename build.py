#!/usr/bin/python3

import os

def get_sources(blacklist = {}, path = './'):
	walker = next(os.walk(path))
	dirs, files = walker[1], walker[2]
	result = []
	for d in dirs:
		if d not in blacklist:
			result += get_sources(blacklist, path + d + '/')
	for f in files:
		if f.endswith('.cpp'):
			result.append(path + f)
	return result
def get_headers(file):
	ignore = {"#pragma"}
	headers = set()
	with open(file, 'r') as file:
		for line in file:
			words = line.split()
			if len(words) == 0 or words[0] in ignore: continue
			if words[0] == "#include" and len(words) == 2:
				header = words[1]
				if header[0] == '"':
					headers.add(header[1:-1])
			else: break
	return headers
def get_headers_recursive(file):
	headers = get_headers(file)
	all_headers = headers
	for h in headers:
		all_headers = set.union(all_headers, get_headers_recursive(h))
	return all_headers
def get_hedaer_dirs_dict(blacklist = set(), path = "./"):
	walker = next(os.walk(path))
	dirs, files = walker[1], walker[2]
	result = {}
	for d in dirs:
		if d not in blacklist:
			result.update(get_hedaer_dirs_dict(blacklist, path + d + '/'))
	for f in files:
		if f.endswith('.h'):
			result[f] = path
	return result
def get_required_paths(header_dirs_dict, headers):
	result = set()
	for h in headers:
		result.add(header_dirs_dict[h])
	return result
def get_obj_name(source_name):
	return (source_name)[2:-4].replace('/', '_') + '.o'
def get_obj_dict(sources):
	obj_dict = {}
	for s in sources:
		obj_dict[s] = get_obj_name(s)
	return obj_dict
def default_target(header_dirs_dict, obj_dict, source_name, flags, marker = "i", tmp_folder = "tmp/", compiler = "g++"):
	headers = get_headers_recursive(source_name)
	header_folders = get_required_paths(header_dirs_dict, headers)
	obj_name = obj_dict[source_name]
	result = tmp_folder + obj_name + ": " + source_name + " " + tmp_folder + marker
	for h in headers:
		result += ' ' + header_dirs_dict[h] + h
	result += '\n\t' + compiler + " -c " + source_name
	result += " -o " + tmp_folder + obj_name
	for hf in header_folders:
		result += " -I " + hf
	result += ' ' + flags + '\n'
	return result
def app_target(sources, obj_dict, app_name = "app", build_folder = "build/", tmp_folder = "tmp/", flags = "$(FLAGS)", compiler = "g++"):
	objs = ""
	for s in sources: objs += " " + tmp_folder + obj_dict[s]
	result = build_folder + app_name + ":" + objs + "\n\t"
	result += compiler + " -o " + build_folder + app_name + objs + " " + flags
	return result
def initial_target(marker = "i", build_folder = "build/", tmp_folder = "tmp/"):
	return tmp_folder + marker + ":\n\tmkdir " + tmp_folder + " -p \n\tmkdir " + build_folder + " -p\n\ttouch " + tmp_folder + marker + "\n" 

def generate_makefile():
	result = "FLAGS=-lsfml-window -lsfml-system -lsfml-graphics -lpthread -std=c++17 -lm -O3\nall: build/app\n\n"

	sources = get_sources({"build", "tmp"})
	header_dirs_dict = get_hedaer_dirs_dict({"build", "tmp"})
	obj_dict = get_obj_dict(sources)
	flags = "$(FLAGS)"

	result += app_target(sources, obj_dict) + "\n"
	for s in sources:
		result += default_target(header_dirs_dict, obj_dict, s, flags)
	result += initial_target()

	return result

mf_name = "mfpy.make"
def build():
	with open(mf_name, 'w') as f: f.write(generate_makefile())
	os.system("make -f" + mf_name)

build()
