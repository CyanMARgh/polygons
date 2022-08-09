#!/usr/bin/python3

import os

blacklist = ["tmp", "build", ".git"]
compiler = "g++"

def read_entire_file(filename, files_dict):
	path = files_dict[filename]
	F = open(path + filename, 'r')
	result = ""
	for line in F:
		result += line
	return result
def find_files(blacklist = dict(), path = "./", depth = 5):
	result = dict()
	if(depth > 0):
		walker = next(os.walk(path))
		dirs, files = walker[1], walker[2]
		for f in files:
			result[f] = path
		for d in dirs:
			fullpath = path + d + "/"
			if d not in blacklist:
				result.update(find_files(blacklist, fullpath, depth - 1))
	return result
def get_sources(files_dict):
	result = []
	for k in files_dict:
		if k.endswith(".cpp"):
			path = files_dict[k]
			obj_name = (path + k)[2:-3].replace('/', '_') + "o"
			result.append((k, obj_name))
	return result
def get_headers(filename, files_dict):
	ignore = {"#pragma"}
	headers = set()
	path = files_dict[filename]
	F = open(path + filename, 'r')
	for line in F:
		words = line.split()
		lw = len(words)
		if lw == 0 or words[0] in ignore: continue
		if words[0] == "#include":
			if lw != 2: raise NameError("invalid include in file: " + filename)
			w1 = words[1]
			if w1[0] == '"' and w1[-1] == '"':
				headers.add(w1[1:-1])
			elif w1[0] != '<' or w1[-1] != ">":
				raise NameError("invalid include in file: " + filename)
		else: break	 
	return headers
def get_headers_recursive(file, files_dict):
	headers = get_headers(file, files_dict)
	total = headers.copy()
	for h in headers:
		total.update(get_headers_recursive(h, files_dict))
	return total
def make_build_dirs():
	walker = next(os.walk("./"))
	dirs = walker[1]
	if "build" not in dirs:
		os.system("mkdir build")
	if "tmp" not in dirs:
		os.system("mkdir tmp")
def compile_obj(source, headers, files_dict, flags):
	include_paths = set()
	for h in headers:
		include_paths.add(files_dict[h])
	command = compiler + " -c " + files_dict[source[0]] + source[0] + " -o ./tmp/" + source[1]
	for ip in include_paths:
		command += " -I " + ip
	command += " " + get_flags(flags, source[0])
	print(command)
	os.system(command)
def compile_total(sources, files_dict, flags):
	command = compiler + " -o ./build/app"
	for s in sources:
		command += " ./tmp/" + s[1]
	command += " " + get_flags(flags, "result")
	print(command)
	os.system(command)
def is_obj_fresh(source, headers, files_dict):
	walker = next(os.walk("./tmp/"))
	src_name = source[0]
	obj_name = source[1]
	files = walker[2]
	if obj_name not in files:
		return False
	target_mtime = os.path.getmtime("./tmp/" + obj_name)

	if os.path.getmtime(files_dict[src_name] + src_name) > target_mtime: return False
	for h in headers:
		if os.path.getmtime(files_dict[h] + h) > target_mtime: return False
	return True
def is_app_fresh(sources, files_dict):
	walker = next(os.walk("./build/"))
	files = walker[2]
	if "app" not in files:
		return False
	target_mtime = os.path.getmtime("./build/app")

	for s in sources:
		if os.path.getmtime("./tmp/" + s[1]) > target_mtime: return False
	return True
def full_compile():
	make_build_dirs()
	files_dict = find_files(blacklist)
	flags = get_all_flags(files_dict, opt_file_name = 'FLAGS')
	sources = get_sources(files_dict)
	for s in sources:
		hs = get_headers_recursive(s[0], files_dict)
		if not is_obj_fresh(s, hs, files_dict):
			compile_obj(s, hs, files_dict, flags)
	if not is_app_fresh(sources, files_dict):
		compile_total(sources, files_dict, flags)

# -> (default_flags, map<source, flags>)
def get_all_flags(files_dict, opt_file_name = 'FLAGS'):
	path = files_dict[opt_file_name]
	F = open(path + opt_file_name, 'r')

	def_flags = ""
	extra_flags = dict()

	for line in F:
		flags = line.split()
		w0 = flags[0]
		if len(w0) == 0: continue
		flags = ' '.join(flags[1:]) + ' '
		if w0 == 'result':
			extra_flags['result'] = extra_flags.get('result', '') + flags
		elif w0 == 'all':
			def_flags += flags
		elif w0 in files_dict:
			extra_flags[w0] = extra_flags.get(w0, '') + flags
		else:
			raise NameError("invalid target: " + w0)
	return (def_flags, extra_flags)

def get_flags(flags, target):
	return flags[0] + flags[1].get(target, '')

# files_dict = find_files(blacklist)
# flags = get_all_flags(files_dict)
# print(flags)
# print(get_flags(flags, 'main.cpp'))


full_compile()
