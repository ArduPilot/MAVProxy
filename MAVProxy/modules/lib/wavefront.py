# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
Module for parsing Wavefront objects. Currently, only a small subset of the
directives are supported.

The directives supported by the object parser are:
    - v
    - vn
    - f
    - mtlib
    - usemtl

The directives supported by the material library parser are:
    - newmtl
    - Ka r g b
    - Kd r g b
    - Ks r g b
    - Ns

Unsupported directives found while parsing are stored in the parser's
ignored_directives attribute.
'''
from __future__ import print_function

import os
try:
    import cPickle as pickle
except ImportError:
    import pickle
import threading
import sys

class Parser(object):
    def __init__(self, filename=None, string='', enable_cache=False):
        self.filename = filename
        self.string = string
        self.enable_cache = enable_cache
        self.deps = []

    def parse(self, progress_callback=None):
        if self.filename:
            return self.parse_file(progress_callback)
        return self.parse_str(progress_callback)

    def from_cache(self, progress_callback=None):
        deps_filename = self.filename + '.deps'
        if not os.path.exists(deps_filename):
            return None

        with open(deps_filename, 'r') as f:
            for line in f:
                filename = line.strip()
                if not os.path.exists(filename):
                    return None

                cache_filename = filename + '.cache'
                if not os.path.exists(cache_filename):
                    return None

                s = os.stat(filename)
                sc = os.stat(cache_filename)
                if sc.st_mtime < s.st_mtime:
                    return None

        if progress_callback:
            progress_callback(-1, -1)

        with open(self.filename + '.cache', 'r') as f:
            try:
                obj = pickle.load(f)
            except:
                print("wavefront parser: error on loading cache, falling back to parsing", file=sys.stderr)
                return None
            else:
                return obj

    def parse_file(self, progress_callback=None):
        if self.enable_cache:
            obj = self.from_cache(progress_callback=progress_callback)
            if obj:
                return obj

        s = os.stat(self.filename)
        try:
            blksize = s.st_blksize
        except AttributeError:
            blksize = 4096

        num_lines = None
        if progress_callback:
            num_lines = 0
            with open(self.filename, 'r') as f:
                try:
                    read = f.raw.read
                except AttributeError:
                    read = f.read
                blk = read(blksize)
                while blk:
                    num_lines += blk.count("\n")
                    blk = read(blksize)

        self.deps = [self.filename]

        with open(self.filename, 'r') as f:
            obj = self.parse_lines(
                f,
                num_lines=num_lines,
                progress_callback=progress_callback,
            )

            if self.enable_cache:
                # FIXME: this will make the cache files be located at the same
                # location as the source file, which might be somewhere the
                # user doesn't have write access.
                with open(self.filename + '.deps', 'w') as deps_file:
                    for filename in self.deps:
                        print(filename, file=deps_file)
                with open(self.filename + '.cache', 'w') as cache_file:
                    pickle.dump(obj, cache_file)

            return obj

    def parse_str(self, progress_callback=None):
        self.deps = []
        lines = self.string.splitlines()
        return self.parse_lines(
            lines,
            num_lines=len(lines) if progress_callback else None,
            progress_callback=progress_callback,
        )

    def parse_lines(self, lines, num_lines=None, progress_callback=None):
        obj = self.new_target()
        self.reset()

        i = 0
        for line in lines:
            line = self.filter_line(line)
            i += 1
            if not line:
                continue
            self.parse_line(line, obj)
            if progress_callback and num_lines:
                progress_callback(i, num_lines)
        return obj

    def filter_line(self, line):
        i = line.find('#')
        if i != -1:
            line = line[:i]
        return line.strip()

class ParserWorker(threading.Thread):
    def __init__(self, parser, progress_callback=None, complete_callback=None):
        super(ParserWorker, self).__init__()
        self.lock = threading.Lock()
        self.parser = parser
        self.complete_callback = complete_callback
        self.progress = 0, 0

    def run(self):
        self.obj = self.parser.parse(progress_callback=self.progress_callback)
        if self.complete_callback:
            self.complete_callback()

    def progress_callback(self, i, num_lines):
        self.lock.acquire()
        self.progress = i, num_lines
        self.lock.release()

    def get_progress(self):
        self.lock.acquire()
        p = self.progress
        self.lock.release()
        return p

class Mtl:
    def __init__(self):
        self.name = None
        self.Ka = 1.0, 1.0, 1.0
        self.Kd = 1.0, 1.0, 1.0
        self.Ks = 1.0, 1.0, 1.0
        self.Ns = 1.0

class Obj:
    def __init__(self):
        self.vertices = []
        self.normals = []
        self.faces = []
        self.materials = {}

class ObjParser(Parser):
    def reset(self):
        self.ignored_directives = set()
        self.current_mtl = Mtl()
        self.mtl_map = {}

    def new_target(self):
        return Obj()

    def parse_line(self, line, obj):
        def parse_vertex_data_ref(v):
            if not v:
                return 0
            try:
                v = int(v)
            except ValueError:
                raise Exception("vertex data references must be integers")
            if not v:
                raise Exception("vertex data references can not be zero")
            return v

        split = line.split()
        directive = split[0]
        args = split[1:]
        n = len(args)
        if directive == 'v':
            if n == 3:
                x, y, z = args
                w = 1.0
            elif n == 4:
                x, y, z, w = args
            else:
                raise Exception("wrong number of arguments for directive v")
            try:
                obj.vertices.append((float(x), float(y), float(z), float(w)))
            except ValueError:
                raise Exception("arguments for directive v must be floating point numbers")
        elif directive == 'vn':
            if n == 3:
                x, y, z = args
            else:
                raise Exception("wrong number of arguments for directive vn")
            try:
                obj.normals.append((float(x), float(y), float(z)))
            except ValueError:
                raise Exception("arguments for directive vn must be floating point numbers")
        elif directive == 'f':
            if n < 3:
                raise Exception("directive f requires at least 3 vertices")
            vertex_data = []
            for s in args:
                s = s.split('/')
                if len(s) != 3:
                    raise Exception("invalid number of references")
                v, t, n = s
                v = parse_vertex_data_ref(v)
                t = parse_vertex_data_ref(t)
                n = parse_vertex_data_ref(n)
                # TODO: address illegal statements like "f 1//1 2/2/2"
                vertex_data.append((v, t, n))
            obj.faces.append((vertex_data, self.current_mtl))
        elif directive == 'mtllib':
            if n < 1:
                raise Exception("wrong number of arguments for directive mtllib")
            d = os.path.dirname(self.filename)
            for filename in args:
                parser = MtlParser(filename=os.path.join(d, filename))
                l = parser.parse()
                self.deps += parser.deps
                for mtl in l:
                    if mtl.name in self.mtl_map:
                        continue
                    self.mtl_map[mtl.name] = mtl
        elif directive == 'usemtl':
            if n != 1:
                raise Exception("wrong number of arguments for directive usemtl")
            name = args[0]
            if name not in self.mtl_map:
                raise Exception("material %s not found" % name)
            if name not in obj.materials:
                obj.materials[name] = self.mtl_map[name]
            self.current_mtl = obj.materials[name]
        else:
            self.ignored_directives.add(directive)

class MtlParser(Parser):
    def reset(self):
        self.current_mtl = None
        self.ignored_directives = set()

    def new_target(self):
        return []

    def parse_line(self, line, mtl_list):
        def rgb():
            try:
                if n == 1:
                    r = float(args[0])
                    g = r
                    b = r
                elif n == 3:
                    r, g, b = float(args[0]), float(args[1]), float(args[2])
                else:
                    raise Exception("wrong number of arguments for directive %s" % directive)
            except ValueError:
                raise Exception("arguments for directive %s must be floating point numbers" % directive)
            else:
                return r, g, b

        def ignore_unsupported_color_statement():
           if n > 0 and args[0] in ('spectral', 'xyz'):
               self.ignored_directives.add('%s %s' % (directive, arg[0]))
               return True
           return False

        split = line.split()
        directive = split[0]
        args = split[1:]
        n = len(args)
        if directive == 'newmtl':
            if n != 1:
                raise Exception("wrong number of arguments to directive newmtl")
            self.current_mtl = Mtl()
            self.current_mtl.name = args[0]
            mtl_list.append(self.current_mtl)
        else:
            if not self.current_mtl:
                raise Exception("create a new material with newmtl before any other statement")

        if directive == 'Ka':
            if ignore_unsupported_color_statement():
                return
            self.current_mtl.Ka = rgb()
        elif directive == 'Kd':
            if ignore_unsupported_color_statement():
                return
            self.current_mtl.Kd = rgb()
        elif directive == 'Ks':
            if ignore_unsupported_color_statement():
                return
            self.current_mtl.Ks = rgb()
        elif directive == 'Ns':
            if n != 1:
                raise Exception("wrong number of arguments for directive Ns")
            try:
                self.current_mtl.Ns = float(args[0])
            except ValueError:
                raise Exception("argument for directive Ns must be a floating point number")
        else:
            self.ignored_directives.add(directive)
