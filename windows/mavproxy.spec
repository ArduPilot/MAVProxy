# -*- mode: python -*-
# spec file for pyinstaller to build mavproxy for windows
a = Analysis(['mavproxy.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv', 'cv2', 'wx', 'pylab', 
                            'numpy', 'dateutil', 'matplotlib',
                            'pymavlink.mavwp', 'pymavlink.mavutil', 
                            'pyreadline'],
             hookspath=None,
             runtime_hooks=None)
pyz = PYZ(a.pure)
exe = EXE(pyz,
          a.scripts,
          exclude_binaries=True,
          name='mavproxy.exe',
          debug=False,
          strip=None,
          upx=True,
          console=True )
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=None,
               upx=True,
               name='mavproxy')
