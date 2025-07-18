# -*- mode: python -*-
# spec file for pyinstaller to build mavproxy for windows
from PyInstaller.utils.hooks import collect_submodules, collect_data_files

MAVProxyAny = Analysis(['mavproxy.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv2', 'wx', 'pylab', 
                            'numpy', 'dateutil', 'matplotlib',
                            'HTMLParser', 'wx.grid', 'wx._grid', 'prompt_toolkit',
                            'wx.lib.agw.genericmessagedialog', 'wx.lib.wordwrap', 'wx.lib.buttons',
                            'wx.lib.embeddedimage', 'wx.lib.imageutils', 'wx.lib.agw.aquabutton', 
                            'wx.lib.agw.gradientbutton',
                            'six','packaging', 'packaging.version', 'packaging.specifiers',
                            'requests', 'future',
                            ] + collect_submodules('MAVProxy.modules') + 
                            collect_submodules('pymavlink') + collect_submodules('yaml') + collect_submodules('pygame'),
             datas= [ ('modules\\mavproxy_map\\data\\*.*', 'MAVProxy\\modules\\mavproxy_map\\data' ),
                      ('modules\\mavproxy_joystick\\joysticks\\*.*', 'MAVProxy\\modules\\mavproxy_joystick\\joysticks' )],
             hookspath=None,
             runtime_hooks=None,
             excludes= ['sphinx', 'docutils', 'alabaster', 'FixTk', 'tcl', 'tk', '_tkinter', 'tkinter', 'Tkinter'])
MAVExpAny = Analysis(['.\\tools\\MAVExplorer.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv2', 'wx', 'pylab', 
                            'numpy', 'dateutil', 'matplotlib',
                            'requests',
                            'prompt_toolkit', 'HTMLParser', 'wx.grid', 'wx._grid',
                            'wx.lib.agw.genericmessagedialog', 'wx.lib.wordwrap', 'wx.lib.buttons',
                            'wx.lib.embeddedimage', 'wx.lib.imageutils', 'wx.lib.agw.aquabutton', 
                            'wx.lib.agw.gradientbutton', 'FileDialog', 'Dialog',
                            ] + collect_submodules('pymavlink'),
             datas= [ ('tools\\graphs\\*.*', 'MAVProxy\\tools\\graphs' ) ],
             hookspath=None,
             runtime_hooks=None,
             excludes= ['sphinx', 'docutils', 'alabaster', 'FixTk', 'tcl', 'tk', 'Tkinter'])
MAVPicViewerAny = Analysis(['.\\tools\\mavpicviewer\\mavpicviewer.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv2', 'wx', 
                            'numpy', 'dateutil', 
                            'wx.lib.agw.genericmessagedialog', 'wx.lib.wordwrap', 'wx.lib.buttons',
                            'wx.lib.embeddedimage', 'wx.lib.imageutils', 'wx.lib.agw.aquabutton', 
                            'wx.lib.agw.gradientbutton', 'FileDialog', 'Dialog',
                            ] + collect_submodules('pymavlink'),
             datas= [],
             hookspath=None,
             runtime_hooks=None,
             excludes= ['sphinx', 'docutils', 'alabaster', 'FixTk', 'tcl', 'tk', '_tkinter', 'tkinter', 'Tkinter'])
# MERGE( (MAVProxyAny, 'mavproxy', 'mavproxy'), (MAVExpAny, 'MAVExplorer', 'MAVExplorer'), (MAVPicViewerAny, 'mavpicviewer', 'mavpicviewer') )
MAVProxy_pyz = PYZ(MAVProxyAny.pure)
MAVProxy_exe = EXE(MAVProxy_pyz,
          MAVProxyAny.scripts,
          exclude_binaries=True,
          name='mavproxy.exe',
          debug=False,
          strip=None,
          upx=True,
          console=True )
MAVProxy_coll = COLLECT(MAVProxy_exe,
               MAVProxyAny.binaries,
               MAVProxyAny.zipfiles,
               MAVProxyAny.datas,
               strip=None,
               upx=True,
               name='mavproxy')

MAVExp_pyz = PYZ(MAVExpAny.pure)
MAVExp_exe = EXE(MAVExp_pyz,
          MAVExpAny.scripts,
          exclude_binaries=True,
          name='MAVExplorer.exe',
          debug=False,
          strip=None,
          upx=True,
          console=True )
MAVExp_coll = COLLECT(MAVExp_exe,
               MAVExpAny.binaries,
               MAVExpAny.zipfiles,
               MAVExpAny.datas,
               strip=None,
               upx=True,
               name='MAVExplorer')

MAVPicViewer_pyz = PYZ(MAVPicViewerAny.pure)
MAVPicViewer_exe = EXE(MAVPicViewer_pyz,
            MAVPicViewerAny.scripts,
            exclude_binaries=True,
            name='mavpicviewer.exe',
            debug=False,
            strip=None,
            upx=True,
            console=True )
MAVPicViewer_coll = COLLECT(MAVPicViewer_exe,
               MAVPicViewerAny.binaries,
               MAVPicViewerAny.zipfiles,
               MAVPicViewerAny.datas,
               strip=None,
               upx=True,
               name='mavpicviewer')
