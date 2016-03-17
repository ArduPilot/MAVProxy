# -*- mode: python -*-
# spec file for pyinstaller to build mavproxy for windows
MAVProxyAny = Analysis(['mavproxy.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv', 'cv2', 'wx', 'pylab', 
                            'numpy', 'dateutil', 'matplotlib',
                            'pymavlink.mavwp', 'pymavlink.mavutil', 
                            'pyreadline',
                            'HTMLParser', 'wx.grid', 'pygame', 'pygame.base',
                            'pygame.constants', 'pygame.version', 'pygame.rect', 'pygame.compat', 
                            'pygame.rwobject', 'pygame.surflock', 'pygame.color', 'pygame.colordict', 
                            'pygame.cdrom', 'pygame.cursors', 'pygame.display', 'pygame.draw', 
                            'pygame.event', 'pygame.image', 'pygame.joystick', 'pygame.key', 
                            'pygame.mouse', 'pygame.sprite', 'pygame.threads', 'pygame.time', 
                            'pygame.transform', 'pygame.surface', 'pygame.bufferproxy', 'wx._grid',
                            'wx.lib.agw.genericmessagedialog', 'wx.lib.wordwrap', 'wx.lib.buttons',
                            'wx.lib.embeddedimage', 'wx.lib.imageutils', 'wx.lib.agw.aquabutton', 
                            'wx.lib.agw.gradientbutton', 'wxversion'],
             hookspath=None,
             runtime_hooks=None)
MAVExpAny = Analysis(['.\\tools\\MAVExplorer.py'],
             pathex=[os.path.abspath('.')],
             # for some unknown reason these hidden imports don't pull in
             # all the needed pieces, so we also import them in mavproxy.py
             hiddenimports=['cv', 'cv2', 'wx', 'pylab', 
                            'numpy', 'dateutil', 'matplotlib',
                            'pymavlink.mavwp', 'pymavlink.mavutil', 
                            'pyreadline', 'HTMLParser', 'wx.grid', 'wx._grid',
                            'wx.lib.agw.genericmessagedialog', 'wx.lib.wordwrap', 'wx.lib.buttons',
                            'wx.lib.embeddedimage', 'wx.lib.imageutils', 'wx.lib.agw.aquabutton', 
                            'wx.lib.agw.gradientbutton', 'FileDialog', 'Dialog'],
             hookspath=None,
             runtime_hooks=None)
MERGE( (MAVProxyAny, 'mavproxy', 'mavproxy'), (MAVExpAny, 'MAVExplorer', 'MAVExplorer') )
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
