# -*- mode: python -*-
# spec file for pyinstaller to build mavproxy for windows
a = Analysis(['mavproxy.py'],
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
                            'wx.lib.agw.gradientbutton'],
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
