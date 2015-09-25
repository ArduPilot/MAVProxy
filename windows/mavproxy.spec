# -*- mode: python -*-
# spec file for pyinstaller to build mavproxy for windows
mavproxy_a = Analysis(['mavproxy.py'],
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
                                     'pygame.transform', 'pygame.surface', 'pygame.bufferproxy', 'wx._grid'],
                      hookspath=None,
                      runtime_hooks=None)

mavexplorer_a = Analysis(['tools/MAVExplorer.py'],
                          pathex=[os.path.abspath('.')],
                          hookspath=None,
                          runtime_hooks=None)

MERGE((mavproxy_a, 'mavproxy', 'mavproxy'),
      (mavexplorer_a, 'mavexplorer', 'mavexplorer'))

mavproxy_pyz    = PYZ(mavproxy_a.pure)
mavexplorer_pyz = PYZ(mavexplorer_a.pure)

mavproxy_exe = EXE(mavproxy_pyz,
                   mavproxy_a.scripts,
                   exclude_binaries=True,
                   name='mavproxy.exe',
                   debug=False,
                   strip=None,
                   upx=True,
                   console=True )

mavexplorer_exe = EXE(mavexplorer_pyz,
                      mavexplorer_a.scripts,
                      exclude_binaries=True,
                      name='MAVExplorer.exe',
                      debug=False,
                      strip=None,
                      upx=True,
                      console=True )

mavproxy_coll = COLLECT(mavproxy_exe,
                        mavproxy_a.binaries,
                        mavproxy_a.zipfiles,
                        mavproxy_a.datas,
                        strip=None,
                        upx=True,
                        name='mavproxy')

mavexplorer_coll = COLLECT(mavexplorer_exe,
                           mavexplorer_a.binaries,
                           mavexplorer_a.zipfiles,
                           mavexplorer_a.datas,
                           strip=None,
                           upx=True,
                           name='mavexplorer')
