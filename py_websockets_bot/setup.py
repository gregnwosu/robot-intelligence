#! /usr/bin/env python

from distutils.core import setup
from distutils.command.install import INSTALL_SCHEMES

for scheme in INSTALL_SCHEMES.values():
    scheme['data'] = scheme['purelib']

setup(name='py_websockets_bot',
      version='0.1',
      description="A library that uses websockets to talk to the raspberry_pi_camera_bot web server",
      author='Alan Broun',
      author_email='abroun@dawnrobotics.co.uk',
      url='',
      data_files=[('py_websockets_bot/websocket', ['py_websockets_bot/websocket/cacert.pem']),
          ('py_websockets_bot/websocket', ['py_websockets_bot/websocket/__init__.py']) ],
      packages=['py_websockets_bot'],
     )
