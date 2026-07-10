#!/usr/bin/env python3
'''
create a self-contained python environment for the SITL GUI in
Mcu/SITL/venv and install the GUI dependencies (PySide6, pyqtgraph,
dronecan) into it. Works the same on Linux, Windows and macOS.

usage: python3 make_gui_env.py
then run the GUI with the interpreter this prints.
'''

import os
import subprocess
import sys
import venv

here = os.path.dirname(os.path.abspath(__file__))
env_dir = os.path.join(here, 'venv')
requirements = os.path.join(here, 'requirements-gui.txt')

if sys.platform == 'win32':
    python = os.path.join(env_dir, 'Scripts', 'python.exe')
else:
    python = os.path.join(env_dir, 'bin', 'python3')


def has_pip(py):
    try:
        subprocess.check_call([py, '-m', 'pip', '--version'],
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL)
        return True
    except (subprocess.CalledProcessError, OSError):
        return False


def bootstrap_pip(py):
    '''Ubuntu/Debian venvs sometimes lack pip even with with_pip=True
    (missing ensurepip package, or a half-created venv).'''
    print('bootstrapping pip into the venv ...')
    try:
        subprocess.check_call([py, '-m', 'ensurepip', '--upgrade'])
    except subprocess.CalledProcessError:
        # last resort: get-pip.py style via pip from the host
        if has_pip(sys.executable):
            subprocess.check_call(
                [sys.executable, '-m', 'pip', 'install', '--upgrade',
                 'pip', '--target',
                 os.path.join(env_dir,
                              'Lib' if sys.platform == 'win32' else
                              'lib/python%d.%d/site-packages' % sys.version_info[:2])])
        else:
            raise SystemExit(
                'venv has no pip and ensurepip failed.\n'
                'On Ubuntu/Debian install:  sudo apt install python3-venv python3-pip\n'
                'Then:  rm -rf Mcu/SITL/venv && python3 Mcu/SITL/make_gui_env.py')


if not os.path.exists(python):
    print('creating %s ...' % env_dir)
    # with_pip can still fail silently on some distros; we check below
    try:
        venv.EnvBuilder(with_pip=True).create(env_dir)
    except Exception as e:
        print('venv create with_pip failed (%s), retrying without ...' % e)
        venv.EnvBuilder(with_pip=False).create(env_dir)

if not os.path.exists(python):
    raise SystemExit('venv python not found at %s' % python)

if not has_pip(python):
    bootstrap_pip(python)
if not has_pip(python):
    raise SystemExit(
        'still no pip in %s after bootstrap.\n'
        'Try:  sudo apt install python3-venv python3-pip\n'
        '      rm -rf Mcu/SITL/venv && python3 Mcu/SITL/make_gui_env.py' % env_dir)

print('installing GUI dependencies ...')
subprocess.check_call([python, '-m', 'pip', 'install', '--upgrade',
                       '-r', requirements])

print('\nGUI environment ready. Run the GUI with:')
print('  %s %s' % (python, os.path.join(here, 'sitl_gui.py')))
