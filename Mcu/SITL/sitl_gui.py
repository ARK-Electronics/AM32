#!/usr/bin/env python3
'''
control GUI for the AM32 SITL: drives PWM/DShot input over UDP and
DroneCAN input over mcast, with live telemetry from BDShot (including
extended DShot telemetry) and DroneCAN esc.Status.

each input has an Enable checkbox that instantly starts/stops its stream,
for exercising failover between inputs. The parameter panel sets
INPUT_SIGNAL_TYPE etc over DroneCAN (needed because the DRONECAN_IN
default disables the PWM/DShot input interrupts).

built on PySide6 (Qt): pyqtgraph plots and QGraphicsScene animation
panels can be added on this foundation. Install the dependencies with
    python3 Mcu/SITL/make_gui_env.py

usage: sitl_gui.py [--port 57733] [--can-uri mcast:0]

with --control-port N the UI can additionally be driven by commands over
a localhost TCP connection (one per line), for scripted testing of the
actual UI paths:
  ds_enable 0|1, ds_type pwm|dshot300|..., ds_bidir 0|1, ds_value N,
  ds_rate N, ds_edt 0|1, zero, edt_enable, edt_disable, can_enable 0|1,
  can_value X, can_rate N, param NAME VALUE, status, quit
responses go back to the client prefixed with OK/STATUS/ERR. A client
disconnect leaves the GUI running.
--log FILE records every UI action with a timestamp; --replay FILE plays
a recording back with its original timing.
'''

import argparse
import os
import queue
import signal
import socket
import sys
import threading
import time

try:
    from PySide6.QtCore import Qt, QTimer, QRectF, QLineF
    from PySide6.QtGui import QFontDatabase, QPen, QBrush, QColor, QPainter
    from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                   QGraphicsScene, QGraphicsView,
                                   QGridLayout, QGroupBox, QHBoxLayout,
                                   QDoubleSpinBox, QLabel, QPushButton,
                                   QSlider, QSpinBox, QWidget)
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if sys.platform == 'win32':
        _venv_py = os.path.join(_here, 'venv', 'Scripts', 'python.exe')
    else:
        _venv_py = os.path.join(_here, 'venv', 'bin', 'python3')
    _run_cmd = ' '.join([_venv_py, os.path.abspath(__file__)] + sys.argv[1:])
    if os.path.exists(_venv_py):
        sys.stderr.write(
            'PySide6 is required for the SITL GUI. The GUI environment is\n'
            'already set up, run:\n'
            '    %s\n' % _run_cmd)
    else:
        sys.stderr.write(
            'PySide6 is required for the SITL GUI. Either install the packages\n'
            'from %s into the system python,\n'
            'or create the self-contained environment with:\n'
            '    python3 %s\n'
            'and then run:\n'
            '    %s\n'
            % (os.path.join(_here, 'requirements-gui.txt'),
               os.path.join(_here, 'make_gui_env.py'),
               _run_cmd))
    sys.exit(1)

try:
    import pyqtgraph as pg
    HAVE_PYQTGRAPH = True
except ImportError:
    HAVE_PYQTGRAPH = False

import glob
import math

import sitl_dshot as sd
from sitl_gui_backend import DshotPanel, CanPanel, SimStream, HAVE_DRONECAN


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=57733)
    ap.add_argument('--state-port', type=int, default=57734)
    ap.add_argument('--can-uri', default='mcast:0')
    ap.add_argument('--poles', type=int, default=14)
    ap.add_argument('--control-port', type=int, default=0,
                    help='TCP port on localhost accepting UI control commands '
                         '(for scripted tests, default off)')
    ap.add_argument('--log', metavar='FILE',
                    help='log all UI actions with timestamps, for later --replay')
    ap.add_argument('--replay', metavar='FILE',
                    help='replay a --log action file with its original timing')
    args = ap.parse_args()

    t0 = time.time()
    logf = open(args.log, 'w') if args.log else None

    def log_action(cmd):
        if logf is not None:
            logf.write('%.3f %s\n' % (time.time() - t0, cmd))
            logf.flush()

    ds = DshotPanel(args.host, args.port)
    ds.poles = args.poles
    sim = SimStream(args.host, args.state_port)

    # spawn the DroneCAN node with SIGINT ignored: its multiprocessing IO
    # child inherits the kernel-level SIG_IGN disposition, so a terminal
    # Ctrl-C only interrupts this process and the child is shut down
    # through node.close() instead of dying with a traceback
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    can = CanPanel(args.can_uri) if HAVE_DRONECAN else None
    if can is not None:
        can.started.wait(5.0)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = QWidget()
    win.setWindowTitle('AM32 SITL control')
    top = QGridLayout(win)

    # ---- PWM/DShot input panel
    f1 = QGroupBox('PWM/DShot input (udp %s:%u)' % (args.host, args.port))
    g1 = QGridLayout(f1)
    top.addWidget(f1, 0, 0)

    ds_enable = QCheckBox('Enable')

    def ds_enable_changed():
        ds.enabled = ds_enable.isChecked()
        log_action('ds_enable %d' % int(ds.enabled))

    ds_enable.toggled.connect(ds_enable_changed)
    g1.addWidget(ds_enable, 0, 0)

    g1.addWidget(QLabel('Type:'), 1, 0)
    ds_type = QComboBox()
    ds_type.addItems(sorted(sd.TYPE_NAMES.keys()))
    ds_type.setCurrentText('dshot300')

    def type_changed(*a):
        ds.ptype = sd.TYPE_NAMES[ds_type.currentText()]
        is_pwm = ds.ptype == sd.TYPE_PWM
        ds_value.setRange(1000 if is_pwm else 0, 2000 if is_pwm else 2047)
        ds_value.setValue(1000 if is_pwm else 0)
        if ds.ptype == sd.TYPE_DSHOT150:
            # checkDshot() only has detection bands for dshot300/600
            ds.status = 'note: AM32 input detection does not support dshot150, use 300/600'
        else:
            ds.status = ''
        log_action('ds_type %s' % ds_type.currentText())
        value_changed()

    ds_type.currentTextChanged.connect(type_changed)
    g1.addWidget(ds_type, 1, 1)

    ds_bidir = QCheckBox('bidir (BDShot)')

    def ds_bidir_changed():
        ds.bidir = ds_bidir.isChecked()
        log_action('ds_bidir %d' % int(ds.bidir))

    ds_bidir.toggled.connect(ds_bidir_changed)
    g1.addWidget(ds_bidir, 1, 2)

    g1.addWidget(QLabel('Throttle:'), 2, 0)
    ds_value = QSlider(Qt.Horizontal)
    ds_value.setRange(0, 2047)
    ds_value.setMinimumWidth(260)

    def value_changed(*a):
        v = ds_value.value()
        log_action('ds_value %d' % v)
        # never stream DShot values 1..47 from the throttle slider: they
        # are commands (direction, bi_direction, save, programming mode)
        # and a slider drag dwells long enough to execute them
        if ds.ptype != sd.TYPE_PWM and 0 < v < 48:
            v = 0
        ds.value = v

    # valueChanged fires for both drags and programmatic setValue()
    ds_value.valueChanged.connect(value_changed)
    g1.addWidget(ds_value, 2, 1, 1, 2)
    ds_value_label = QLabel('0')
    g1.addWidget(ds_value_label, 2, 3)

    g1.addWidget(QLabel('Rate Hz:'), 3, 0)
    ds_rate = QSpinBox()
    ds_rate.setRange(10, 4000)
    ds_rate.setValue(500)

    def rate_changed(*a):
        ds.rate = float(ds_rate.value())
        log_action('ds_rate %d' % ds_rate.value())

    ds_rate.valueChanged.connect(rate_changed)
    g1.addWidget(ds_rate, 3, 1)

    bf = QHBoxLayout()
    zero_btn = QPushButton('Zero throttle')
    zero_btn.clicked.connect(
        lambda: ds_value.setValue(1000 if ds.ptype == sd.TYPE_PWM else 0))
    bf.addWidget(zero_btn)

    ds_edt = QCheckBox('EDT (extended telemetry)')

    def ds_edt_changed():
        ds.edt_want = ds_edt.isChecked()
        log_action('ds_edt %d' % int(ds.edt_want))

    ds_edt.toggled.connect(ds_edt_changed)
    bf.addWidget(ds_edt)
    bf.addStretch(1)
    g1.addLayout(bf, 4, 0, 1, 4)

    ds_status = QLabel('arm: enable + hold zero throttle >1.5s')
    g1.addWidget(ds_status, 5, 0, 1, 4)

    # ---- DroneCAN input panel
    f2 = QGroupBox('DroneCAN input (%s)' % args.can_uri)
    g2 = QGridLayout(f2)
    top.addWidget(f2, 0, 1)

    if can is not None:
        can_enable = QCheckBox('Enable')

        def can_enable_changed():
            can.enabled = can_enable.isChecked()
            log_action('can_enable %d' % int(can.enabled))

        can_enable.toggled.connect(can_enable_changed)
        g2.addWidget(can_enable, 0, 0)

        g2.addWidget(QLabel('Throttle:'), 1, 0)
        # slider in 0..1000 -> throttle 0..1
        can_value = QSlider(Qt.Horizontal)
        can_value.setRange(0, 1000)
        can_value.setMinimumWidth(200)

        def can_value_changed(*a):
            can.throttle = can_value.value() / 1000.0
            log_action('can_value %.4f' % can.throttle)

        can_value.valueChanged.connect(can_value_changed)
        g2.addWidget(can_value, 1, 1, 1, 2)
        can_value_label = QLabel('0.00')
        g2.addWidget(can_value_label, 1, 3)

        g2.addWidget(QLabel('Rate Hz:'), 2, 0)
        can_rate = QSpinBox()
        can_rate.setRange(1, 1000)
        can_rate.setValue(50)

        def can_rate_changed(*a):
            can.rate = float(can_rate.value())
            log_action('can_rate %d' % can_rate.value())

        can_rate.valueChanged.connect(can_rate_changed)
        g2.addWidget(can_rate, 2, 1)

        can_zero_btn = QPushButton('Zero throttle')
        can_zero_btn.clicked.connect(lambda: can_value.setValue(0))
        g2.addWidget(can_zero_btn, 3, 0)

        # parameter panel
        pf = QGroupBox('parameters (set + save + restart)')
        gp = QGridLayout(pf)
        g2.addWidget(pf, 4, 0, 1, 4)
        gp.addWidget(QLabel('INPUT_SIGNAL_TYPE:'), 0, 0)
        ptype_var = QSpinBox()
        ptype_var.setRange(0, 5)
        ptype_var.setValue(1)
        gp.addWidget(ptype_var, 0, 1)
        gp.addWidget(QLabel('(0=auto 1=dshot 2=servo 5=dronecan)'), 0, 2)
        param_status = QLabel('')
        gp.addWidget(param_status, 1, 0, 1, 3)

        def param_apply():
            log_action('param INPUT_SIGNAL_TYPE %d' % ptype_var.value())
            can.set_param('INPUT_SIGNAL_TYPE', ptype_var.value())

        apply_btn = QPushButton('Apply')
        apply_btn.clicked.connect(param_apply)
        gp.addWidget(apply_btn, 0, 3)
    else:
        g2.addWidget(QLabel('pydronecan not available'), 0, 0)

    # ---- simulation panel: motor model selection and the optional high
    # rate graph/animation views fed by the SITL state stream
    f4 = QGroupBox('simulation')
    g4 = QGridLayout(f4)
    top.addWidget(f4, 2, 0, 1, 2)

    models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')

    g4.addWidget(QLabel('Motor model:'), 0, 0)
    model_combo = QComboBox()
    model_paths = {}
    for path in sorted(glob.glob(os.path.join(models_dir, '*.json'))):
        name = os.path.splitext(os.path.basename(path))[0]
        model_paths[name] = path
        model_combo.addItem(name)
    g4.addWidget(model_combo, 0, 1)

    def model_load():
        name = model_combo.currentText()
        if name in model_paths:
            log_action('model %s' % name)
            sim.load_model(model_paths[name])

    load_btn = QPushButton('Load')
    load_btn.clicked.connect(model_load)
    g4.addWidget(load_btn, 0, 2)
    model_status = QLabel('')
    g4.addWidget(model_status, 0, 3, 1, 2)

    graph_i_check = QCheckBox('Current graph')
    graph_v_check = QCheckBox('Voltage graph')
    motorview_check = QCheckBox('Motor view')
    g4.addWidget(graph_i_check, 1, 0)
    g4.addWidget(graph_v_check, 1, 1)
    g4.addWidget(motorview_check, 1, 2)
    sim_rate_label = QLabel('')
    g4.addWidget(sim_rate_label, 1, 3, 1, 2)

    # simulation speedup, logarithmic 0.001x .. 2x, for slow motion in
    # the motor view
    g4.addWidget(QLabel('Speedup:'), 2, 0)
    speed_slider = QSlider(Qt.Horizontal)
    speed_slider.setRange(0, 165)
    speed_slider.setValue(150)
    speed_slider.setMinimumWidth(200)
    speed_label = QLabel('1.000x')

    def slider_to_speedup(v):
        return 10.0 ** ((v - 150) / 50.0)

    def speed_changed(*a):
        speedup = slider_to_speedup(speed_slider.value())
        speed_label.setText('%.3fx' % speedup)
        log_action('speedup %.4f' % speedup)
        sim.set_speedup(speedup)

    speed_slider.valueChanged.connect(speed_changed)
    g4.addWidget(speed_slider, 2, 1, 1, 2)
    g4.addWidget(speed_label, 2, 3)
    speed_1x = QPushButton('1x')
    speed_1x.clicked.connect(lambda: speed_slider.setValue(150))
    g4.addWidget(speed_1x, 2, 4)

    # scope controls: sample period and window, shared by both graph
    # windows. Fine sample periods (down to the 500ns physics step,
    # where the dead time windows are visible on the phase voltages) are
    # meant to be used together with a low speedup to keep the wall
    # clock data rate sane
    sample_spin = QDoubleSpinBox()
    sample_spin.setRange(0.5, 1000.0)
    sample_spin.setValue(50.0)
    sample_spin.setSuffix(' us sample')
    sample_spin.setDecimals(1)
    g4.addWidget(sample_spin, 3, 0, 1, 2)
    window_spin = QDoubleSpinBox()
    window_spin.setRange(0.05, 2000.0)
    window_spin.setValue(50.0)
    window_spin.setSuffix(' ms window')
    window_spin.setDecimals(2)
    g4.addWidget(window_spin, 3, 2)

    def sample_changed(*a):
        sim.period_us = sample_spin.value()
        log_action('sample_us %.1f' % sample_spin.value())

    sample_spin.valueChanged.connect(sample_changed)

    def window_changed(*a):
        log_action('window_ms %.2f' % window_spin.value())

    window_spin.valueChanged.connect(window_changed)

    # the scopes: each signal set gets its own top level pyqtgraph
    # window, created lazily on first enable. Closing a window unchecks
    # its box
    SIGNAL_SETS = {
        # key -> ((label, colour, sample column), ...), y axis label/unit
        'i': ((('iu', 'r', 4), ('iv', 'g', 5), ('iw', 'b', 6),
               ('ibus', 'w', 11)), 'current', 'A'),
        'v': ((('vu', 'r', 7), ('vv', 'g', 8), ('vw', 'b', 9),
               ('vbus', 'w', 10)), 'voltage', 'V'),
    }
    graph_windows = {}

    def update_sim_enable():
        sim.enabled = (graph_i_check.isChecked() or graph_v_check.isChecked()
                       or motorview_check.isChecked())

    def graph_toggled(key, check, title):
        log_action('graph_%s %d' % (key, int(check.isChecked())))
        if check.isChecked() and key not in graph_windows:
            if not HAVE_PYQTGRAPH:
                model_status.setText('pyqtgraph not available')
                check.setChecked(False)
                return

            class GraphWindow(pg.PlotWidget):
                def closeEvent(self, ev):
                    check.setChecked(False)
                    ev.accept()

            w = GraphWindow()
            w.setWindowTitle('AM32 SITL %s' % title)
            w.resize(700, 300)
            w.addLegend(offset=(10, 10))
            w.setLabel('bottom', 'time', 's')
            defs, label, unit = SIGNAL_SETS[key]
            w.setLabel('left', label, unit)
            curves = [(w.plot(pen=pg.mkPen(color, width=1), name=name), col)
                      for name, color, col in defs]
            graph_windows[key] = (w, curves)
        if key in graph_windows:
            graph_windows[key][0].setVisible(check.isChecked())
        update_sim_enable()

    graph_i_check.toggled.connect(
        lambda: graph_toggled('i', graph_i_check, 'phase currents'))
    graph_v_check.toggled.connect(
        lambda: graph_toggled('v', graph_v_check, 'phase voltages'))

    # motor/bridge animation, created lazily on first enable
    view = None
    scene = None
    anim = {}

    def make_motor_view():
        nonlocal view, scene
        scene = QGraphicsScene(0, 0, 420, 220)
        view = QGraphicsView(scene)
        view.setRenderHint(QPainter.Antialiasing)
        view.setFixedHeight(240)
        # rotor dial
        scene.addEllipse(20, 20, 180, 180, QPen(QColor('gray'), 2))
        anim['needle'] = scene.addLine(QLineF(110, 110, 110, 30), QPen(QColor('orange'), 4))
        anim['e_needle'] = scene.addLine(QLineF(110, 110, 110, 60), QPen(QColor('cyan'), 2))
        scene.addSimpleText('rotor').setPos(95, 202)
        # bridge legs: three vertical phase bars, coloured by mode
        anim['legs'] = []
        for p, name in enumerate(('U', 'V', 'W')):
            x = 240 + p * 55
            rect = scene.addRect(QRectF(x, 40, 36, 120), QPen(Qt.NoPen), QBrush(QColor('gray')))
            anim['legs'].append(rect)
            label = scene.addSimpleText(name)
            label.setPos(x + 12, 165)
        anim['comp'] = scene.addSimpleText('')
        anim['comp'].setPos(240, 190)
        anim['rpm'] = scene.addSimpleText('')
        anim['rpm'].setPos(240, 12)
        top.addWidget(view, 4, 0, 1, 2)

    MODE_COLORS = {
        0: QColor(90, 90, 90),      # FLOAT
        1: QColor(60, 100, 220),    # LOW
        2: QColor(60, 190, 60),     # PWM
        3: QColor(60, 190, 120),    # PWM_NOCOMP
        4: QColor(220, 140, 40),    # BRAKE_PWM
    }

    def motorview_changed():
        log_action('motorview %d' % int(motorview_check.isChecked()))
        if motorview_check.isChecked() and view is None:
            make_motor_view()
        if view is not None:
            view.setVisible(motorview_check.isChecked())
        update_sim_enable()
        win.adjustSize()

    motorview_check.toggled.connect(motorview_changed)

    def update_sim_views():
        visible = [gw for gw, _ in graph_windows.values() if gw.isVisible()]
        if visible:
            win_s = window_spin.value() * 1e-3
            w = sim.window(win_s)
            if w:
                t0 = w[-1][0] - win_s
                ts = [smp[0] - t0 for smp in w]
                for gw, curves in graph_windows.values():
                    if gw.isVisible():
                        for curve, col in curves:
                            curve.setData(ts, [smp[col] for smp in w])
                        # the x axis is the window control, not auto range
                        gw.setXRange(0, win_s, padding=0)
        if motorview_check.isChecked() and view is not None:
            smp = sim.latest()
            if smp is not None:
                t, omega, theta, theta_e = smp[0], smp[1], smp[2], smp[3]
                modes, comp_ph, comp_out = smp[12], smp[13], smp[14]
                cx, cy, r = 110, 110, 80
                anim['needle'].setLine(QLineF(cx, cy, cx + r * math.sin(theta),
                                              cy - r * math.cos(theta)))
                re = r * 0.6
                anim['e_needle'].setLine(QLineF(cx, cy, cx + re * math.sin(theta_e),
                                                cy - re * math.cos(theta_e)))
                for p in range(3):
                    anim['legs'][p].setBrush(QBrush(MODE_COLORS.get(modes[p], QColor('gray'))))
                anim['comp'].setText('comparator: phase %s out=%d' % ('UVW'[comp_ph], comp_out))
                anim['rpm'].setText('%.0f rpm' % (omega * 60 / (2 * math.pi)))

    sim_view_timer = QTimer()
    sim_view_timer.timeout.connect(update_sim_views)
    sim_view_timer.start(33)

    # ---- telemetry panel
    f3 = QGroupBox('telemetry')
    g3 = QGridLayout(f3)
    top.addWidget(f3, 1, 0, 1, 2)
    fixed = QFontDatabase.systemFont(QFontDatabase.FixedFont)
    bds_label = QLabel('BDShot: -')
    bds_label.setFont(fixed)
    g3.addWidget(bds_label, 0, 0)
    can_label = QLabel('DroneCAN: -')
    can_label.setFont(fixed)
    g3.addWidget(can_label, 1, 0)

    # ---- optional TCP control interface, driving the same widgets and
    # handlers as the mouse, so scripted tests cover the UI paths.
    # Commands are one per line; OK/STATUS/ERR responses go back to the
    # issuing client. A client disconnect leaves the GUI running; the
    # quit command closes it
    cmd_queue = queue.Queue()   # (line, reply function)

    def emit(msg):
        print(msg)
        sys.stdout.flush()

    def control_client(conn):
        def reply(msg):
            try:
                conn.sendall((msg + '\n').encode())
            except OSError:
                pass
        f = conn.makefile('r')
        for line in f:
            cmd_queue.put((line, reply))
        conn.close()

    def control_server():
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', args.control_port))
        srv.listen(4)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=control_client, args=(conn,), daemon=True).start()

    def handle_command(line, reply):
        parts = line.split()
        if not parts:
            return
        cmd, cargs = parts[0], parts[1:]
        if cmd == 'ds_enable':
            ds_enable.setChecked(bool(int(cargs[0])))
        elif cmd == 'ds_type':
            ds_type.setCurrentText(cargs[0])
        elif cmd == 'ds_value':
            ds_value.setValue(int(cargs[0]))
        elif cmd == 'ds_bidir':
            ds_bidir.setChecked(bool(int(cargs[0])))
        elif cmd == 'ds_rate':
            ds_rate.setValue(int(cargs[0]))
        elif cmd == 'zero':
            ds_value.setValue(1000 if ds.ptype == sd.TYPE_PWM else 0)
        elif cmd == 'ds_edt':
            ds_edt.setChecked(bool(int(cargs[0])))
        elif cmd == 'edt_enable':
            ds_edt.setChecked(True)
        elif cmd == 'edt_disable':
            ds_edt.setChecked(False)
        elif cmd == 'can_enable' and can is not None:
            can_enable.setChecked(bool(int(cargs[0])))
        elif cmd == 'can_value' and can is not None:
            can_value.setValue(int(float(cargs[0]) * 1000))
        elif cmd == 'can_rate' and can is not None:
            can_rate.setValue(int(cargs[0]))
        elif cmd == 'param' and can is not None:
            can.set_param(cargs[0], int(cargs[1]))
        elif cmd == 'motorview':
            motorview_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'model':
            if cargs[0] in model_paths:
                model_combo.setCurrentText(cargs[0])
                model_load()
            else:
                sim.load_model(cargs[0])
        elif cmd == 'graph_i' or cmd == 'graphs':
            graph_i_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'graph_v':
            graph_v_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'signals':
            # compatibility with recordings from the single-graph UI
            if cargs[0] == 'voltages':
                graph_v_check.setChecked(True)
            else:
                graph_i_check.setChecked(True)
        elif cmd == 'sample_us':
            sample_spin.setValue(float(cargs[0]))
        elif cmd == 'window_ms':
            window_spin.setValue(float(cargs[0]))
        elif cmd == 'speedup':
            x = float(cargs[0])
            pos = int(round(150 + 50 * math.log10(max(0.001, min(2.0, x)))))
            if pos == speed_slider.value():
                speed_changed()
            else:
                speed_slider.setValue(pos)
        elif cmd == 'status':
            reply('STATUS %s' % bds_label.text())
            reply('STATUS %s' % can_label.text())
            reply('STATUS ds: %s' % (ds.status or '-'))
            reply('STATUS sim: %s rate=%.0f/s' % (sim.model_status or '-', sim.rate.hz()))
            return
        elif cmd == 'quit':
            app.quit()
            return
        else:
            reply('ERR unknown command: %s' % line.strip())
            return
        reply('OK %s' % line.strip())

    def cmd_poll():
        while True:
            try:
                line, reply = cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                handle_command(line, reply)
            except Exception as ex:
                reply('ERR %s: %s' % (line.strip(), ex))

    def replay_reader():
        with open(args.replay) as f:
            entries = []
            for line in f:
                parts = line.strip().split(None, 1)
                if len(parts) == 2:
                    entries.append((float(parts[0]), parts[1]))
        rt0 = time.time()
        for when, cmd in entries:
            delay = rt0 + when - time.time()
            if delay > 0:
                time.sleep(delay)
            cmd_queue.put((cmd + '\n', emit))
        emit('REPLAY done (%u actions)' % len(entries))

    cmd_timer = QTimer()
    cmd_timer.timeout.connect(cmd_poll)
    if args.control_port > 0:
        threading.Thread(target=control_server, daemon=True).start()
    if args.replay:
        threading.Thread(target=replay_reader, daemon=True).start()
    if args.control_port > 0 or args.replay:
        cmd_timer.start(50)

    def update():
        ds_value_label.setText(str(ds_value.value()))
        ds_status.setText(ds.status or 'arm: enable + hold zero throttle >1.5s')
        edt = ' '.join('%s=%s' % kv for kv in sorted(ds.edt_fresh().items()))
        bds_label.setText('BDShot:   rpm=%-6.0f %-8s EDT:%-3s sent=%.0f/s replies=%.0f/s badcrc=%u %s'
                          % (ds.rpm, 'spinning' if ds.spinning else 'stopped',
                             'on' if ds.edt_active() else 'off',
                             ds.sent.hz(), ds.replies.hz(), ds.badcrc, edt))
        if can is not None:
            can_value_label.setText('%.2f' % (can_value.value() / 1000.0))
            if can.error:
                can_label.setText('DroneCAN: error: %s' % can.error)
            else:
                s = can.status
                can_label.setText(
                    'DroneCAN: rpm=%-6s volt=%-5s cur=%-5s temp=%-5s err=%-3s '
                    'esc.Status=%.0f/s cmds=%.0f/s node=%s up=%us'
                    % (s.get('rpm', '-'),
                       ('%.1f' % s['voltage']) if 'voltage' in s else '-',
                       ('%.1f' % s['current']) if 'current' in s else '-',
                       ('%.0f' % s['temp']) if 'temp' in s else '-',
                       s.get('errors', '-'),
                       can.esc_rate.hz(), can.sent.hz(),
                       can.node_id, can.uptime))
            try:
                param_status.setText(can.param_result.get_nowait())
            except queue.Empty:
                pass
        model_status.setText(sim.model_status)
        if sim.enabled:
            sim_rate_label.setText('%.0f samples/s' % sim.rate.hz())
        else:
            sim_rate_label.setText('')

    update_timer = QTimer()
    update_timer.timeout.connect(update)
    update_timer.start(100)

    # graceful Ctrl-C / SIGTERM: quit the event loop. The handler runs
    # from the 100ms update timer, the next time python bytecode executes
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    signal.signal(signal.SIGTERM, lambda *a: app.quit())

    win.show()
    try:
        app.exec()
    finally:
        ds.running = False
        sim.close()
        if can is not None:
            can.running = False
            # let the CAN thread close the node and its IO child
            can.thread.join(2.0)


if __name__ == '__main__':
    main()
