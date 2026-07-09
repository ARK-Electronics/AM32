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
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QFontDatabase
    from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                   QGridLayout, QGroupBox, QHBoxLayout,
                                   QLabel, QPushButton, QSlider, QSpinBox,
                                   QWidget)
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

import sitl_dshot as sd
from sitl_gui_backend import DshotPanel, CanPanel, HAVE_DRONECAN


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=57733)
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
        elif cmd == 'status':
            reply('STATUS %s' % bds_label.text())
            reply('STATUS %s' % can_label.text())
            reply('STATUS ds: %s' % (ds.status or '-'))
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
        if can is not None:
            can.running = False
            # let the CAN thread close the node and its IO child
            can.thread.join(2.0)


if __name__ == '__main__':
    main()
