'''
UI-independent backends for the AM32 SITL control GUI: the PWM/DShot
sender, the DroneCAN node and rate counters. These run in their own
threads and expose plain attributes/queues, so they can be driven by any
front end or by headless tests without a display.
'''

import queue
import threading
import time

import sitl_dshot as sd

try:
    import dronecan
    HAVE_DRONECAN = True
except ImportError:
    HAVE_DRONECAN = False


class RateCounter(object):
    def __init__(self):
        self.count = 0
        self.rate = 0.0
        self.last = time.time()
        self.last_count = 0

    def tick(self, n=1):
        self.count += n

    def hz(self):
        now = time.time()
        dt = now - self.last
        if dt >= 1.0:
            self.rate = (self.count - self.last_count) / dt
            self.last = now
            self.last_count = self.count
        return self.rate


class DshotPanel(object):
    '''PWM/DShot sender thread + state'''

    def __init__(self, host, port):
        self.port = sd.InputPort(host, port)
        self.enabled = False
        self.ptype = sd.TYPE_DSHOT300
        self.bidir = False
        self.telem_bit = False
        self.value = 0          # dshot value or pwm width
        self.rate = 500.0
        self.cmd_queue = queue.Queue()
        self.sent = RateCounter()
        self.replies = RateCounter()
        self.rpm = 0.0
        self.spinning = False
        self.badcrc = 0
        self.edt = {}           # kind -> (value, time received)
        self.edt_want = False
        self.last_edt_seen = 0.0
        self.last_edt_cmd = 0.0
        self.poles = 14
        self.status = ''
        self.running = True
        threading.Thread(target=self._sender, daemon=True).start()

    def _sender(self):
        next_send = time.time()
        while self.running:
            now = time.time()
            if not self.enabled:
                next_send = now
                time.sleep(0.02)
                self._collect()
                continue
            if now < next_send:
                time.sleep(next_send - now)
            next_send += 1.0 / max(1.0, self.rate)
            try:
                cmd = self.cmd_queue.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd is not None:
                self.port.send_dshot(cmd, ptype=self.ptype, telem=True, bidir=self.bidir)
            elif self.ptype == sd.TYPE_PWM:
                self.port.send_pwm(int(self.value))
            else:
                self.port.send_dshot(int(self.value), ptype=self.ptype,
                                     telem=self.telem_bit, bidir=self.bidir)
            self.sent.tick()
            self._collect()
            self._edt_maintain(now)
        self.port.close()

    def _collect(self):
        for r in self.port.get_replies():
            self.replies.tick()
            kind, val = sd.decode_reply(r[3], edt_expected=True)
            if kind == 'erpm':
                self.spinning = val < 65408
                self.rpm = sd.erpm_period_to_rpm(val, self.poles)
            elif kind == 'badcrc':
                self.badcrc += 1
            else:
                self.last_edt_seen = time.time()
                if kind == 'edt' and val in (0xE00, 0xEFF):
                    # EDT init/deinit acknowledgement frames, not data
                    continue
                self.edt[kind] = (val, time.time())

    def edt_active(self):
        '''true when EDT frames are actually arriving from the ESC'''
        return time.time() - self.last_edt_seen < 3.0

    def edt_fresh(self, max_age=15.0):
        '''EDT values received recently. The age allows for the slow EDT
        schedule at low frame rates (temp/voltage every ~400 replies)'''
        if not self.edt_active():
            return {}
        now = time.time()
        return {k: v for k, (v, t) in self.edt.items() if now - t < max_age}

    def send_command(self, cmd, count=8):
        for _ in range(count):
            self.cmd_queue.put(cmd)

    def _edt_maintain(self, now):
        '''EDT is a maintained state: the firmware only processes DShot
        commands while armed with the motor stopped, silently discarding
        them otherwise, and a reboot clears EDT. Keep (re)sending the
        enable/disable command until the reply stream matches the
        requested state'''
        if self.ptype == sd.TYPE_PWM or not self.bidir:
            return
        active = self.edt_active()
        if self.edt_want == active:
            if self.edt_want and self.status.startswith('EDT'):
                self.status = ''
            return
        if int(self.value) != 0 or self.spinning:
            if self.edt_want:
                self.status = 'EDT pending: needs the motor stopped at zero throttle'
            return
        if now - self.last_edt_cmd > 1.5:
            self.last_edt_cmd = now
            if self.edt_want:
                self.send_command(sd.DSHOT_CMD_EDT_ENABLE)
                self.status = 'EDT enable sent, waiting for EDT frames (arms after >1.5s at zero)'
            else:
                self.send_command(sd.DSHOT_CMD_EDT_DISABLE)
                self.status = 'EDT disable sent'


class CanPanel(object):
    '''DroneCAN node thread: RawCommand/ArmingStatus stream, telemetry
    handlers and parameter set requests'''

    def __init__(self, uri):
        self.uri = uri
        self.enabled = False
        self.throttle = 0.0     # 0..1
        self.rate = 50.0
        self.esc_index = 0
        self.status = {}
        self.node_id = None
        self.uptime = 0
        self.esc_rate = RateCounter()
        self.sent = RateCounter()
        self.param_queue = queue.Queue()
        self.param_result = queue.Queue()
        self.error = None
        self.running = True
        # set once make_node has spawned the IO child (or failed), so the
        # caller can sequence signal handler setup around the spawn
        self.started = threading.Event()
        self.thread = threading.Thread(target=self._can_thread, daemon=True)
        self.thread.start()

    def _can_thread(self):
        try:
            node = dronecan.make_node(self.uri, node_id=126, bitrate=1000000)
        except Exception as ex:
            self.error = str(ex)
            self.started.set()
            return
        self.started.set()

        def on_esc_status(e):
            m = e.message
            if m.esc_index != self.esc_index:
                return
            # the ESC is unambiguously the sender of esc.Status; other
            # nodes on the bus also send NodeStatus
            self.node_id = e.transfer.source_node_id
            self.esc_rate.tick()
            self.status = {
                'rpm': m.rpm,
                'voltage': m.voltage,
                'current': m.current,
                'temp': m.temperature - 273.15,
                'errors': m.error_count,
            }

        def on_node_status(e):
            if e.transfer.source_node_id == self.node_id:
                self.uptime = e.message.uptime_sec

        node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)
        node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)

        next_send = time.time()
        while self.running:
            try:
                node.spin(0.002)
            except Exception:
                pass
            self._handle_param(node)
            if not self.enabled:
                next_send = time.time()
                continue
            now = time.time()
            if now >= next_send:
                next_send += 1.0 / max(1.0, self.rate)
                cmds = [0] * (self.esc_index + 1)
                cmds[self.esc_index] = int(8191 * self.throttle)
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=cmds))
                self.sent.tick()
        # orderly shutdown of the mcast IO child process
        try:
            node.close()
        except Exception:
            pass

    def _handle_param(self, node):
        try:
            name, value = self.param_queue.get_nowait()
        except queue.Empty:
            return
        if self.node_id is None:
            self.param_result.put('no node seen yet')
            return
        target = self.node_id
        result = {}

        def cb(e):
            result['rsp'] = e.response if e is not None else None
            result['done'] = True

        def wait(req, timeout=2.0):
            result.clear()
            node.request(req, target, cb)
            deadline = time.time() + timeout
            while 'done' not in result and time.time() < deadline:
                node.spin(0.05)
            return result.get('rsp')

        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
        rsp = wait(req)
        if rsp is None or len(rsp.name) == 0:
            self.param_result.put('%s: set failed' % name)
            return
        req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
        req.opcode = req.OPCODE_SAVE
        wait(req)
        req = dronecan.uavcan.protocol.RestartNode.Request()
        req.magic_number = req.MAGIC_NUMBER
        wait(req, timeout=1.0)
        self.param_result.put('%s=%d saved, node %d restarted' % (name, int(value), target))

    def set_param(self, name, value):
        self.param_queue.put((name, value))
