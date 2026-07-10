'''
protocol library for AM32 SITL PWM/DShot input over UDP

packet format (little endian):
  u16 magic 0x4453
  u8  type: 0=PWM, 1=DSHOT150, 2=DSHOT300, 3=DSHOT600
  u8  len: payload bytes after the 6 byte header (4)
  u16 flags: bit0 = line idle level (1 = idle high, inverted/bidir DShot)
  u16 data: PWM pulse width in microseconds, or the full 16 bit DShot
            frame (11 bit value, telemetry bit, 4 bit CRC)

replies (bidirectional DShot) use the same format with data being the 16
bit GCR-decoded frame: 12 bit eRPM/EDT value plus 4 bit CRC
'''

import socket
import struct
import time
import threading

MAGIC = 0x4453
TYPE_PWM = 0
TYPE_DSHOT150 = 1
TYPE_DSHOT300 = 2
TYPE_DSHOT600 = 3
FLAG_IDLE_HIGH = 0x0001

TYPE_NAMES = {
    'pwm': TYPE_PWM,
    'dshot150': TYPE_DSHOT150,
    'dshot300': TYPE_DSHOT300,
    'dshot600': TYPE_DSHOT600,
}

# DShot commands (value field 1..47 at zero throttle)
DSHOT_CMD_EDT_ENABLE = 13
DSHOT_CMD_EDT_DISABLE = 14


def dshot_crc(value12, bidir=False):
    '''4 bit CRC over the 12 bit value+telemetry field'''
    csum = value12 ^ (value12 >> 4) ^ (value12 >> 8)
    if bidir:
        csum = ~csum
    return csum & 0xF


def dshot_frame(value11, telem=False, bidir=False, corrupt=False):
    '''compose the full 16 bit DShot frame from an 11 bit value'''
    v = ((value11 & 0x7FF) << 1) | (1 if telem else 0)
    crc = dshot_crc(v, bidir)
    if corrupt:
        crc ^= 0x5
    return (v << 4) | crc


def check_reply_crc(frame16):
    '''BDShot reply CRC: xor of the four nibbles must be 0xF'''
    n = frame16
    return ((n ^ (n >> 4) ^ (n >> 8) ^ (n >> 12)) & 0xF) == 0xF


def decode_reply(frame16, edt_expected=False):
    '''decode a 16 bit BDShot reply frame.
    returns (kind, value) where kind is one of:
      'erpm'    - value is the eRPM period in microseconds (65408 = stopped)
      'temp'    - degrees C
      'volt'    - volts
      'current' - amps
      'edt'     - other extended frame, value is the raw 12 bits
      'badcrc'  - CRC failure, value is the raw frame
    '''
    if not check_reply_crc(frame16):
        return ('badcrc', frame16)
    val = frame16 >> 4
    # extended telemetry frames have bit 8 clear (eRPM mantissa is
    # normalised so real eRPM frames have it set)
    if edt_expected and (val & 0x100) == 0 and val != 0:
        etype = val >> 8
        data = val & 0xFF
        if etype == 0x2:
            return ('temp', data)
        if etype == 0x4:
            return ('volt', data * 0.25)
        if etype == 0x6:
            # AM32 encodes centiamps/50, ie 0.5A units
            return ('current', data * 0.5)
        return ('edt', val)
    period_us = (val & 0x1FF) << (val >> 9)
    return ('erpm', period_us)


def erpm_period_to_rpm(period_us, motor_poles=14):
    '''convert eRPM period in us to mechanical RPM'''
    if period_us == 0 or period_us >= 65408:
        return 0.0
    erpm = 60.0e6 / period_us
    return erpm / (motor_poles / 2)


def pack(ptype, flags, data):
    return struct.pack('<HBBHH', MAGIC, ptype, 4, flags, data)


def unpack(buf):
    if len(buf) < 8:
        return None
    magic, ptype, length, flags, data = struct.unpack('<HBBHH', buf[:8])
    if magic != MAGIC or length != 4:
        return None
    return (ptype, flags, data)


class InputPort(object):
    '''UDP connection to the SITL input port, with a reader thread
    collecting BDShot replies'''

    def __init__(self, host='127.0.0.1', port=57733):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.replies = []          # (time, type, flags, data)
        self.reply_count = 0
        self.sent_count = 0
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            try:
                buf = self.sock.recv(64)
            except socket.timeout:
                continue
            except OSError:
                return
            p = unpack(buf)
            if p is None:
                continue
            with self.lock:
                self.replies.append((time.time(),) + p)
                self.reply_count += 1
                if len(self.replies) > 1000:
                    self.replies = self.replies[-500:]

    def close(self):
        self.running = False
        self.sock.close()

    def send_pwm(self, width_us, idle_high=False):
        flags = FLAG_IDLE_HIGH if idle_high else 0
        self.sock.sendto(pack(TYPE_PWM, flags, width_us), self.addr)
        self.sent_count += 1

    def send_dshot(self, value11, ptype=TYPE_DSHOT300, telem=False,
                   bidir=False, corrupt=False):
        flags = FLAG_IDLE_HIGH if bidir else 0
        frame = dshot_frame(value11, telem=telem, bidir=bidir, corrupt=corrupt)
        self.sock.sendto(pack(ptype, flags, frame), self.addr)
        self.sent_count += 1

    def get_replies(self):
        with self.lock:
            r = self.replies
            self.replies = []
        return r


def set_dronecan_param(name, value, uri='mcast:0', timeout=10.0,
                       save=True, restart=True, node_id=None):
    '''set a DroneCAN parameter on the (single) AM32 node, optionally
    save parameters and restart the node. Needs pydronecan'''
    import dronecan
    node = dronecan.make_node(uri, node_id=126, bitrate=1000000)
    found = {}

    def on_status(e):
        found[e.transfer.source_node_id] = True

    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_status)
    # NodeStatus is 1Hz: collect for a couple of seconds so all bus
    # participants are seen, not just the first
    t0 = time.time()
    while time.time() - t0 < timeout:
        node.spin(0.1)
        if found and time.time() - t0 > 2.5:
            break
    if not found:
        raise RuntimeError('no DroneCAN node found')

    target = None

    def request_wait(req, timeout=2.0):
        '''send a service request, spin until the response (or timeout
        callback, which pydronecan delivers as None)'''
        result = {}

        def cb(e):
            result['response'] = e.response if e is not None else None
            result['done'] = True

        node.request(req, target, cb)
        deadline = time.time() + timeout
        while 'done' not in result and time.time() < deadline:
            node.spin(0.05)
        return result.get('response')

    # other tools may be on the bus: find the AM32 node by probing for
    # the parameter itself (a GetSet read of an unknown name returns an
    # empty response name)
    candidates = [node_id] if node_id is not None else sorted(found.keys())
    for cand in candidates:
        target = cand
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        rsp = request_wait(req, timeout=1.0)
        if rsp is not None and len(rsp.name) > 0:
            break
    else:
        raise RuntimeError('no node with parameter %s found' % name)

    req = dronecan.uavcan.protocol.param.GetSet.Request()
    req.name = name
    req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
    for attempt in range(3):
        rsp = request_wait(req)
        if rsp is not None and rsp.value.integer_value == int(value):
            break
    else:
        raise RuntimeError('param set of %s failed' % name)

    if save:
        req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
        req.opcode = req.OPCODE_SAVE
        for attempt in range(3):
            rsp = request_wait(req)
            if rsp is not None and rsp.ok:
                break
        else:
            raise RuntimeError('param save failed')

    if restart:
        req = dronecan.uavcan.protocol.RestartNode.Request()
        req.magic_number = req.MAGIC_NUMBER
        request_wait(req, timeout=1.0)

    node.close()
    return target
