"""PC-side NXT probe: talk to the brick with direct commands over USB or a Bluetooth COM port.

Used for the one-time setup (the NXT's LCD is dead, so `screen` prints it as text) and for
checking motor behaviour before trusting it in the web app.

  python nxt_probe.py --usb info                 # needs WinUSB bound to the NXT (Zadig) + pip install pyusb libusb-package
  python nxt_probe.py --port COM12 info          # Bluetooth: pair in Windows first, use the *outgoing* COM port
  python nxt_probe.py --usb screen               # LCD as text
  python nxt_probe.py --usb press right enter    # front-panel buttons: left / enter / right / exit
  python nxt_probe.py --usb bt                   # Bluetooth on/visible state
  python nxt_probe.py --usb beep
  python nxt_probe.py --usb move B 360           # tacho-limited move, prints where it ended
  python nxt_probe.py --usb limittest B          # does a new tacho limit count from the current position?
"""
import argparse
import struct
import sys
import time

DIRECT, SYSTEM, NO_REPLY = 0x00, 0x01, 0x80
MOD_DISPLAY, MOD_UI = 0x000A0001, 0x000C0001
DISP_FLAGS, DISP_NORMAL, DISP_POPUP = 117, 119, 919
UI_BUTTON, UI_BT_STATE = 28, 31
BUTTONS = {'left': 1, 'enter': 2, 'right': 3, 'exit': 4}
PORTS = {'A': 0, 'B': 1, 'C': 2}
MODE_ON, MODE_BRAKE, MODE_REG = 0x01, 0x02, 0x04
REG_SPEED, RUN_RUNNING, RUN_IDLE = 1, 0x20, 0x00


class UsbLink:
    def __init__(self):
        import usb.core
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except ImportError:
            backend = None
        self.dev = usb.core.find(idVendor=0x0694, idProduct=0x0002, backend=backend)
        if self.dev is None:
            sys.exit('NXT not found on USB (is it on, and is WinUSB bound to it with Zadig?)')
        self.dev.set_configuration()
        intf = self.dev.get_active_configuration()[(0, 0)]
        self.ep_out = next(e for e in intf if not e.bEndpointAddress & 0x80)
        self.ep_in = next(e for e in intf if e.bEndpointAddress & 0x80)

    def send(self, tel):
        self.ep_out.write(tel, 1000)

    def recv(self):
        return bytes(self.ep_in.read(64, 2000))


class SerialLink:
    """Bluetooth SPP: telegrams carry a 2-byte little-endian length prefix."""

    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=3)

    def send(self, tel):
        self.s.write(struct.pack('<H', len(tel)) + tel)

    def recv(self):
        hdr = self.s.read(2)
        if len(hdr) < 2:
            raise TimeoutError('NXT did not answer')
        (n,) = struct.unpack('<H', hdr)
        return self.s.read(n)


class Nxt:
    def __init__(self, link):
        self.link = link

    def req(self, typ, cmd, payload=b'', reply=True):
        self.link.send(bytes([typ if reply else typ | NO_REPLY, cmd]) + payload)
        if not reply:
            return None
        r = self.link.recv()
        if len(r) < 3 or r[0] != 0x02 or r[1] != cmd:
            raise IOError(f'unexpected reply {r.hex()}')
        if r[2]:
            raise IOError(f'NXT error 0x{r[2]:02x} for command 0x{cmd:02x}')
        return r

    def info(self):
        r = self.req(SYSTEM, 0x9B)
        name = r[3:18].split(b'\0')[0].decode('ascii', 'replace')
        addr = ':'.join(f'{b:02X}' for b in r[18:24])
        f = self.req(SYSTEM, 0x88)
        mv = struct.unpack_from('<H', self.req(DIRECT, 0x0B), 3)[0]
        return {'name': name, 'bt_addr': addr, 'firmware': f'{f[6]}.{f[5]:02d}', 'protocol': f'{f[4]}.{f[3]}', 'battery_mV': mv}

    def read_iomap(self, mod, off, n):
        r = self.req(SYSTEM, 0x94, struct.pack('<IHH', mod, off, n))
        got = struct.unpack_from('<H', r, 7)[0]
        return r[9:9 + got]

    def write_iomap(self, mod, off, data):
        self.req(SYSTEM, 0x95, struct.pack('<IHH', mod, off, len(data)) + bytes(data))

    def screen(self):
        flags = self.read_iomap(MOD_DISPLAY, DISP_FLAGS, 1)[0]
        base = DISP_POPUP if flags & 0x08 else DISP_NORMAL
        buf = b''.join(self.read_iomap(MOD_DISPLAY, base + o, 50) for o in range(0, 800, 50))
        return buf, bool(flags & 0x08)

    def press(self, btn):
        self.write_iomap(MOD_UI, UI_BUTTON, [BUTTONS[btn]])

    def bt_state(self):
        return self.read_iomap(MOD_UI, UI_BT_STATE, 1)[0]

    def set_output(self, port, power, limit=0, reply=True):
        mode = MODE_ON | MODE_BRAKE | MODE_REG
        self.req(DIRECT, 0x04, struct.pack('<BbBBbBI', port, power, mode, REG_SPEED, 0, RUN_RUNNING, limit), reply)

    def brake(self, port=0xFF):
        self.req(DIRECT, 0x04, struct.pack('<BbBBbBI', port, 0, MODE_ON | MODE_BRAKE, 0, 0, RUN_RUNNING, 0))

    def get_output(self, port):
        r = self.req(DIRECT, 0x06, bytes([port]))
        power, mode, reg, turn, run, limit, tacho, block, rot = struct.unpack_from('<bBBbBIiii', r, 4)
        return {'power': power, 'run': run, 'limit': limit, 'tacho': tacho, 'block': block, 'rotation': rot}


def render(buf):
    """Two pixel rows per text line, using half-block characters."""
    px = lambda x, y: (buf[(y >> 3) * 100 + x] >> (y & 7)) & 1
    lines = ['+' + '-' * 100 + '+']
    for y in range(0, 64, 2):
        row = ''
        for x in range(100):
            top, bot = px(x, y), px(x, y + 1)
            row += '█' if top and bot else '▀' if top else '▄' if bot else ' '
        lines.append('|' + row + '|')
    lines.append(lines[0])
    return '\n'.join(lines)


def bt_text(s):
    if s & 0x80:
        return 'PIN/passkey request pending (press enter)'
    if s & 0x04:
        return 'OFF'
    return 'ON, ' + ('visible' if s & 0x01 else 'hidden') + (', connected' if s & 0x02 else '')


def wait_idle(nxt, port, timeout=10):
    t0, last, last_change = time.time(), None, time.time()
    while time.time() - t0 < timeout:
        s = nxt.get_output(port)
        if s['rotation'] != last:
            last, last_change = s['rotation'], time.time()
        if s['run'] == RUN_IDLE or time.time() - last_change > 0.7:
            return s
        time.sleep(0.05)
    return nxt.get_output(port)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--usb', action='store_true')
    g.add_argument('--port', help='Bluetooth outgoing COM port, e.g. COM12')
    ap.add_argument('cmd', choices=['info', 'screen', 'press', 'bt', 'beep', 'move', 'limittest'])
    ap.add_argument('args', nargs='*')
    a = ap.parse_args()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    nxt = Nxt(UsbLink() if a.usb else SerialLink(a.port))

    if a.cmd == 'info':
        for k, v in nxt.info().items():
            print(f'{k:11} {v}')
        print(f'{"bluetooth":11} {bt_text(nxt.bt_state())}')
    elif a.cmd == 'screen':
        buf, popup = nxt.screen()
        print(render(buf) + ('\n(popup layer)' if popup else ''))
    elif a.cmd == 'press':
        for b in a.args:
            nxt.press(b)
            time.sleep(0.6)
        buf, popup = nxt.screen()
        print(render(buf) + ('\n(popup layer)' if popup else ''))
    elif a.cmd == 'bt':
        print(bt_text(nxt.bt_state()))
    elif a.cmd == 'beep':
        nxt.req(DIRECT, 0x03, struct.pack('<HH', 880, 300), reply=False)
    elif a.cmd == 'move':
        port, deg = PORTS[a.args[0].upper()], int(a.args[1])
        power = int(a.args[2]) if len(a.args) > 2 else 60
        start = nxt.get_output(port)['rotation']
        nxt.set_output(port, power if deg > 0 else -power, abs(deg))
        end = wait_idle(nxt, port)
        print(f'asked {deg:+d}, moved {end["rotation"] - start:+d} (rotation {start} -> {end["rotation"]}), state {end}')
    elif a.cmd == 'limittest':
        port = PORTS[(a.args or ['B'])[0].upper()]
        for i in range(3):
            start = nxt.get_output(port)['rotation']
            nxt.set_output(port, 50, 180)
            end = wait_idle(nxt, port)
            print(f'move {i + 1}: limit 180 -> moved {end["rotation"] - start:+d}, tacho {end["tacho"]}, block {end["block"]}')
        print('relative limits work' if abs(end['rotation'] - start) > 90 else 'limit is absolute: the app must use soft limits')
        nxt.brake()


if __name__ == '__main__':
    main()
