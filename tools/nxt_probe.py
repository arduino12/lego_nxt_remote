"""PC-side NXT probe: talk to the brick with direct commands over USB or a Bluetooth COM port.

Used for the one-time setup (the NXT's LCD is dead, so `screen` prints it as text) and for
checking motor behaviour before trusting it in the web app.

  python nxt_probe.py --usb info                 # needs WinUSB bound to the NXT (Zadig) + pip install pyusb libusb-package
  python nxt_probe.py --port COM12 info          # Bluetooth: pair in Windows first, use the *outgoing* COM port
  python nxt_probe.py --usb screen               # LCD as text
  python nxt_probe.py --usb press right enter    # front-panel buttons: left / enter / right / exit
  python nxt_probe.py --usb bt                   # Bluetooth on/visible state
  python nxt_probe.py --usb beep
  python nxt_probe.py --usb get ABC              # motor state (rotation counter, run state)
  python nxt_probe.py --usb move BC 360 [power]  # tacho-limited move, prints where it ended
  python nxt_probe.py --usb limittest BC 120 3   # does a new tacho limit count from the current position?
  python nxt_probe.py --usb serve cmds.txt out.txt   # keep the brick awake, run commands appended to cmds.txt
"""
import argparse
import struct
import sys
import time

DIRECT, SYSTEM, NO_REPLY = 0x00, 0x01, 0x80
MOD_OUTPUT, MOD_DISPLAY, MOD_UI = 0x00020001, 0x000A0001, 0x000C0001
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


def wait_idle(nxt, ports, timeout=10):
    """Poll until every port reports RUN_IDLE or nothing has moved for 0.7 s."""
    t0, last, last_change = time.time(), None, time.time()
    while time.time() - t0 < timeout:
        s = [nxt.get_output(p) for p in ports]
        rot = [x['rotation'] for x in s]
        if rot != last:
            last, last_change = rot, time.time()
        if all(x['run'] == RUN_IDLE for x in s) or time.time() - last_change > 0.7:
            return s
        time.sleep(0.05)
    return [nxt.get_output(p) for p in ports]


def run(nxt, cmd, args, out):
    """One probe command; `out` prints a line. Shared by the CLI and `serve`."""
    def show_screen():
        buf, popup = nxt.screen()
        out(render(buf) + ('\n(popup layer)' if popup else ''))

    if cmd == 'info':
        for k, v in nxt.info().items():
            out(f'{k:11} {v}')
        out(f'{"bluetooth":11} {bt_text(nxt.bt_state())}')
    elif cmd == 'screen':
        show_screen()
    elif cmd == 'press':
        for b in args:
            nxt.press(b)
            time.sleep(0.7)
        show_screen()
    elif cmd == 'bt':
        out(bt_text(nxt.bt_state()))
    elif cmd == 'beep':
        nxt.req(DIRECT, 0x03, struct.pack('<HH', 880, 300), reply=False)
    elif cmd == 'get':
        for p in args[0].upper():
            out(f'{p}: {nxt.get_output(PORTS[p])}')
    elif cmd == 'reset':   # zero RotationCount (absolute position)
        for p in args[0].upper():
            nxt.req(DIRECT, 0x0A, bytes([PORTS[p], 0]))
        out('rotation counters zeroed')
    elif cmd == 'brake':
        nxt.brake()
    elif cmd == 'backup':   # backup DIR : copy every file off the brick
        import os
        os.makedirs(args[0], exist_ok=True)
        names = []
        run(nxt, 'files', [], lambda line: names.append(line[:20].rstrip()) if ' bytes' in line else None)
        for name in names:
            r = nxt.req(SYSTEM, 0x80, name.encode().ljust(20, b'\0'))   # OPEN READ
            handle, size = r[3], struct.unpack_from('<I', r, 4)[0]
            data = b''
            while len(data) < size:
                r = nxt.req(SYSTEM, 0x82, struct.pack('<BH', handle, min(58, size - len(data))))   # READ
                data += r[6:6 + struct.unpack_from('<H', r, 4)[0]]
            nxt.req(SYSTEM, 0x84, bytes([handle]))
            with open(os.path.join(args[0], name), 'wb') as f:
                f.write(data)
            out(f'saved {name} ({len(data)} bytes)')
    elif cmd == 'restore':   # restore DIR [names...] : upload files (linear, as NXT-G does for programs)
        import os
        for name in args[1:] or sorted(os.listdir(args[0])):
            data = open(os.path.join(args[0], name), 'rb').read()
            try:
                r = nxt.req(SYSTEM, 0x89, name.encode().ljust(20, b'\0') + struct.pack('<I', len(data)))   # OPEN WRITE LINEAR
            except IOError as e:
                out(f'skip {name}: {e}')
                continue
            handle = r[3]
            for i in range(0, len(data), 61):
                nxt.req(SYSTEM, 0x83, bytes([handle]) + data[i:i + 61])   # WRITE
            nxt.req(SYSTEM, 0x84, bytes([handle]))
            out(f'restored {name} ({len(data)} bytes)')
    elif cmd == 'rm':   # rm NAME... (names may contain spaces: separate them with '|')
        for name in ' '.join(args).split('|'):
            name = name.strip()
            try:
                nxt.req(SYSTEM, 0x85, name.encode().ljust(20, b'\0'))   # DELETE
                out(f'deleted {name}')
            except IOError as e:
                out(f'could not delete {name}: {e}')
    elif cmd == 'files':   # files [pattern] : list the brick's flash file system
        pat = (args[0] if args else '*.*').encode()[:19].ljust(20, b'\0')
        try:
            r = nxt.req(SYSTEM, 0x86, pat)   # FIND FIRST
            while True:
                handle = r[3]
                name = r[4:24].split(b'\0')[0].decode('ascii', 'replace')
                out(f'{name:20} {struct.unpack_from("<I", r, 24)[0]:7d} bytes')
                try:
                    r = nxt.req(SYSTEM, 0x87, bytes([handle]))   # FIND NEXT
                except IOError:
                    break
            nxt.req(SYSTEM, 0x84, bytes([handle]))   # CLOSE
        except IOError as e:
            out(f'no (more) files: {e}')
    elif cmd == 'pos':
        # pos B 360 [maxspeed] [maxaccel] [seconds] : NXT Improved Firmware absolute position
        # regulation (RegMode 4). Zero the internal count first (UPDATE_RESET_COUNT also zeroes
        # the regulator's target), optionally set the speed profile, then send the target.
        port, deg = PORTS[args[0].upper()], int(args[1])
        maxspeed = int(args[2]) if len(args) > 2 else 0
        maxacc = int(args[3]) if len(args) > 3 else 0
        secs = float(args[4]) if len(args) > 4 else 1.5
        base = port * 32
        if len(args) > 7:   # P I D (firmware defaults 96 32 32)
            nxt.write_iomap(MOD_OUTPUT, base + 22, [int(args[5]), int(args[6]), int(args[7])])
        nxt.write_iomap(MOD_OUTPUT, base + 30, [maxspeed & 0xFF, maxacc & 0xFF])   # MaxSpeed, MaxAcceleration
        nxt.write_iomap(MOD_OUTPUT, base + 18, [0x08 | 0x10])                       # Flags: RESET_COUNT | PID_VALUES
        time.sleep(0.02)
        s0 = nxt.get_output(port)
        out(f'before: {s0}')
        payload = struct.pack('<BbBBbBi', port, 0, MODE_ON | MODE_BRAKE | MODE_REG, 4, 0, RUN_RUNNING, deg)
        t0 = time.time()
        for _ in range(2):   # the first one may land before the regulation mode switch
            nxt.req(DIRECT, 0x04, payload)
        while time.time() - t0 < secs:
            s = nxt.get_output(port)
            out(f'{(time.time() - t0) * 1000:6.0f} ms  run 0x{s["run"]:02x}  tacho {s["tacho"]:+5d}  rot {s["rotation"] - s0["rotation"]:+5d}')
            time.sleep(0.02)
    elif cmd == 'ior':   # ior MODULE_HEX OFFSET N
        out(nxt.read_iomap(int(args[0], 16), int(args[1]), int(args[2])).hex(' '))
    elif cmd == 'trace':
        # trace B 360 [power] [seconds] : start a tacho-limited move and sample the motor state over time
        port, deg = PORTS[args[0].upper()], int(args[1])
        power = int(args[2]) if len(args) > 2 else 50
        secs = float(args[3]) if len(args) > 3 else 1.5
        s0 = nxt.get_output(port)
        out(f'before: {s0}')
        t0 = time.time()
        nxt.set_output(port, power if deg > 0 else -power, abs(deg), reply=False)
        while time.time() - t0 < secs:
            s = nxt.get_output(port)
            out(f'{(time.time() - t0) * 1000:6.0f} ms  run 0x{s["run"]:02x} pw {s["power"]:4d}  tacho {s["tacho"] - s0["tacho"]:+5d}'
                f'  block {s["block"] - s0["block"]:+5d}  rot {s["rotation"] - s0["rotation"]:+5d}')
    elif cmd in ('move', 'limittest'):
        # move BC 360 [power]         : one tacho-limited move on each listed port
        # limittest BC 120 3 [power]  : repeated limited moves - does each new limit count from where the motor is?
        ports = [PORTS[p] for p in args[0].upper()]
        deg = int(args[1])
        reps = int(args[2]) if cmd == 'limittest' and len(args) > 2 else 1
        power = int(args[3 if cmd == 'limittest' else 2]) if len(args) > (3 if cmd == 'limittest' else 2) else 50
        first = [nxt.get_output(p)['rotation'] for p in ports]
        for i in range(reps):
            start = [nxt.get_output(p)['rotation'] for p in ports]
            t0 = time.time()
            for p in ports:
                nxt.set_output(p, power if deg > 0 else -power, abs(deg), reply=False)
            end = wait_idle(nxt, ports)
            out(f'move {i + 1}: limit {deg:+d} -> moved ' + ', '.join(f'{args[0].upper()[j]} {end[j]["rotation"] - start[j]:+d}' for j in range(len(ports)))
                + f' in {time.time() - t0:.2f}s (tacho {[e["tacho"] for e in end]}, block {[e["block"] for e in end]})')
        time.sleep(0.3)
        final = [nxt.get_output(p)['rotation'] for p in ports]
        out('after settling: ' + ', '.join(f'{args[0].upper()[j]} rotation {final[j]} (total {final[j] - first[j]:+d})' for j in range(len(ports))))
    else:
        out(f'unknown command {cmd}')


def serve(make_link, cmd_path, out_path):
    """Wait for the brick, keep it awake, and run commands appended to cmd_path (one per
    line, same syntax as the CLI). Output is appended to out_path. For a brick that
    switches itself off before you can type the next command."""
    import os

    def log(s):
        with open(out_path, 'a', encoding='utf-8') as f:
            f.write(s + '\n')

    open(cmd_path, 'a').close()
    done = os.path.getsize(cmd_path)
    nxt = None
    last_keep = 0
    last_mv = 0
    while True:
        try:
            if nxt is None:
                nxt = Nxt(make_link())
                nxt.req(DIRECT, 0x0D)   # KEEPALIVE resets the sleep timer
                log(f'=== connected {time.strftime("%H:%M:%S")}: {nxt.info()}, bluetooth {bt_text(nxt.bt_state())}')
            if time.time() - last_keep > 5:
                nxt.req(DIRECT, 0x0D)
                mv = struct.unpack_from('<H', nxt.req(DIRECT, 0x0B), 3)[0]
                if abs(mv - last_mv) >= 100:
                    log(f'{time.strftime("%H:%M:%S")} battery {mv} mV')
                    last_mv = mv
                last_keep = time.time()
            size = os.path.getsize(cmd_path)
            if size > done:
                with open(cmd_path, encoding='utf-8') as f:
                    f.seek(done)
                    lines = f.read().splitlines()
                done = size
                for line in filter(None, (l.strip() for l in lines)):
                    cmd, *args = line.split()
                    log(f'> {line}')
                    try:
                        run(nxt, cmd, args, log)
                    except (IOError, ValueError, KeyError, IndexError) as e:
                        log(f'error: {e!r}')
                    log('<')
            time.sleep(0.2)
        except SystemExit:
            time.sleep(0.3)   # not on USB yet
        except Exception as e:
            if nxt is not None:
                log(f'=== lost {time.strftime("%H:%M:%S")}: {e!r}')
            nxt = None
            time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--usb', action='store_true')
    g.add_argument('--port', help='Bluetooth outgoing COM port, e.g. COM12')
    ap.add_argument('cmd')
    ap.add_argument('args', nargs='*')
    a = ap.parse_args()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    make_link = UsbLink if a.usb else lambda: SerialLink(a.port)
    if a.cmd == 'serve':
        return serve(make_link, *a.args[:2])
    run(Nxt(make_link()), a.cmd, a.args, print)


if __name__ == '__main__':
    main()
