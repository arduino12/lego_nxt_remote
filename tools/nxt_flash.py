"""Flash NXT firmware over USB through the AT91SAM7 SAM-BA boot monitor (a Python port of
libnxt's fwflash: https://sources.debian.org/src/libnxt/0.5.2-1/firmware.c).

  pip install pyusb libusb-package
  python nxt_flash.py nxt_firmware.bin            # brick running LEGO firmware, or already in SAM-BA
  python nxt_flash.py nxt_firmware.bin --verify-only

Firmware image: Debian package nxt-firmware (NXT Improved Firmware, LEGO 1.29 based),
/usr/share/nxt-firmware/nxt_firmware.bin, 262144 bytes.

Pages are written with SAM-BA word writes into the flash latch followed by an EFC
"write page" command (libnxt does the same copy from a small RAM routine). If anything
fails, the brick stays in SAM-BA mode (ticking) and can simply be flashed again.
"""
import argparse
import struct
import sys
import time

import usb.core

try:
    import libusb_package
    BACKEND = libusb_package.get_libusb1_backend()
except ImportError:
    BACKEND = None

LEGO = (0x0694, 0x0002)
SAMBA = (0x03EB, 0x6124)
FLASH_BASE, PAGE, PAGES = 0x100000, 256, 1024
MC_FMR, MC_FCR, MC_FSR = 0xFFFFFF60, 0xFFFFFF64, 0xFFFFFF68
PMC_MCKR = 0xFFFFFC30


def find(ids):
    return usb.core.find(idVendor=ids[0], idProduct=ids[1], backend=BACKEND)


def check_image(img):
    if len(img) != PAGES * PAGE:
        sys.exit(f'image is {len(img)} bytes, expected {PAGES * PAGE}')
    for v in struct.unpack('<6I', img[:24]):   # ARM exception vectors, as libnxt checks
        if not (v == 0xE1A00000 or v == 0xEAFFFFFE or v & 0xFFFF0000 == 0xEA000000 or v & 0xFFFFF000 == 0xE59FF000):
            sys.exit('image does not start with ARM vectors - not an NXT firmware')


def reboot_to_samba():
    """LEGO firmware system command BOOT (USB only): the brick resets into SAM-BA."""
    dev = find(LEGO)
    if dev is None:
        return False
    dev.set_configuration()
    intf = dev.get_active_configuration()[(0, 0)]
    out = next(e for e in intf if not e.bEndpointAddress & 0x80)
    inp = next(e for e in intf if e.bEndpointAddress & 0x80)
    out.write(bytes([0x01, 0x97]) + b"Let's dance: SAMBA\0", 1000)
    try:
        r = bytes(inp.read(64, 2000))
        print(f'boot command reply: {r[2:]!r}')
    except usb.core.USBError:
        pass   # the brick may reset before answering
    usb.util.dispose_resources(dev)
    return True


class UsbPipe:
    """SAM-BA CDC data interface on raw bulk endpoints (WinUSB bound by the LEGO driver)."""

    def __init__(self, dev):
        self.dev = dev
        dev.set_configuration()
        usb.util.claim_interface(dev, 1)

    def write(self, b):
        self.dev.write(0x01, b, 1000)

    def read(self, ms):
        try:
            return bytes(self.dev.read(0x82, 64, ms))
        except usb.core.USBTimeoutError:
            return b''


class SerialPipe:
    """Same protocol through the COM port, when Windows bound a CDC serial driver instead."""

    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0)

    def write(self, b):
        self.s.write(b)

    def read(self, ms):
        self.s.timeout = ms / 1000
        b = self.s.read(1)   # wait for the first byte only, then take whatever has arrived
        return b + self.s.read(self.s.in_waiting) if b else b


def open_pipe(dev):
    try:
        return UsbPipe(dev)
    except (usb.core.USBError, NotImplementedError) as e:   # e.g. Arduino's "Bossa Program Port" (usbser) owns it
        from serial.tools import list_ports
        port = next((p.device for p in list_ports.comports() if (p.vid, p.pid) == SAMBA), None)
        if port is None:
            raise IOError(f'cannot open SAM-BA over USB ({e}) and no COM port for it')
        print(f'raw USB unavailable ({e}); using {port}')
        return SerialPipe(port)


class Samba:
    def __init__(self, pipe):
        self.p = pipe
        self.drain()

    def drain(self, ms=50):
        got = b''
        while True:
            b = self.p.read(ms)
            if not b:
                return got
            got += b

    def cmd(self, s):
        self.p.write(s.encode())

    def recv(self, n):
        buf = b''
        t0 = time.time()
        while len(buf) < n:
            buf += self.p.read(200)
            if time.time() - t0 > 3:
                raise IOError(f'SAM-BA read timed out ({len(buf)}/{n} bytes)')
        return buf

    def handshake(self):
        # N# switches the monitor to binary mode and answers "\n\r" - only the first time.
        self.cmd('N#')
        r = self.drain(200)
        if r not in (b'\n\r', b''):
            raise IOError(f'SAM-BA handshake failed: {r!r}')

    def version(self):
        self.cmd('V#')
        return self.drain(150).decode('ascii', 'replace').strip()   # longer than libnxt's 4 bytes

    def write_word(self, addr, val):
        self.cmd(f'W{addr:08X},{val:08X}#')

    def read_word(self, addr):
        self.cmd(f'w{addr:08X},4#')
        return struct.unpack('<I', self.recv(4))[0]

    def read_block(self, addr, n):
        # Windows' usbser holds back a transfer that is an exact multiple of 64 bytes until the
        # next packet arrives, so read n - 4 bytes in bulk and the last word separately.
        self.cmd(f'R{addr:08X},{n - 4:08X}#')
        data = self.recv(n - 4)
        return data + struct.pack('<I', self.read_word(addr + n - 4))

    def jump(self, addr):
        self.cmd(f'G{addr:08X}#')

    def flash_wait(self):
        t0 = time.time()
        while not self.read_word(MC_FSR) & 1:   # FRDY
            if time.time() - t0 > 2:
                raise IOError('flash controller stuck busy')

    def prepare(self):
        self.write_word(PMC_MCKR, 0x7)   # clock to PLL/2 (libnxt)
        for region in range(16):         # clear every lock bit
            self.flash_wait()
            self.write_word(MC_FMR, 0x00050100)
            self.write_word(MC_FCR, 0x5A000000 | ((64 * region) << 8) | 0x4)
            self.write_word(MC_FMR, 0x00340100)
        self.flash_wait()

    def write_page(self, n, data):
        self.flash_wait()
        base = FLASH_BASE + n * PAGE
        for i, (w,) in enumerate(struct.iter_unpack('<I', data)):
            self.write_word(base + 4 * i, w)              # fills the EFC latch buffer
        self.write_word(MC_FCR, 0x5A000001 | (n << 8))    # KEY | WP (erase + program page n)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image')
    ap.add_argument('--verify-only', action='store_true')
    ap.add_argument('--save-old', metavar='FILE', help='read the current flash into FILE before writing')
    ap.add_argument('--wait', type=int, default=0, metavar='S', help='wait up to S seconds for the brick to appear')
    a = ap.parse_args()
    img = open(a.image, 'rb').read()
    check_image(img)

    dev = find(SAMBA)
    if dev is None and a.wait:
        print(f'waiting up to {a.wait} s for the brick (hold the reset button 5 s for SAM-BA mode)...', flush=True)
        t0 = time.time()
        while dev is None and time.time() - t0 < a.wait:
            time.sleep(0.3)
            dev = find(SAMBA)
            if dev is None and find(LEGO) is not None:   # firmware running: ask it to reboot into SAM-BA
                print('brick running firmware - sending the boot command', flush=True)
                try:
                    reboot_to_samba()
                except usb.core.USBError as e:   # a half-flashed firmware may vanish again right away
                    print(f'  ({e})', flush=True)
    if dev is None:
        if not reboot_to_samba():
            sys.exit('no NXT on USB (neither LEGO firmware nor SAM-BA mode)')
        print('waiting for SAM-BA mode...')
        for _ in range(40):
            time.sleep(0.5)
            dev = find(SAMBA)
            if dev is not None:
                break
        else:
            sys.exit('the brick did not show up in SAM-BA mode (Windows may need a driver for 03EB:6124)')
    sb = Samba(open_pipe(dev))
    sb.handshake()
    print(f'SAM-BA {sb.version()!r}')

    if a.save_old:
        old = b''.join(sb.read_block(FLASH_BASE + n * PAGE, PAGE) for n in range(PAGES))
        with open(a.save_old, 'wb') as f:
            f.write(old)
        print(f'saved the current flash to {a.save_old} ({len(old)} bytes, vectors ok: {old[:4] == img[:4]})')

    if not a.verify_only:
        sb.prepare()
        t0 = time.time()
        for n in range(PAGES):
            sb.write_page(n, img[n * PAGE:(n + 1) * PAGE])
            if n % 64 == 63:
                print(f'  written {n + 1}/{PAGES} pages ({time.time() - t0:.0f} s)', flush=True)
        sb.flash_wait()

    bad = []
    for n in range(PAGES):
        if sb.read_block(FLASH_BASE + n * PAGE, PAGE) != img[n * PAGE:(n + 1) * PAGE]:
            bad.append(n)
    if bad:
        sys.exit(f'VERIFY FAILED on {len(bad)} pages (first {bad[:8]}); the brick is still in SAM-BA - run again')
    print('verify OK - starting the new firmware')
    sb.jump(FLASH_BASE)


if __name__ == '__main__':
    main()
