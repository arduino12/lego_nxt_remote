"""Pair this Windows PC with an NXT over Bluetooth (PIN 1234), so it gets a Bluetooth COM port.

  pip install winrt-runtime winrt-Windows.Devices.Bluetooth winrt-Windows.Devices.Enumeration winrt-Windows.Foundation winrt-Windows.Foundation.Collections
  python bt_pair.py 00:16:53:04:82:50

The NXT beeps and asks for the passkey on its screen - press its orange button to accept 1234.
"""
import asyncio
import sys

from winrt.windows.devices.bluetooth import BluetoothDevice
from winrt.windows.devices.enumeration import DevicePairingKinds, DevicePairingResultStatus

PIN = '1234'


async def main(addr):
    dev = await BluetoothDevice.from_bluetooth_address_async(int(addr.replace(':', ''), 16))
    if dev is None:
        sys.exit(f'{addr} not found (is the NXT on, with Bluetooth on?)')
    info = dev.device_information
    print(f'{dev.name or "?"} {addr}: paired={info.pairing.is_paired}')
    if info.pairing.is_paired:
        return

    def on_request(sender, args):
        print(f'pairing request: {args.pairing_kind}')
        if args.pairing_kind == DevicePairingKinds.PROVIDE_PIN:
            args.accept_with_pin(PIN)
        else:
            args.accept()

    custom = info.pairing.custom
    token = custom.add_pairing_requested(on_request)
    print('pairing... press the orange button on the NXT when it beeps')
    kinds = DevicePairingKinds.PROVIDE_PIN | DevicePairingKinds.CONFIRM_ONLY | DevicePairingKinds.CONFIRM_PIN_MATCH
    res = await custom.pair_async(kinds)
    custom.remove_pairing_requested(token)
    status = DevicePairingResultStatus(res.status)
    print(f'result: {status.name}')
    sys.exit(0 if status in (DevicePairingResultStatus.PAIRED, DevicePairingResultStatus.ALREADY_PAIRED) else 1)


if __name__ == '__main__':
    asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else '00:16:53:04:82:50'))
