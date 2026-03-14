import serial
import colorama
from cobs import cobs

ser = serial.Serial('/dev/ttyACM1')
buffer = bytearray()


def read() -> tuple[bytes, bytes]:
    global buffer, ser
    msg = None, None
    while msg[0] is None:
        while b'\0' not in buffer:
            buffer.extend(ser.read(1))
            buffer.extend(ser.read(-1))
        assert b'\0' in buffer
        msg, buffer = buffer.split(b'\0', 1)
        try:
            msg = cobs.decode(msg)
        except cobs.DecodeError:
            print(f'Could not decode bytes: {msg.hex()}')
        assert len(msg) >= 4
        msg = bytes(msg[:4]), bytes(msg[4:])
    return msg


while True:
    data = read()
    if not data:
        continue
    if data[0] == b'sout':
        text = data[1].decode()
    elif data[0] == b'serr':
        text = '{}{}{}'.format(colorama.Fore.RED,
                               data[1].decode(), colorama.Style.RESET_ALL)
    elif data[0] == b'kdbg':
        text = '{}\n\nKERNEL DEBUG:\t{}{}\n'.format(colorama.Back.GREEN + colorama.Style.BRIGHT,
                                                    data[1].decode(),
                                                    colorama.Style.RESET_ALL)
    elif data[0] != b'':
        text = '{}{}'.format(data[0].decode, data[1].decode())
    else:
        text = "{}".format(data[1].decode())
    print(text, end='')
