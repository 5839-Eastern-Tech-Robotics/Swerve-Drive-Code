from cobs import cobs
from collections import deque
import colorama
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
import threading

ser = serial.Serial('/dev/ttyACM1')
buffer = bytearray()

max_points = 200
current_data = deque([0]*max_points, maxlen=max_points)
desired_data = deque([0]*max_points, maxlen=max_points)
error_data = deque([0]*max_points, maxlen=max_points)
output_data = deque([0]*max_points, maxlen=max_points)

fig, ax = plt.subplots()

line_current, = ax.plot(current_data, label="current")
line_desired, = ax.plot(desired_data, label="desired")
line_error, = ax.plot(error_data, label="error")
line_output, = ax.plot(output_data, label="output")

ax.legend()
ax.set_title("Swerve Turn PID")
ax.set_xlabel("Samples")
ax.set_ylabel("Value")
ax.set_ylim(-180, 180)


def read() -> tuple[bytes, bytes]:
    global buffer, ser
    msg = None, None
    while msg[0] is None:
        while b'\0' not in buffer:
            buffer.extend(ser.read(ser.in_waiting or 1))
        assert b'\0' in buffer
        msg, buffer = buffer.split(b'\0', 1)
        try:
            msg = cobs.decode(msg)
        except cobs.DecodeError:
            print(f'Could not decode bytes: {msg.hex()}')
        assert len(msg) >= 4
        msg = bytes(msg[:4]), bytes(msg[4:])
    return msg


def write(self, data: str | bytes):
    if isinstance(data, str):
        data = data.encode(encoding='ascii')
    self.ser.write(data)


def console_thread():
    while True:
        try:
            line = input()

            if not line:
                continue

            # Example command: 0.1 0.0 0.2
            parts = line.split()

            if len(parts) != 3:
                continue

            kp, ki, kd = parts
            payload = f"{kp} {ki} {kd}".encode()

            write(payload)

        except Exception as e:
            print("Console error:", e)


def update(frame):
    global time
    data = read()
    if not data:
        return
    if data[0] == b'sout':
        text = data[1].decode()
    elif data[0] == b'serr':
        text = '{}{}{}'.format(colorama.Fore.RED,
                               data[1].decode(), colorama.Style.RESET_ALL)
    elif data[0] == b'kdbg':
        text = '{}\n\nKERNEL DEBUG:\t{}{}\n'.format(
            colorama.Back.GREEN + colorama.Style.BRIGHT,
            data[1].decode(),
            colorama.Style.RESET_ALL
        )
    elif data[0] != b'':
        text = '{}{}'.format(data[0].decode(), data[1].decode())
    else:
        text = "{}".format(data[1].decode())

    if text.startswith('[MODULE_TURN]'):
        _, current, desired, error, output = text.split(' ')

        current = float(current)
        desired = float(desired)
        error = float(error)
        output = float(output)

        current_data.append(current)
        desired_data.append(desired)
        error_data.append(error)
        output_data.append(output)

        x = range(len(current_data))

        line_current.set_data(x, current_data)
        line_desired.set_data(x, desired_data)
        line_error.set_data(x, error_data)
        line_output.set_data(x, output_data)

        # print(current, desired, error, output)

        # ax.relim()
        # ax.autoscale_view()
    else:
        print(text)

    return line_current, line_desired, line_error, line_output


line_current.set_linestyle("None")
line_desired.set_linestyle("None")
line_output.set_linestyle("None")

threading.Thread(target=console_thread, daemon=True).start()
ani = animation.FuncAnimation(fig, update, interval=10, cache_frame_data=False)
plt.show()
