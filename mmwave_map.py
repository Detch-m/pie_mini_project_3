"""Maps the sensor data"""

import math
import serial
import matplotlib.patches as patches
import matplotlib.pyplot as plt

PORT = "COM9"  # change this to your port, e.g. "/dev/ttyACM0" on Linux

ser = serial.Serial(PORT, 115200, timeout=1)

plt.ion()
fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
ax.plot()

while plt.fignum_exists(fig.number):
    line = ser.readline().decode(errors="ignore").strip()
    try:
        values = [float(v) for v in line.split(",")] if line else []
    except ValueError:
        continue
    if len(values) % 2 != 0:
        continue

    xs = values[0::2]  # every even item is an x
    ys = values[1::2]  # every odd item is a y
    if len(xs) > 0 and len(ys) > 0:
        angle = 90 - math.atan2(ys[0], xs[0]) * (180 / math.pi)
    else:
        angle = 0

    ax.clear()
    ax.scatter(xs, ys, s=100)
    ax.plot(0, 0, "k^")  # the radar

    # 3. Create the bounding circle overlay (e.g., 1.5-meter detection radius)
    radar_boundary = patches.Circle(
        (0, 0), radius=1.5, transform=ax.transData, fill=False, color="black", lw=2
    )
    ax.add_patch(radar_boundary)

    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(0, 1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{len(xs)} people")
    print(angle)
    plt.pause(0.01)
