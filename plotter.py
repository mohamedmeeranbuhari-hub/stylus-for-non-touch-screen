"""
Real-Time 6-Axis MPU-6050 Live Waveform Plotter
Usage: python plotter.py [COM_PORT] (Defaults to COM7 at 115200 baud)
"""

import sys
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
BAUD = 115200
HISTORY_LEN = 100

print(f"Connecting to {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Failed to open {PORT}: {e}")
    print("Available ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")
    sys.exit(1)

# Ring buffers for 100 historical points
time_pts = deque(maxlen=HISTORY_LEN)
ax_pts = deque(maxlen=HISTORY_LEN)
ay_pts = deque(maxlen=HISTORY_LEN)
az_pts = deque(maxlen=HISTORY_LEN)
gx_pts = deque(maxlen=HISTORY_LEN)
gy_pts = deque(maxlen=HISTORY_LEN)
gz_pts = deque(maxlen=HISTORY_LEN)

fig, (ax_plot, gyro_plot) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.canvas.manager.set_window_title('MPU-6050 Live Telemetry Plotter')

# Accelerometer Lines
line_ax, = ax_plot.plot([], [], label='Accel X (g)', color='#e74c3c', lw=1.8)
line_ay, = ax_plot.plot([], [], label='Accel Y (g)', color='#2ecc71', lw=1.8)
line_az, = ax_plot.plot([], [], label='Accel Z (g)', color='#3498db', lw=1.8)
ax_plot.set_ylabel('Acceleration (g)')
ax_plot.set_ylim(-3.0, 3.0)
ax_plot.grid(True, linestyle='--', alpha=0.6)
ax_plot.legend(loc='upper right')
ax_plot.set_title('Accelerometer Output')

# Gyroscope Lines
line_gx, = gyro_plot.plot([], [], label='Gyro X (°/s)', color='#e67e22', lw=1.8)
line_gy, = gyro_plot.plot([], [], label='Gyro Y (°/s)', color='#9b59b6', lw=1.8)
line_gz, = gyro_plot.plot([], [], label='Gyro Z (°/s)', color='#1abc9c', lw=1.8)
gyro_plot.set_ylabel('Angular Rate (°/s)')
gyro_plot.set_xlabel('Sample Count')
gyro_plot.set_ylim(-250.0, 250.0)
gyro_plot.grid(True, linestyle='--', alpha=0.6)
gyro_plot.legend(loc='upper right')
gyro_plot.set_title('Gyroscope Output')

plt.tight_layout()

sample_idx = 0

def update_plot(frame):
    global sample_idx
    # Read all pending serial lines
    while ser.in_waiting > 0:
        try:
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw_line or ':' not in raw_line:
                continue

            # Parse "Label:Value" pairs
            tokens = raw_line.split()
            data = {}
            for t in tokens:
                if ':' in t:
                    k, v = t.split(':', 1)
                    try:
                        data[k.strip()] = float(v.strip())
                    except ValueError:
                        pass

            if 'Accel_X' in data and 'Gyro_X' in data:
                sample_idx += 1
                time_pts.append(sample_idx)
                ax_pts.append(data.get('Accel_X', 0.0))
                ay_pts.append(data.get('Accel_Y', 0.0))
                az_pts.append(data.get('Accel_Z', 0.0))
                gx_pts.append(data.get('Gyro_X', 0.0))
                gy_pts.append(data.get('Gyro_Y', 0.0))
                gz_pts.append(data.get('Gyro_Z', 0.0))
        except Exception:
            pass

    if len(time_pts) > 0:
        x_data = list(time_pts)
        line_ax.set_data(x_data, list(ax_pts))
        line_ay.set_data(x_data, list(ay_pts))
        line_az.set_data(x_data, list(az_pts))
        ax_plot.set_xlim(x_data[0], max(x_data[-1], x_data[0] + 10))

        line_gx.set_data(x_data, list(gx_pts))
        line_gy.set_data(x_data, list(gy_pts))
        line_gz.set_data(x_data, list(gz_pts))
        gyro_plot.set_xlim(x_data[0], max(x_data[-1], x_data[0] + 10))

    return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

ani = animation.FuncAnimation(fig, update_plot, interval=33, blit=False)

try:
    plt.show()
finally:
    ser.close()
    print("Serial port closed.")
