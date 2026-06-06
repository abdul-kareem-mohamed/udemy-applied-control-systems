import json
import logging
import socket

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

HOST_UI = "127.0.0.1"
PORT_UI = 5001

# --- connect to tank ---
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST_UI, PORT_UI))

buffer = ""

t_vals = []
v_vals = []
m_vals = []

# --- Plot Setup ---
fig = plt.figure(figsize=(12, 7), facecolor=(0.85, 0.85, 0.85))
gs = gridspec.GridSpec(1, 2, width_ratios=[1, 2])

    
# Tank 1
ax1 = fig.add_subplot(gs[0, 0], facecolor="#e6e6e6")
fluid, = ax1.plot([0, 0], [-80, -80], 'royalblue', linewidth=140, solid_capstyle='butt')
target_indicator, = ax1.plot([-5, 5], [0, 0], 'r--', linewidth=2, label="Target Level")
ax1.set_title("Tank 1", fontsize=11)
ax1.axhline(0, color="maroon", linewidth=2)
ax1.set_xlim(-5, 5)
ax1.set_ylim(0, 100)
plt.ylabel('Tank Volume [m^3]')
plt.title('Physical Tank State')
ax1.get_xaxis().set_visible(False) # Hide pointless horizontal tracking metrics here
plt.legend(loc='upper right')

# Right Window: Chronological History Plot Line
ax_graph = fig.add_subplot(gs[0, 1], facecolor=(0.95, 0.95, 0.95))
target_history_line, = ax_graph.plot([], [], 'r--', linewidth=1.5, label='Target Trajectory')
actual_history_line, = ax_graph.plot([], [], 'blue', linewidth=3, label='Actual Fluid Level')
plt.xlabel('Time [seconds]')
plt.ylabel('Volume [m^3]')
plt.title('System Timeline Response')
plt.grid(True)
plt.legend(loc='lower right')


def update(frame_index):
  global buffer
  
  # --- read socket ---
  data = client.recv(1024).decode()
  buffer += data

  while "\n" in buffer:
    line, buffer = buffer.split("\n", 1)
    msg = json.loads(line)
    t_vals.append(msg["t"])
    v_vals.append(msg["volume"] * 1e3)
    m_vals.append(msg["m_dot"]) 
  
    if len(v_vals) == 0:
      return fluid, actual_history_line
  
    # --- update tank visual ---  
    current_volume = v_vals[frame_index]
    current_volume = max(0, min(current_volume, 100))
    fluid.set_data([0, 0], [0, current_volume])
    
    # --- update graph ---
    time_so_far = t_vals[:frame_index + 1]
    vol_so_far = v_vals[:frame_index + 1] 
    actual_history_line.set_data(time_so_far, vol_so_far)
    # target_history_line.set_data([0, t_end], [target_level, target_level])
    
    return (fluid, actual_history_line)
    # return (fluid, actual_history_line, target_history_line)


fps = 50
interval = 1000 / fps
ani = animation.FuncAnimation(
  fig, update, frames=range(0, 101, 2), blit=True, interval=interval
)

plt.show()