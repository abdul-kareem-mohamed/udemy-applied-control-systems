import json
import logging
import select
import socket
import time

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


HOST_UI = "127.0.0.1"
PORT_UI = 5001

ui_server = None
buffer = ""

t_vals = []
v_vals = []
m_vals = []
target_vals = []

def connect_to_tank_ui_server():
  global ui_server
  
  if ui_server:
    try:
      ui_server.close()
    except OSError:
      pass
    
  while True:
    try:
      logger.info("Attempting to connect to tank at %s:%s...", HOST_UI, PORT_UI)
      ui_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      ui_server.connect((HOST_UI, PORT_UI))
      logger.info("Successfully connected to tank!")
      break
    except (ConnectionRefusedError, OSError) as e:
      logger.warning("Connection failed (%s). Retrying in 1 seconds...", e)
      time.sleep(1)
      
def get_tank_data():
  """Read data if available. Return false if the connection is dropped"""
  global t_vals, v_vals, m_vals, target_vals, buffer

  try:
    ready_to_read, _, _ = select.select([ui_server], [], [], 0.0)
    
    if ready_to_read:
      data = ui_server.recv(1024).decode()
      
      if not data:
        logger.warning("UI Server closed the connection ")
      
      buffer += data

      while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        try:
          msg = json.loads(line)
          t_vals.append(msg.get("t", 0.0))
          v_vals.append(msg.get("volume", 0.0))
          m_vals.append(msg.get("m_dot", 0.0))         # <--- Changed to safely default to 0.0
          target_vals.append(msg.get("target", 0.0))
        except json.JSONDecodeError:
          logger.warning("Failed to parse JSON: %s", line)

    return True

  except (ConnectionResetError, BrokenPipeError, OSError) as e:
    logger.warning("Connection error while reading: %s", e)
    return False 


connect_to_tank_ui_server()

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
ax_graph.set_ylim(0, 120)
ax_graph.set_xlim(0, 10)

def update(frame_index):
  global t_vals, v_vals, m_vals, target_vals
  
  if not get_tank_data():
    logger.info("Reconnecting to tank simulaiton...")
    connect_to_tank_ui_server()
  
  if len(v_vals) == 0:
    return fluid, actual_history_line, target_indicator, target_history_line

  # --- update tank visual ---  
  current_volume = v_vals[-1]
  current_volume = max(0, min(current_volume, 100))
  fluid.set_data([0, 0], [0, current_volume])
  
  # --- update target indicator ---
  current_target = target_vals[-1]
  target_indicator.set_data([-5, 5], [current_target, current_target])
  
  # --- update graph ---
  actual_history_line.set_data(t_vals, v_vals)
  target_history_line.set_data(t_vals, target_vals)
  
  current_time = t_vals[-1]
  if current_time > ax_graph.get_xlim()[1]:
    ax_graph.set_xlim(current_time - 10, current_time + 2)
    fig.canvas.draw_idle()
    
  return (fluid, actual_history_line, target_indicator, target_history_line)


fps = 50
interval = 1000 / fps
ani = animation.FuncAnimation(
  fig, update, frames=range(0, 101, 2), blit=True, interval=interval
)

plt.show()