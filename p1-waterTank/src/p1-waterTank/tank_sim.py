import json
import logging
import socket
import time

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

HOST_CTRL = "127.0.0.1"
PORT_CTRL = 5000

HOST_UI = "127.0.0.1"
PORT_UI = 5001

# --- connect to controlller ---
ctrl_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ctrl_server.connect((HOST_CTRL, PORT_CTRL))

# --- UI server ----
ui_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ui_server.bind((HOST_UI, PORT_UI))
ui_server.listen(1)

ui_conn, _ = ui_server.accept()

buffer = ""

def controller_listener():
  global m_dot, buffer

  while True:  
    data = ctrl_server.recv(1024).decode()
    
    if not data:
      break
    
    buffer += data

    while "\n" in buffer:    
      line, buffer = buffer.split("\n", 1)
      msg = json.loads(line)
      m_dot = msg["m_dot"]


t = 0.0
dt = 0.04
density = 1000
volume = 0.0
target_level = 10
m_dot = 0.0

while True:
  # volume_1[i] = volume_1[i-1] + (m_dot1[i] + m_dot1[i-1]) / (2 * density) * dt
  volume += (m_dot / density) * dt
  t += dt

  controller_listener()
  
  # ---- send to controller ----
  ctrl_server.send((json.dumps({
      "t": t,
      "volume": volume
  }) + "\n").encode())

  # ---- send to visualizer ----
  ui_server.send((json.dumps({
      "t": t,
      "volume": volume,
      "m_dot": m_dot
  }) + "\n").encode())
  
  time.sleep(dt)