import glob
import json
import logging
import select
import socket
import time

from sympy import false

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

HOST_CTRL = "127.0.0.1"
PORT_CTRL = 5000

HOST_UI = "127.0.0.1"
PORT_UI = 5001

buffer = ""
m_dot = 0.0

ctrl_server = None
ui_server = None
ui_conn = None

def connect_to_controller():
  global ctrl_server
  
  if ctrl_server:
    try:
      ctrl_server.close()
    except OSError:
      pass
  while True:
    try:
      logger.info("Attempting to connect to controller at %s:%s...", HOST_CTRL, PORT_CTRL)
      ctrl_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      ctrl_server.connect((HOST_CTRL, PORT_CTRL))
      logger.info("Successfully connected to controller!")
      break
    except (ConnectionRefusedError, OSError) as e:
      logger.warning("Connection failed (%s). Retrying in 1 seconds...", e)
      time.sleep(1)


def create_ui_server():
  global ui_server
  
  if ui_server:
    try:
      ui_server.close()
    except OSError:
      pass

  logger.info("Setting up UI server on %s:%s...", HOST_UI, PORT_UI)
  ui_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  ui_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  ui_server.bind((HOST_UI, PORT_UI))
  ui_server.listen(1)
  ui_server.setblocking(False)
  logger.info("Waiting for UI to connect on %s:%s...", HOST_UI, PORT_UI)
        

def check_ui_connection():
  global ui_server, ui_conn
  
  if ui_conn is None:
    try:     
      ui_conn, ui_addr = ui_server.accept()
      logger.info("UI connected from %s!", ui_addr)        
    except BlockingIOError:
      # Expected error when no client is waiting to connect
      pass


def process_controller_data():
  """Read data if available. Return false if the connection is dropped"""
  global m_dot, buffer

  try:
    ready_to_read, _, _ = select.select([ctrl_server], [], [], 0.0)
      
    if ready_to_read:
      data = ctrl_server.recv(1024).decode()
    
      if not data:
        logger.warning("Controller closed the connection.")  
        return
      
      buffer += data

      while "\n" in buffer:    
        line, buffer = buffer.split("\n", 1)
        try: 
          msg = json.loads(line)
          m_dot = msg.get("m_dot", m_dot)
        except json.JSONDecodeError:
          logger.warning("Failed to parse JSON: %s", line)
      
      return True
    
  except (ConnectionResetError, BrokenPipeError, OSError) as e:
    logger.warning("Connection error while reading: %s", e)
    return False 

  
t = 0.0
dt = 0.04
density = 1000
volume = 0.0
target_level = 100
m_dot = 0.0

connect_to_controller()
create_ui_server()

while True:

  if not process_controller_data():
    logger.info("Reconnecting to controller ...")
    connect_to_controller()
    
  
  # volume_1[i] = volume_1[i-1] + (m_dot1[i] + m_dot1[i-1]) / (2 * density) * dt
  volume += (m_dot / density) * dt
  t += dt

  
  # ---- send to controller ----
  ctrl_server.send((json.dumps({
      "t": t,
      "volume": volume,
      "target": target_level
  }) + "\n").encode())
  
  logger.debug("t: %.2f, volume: %.6f, m_dot: %.2f", t, volume, m_dot)

  # ---- send to visualizer ----
  check_ui_connection()
  
  if ui_conn:
    try:
      ui_conn.send((json.dumps({
          "t": t,
          "volume": volume,
          "m_dot": m_dot,
          "target": target_level
      }) + "\n").encode())
    except (ConnectionResetError, BrokenPipeError, OSError) as e:
      logger.warning("UI disconneted.")
      ui_conn.close()
      ui_conn = None
      
  time.sleep(dt)