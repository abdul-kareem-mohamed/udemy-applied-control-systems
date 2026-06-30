import json
import logging
import socket

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

HOST_CTRL = "127.0.0.1"
PORT_CTRL = 5000


class PIDController:
  def __init__(self, kp=1000.0, ki=0.0, kd = 0.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd

  def compute(self, target, value):
    error = target - value
    return max(0.0, self.kp * error)
  

ctrl = PIDController()

ctrl_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ctrl_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
ctrl_server.bind((HOST_CTRL, PORT_CTRL))
ctrl_server.listen(1)

logger.info("Controller running...")

try:
    while True:
        ctrl_conn, addr = ctrl_server.accept()
        with ctrl_conn:
            buffer = ""
            while True:
                try:
                    data = ctrl_conn.recv(1024).decode()
                    if not data:
                        logger.error("No data recieved")
                        break
                    
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        try:
                            msg = json.loads(line)
                            m_dot = ctrl.compute(msg.get("target", 1.0), msg.get("volume", 0.0))
                            ctrl_conn.send((json.dumps({"m_dot": m_dot}) + "\n").encode())
                            logger.debug("Sent: m_dot: %.2f", m_dot)
                            
                        except json.JSONDecodeError:
                            logger.error(f"Invalid JSON: {line}")
                except Exception as e:
                    logger.error(f"Connection error with {addr}: {e}")
                    break
except KeyboardInterrupt:
    logger.info("Shutting down...")
finally:
    ctrl_server.close()
