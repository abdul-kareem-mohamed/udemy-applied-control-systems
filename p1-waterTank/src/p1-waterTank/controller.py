import json
import socket

HOST_CTRL = "127.0.0.1"
PORT_CTRL = 5000

class PIDController:
  def __init__(self, kp=0.5, ki=0.0, kd = 0.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd

  def compute(self, target, value):
    error = target - value
    return max(0.0, self.kp * error)
  

ctrl = PIDController()

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST_CTRL, PORT_CTRL))
server.listen(5)

print("Controller running...")

try:
    while True:
        conn, addr = server.accept()
        with conn:
            buffer = ""
            while True:
                try:
                    data = conn.recv(1024).decode()
                    if not data:
                        break
                    
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        try:
                            msg = json.loads(line)
                            m_dot = ctrl.compute(msg.get("target", 10.0), msg.get("volume", 0.0))
                            conn.send((json.dumps({"m_dot": m_dot}) + "\n").encode())
                        except json.JSONDecodeError:
                            print(f"Invalid JSON: {line}")
                except Exception as e:
                    print(f"Connection error with {addr}: {e}")
                    break
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    server.close()
