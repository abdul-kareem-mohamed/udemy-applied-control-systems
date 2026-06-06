import os
import signal
import socket
import subprocess
import time
import json
import pytest

HOST_CTRL = "127.0.0.1"
PORT_CTRL = 5000
TIMEOUT = 2

# Path to the controller relative to project root
CONTROLLER_PATH = "p1-waterTank/src/p1-waterTank/controller.py"

@pytest.fixture(scope="module")
def controller_process():
    """Starts the controller script and ensures it is cleaned up."""
    # Ensure we use an absolute path or correct relative path from root
    root_dir = os.getcwd()
    script_path = os.path.join(root_dir, CONTROLLER_PATH)
    
    proc = subprocess.Popen(
        ["python3", "-u", script_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid 
    )
    
    # Wait for the server to be ready
    ready = False
    timeout = 5 
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            with socket.create_connection((HOST_CTRL, PORT_CTRL), timeout=0.1):
                ready = True
                break
        except (ConnectionRefusedError, socket.timeout):
            time.sleep(0.1)
    
    if not ready:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        pytest.fail("Controller failed to start or port 5000 is occupied.")

    yield proc
    
    # Cleanup: Kill the process group
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=2)
    except Exception:
        pass

def test_json_protocol(controller_process):
    """Test valid JSON communication with the expected protocol."""
    with socket.create_connection((HOST_CTRL, PORT_CTRL), timeout=TIMEOUT) as s:
        payload = {"target": 10.0, "volume": 5.0}
        s.sendall((json.dumps(payload) + "\n").encode())
        
        data = s.recv(1024).decode()
        resp = json.loads(data.strip())
        
        assert "m_dot" in resp
        # (10.0 - 5.0) * 0.5 = 2.5
        assert resp["m_dot"] == 2.5

def test_multi_packet_buffer(controller_process):
    """Test that the server handles multiple JSON messages in one stream."""
    with socket.create_connection((HOST_CTRL, PORT_CTRL), timeout=TIMEOUT) as s:
        # Send two messages at once
        msg1 = json.dumps({"target": 10.0, "volume": 2.0}) + "\n"
        msg2 = json.dumps({"target": 10.0, "volume": 8.0}) + "\n"
        s.sendall((msg1 + msg2).encode())
        
        # Receive responses
        data = ""
        while data.count("\n") < 2:
            chunk = s.recv(1024).decode()
            if not chunk: break
            data += chunk
            
        lines = data.strip().split("\n")
        assert len(lines) == 2
        
        resp1 = json.loads(lines[0])
        resp2 = json.loads(lines[1])
        
        assert resp1["m_dot"] == 4.0 # (10-2)*0.5
        assert resp2["m_dot"] == 1.0 # (10-8)*0.5

def test_malformed_json_resilience(controller_process):
    """Ensure the server survives malformed data and responds to the next valid one."""
    with socket.create_connection((HOST_CTRL, PORT_CTRL), timeout=TIMEOUT) as s:
        # Send garbage
        s.sendall(b"INVALID_DATA\n")
        
        # Send valid data immediately after
        payload = {"target": 10.0, "volume": 0.0}
        s.sendall((json.dumps(payload) + "\n").encode())
        
        data = s.recv(1024).decode()
        resp = json.loads(data.strip())
        assert resp["m_dot"] == 5.0
