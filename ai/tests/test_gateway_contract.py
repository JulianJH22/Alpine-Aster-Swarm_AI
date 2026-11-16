"""Smoke-test C++ gateway against a stdlib Python protocol stub."""
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

exe = Path(sys.argv[1])
server = Path(__file__).with_name('mock_gateway.py')
with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
p = subprocess.Popen([sys.executable, str(server), str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
try:
    for _ in range(50):
        if p.poll() is not None:
            raise RuntimeError('Stub gateway exited early')
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1', port)) == 0:
                break
        time.sleep(.03)
    result = subprocess.run([str(exe), '127.0.0.1', str(port)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert '6 agents, 1 observed objects' in result.stdout, result.stdout
    print('Gateway contract smoke test passed')
finally:
    p.terminate()
    p.wait(timeout=5)
