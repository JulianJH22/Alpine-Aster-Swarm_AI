"""Contract test of C++ two-arm demo; synthetic TCP peer, no PyBullet."""
import json
import socketserver
import subprocess
import sys
import threading

class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    def __init__(self, addr, handler):
        super().__init__(addr, handler)
        self.jobs = {}
        self.calls = []

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            req = json.loads(line)
            op, args = req['op'], req.get('args', {})
            self.server.calls.append((op, args))
            try:
                if op == 'hello':
                    result = {'protocol': 'swarm-sim/0.1', 'robots': [f'arm_{i}' for i in range(1, 7)]}
                elif op == 'observe':
                    result = {'objects': [{'object_id': f'obj_{i}', 'valid': True} for i in (1, 2)]}
                elif op == 'submit':
                    cmd = args['command']
                    assert cmd['robot_id'] in ('arm_1', 'arm_4')
                    assert cmd['execution_id'] not in self.server.jobs
                    self.server.jobs[cmd['execution_id']] = 'accepted'
                    result = {'status': 'accepted', 'detail': ''}
                elif op == 'run_until_idle':
                    assert len(self.server.jobs) in (2, 4, 6, 8)
                    for key in self.server.jobs:
                        self.server.jobs[key] = 'completed'
                    result = {'idle': True}
                elif op == 'status':
                    result = {'status': self.server.jobs[args['execution_id']], 'detail': 'ok'}
                elif op == 'cancel':
                    result = {'cancelled': True}
                else:
                    raise ValueError(op)
                response = {'id': req['id'], 'ok': True, 'result': result}
            except Exception as exc:
                response = {'id': req['id'], 'ok': False, 'error': str(exc)}
            self.wfile.write((json.dumps(response) + '\n').encode())

if __name__ == '__main__':
    with Server(('127.0.0.1', 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            runs = [subprocess.run([sys.argv[1], '127.0.0.1', str(server.server_address[1]), sys.argv[2]],
                                    capture_output=True, text=True, timeout=15) for _ in range(2)]
            for result in runs:
                assert result.returncode == 0, result.stdout + result.stderr
            calls = server.calls
            assert [op for op, _ in calls].count('submit') == 8, calls
            assert [op for op, _ in calls].count('run_until_idle') == 4, calls
            first_wait = next(i for i, (op, _) in enumerate(calls) if op == 'run_until_idle')
            assert [op for op, _ in calls[:first_wait]].count('submit') == 2
            assert len(server.jobs) == 8
            ids = [args['command']['execution_id'] for op, args in calls if op == 'submit']
            assert len(ids) == len(set(ids)), ids
            assert len({identifier.split(':')[0] for identifier in ids}) == 2
            assert 'Demo completada' in result.stdout
            print(result.stdout)
        finally:
            server.shutdown()
