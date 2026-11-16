"""Contract smoke-test server. No PyBullet required; NOT a physics simulator."""
import json
import socketserver
import sys

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            req = json.loads(line)
            op, args = req['op'], req.get('args', {})
            if op == 'hello':
                result = {'protocol': 'swarm-sim/0.1', 'robots': [f'arm_{i}' for i in range(1, 7)]}
            elif op == 'state':
                result = {'id': args['robot_id'], 'available': True, 'valid': True,
                          'status': 'idle', 'tcp': {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]}}
            elif op == 'capabilities':
                result = {'maneuvers': ['Inspect', 'Grasp', 'Move', 'Hold', 'Release', 'Sort']}
            elif op == 'observe':
                result = {'objects': [{'object_id': 'test_object', 'valid': True,
                                       'pose': {'position': [0.1, 0.2, 0.3]}}]}
            else:
                self.wfile.write((json.dumps({'id': req['id'], 'ok': False, 'error': 'unsupported'})+'\n').encode())
                continue
            self.wfile.write((json.dumps({'id': req['id'], 'ok': True, 'result': result})+'\n').encode())

if __name__ == '__main__':
    with socketserver.TCPServer(('127.0.0.1', int(sys.argv[1])), Handler) as server:
        server.serve_forever()
