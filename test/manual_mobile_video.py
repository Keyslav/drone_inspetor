"""Servidor visual de teste com vídeo sintético; nunca importa ROS ou comanda voo.

PYTHONPATH=. .webrtc-venv/bin/python test/manual_mobile_video.py
Abra http://127.0.0.1:8766 e conecte com token demo. Ctrl+C encerra.
"""
import argparse
from pathlib import Path
from threading import Event, Thread

import cv2
import numpy as np

from drone_inspetor.mobile_gateway.api import MobileAPI
from drone_inspetor.mobile_gateway.server import DemoAdapter, GatewayServer, resource_path
from drone_inspetor.mobile_gateway.state import MobileState
from drone_inspetor.mobile_gateway.webrtc import WebRTCService


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--pause-file', type=Path,
                        help='Enquanto este arquivo existir, interrompe somente a câmera sintética.')
    args = parser.parse_args()
    state, stop = MobileState(demo=True), Event()

    def producer():
        counter = 0
        while not stop.is_set():
            if args.pause_file is None or not args.pause_file.exists():
                for index, channel in enumerate(('camera', 'cv', 'depth')):
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    frame[:] = (30 + 25 * index, 25, 15)
                    cv2.putText(frame, 'VIDEO SINTETICO / ' + channel.upper(), (25, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, .75, (195, 230, 240), 2)
                    x = 30 + counter % 440
                    cv2.rectangle(frame, (x, 180), (x + 80, 260), (170, 210, 45), -1)
                    cv2.putText(frame, f'Quadro {counter}', (25, 425),
                                cv2.FONT_HERSHEY_SIMPLEX, .7, (195, 230, 240), 2)
                    state.put_frame(channel, cv2.imencode('.jpg', frame)[1].tobytes())
                counter += 5
            stop.wait(1 / 15)

    thread = Thread(target=producer, daemon=True)
    server = GatewayServer((args.host, args.port),
                           MobileAPI(state, DemoAdapter(), {'Flare': {}}),
                           'demo', resource_path('mobile_web'), video=WebRTCService(state))
    thread.start()
    print(f'Vídeo sintético: http://{args.host}:{args.port} — token demo', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join(3)


if __name__ == '__main__':
    main()
