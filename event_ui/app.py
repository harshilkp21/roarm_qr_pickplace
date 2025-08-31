# app.py (modified)
from flask import Flask, render_template, Response
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

app = Flask(__name__)
guest_listener = None

class GuestListener(Node):
    def __init__(self):
        super().__init__('guest_web_listener')
        self.qr_sub = self.create_subscription(String, 'qr_visitor', self.qr_callback, 10)
        self.resume_sub = self.create_subscription(String, 'resume_scanner', self.resume_callback, 10)

        self.latest_name = ""
        self.status = "success"
        self.resume_signal = False

        self.seen = set()       # track who already got a memento
        self.msg_seq = 0        # increment on every scan -> forces SSE event
        self.lock = threading.Lock()

    def qr_callback(self, msg):
        # parse incoming msg: expect "status:name" (e.g., "success:John Doe", "unlisted:XYZ")
        parts = msg.data.split(":", 1)
        if len(parts) == 2:
            status_in, name = parts[0].strip(), parts[1].strip()
        else:
            status_in, name = "success", msg.data.strip()

        with self.lock:
            # if already seen -> force "already"
            if name in self.seen:
                self.status = "already"
            else:
                # if the incoming status is success, mark them seen
                if status_in == "success":
                    self.seen.add(name)
                self.status = status_in

            self.latest_name = name
            self.msg_seq += 1   # IMPORTANT: increment so SSE sends event even if name same

        print(f"[QR Web] [{self.status}] Guest: {self.latest_name}")

    def resume_callback(self, msg):
        with self.lock:
            self.resume_signal = True
        print(f"[QR Web] Resume signal received.")

def ros_spin():
    global guest_listener
    rclpy.init()
    guest_listener = GuestListener()
    rclpy.spin(guest_listener)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/events')
def sse():
    def event_stream():
        last_seq = -1
        while True:
            if guest_listener is None:
                time.sleep(0.1)
                continue

            # copy shared state under lock
            with guest_listener.lock:
                seq = guest_listener.msg_seq
                status = guest_listener.status
                name = guest_listener.latest_name
                resume_flag = guest_listener.resume_signal
                if resume_flag:
                    guest_listener.resume_signal = False

            # If seq changed -> send qr event (this handles repeated scans)
            if seq != last_seq and name.strip() != "":
                last_seq = seq
                yield f"event: qr\n"
                yield f"data: {status}:{name}\n\n"

            # If resume flag was set -> send resume event
            if resume_flag:
                yield f"event: resume\n"
                yield f"data: resume\n\n"

            time.sleep(0.15)

    headers = {
        "Cache-Control": "no-cache",
        "X-Accel-Buffering": "no"   # disable buffering in some proxies
    }
    return Response(event_stream(), mimetype='text/event-stream', headers=headers)

if __name__ == '__main__':
    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(debug=False, host='0.0.0.0', port=5000)
