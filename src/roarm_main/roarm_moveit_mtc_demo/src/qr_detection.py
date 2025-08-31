#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar.pyzbar import decode
import os
from pydub import AudioSegment
from pydub.playback import play
import openai
import threading
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        self.publisher_ = self.create_publisher(String, 'qr_visitor', 10)
        self.publisher_pick_ = self.create_publisher(String, 'pick', 10)
        self.resume_subscriber = self.create_subscription(String,'resume_scanner',self.resume_callback,10)     # added this
        self.timer = self.create_timer(0.05, self.scan_callback)  # 20 FPS
        self.paused = False

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.scanned_file = 'scanned_guests.txt'
        self.allowed_file = 'allowed_guests.txt'
        self.scanned_names = set()
        self.allowed_names = set()
        self.cooldowns = {}  # per-name cooldown timestamps

        self.load_scanned_names()
        self.load_allowed_names()
        self.clear_scanned_file()  # ✅ Clear file on startup

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            exit(1)

        openai.api_key = os.getenv("OPENAI_API_KEY")

    def load_scanned_names(self):
        if os.path.exists(self.scanned_file):
            with open(self.scanned_file, 'r') as f:
                self.scanned_names = set(line.strip() for line in f.readlines())

    def load_allowed_names(self):
        if os.path.exists(self.allowed_file):
            with open(self.allowed_file, 'r') as f:
                self.allowed_names = set(line.strip() for line in f.readlines())
        else:
            self.get_logger().warn(f"{self.allowed_file} not found. No guests will be allowed.")

    def clear_scanned_file(self):  # ✅ New function to clear the scanned file
        with open(self.scanned_file, 'w') as f:
            f.write('')

    def save_name(self, name):
        with open(self.scanned_file, 'a') as f:
            f.write(f"{name}\n")

    def scan_callback(self):
        ret, frame = self.cap.read()

        if ret:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image publish failed: {e}")

        if self.paused:                                                         # added this
            # cv2.imshow("QR Scanner", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                rclpy.shutdown()
            return

        if not ret:
            self.get_logger().warn("Frame drop detected, skipping...")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small_gray = cv2.resize(gray, (640, 480))

        decoded_objects = decode(small_gray)
        current_time = time.time()

        if decoded_objects:
            obj = decoded_objects[0]
            name = obj.data.decode('utf-8').strip()

            last_time = self.cooldowns.get(name, 0)
            if current_time - last_time < 5.0:
                # Skip if within cooldown
                return

            self.cooldowns[name] = current_time

            self.publish_name(name)

            if name in self.allowed_names:
                if name not in self.scanned_names:
                    self.get_logger().info(f"Allowed guest {name}, first scan.")
                    self.scanned_names.add(name)
                    self.save_name(name)

                    self.get_logger().info("Publishing pick message to robot.")
                    self.publisher_pick_.publish(String(data="pick"))

                    self.paused = True                                              # (Added this later) Pause scanning until robot completes pick                                                 
                    self.get_logger().info("Scanning paused until robot finishes.")

                    threading.Thread(target=self.speak_welcome, args=(name,), daemon=True).start()
                else:
                    self.publish_name(name, status="already")
                    self.get_logger().info(f"{name} already scanned. You already received your memento.")
                    threading.Thread(target=self.speak_already_received, args=(name,), daemon=True).start()
            else:
                self.publish_name(name, status="unlisted")
                self.get_logger().info(f"Visitor {name} not in allowed list—no pick message sent.")
                threading.Thread(target=self.speak_unlisted, args=(name,), daemon=True).start()

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def publish_name(self, name, status="success"):
        msg = String()
        msg.data = f"{status}:{name}"
        self.publisher_.publish(msg)


    def speak_welcome(self, name):
        text = f"Welcome to TechXchange event, {name}."
        self.play_audio(text)
        self.get_logger().info(f"Welcome audio would play for {name}.")

    def speak_already_received(self, name):
        text = f"{name}, you have already received your memento."
        self.play_audio(text)
        self.get_logger().info(f"Already received audio would play for {name}.")

    def speak_unlisted(self, name):
        text = f"Sorry, you are not on the guest list."
        self.play_audio(text)
        self.get_logger().info(f"Unlisted audio would play for {name}.")

    def resume_callback(self, msg):                                                    # added this function                                                
        self.get_logger().info("Resuming QR scanning after robot completed pick.")
        self.paused = False

    def play_audio(self, text):
        try:
            response = openai.audio.speech.create(
                model="tts-1",
                voice="shimmer",
                input=text
            )
            audio_data = response.content
            temp_path = "temp_audio.mp3"
            with open(temp_path, "wb") as f:
                f.write(audio_data)
            audio = AudioSegment.from_mp3(temp_path)
            slower_audio = audio._spawn(audio.raw_data, overrides={"frame_rate": int(audio.frame_rate * 0.9)})
            slower_audio = slower_audio.set_frame_rate(audio.frame_rate)
            play(slower_audio)
            os.remove(temp_path)
        except Exception as e:
            self.get_logger().error(f"Audio generation failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info("Shutting down and clearing scanned_guests.txt.")  # ✅ Log message on shutdown
        node.clear_scanned_file()  # ✅ Clear file on shutdown
        node.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()