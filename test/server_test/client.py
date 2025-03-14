import sys
import socket
import struct
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QPushButton, QSlider, QVBoxLayout, QHBoxLayout,
                             QGridLayout, QCheckBox, QFrame, QSpacerItem, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage

SERVER_IP = "localhost"
SERVER_PORT = 8080

class ClientGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.sock = None
        self.connected = False

        # State variables
        self.auto_exposure = False
        self.continuous_capture = False

        # Initial exposure and brightness values
        self.exposure_time = 1000
        self.exposure_gain = 1
        self.target_brightness = 100

        self.init_ui()
        self.connect_to_server()

        # Use a QTimer to request images periodically for continuous preview
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(100)  # Try to update the image every 100ms

    def init_ui(self):
        # Main layout
        main_layout = QHBoxLayout(self)

        # Left side image preview
        left_layout = QVBoxLayout()

        self.preview_label = QLabel("No Image")
        self.preview_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.preview_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.preview_label)

        bottom_layout = QHBoxLayout()
        self.btn_single = QPushButton("Single Capture")
        self.btn_single.clicked.connect(self.single_capture)
        self.btn_continuous = QPushButton("Continuous Capture")
        self.btn_continuous.clicked.connect(self.toggle_continuous_capture)
        bottom_layout.addWidget(self.btn_single)
        bottom_layout.addWidget(self.btn_continuous)
        left_layout.addLayout(bottom_layout)

        # Right side parameter adjustment area
        right_layout = QVBoxLayout()

        # Exposure time slider
        self.slider_exposure = QSlider(Qt.Horizontal)
        self.slider_exposure.setRange(20, 2000)
        self.slider_exposure.setValue(self.exposure_time)
        self.slider_exposure.valueChanged.connect(self.change_exposure)
        right_layout.addWidget(QLabel("Exposure Time"))
        right_layout.addWidget(self.slider_exposure)

        # Target brightness slider
        self.slider_brightness = QSlider(Qt.Horizontal)
        self.slider_brightness.setRange(0, 255)
        self.slider_brightness.setValue(self.target_brightness)
        self.slider_brightness.valueChanged.connect(self.change_brightness)
        right_layout.addWidget(QLabel("Target Brightness"))
        right_layout.addWidget(self.slider_brightness)

        # Spacer for stretching
        right_layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        self.setWindowTitle("Client Preview")
        self.resize(800, 600)

    def connect_to_server(self):
        """Connect to the server."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((SERVER_IP, SERVER_PORT))
            self.connected = True
        except Exception as e:
            print("Failed to connect server:", e)
            self.connected = False

    def send_command(self, cmd: str):
        """Send a command to the server.

        Args:
            cmd (str): The command to send.
        """
        if not self.connected:
            return
        data = cmd.encode('utf-8')
        length = len(data)
        len_pack = struct.pack('!I', length)
        try:
            self.sock.sendall(len_pack)
            self.sock.sendall(data)
        except Exception as e:
            print("Send command error:", e)
            self.connected = False

    def receive_image(self):
        """Receive an image from the server.

        Returns:
            frame: The received image frame.
        """
        try:
            size_data = self.recv_all(4)
            if not size_data:
                return None
            size = struct.unpack('!I', size_data)[0]
            img_data = self.recv_all(size)
            if img_data is None:
                return None
            img_array = np.frombuffer(img_data, np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            print("Receive image error:", e)
            self.connected = False
            return None

    def recv_all(self, size):
        """Receive all data of a given size.

        Args:
            size (int): The size of the data to receive.

        Returns:
            buf: The received data buffer.
        """
        buf = b''
        while len(buf) < size:
            data = self.sock.recv(size - len(buf))
            if not data:
                return None
            buf += data
        return buf

    def single_capture(self):
        """Capture a single image."""
        if not self.connected:
            return
        self.continuous_capture = False
        self.btn_continuous.setText("Continuous Capture Off")
        self.send_command("GET_FRAME")
        frame = self.receive_image()
        if frame is not None:
            self.show_image(frame)

    def toggle_continuous_capture(self):
        """Toggle continuous image capture."""
        if not self.connected:
            return
        self.continuous_capture = not self.continuous_capture
        self.btn_continuous.setText("Continuous Capture On" if self.continuous_capture else "Continuous Capture Off")

    def change_exposure(self, val):
        """Change the exposure time.

        Args:
            val (int): The new exposure time value.
        """
        self.exposure_time = val
        self.send_command(f"ADJUST_EXPOSURE:{self.exposure_time}:{self.target_brightness}")

    def change_brightness(self, val):
        """Change the target brightness.

        Args:
            val (int): The new target brightness value.
        """
        self.target_brightness = val
        self.send_command(f"ADJUST_BRIGHTNESS:{self.target_brightness}")

    def update_frame(self):
        """Update the image frame if continuous capture is enabled."""
        if self.continuous_capture and self.connected:
            self.send_command("GET_FRAME")
            frame = self.receive_image()
            if frame is not None:
                self.show_image(frame)

    def show_image(self, frame):
        """Display the image frame.

        Args:
            frame: The image frame to display.
        """
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self.preview_label.setPixmap(pix)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ClientGUI()
    window.show()
    sys.exit(app.exec_())