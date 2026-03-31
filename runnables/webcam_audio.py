#!/usr/bin/env python3
"""
webcam_audio.py — Webcam + Microphone capture node
====================================================
Auto-discovered and launched by start.py as a standalone process.

Publishes:
  /camera/image_raw    sensor_msgs/Image       (BGR8, 640x480 default)
  /audio/raw           std_msgs/Int16MultiArray (16-bit PCM, mono, 16 kHz)

These topics are designed to be consumed by downstream nodes such as:
  - A VLM (Vision-Language Model) input source that periodically takes a
    frame from /camera/image_raw, describes the scene, and sends a task
    to the agent via BaseInputSource.send_task().
  - An STT (Speech-to-Text) input source that buffers audio from /audio/raw,
    transcribes speech, and sends the transcript as a task to the agent.

Configuration constants below can be adjusted for your hardware.

Requires:
  pip install opencv-python sounddevice numpy
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

# ── Configuration ─────────────────────────────────────────────────────────
CAMERA_DEVICE = 0              # /dev/video0 — change for other cameras
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 10                # publish rate for images (Hz)
CAMERA_FRAME_ID = "camera_link"

AUDIO_SAMPLE_RATE = 16000      # 16 kHz — standard for most STT models
AUDIO_CHANNELS = 1             # mono
AUDIO_CHUNK_DURATION_S = 0.1   # publish audio every 100ms
AUDIO_DEVICE = None            # None = system default mic
# ──────────────────────────────────────────────────────────────────────────


class WebcamAudioNode(Node):
    """Captures webcam frames and microphone audio, publishes to ROS topics."""

    def __init__(self):
        super().__init__("webcam_audio_node")

        # ── Publishers ────────────────────────────────────────────────
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._audio_pub = self.create_publisher(Int16MultiArray, "/audio/raw", 10)

        # ── Webcam setup ──────────────────────────────────────────────
        self._cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self._cap.isOpened():
            self.get_logger().error(
                f"Cannot open camera device {CAMERA_DEVICE}. "
                "Image publishing disabled."
            )
            self._cap = None
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f"Camera opened: device={CAMERA_DEVICE} "
                f"resolution={actual_w}x{actual_h} publish_rate={CAMERA_FPS}Hz"
            )

        # ── Audio setup ───────────────────────────────────────────────
        self._audio_stream = None
        try:
            import sounddevice as sd
            self._sd = sd

            chunk_samples = int(AUDIO_SAMPLE_RATE * AUDIO_CHUNK_DURATION_S)
            self._audio_stream = sd.InputStream(
                samplerate=AUDIO_SAMPLE_RATE,
                channels=AUDIO_CHANNELS,
                dtype="int16",
                blocksize=chunk_samples,
                device=AUDIO_DEVICE,
                callback=self._audio_callback,
            )
            self._audio_stream.start()
            self.get_logger().info(
                f"Microphone opened: rate={AUDIO_SAMPLE_RATE}Hz "
                f"channels={AUDIO_CHANNELS} chunk={AUDIO_CHUNK_DURATION_S}s"
            )
        except Exception as e:
            self.get_logger().error(
                f"Cannot open microphone: {e}. Audio publishing disabled."
            )
            self._sd = None

        # ── Camera timer ──────────────────────────────────────────────
        if self._cap is not None:
            self._camera_timer = self.create_timer(
                1.0 / CAMERA_FPS, self._capture_and_publish_image
            )

        self.get_logger().info("WebcamAudioNode ready")

    # ── Image capture (timer callback) ────────────────────────────────

    def _capture_and_publish_image(self):
        ret, frame = self._cap.read()
        if not ret:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_FRAME_ID
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3  # 3 bytes per pixel (BGR)
        msg.data = frame.tobytes()

        self._image_pub.publish(msg)

    # ── Audio capture (sounddevice callback, runs on audio thread) ────

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")

        samples = indata[:, 0].astype(np.int16)

        msg = Int16MultiArray()
        dim = MultiArrayDimension()
        dim.label = "samples"
        dim.size = len(samples)
        dim.stride = len(samples)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = samples.tolist()

        self._audio_pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
            self.get_logger().info("Camera released")
        if self._audio_stream is not None:
            self._audio_stream.stop()
            self._audio_stream.close()
            self.get_logger().info("Microphone released")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamAudioNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
