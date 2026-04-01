"""Voice input source — speech-to-text via Whisper.

Subscribes to /audio/raw (published by webcam_audio runnable), buffers
audio, detects speech with Silero VAD, transcribes with faster-whisper,
and sends recognized commands to the agent pipeline.

Background noise handling:
  1. Silero VAD filters out non-speech audio
  2. A wake word ("hey robot" by default) gates command mode — only
     speech following the wake word is sent as a task
  3. Low-confidence transcriptions are discarded
  4. Short utterances (< 2 words) are ignored

Requires:
  pip install faster-whisper silero-vad numpy

Run standalone:  python -m inputs.voice
Or let start.py auto-discover it.
"""

from __future__ import annotations

import collections
import os
import tempfile
import threading
import time
import wave
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int16MultiArray

from andr import BaseInputSource

# ── Configuration ────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
WAKE_WORDS = os.environ.get(
    "ANDR_WAKE_WORDS", "hey robot,okay robot,hey andr"
).lower().split(",")
WAKE_WORD_ENABLED = os.environ.get("ANDR_WAKE_WORD_ENABLED", "true").lower() == "true"

# After wake word, how long to keep listening for the command (seconds)
COMMAND_TIMEOUT_S = float(os.environ.get("ANDR_VOICE_COMMAND_TIMEOUT", "8.0"))

# Silence duration that marks end of an utterance (seconds)
SILENCE_THRESHOLD_S = float(os.environ.get("ANDR_VOICE_SILENCE_THRESHOLD", "1.5"))

# Minimum transcription confidence (0-1) to accept
MIN_CONFIDENCE = float(os.environ.get("ANDR_VOICE_MIN_CONFIDENCE", "0.4"))

# Whisper model size — "tiny", "base", "small" recommended for Jetson
WHISPER_MODEL = os.environ.get("ANDR_WHISPER_MODEL", "base")

# Device for whisper — "cuda" for Jetson GPU, "cpu" for fallback
WHISPER_DEVICE = os.environ.get("ANDR_WHISPER_DEVICE", "auto")

# Max audio buffer before forced transcription (seconds)
MAX_UTTERANCE_S = 15.0

# VAD parameters
VAD_THRESHOLD = 0.5  # Silero VAD speech probability threshold
# ─────────────────────────────────────────────────────────────────────────


class ListenState(Enum):
    IDLE = auto()         # Waiting for wake word (or always listening)
    LISTENING = auto()    # Actively capturing a command utterance


class VoiceInput(BaseInputSource):
    SOURCE_NAME = "voice"
    SOURCE_DESCRIPTION = "Microphone speech-to-text input via Whisper"

    def __init__(self):
        super().__init__()

        self._cb_group = ReentrantCallbackGroup()
        self._state = ListenState.IDLE if WAKE_WORD_ENABLED else ListenState.LISTENING

        # Audio buffer (ring buffer of int16 samples)
        self._audio_lock = threading.Lock()
        self._utterance_buffer: list[np.ndarray] = []
        self._vad_buffer: collections.deque = collections.deque(
            maxlen=int(SAMPLE_RATE * MAX_UTTERANCE_S / 512)
        )

        # Timing
        self._last_speech_time = 0.0
        self._command_start_time = 0.0

        # Subscribe to raw audio from webcam_audio node
        self.create_subscription(
            Int16MultiArray, "/audio/raw",
            self._on_audio, 10, callback_group=self._cb_group,
        )

        # Load models in background thread to not block startup
        self._whisper_model = None
        self._vad_model = None
        self._models_ready = threading.Event()
        self._model_thread = threading.Thread(
            target=self._load_models, daemon=True
        )
        self._model_thread.start()

        # Processing timer — runs every 100ms
        self._process_timer = self.create_timer(
            0.1, self._process_tick, callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"VoiceInput starting — wake_word={'enabled' if WAKE_WORD_ENABLED else 'disabled'} "
            f"words={WAKE_WORDS} model={WHISPER_MODEL}"
        )

    # ── Model loading ────────────────────────────────────────────────────

    def _load_models(self):
        try:
            # Load Silero VAD
            import torch
            self._vad_model, _utils = torch.hub.load(
                repo_or_dir="snakers4/silero-vad",
                model="silero_vad",
                trust_repo=True,
            )
            self.get_logger().info("Silero VAD loaded")
        except Exception as e:
            self.get_logger().error(f"Failed to load VAD: {e}")
            self._vad_model = None

        try:
            # Load faster-whisper
            from faster_whisper import WhisperModel

            device = WHISPER_DEVICE
            if device == "auto":
                try:
                    import torch
                    device = "cuda" if torch.cuda.is_available() else "cpu"
                except ImportError:
                    device = "cpu"

            compute_type = "float16" if device == "cuda" else "int8"
            self._whisper_model = WhisperModel(
                WHISPER_MODEL, device=device, compute_type=compute_type,
            )
            self.get_logger().info(
                f"Whisper loaded: model={WHISPER_MODEL} device={device} "
                f"compute={compute_type}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper: {e}")
            self._whisper_model = None

        self._models_ready.set()

    # ── Audio callback ───────────────────────────────────────────────────

    def _on_audio(self, msg: Int16MultiArray):
        if not self._models_ready.is_set():
            return

        samples = np.array(msg.data, dtype=np.int16)
        if len(samples) == 0:
            return

        # Run VAD on this chunk
        is_speech = self._check_vad(samples)

        with self._audio_lock:
            if is_speech:
                self._last_speech_time = time.monotonic()

                if self._state == ListenState.IDLE:
                    # Buffer speech for wake word detection
                    self._utterance_buffer.append(samples)
                    # Keep buffer bounded in idle mode
                    total = sum(len(c) for c in self._utterance_buffer)
                    if total > SAMPLE_RATE * 5:
                        self._utterance_buffer.pop(0)

                elif self._state == ListenState.LISTENING:
                    self._utterance_buffer.append(samples)

            elif self._state == ListenState.LISTENING:
                # Keep buffering during silence gaps within an utterance
                self._utterance_buffer.append(samples)

    def _check_vad(self, samples: np.ndarray) -> bool:
        if self._vad_model is None:
            # Fallback: energy-based VAD
            energy = np.sqrt(np.mean(samples.astype(np.float32) ** 2))
            return energy > 500.0

        try:
            import torch
            # Silero VAD expects float32 in [-1, 1], 512 sample chunks
            audio_f32 = samples.astype(np.float32) / 32768.0
            # Process in 512-sample chunks, return True if any chunk has speech
            for i in range(0, len(audio_f32), 512):
                chunk = audio_f32[i:i + 512]
                if len(chunk) < 512:
                    chunk = np.pad(chunk, (0, 512 - len(chunk)))
                tensor = torch.from_numpy(chunk)
                prob = self._vad_model(tensor, SAMPLE_RATE).item()
                if prob > VAD_THRESHOLD:
                    return True
        except Exception:
            pass
        return False

    # ── Processing tick ──────────────────────────────────────────────────

    def _process_tick(self):
        if not self._models_ready.is_set():
            return

        now = time.monotonic()

        with self._audio_lock:
            has_audio = len(self._utterance_buffer) > 0
            silence_duration = now - self._last_speech_time if self._last_speech_time > 0 else float("inf")
            total_samples = sum(len(c) for c in self._utterance_buffer)
            total_duration = total_samples / SAMPLE_RATE

        if self._state == ListenState.IDLE and WAKE_WORD_ENABLED:
            # Check if buffered audio contains a wake word
            if has_audio and silence_duration > SILENCE_THRESHOLD_S:
                self._check_wake_word()

        elif self._state == ListenState.LISTENING:
            # Check if utterance is complete (silence after speech)
            timed_out = (now - self._command_start_time) > COMMAND_TIMEOUT_S
            silence_ended = silence_duration > SILENCE_THRESHOLD_S and total_duration > 0.3
            too_long = total_duration > MAX_UTTERANCE_S

            if (silence_ended and has_audio) or timed_out or too_long:
                self._process_command()

    def _check_wake_word(self):
        """Transcribe buffered audio and check for wake word."""
        text = self._transcribe_buffer(consume=True)
        if not text:
            return

        text_lower = text.lower().strip()
        self.get_logger().debug(f"[VOICE] Wake check: \"{text_lower}\"")

        for wake in WAKE_WORDS:
            wake = wake.strip()
            if wake in text_lower:
                # Extract any command that came after the wake word
                idx = text_lower.index(wake) + len(wake)
                remainder = text[idx:].strip().lstrip(".,!? ")

                self.get_logger().info(
                    f"[VOICE] Wake word detected: \"{wake}\""
                )

                if remainder and len(remainder.split()) >= 2:
                    # Command was in the same utterance as wake word
                    self._send_voice_command(remainder)
                else:
                    # Switch to listening mode for the follow-up command
                    self._state = ListenState.LISTENING
                    self._command_start_time = time.monotonic()
                    self._last_speech_time = time.monotonic()
                    self.get_logger().info("[VOICE] Listening for command...")
                return

    def _process_command(self):
        """Transcribe the command buffer and send to agent."""
        text = self._transcribe_buffer(consume=True)

        # Reset to idle
        self._state = ListenState.IDLE if WAKE_WORD_ENABLED else ListenState.LISTENING

        if not text:
            self.get_logger().debug("[VOICE] Empty transcription, ignoring")
            return

        # Filter out very short or nonsense utterances
        words = text.strip().split()
        if len(words) < 2:
            self.get_logger().debug(f"[VOICE] Too short, ignoring: \"{text}\"")
            return

        self._send_voice_command(text)

    def _send_voice_command(self, text: str):
        """Send transcribed text as a task to the agent."""
        if self.is_busy:
            self.get_logger().info(
                f"[VOICE] Agent busy, dropping: \"{text[:80]}\""
            )
            return

        self.get_logger().info(f"[VOICE] Sending command: \"{text}\"")
        self.send_task(
            prompt=text,
            context="source:voice_command",
        )

    # ── Transcription ────────────────────────────────────────────────────

    def _transcribe_buffer(self, consume: bool = True) -> str | None:
        """Transcribe audio in the utterance buffer using Whisper."""
        if self._whisper_model is None:
            with self._audio_lock:
                if consume:
                    self._utterance_buffer.clear()
            return None

        with self._audio_lock:
            if not self._utterance_buffer:
                return None
            audio = np.concatenate(self._utterance_buffer)
            if consume:
                self._utterance_buffer.clear()

        if len(audio) < SAMPLE_RATE * 0.3:
            return None

        # Convert to float32 for whisper
        audio_f32 = audio.astype(np.float32) / 32768.0

        try:
            segments, info = self._whisper_model.transcribe(
                audio_f32,
                beam_size=3,
                language="en",
                vad_filter=True,
                vad_parameters=dict(
                    min_silence_duration_ms=500,
                    speech_pad_ms=200,
                ),
            )

            texts = []
            total_prob = 0.0
            count = 0
            for seg in segments:
                texts.append(seg.text)
                total_prob += seg.avg_log_prob
                count += 1

            if count == 0:
                return None

            avg_prob = total_prob / count
            full_text = " ".join(texts).strip()

            # Filter low confidence (avg_log_prob is negative, closer to 0 = better)
            # Typical threshold: -1.0 is decent, -2.0 is poor
            if avg_prob < -1.5:
                self.get_logger().debug(
                    f"[VOICE] Low confidence ({avg_prob:.2f}), "
                    f"dropping: \"{full_text[:60]}\""
                )
                return None

            # Filter Whisper hallucinations (common patterns)
            hallucinations = [
                "thank you", "thanks for watching", "subscribe",
                "you", "bye", "the end",
            ]
            text_lower = full_text.lower().strip().rstrip(".")
            if text_lower in hallucinations:
                self.get_logger().debug(
                    f"[VOICE] Hallucination filtered: \"{full_text}\""
                )
                return None

            return full_text

        except Exception as e:
            self.get_logger().error(f"[VOICE] Transcription error: {e}")
            return None

    # ── BaseInputSource hooks ────────────────────────────────────────────

    def on_task_accepted(self, prompt: str):
        self.get_logger().info("[VOICE] Task accepted by agent")

    def on_task_completed(self, prompt: str, success: bool, summary: str):
        self.get_logger().info(
            f"[VOICE] Task {'succeeded' if success else 'failed'}: "
            f"{summary[:100]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInput()
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
