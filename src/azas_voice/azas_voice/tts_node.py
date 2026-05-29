from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - allows helper tests without sourced ROS
    rclpy = None
    Node = object
    String = None

try:
    from gtts import gTTS
    import pygame

    _GTTS_PYGAME_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on optional host packages
    _GTTS_PYGAME_AVAILABLE = False


def build_ui_state(state: str, text: str = "", emotion: str = "neutral") -> str:
    return json.dumps(
        {
            "state": state,
            "emotion": emotion,
            "text": text,
        },
        ensure_ascii=False,
    )


class _SpeechEngine:
    def speak(self, text: str) -> bool:
        raise NotImplementedError

    def shutdown(self) -> None:
        pass


class _SilentSpeechEngine(_SpeechEngine):
    def speak(self, text: str) -> bool:
        return False


class _GttsPygameSpeechEngine(_SpeechEngine):
    def __init__(self, language: str, speech_rate: float):
        if not _GTTS_PYGAME_AVAILABLE:
            raise RuntimeError("gTTS/pygame are not available")
        self._language = language
        self._speech_rate = max(0.5, min(speech_rate, 2.0))
        self._cache: dict[tuple[str, float], str] = {}
        self._ffmpeg = shutil.which("ffmpeg")
        pygame.mixer.init()

    def speak(self, text: str) -> bool:
        cache_key = (text, self._speech_rate)
        path = self._cache.get(cache_key)
        if not path or not os.path.exists(path):
            fd, raw_path = tempfile.mkstemp(suffix=".mp3", prefix="azas_tts_raw_")
            os.close(fd)
            gTTS(text=text, lang=self._language).save(raw_path)
            path = self._speed_adjusted_path(raw_path)
            self._cache[cache_key] = path

        pygame.mixer.music.load(path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.05)
        return True

    def _speed_adjusted_path(self, raw_path: str) -> str:
        if self._speech_rate == 1.0 or self._ffmpeg is None:
            return raw_path

        fd, adjusted_path = tempfile.mkstemp(suffix=".mp3", prefix="azas_tts_fast_")
        os.close(fd)
        subprocess.run(
            [
                self._ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-i",
                raw_path,
                "-filter:a",
                f"atempo={self._speech_rate:.2f}",
                adjusted_path,
            ],
            check=True,
        )
        return adjusted_path

    def shutdown(self) -> None:
        if _GTTS_PYGAME_AVAILABLE:
            pygame.mixer.music.stop()


class TtsNode(Node):
    """Speak confirmation text and publish avatar/UI conversation state."""

    def __init__(self):
        if rclpy is None or String is None:
            raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
        super().__init__("tts_node")

        self.declare_parameter("confirmation_topic", "/azas/voice/confirmation")
        self.declare_parameter("ui_state_topic", "/azas/voice/ui_state")
        self.declare_parameter("language", "ko")
        self.declare_parameter("enable_audio", True)
        self.declare_parameter("speech_rate", 1.25)
        self.declare_parameter("startup_prompt", "주문하시겠어요?")

        confirmation_topic = str(self.get_parameter("confirmation_topic").value)
        ui_state_topic = str(self.get_parameter("ui_state_topic").value)
        language = str(self.get_parameter("language").value)
        enable_audio = bool(self.get_parameter("enable_audio").value)
        speech_rate = float(self.get_parameter("speech_rate").value)
        startup_prompt = str(self.get_parameter("startup_prompt").value).strip()

        self._ui_state_pub = self.create_publisher(String, ui_state_topic, 10)
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._engine = self._build_engine(language, enable_audio, speech_rate)
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        self.create_subscription(String, confirmation_topic, self._on_confirmation, 10)
        self._publish_state("idle")
        if startup_prompt:
            self._queue.put(startup_prompt)
        self.get_logger().info(
            "TTS ready: "
            f"confirmation_topic={confirmation_topic} ui_state_topic={ui_state_topic} "
            f"audio={'on' if not isinstance(self._engine, _SilentSpeechEngine) else 'off'} "
            f"speech_rate={speech_rate:.2f}"
        )

    def _build_engine(self, language: str, enable_audio: bool, speech_rate: float) -> _SpeechEngine:
        if not enable_audio:
            return _SilentSpeechEngine()
        try:
            return _GttsPygameSpeechEngine(language, speech_rate)
        except Exception as exc:
            self.get_logger().warn(f"TTS audio disabled: {exc}")
            return _SilentSpeechEngine()

    def _on_confirmation(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self._queue.put(text)

    def _run_worker(self) -> None:
        while rclpy is not None and rclpy.ok():
            text = self._queue.get()
            if text is None:
                break

            self._publish_state("speaking", text=text, emotion="friendly")
            try:
                spoke = self._engine.speak(text)
                if not spoke:
                    self.get_logger().info(f"[TTS muted] {text}")
            except Exception as exc:
                self.get_logger().error(f"TTS playback failed: {exc}")
                self._publish_state("error", text=text, emotion="concerned")
            finally:
                self._publish_state("idle")

    def _publish_state(self, state: str, text: str = "", emotion: str = "neutral") -> None:
        msg = String()
        msg.data = build_ui_state(state, text, emotion)
        self._ui_state_pub.publish(msg)

    def destroy_node(self):
        self._queue.put(None)
        self._worker.join(timeout=1.0)
        self._engine.shutdown()
        super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not available. Source the ROS environment first.")
    rclpy.init(args=args)
    node = TtsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
