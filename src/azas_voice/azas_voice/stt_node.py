import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import speech_recognition as sr
except ImportError:  # pragma: no cover - exercised on systems without microphone deps
    sr = None


class SttNode(Node):
    """17차시 STT example adapted for Azas.

    Microphone audio is converted to Korean text and published to /stt_result.
    Downstream nodes can be tested without a microphone by publishing String
    messages directly to the same topic.
    """

    def __init__(self):
        super().__init__("stt_node")

        self.declare_parameter("language", "ko-KR")
        self.declare_parameter("device_index", -1)
        self.declare_parameter("energy_threshold", 300.0)
        self.declare_parameter("pause_threshold", 0.8)
        self.declare_parameter("phrase_time_limit", 5.0)
        self.declare_parameter("dynamic_energy", True)
        self.declare_parameter("ambient_duration", 1.0)
        self.declare_parameter("stt_topic", "/stt_result")

        if sr is None:
            raise RuntimeError(
                "speech_recognition is not installed. Install lecture dependencies: "
                "sudo apt install portaudio19-dev && pip install SpeechRecognition pyaudio"
            )

        self._lang = self.get_parameter("language").value
        self._device_idx = self.get_parameter("device_index").value
        self._phrase_lim = self.get_parameter("phrase_time_limit").value
        stt_topic = self.get_parameter("stt_topic").value

        self._pub = self.create_publisher(String, stt_topic, 10)
        self._log_devices()

        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = self.get_parameter("energy_threshold").value
        self._recognizer.pause_threshold = self.get_parameter("pause_threshold").value
        self._recognizer.dynamic_energy_threshold = self.get_parameter("dynamic_energy").value

        device = self._device_idx if self._device_idx >= 0 else None
        self._mic = sr.Microphone(device_index=device)

        ambient_duration = self.get_parameter("ambient_duration").value
        with self._mic as source:
            self.get_logger().info(f"주변 소음 측정 중 ({ambient_duration:.1f}s) ...")
            self._recognizer.adjust_for_ambient_noise(source, duration=ambient_duration)
            self.get_logger().info(
                f"energy_threshold={self._recognizer.energy_threshold:.1f}"
            )

        self._stop_listen = self._recognizer.listen_in_background(
            self._mic,
            self._on_audio,
            phrase_time_limit=self._phrase_lim,
        )
        self.get_logger().info(
            f"STT ready language={self._lang} device_index={self._device_idx} topic={stt_topic}"
        )

    def _log_devices(self):
        self.get_logger().info("=== microphone devices ===")
        for idx, name in enumerate(sr.Microphone.list_microphone_names()):
            mark = " <- selected" if idx == self._device_idx else ""
            self.get_logger().info(f"  [{idx}] {name}{mark}")

    def _on_audio(self, recognizer, audio):
        try:
            text = recognizer.recognize_google(audio, language=self._lang)
        except sr.UnknownValueError:
            self.get_logger().warn("STT could not understand audio")
            return
        except sr.RequestError as exc:
            self.get_logger().error(f"Google Web Speech request failed: {exc}")
            return

        msg = String()
        msg.data = text
        self._pub.publish(msg)
        self.get_logger().info(f"[STT] {text}")

    def destroy_node(self):
        if hasattr(self, "_stop_listen") and self._stop_listen is not None:
            self._stop_listen(wait_for_stop=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
