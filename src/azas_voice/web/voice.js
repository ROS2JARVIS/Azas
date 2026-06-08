const orb = document.querySelector("#voice-orb");
const waveform = document.querySelector("#waveform");
const ctx = waveform.getContext("2d");
const micButton = document.querySelector("#mic-button");
const statePill = document.querySelector("#state-pill");
const userText = document.querySelector("#user-text");
const azasText = document.querySelector("#azas-text");
const recipeId = document.querySelector("#recipe-id");
const intent = document.querySelector("#intent");
const confirmed = document.querySelector("#confirmed");
const testForm = document.querySelector("#test-form");
const testUtterance = document.querySelector("#test-utterance");

let analyser = null;
let timeData = null;
let micLevel = 0;
let micReady = false;
let currentUiState = "idle";
let recognition = null;
let recognitionActive = false;
let browserTranscript = "";
let browserTranscriptAt = 0;

function labelForState(state) {
  if (state === "speaking") return "Azas 응답 중";
  if (micLevel > 0.08) return "듣는 중";
  if (state === "error") return "오류";
  return "대기 중";
}

async function enableMicVisualizer() {
  const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
  const audioContext = new AudioContext();
  const source = audioContext.createMediaStreamSource(stream);
  analyser = audioContext.createAnalyser();
  analyser.fftSize = 256;
  analyser.smoothingTimeConstant = 0.74;
  timeData = new Uint8Array(analyser.fftSize);
  source.connect(analyser);
  micReady = true;
}

function enableBrowserSpeechRecognition() {
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRecognition) {
    micButton.textContent = "마이크 시각화 켜짐";
    azasText.textContent = "이 브라우저는 음성 인식을 지원하지 않습니다. 테스트 발화 입력창을 사용해주세요.";
    return;
  }

  recognition = new SpeechRecognition();
  recognition.lang = "ko-KR";
  recognition.continuous = true;
  recognition.interimResults = true;

  recognition.addEventListener("result", async (event) => {
    let interim = "";
    for (let index = event.resultIndex; index < event.results.length; index += 1) {
      const transcript = event.results[index][0].transcript.trim();
      if (!transcript) continue;
      if (event.results[index].isFinal) {
        browserTranscript = transcript;
        browserTranscriptAt = Date.now();
        userText.textContent = transcript;
        try {
          await postUtterance(transcript);
          await refreshState();
        } catch (error) {
          azasText.textContent = error.message || String(error);
        }
      } else {
        interim = transcript;
      }
    }

    if (interim) {
      browserTranscript = interim;
      browserTranscriptAt = Date.now();
      userText.textContent = `${interim} ...`;
    }
  });

  recognition.addEventListener("end", () => {
    if (recognitionActive) {
      try {
        recognition.start();
      } catch (error) {
        recognitionActive = false;
        micButton.textContent = "음성 인식 다시 켜기";
        micButton.disabled = false;
      }
    }
  });

  recognition.addEventListener("error", (event) => {
    if (event.error === "no-speech") return;
    azasText.textContent = `브라우저 음성 인식 오류: ${event.error}`;
  });

  recognition.start();
  recognitionActive = true;
  micButton.textContent = "음성 인식 중";
  micButton.disabled = true;
}

function drawWaveform() {
  const width = waveform.width;
  const height = waveform.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "rgba(255, 255, 255, 0.34)";
  ctx.fillRect(0, 0, width, height);

  if (analyser && timeData) {
    analyser.getByteTimeDomainData(timeData);
    let sum = 0;
    for (const value of timeData) {
      const normalized = (value - 128) / 128;
      sum += normalized * normalized;
    }
    micLevel = Math.min(1, Math.sqrt(sum / timeData.length) * 3.4);
  } else {
    micLevel = Math.max(0, micLevel * 0.94);
  }

  const bars = 36;
  const gap = 5;
  const barWidth = (width - gap * (bars - 1)) / bars;
  const styles = getComputedStyle(document.documentElement);
  ctx.fillStyle =
    currentUiState === "speaking"
      ? styles.getPropertyValue("--wave-speak").trim()
      : styles.getPropertyValue("--wave-listen").trim();

  for (let index = 0; index < bars; index += 1) {
    const phase = performance.now() / 180 + index * 0.52;
    const idle = (Math.sin(phase) + 1) * 0.18;
    const level = micReady ? micLevel : idle;
    const heightScale = Math.max(0.08, idle + level * (0.55 + (index % 5) * 0.06));
    const barHeight = height * Math.min(0.9, heightScale);
    const x = index * (barWidth + gap);
    const y = (height - barHeight) / 2;
    roundRect(ctx, x, y, barWidth, barHeight, 999);
    ctx.fill();
  }

  const speakLevel = currentUiState === "speaking" ? 1 : 0;
  orb.style.setProperty("--level", micLevel.toFixed(3));
  orb.style.setProperty("--speak", speakLevel);
  orb.classList.toggle("listening", micLevel > 0.08 && currentUiState !== "speaking");
  orb.classList.toggle("speaking", currentUiState === "speaking");
  statePill.textContent = labelForState(currentUiState);
  requestAnimationFrame(drawWaveform);
}

function roundRect(context, x, y, width, height, radius) {
  const r = Math.min(radius, width / 2, height / 2);
  context.beginPath();
  context.moveTo(x + r, y);
  context.arcTo(x + width, y, x + width, y + height, r);
  context.arcTo(x + width, y + height, x, y + height, r);
  context.arcTo(x, y + height, x, y, r);
  context.arcTo(x, y, x + width, y, r);
  context.closePath();
}

async function refreshState() {
  const response = await fetch("/api/state");
  const state = await response.json();
  const ui = state.ui_state || {};
  const decision = state.decision || {};
  const confirmedDecision = state.confirmed_decision || {};
  const recentBrowserSpeech = Date.now() - browserTranscriptAt < 1800;

  currentUiState = ui.state || "idle";
  userText.textContent = recentBrowserSpeech
    ? browserTranscript
    : state.last_stt || "아직 인식된 발화가 없습니다.";
  azasText.textContent = state.last_confirmation || ui.text || "말씀해주시면 주문을 도와드릴게요.";
  recipeId.textContent = decision.recipe_id || confirmedDecision.recipe_id || "-";
  intent.textContent = decision.intent || "대기";
  confirmed.textContent = confirmedDecision.confirmed ? "확정됨" : "대기";
}

async function postUtterance(text) {
  const response = await fetch("/api/utterance", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ text }),
  });
  const result = await response.json();
  if (!response.ok || !result.ok) {
    throw new Error(result.error || "발화를 전송하지 못했습니다.");
  }
}

micButton.addEventListener("click", async () => {
  try {
    await enableMicVisualizer();
    enableBrowserSpeechRecognition();
  } catch (error) {
    micButton.textContent = "마이크 권한 필요";
    azasText.textContent = error.message || String(error);
  }
});

testForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const text = testUtterance.value.trim();
  if (!text) return;
  try {
    await postUtterance(text);
    testUtterance.value = "";
    await refreshState();
  } catch (error) {
    azasText.textContent = error.message || String(error);
  }
});

drawWaveform();
refreshState().catch(() => {});
setInterval(() => {
  refreshState().catch(() => {});
}, 500);
