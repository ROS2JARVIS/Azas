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
const menuBadge = document.querySelector("#menu-badge");
const menuEmpty = document.querySelector("#menu-empty");
const menuCard = document.querySelector("#menu-card");
const menuName = document.querySelector("#menu-name");
const menuDesc = document.querySelector("#menu-desc");
const glassLayers = document.querySelector("#glass-layers");
const ingredientChips = document.querySelector("#ingredient-chips");
const pipelineSteps = [...document.querySelectorAll("#pipeline-steps li")];
const catalogCount = document.querySelector("#catalog-count");
const catalogSummary = document.querySelector("#catalog-summary");
const catalogList = document.querySelector("#catalog-list");
const statSweetness = document.querySelector("#stat-sweetness");
const statAcidity = document.querySelector("#stat-acidity");
const statStrength = document.querySelector("#stat-strength");
const robotScene = document.querySelector("#robot-scene");
const robotStatusText = document.querySelector("#robot-status-text");

const INGREDIENTS = {
  red: { label: "주스", color: "#ff7e96" },
  yellow: { label: "시럽", color: "#ffd464" },
  green: { label: "리큐르", color: "#69d98a" },
  blue: { label: "럼", color: "#6db4ff" },
};

const RECIPE_NAMES = {
  recipe_01: "레드 메뉴",
  recipe_02: "옐로우 메뉴",
  recipe_03: "그린 메뉴",
  recipe_04: "블루 메뉴",
  custom_preference_mix: "나만의 추천 믹스",
  custom_color_selection: "커스텀 선택",
};

const RECIPE_DESCRIPTIONS = {
  recipe_01: "주스 중심이라 과일감이 선명하고 가볍게 마시기 좋아요.",
  recipe_02: "시럽 중심이라 달콤하고 부드러운 느낌이 강해요.",
  recipe_03: "리큐르 중심이라 향이 선명하고 깔끔한 여운이 있어요.",
  recipe_04: "럼 중심이라 칵테일다운 존재감과 깊이가 있어요.",
  custom_preference_mix: "말씀하신 취향에 맞춰 재료 비율을 조합했어요.",
  custom_color_selection: "고르신 색 재료 그대로 만들어드려요.",
};

let catalogSignature = "";

// 라우터 단계명(/azas/voice/pipeline_status의 stage) -> 진행 스텝 인덱스
const STAGE_TO_STEP = {
  "디스펜서 색 스캔": 0,
  "컵 픽업 (세워진 컵)": 1,
  "컵 픽업 (쓰러진 컵)": 1,
  "디스펜서 레시피 진행": 2,
  "중단 지점 복구": 2,
  "뚜껑 체결 / 쉐이킹": 3,
  "완료": 4,
};

// 잔 내부(clip-path 기준): y 30~167, x 33~127
const GLASS_TOP = 30;
const GLASS_BOTTOM = 167;
const FILL_RATIO = 0.86;

function amountsFromDecision(decision) {
  const amounts = {};
  const payload = decision.dispenser_amounts || {};
  for (const color of Object.keys(INGREDIENTS)) {
    const value = Number(payload[color] || 0);
    if (value > 0) amounts[color] = Math.min(value, 3);
  }
  if (Object.keys(amounts).length === 0 && Array.isArray(decision.dispenser_ids)) {
    for (const color of decision.dispenser_ids) {
      if (INGREDIENTS[color]) amounts[color] = 1;
    }
  }
  return amounts;
}

function recipeCatalog(state) {
  const recipes = state.catalog && Array.isArray(state.catalog.recipes) ? state.catalog.recipes : [];
  return recipes;
}

function recipeInfo(state, recipeKey) {
  return recipeCatalog(state).find((recipe) => recipe.recipe_id === recipeKey) || null;
}

function amountsFromRecipeInfo(info) {
  const amounts = {};
  const payload = (info && info.dispenser_amounts) || {};
  for (const color of Object.keys(INGREDIENTS)) {
    const value = Number(payload[color] || 0);
    if (value > 0) amounts[color] = Math.min(value, 3);
  }
  if (Object.keys(amounts).length === 0 && info && Array.isArray(info.dispenser_ids)) {
    for (const color of info.dispenser_ids) {
      if (INGREDIENTS[color]) amounts[color] = 1;
    }
  }
  return amounts;
}

function selectedDecision(state) {
  const decision = state.decision || {};
  const confirmedDecision = state.confirmed_decision || {};
  if (confirmedDecision.intent === "make_cocktail") return confirmedDecision;
  if (decision.intent === "make_cocktail") return decision;
  return null;
}

function selectedRecipeKey(state) {
  const shown = selectedDecision(state);
  return shown ? String(shown.recipe_id || "") : "";
}

function renderGlass(amounts) {
  const total = Object.values(amounts).reduce((sum, value) => sum + value, 0);
  glassLayers.replaceChildren();
  if (total <= 0) return;
  const innerHeight = (GLASS_BOTTOM - GLASS_TOP) * FILL_RATIO;
  let bottom = GLASS_BOTTOM;
  for (const color of ["blue", "green", "yellow", "red"]) {
    const value = amounts[color];
    if (!value) continue;
    const height = (value / total) * innerHeight;
    const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    rect.setAttribute("class", "glass-layer");
    rect.setAttribute("x", "20");
    rect.setAttribute("width", "120");
    rect.setAttribute("y", String(bottom - height));
    rect.setAttribute("height", String(height + 1));
    rect.setAttribute("fill", INGREDIENTS[color].color);
    glassLayers.appendChild(rect);
    bottom -= height;
  }
}

function renderStats(info) {
  const fields = [
    [statSweetness, info && info.sweetness],
    [statAcidity, info && info.acidity],
    [statStrength, info && info.strength],
  ];
  for (const [node, value] of fields) {
    node.textContent = value === null || value === undefined || value === "" ? "-" : `${value}/5`;
  }
}

function renderChips(amounts) {
  ingredientChips.replaceChildren();
  for (const color of ["red", "yellow", "green", "blue"]) {
    const value = amounts[color];
    if (!value) continue;
    const item = document.createElement("li");
    const swatch = document.createElement("span");
    swatch.className = "swatch";
    swatch.style.background = INGREDIENTS[color].color;
    item.append(swatch, `${INGREDIENTS[color].label} ×${value}`);
    ingredientChips.appendChild(item);
  }
}

function setBadge(kind, text) {
  menuBadge.className = `menu-badge ${kind}`;
  menuBadge.textContent = text;
}

function renderSteps(pipeline) {
  const status = pipeline.status || "";
  let activeIndex = -1;
  if (status === "running" && pipeline.stage in STAGE_TO_STEP) {
    activeIndex = STAGE_TO_STEP[pipeline.stage];
  } else if (status === "starting") {
    activeIndex = 0;
  } else if (status === "completed") {
    activeIndex = pipelineSteps.length;
  }
  pipelineSteps.forEach((step, index) => {
    step.classList.toggle("done", activeIndex > index);
    step.classList.toggle("active", activeIndex === index);
  });
  return activeIndex;
}

function renderRobot(activeIndex, pipeline, hasMenu) {
  if (!hasMenu) {
    robotScene.dataset.step = "idle";
    robotStatusText.textContent = "주문 대기";
    return;
  }
  const status = pipeline.status || "";
  if (status === "failed") {
    robotScene.dataset.step = "idle";
    robotStatusText.textContent = "제조 중단";
    return;
  }
  if (status === "completed" || activeIndex >= pipelineSteps.length) {
    robotScene.dataset.step = "done";
    robotStatusText.textContent = "완료";
    return;
  }
  const stepNames = ["scan", "pick", "dispense", "shake", "done"];
  const statusText = ["디스펜서 색 스캔", "컵 픽업", "디스펜서 토출", "뚜껑 체결 / 쉐이킹", "완료"];
  const index = activeIndex >= 0 ? activeIndex : 0;
  robotScene.dataset.step = stepNames[Math.min(index, stepNames.length - 1)];
  robotStatusText.textContent = pipeline.stage || statusText[Math.min(index, statusText.length - 1)];
}

function catalogArt(amounts) {
  const wrapper = document.createElement("span");
  wrapper.className = "catalog-art";
  const colors = Object.entries(amounts).filter(([, value]) => value > 0);
  if (!colors.length) {
    const empty = document.createElement("span");
    empty.className = "catalog-layer";
    empty.style.background = "rgba(110, 129, 121, 0.25)";
    wrapper.appendChild(empty);
    return wrapper;
  }
  for (const [color, value] of colors) {
    const layer = document.createElement("span");
    layer.className = "catalog-layer";
    layer.style.background = INGREDIENTS[color].color;
    layer.style.flexGrow = String(value);
    wrapper.appendChild(layer);
  }
  return wrapper;
}

function renderCatalog(state) {
  const recipes = recipeCatalog(state);
  const selectedKey = selectedRecipeKey(state);
  catalogCount.textContent = `메뉴 ${recipes.length}개`;
  catalogSummary.textContent = recipes.length ? "클릭해서 주문 입력" : "YAML 카탈로그 대기";
  const signature = JSON.stringify(recipes.map((recipe) => [
    recipe.recipe_id,
    recipe.name,
    recipe.description,
    recipe.dispenser_amounts,
  ]));
  if (signature !== catalogSignature) {
    catalogSignature = signature;
    catalogList.replaceChildren();
    for (const recipe of recipes) {
      const amounts = amountsFromRecipeInfo(recipe);
      const button = document.createElement("button");
      button.type = "button";
      button.className = "catalog-item";
      button.dataset.recipeId = recipe.recipe_id;
      button.setAttribute("aria-label", `${recipe.name} 주문`);
      button.appendChild(catalogArt(amounts));

      const copy = document.createElement("span");
      copy.className = "catalog-copy";
      const name = document.createElement("strong");
      name.textContent = recipe.name || recipe.recipe_id;
      const desc = document.createElement("span");
      desc.textContent = recipe.description || "카탈로그 메뉴";
      copy.append(name, desc);
      button.appendChild(copy);
      button.addEventListener("click", async () => {
        try {
          await postUtterance(`${recipe.name || recipe.recipe_id} 만들어줘`);
          await refreshState();
        } catch (error) {
          azasText.textContent = error.message || String(error);
        }
      });
      catalogList.appendChild(button);
    }
  }
  for (const item of catalogList.querySelectorAll(".catalog-item")) {
    item.classList.toggle("selected", item.dataset.recipeId === selectedKey);
  }
}

function renderMenu(state) {
  renderCatalog(state);
  const confirmedDecision = state.confirmed_decision || {};
  const pipeline = state.pipeline_status || {};
  const shown = selectedDecision(state);

  if (!shown) {
    menuCard.hidden = true;
    menuEmpty.hidden = false;
    renderRobot(-1, pipeline, false);
    setBadge("idle", "대기 중");
    return;
  }

  const recipeKey = String(shown.recipe_id || "");
  const info = recipeInfo(state, recipeKey);
  const amounts = Object.keys(amountsFromDecision(shown)).length
    ? amountsFromDecision(shown)
    : amountsFromRecipeInfo(info);
  if (Object.keys(amounts).length === 0) {
    menuCard.hidden = true;
    menuEmpty.hidden = false;
    renderRobot(-1, pipeline, false);
    setBadge("idle", "대기 중");
    return;
  }

  menuEmpty.hidden = true;
  menuCard.hidden = false;
  menuName.textContent = (info && info.name) || RECIPE_NAMES[recipeKey] || "커스텀 칵테일";
  menuDesc.textContent =
    (info && info.description) || RECIPE_DESCRIPTIONS[recipeKey] || "주문하신 조합으로 준비할게요.";
  renderStats(info);
  renderGlass(amounts);
  renderChips(amounts);
  const activeIndex = renderSteps(pipeline);
  renderRobot(activeIndex, pipeline, true);

  const pipelineStatus = pipeline.status || "";
  if (pipelineStatus === "failed") {
    setBadge("failed", "제조 실패");
  } else if (pipelineStatus === "completed") {
    setBadge("done", "완성! 맛있게 드세요");
  } else if (pipelineStatus === "running" || pipelineStatus === "starting") {
    setBadge("making", pipeline.stage ? `제조 중 · ${pipeline.stage}` : "제조 중");
  } else if (pipelineStatus === "dry_run") {
    setBadge("making", "리허설 (dry run)");
  } else if (confirmedDecision.confirmed) {
    setBadge("confirmed", "주문 확정");
  } else {
    setBadge("recommended", "추천 메뉴 · \"응\" 하시면 시작해요");
  }
}

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
  renderMenu(state);
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
