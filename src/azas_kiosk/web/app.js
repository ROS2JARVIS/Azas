const state = {
  menus: [],
  selectedRecipeId: null,
};

const menuGrid = document.querySelector("#menu-grid");
const confirmationText = document.querySelector("#confirmation-text");
const lastCommand = document.querySelector("#last-command");
const cocktailStatus = document.querySelector("#cocktail-status");
const voiceState = document.querySelector("#voice-state");

const colorLabels = {
  red: "Red",
  yellow: "Yellow",
  green: "Green",
  blue: "Blue",
};

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok || !data.ok) {
    throw new Error(data.error || "요청을 처리하지 못했습니다.");
  }
  return data;
}

function renderMenus(menus) {
  if (!menus.length) return;
  menuGrid.innerHTML = "";
  for (const menu of menus) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `menu-card tone-${menu.color}`;
    button.dataset.recipeId = menu.recipe_id;
    button.innerHTML = `
      <span class="menu-tone">${colorLabels[menu.color] || menu.color}</span>
      <strong>${menu.name}</strong>
      <span class="menu-role">${menu.role}</span>
      <span class="menu-description">${menu.description}</span>
    `;
    button.addEventListener("click", async () => {
      await selectMenu(menu.recipe_id);
    });
    menuGrid.appendChild(button);
  }
}

async function selectMenu(recipeId) {
  state.selectedRecipeId = recipeId;
  setSelectedCard(recipeId);
  await postJson("/api/order", { recipe_id: recipeId });
  await refreshState();
}

function setSelectedCard(recipeId) {
  for (const card of menuGrid.querySelectorAll(".menu-card")) {
    card.classList.toggle("selected", card.dataset.recipeId === recipeId);
  }
}

async function refreshState() {
  const response = await fetch("/api/state");
  const payload = await response.json();
  if (Array.isArray(payload.menus) && payload.menus.length && !state.menus.length) {
    state.menus = payload.menus;
    renderMenus(state.menus);
  }

  const ui = payload.ui_state || {};
  const status = payload.cocktail_status || {};
  const prompt = payload.last_confirmation || ui.text || "주문을 기다리고 있습니다.";

  confirmationText.textContent = prompt;
  lastCommand.textContent = payload.last_command || "-";
  voiceState.textContent = ui.state === "speaking" ? "안내 중" : "대기 중";
  cocktailStatus.textContent = status.status || "대기";
}

function showError(error) {
  confirmationText.textContent = error.message || String(error);
}

document.querySelector("#recommend-button").addEventListener("click", async () => {
  try {
    state.selectedRecipeId = null;
    setSelectedCard(null);
    await postJson("/api/recommend");
    await refreshState();
  } catch (error) {
    showError(error);
  }
});

document.querySelector("#cancel-button").addEventListener("click", async () => {
  try {
    state.selectedRecipeId = null;
    setSelectedCard(null);
    await postJson("/api/cancel");
    await refreshState();
  } catch (error) {
    showError(error);
  }
});

document.querySelector("#confirm-button").addEventListener("click", async () => {
  try {
    await postJson("/api/confirm");
    await refreshState();
  } catch (error) {
    showError(error);
  }
});

refreshState().catch(showError);
setInterval(() => {
  refreshState().catch(showError);
}, 1000);
