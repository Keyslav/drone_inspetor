(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const C = window.DroneCore;
  const state = {
    snapshot: null,
    token: "",
    connected: false,
    lastSeen: 0,
    pending: false,
    suspended: false,
    generation: 0,
    missions: {},
    models: [],
    logs: [],
    eventKeys: new Set(),
    clearedKeys: new Set(),
    controllers: new Set(),
    nonce: "",
  };
  let stateBusy = false,
    confirmAction = null,
    map,
    tileLayer,
    marker,
    routeLayer,
    pointsLayer;
  let fitted = false;
  let autoConnect = false;
  let copilot = null;
  let video = null;
  const commandButtons = {
    "mission-start": "mission.start",
    "mission-cancel": "mission.cancel",
    "models-apply": "cv.models",
  };
  function notice(message, error = false) {
    $("notice").textContent = message;
    $("notice").classList.toggle("error", error);
  }
  function log(message, level = "info", time = new Date().toISOString()) {
    state.logs.unshift({ message, level, time });
    state.logs = state.logs.slice(0, 120);
    renderLogs();
  }
  function renderLogs() {
    $("events").textContent =
      state.logs
        .map((item) => {
          const date = new Date(item.time);
          const time = Number.isNaN(date.getTime())
            ? String(item.time || "")
            : date.toLocaleTimeString("pt-BR");
          return `${time}  ${String(item.level || "info").toUpperCase()}\n${item.message || ""}`;
        })
        .join("\n\n") || "Nenhum evento recebido.";
  }
  function enabled(command) {
    return (
      C.canWrite({
        connected: state.connected,
        enabled: state.snapshot?.commands_enabled,
        lastSeen: state.lastSeen,
        now: performance.now(),
        suspended: state.suspended || document.hidden,
        pending: state.pending,
      }) &&
      Boolean(state.nonce) &&
      (state.snapshot?.capabilities || []).includes(command) &&
      C.canCommand(state.snapshot, command)
    );
  }
  function syncButtons() {
    for (const [id, command] of Object.entries(commandButtons))
      $(id).disabled =
        !enabled(command) ||
        (id === "mission-start" && !$("mission-select").value) ||
        (id === "models-apply" &&
          (!$("object-model").value || !$("anomaly-model").value));
    document.querySelectorAll("[data-command]").forEach((button) => {
      button.disabled = !enabled(button.dataset.command);
    });
    $("confirm-send").disabled =
      !confirmAction || !enabled(confirmAction.command);
    copilot?.update();
  }
  async function request(path, options = {}) {
    const { authToken = state.token, ...fetchOptions } = options;
    const controller = new AbortController();
    state.controllers.add(controller);
    const timer = setTimeout(
      () => controller.abort(),
      options.method ? 12000 : 4000,
    );
    try {
      const headers = {
        Accept: "application/json",
        ...(authToken ? { Authorization: `Bearer ${authToken}` } : {}),
        ...(options.body ? { "Content-Type": "application/json" } : {}),
      };
      const response = await fetch(path, {
        ...fetchOptions,
        headers,
        cache: "no-store",
        credentials: "omit",
        redirect: "error",
        signal: controller.signal,
      });
      if (!response.ok) {
        const detail = await response.json().catch(() => ({}));
        const error = new Error(
          detail.error || `Gateway respondeu HTTP ${response.status}.`,
        );
        error.status = response.status;
        throw error;
      }
      return options.image ? response.blob() : response.json();
    } finally {
      clearTimeout(timer);
      state.controllers.delete(controller);
    }
  }
  function abortRequests() {
    for (const controller of state.controllers) controller.abort();
    state.controllers.clear();
  }
  function hideFrames() {
    video?.stop();
  }
  function disconnect(message = "Desconectado. Comandos indisponíveis.") {
    hideFrames();
    autoConnect = false;
    state.generation++;
    state.connected = false;
    state.token = "";
    state.snapshot = null;
    state.nonce = "";
    state.pending = false;
    abortRequests();
    copilot?.reset();
    hideFrames();
    closeConfirm();
    render();
    notice(message);
  }
  async function connect() {
    let origin;
    try {
      origin = C.gatewayOrigin($("host").value.trim());
    } catch (error) {
      notice(error.message, true);
      return;
    }
    if (origin !== location.origin) {
      notice(
        "Abra esse endereço pela configuração de conexão do Android ou pelo navegador. O token não é transferido entre estações.",
        true,
      );
      return;
    }
    autoConnect = true;
    state.generation++;
    abortRequests();
    hideFrames();
    state.snapshot = null;
    state.connected = false;
    state.nonce = "";
    state.pending = false;
    state.token = $("token").value.trim();
    copilot?.reset();
    $("token").value = "";
    state.logs = [];
    state.eventKeys.clear();
    state.clearedKeys.clear();
    state.missions = {};
    state.models = [];
    state.lastSeen = 0;
    $("connection-dialog").close();
    notice("Conectando ao gateway…");
    render();
    stateBusy = false;
    await pollState();
    if (state.connected) await loadCatalogues();
  }
  async function loadCatalogues() {
    const generation = state.generation;
    const [missions, models] = await Promise.allSettled([
      request("/api/v1/missions"),
      request("/api/v1/models"),
    ]);
    if (generation !== state.generation) return;
    if (missions.status === "fulfilled") {
      state.missions = missions.value.missions || {};
      $("mission-select").replaceChildren(
        new Option("Selecione uma missão", ""),
      );
      for (const [name, mission] of Object.entries(state.missions))
        $("mission-select").add(new Option(mission.nome || name, name));
      $("mission-select").disabled = false;
      updateMission();
    } else
      log(
        `Catálogo de missões indisponível: ${missions.reason.message}`,
        "warning",
      );
    if (models.status === "fulfilled") fillModels(models.value);
    else
      log(
        `Catálogo de redes indisponível: ${models.reason.message}`,
        "warning",
      );
  }
  async function pollState() {
    if (stateBusy || state.suspended || document.hidden || !autoConnect) return;
    stateBusy = true;
    const generation = state.generation;
    try {
      const snapshot = await request("/api/v1/state");
      if (generation !== state.generation) return;
      if (
        snapshot.version !== 1 ||
        !snapshot.topics ||
        typeof snapshot.topics !== "object"
      )
        throw new Error("Versão do gateway incompatível.");
      const recovered = !state.connected;
      state.snapshot = snapshot;
      state.connected = true;
      state.lastSeen = performance.now();
      state.nonce = snapshot.command_nonce || "";
      if (recovered) {
        log("Conexão estabelecida.");
        notice(
          snapshot.mode === "demo"
            ? "Demonstração · dados simulados, comandos desativados."
            : snapshot.commands_enabled
              ? "Conectado. Acompanhe a telemetria para confirmar a execução de comandos."
              : "Conectado em modo de leitura. O gateway está com comandos desativados.",
        );
      }
      for (const event of snapshot.events || []) {
        const key = JSON.stringify(event);
        if (!state.eventKeys.has(key) && !state.clearedKeys.has(key)) {
          state.eventKeys.add(key);
          log(event.message, event.level, event.time);
        }
      }
      if (state.eventKeys.size > 500)
        state.eventKeys = new Set([...state.eventKeys].slice(-250));
      render();
    } catch (error) {
      if (generation !== state.generation) return;
      const wasConnected = state.connected;
      state.connected = false;
      state.nonce = "";
      closeConfirm();
      if (wasConnected) copilot?.reset();
      render();
      if (!wasConnected && error.status === 401) autoConnect = false;
      notice(
        !wasConnected && error.status === 401
          ? "Sem conexão. Abra Conectar e informe o token da estação."
          : wasConnected
            ? `Conexão perdida: ${error.message} Comandos bloqueados.`
            : "Sem conexão com a estação. O dashboard continua disponível.",
        wasConnected,
      );
      if (wasConnected)
        log("Conexão perdida; controles bloqueados.", "warning");
    } finally {
      if (generation === state.generation) stateBusy = false;
    }
  }
  function render() {
    const online =
      state.connected &&
      performance.now() - state.lastSeen < 3000 &&
      !state.suspended &&
      !document.hidden;
    $("link-status").textContent = online
      ? state.snapshot?.mode === "demo"
        ? "DEMO"
        : "Conectado"
      : "Sem conexão";
    $("link-status").className = `badge ${online ? "live" : "waiting"}`;
    $("mode-note").textContent = online
      ? state.snapshot?.mode === "demo"
        ? "Simulação visual · somente leitura"
        : `Gateway ${location.host}`
      : "Sem conexão com a estação";
    const values = (key) => C.liveValues(state.snapshot, key, online);
    const drone = values("drone"),
      status = values("status"),
      mission = values("mission"),
      battery = values("battery");
    const names = {
      drone: drone.state_name,
      status: status.nav_state_name,
      mission: mission.state_name || mission.phase || mission.state,
      battery:
        battery.connected &&
        C.finite(battery.remaining) &&
        battery.remaining >= 0 &&
        battery.remaining <= 1
          ? C.numeric(battery.remaining * 100, "%", 0)
          : "—",
    };
    for (const key of ["drone", "status", "mission", "battery"]) {
      $(`${key}-value`).textContent = String(names[key] ?? "—").replaceAll(
        "_",
        " ",
      );
      $(`${key}-note`).textContent = online
        ? C.healthLabel(state.snapshot?.topics?.[key]?.health)
        : "Sem conexão";
    }
    $("lidar-distance").textContent =
      `Mais próximo: ${C.numeric(values("lidar").minimum_distance, "m")}`;
    $("down-distance").textContent =
      `Abaixo: ${C.numeric(values("down").minimum_distance, "m")}`;
    $("depth-distance").textContent =
      `Distância mínima: ${C.numeric(values("depth").minimum_distance, "m")}`;
    renderTopics(online);
    updatePosition(values("global"));
    drawRadar(online);
    video?.tick();
    syncButtons();
  }
  const topicLabels = {
    drone: "Estado do drone",
    status: "Estado PX4",
    mission: "Estado da missão",
    battery: "Bateria",
    global: "Posição GPS",
    local: "Posição local",
    lidar: "LiDAR",
    down: "Distância abaixo",
    depth: "Profundidade",
    cv: "Visão computacional",
  };
  function renderTopics(online) {
    const topics = state.snapshot?.topics || {};
    const keys = [
      ...new Set([...Object.keys(topicLabels), ...Object.keys(topics)]),
    ];
    const selected = $("topic-select").value || "drone";
    if (keys.join("|") !== $("topic-select").dataset.keys) {
      $("topic-select").replaceChildren(
        ...keys.map(
          (key) =>
            new Option(topics[key]?.label || topicLabels[key] || key, key),
        ),
      );
      $("topic-select").dataset.keys = keys.join("|");
      $("topic-select").value = keys.includes(selected) ? selected : keys[0];
    }
    const key = $("topic-select").value;
    const sample = topics[key] || {};
    const health = online ? sample.health : "waiting";
    const active = online
      ? Object.values(topics).filter((item) => item.health === "live").length
      : 0;
    $("topic-count").textContent = `${active} ativos`;
    $("topic-count").className = `badge ${active ? "live" : "waiting"}`;
    $("topic-health").className =
      `badge ${["live", "stale"].includes(health) ? health : "waiting"}`;
    $("topic-health").textContent = !online
      ? "Sem conexão"
      : C.healthLabel(health);
    $("topic-rate").textContent = online
      ? `${C.numeric(sample.hz, "Hz")} · idade ${C.numeric(sample.age_s, "s")}`
      : "Aguardando dados";
    $("topic-path").textContent = sample.topic || "Tópico ainda não recebido";
    const text = online
      ? JSON.stringify(sample.values || {}, null, 2)
      : "Sem conexão. Os valores serão exibidos quando a estação estiver disponível.";
    // Only changing content resets pagination; polling must not steal the selected page.
    setReaderText("topic-data", text, false);
  }
  function setupMap() {
    if (!window.L) {
      $("map").textContent =
        "Mapa indisponível: biblioteca local não carregada.";
      return;
    }
    map = L.map("map", { attributionControl: true }).setView([0, 0], 2);
    routeLayer = L.polyline([], {
      color: "#4de0c1",
      weight: 3,
      dashArray: "7 6",
    }).addTo(map);
    pointsLayer = L.layerGroup().addTo(map);
    new ResizeObserver(scheduleLayout).observe($("map"));
  }
  function updatePosition(values) {
    const position = C.gps(values);
    $("position-note").textContent = position
      ? `${position[0].toFixed(6)}, ${position[1].toFixed(6)} · ${C.numeric(values.alt, "m AMSL")}`
      : "GPS sem dados recentes";
    $("map-empty").hidden = Boolean(position);
    if (!map) return;
    if (!position) {
      if (marker) {
        map.removeLayer(marker);
        marker = null;
      }
      return;
    }
    if (!marker)
      marker = L.circleMarker(position, {
        radius: 8,
        color: "#f6fbff",
        weight: 3,
        fillColor: "#4de0c1",
        fillOpacity: 1,
      }).addTo(map);
    else marker.setLatLng(position);
    if (!fitted) {
      map.setView(position, 18);
      fitted = true;
    }
  }
  function updateMission() {
    const mission = state.missions[$("mission-select").value];
    const points = C.route(mission);
    const previous = $("mission-point-select").value;
    $("mission-point-select").replaceChildren(
      new Option("Resumo da missão", "summary"),
    );
    points.forEach((point, index) =>
      $("mission-point-select").add(
        new Option(`Ponto ${index + 1}`, String(index)),
      ),
    );
    if (
      [...$("mission-point-select").options].some(
        (option) => option.value === previous,
      )
    )
      $("mission-point-select").value = previous;
    $("mission-points").replaceChildren();
    for (const text of mission
      ? [`${points.length} pontos`, mission.objeto_detectavel || "Inspeção"]
      : ["Sem rota disponível"]) {
      const pill = document.createElement("span");
      pill.textContent = text;
      $("mission-points").append(pill);
    }
    updateMissionDetail();
    $("route-note").textContent = points.length
      ? `${points.length} pontos planejados`
      : "Nenhuma rota selecionada";
    if (map) {
      routeLayer.setLatLngs(points);
      pointsLayer.clearLayers();
      points.forEach((point, index) =>
        L.circleMarker(point, { radius: 5, color: "#4de0c1", fillOpacity: 0.5 })
          .bindTooltip(String(index + 1))
          .addTo(pointsLayer),
      );
      if (points.length)
        map.fitBounds(routeLayer.getBounds(), {
          padding: [28, 28],
          maxZoom: 19,
        });
    }
    syncButtons();
  }
  function updateMissionDetail() {
    const mission = state.missions[$("mission-select").value];
    const points = C.route(mission);
    const selected = $("mission-point-select").value;
    const index = Number(selected);
    $("mission-description").textContent =
      selected !== "summary" && points[index]
        ? `Ponto ${index + 1} de ${points.length}\n\nLatitude: ${points[index][0].toFixed(7)}°\nLongitude: ${points[index][1].toFixed(7)}°\n\nCoordenadas geográficas da rota cadastrada.`
        : mission?.descricao ||
          (state.connected
            ? "Selecione uma missão para revisar o plano."
            : "Sem conexão. O catálogo será recebido da estação.");
  }
  function drawRadar(online) {
    const canvas = $("radar"),
      rect = canvas.getBoundingClientRect(),
      dpr = Math.min(devicePixelRatio || 1, 2);
    if (!rect.width || !rect.height) return;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    const cx = rect.width / 2,
      cy = rect.height / 2,
      r = Math.max(1, Math.min(cx, cy) - (rect.height < 130 ? 7 : 24));
    ctx.strokeStyle = "#263b50";
    ctx.fillStyle = "#93a4bb";
    ctx.lineWidth = 1;
    ctx.font = "10px system-ui";
    ctx.textAlign = "center";
    const rings = r < 55 ? 1 : r < 90 ? 2 : 4;
    for (let ring = 1; ring <= rings; ring++) {
      ctx.beginPath();
      ctx.arc(cx, cy, (r * ring) / rings, 0, Math.PI * 2);
      ctx.stroke();
      if (r >= 25)
        ctx.fillText(
          `${Math.round((20 * ring) / rings)} m`,
          cx + (r * ring) / rings - 12,
          cy + 12,
        );
    }
    ctx.beginPath();
    ctx.moveTo(cx - r, cy);
    ctx.lineTo(cx + r, cy);
    ctx.moveTo(cx, cy - r);
    ctx.lineTo(cx, cy + r);
    ctx.stroke();
    if (rect.height >= 130) ctx.fillText("FRENTE", cx, cy - r - 10);
    const radar = state.snapshot?.radar;
    const valid = online && C.finite(radar?.age_s) && radar.age_s < 3;
    $("radar-note").textContent = valid
      ? `Varredura há ${C.numeric(radar.age_s, "s")}`
      : "LiDAR sem dados recentes";
    if (valid) {
      ctx.fillStyle = "#4de0c1";
      for (const point of radar.points || []) {
        const [distance, angle] = point;
        if (
          !C.finite(distance) ||
          !C.finite(angle) ||
          distance < 0 ||
          distance > 20
        )
          continue;
        ctx.beginPath();
        ctx.arc(
          cx - ((Math.sin(angle) * distance) / 20) * r,
          cy - ((Math.cos(angle) * distance) / 20) * r,
          2,
          0,
          Math.PI * 2,
        );
        ctx.fill();
      }
    }
    if (!valid && rect.height >= 150) {
      ctx.fillStyle = "#93a4bb";
      ctx.fillText(
        online ? "Sem varredura recente" : "Sem conexão",
        cx,
        cy + Math.min(r * 0.6, 28),
      );
    }
    ctx.fillStyle = "#e6edf5";
    ctx.beginPath();
    ctx.moveTo(cx, cy - 7);
    ctx.lineTo(cx - 5, cy + 5);
    ctx.lineTo(cx + 5, cy + 5);
    ctx.closePath();
    ctx.fill();
  }
  function fillModels(data) {
    state.models = Array.isArray(data.models) ? data.models : [];
    for (const [id, type, current] of [
      ["object-model", "equipment", data.current_object_model],
      ["anomaly-model", "anomaly", data.current_anomaly_model],
    ]) {
      $(id).replaceChildren(new Option("Selecione uma rede", ""));
      for (const model of state.models.filter(
        (item) => item.object_type === type,
      )) {
        const option = new Option(
          `${model.name || model.file_name}${model.available === false ? " · indisponível" : ""}`,
          model.file_name,
        );
        option.disabled = model.available === false;
        $(id).add(option);
      }
      if (current) $(id).value = current;
      modelInfo(id);
    }
    syncButtons();
  }
  function modelInfo(id) {
    const model = state.models.find((item) => item.file_name === $(id).value);
    $(`${id}-info`).textContent = model
      ? [
          model.model,
          model.type,
          model.classes?.join(", "),
          model.validation_warning,
        ]
          .filter(Boolean)
          .join(" · ")
      : "Escolha uma rede disponível no computador ROS.";
    syncButtons();
  }
  function closeConfirm() {
    confirmAction = null;
    if ($("confirm-dialog").open) $("confirm-dialog").close();
  }
  function askConfirm(command, args, title, description) {
    if (!enabled(command)) return;
    confirmAction = { command, args };
    $("confirm-title").textContent = title;
    $("confirm-description").textContent = description;
    $("confirm-dialog").showModal();
    syncButtons();
  }
  async function command(name, args = {}, confirmed = false) {
    if (!enabled(name)) {
      notice(
        "Comando bloqueado. Aguarde uma conexão e telemetria recentes.",
        true,
      );
      return;
    }
    const generation = state.generation;
    const bytes = new Uint8Array(16);
    crypto.getRandomValues(bytes);
    bytes[6] = (bytes[6] & 15) | 64;
    bytes[8] = (bytes[8] & 63) | 128;
    const hex = Array.from(bytes, (value) =>
      value.toString(16).padStart(2, "0"),
    ).join("");
    const id = `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
    const nonce = state.nonce;
    state.nonce = "";
    state.pending = true;
    syncButtons();
    notice("Enviando comando…");
    try {
      const result = await request("/api/v1/commands", {
        method: "POST",
        body: JSON.stringify({ id, nonce, command: name, args, confirmed }),
      });
      if (generation !== state.generation) return;
      const labels = {
        submitted: "Enviado. Aguarde confirmação pela telemetria.",
        completed: "Concluído pelo gateway.",
        rejected: "Comando rejeitado.",
        uncertain:
          "Resultado incerto. Consulte o estado antes de enviar outra ação.",
      };
      const message = `${labels[result.status] || "Resposta desconhecida; consulte o estado."} ${result.message || ""}`;
      notice(message, ["rejected", "uncertain"].includes(result.status));
      log(
        `${name}: ${message}`,
        ["rejected", "uncertain"].includes(result.status) ? "warning" : "info",
      );
    } catch (error) {
      if (generation !== state.generation) return;
      const message =
        error.status >= 400 && error.status < 500
          ? `Comando rejeitado: ${error.message}`
          : "Não foi possível confirmar a resposta. O comando pode ter sido recebido; consulte a telemetria. Nenhum reenvio automático.";
      notice(message, true);
      log(`${name}: ${message}`, "warning");
    } finally {
      if (generation === state.generation) state.pending = false;
      syncButtons();
    }
  }
  async function openHistory() {
    $("history-dialog").showModal();
    $("session-events").textContent = "Carregando…";
    const generation = state.generation;
    try {
      const data = await request("/api/v1/sessions");
      if (generation !== state.generation) return;
      $("session-select").replaceChildren(
        new Option("Selecione uma sessão", ""),
      );
      for (const session of data.sessions || [])
        $("session-select").add(new Option(session.name, session.name));
      $("session-events").textContent = data.sessions?.length
        ? "Selecione uma sessão."
        : "Nenhuma sessão registrada.";
    } catch (error) {
      $("session-events").textContent = error.message;
    }
  }
  function suspend() {
    state.suspended = true;
    state.connected = false;
    state.nonce = "";
    state.generation++;
    state.pending = false;
    abortRequests();
    stateBusy = false;
    closeConfirm();
    copilot?.reset();
    render();
  }
  function resume() {
    state.suspended = false;
    pollState();
  }
  const readers = new Map();
  let layoutFrame = 0;
  function setReaderText(id, text, reset = true) {
    const reader = readers.get(id);
    if (!reader) {
      $(id).textContent = text;
      return;
    }
    if (reader.text === text) return;
    reader.text = text;
    if (reset) reader.page = 0;
    drawReader(reader);
  }
  function drawReader(reader) {
    const { pre } = reader;
    if (!pre.clientWidth || !pre.clientHeight) return;
    const style = getComputedStyle(pre);
    const context = document.createElement("canvas").getContext("2d");
    context.font = style.font;
    const charWidth = context.measureText("M").width || 8;
    const columns = Math.max(
      1,
      Math.floor(
        (pre.clientWidth -
          parseFloat(style.paddingLeft) -
          parseFloat(style.paddingRight)) /
          charWidth,
      ),
    );
    const rows = Math.max(
      1,
      Math.floor(
        (pre.clientHeight -
          parseFloat(style.paddingTop) -
          parseFloat(style.paddingBottom)) /
          parseFloat(style.lineHeight),
      ),
    );
    const pages = C.paginateText(reader.text, columns, rows);
    reader.page = Math.min(reader.page, pages.length - 1);
    reader.rendered = pages[reader.page];
    if (pre.textContent !== reader.rendered) pre.textContent = reader.rendered;
    reader.label.textContent = `${reader.page + 1} / ${pages.length}`;
    reader.previous.disabled = reader.page <= 0;
    reader.next.disabled = reader.page >= pages.length - 1;
  }
  function scheduleLayout() {
    if (layoutFrame) return;
    layoutFrame = requestAnimationFrame(() => {
      layoutFrame = 0;
      if (map && !$("flight").hidden) map.invalidateSize({ pan: false });
      drawRadar(state.connected && !state.suspended);
      readers.forEach(drawReader);
    });
  }
  function showTab(id, focus = false) {
    if (!$(id)?.classList.contains("view")) return;
    document.querySelectorAll(".view").forEach((view) => {
      view.hidden = view.id !== id;
    });
    document.querySelectorAll('[role="tab"]').forEach((button) => {
      const active = button.dataset.openTab === id;
      button.setAttribute("aria-selected", String(active));
      button.tabIndex = active ? 0 : -1;
      if (active && focus) button.focus();
    });
    video?.tick();
    scheduleLayout();
  }
  function initializeLayout() {
    document.querySelectorAll(".reader > pre").forEach((pre) => {
      const pager = document.createElement("div");
      pager.className = "pager";
      const previous = document.createElement("button");
      previous.type = "button";
      previous.textContent = "‹";
      previous.setAttribute("aria-label", "Página anterior");
      const next = document.createElement("button");
      next.type = "button";
      next.textContent = "›";
      next.setAttribute("aria-label", "Próxima página");
      const label = document.createElement("span");
      label.setAttribute("aria-live", "polite");
      pager.append(previous, label, next);
      pre.parentElement.append(pager);
      const reader = {
        pre,
        text: pre.textContent,
        rendered: pre.textContent,
        page: 0,
        previous,
        next,
        label,
      };
      readers.set(pre.id, reader);
      previous.addEventListener("click", () => {
        reader.page--;
        drawReader(reader);
      });
      next.addEventListener("click", () => {
        reader.page++;
        drawReader(reader);
      });
      new MutationObserver(() => {
        if (pre.textContent === reader.rendered) return;
        setReaderText(pre.id, pre.textContent);
      }).observe(pre, { childList: true, characterData: true, subtree: true });
      new ResizeObserver(scheduleLayout).observe(pre.parentElement);
    });
    document
      .querySelectorAll("[data-open-tab]")
      .forEach((button) =>
        button.addEventListener("click", () => showTab(button.dataset.openTab)),
      );
    document.querySelectorAll('[role="tab"]').forEach((button) =>
      button.addEventListener("keydown", (event) => {
        const tabs = [...document.querySelectorAll('[role="tab"]')];
        let index = tabs.indexOf(button);
        if (event.key === "ArrowRight") index = (index + 1) % tabs.length;
        else if (event.key === "ArrowLeft")
          index = (index + tabs.length - 1) % tabs.length;
        else if (event.key === "Home") index = 0;
        else if (event.key === "End") index = tabs.length - 1;
        else return;
        event.preventDefault();
        showTab(tabs[index].dataset.openTab, true);
      }),
    );
    for (const [id, key] of [
      ["flight-view-select", "flightView"],
      ["camera-view-select", "cameraView"],
      ["diagnostic-view-select", "diagnosticView"],
      ["copilot-view-select", "copilotView"],
    ]) {
      $(id).addEventListener("change", () => {
        document.body.dataset[key] = $(id).value;
        video?.tick();
        scheduleLayout();
      });
    }
    const android =
      new URLSearchParams(location.search).get("app") === "android";
    document.querySelectorAll(".native-only").forEach((element) => {
      element.hidden = !android;
    });
    const resizeViewport = () => {
      // The visual viewport shrinks when the keyboard opens, without resetting the active tab.
      document.body.style.height = `${Math.round(window.visualViewport?.height || innerHeight)}px`;
      scheduleLayout();
    };
    window.visualViewport?.addEventListener("resize", resizeViewport);
    window.addEventListener("resize", resizeViewport);
    window.DroneLayout = {
      showTab,
      setText: setReaderText,
      showCopilotPane: (pane) => {
        $("copilot-view-select").value = pane;
        document.body.dataset.copilotView = pane;
        scheduleLayout();
      },
    };
    resizeViewport();
  }
  $("host").value = location.origin;
  $("mission-point-select").addEventListener("change", updateMissionDetail);
  $("topic-select").addEventListener("change", () =>
    renderTopics(state.connected),
  );
  $("notice-open").addEventListener("click", () => {
    $("notice-detail").textContent = $("notice").textContent;
    $("notice-dialog").showModal();
  });
  $("connection-open").addEventListener("click", () => {
    $("connection-dialog").showModal();
  });
  $("connection-form").addEventListener("submit", (event) => {
    event.preventDefault();
    connect();
  });
  $("disconnect").addEventListener("click", () => {
    disconnect();
    $("connection-dialog").close();
    state.suspended = true;
  });
  $("connection-form").addEventListener(
    "submit",
    () => {
      state.suspended = false;
    },
    true,
  );
  document
    .querySelectorAll(".dialog-close")
    .forEach((button) =>
      button.addEventListener("click", () => button.closest("dialog").close()),
    );
  $("confirm-dialog").addEventListener("close", () => {
    confirmAction = null;
  });
  $("confirm-send").addEventListener("click", () => {
    const action = confirmAction;
    closeConfirm();
    if (action) command(action.command, action.args, true);
  });
  $("mission-select").addEventListener("change", updateMission);
  $("mission-start").addEventListener("click", () => {
    const mission = $("mission-select").value;
    askConfirm(
      "mission.start",
      { mission },
      "Iniciar missão?",
      `Enviar a missão “${state.missions[mission]?.nome || mission}” para o controlador? Verifique a rota, a área de voo e os estados do drone antes de confirmar.`,
    );
  });
  $("mission-cancel").addEventListener("click", () =>
    askConfirm(
      "mission.cancel",
      {},
      "Cancelar missão?",
      "Solicitar o cancelamento ao controlador de missão. Acompanhe o estado do drone e da missão para confirmar a interrupção.",
    ),
  );
  document
    .querySelectorAll("[data-command]")
    .forEach((button) =>
      button.addEventListener("click", () =>
        command(
          button.dataset.command,
          button.dataset.enabled === undefined
            ? {}
            : { enabled: button.dataset.enabled === "true" },
        ),
      ),
    );
  $("models-open").addEventListener("click", async () => {
    $("models-dialog").showModal();
    try {
      const data = await request("/api/v1/models");
      fillModels(data);
    } catch (error) {
      log(error.message, "warning");
    }
  });
  for (const id of ["object-model", "anomaly-model"])
    $(id).addEventListener("change", () => modelInfo(id));
  $("models-apply").addEventListener("click", () =>
    command("cv.models", {
      object_model: $("object-model").value,
      anomaly_model: $("anomaly-model").value,
    }),
  );
  $("map-fit").addEventListener("click", () => {
    if (!map) return;
    const bounds = routeLayer.getBounds();
    if (marker) bounds.extend(marker.getLatLng());
    if (bounds.isValid())
      map.fitBounds(bounds, { padding: [28, 28], maxZoom: 19 });
  });
  $("map-tiles").addEventListener("change", () => {
    if (!map) return;
    if ($("map-tiles").checked) {
      tileLayer = L.tileLayer(
        "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
          maxZoom: 19,
          attribution:
            '© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a>',
        },
      ).addTo(map);
    } else if (tileLayer) {
      map.removeLayer(tileLayer);
      tileLayer = null;
    }
  });
  document.querySelectorAll(".expand-video").forEach((button) =>
    button.addEventListener("click", () => {
      $("expanded-title").textContent = {
        camera: "Câmera", cv: "Detecção", depth: "Profundidade",
      }[button.dataset.kind];
      $("video-dialog").showModal();
      video.expanded();
    }),
  );
  $("video-dialog").addEventListener("close", () => video.expanded());
  $("logs-clear").addEventListener("click", () => {
    state.clearedKeys = new Set(state.eventKeys);
    state.logs = [];
    renderLogs();
  });
  $("history-open").addEventListener("click", openHistory);
  $("session-select").addEventListener("change", async () => {
    const name = $("session-select").value;
    if (!name) return;
    $("session-events").textContent = "Carregando…";
    try {
      const data = await request(
        `/api/v1/sessions/${encodeURIComponent(name)}`,
      );
      if ($("session-select").value === name)
        $("session-events").textContent =
          (data.events || [])
            .map((event) => JSON.stringify(event, null, 2))
            .join("\n\n") || "Sessão sem eventos.";
    } catch (error) {
      $("session-events").textContent = error.message;
    }
  });
  document.addEventListener("visibilitychange", () =>
    document.hidden ? suspend() : resume(),
  );
  window.addEventListener("app:pause", suspend);
  window.addEventListener("app:resume", resume);
  window.addEventListener("pagehide", () => {
    suspend();
    hideFrames();
    state.token = "";
  });
  new ResizeObserver(scheduleLayout).observe($("radar"));
  initializeLayout();
  copilot = new window.DroneCopilot({
    request,
    canCommand: enabled,
    getNonce: () => state.nonce,
    consumeNonce: () => {
      state.nonce = "";
    },
    isConnected: () => state.connected && !state.suspended && !document.hidden,
    showNotice: notice,
  });
  video = new window.DroneVideo({
    getContext: () => ({
      active: state.connected && performance.now() - state.lastSeen < 3000 &&
        !state.suspended && !document.hidden && !$("payload").hidden,
      stream: $("camera-view-select").value,
      frames: state.snapshot?.frames,
    }),
    getRequest: () => {
      const authToken = state.token;
      return (path, options = {}) => request(path, { ...options, authToken });
    },
    notice,
  });
  setupMap();
  render();
  fillModels({ models: [] });
  pollState().then(() => {
    if (state.connected) loadCatalogues();
  });
  setInterval(pollState, 1000);
  setInterval(() => video.tick(), 500);
  setInterval(() => {
    if (state.connected && performance.now() - state.lastSeen >= 3000) {
      state.connected = false;
      state.nonce = "";
      closeConfirm();
      copilot?.reset();
      notice("Telemetria sem atualização. Comandos bloqueados.", true);
      render();
    }
    syncButtons();

  }, 500);
})();
