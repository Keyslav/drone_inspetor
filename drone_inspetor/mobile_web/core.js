/* Shared pure helpers: also exercised with node --test. */
(function (root) {
  "use strict";
  const finite = (value) => typeof value === "number" && Number.isFinite(value);
  const numeric = (value, unit = "", digits = 1) =>
    finite(value) ? `${value.toFixed(digits)}${unit ? ` ${unit}` : ""}` : "—";
  const healthLabel = (health) =>
    ({
      live: "Recebendo",
      stale: "Dados desatualizados",
      waiting: "Aguardando dados",
    })[health] || "Aguardando dados";
  function liveValues(snapshot, key, connected) {
    const sample = snapshot?.topics?.[key];
    return connected && sample?.health === "live" ? sample.values || {} : {};
  }
  function gps(values) {
    const lat = values.lat ?? values.latitude;
    const lon = values.lon ?? values.longitude;
    return finite(lat) &&
      finite(lon) &&
      Math.abs(lat) <= 90 &&
      Math.abs(lon) <= 180
      ? [lat, lon]
      : null;
  }
  function route(mission) {
    return (mission?.pontos_de_inspecao || [])
      .map((point) => gps(point))
      .filter(Boolean);
  }
  function canWrite({ connected, enabled, lastSeen, now, suspended, pending }) {
    return (
      connected === true &&
      enabled === true &&
      !suspended &&
      !pending &&
      now - lastSeen >= 0 &&
      now - lastSeen < 3000
    );
  }
  function canCommand(snapshot, command) {
    const topics = snapshot?.topics || {},
      mission = topics.mission;
    const phase = mission?.values?.state_name || "";
    if (command.startsWith("mission.")) {
      if (
        !["mission", "drone", "status"].every(
          (key) => topics[key]?.health === "live",
        )
      )
        return false;
      return command === "mission.start"
        ? phase === "PRONTO"
        : phase.startsWith("EXECUTANDO") || phase === "INSPECAO_FINALIZADA";
    }
    if (command.startsWith("cv."))
      return (
        mission?.health === "live" &&
        !mission.values?.on_mission &&
        !/^(EXECUTANDO|RETORNANDO)/.test(phase)
      );
    return true;
  }
  function gatewayOrigin(value) {
    const url = new URL(value);
    if (
      !["http:", "https:"].includes(url.protocol) ||
      url.username ||
      url.password ||
      url.search ||
      url.hash ||
      !["", "/"].includes(url.pathname)
    )
      throw new Error(
        "Use somente http(s)://host:porta, sem caminho ou credenciais.",
      );
    return url.origin;
  }
  // Fixed-height readers paginate full values rather than hiding overflow or adding scroll.
  function paginateText(text, columns, rows) {
    columns = Math.max(1, Math.floor(columns) || 1);
    rows = Math.max(1, Math.floor(rows) || 1);
    const lines = [];
    for (const raw of String(text ?? "")
      .replaceAll("\r", "")
      .split("\n")) {
      let chars = Array.from(raw);
      if (!chars.length) lines.push("");
      while (chars.length) {
        let size = Math.min(columns, chars.length);
        if (size < chars.length) {
          const space = chars.slice(0, size).lastIndexOf(" ");
          if (space > size / 2) size = space + 1;
        }
        lines.push(chars.splice(0, size).join(""));
      }
    }
    const pages = [];
    for (let offset = 0; offset < lines.length; offset += rows)
      pages.push(lines.slice(offset, offset + rows).join("\n"));
    return pages.length ? pages : [""];
  }
  const api = {
    paginateText,
    finite,
    numeric,
    healthLabel,
    liveValues,
    gps,
    route,
    canWrite,
    canCommand,
    gatewayOrigin,
  };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.DroneCore = api;
})(typeof window === "undefined" ? globalThis : window);
