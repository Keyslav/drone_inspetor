/* Regressões de apresentação e habilitação; nenhuma conexão de rede. */
const test = require("node:test");
const assert = require("node:assert/strict");
const C = require("../drone_inspetor/mobile_web/core.js");

test("ausência e coordenadas inválidas não viram zero", () => {
  for (const value of [null, undefined, NaN, Infinity, "1"])
    assert.equal(C.numeric(value), "—");
  assert.deepEqual(C.gps({ lat: 0, lon: 0 }), [0, 0]);
  assert.equal(C.gps({ lat: 91, lon: 0 }), null);
  assert.deepEqual(
    C.liveValues(
      { topics: { drone: { health: "stale", values: { x: 1 } } } },
      "drone",
      true,
    ),
    {},
  );
});
test("comandos bloqueados após perda, suspensão e resposta pendente", () => {
  const base = {
    connected: true,
    enabled: true,
    lastSeen: 1000,
    now: 2000,
    suspended: false,
    pending: false,
  };
  assert.equal(C.canWrite(base), true);
  for (const patch of [
    { connected: false },
    { enabled: false },
    { now: 4000 },
    { now: 999 },
    { suspended: true },
    { pending: true },
  ])
    assert.equal(C.canWrite({ ...base, ...patch }), false);
});
test("missão exige produtores recentes e estado compatível", () => {
  const snapshot = {
    topics: {
      mission: { health: "live", values: { state_name: "PRONTO" } },
      drone: { health: "live" },
      status: { health: "live" },
    },
  };
  assert.equal(C.canCommand(snapshot, "mission.start"), true);
  assert.equal(C.canCommand(snapshot, "mission.cancel"), false);
  snapshot.topics.mission.values.state_name = "INSPECAO_FINALIZADA";
  assert.equal(C.canCommand(snapshot, "mission.cancel"), true);
  snapshot.topics.status.health = "stale";
  assert.equal(C.canCommand(snapshot, "mission.cancel"), false);
});
test("CV preserva recursos durante missão", () => {
  assert.equal(
    C.canCommand(
      { topics: { mission: { health: "live", values: { on_mission: true } } } },
      "cv.record",
    ),
    false,
  );
  assert.equal(
    C.canCommand(
      {
        topics: {
          mission: { health: "live", values: { state_name: "PRONTO" } },
        },
      },
      "cv.models",
    ),
    true,
  );
});
test("configuração aceita apenas origem sem segredo ou caminho", () => {
  assert.equal(
    C.gatewayOrigin("http://192.168.1.1:8765/"),
    "http://192.168.1.1:8765",
  );
  for (const value of [
    "javascript:alert(1)",
    "http://u:p@host",
    "https://host/?token=abc",
    "https://host/x",
  ])
    assert.throws(() => C.gatewayOrigin(value));
});

test("paginação preserva texto longo e caracteres Unicode sem rolagem", () => {
  const text = "Drone Flare — ação confirmada 🚁 ".repeat(12);
  const pages = C.paginateText(text, 17, 3);
  assert.ok(pages.length > 1);
  assert.equal(pages.join("\n").replaceAll("\n", ""), text);
  for (const page of pages) {
    const lines = page.split("\n");
    assert.ok(lines.length <= 3);
    for (const line of lines) assert.ok(Array.from(line).length <= 17);
  }
});

test("paginação mantém linhas vazias e suporta área mínima", () => {
  assert.deepEqual(C.paginateText("a\n\nb", 10, 2), ["a\n", "b"]);
  assert.deepEqual(C.paginateText("🚁ç", 0, 0), ["🚁", "ç"]);
  assert.deepEqual(C.paginateText("", 10, 2), [""]);
});
