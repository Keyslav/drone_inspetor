/* Fluxo de voz e respostas tardias: nenhuma chamada ROS ou de rede. */
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

function rig(request = async () => ({})) {
  const nodes = new Map(),
    events = {};
  const element = (id) => {
    if (!nodes.has(id))
      nodes.set(id, {
        value: "",
        open: false,
        disabled: false,
        textContent: "",
        addEventListener() {},
        scrollIntoView() {},
        replaceChildren() {},
        add() {},
        close() {
          this.open = false;
        },
        showModal() {
          this.open = true;
        },
      });
    return nodes.get(id);
  };
  const window = {
    addEventListener: (name, fn) => {
      events[name] = fn;
    },
  };
  vm.runInNewContext(
    fs.readFileSync(
      path.join(__dirname, "../drone_inspetor/mobile_web/copilot.js"),
      "utf8",
    ),
    {
      window,
      document: { getElementById: element },
      performance: { now: () => 0 },
      Option: function (label, value) {
        this.label = label;
        this.value = value;
      },
    },
  );
  const calls = [];
  let connected = true;
  const copilot = new window.DroneCopilot({
    request: (...args) => {
      calls.push(args);
      return request(...args);
    },
    canCommand: () => true,
    getNonce: () => "nonce",
    consumeNonce() {},
    isConnected: () => connected,
    showNotice() {},
  });
  return {
    copilot,
    element,
    events,
    calls,
    disconnect() {
      connected = false;
      copilot.reset();
    },
  };
}

test("voz preenche o pedido sem preparar nem executar", () => {
  const r = rig();
  r.events["copilot:transcript"]({ detail: "iniciar Flare" });
  assert.equal(r.element("copilot-prompt").value, "iniciar Flare");
  assert.equal(r.calls.length, 0);
});

test("resposta de preparação após desconexão não restaura uma proposta", async () => {
  let resolve;
  const r = rig(
    () =>
      new Promise((done) => {
        resolve = done;
      }),
  );
  r.element("copilot-prompt").value = "iniciar Flare";
  const preparing = r.copilot.prepare();
  r.disconnect();
  resolve({ id: "old" });
  await preparing;
  assert.equal(r.copilot.proposal, null);
  assert.equal(r.copilot.data, null);
  assert.equal(r.calls.length, 1);
});

test("confirmar não troca a proposta revisada por outra selecionada", async () => {
  const r = rig();
  r.copilot.proposal = {
    id: "one",
    can_execute: true,
    command: "camera.capture",
  };
  r.copilot.review();
  r.copilot.proposal = {
    id: "two",
    can_execute: true,
    command: "mission.start",
  };
  await r.copilot.confirm();
  assert.equal(r.calls.length, 0);
});

test("falha ao consultar propostas remove possibilidade de confirmar", async () => {
  const r = rig(async () => {
    throw new Error("offline");
  });
  r.copilot.proposal = {
    id: "one",
    can_execute: true,
    command: "camera.capture",
  };
  r.copilot.review();
  await r.copilot.refresh();
  assert.equal(r.copilot.proposal, null);
  assert.equal(r.element("copilot-confirm-dialog").open, false);
  assert.equal(r.element("copilot-confirm-send").disabled, true);
});
