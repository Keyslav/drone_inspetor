/* Propostas de IA são apenas dados. Confirmar usa o ID guardado no servidor. */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const statusLabel = (proposal) => {
    if (proposal.status === "proposed" && proposal.expires_in_s <= 0)
      return "expirada";
    return (
      {
        proposed: "aguarda revisão",
        observation: "observação",
        submitted: "enviada",
        completed: "concluída",
        rejected: "recusada",
        uncertain: "resultado incerto",
      }[proposal.status] || proposal.status
    );
  };
  window.DroneCopilot = class {
    constructor({
      request,
      canCommand,
      getNonce,
      consumeNonce,
      isConnected,
      showNotice,
    }) {
      Object.assign(this, {
        request,
        canCommand,
        getNonce,
        consumeNonce,
        isConnected,
        showNotice,
      });
      this.data = null;
      this.busy = false;
      this.refreshing = false;
      this.lastRefresh = 0;
      this.generation = 0;
      this.proposal = null;
      this.reviewedId = null;
      $("copilot-prepare").addEventListener("click", () => this.prepare());
      $("copilot-proposals").addEventListener("change", () => this.select());
      $("copilot-review").addEventListener("click", () => this.review());
      $("copilot-confirm-send").addEventListener("click", () => this.confirm());
      window.addEventListener("copilot:transcript", (event) => {
        if (typeof event.detail !== "string" || event.detail.length > 2000) {
          this.showNotice(
            "Transcrição longa demais; divida o pedido em uma ação.",
            true,
          );
          return;
        }
        $("copilot-prompt").value = event.detail;
        window.DroneLayout?.showTab("copilot-section");
        window.DroneLayout?.showCopilotPane("compose");
        this.showNotice(
          "Fala transcrita. Revise o texto e toque em Preparar proposta.",
        );
      });
    }
    update() {
      $("copilot-prepare").disabled =
        !this.isConnected() || !this.data?.enabled || this.busy;
      $("copilot-review").disabled =
        !this.proposal?.can_execute ||
        !this.canCommand(this.proposal.command) ||
        this.busy;
      $("copilot-confirm-send").disabled =
        !this.proposal?.can_execute ||
        !this.canCommand(this.proposal.command) ||
        this.busy;
      if (
        this.isConnected() &&
        !this.refreshing &&
        performance.now() - this.lastRefresh > 2500
      )
        this.refresh();
    }
    reset() {
      this.generation++;
      this.data = null;
      this.proposal = null;
      this.reviewedId = null;
      this.busy = false;
      this.refreshing = false;
      this.lastRefresh = 0;
      if ($("copilot-confirm-dialog").open) $("copilot-confirm-dialog").close();
      $("copilot-result").textContent =
        "Conecte ao gateway para consultar as propostas.";
      this.update();
    }
    async refresh() {
      if (this.refreshing) return;
      this.refreshing = true;
      const generation = this.generation;
      try {
        const data = await this.request("/api/v1/copilot");
        if (generation !== this.generation) return;
        this.data = data;
        $("copilot-mode").textContent = !data.enabled
          ? "Desativado no gateway"
          : data.mode === "shadow"
            ? "Observação · sem execução"
            : "Revisão antes de executar";
        const provider = $("copilot-provider").value;
        $("copilot-provider").replaceChildren();
        for (const item of data.providers || []) {
          const label =
            item.name === "demo"
              ? "Demonstração local (sem IA)"
              : `${item.name} · ${item.model || "modelo não configurado"}`;
          const option = new Option(
            label + (item.configured ? "" : " · configurar chave"),
            item.name,
          );
          option.disabled = !item.configured;
          $("copilot-provider").add(option);
        }
        if (data.providers?.some((item) => item.name === provider))
          $("copilot-provider").value = provider;
        const selected = $("copilot-proposals").value;
        $("copilot-proposals").replaceChildren(
          new Option("Selecione uma proposta", ""),
        );
        for (const proposal of data.proposals || [])
          $("copilot-proposals").add(
            new Option(
              `${proposal.source} · ${proposal.label} · ${statusLabel(proposal)}`,
              proposal.id,
            ),
          );
        if (data.proposals?.some((item) => item.id === selected))
          $("copilot-proposals").value = selected;
        else if (data.proposals?.length)
          $("copilot-proposals").value = data.proposals[0].id;
        this.select();
      } catch (error) {
        if (generation === this.generation) {
          this.data = null;
          this.proposal = null;
          if ($("copilot-confirm-dialog").open)
            $("copilot-confirm-dialog").close();
          $("copilot-mode").textContent = "Copiloto indisponível";
        }
      } finally {
        if (generation === this.generation) {
          this.refreshing = false;
          this.lastRefresh = performance.now();
          this.update();
        }
      }
    }
    select() {
      const previousId = this.proposal?.id;
      this.proposal =
        this.data?.proposals?.find(
          (item) => item.id === $("copilot-proposals").value,
        ) || null;
      const p = this.proposal;
      const resultText = p
        ? `${p.label}\n${p.explanation}\n${p.source} · ${p.model || "cliente MCP"} · ${statusLabel(p)} · validade ${Math.ceil(p.expires_in_s)} s${p.confidence == null ? "" : ` · confiança ${(p.confidence * 100).toFixed(1)}% (não é garantia de acerto)`}`
        : "Nenhuma proposta selecionada. Fale no app ou digite um pedido.";
      if (window.DroneLayout)
        window.DroneLayout.setText(
          "copilot-result",
          resultText,
          previousId !== p?.id,
        );
      else $("copilot-result").textContent = resultText;
      this.update();
    }
    async prepare() {
      if (!this.isConnected() || this.busy) return;
      const prompt = $("copilot-prompt").value.trim();
      if (!prompt) {
        this.showNotice("Digite ou fale um pedido primeiro.", true);
        return;
      }
      const generation = this.generation;
      this.busy = true;
      this.update();
      try {
        const result = await this.request("/api/v1/copilot/propose", {
          method: "POST",
          body: JSON.stringify({
            provider: $("copilot-provider").value,
            prompt,
          }),
        });
        if (generation !== this.generation) return;
        await this.refresh();
        $("copilot-proposals").value = result.id;
        this.select();
        window.DroneLayout?.showCopilotPane("review");
        this.showNotice("Proposta preparada. Nenhum comando foi executado.");
      } catch (error) {
        if (generation === this.generation)
          this.showNotice(`Não foi possível preparar: ${error.message}`, true);
      } finally {
        if (generation === this.generation) this.busy = false;
        this.update();
      }
    }
    review() {
      if (
        !this.proposal?.can_execute ||
        !this.canCommand(this.proposal.command)
      )
        return;
      this.reviewedId = this.proposal.id;
      $("copilot-confirm-description").textContent =
        `${this.proposal.label}. ${this.proposal.explanation} Confirma o envio desta ação ao controlador?`;
      $("copilot-confirm-dialog").showModal();
    }
    async confirm() {
      const proposal = this.proposal;
      if (
        !proposal ||
        proposal.id !== this.reviewedId ||
        !proposal.can_execute ||
        !this.canCommand(proposal.command) ||
        this.busy
      )
        return;
      $("copilot-confirm-dialog").close();
      const generation = this.generation;
      const nonce = this.getNonce();
      this.consumeNonce();
      this.busy = true;
      this.update();
      try {
        const result = await this.request("/api/v1/copilot/confirm", {
          method: "POST",
          body: JSON.stringify({
            proposal_id: proposal.id,
            nonce,
            confirmed: true,
          }),
        });
        if (generation !== this.generation) return;
        this.showNotice(
          `${result.status}: ${result.message}`,
          ["rejected", "uncertain"].includes(result.status),
        );
        await this.refresh();
      } catch (error) {
        if (generation === this.generation)
          this.showNotice(
            error.status >= 400 && error.status < 500
              ? error.message
              : "Resultado incerto. Confira o estado; nenhum comando será reenviado.",
            true,
          );
      } finally {
        if (generation === this.generation) this.busy = false;
        this.update();
      }
    }
  };
})();
