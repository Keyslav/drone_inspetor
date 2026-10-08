/* One visible camera, one connection. Signalling uses the authenticated HTTP API;
   video uses WebRTC. JPEG remains available even on gateways without aiortc. */
(function () {
  "use strict";
  class DroneVideo {
    constructor({ getContext, getRequest, notice }) {
      this.context = getContext;
      this.getRequest = getRequest;
      this.notice = notice;
      this.session = null;
      this.$ = (id) => document.getElementById(id);
      this.$("video-transport").addEventListener("change", () => {
        this.stop();
        this.tick();
      });
    }
    clear() {
      for (const kind of ["camera", "cv", "depth"]) {
        const image = this.$(`frame-${kind}`), video = this.$(`video-${kind}`);
        image.hidden = video.hidden = true;
        image.removeAttribute("src");
        video.srcObject = null;
        this.$(`frame-${kind}-note`).textContent = "Sem conexão · imagem indisponível";
      }
      this.$("expanded-frame").removeAttribute("src");
      this.$("expanded-frame").hidden = this.$("expanded-video").hidden = true;
      this.$("expanded-video").srcObject = null;
      this.$("expanded-note").textContent = "Sem conexão · imagem indisponível";
    }
    closePeer(session) {
      clearTimeout(session.timer);
      if (session.pc) {
        session.pc.ontrack = session.pc.onconnectionstatechange = null;
        session.pc.close();
        session.pc = null;
      }
      if (session.id) {
        const id = session.id;
        session.id = null;
        // Best effort: closing ICE also lets the server reclaim a lost client.
        session.request("/api/v1/video/close", {
          method: "POST", body: JSON.stringify({ session_id: id }), keepalive: true,
        }).catch(() => {});
      }
    }
    stop() {
      const old = this.session;
      this.session = null; // Late offers/JPEG replies may never restore an old view.
      if (old) {
        this.closePeer(old);
        if (old.url) URL.revokeObjectURL(old.url);
      }
      this.clear();
    }
    display(session, text, visible = false) {
      if (session !== this.session) return;
      session.note = text;
      this.$(`frame-${session.stream}-note`).textContent = text;
      this.$(`frame-${session.stream}`).hidden = !(visible && session.kind === "jpeg");
      this.$(`video-${session.stream}`).hidden = !(visible && session.kind === "webrtc");
      this.expanded();
    }
    expanded() {
      const session = this.session;
      const image = this.$("expanded-frame"), video = this.$("expanded-video");
      if (!session || !this.$("video-dialog").open) {
        video.srcObject = null;
        return;
      }
      const sourceImage = this.$(`frame-${session.stream}`);
      const sourceVideo = this.$(`video-${session.stream}`);
      image.hidden = sourceImage.hidden;
      if (session.url) image.src = session.url;
      else image.removeAttribute("src");
      video.hidden = sourceVideo.hidden;
      if (video.srcObject !== sourceVideo.srcObject) {
        video.srcObject = sourceVideo.srcObject;
        if (video.srcObject) video.play().catch(() => {});
      }
      this.$("expanded-note").textContent = session.note;
    }
    async startRTC(session) {
      try {
        const capabilities = await session.request("/api/v1/video");
        if (session !== this.session) return;
        if (!capabilities.webrtc) throw new Error(capabilities.reason || "Desativado na estação");
        if (!window.RTCPeerConnection) throw new Error("Não suportado neste navegador");
        const pc = new window.RTCPeerConnection({ iceServers: [] });
        session.pc = pc;
        pc.addTransceiver("video", { direction: "recvonly" });
        pc.ontrack = (event) => {
          if (session !== this.session || session.kind !== "webrtc") return;
          const video = this.$(`video-${session.stream}`);
          video.srcObject = event.streams[0] || new MediaStream([event.track]);
          video.play().catch(() => this.fail(session, new Error("Reprodução de vídeo bloqueada")));
          session.progressAt = performance.now();
          this.expanded();
        };
        pc.onconnectionstatechange = () => {
          if (pc.connectionState === "failed" || pc.connectionState === "closed")
            this.fail(session, new Error("Conexão de vídeo interrompida"));
        };
        session.timer = setTimeout(() => this.fail(session, new Error("Sem conexão WebRTC; verifique a rede UDP")), 12000);
        await pc.setLocalDescription(await pc.createOffer());
        if (session !== this.session) return;
        // Send the complete offer, including local ICE candidates (no trickle API).
        await new Promise((resolve, reject) => {
          if (pc.iceGatheringState === "complete") return resolve();
          const timer = setTimeout(() => { pc.removeEventListener("icegatheringstatechange", change); reject(new Error("Tempo de negociação esgotado")); }, 4000);
          const change = () => {
            if (pc.iceGatheringState === "complete") {
              clearTimeout(timer);
              pc.removeEventListener("icegatheringstatechange", change);
              resolve();
            }
          };
          pc.addEventListener("icegatheringstatechange", change);
        });
        if (session !== this.session || session.kind !== "webrtc") return;
        const answer = await session.request("/api/v1/video/offer", {
          method: "POST", body: JSON.stringify({ type: "offer", sdp: pc.localDescription.sdp, stream: session.stream }),
        });
        session.id = answer.session_id;
        if (session !== this.session || session.kind !== "webrtc") {
          this.closePeer(session);
          return;
        }
        await pc.setRemoteDescription({ type: answer.type, sdp: answer.sdp });
      } catch (error) { this.fail(session, error); }
    }
    fail(session, error) {
      if (session !== this.session || session.failed || session.kind !== "webrtc") return;
      this.closePeer(session);
      this.$(`video-${session.stream}`).srcObject = null;
      if (session.preference === "auto") {
        session.kind = "jpeg";
        session.fallback = true;
        this.notice(`Vídeo em JPEG: ${error.message}. Você pode tentar WebRTC no seletor de transmissão.`);
        this.display(session, "JPEG · conectando…");
        this.jpeg(session);
      } else {
        session.failed = true;
        this.display(session, `WebRTC indisponível: ${error.message}. Selecione JPEG ou Automático.`);
      }
    }
    async jpeg(session) {
      if (session.busy) return;
      session.busy = true;
      session.polledAt = performance.now();
      try {
        const blob = await session.request(`/api/v1/frame/${session.stream}`, { image: true });
        if (session !== this.session) return;
        if (!blob.type.startsWith("image/")) throw new Error("Resposta de imagem inválida");
        const previous = session.url;
        session.url = URL.createObjectURL(blob);
        this.$(`frame-${session.stream}`).src = session.url;
        this.display(session, session.fallback ? "JPEG · WebRTC indisponível" : "JPEG · recebendo", true);
        if (previous) URL.revokeObjectURL(previous);
      } catch (error) {
        this.display(session, error.status === 404 ? "JPEG · sem imagem recente" : "JPEG · imagem indisponível");
      } finally { session.busy = false; }
    }
    tick() {
      const context = this.context();
      if (!context.active) {
        if (this.session) this.stop();
        return;
      }
      const preference = this.$("video-transport").value;
      const key = `${context.stream}:${preference}`;
      if (!this.session || this.session.key !== key) {
        this.stop();
        const session = this.session = {
          key, stream: context.stream, preference, request: this.getRequest(),
          kind: preference === "jpeg" ? "jpeg" : "webrtc", polledAt: -Infinity,
          progressAt: performance.now(), currentTime: -1,
        };
        this.display(session, `${session.kind === "jpeg" ? "JPEG" : "WebRTC"} · conectando…`);
        if (session.kind === "webrtc") this.startRTC(session);
      }
      const session = this.session;
      if (session.kind === "jpeg") {
        if (performance.now() - session.polledAt >= 500) this.jpeg(session);
      } else if (!session.failed) {
        const video = this.$(`video-${session.stream}`);
        if (video.readyState >= 2 && video.currentTime !== session.currentTime) {
          session.currentTime = video.currentTime;
          session.progressAt = performance.now();
          clearTimeout(session.timer);
          session.playing = true;
        }
        if (session.playing) {
          if (performance.now() - session.progressAt > 3000)
            this.fail(session, new Error("Vídeo sem atualização"));
          else {
            const available = context.frames?.[session.stream]?.available === true;
            this.display(session, available ? "WebRTC · recebendo" : "WebRTC · sem imagem recente", available);
          }
        }
      }
    }
  }
  window.DroneVideo = DroneVideo;
})();
