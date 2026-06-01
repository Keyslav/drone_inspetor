"""
mission_report.py
=================================================================================================
Gerador do relatório de missão (relatorio_da_missao.md).

Este módulo é INDEPENDENTE de ROS2: recebe eventos e medições do cv_node e mantém um
arquivo Markdown que é criado no início da missão e atualizado continuamente durante a
execução. O objetivo é registrar TUDO o que aconteceu na missão e, principalmente, as
taxas de quadros (FR) das imagens recebidas e dos modelos de visão computacional, de
forma a permitir a comparação do mesmo pacote rodando em diferentes plataformas
(Raspberry Pi 5, NVIDIA Jetson e PC) e a comparação entre modelos YOLO.

ESTRUTURA POR MARCO DE WAYPOINT:
A linha do tempo de desempenho é segmentada por MARCOS da missão (não por intervalo fixo
de tempo). Cada trecho vai de um marco ao seguinte:
    Decolagem  →  Ponto P01  →  Ponto P02  →  ...  →  RTL (retorno)
Para cada trecho são calculadas as médias de tempos e frequências e a média de
bounding boxes (Bbox) encontrados por frame em cada modelo.

LATÊNCIAS (medidas em TEMPO DE PAREDE real, time.perf_counter):
- Latência de detecção de objeto .... tempo do modelo de objetos por frame (ms).
- Latência de escaneamento de anomalia tempo do modelo de anomalias por frame (ms).
- Latência total de processamento ... SOMA das duas acima (ms).
- FR recebida ....................... imagens que chegam ao cv_node por segundo.
- FR processada ..................... frames efetivamente anotados/publicados por segundo.

IMPORTANTE — RELÓGIO DE PAREDE:
Todas as medições de tempo usam time.perf_counter()/time.time() (relógio de parede real),
NÃO o relógio do ROS. Como as simulações usam use_sim_time=True, o tempo do ROS pode não
corresponder ao tempo real, e o que interessa para comparar hardware é quantos frames por
segundo reais a máquina consegue processar.

THREAD-SAFE:
O cv_node usa um MultiThreadedExecutor, então os métodos públicos desta classe podem ser
chamados de múltiplas threads (callback de imagem, callback da FSM e callbacks de serviço).
Todo o estado é protegido por um Lock e nenhuma exceção interna deve se propagar para o nó
(o relatório nunca pode derrubar o pipeline de visão).
=================================================================================================
"""

import os
import socket
import platform
import threading
import time
from datetime import datetime


def _safe(fn, default="n/d"):
    """Executa fn() e devolve o valor; em qualquer erro devolve `default`."""
    try:
        v = fn()
        return v if v not in (None, "") else default
    except Exception:
        return default


def _fmt(value, nd=1):
    """Formata um número com `nd` casas decimais; devolve '—' para None/NaN."""
    try:
        if value is None:
            return "—"
        f = float(value)
        if f != f:  # NaN
            return "—"
        return f"{f:.{nd}f}"
    except Exception:
        return "—"


def _mmss(seconds):
    """Converte segundos em string mm:ss (ou hh:mm:ss se >= 1h)."""
    try:
        s = int(round(float(seconds)))
        if s < 0:
            s = 0
        h, rem = divmod(s, 3600)
        m, sec = divmod(rem, 60)
        if h > 0:
            return f"{h:02d}:{m:02d}:{sec:02d}"
        return f"{m:02d}:{sec:02d}"
    except Exception:
        return "00:00"


class _Accumulator:
    """Acumulador de latência (ms) e de bounding boxes para um modelo (toda a missão)."""

    __slots__ = ("count", "sum_ms", "min_ms", "max_ms", "bbox_sum")

    def __init__(self):
        self.count = 0
        self.sum_ms = 0.0
        self.min_ms = None
        self.max_ms = None
        self.bbox_sum = 0

    def add(self, ms, bbox=0):
        self.count += 1
        self.sum_ms += ms
        if self.min_ms is None or ms < self.min_ms:
            self.min_ms = ms
        if self.max_ms is None or ms > self.max_ms:
            self.max_ms = ms
        self.bbox_sum += int(bbox)

    @property
    def mean_ms(self):
        return (self.sum_ms / self.count) if self.count else None

    @property
    def mean_fps(self):
        m = self.mean_ms
        return (1000.0 / m) if (m and m > 0) else None

    @property
    def mean_bbox(self):
        return (self.bbox_sum / self.count) if self.count else None


class MissionReport:
    """
    Mantém o arquivo relatorio_da_missao.md de uma missão.

    Ciclo de vida:
        report = MissionReport(mission_folder, mission_name, device_label, available_models)
        report.start()                      # cria o .md e abre o 1º trecho
        report.mark_frame_received()        # a cada frame que entra no cv_node
        report.record_frame(...)            # a cada frame processado (tempos + nº de bbox)
        report.mark_waypoint(label)         # a cada marco (Decolagem, Ponto Pk, RTL)
        report.note_state_transition(...)   # mudanças de estado da FSM
        report.note_model_change(...)       # trocas de modelo
        report.note_photo(...)/note_video(...)/note_anomaly_moment(...)
        report.finish("CONCLUÍDA")          # fecha o relatório
    """

    def __init__(self, mission_folder, mission_name, device_label,
                 available_models, logger=None):
        self.mission_folder = mission_folder or ""
        self.report_path = os.path.join(self.mission_folder, "relatorio_da_missao.md")
        self.mission_name = mission_name or "(sem nome)"
        self.device_label_param = device_label or "auto"
        self._logger = logger

        # Metadados dos modelos disponíveis, indexados por nome de arquivo.
        self._models_meta = {}
        for m in (available_models or []):
            fn = m.get("file_name", "")
            if fn:
                self._models_meta[fn] = m

        # ---- Estado geral ----
        self._lock = threading.RLock()
        self._active = False
        self._status = "EM ANDAMENTO"
        self._start_dt = None
        self._t0_mono = None        # baseline monotônico (perf_counter) para tempos relativos
        self._end_elapsed = None    # elapsed congelado no finish()

        # ---- Hardware ----
        self._hw = {}
        self._device_label = self.device_label_param

        # ---- Modelos ----
        self._obj_model = ""
        self._anom_model = ""
        self._initial_obj_model = ""
        self._initial_anom_model = ""
        self._model_changes = []    # lista de dicts: {t, kind, old, new}

        # ---- Acumuladores POR MODELO (toda a missão) ----
        self._obj_stats = {}        # file -> _Accumulator (modelo de objetos)
        self._anom_stats = {}       # file -> _Accumulator (modelo de anomalias)

        # ---- Trecho (segmento) atual e trechos fechados ----
        self._seg = None            # segmento em andamento
        self._segments = []         # lista de dicts de métricas já calculadas

        # ---- Contadores globais ----
        self._total_received = 0
        self._total_processed = 0
        self._photos_cv = 0
        self._photos_anom = 0
        self._videos_cv = 0
        self._anomaly_moments = 0
        self._target_detections = 0
        self._points_total = 0
        self._points_reached = 0

        # ---- Linha do tempo de eventos (texto) ----
        self._events = []           # lista de dicts: {t, tag, msg}

    # =============================================================================================
    # CICLO DE VIDA
    # =============================================================================================

    def start(self):
        """Coleta o hardware, cria o arquivo .md e abre o primeiro trecho."""
        try:
            with self._lock:
                if self._active:
                    return
                # Coleta o hardware ANTES de zerar o cronômetro (a 1ª importação do torch
                # pode demorar; assim o tempo decorrido começa em 0 quando a missão começa).
                self._hw = self._gather_hardware_info()
                self._device_label = self._resolve_device_label()
                self._active = True
                self._status = "EM ANDAMENTO"
                self._start_dt = datetime.now()
                self._end_elapsed = None
                self._t0_mono = time.perf_counter()
                self._open_segment("Início")
                self._add_event("INÍCIO", f"Missão '{self.mission_name}' iniciada no dispositivo '{self._device_label}'.")
                self._render()
            self._log(f"Relatório da missão criado: {self.report_path}")
        except Exception as e:
            self._log(f"Erro ao iniciar relatório da missão: {e}", error=True)

    def finish(self, status="CONCLUÍDA"):
        """Fecha o trecho corrente e grava a versão final do relatório."""
        try:
            with self._lock:
                if not self._active:
                    return
                self._end_elapsed = self._elapsed()   # congela a duração total
                self._flush_segment()                  # fecha o último trecho (ex.: RTL)
                self._status = status
                self._add_event("FIM", f"Missão encerrada ({status}).")
                self._active = False
                self._render()
            self._log(f"Relatório da missão finalizado ({status}): {self.report_path}")
        except Exception as e:
            self._log(f"Erro ao finalizar relatório da missão: {e}", error=True)

    # =============================================================================================
    # MARCOS (delimitam os trechos da missão)
    # =============================================================================================

    def mark_waypoint(self, label):
        """
        Fecha o trecho atual e abre um novo trecho com o rótulo `label`.

        Chamado a cada marco da missão: 'Decolagem', 'Ponto P01', ..., 'RTL (retorno)'.
        """
        try:
            with self._lock:
                if not self._active:
                    return
                self._flush_segment()
                self._open_segment(label)
                self._add_event("MARCO", f"Início do trecho: {label}")
                self._render()
        except Exception:
            pass

    # =============================================================================================
    # MEDIÇÕES DE FRAME (chamadas pelo image_callback)
    # =============================================================================================

    def mark_frame_received(self):
        """Conta um frame que CHEGOU ao cv_node (antes do processamento)."""
        try:
            with self._lock:
                if not self._active:
                    return
                self._total_received += 1
                if self._seg is not None:
                    self._seg["recv"] += 1
        except Exception:
            pass

    def record_frame(self, obj_ms=None, anom_ms=None, n_obj_bboxes=0,
                     n_anom_bboxes=0, anomaly_active=False):
        """
        Registra um frame PROCESSADO com sucesso.

        Args:
            obj_ms:        tempo do modelo de objetos (ms), ou None se não rodou.
            anom_ms:       tempo somado do modelo de anomalias (ms), ou None se não rodou.
            n_obj_bboxes:  nº de bounding boxes de objetos detectados neste frame.
            n_anom_bboxes: nº de bounding boxes de anomalias detectados neste frame.
            anomaly_active: True se a detecção de anomalias estava habilitada neste frame.
        """
        try:
            with self._lock:
                if not self._active:
                    return
                self._total_processed += 1
                seg = self._seg
                if seg is not None:
                    seg["done"] += 1
                    if anomaly_active:
                        seg["anom_frames"] += 1

                if obj_ms is not None:
                    if seg is not None:
                        seg["obj_ms"] += float(obj_ms)
                        seg["obj_n"] += 1
                        seg["obj_bbox"] += int(n_obj_bboxes)
                    acc = self._obj_stats.get(self._obj_model)
                    if acc is None:
                        acc = self._obj_stats[self._obj_model] = _Accumulator()
                    acc.add(float(obj_ms), bbox=n_obj_bboxes)

                if anom_ms is not None and anom_ms > 0.0:
                    if seg is not None:
                        seg["anom_ms"] += float(anom_ms)
                        seg["anom_n"] += 1
                        seg["anom_bbox"] += int(n_anom_bboxes)
                    acc = self._anom_stats.get(self._anom_model)
                    if acc is None:
                        acc = self._anom_stats[self._anom_model] = _Accumulator()
                    acc.add(float(anom_ms), bbox=n_anom_bboxes)
        except Exception:
            pass

    # =============================================================================================
    # EVENTOS DA MISSÃO (chamados pelo cv_node)
    # =============================================================================================

    def set_initial_models(self, obj_model_file, anom_model_file):
        """Define os modelos ativos no início da missão (antes de qualquer troca)."""
        with self._lock:
            self._obj_model = obj_model_file or ""
            self._anom_model = anom_model_file or ""
            self._initial_obj_model = self._obj_model
            self._initial_anom_model = self._anom_model

    def set_mission_info(self, points_total=None):
        """Atualiza informações gerais da missão (ex.: total de pontos de inspeção)."""
        with self._lock:
            if points_total is not None:
                self._points_total = int(points_total)

    def note_state_transition(self, old_state, new_state, ponto_idx=None, objeto_alvo=""):
        """Registra uma mudança de estado da FSM na linha do tempo de eventos."""
        try:
            with self._lock:
                if not self._active:
                    return
                extra = ""
                if objeto_alvo:
                    extra = f" — alvo: {objeto_alvo}"
                if ponto_idx is not None and ponto_idx >= 0:
                    extra += f" (ponto P{ponto_idx + 1:02d})"
                self._add_event("ESTADO", f"{old_state} → {new_state}{extra}")
                self._render()
        except Exception:
            pass

    def note_point_reached(self, ponto_idx):
        """Registra a chegada a um ponto de inspeção."""
        with self._lock:
            self._points_reached = max(self._points_reached, int(ponto_idx) + 1)

    def note_model_change(self, kind, old_file, new_file):
        """
        Registra uma troca de modelo de visão computacional.

        Args:
            kind: "objetos" ou "anomalias".
            old_file / new_file: nomes de arquivo dos modelos.
        """
        try:
            with self._lock:
                if kind == "objetos":
                    self._obj_model = new_file or ""
                elif kind == "anomalias":
                    self._anom_model = new_file or ""

                old_name = self._model_display(old_file)
                new_name = self._model_display(new_file)
                self._model_changes.append({
                    "t": self._elapsed(), "kind": kind, "old": old_file, "new": new_file,
                })
                self._add_event("MODELO", f"Troca de modelo de {kind}: {old_name} → {new_name}")
                if self._active:
                    self._render()
        except Exception:
            pass

    def note_photo(self, kind="cv"):
        """Registra uma foto salva. kind: 'cv' (detecção) ou 'anomalia'."""
        try:
            with self._lock:
                if kind == "anomalia":
                    self._photos_anom += 1
                else:
                    self._photos_cv += 1
        except Exception:
            pass

    def note_anomaly_moment(self, object_name, n_anomalias, ponto_idx=None):
        """Registra um momento em que anomalias foram detectadas (e 5 fotos salvas)."""
        try:
            with self._lock:
                if not self._active:
                    return
                self._anomaly_moments += 1
                ponto = f"P{ponto_idx + 1:02d} " if (ponto_idx is not None and ponto_idx >= 0) else ""
                self._add_event("ANOMALIA", f"{ponto}{object_name}: {n_anomalias} anomalia(s) detectada(s) — 5 fotos salvas.")
                self._render()
        except Exception:
            pass

    def note_target_detected(self, object_name, confidence=None, ponto_idx=None):
        """Registra a detecção do objeto-alvo em um ponto de inspeção."""
        try:
            with self._lock:
                if not self._active:
                    return
                self._target_detections += 1
                ponto = f"P{ponto_idx + 1:02d} " if (ponto_idx is not None and ponto_idx >= 0) else ""
                conf = f" (conf. {confidence:.2f})" if confidence is not None else ""
                self._add_event("DETECÇÃO", f"{ponto}objeto-alvo '{object_name}' detectado{conf}.")
                self._render()
        except Exception:
            pass

    def note_video(self, path):
        """Registra um vídeo de detecção salvo."""
        try:
            with self._lock:
                if not self._active:
                    return
                self._videos_cv += 1
                self._add_event("VÍDEO", f"Vídeo de detecção salvo: {os.path.basename(path)}")
                self._render()
        except Exception:
            pass

    # =============================================================================================
    # TRECHOS (segmentos entre marcos)
    # =============================================================================================

    def _open_segment(self, label):
        """Abre um novo trecho (segmento) com o rótulo dado, zerando os acumuladores."""
        self._seg = {
            "label": label,
            "start": self._elapsed(),
            "recv": 0, "done": 0,
            "obj_ms": 0.0, "obj_n": 0, "obj_bbox": 0,
            "anom_ms": 0.0, "anom_n": 0, "anom_bbox": 0,
            "anom_frames": 0,
        }

    def _flush_segment(self):
        """Fecha o trecho atual: calcula médias e guarda como uma linha em _segments."""
        seg = self._seg
        if seg is None:
            return
        # Não cria linha para trecho sem nenhum frame (ex.: "Início" instantâneo).
        if seg["recv"] == 0 and seg["done"] == 0:
            self._seg = None
            return
        self._segments.append(self._segment_metrics(seg, partial=False))
        self._seg = None

    def _segment_metrics(self, seg, partial=False):
        """Calcula as métricas de um trecho (usado no fechamento e no trecho em andamento)."""
        now = self._elapsed()
        dur = max(now - seg["start"], 1e-6)
        obj_n = seg["obj_n"]
        anom_n = seg["anom_n"]

        lat_obj = (seg["obj_ms"] / obj_n) if obj_n else None
        lat_anom = (seg["anom_ms"] / anom_n) if anom_n else None
        parts = [x for x in (lat_obj, lat_anom) if x is not None]
        lat_total = sum(parts) if parts else None

        return {
            "label": seg["label"],
            "partial": partial,
            "dur": dur,
            "frames": seg["done"],
            "fr_recv": seg["recv"] / dur,
            "fr_proc": seg["done"] / dur,
            "lat_obj": lat_obj,
            "lat_anom": lat_anom,
            "lat_total": lat_total,
            "bbox_obj": (seg["obj_bbox"] / obj_n) if obj_n else None,
            "bbox_anom": (seg["anom_bbox"] / anom_n) if anom_n else None,
            "obj_model": self._model_short(self._obj_model),
            "anom_model": self._model_short(self._anom_model) if seg["anom_frames"] > 0 else "—",
        }

    def _elapsed(self):
        """Segundos de parede desde o início da missão."""
        if self._end_elapsed is not None:
            return self._end_elapsed
        if self._t0_mono is None:
            return 0.0
        return time.perf_counter() - self._t0_mono

    # =============================================================================================
    # HARDWARE
    # =============================================================================================

    def _gather_hardware_info(self):
        """Coleta informações de hardware/software úteis para comparar plataformas."""
        hw = {}
        hw["hostname"] = _safe(socket.gethostname)
        hw["board_model"] = self._read_board_model()
        hw["system"] = _safe(lambda: f"{platform.system()} {platform.release()}")
        hw["arch"] = _safe(platform.machine)
        hw["cpu_model"] = self._read_cpu_model()

        # Núcleos e frequência (psutil é opcional).
        cores_phys = cores_log = freq = ram = None
        try:
            import psutil
            cores_phys = psutil.cpu_count(logical=False)
            cores_log = psutil.cpu_count(logical=True)
            try:
                cf = psutil.cpu_freq()
                freq = cf.max if (cf and cf.max) else (cf.current if cf else None)
            except Exception:
                freq = None
            ram = psutil.virtual_memory().total
        except Exception:
            cores_log = _safe(lambda: os.cpu_count(), None)

        if cores_phys or cores_log:
            phys = cores_phys if cores_phys else "?"
            logi = cores_log if cores_log else "?"
            freq_txt = f" @ {freq/1000.0:.2f} GHz" if freq else ""
            hw["cpu_cores"] = f"{phys} núcleos / {logi} threads{freq_txt}"
        else:
            hw["cpu_cores"] = "n/d"
        hw["ram"] = f"{ram / (1024**3):.1f} GB" if ram else "n/d"

        # GPU / backend de inferência (torch é opcional).
        gpu = "n/d"
        backend = "n/d"
        torch_ver = "n/d"
        try:
            import torch
            torch_ver = torch.__version__
            if torch.cuda.is_available():
                name = torch.cuda.get_device_name(0)
                try:
                    vram = torch.cuda.get_device_properties(0).total_memory / (1024**3)
                    gpu = f"{name} ({vram:.1f} GB VRAM)"
                except Exception:
                    gpu = name
                backend = f"CUDA {getattr(torch.version, 'cuda', '?')}"
            else:
                gpu = "Sem CUDA (inferência na CPU)"
                backend = "CPU"
        except Exception:
            pass
        hw["gpu"] = gpu
        hw["backend"] = backend
        hw["torch"] = torch_ver
        hw["ultralytics"] = _safe(lambda: __import__("ultralytics").__version__)
        hw["opencv"] = _safe(lambda: __import__("cv2").__version__)
        hw["python"] = _safe(platform.python_version)
        return hw

    @staticmethod
    def _read_board_model():
        """Lê /proc/device-tree/model (presente em Raspberry Pi e Jetson)."""
        try:
            with open("/proc/device-tree/model", "rb") as f:
                return f.read().decode("utf-8", "ignore").replace("\x00", "").strip()
        except Exception:
            return ""

    @staticmethod
    def _read_cpu_model():
        """Extrai o modelo da CPU de /proc/cpuinfo (x86 e ARM)."""
        try:
            with open("/proc/cpuinfo", "r") as f:
                lines = f.readlines()
            for key in ("model name", "Model", "Hardware", "cpu model"):
                for ln in lines:
                    if ln.lower().startswith(key.lower()):
                        val = ln.split(":", 1)[1].strip()
                        if val:
                            return val
        except Exception:
            pass
        return _safe(platform.processor)

    def _resolve_device_label(self):
        """Resolve o rótulo do dispositivo (param 'auto' => deduz de placa/hostname)."""
        lbl = (self.device_label_param or "").strip()
        if lbl and lbl.lower() != "auto":
            return lbl
        board = self._hw.get("board_model", "") or ""
        low = board.lower()
        if "raspberry pi 5" in low:
            return "RPI5"
        if "raspberry pi 4" in low:
            return "RPI4"
        if "raspberry pi" in low:
            return "RPi"
        if "jetson" in low or "tegra" in low or "orin" in low or "xavier" in low:
            return board or "Jetson"
        host = self._hw.get("hostname", "") or "PC"
        return f"PC ({host})" if host not in ("", "n/d") else "PC"

    # =============================================================================================
    # AJUDA DE MODELOS
    # =============================================================================================

    def _model_display(self, file_name):
        """Nome legível do modelo (nome do models.json + arquivo)."""
        if not file_name:
            return "—"
        meta = self._models_meta.get(file_name)
        if meta:
            return f"{meta.get('name', file_name)} (`{file_name}`)"
        return f"`{file_name}`"

    def _model_short(self, file_name):
        """Identificador curto do modelo para as tabelas."""
        if not file_name:
            return "—"
        meta = self._models_meta.get(file_name)
        if meta and meta.get("model"):
            return meta["model"]
        return file_name.replace(".pt", "")

    # =============================================================================================
    # RENDERIZAÇÃO DO MARKDOWN
    # =============================================================================================

    def _add_event(self, tag, msg):
        self._events.append({"t": self._elapsed(), "tag": tag, "msg": msg})

    def _render(self):
        """Re-renderiza TODO o arquivo .md a partir do estado em memória (escrita atômica)."""
        if not self.mission_folder:
            return
        try:
            content = self._build_markdown()
            tmp = self.report_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(content)
            os.replace(tmp, self.report_path)
        except Exception as e:
            self._log(f"Erro ao gravar relatório: {e}", error=True)

    def _build_markdown(self):
        L = []
        a = L.append

        # -------- Cabeçalho --------
        a(f"# Relatório da Missão — {self.mission_name}")
        a("")
        a("> Documento gerado automaticamente pelo `cv_node` do pacote **drone_inspetor**.")
        a("> Criado no início da missão e preenchido continuamente durante a execução.")
        a("> Tempos e taxas de quadros (FR) usam **tempo de parede real** "
          "(independente de `use_sim_time`), para permitir a comparação entre plataformas.")
        a("")

        # -------- 1. Identificação --------
        start_txt = self._start_dt.strftime("%d/%m/%Y %H:%M:%S") if self._start_dt else "n/d"
        a("## 1. Identificação")
        a("")
        a("| Campo | Valor |")
        a("|---|---|")
        a(f"| Missão | {self.mission_name} |")
        a(f"| Dispositivo | **{self._device_label}** |")
        a(f"| Status | **{self._status}** |")
        a(f"| Início | {start_txt} |")
        a(f"| Duração | {_mmss(self._elapsed())} |")
        a(f"| Pasta da missão | `{self.mission_folder}` |")
        a("")

        # -------- 2. Hardware --------
        hw = self._hw
        a("## 2. Plataforma de Hardware")
        a("")
        a("_Seção dedicada à comparação RPI5 × Jetson × PC._")
        a("")
        a("| Componente | Detalhe |")
        a("|---|---|")
        a(f"| Hostname | {hw.get('hostname', 'n/d')} |")
        if hw.get("board_model"):
            a(f"| Placa (device-tree) | {hw.get('board_model')} |")
        a(f"| Sistema | {hw.get('system', 'n/d')} |")
        a(f"| Arquitetura | {hw.get('arch', 'n/d')} |")
        a(f"| CPU | {hw.get('cpu_model', 'n/d')} |")
        a(f"| Núcleos | {hw.get('cpu_cores', 'n/d')} |")
        a(f"| Memória RAM | {hw.get('ram', 'n/d')} |")
        a(f"| Acelerador (GPU) | {hw.get('gpu', 'n/d')} |")
        a(f"| Backend de inferência | **{hw.get('backend', 'n/d')}** |")
        a(f"| PyTorch | {hw.get('torch', 'n/d')} |")
        a(f"| Ultralytics | {hw.get('ultralytics', 'n/d')} |")
        a(f"| OpenCV | {hw.get('opencv', 'n/d')} |")
        a(f"| Python | {hw.get('python', 'n/d')} |")
        a("")

        # -------- 3. Modelos --------
        a("## 3. Modelos de Visão Computacional")
        a("")
        a("### Modelos ativos no início")
        a("")
        a("| Função | Modelo | Tipo | Dataset | Classes | Arquivo |")
        a("|---|---|---|---|---|---|")
        a(self._model_row("Objetos/Equipamentos", self._initial_obj_model))
        a(self._model_row("Anomalias", self._initial_anom_model))
        a("")
        a("### Trocas de modelo durante a missão")
        a("")
        if self._model_changes:
            a("| Tempo | Função | De | Para |")
            a("|---|---|---|---|")
            for ch in self._model_changes:
                a(f"| {_mmss(ch['t'])} | {ch['kind']} | {self._model_display(ch['old'])} | {self._model_display(ch['new'])} |")
        else:
            a("_Nenhuma troca de modelo registrada — modelos mantidos durante toda a missão._")
        a("")

        # -------- 4. Marcos da missão (por waypoint) --------
        a("## 4. Desempenho por marco da missão (waypoints)")
        a("")
        a("Cada linha resume um trecho entre marcos: **Decolagem → Ponto P01 → ... → "
          "RTL (retorno)**. Tempos e FR são médias do trecho (tempo de parede real).")
        a("")
        a("- **Lat. obj.** — latência média de detecção de objeto (modelo de objetos).")
        a("- **Lat. anom.** — latência média de escaneamento de anomalia (modelo de anomalias).")
        a("- **Lat. total** — latência total de processamento = **Lat. obj. + Lat. anom.**")
        a("- **FR rec. / FR proc.** — frequência de imagens recebidas / frames processados.")
        a("- **Bbox obj./fr** e **Bbox anom./fr** — média de bounding boxes por frame em cada modelo.")
        a("")
        a("| Marco | Duração | Frames | FR rec. (Hz) | FR proc. (Hz) | Lat. obj. (ms) | "
          "Lat. anom. (ms) | Lat. total (ms) | Bbox obj./fr | Bbox anom./fr | Mod. obj. | Mod. anom. |")
        a("|---|---|---|---|---|---|---|---|---|---|---|---|")

        rows = list(self._segments)
        # Inclui o trecho em andamento (se tiver frames) como linha provisória.
        if self._seg is not None and (self._seg["recv"] > 0 or self._seg["done"] > 0):
            rows.append(self._segment_metrics(self._seg, partial=True))

        if rows:
            for r in rows:
                a(self._segment_row(r))
        else:
            a("| _(aguardando primeiro trecho com frames...)_ | | | | | | | | | | | |")
        a("")

        # -------- 5. Comparação por modelo --------
        a("## 5. Comparação de desempenho por modelo YOLO")
        a("")
        a("Estatísticas agregadas de toda a missão, por modelo efetivamente utilizado neste "
          "hardware (inclui a média de bounding boxes por frame). Útil para comparar "
          "**YOLOv8n × YOLOv8x** e o mesmo modelo entre **RPI5 × Jetson × PC**.")
        a("")
        a("### Modelos de objetos")
        a("")
        a(self._stats_table(self._obj_stats, count_label="Frames", bbox_label="Bbox méd./frame"))
        a("")
        a("### Modelos de anomalias")
        a("")
        a(self._stats_table(self._anom_stats, count_label="Execuções", bbox_label="Bbox méd./exec."))
        a("")

        # -------- 6. Resumo --------
        a("## 6. Resumo de detecções e mídia")
        a("")
        a("| Item | Quantidade |")
        a("|---|---|")
        a(f"| Frames recebidos (total) | {self._total_received} |")
        a(f"| Frames processados (total) | {self._total_processed} |")
        a(f"| FR média da missão (processada) | {_fmt(self._overall_fps())} Hz |")
        if self._points_total:
            a(f"| Pontos de inspeção | {self._points_reached}/{self._points_total} |")
        a(f"| Detecções de objeto-alvo | {self._target_detections} |")
        a(f"| Momentos com anomalias | {self._anomaly_moments} |")
        a(f"| Fotos de detecção (CV) salvas | {self._photos_cv} |")
        a(f"| Fotos de anomalias salvas | {self._photos_anom} |")
        a(f"| Vídeos de detecção salvos | {self._videos_cv} |")
        a("")

        # -------- 7. Eventos --------
        a("## 7. Linha do tempo de eventos da missão")
        a("")
        if self._events:
            for ev in self._events:
                a(f"- `{_mmss(ev['t'])}` **[{ev['tag']}]** {ev['msg']}")
        else:
            a("_Nenhum evento registrado ainda._")
        a("")
        a("---")
        a(f"_Última atualização: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')} "
          f"(tempo decorrido {_mmss(self._elapsed())})._")
        a("")
        return "\n".join(L)

    def _segment_row(self, r):
        label = r["label"] + (" *(em andamento)*" if r.get("partial") else "")
        return "| {lbl} | {dur} | {fr} | {recv} | {proc} | {lo} | {la} | {lt} | {bo} | {ba} | {om} | {am} |".format(
            lbl=label,
            dur=_mmss(r["dur"]),
            fr=r["frames"],
            recv=_fmt(r["fr_recv"]),
            proc=_fmt(r["fr_proc"]),
            lo=_fmt(r["lat_obj"]),
            la=_fmt(r["lat_anom"]),
            lt=_fmt(r["lat_total"]),
            bo=_fmt(r["bbox_obj"], nd=2),
            ba=_fmt(r["bbox_anom"], nd=2),
            om=r["obj_model"],
            am=r["anom_model"],
        )

    def _model_row(self, funcao, file_name):
        meta = self._models_meta.get(file_name, {})
        model = meta.get("model", "—")
        tipo = meta.get("type", "—")
        dataset = meta.get("dataset", "—")
        classes = meta.get("classes", [])
        n_classes = f"{len(classes)} ({', '.join(classes)})" if classes else "—"
        arq = f"`{file_name}`" if file_name else "—"
        return f"| {funcao} | {model} | {tipo} | {dataset} | {n_classes} | {arq} |"

    def _stats_table(self, stats, count_label="Frames", bbox_label="Bbox méd./frame"):
        if not stats:
            return "_Nenhum modelo utilizado nesta categoria durante a missão._"
        lines = [
            f"| Modelo | Arquivo | {count_label} | Latência média (ms) | "
            f"Mín (ms) | Máx (ms) | FR média (Hz) | {bbox_label} |",
            "|---|---|---|---|---|---|---|---|",
        ]
        for file_name, acc in stats.items():
            meta = self._models_meta.get(file_name, {})
            model = meta.get("model", file_name.replace(".pt", "") if file_name else "—")
            lines.append(
                f"| {model} | `{file_name}` | {acc.count} | {_fmt(acc.mean_ms)} | "
                f"{_fmt(acc.min_ms)} | {_fmt(acc.max_ms)} | {_fmt(acc.mean_fps)} | "
                f"{_fmt(acc.mean_bbox, nd=2)} |"
            )
        return "\n".join(lines)

    def _overall_fps(self):
        el = self._elapsed()
        return (self._total_processed / el) if el > 0 else None

    # =============================================================================================
    # LOG
    # =============================================================================================

    def _log(self, msg, error=False):
        if self._logger is not None:
            try:
                (self._logger.error if error else self._logger.info)(msg)
                return
            except Exception:
                pass
