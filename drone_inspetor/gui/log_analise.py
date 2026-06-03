"""
log_analise.py
=================================================================================================
Janela de análise de logs de missões.

Exibe lista de missões salvas à esquerda e relatório detalhado à direita com:
- Informações básicas da missão (data/hora início e fim)
- Miniaturas de fotos organizadas por ponto de inspeção
- Links para vídeos
=================================================================================================
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QMainWindow, 
    QTextBrowser, QListWidget, QListWidgetItem, QSplitter,
    QPushButton, QFrame, QScrollArea, QGridLayout, QMessageBox
)
from PyQt6.QtGui import QPixmap, QCursor, QIcon, QImage
from PyQt6.QtCore import Qt, QSize
from datetime import datetime
from typing import Optional, List, Dict
import os
import yaml
import re
import subprocess


def _get_missions_directory() -> str:
    """
    Lê o diretório de missões do param_gui.yaml.
    Retorna o caminho expandido (resolve ~).
    """
    # Caminho relativo ao próprio arquivo do projeto
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
        "config", "param_gui.yaml"
    )
    
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                params = yaml.safe_load(f)
                missions_dir = params.get('missions_directory', '~/Drone_Inspetor_Missoes')
                return os.path.expanduser(missions_dir)
        except Exception:
            pass
    
    # Fallback
    return os.path.expanduser("~/Drone_Inspetor_Missoes")


def list_missions() -> List[Dict]:
    """
    Lista todas as missões salvas no diretório de missões.
    Retorna lista de dicionários com informações de cada missão.
    """
    missions = []
    missions_dir = _get_missions_directory()
    
    if not os.path.exists(missions_dir):
        return missions
    
    for folder in sorted(os.listdir(missions_dir), reverse=True):
        folder_path = os.path.join(missions_dir, folder)
        
        if not os.path.isdir(folder_path):
            continue
        
        # Extrai data/hora do nome da pasta (mission_YYYYMMDD_HHMMSS)
        match = re.match(r'mission_(\d{8})_(\d{6})', folder)
        if match:
            date_str = match.group(1)
            time_str = match.group(2)
            
            try:
                start_dt = datetime.strptime(f"{date_str}_{time_str}", "%Y%m%d_%H%M%S")
            except:
                start_dt = None
        else:
            start_dt = None
        
        # Conta arquivos de mídia
        fotos_count = 0
        fotos_cv_count = 0
        videos_count = 0
        
        fotos_path = os.path.join(folder_path, "fotos")
        if os.path.exists(fotos_path):
            fotos_count = len([f for f in os.listdir(fotos_path) if f.endswith('.jpg')])
        
        fotos_cv_path = os.path.join(folder_path, "fotos_cv")
        if os.path.exists(fotos_cv_path):
            fotos_cv_count = len([f for f in os.listdir(fotos_cv_path) if f.endswith('.jpg')])
        
        videos_path = os.path.join(folder_path, "videos")
        if os.path.exists(videos_path):
            videos_count = len([f for f in os.listdir(videos_path) if f.endswith(('.mp4', '.avi'))])
        
        videos_cv_path = os.path.join(folder_path, "videos_cv")
        if os.path.exists(videos_cv_path):
            videos_count += len([f for f in os.listdir(videos_cv_path) if f.endswith(('.mp4', '.avi'))])
        
        # Determina hora de fim baseado no último arquivo modificado
        end_time = None
        try:
            all_files = []
            for root, dirs, files in os.walk(folder_path):
                for f in files:
                    all_files.append(os.path.join(root, f))
            if all_files:
                latest_file = max(all_files, key=os.path.getmtime)
                end_time = datetime.fromtimestamp(os.path.getmtime(latest_file))
        except:
            pass
        
        missions.append({
            'folder': folder,
            'path': folder_path,
            'start_time': start_dt,
            'end_time': end_time,
            'fotos_count': fotos_count,
            'fotos_cv_count': fotos_cv_count,
            'videos_count': videos_count
        })
    
    return missions


# =============================================================================
# CLASSE PHOTOWINDOW
# =============================================================================

class PhotoWindow(QMainWindow):
    """Janela para exibir fotos individuais em tamanho grande."""
    
    def __init__(self, photo_path: str, parent=None):
        super().__init__(parent)
        self.photo_path = photo_path
        photo_name = os.path.basename(photo_path)
        self.setWindowTitle(f"📷 {photo_name}")
        self.setGeometry(200, 200, 1000, 750)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #1a1a2e; }
            QLabel { color: #ecf0f1; }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Info do arquivo
        info_label = QLabel()
        info_label.setStyleSheet("font-size: 12px; color: #7f8c8d; padding: 5px;")
        if os.path.exists(photo_path):
            size = os.path.getsize(photo_path)
            mtime = datetime.fromtimestamp(os.path.getmtime(photo_path))
            info_label.setText(f"📁 {photo_name}  |  📏 {size/1024:.1f} KB  |  🕐 {mtime.strftime('%d/%m/%Y %H:%M:%S')}")
        layout.addWidget(info_label)
        
        # Foto
        photo_label = QLabel()
        photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        photo_label.setStyleSheet("""
            background-color: #16213e;
            border: 2px solid #0f3460;
            border-radius: 8px;
        """)
        
        if os.path.exists(photo_path):
            pixmap = QPixmap(photo_path)
            if not pixmap.isNull():
                scaled = pixmap.scaled(
                    980, 680, 
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation
                )
                photo_label.setPixmap(scaled)
            else:
                photo_label.setText(f"⚠️ Erro ao carregar imagem")
        else:
            photo_label.setText(f"❌ Arquivo não encontrado")
        
        layout.addWidget(photo_label)
        
        # Botão fechar
        close_btn = QPushButton("✕ Fechar (duplo clique)")
        close_btn.setStyleSheet("""
            QPushButton { background-color: #e74c3c; color: white; padding: 8px; 
                         border-radius: 5px; font-size: 12px; }
            QPushButton:hover { background-color: #c0392b; }
        """)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)
        
        central_widget.setLayout(layout)
    
    def mouseDoubleClickEvent(self, event):
        self.close()


# =============================================================================
# FUNÇÃO PARA ABRIR VÍDEO COM PLAYER DO SISTEMA
# =============================================================================

def open_video_with_system_player(video_path: str, parent=None):
    """
    Abre vídeo com o player padrão do sistema.
    Usa xdg-open no Linux.
    """
    if not os.path.exists(video_path):
        if parent:
            QMessageBox.warning(parent, "Erro", f"Arquivo não encontrado:\n{video_path}")
        return False
    
    try:
        subprocess.Popen(['xdg-open', video_path], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        if parent:
            QMessageBox.warning(parent, "Erro", f"Erro ao abrir vídeo:\n{e}")
        return False


# =============================================================================
# RELATÓRIO DA MISSÃO (.md) — LEITURA, PARSING E GRÁFICOS
# =============================================================================

REPORT_MD_NAME = "relatorio_da_missao.md"


def _md_num(text: Optional[str]):
    """Converte uma célula da tabela Markdown em float; '—'/vazio viram None."""
    if text is None:
        return None
    s = text.strip().replace("*", "").replace("`", "")
    if s in ("—", "-", ""):
        return None
    # Mantém apenas dígitos, sinal e separador decimal (remove unidades coladas).
    cleaned = "".join(ch for ch in s if ch.isdigit() or ch in ".-+")
    try:
        return float(cleaned)
    except ValueError:
        return None


def _parse_md_table_after(md_text: str, heading: str):
    """
    Extrai a primeira tabela Markdown que aparece após uma linha de cabeçalho.

    Args:
        md_text: conteúdo completo do .md.
        heading: texto do cabeçalho (ex.: '## 4.' ou '### Modelos de objetos').

    Returns:
        (header: List[str], rows: List[List[str]]) ou None se não encontrar.
    """
    lines = md_text.splitlines()
    start = None
    for i, ln in enumerate(lines):
        if ln.strip().startswith(heading):
            start = i + 1
            break
    if start is None:
        return None

    # Avança até a primeira linha de tabela, parando se topar outro cabeçalho.
    i = start
    while i < len(lines) and not lines[i].lstrip().startswith("|"):
        if lines[i].lstrip().startswith("#"):
            return None
        i += 1

    table = []
    while i < len(lines) and lines[i].lstrip().startswith("|"):
        cells = [c.strip() for c in lines[i].strip().strip("|").split("|")]
        # Ignora a linha separadora (|---|---|).
        if not all(set(c) <= set("-: ") and c != "" for c in cells):
            table.append(cells)
        i += 1

    if len(table) < 2:
        return None
    return table[0], table[1:]


def _fig_to_pixmap(fig) -> QPixmap:
    """Renderiza uma figura matplotlib (backend Agg) para um QPixmap."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    w, h = canvas.get_width_height()
    image = QImage(bytes(canvas.buffer_rgba()), w, h, QImage.Format.Format_RGBA8888)
    return QPixmap.fromImage(image.copy())


# Paleta alinhada ao tema escuro da janela.
_CHART_BG = "#1a1a2e"
_CHART_FG = "#ecf0f1"
_CHART_GRID = "#2c3e6b"
_CHART_COLORS = ["#e94560", "#00d9ff", "#f5a623", "#7ed957", "#a78bfa"]


def _style_axes(ax, title: str):
    """Aplica o tema escuro a um eixo matplotlib."""
    ax.set_title(title, color=_CHART_FG, fontsize=11, fontweight="bold", pad=10)
    ax.set_facecolor(_CHART_BG)
    ax.tick_params(colors=_CHART_FG, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(_CHART_GRID)
    ax.grid(axis="y", color=_CHART_GRID, linewidth=0.6, alpha=0.6)
    ax.set_axisbelow(True)


def build_report_charts(md_text: str) -> List[QPixmap]:
    """
    Gera gráficos analíticos a partir das tabelas do relatório .md.

    - Por marco (seção 4): FR recebida × processada e latências (obj/anom/total).
    - Por modelo (seção 5): latência média e FR média comparadas entre modelos.

    Retorna lista de QPixmap (vazia se matplotlib indisponível ou sem dados).
    """
    try:
        from matplotlib.figure import Figure
    except Exception:
        return []

    pixmaps: List[QPixmap] = []

    # ---- Seção 4: desempenho por marco ----
    seg = _parse_md_table_after(md_text, "## 4.")
    if seg:
        _, rows = seg
        labels, fr_rec, fr_proc = [], [], []
        lat_obj, lat_anom, lat_total = [], [], []
        for r in rows:
            if len(r) < 8:
                continue
            labels.append(r[0].replace("*(em andamento)*", "").strip())
            fr_rec.append(_md_num(r[3]) or 0.0)
            fr_proc.append(_md_num(r[4]) or 0.0)
            lat_obj.append(_md_num(r[5]) or 0.0)
            lat_anom.append(_md_num(r[6]) or 0.0)
            lat_total.append(_md_num(r[7]) or 0.0)

        if labels:
            x = range(len(labels))

            # Gráfico FR rec × proc
            fig = Figure(figsize=(7.6, 3.2), dpi=100, facecolor=_CHART_BG)
            ax = fig.add_subplot(111)
            _style_axes(ax, "Frequência de quadros por marco (Hz)")
            width = 0.4
            ax.bar([i - width / 2 for i in x], fr_rec, width, label="FR recebida", color=_CHART_COLORS[1])
            ax.bar([i + width / 2 for i in x], fr_proc, width, label="FR processada", color=_CHART_COLORS[0])
            ax.set_xticks(list(x))
            ax.set_xticklabels(labels, rotation=30, ha="right")
            ax.set_ylabel("Hz", color=_CHART_FG, fontsize=9)
            leg = ax.legend(facecolor=_CHART_BG, edgecolor=_CHART_GRID, labelcolor=_CHART_FG, fontsize=8)
            fig.tight_layout()
            pixmaps.append(_fig_to_pixmap(fig))

            # Gráfico de latências por marco (apenas se houver algum valor)
            if any(lat_total) or any(lat_obj) or any(lat_anom):
                fig = Figure(figsize=(7.6, 3.2), dpi=100, facecolor=_CHART_BG)
                ax = fig.add_subplot(111)
                _style_axes(ax, "Latência de processamento por marco (ms)")
                w3 = 0.27
                ax.bar([i - w3 for i in x], lat_obj, w3, label="Lat. objeto", color=_CHART_COLORS[2])
                ax.bar(list(x), lat_anom, w3, label="Lat. anomalia", color=_CHART_COLORS[3])
                ax.bar([i + w3 for i in x], lat_total, w3, label="Lat. total", color=_CHART_COLORS[0])
                ax.set_xticks(list(x))
                ax.set_xticklabels(labels, rotation=30, ha="right")
                ax.set_ylabel("ms", color=_CHART_FG, fontsize=9)
                ax.legend(facecolor=_CHART_BG, edgecolor=_CHART_GRID, labelcolor=_CHART_FG, fontsize=8)
                fig.tight_layout()
                pixmaps.append(_fig_to_pixmap(fig))

    # ---- Seção 5: comparação por modelo ----
    model_names, model_lat, model_fps = [], [], []
    for sub in ("### Modelos de objetos", "### Modelos de anomalias"):
        tbl = _parse_md_table_after(md_text, sub)
        if not tbl:
            continue
        _, rows = tbl
        kind = "obj" if "objetos" in sub else "anom"
        for r in rows:
            if len(r) < 7:
                continue
            name = r[0].replace("`", "").strip()
            lat = _md_num(r[3])
            fps = _md_num(r[6])
            if name and lat is not None:
                model_names.append(f"{name}\n({kind})")
                model_lat.append(lat)
                model_fps.append(fps or 0.0)

    if model_names:
        fig = Figure(figsize=(7.6, 3.4), dpi=100, facecolor=_CHART_BG)
        ax1 = fig.add_subplot(111)
        _style_axes(ax1, "Comparação por modelo YOLO — latência × FR")
        xm = range(len(model_names))
        bars = ax1.bar(list(xm), model_lat, 0.5, color=_CHART_COLORS[0], label="Latência média (ms)")
        ax1.set_ylabel("ms", color=_CHART_COLORS[0], fontsize=9)
        ax1.set_xticks(list(xm))
        ax1.set_xticklabels(model_names, fontsize=8)
        for b, v in zip(bars, model_lat):
            ax1.text(b.get_x() + b.get_width() / 2, b.get_height(), f"{v:.0f}",
                     ha="center", va="bottom", color=_CHART_FG, fontsize=8)

        ax2 = ax1.twinx()
        ax2.plot(list(xm), model_fps, "o-", color=_CHART_COLORS[1], label="FR média (Hz)", linewidth=2)
        ax2.set_ylabel("Hz", color=_CHART_COLORS[1], fontsize=9)
        ax2.tick_params(colors=_CHART_FG, labelsize=8)
        for spine in ax2.spines.values():
            spine.set_color(_CHART_GRID)
        fig.tight_layout()
        pixmaps.append(_fig_to_pixmap(fig))

    return pixmaps


class _AutoTextBrowser(QTextBrowser):
    """QTextBrowser que cresce para caber todo o conteúdo (sem rolagem própria),
    para ser embutido dentro da QScrollArea do relatório."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setOpenExternalLinks(True)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.document().contentsChanged.connect(self._adjust_height)

    def _adjust_height(self):
        self.document().setTextWidth(self.viewport().width())
        height = self.document().size().height()
        self.setFixedHeight(int(height) + 12)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._adjust_height()


# =============================================================================
# CLASSE LOGANALYSISWINDOW
# =============================================================================

class LogAnalysisWindow(QMainWindow):
    """Janela para exibir logs de análise das missões gravadas."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🔍 Drone Inspetor - Log de Análise")
        self.setGeometry(100, 100, 1400, 900)
        self.parent_dashboard = parent
        self.media_windows = []  # Janelas de foto/vídeo abertas
        self.current_mission_path = None
        
        self.setStyleSheet("""
            QMainWindow { background-color: #1a1a2e; color: #ecf0f1; }
        """)
        
        self._setup_ui()
        self._load_missions_list()
    
    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # === Painel Esquerdo: Lista de Missões ===
        left_panel = QFrame()
        left_panel.setStyleSheet("""
            QFrame { background-color: #16213e; border-radius: 10px; }
        """)
        left_panel.setFixedWidth(320)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)
        
        missions_title = QLabel("📋 Missões Gravadas")
        missions_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #e94560; padding: 10px;")
        left_layout.addWidget(missions_title)
        
        # Caminho
        path_label = QLabel(f"📁 {_get_missions_directory()}")
        path_label.setStyleSheet("font-size: 10px; color: #7f8c8d; padding: 5px;")
        path_label.setWordWrap(True)
        left_layout.addWidget(path_label)
        
        self.missions_list = QListWidget()
        self.missions_list.setStyleSheet("""
            QListWidget { 
                background-color: #0f3460; 
                border: none; 
                border-radius: 8px;
                color: #ecf0f1; 
                font-size: 13px; 
            }
            QListWidget::item { 
                padding: 12px; 
                border-bottom: 1px solid #1a1a2e; 
            }
            QListWidget::item:selected { background-color: #e94560; }
            QListWidget::item:hover { background-color: #533483; }
        """)
        self.missions_list.itemClicked.connect(self._on_mission_selected)
        left_layout.addWidget(self.missions_list)
        
        refresh_btn = QPushButton("🔄 Atualizar Lista")
        refresh_btn.setStyleSheet("""
            QPushButton { 
                background-color: #e94560; 
                color: white; 
                padding: 12px; 
                border-radius: 8px; 
                font-size: 14px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #ff6b6b; }
        """)
        refresh_btn.clicked.connect(self._load_missions_list)
        left_layout.addWidget(refresh_btn)
        
        main_layout.addWidget(left_panel)
        
        # === Painel Direito: Relatório da Missão ===
        right_panel = QFrame()
        right_panel.setStyleSheet("""
            QFrame { background-color: #16213e; border-radius: 10px; }
        """)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        details_title = QLabel("📊 Relatório da Missão")
        details_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #e94560; padding: 10px;")
        right_layout.addWidget(details_title)
        
        # Área de scroll para o relatório
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("""
            QScrollArea { 
                background-color: #0f3460; 
                border: none; 
                border-radius: 8px;
            }
            QScrollBar:vertical {
                background: #16213e;
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background: #533483;
                border-radius: 6px;
            }
        """)
        
        self.report_widget = QWidget()
        self.report_layout = QVBoxLayout(self.report_widget)
        self.report_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        scroll_area.setWidget(self.report_widget)
        
        right_layout.addWidget(scroll_area)
        main_layout.addWidget(right_panel)
        
        central_widget.setLayout(main_layout)
    
    def _load_missions_list(self):
        """Carrega lista de missões do diretório parametrizado."""
        self.missions_list.clear()
        missions = list_missions()
        
        if not missions:
            item = QListWidgetItem("(Nenhuma missão encontrada)")
            item.setData(Qt.ItemDataRole.UserRole, None)
            self.missions_list.addItem(item)
            return
        
        for mission in missions:
            start = mission.get('start_time')
            if start:
                date_str = start.strftime("%d/%m/%Y")
                time_str = start.strftime("%H:%M:%S")
            else:
                date_str = "Data desconhecida"
                time_str = ""
            
            fotos = mission.get('fotos_count', 0) + mission.get('fotos_cv_count', 0)
            videos = mission.get('videos_count', 0)
            
            text = f"🗓️ {date_str}  🕐 {time_str}\n📷 {fotos} fotos  |  🎬 {videos} vídeos"
            item = QListWidgetItem(text)
            item.setData(Qt.ItemDataRole.UserRole, mission)
            self.missions_list.addItem(item)
    
    def _on_mission_selected(self, item: QListWidgetItem):
        """Callback quando uma missão é selecionada."""
        mission = item.data(Qt.ItemDataRole.UserRole)
        if not mission:
            return
        self.current_mission_path = mission.get('path')
        self._display_mission_report(mission)
    
    def _display_mission_report(self, mission: Dict):
        """Exibe relatório completo da missão selecionada."""
        # Limpa layout anterior
        while self.report_layout.count():
            child = self.report_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        
        mission_path = mission.get('path', '')
        
        # === SEÇÃO: Informações Básicas ===
        self._add_section_title("📋 Informações da Missão")
        
        info_frame = QFrame()
        info_frame.setStyleSheet("""
            QFrame { background-color: #1a1a2e; border-radius: 8px; padding: 10px; }
            QLabel { color: #ecf0f1; font-size: 13px; }
        """)
        info_layout = QVBoxLayout(info_frame)
        
        start = mission.get('start_time')
        end = mission.get('end_time')
        
        if start:
            info_layout.addWidget(QLabel(f"📅 <b>Início:</b> {start.strftime('%d/%m/%Y às %H:%M:%S')}"))
        if end:
            info_layout.addWidget(QLabel(f"🏁 <b>Término:</b> {end.strftime('%d/%m/%Y às %H:%M:%S')}"))
        if start and end:
            duration = (end - start).total_seconds()
            mins = int(duration // 60)
            secs = int(duration % 60)
            info_layout.addWidget(QLabel(f"⏱️ <b>Duração:</b> {mins}min {secs}s"))
        
        info_layout.addWidget(QLabel(f"📷 <b>Fotos câmera:</b> {mission.get('fotos_count', 0)}"))
        info_layout.addWidget(QLabel(f"🎯 <b>Fotos CV:</b> {mission.get('fotos_cv_count', 0)}"))
        info_layout.addWidget(QLabel(f"🎬 <b>Vídeos:</b> {mission.get('videos_count', 0)}"))
        
        self.report_layout.addWidget(info_frame)
        
        # === SEÇÃO: Vídeos ===
        self._add_videos_section(mission_path)
        
        # === SEÇÃO: Fotos da Câmera (por ponto) ===
        fotos_path = os.path.join(mission_path, "fotos")
        if os.path.exists(fotos_path):
            self._add_photos_section("📸 Fotos da Câmera", fotos_path)
        
        # === SEÇÃO: Fotos CV (por ponto) ===
        fotos_cv_path = os.path.join(mission_path, "fotos_cv")
        if os.path.exists(fotos_cv_path):
            self._add_photos_section("🎯 Fotos CV (Detecções/Anomalias)", fotos_cv_path)

        # === SEÇÃO: Relatório de Missão (.md) + gráficos analíticos ===
        self._add_mission_report_section(mission_path)

        self.report_layout.addStretch()
    
    def _add_section_title(self, title: str):
        """Adiciona título de seção ao relatório."""
        label = QLabel(title)
        label.setStyleSheet("""
            font-size: 16px; 
            font-weight: bold; 
            color: #e94560; 
            padding: 10px 0px 5px 0px;
        """)
        self.report_layout.addWidget(label)
    
    def _add_videos_section(self, mission_path: str):
        """Adiciona seção de vídeos ao relatório."""
        videos = []
        
        for folder in ["videos", "videos_cv"]:
            folder_path = os.path.join(mission_path, folder)
            if os.path.exists(folder_path):
                for f in sorted(os.listdir(folder_path)):
                    if f.endswith(('.mp4', '.avi')):
                        videos.append({
                            'name': f,
                            'path': os.path.join(folder_path, f),
                            'type': 'camera' if folder == 'videos' else 'cv'
                        })
        
        if not videos:
            return
        
        self._add_section_title("🎬 Vídeos")
        
        videos_frame = QFrame()
        videos_frame.setStyleSheet("""
            QFrame { background-color: #1a1a2e; border-radius: 8px; padding: 10px; }
        """)
        videos_layout = QVBoxLayout(videos_frame)
        
        for video in videos:
            emoji = "📹" if video['type'] == 'camera' else "🔍"
            btn = QPushButton(f"{emoji} {video['name']}")
            btn.setStyleSheet("""
                QPushButton { 
                    background-color: #533483; 
                    color: white; 
                    padding: 10px; 
                    border-radius: 5px;
                    text-align: left;
                    font-size: 12px;
                }
                QPushButton:hover { background-color: #e94560; }
            """)
            btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
            btn.clicked.connect(lambda checked, p=video['path']: self._open_video(p))
            videos_layout.addWidget(btn)
        
        self.report_layout.addWidget(videos_frame)
    
    def _add_photos_section(self, title: str, photos_path: str):
        """Adiciona seção de fotos com miniaturas ao relatório."""
        if not os.path.exists(photos_path):
            return
        
        photos = sorted([f for f in os.listdir(photos_path) if f.endswith('.jpg')])
        if not photos:
            return
        
        self._add_section_title(title)
        
        # Agrupa fotos por ponto de inspeção (PXX)
        points = {}
        for photo in photos:
            match = re.match(r'(P\d+)', photo)
            point = match.group(1) if match else "Outros"
            if point not in points:
                points[point] = []
            points[point].append(photo)
        
        for point, point_photos in sorted(points.items()):
            # Título do ponto
            point_label = QLabel(f"📍 {point} ({len(point_photos)} fotos)")
            point_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #00d9ff; padding: 5px;")
            self.report_layout.addWidget(point_label)
            
            # Grid de miniaturas
            grid_frame = QFrame()
            grid_frame.setStyleSheet("QFrame { background-color: #1a1a2e; border-radius: 8px; padding: 5px; }")
            grid_layout = QGridLayout(grid_frame)
            grid_layout.setSpacing(8)
            
            row, col = 0, 0
            max_cols = 5
            
            for photo in point_photos[:20]:  # Limita a 20 fotos por ponto
                full_path = os.path.join(photos_path, photo)
                
                thumb_btn = QPushButton()
                thumb_btn.setFixedSize(120, 90)
                thumb_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
                thumb_btn.setToolTip(photo)
                
                # Carrega miniatura
                if os.path.exists(full_path):
                    pixmap = QPixmap(full_path)
                    if not pixmap.isNull():
                        scaled = pixmap.scaled(
                            116, 86,
                            Qt.AspectRatioMode.KeepAspectRatio,
                            Qt.TransformationMode.SmoothTransformation
                        )
                        thumb_btn.setIcon(QIcon(scaled))
                        thumb_btn.setIconSize(QSize(116, 86))
                
                thumb_btn.setStyleSheet("""
                    QPushButton { 
                        background-color: #0f3460; 
                        border: 2px solid #533483; 
                        border-radius: 5px; 
                    }
                    QPushButton:hover { border: 2px solid #e94560; }
                """)
                thumb_btn.clicked.connect(lambda checked, p=full_path: self._open_photo(p))
                
                grid_layout.addWidget(thumb_btn, row, col)
                col += 1
                if col >= max_cols:
                    col = 0
                    row += 1
            
            if len(point_photos) > 20:
                more_label = QLabel(f"... e mais {len(point_photos) - 20} fotos")
                more_label.setStyleSheet("color: #7f8c8d; font-size: 11px;")
                grid_layout.addWidget(more_label, row + 1, 0, 1, max_cols)
            
            self.report_layout.addWidget(grid_frame)
    
    def _add_mission_report_section(self, mission_path: str):
        """
        Adiciona, ao fim do relatório, o conteúdo do relatorio_da_missao.md
        renderizado (Markdown) e gráficos analíticos gerados das suas tabelas.
        """
        md_path = os.path.join(mission_path, REPORT_MD_NAME)
        if not os.path.exists(md_path):
            return

        try:
            with open(md_path, "r", encoding="utf-8") as f:
                md_text = f.read()
        except Exception as e:
            self._add_section_title("📄 Relatório de Missão (.md)")
            err = QLabel(f"⚠️ Não foi possível ler o relatório: {e}")
            err.setStyleSheet("color: #e94560; font-size: 12px; padding: 5px;")
            self.report_layout.addWidget(err)
            return

        self._add_section_title("📄 Relatório de Missão")

        # --- Gráficos analíticos (antes do texto, como dashboard de desempenho) ---
        try:
            charts = build_report_charts(md_text)
        except Exception as e:
            charts = []
            print(f"[LogAnalise] Erro ao gerar gráficos do relatório: {e}")

        if charts:
            charts_frame = QFrame()
            charts_frame.setStyleSheet("QFrame { background-color: #1a1a2e; border-radius: 8px; padding: 8px; }")
            charts_layout = QVBoxLayout(charts_frame)
            charts_layout.setSpacing(10)

            graphs_label = QLabel("📈 Análise gráfica de desempenho")
            graphs_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #00d9ff; padding: 4px;")
            charts_layout.addWidget(graphs_label)

            for pix in charts:
                img_label = QLabel()
                img_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                img_label.setPixmap(pix)
                img_label.setStyleSheet("background-color: #1a1a2e; border-radius: 6px;")
                charts_layout.addWidget(img_label)

            self.report_layout.addWidget(charts_frame)

        # --- Texto do relatório renderizado (Markdown) ---
        browser = _AutoTextBrowser()
        browser.document().setDefaultStyleSheet(
            "body { color: #ecf0f1; font-size: 13px; }"
            "h1 { color: #e94560; } h2 { color: #e94560; } h3 { color: #00d9ff; }"
            "table { border-collapse: collapse; }"
            "th, td { border: 1px solid #533483; padding: 4px 8px; }"
            "th { background-color: #0f3460; color: #ecf0f1; }"
            "code { color: #f5a623; }"
            "blockquote { color: #7f8c8d; }"
        )
        browser.setMarkdown(md_text)
        browser.setStyleSheet(
            "QTextBrowser { background-color: #16213e; color: #ecf0f1;"
            " border: 1px solid #533483; border-radius: 8px; padding: 10px; }"
        )
        self.report_layout.addWidget(browser)

    def _open_photo(self, photo_path: str):
        """Abre janela para exibir foto em tamanho grande."""
        photo_window = PhotoWindow(photo_path, self)
        self.media_windows.append(photo_window)
        photo_window.show()
    
    def _open_video(self, video_path: str):
        """Abre vídeo com o player padrão do sistema."""
        open_video_with_system_player(video_path, self)
    
    def mouseDoubleClickEvent(self, event):
        self.close()
