"""Janela de apresentação dos relatórios CV; ações pertencem ao controlador."""

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit

from .screens import ExpandedWindow
from ..theme import COMMON_STYLES

class AnalysisWindow(ExpandedWindow):
    """Compõe os controles e o texto sem depender de ROS ou inferência."""

    def __init__(self, on_clear, on_export):
        analysis_widget = QWidget()
        analysis_widget.setWindowTitle("Logs de Análise CV")
        analysis_widget.setGeometry(200, 200, 800, 600)

        layout = QVBoxLayout()

        # Título da janela de análise
        title = QLabel("Análise de Visão Computacional")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet(f"""
            color: {COMMON_STYLES["text_color"]};
            background-color: {COMMON_STYLES["accent_color"]};
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 10px;
        """)
        layout.addWidget(title)

        # Layout para botões de controle (Limpar Logs, Exportar Logs)
        controls_layout = QHBoxLayout()

        clear_button = QPushButton("Limpar Logs")
        clear_button.clicked.connect(on_clear)
        clear_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {COMMON_STYLES["error_color"]};
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #c0392b;
            }}
        """)
        controls_layout.addWidget(clear_button)

        export_button = QPushButton("Exportar Logs")
        export_button.clicked.connect(on_export)
        export_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {COMMON_STYLES["success_color"]};
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #229954;
            }}
        """)
        controls_layout.addWidget(export_button)

        controls_layout.addStretch()
        layout.addLayout(controls_layout)

        # Área de texto para exibir os logs de análise
        analysis_text = QTextEdit()
        analysis_text.setReadOnly(True)
        analysis_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COMMON_STYLES["light_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid {COMMON_STYLES["border_color"]};
                border-radius: 5px;
                font-family: "Courier New", monospace;
                font-size: 12px;
            }}
        """)
        layout.addWidget(analysis_text)

        analysis_widget.setLayout(layout)
        analysis_widget.setStyleSheet(f"""
            QWidget {{
                background-color: {COMMON_STYLES["dark_background"]};
            }}
        """)

        super().__init__("Análise CV", analysis_widget, None)
        self.analysis_text = analysis_text
