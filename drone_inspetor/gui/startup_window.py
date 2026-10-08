"""Janela de partida independente do dashboard e do executor ROS.

O backend de startup é compartilhado com o menu de terminal. Esta janela só
apresenta perfis, disponibilidade do ambiente e logs do launch iniciado.
"""

import shlex
import sys
from concurrent.futures import ThreadPoolExecutor

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QPushButton, QScrollArea, QTextEdit,
    QVBoxLayout, QWidget,
)

from drone_inspetor.startup import (
    MODE_SPECS, StartupController, build_command, environment_status,
)
from drone_inspetor.startup.controller import node_defaults


STYLE = """
QWidget { background: #0b1220; color: #e8eff8; font-size: 13px; }
QLabel { background: transparent; }
QLabel#eyebrow { color: #59dec7; font-size: 11px; font-weight: 700; }
QLabel#heading { color: #f4f8fc; font-size: 26px; font-weight: 700; }
QLabel#subtle { color: #9aadc5; }
QLabel#section { color: #e8eff8; font-size: 16px; font-weight: 700; }
QFrame#panel, QFrame#modeCard, QFrame#statusCard {
    background: #121e30; border: 1px solid #2b3e58; border-radius: 12px;
}
QPushButton#modeButton {
    background: transparent; color: #edf5fc; border: 0; text-align: left;
    font-size: 16px; font-weight: 700; padding: 0;
}
QPushButton#modeButton:focus { color: #62e2cb; }
QFrame#modeCard[selected="true"] { border: 2px solid #52dbc3; background: #142b37; }
QFrame#modeCard:hover { border-color: #69bdba; }
QPushButton#start {
    background: #55dfc6; color: #072c2c; border: 1px solid #55dfc6;
    border-radius: 8px; padding: 11px 20px; font-weight: 700;
}
QPushButton#start:hover { background: #8aefda; }
QPushButton#stop, QPushButton#secondary {
    background: #1b2d43; color: #e4edf7; border: 1px solid #38506c;
    border-radius: 8px; padding: 10px 15px; font-weight: 600;
}
QPushButton#stop:hover, QPushButton#secondary:hover { border-color: #69dfcc; }
QPushButton:disabled { background: #172438; color: #70849b; border-color: #26374d; }
QComboBox, QLineEdit, QTextEdit {
    background: #0c1728; color: #e2edf8; border: 1px solid #2b3e58;
    border-radius: 7px; padding: 8px;
}
QComboBox QAbstractItemView { background: #142238; color: #e2edf8; }
QScrollArea { border: 0; }
QScrollBar:vertical { background: #0b1220; width: 10px; }
QScrollBar::handle:vertical { background: #39526d; border-radius: 5px; }
"""


STATUS_LABELS = (
    ('application', 'Sessão do projeto'),
    ('ros2', 'ROS 2'), ('package', 'Pacote ROS'),
    ('gazebo', 'Gazebo'), ('px4', 'PX4'),
    ('micro_xrce_agent', 'Micro XRCE-DDS'), ('bridges', 'Bridges'),
    ('clock', 'Relógio /clock'), ('qgroundcontrol', 'QGroundControl'),
)
STATUS_NAMES = {
    'running': 'Ativo', 'available': 'Disponível',
    'not_detected': 'Não detectado', 'unavailable': 'Indisponível',
    'unknown': 'Não verificado',
}


class StartupWindow(QMainWindow):
    """Seleciona um perfil, acompanha requisitos e controla seu launch."""

    def __init__(self, controller=None, *, probe=True):
        """Cria a interface e consulta o ambiente sem iniciar nós ROS."""
        super().__init__()
        self.controller = controller if controller is not None else StartupController()
        self._mode = 'sim'
        self._profile_options = {}
        self._options_ready = False
        self._loading_options = False
        self._command_valid = False
        self._last_running = False
        self._external_session = False
        self._clock_available = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='startup-probe')
        self._probe_future = None
        self._probe_enabled = probe
        self._mode_cards = {}
        self._status_labels = {}
        self._columns = None
        self.setWindowTitle('Drone Inspetor · Inicialização')
        self.setMinimumSize(620, 560)
        self.resize(1040, 820)
        self.setStyleSheet(STYLE)
        self._build_ui()
        self._select_mode('sim')

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(300)
        self._environment_timer = QTimer(self)
        self._environment_timer.timeout.connect(self._request_probe)
        self._environment_timer.start(8000)
        if probe:
            self._probe_future = self._executor.submit(environment_status)

    def _build_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(24, 20, 24, 24)
        root.setSpacing(16)

        eyebrow = QLabel('DRONE INSPETOR  /  INICIALIZAÇÃO')
        eyebrow.setObjectName('eyebrow')
        title = QLabel('Preparar operação')
        title.setObjectName('heading')
        subtitle = QLabel(
            'Escolha onde executar os nós. Nos modos com dashboard, '
            'ele abrirá em uma janela separada.'
        )
        subtitle.setObjectName('subtle')
        subtitle.setWordWrap(True)
        root.addWidget(eyebrow)
        root.addWidget(title)
        root.addWidget(subtitle)

        root.addWidget(self._section_title('Modo de execução'))
        self.mode_grid = QGridLayout()
        self.mode_grid.setSpacing(12)
        self.mode_grid.setContentsMargins(0, 0, 0, 0)
        for mode in ('sim', 'companion', 'dashboard'):
            spec = MODE_SPECS[mode]
            card = QFrame()
            card.setObjectName('modeCard')
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(16, 14, 16, 14)
            card_layout.setSpacing(7)
            button = QPushButton(spec.label)
            button.setObjectName('modeButton')
            button.setAccessibleName(f'Selecionar modo {spec.label}')
            button.clicked.connect(lambda _checked=False, choice=mode: self._select_mode(choice))
            description = QLabel(spec.description)
            description.setObjectName('subtle')
            description.setWordWrap(True)
            description.setMinimumHeight(48)
            card_layout.addWidget(button)
            card_layout.addWidget(description)
            card.mousePressEvent = lambda event, choice=mode: self._select_mode(choice)
            self._mode_cards[mode] = card
        root.addLayout(self.mode_grid)

        options = QFrame()
        options.setObjectName('panel')
        options_layout = QVBoxLayout(options)
        options_layout.setContentsMargins(16, 14, 16, 14)
        options_layout.setSpacing(8)
        options_layout.addWidget(self._section_title('Bridges Gazebo–ROS'))
        self.bridges_combo = QComboBox()
        self.bridges_combo.addItem('Automático — iniciar se não houver bridge', None)
        self.bridges_combo.addItem('Iniciar bridges com o projeto', True)
        self.bridges_combo.addItem('Usar bridges já existentes', False)
        self.bridges_combo.currentIndexChanged.connect(self._refresh_command)
        options_layout.addWidget(self.bridges_combo)
        help_text = QLabel(
            'Esta opção se aplica apenas ao modo Simulação. Gazebo, PX4 e '
            'Micro XRCE-DDS devem ser iniciados à parte.'
        )
        help_text.setObjectName('subtle')
        help_text.setWordWrap(True)
        options_layout.addWidget(help_text)
        self.options_panel = options
        root.addWidget(options)

        clock_panel = QFrame()
        clock_panel.setObjectName('panel')
        clock_layout = QVBoxLayout(clock_panel)
        clock_layout.setContentsMargins(16, 14, 16, 14)
        clock_layout.setSpacing(8)
        clock_layout.addWidget(self._section_title('Relógio do dashboard'))
        self.clock_combo = QComboBox()
        self.clock_combo.addItem('Automático — detectar /clock', None)
        self.clock_combo.addItem('Tempo de simulação', True)
        self.clock_combo.addItem('Tempo real', False)
        self.clock_combo.currentIndexChanged.connect(self._refresh_command)
        clock_layout.addWidget(self.clock_combo)
        clock_hint = QLabel('Escolha tempo de simulação quando o dashboard acompanhar o Gazebo.')
        clock_hint.setObjectName('subtle')
        clock_hint.setWordWrap(True)
        clock_layout.addWidget(clock_hint)
        self.clock_panel = clock_panel
        root.addWidget(clock_panel)

        configuration = QFrame()
        configuration.setObjectName('panel')
        configuration_layout = QVBoxLayout(configuration)
        configuration_layout.setContentsMargins(16, 14, 16, 14)
        configuration_layout.addWidget(self._section_title('Arquivos e nós do perfil'))
        configuration_hint = QLabel(
            'Campos vazios usam os arquivos padrão. Cada perfil mantém suas '
            'próprias escolhas enquanto esta janela estiver aberta.')
        configuration_hint.setObjectName('subtle')
        configuration_hint.setWordWrap(True)
        configuration_layout.addWidget(configuration_hint)
        self.file_inputs = {}
        self.file_buttons = {}
        for key, title, placeholder in (
                ('params_file', 'Parâmetros', 'Padrão: config/param_ros.yaml'),
                ('missions_file', 'Missões', 'Padrão: missions/missions.json'),
                ('bridges_file', 'Tópicos das bridges', 'Padrão: config/ros_gz_bridges.yaml')):
            row = QHBoxLayout()
            label = QLabel(title)
            label.setMinimumWidth(140)
            field = QLineEdit()
            field.setPlaceholderText(placeholder)
            field.setAccessibleName(f'Arquivo de {title.lower()}')
            field.textChanged.connect(self._refresh_command)
            browse = QPushButton('Escolher…')
            browse.clicked.connect(
                lambda _checked=False, name=key: self._choose_file(name))
            row.addWidget(label)
            row.addWidget(field, 1)
            row.addWidget(browse)
            self.file_inputs[key] = field
            self.file_buttons[key] = browse
            configuration_layout.addLayout(row)
        node_grid = QGridLayout()
        self.node_checks = {}
        for index, node in enumerate(node_defaults('sim')):
            checkbox = QCheckBox(node)
            checkbox.setAccessibleName(f'Iniciar {node}_node')
            checkbox.toggled.connect(self._refresh_command)
            node_grid.addWidget(checkbox, index // 3, index % 3)
            self.node_checks[node] = checkbox
        configuration_layout.addLayout(node_grid)
        reset = QPushButton('Restaurar padrões deste perfil')
        reset.setObjectName('secondary')
        reset.clicked.connect(self._reset_options)
        configuration_layout.addWidget(reset)
        root.addWidget(configuration)

        command_panel = QFrame()
        command_panel.setObjectName('panel')
        command_layout = QVBoxLayout(command_panel)
        command_layout.setContentsMargins(16, 14, 16, 14)
        command_layout.addWidget(self._section_title('Comando que será executado'))
        self.command_preview = QLineEdit()
        self.command_preview.setReadOnly(True)
        self.command_preview.setAccessibleName('Comando de inicialização')
        command_layout.addWidget(self.command_preview)
        root.addWidget(command_panel)

        actions = QHBoxLayout()
        actions.setSpacing(9)
        self.start_button = QPushButton('Iniciar modo')
        self.start_button.setObjectName('start')
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton('Parar execução')
        self.stop_button.setObjectName('stop')
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._stop)
        self.refresh_button = QPushButton('Verificar ambiente')
        self.refresh_button.setObjectName('secondary')
        self.refresh_button.clicked.connect(self._request_probe)
        actions.addWidget(self.start_button)
        actions.addWidget(self.stop_button)
        actions.addStretch()
        actions.addWidget(self.refresh_button)
        root.addLayout(actions)

        self.execution_status = QLabel('Nenhuma execução iniciada nesta janela.')
        self.execution_status.setObjectName('subtle')
        self.execution_status.setWordWrap(True)
        root.addWidget(self.execution_status)

        root.addWidget(self._section_title('Ambiente'))
        self.status_grid = QGridLayout()
        self.status_grid.setSpacing(8)
        for index, (key, label) in enumerate(STATUS_LABELS):
            card = QFrame()
            card.setObjectName('statusCard')
            layout = QVBoxLayout(card)
            layout.setContentsMargins(13, 9, 13, 9)
            heading = QLabel(label)
            heading.setStyleSheet('color: #e8eff8; font-weight: 700;')
            result = QLabel('Verificando…')
            result.setWordWrap(True)
            result.setObjectName('subtle')
            layout.addWidget(heading)
            layout.addWidget(result)
            self._status_labels[key] = result
            self.status_grid.addWidget(card, index // 2, index % 2)
        root.addLayout(self.status_grid)

        root.addWidget(self._section_title('Saída do processo'))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(170)
        self.log_view.setPlaceholderText('Os logs do launch aparecerão aqui.')
        self.log_view.document().setMaximumBlockCount(600)
        monospace = QFont('monospace')
        monospace.setStyleHint(QFont.StyleHint.Monospace)
        self.log_view.setFont(monospace)
        root.addWidget(self.log_view)
        hint = QLabel(
            'Fechar esta janela encerra o launch iniciado por ela. '
            'Processos externos não são encerrados.'
        )
        hint.setObjectName('subtle')
        hint.setWordWrap(True)
        root.addWidget(hint)
        scroll.setWidget(content)
        self.setCentralWidget(scroll)
        self._reflow()

    @staticmethod
    def _section_title(text):
        label = QLabel(text)
        label.setObjectName('section')
        return label

    def _select_mode(self, mode):
        if self._options_ready:
            self._profile_options[self._mode] = self._capture_options()
        self._mode = mode
        self._restore_options(self._profile_options.get(mode))
        self._options_ready = True
        for key, card in self._mode_cards.items():
            card.setProperty('selected', key == mode)
            card.style().unpolish(card)
            card.style().polish(card)
        self.options_panel.setVisible(mode == 'sim')
        self.clock_panel.setVisible(mode == 'dashboard')
        self.file_inputs['bridges_file'].setEnabled(mode == 'sim')
        self.file_buttons['bridges_file'].setEnabled(mode == 'sim')
        self._refresh_command()

    def _capture_options(self):
        return {
            'files': {key: field.text() for key, field in self.file_inputs.items()},
            'nodes': {node: check.isChecked() for node, check in self.node_checks.items()},
            'bridges': self.bridges_combo.currentIndex(),
            'clock': self.clock_combo.currentIndex(),
        }

    def _restore_options(self, saved=None):
        saved = saved or {'files': {}, 'nodes': node_defaults(self._mode),
                          'bridges': 0, 'clock': 0}
        self._loading_options = True
        try:
            for key, field in self.file_inputs.items():
                field.setText(saved['files'].get(key, ''))
            for node, check in self.node_checks.items():
                check.setChecked(saved['nodes'][node])
            self.bridges_combo.setCurrentIndex(saved['bridges'])
            self.clock_combo.setCurrentIndex(saved['clock'])
        finally:
            self._loading_options = False

    def _reset_options(self):
        self._restore_options()
        self._refresh_command()

    def _choose_file(self, key):
        pattern = 'JSON (*.json);;Todos os arquivos (*)' if key == 'missions_file' else (
            'YAML (*.yaml *.yml);;Todos os arquivos (*)')
        path, _ = QFileDialog.getOpenFileName(
            self, 'Selecionar arquivo', self.file_inputs[key].text(), pattern)
        if path:
            self.file_inputs[key].setText(path)

    def _launch_options(self):
        options = {key: field.text() or None for key, field in self.file_inputs.items()}
        if self._mode != 'sim':
            options['bridges_file'] = None
        options.update({f'with_{node}': check.isChecked()
                        for node, check in self.node_checks.items()})
        return options

    def _refresh_command(self):
        if self._loading_options:
            return
        self._command_valid = False
        bridges = self.bridges_combo.currentData() if self._mode == 'sim' else None
        sim_time = self.clock_combo.currentData() if self._mode == 'dashboard' else None
        if self._mode == 'dashboard' and sim_time is None:
            if self._clock_available is None:
                self.command_preview.setText('Aguardando consulta de /clock…')
                self.start_button.setEnabled(False)
                return
            sim_time = self._clock_available
        try:
            command = build_command(
                self._mode, bridges=bridges,
                use_sim_time=sim_time, **self._launch_options(),
            )
            self._command_valid = True
            self.command_preview.setText(shlex.join(command))
            self.command_preview.setCursorPosition(0)
        except (RuntimeError, ValueError, OSError) as exc:
            self.command_preview.setText(str(exc))
        self.start_button.setEnabled(
            self._command_valid and not self.controller.running and not self._external_session)

    def _start(self):
        bridges = self.bridges_combo.currentData() if self._mode == 'sim' else None
        sim_time = self.clock_combo.currentData() if self._mode == 'dashboard' else None
        try:
            command = self.controller.start(
                self._mode, bridges=bridges, use_sim_time=sim_time,
                **self._launch_options(),
            )
        except (RuntimeError, ValueError, OSError) as exc:
            self.execution_status.setText(f'Falha ao iniciar: {exc}')
            self._append_log(f'ERRO: {exc}')
            return
        self._last_running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.execution_status.setText(f'Executando {MODE_SPECS[self._mode].label}.')
        self._append_log(f'$ {shlex.join(command)}')

    def _stop(self):
        try:
            if self.controller.running:
                self.controller.stop()
            elif self._external_session:
                self.controller.stop_active(wait=False)
                self._external_session = False
                self._request_probe()
        except (RuntimeError, OSError) as exc:
            self._append_log(f'ERRO ao parar: {exc}')
            return
        self.execution_status.setText('Parada solicitada. Aguardando encerramento…')

    def _tick(self):
        try:
            lines = self.controller.recent_logs()
            for line in lines:
                self._append_log(line.rstrip('\n'))
            running = self.controller.running
            if self._last_running and not running:
                code = self.controller.poll()
                self.execution_status.setText(f'Execução encerrada (código {code}).')
                self._last_running = False
            self.start_button.setEnabled(
                self._command_valid and not running and not self._external_session)
            self.stop_button.setEnabled(running or self._external_session)
        except (RuntimeError, OSError) as exc:
            self._append_log(f'ERRO ao consultar execução: {exc}')

        if self._probe_future is not None and self._probe_future.done():
            future, self._probe_future = self._probe_future, None
            try:
                self._show_environment(future.result())
            except (RuntimeError, OSError) as exc:
                self._append_log(f'ERRO ao verificar ambiente: {exc}')
            self._refresh_command()

    def _show_environment(self, report):
        # Só oferecemos Parar para sessões registradas por este iniciador.
        # Um `ros2 launch` aberto manualmente pode aparecer no diagnóstico,
        # mas nunca deve ser encerrado por esta janela.
        active_session = getattr(self.controller, 'active_session', lambda: None)()
        self._external_session = bool(active_session) and not self.controller.running
        if 'clock' in report:
            self._clock_available = bool(report['clock'].get('available'))
        for key, _label in STATUS_LABELS:
            result = report.get(key, {})
            ready = bool(result.get('available'))
            state = result.get('state', 'indisponível')
            detail = result.get('detail', '')
            label = self._status_labels[key]
            state_label = STATUS_NAMES.get(state, state)
            label.setText(f'{state_label} · {detail}' if detail else state_label)
            label.setStyleSheet('color: #62e2cb;' if ready else 'color: #edab79;')

    def _request_probe(self):
        if self._probe_future is None:
            self._probe_future = self._executor.submit(environment_status)

    def _append_log(self, text):
        if text:
            self.log_view.append(text)

    def _reflow(self):
        columns = 1 if self.width() < 800 else 3
        if columns == self._columns:
            return
        self._columns = columns
        for mode in self._mode_cards:
            self.mode_grid.removeWidget(self._mode_cards[mode])
        for index, mode in enumerate(('sim', 'companion', 'dashboard')):
            self.mode_grid.addWidget(self._mode_cards[mode], index // columns, index % columns)

    def resizeEvent(self, event):
        """Empilha perfis em janelas estreitas sem recriar controles."""
        super().resizeEvent(event)
        if self._mode_cards:
            self._reflow()

    def closeEvent(self, event):
        """Encerra somente o launch pertencente a esta janela."""
        self._timer.stop()
        self._environment_timer.stop()
        if self.controller.running:
            self.controller.stop(wait=True)
        self._executor.shutdown(wait=False, cancel_futures=True)
        super().closeEvent(event)


def main():
    """Abre o iniciador sem criar nós ROS na própria interface."""
    app = QApplication.instance() or QApplication(sys.argv)
    window = StartupWindow()
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
