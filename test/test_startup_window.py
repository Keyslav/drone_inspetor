"""A tela usa os perfis compartilhados e não inicia ROS ao ser construída."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt6.QtWidgets import QApplication  # noqa: E402

from drone_inspetor.gui.startup_window import StartupWindow  # noqa: E402


class FakeController:
    """Registra apenas operações iniciadas pela janela de teste."""

    def __init__(self):
        """Inicializa o histórico isolado de cada teste."""
        self.running = False
        self.started = []
        self.options = []
        self.stopped = 0

    def start(self, mode, *, bridges=None, use_sim_time=None, **options):
        """Registra a solicitação sem criar processos."""
        self.started.append((mode, bridges, use_sim_time))
        self.options.append(options)
        self.running = True
        return ['ros2', 'launch', 'drone_inspetor', mode]

    def stop(self, *, wait=False):
        """Registra o encerramento solicitado pela janela."""
        self.stopped += 1
        self.running = False

    def poll(self):
        """Retorna o resultado simulado."""
        return 0

    def recent_logs(self):
        """Fornece um buffer vazio."""
        return []

    def active_session(self):
        """Simula a ausência de sessões de outro inicializador."""
        return None


def test_startup_window_selects_mode_and_stops_owned_launch():
    """Seleção muda as opções e fechar a tela limpa o processo iniciado."""
    app = QApplication.instance() or QApplication([])
    controller = FakeController()
    window = StartupWindow(controller, probe=False)
    try:
        window.show()
        app.processEvents()
        assert window.options_panel.isVisible()
        window.bridges_combo.setCurrentIndex(1)
        window.clock_combo.setCurrentIndex(2)
        window._select_mode('companion')
        app.processEvents()
        assert not window.options_panel.isVisible()
        window._start()
        assert controller.started == [('companion', None, None)]
        assert not window.start_button.isEnabled()
        assert window.stop_button.isEnabled()
        window.resize(680, 700)
        app.processEvents()
        assert window._columns == 1
        window.resize(1000, 700)
        app.processEvents()
        assert window._columns == 3
    finally:
        window.close()
    assert controller.stopped == 1


def test_startup_window_does_not_stop_without_start():
    """Fechar a tela vazia não interfere em processos externos."""
    app = QApplication.instance() or QApplication([])
    controller = FakeController()
    window = StartupWindow(controller, probe=False)
    window.close()
    app.processEvents()
    assert controller.stopped == 0


def test_files_and_nodes_are_kept_separate_for_each_profile(tmp_path):
    """Trocar simulação/companion não reutiliza silenciosamente um YAML."""
    app = QApplication.instance() or QApplication([])
    controller = FakeController()
    window = StartupWindow(controller, probe=False)
    config = tmp_path / 'sim parameters.yaml'
    config.write_text('{}')
    try:
        window.file_inputs['params_file'].setText(str(config))
        window.node_checks['cv'].setChecked(False)
        assert f'params_file:={config}' in window.command_preview.text()
        window._select_mode('companion')
        assert window.file_inputs['params_file'].text() == ''
        assert window.node_checks['cv'].isChecked()
        assert not window.node_checks['dashboard'].isChecked()
        window.node_checks['lidar'].setChecked(False)
        window._select_mode('sim')
        assert window.file_inputs['params_file'].text() == str(config)
        assert not window.node_checks['cv'].isChecked()
        assert window.node_checks['lidar'].isChecked()
        window._start()
        assert controller.options[-1]['params_file'] == str(config)
        assert controller.options[-1]['with_cv'] is False
        window._reset_options()
        assert window.file_inputs['params_file'].text() == ''
        assert window.node_checks['cv'].isChecked()
        window._select_mode('companion')
        assert not window.node_checks['lidar'].isChecked()
    finally:
        window.close()
        app.processEvents()


def test_invalid_file_disables_start_until_corrected(tmp_path):
    """A atualização periódica não reabilita uma partida inválida."""
    app = QApplication.instance() or QApplication([])
    window = StartupWindow(FakeController(), probe=False)
    try:
        window.file_inputs['missions_file'].setText(str(tmp_path / 'missing.json'))
        assert 'missions_file: arquivo inválido' in window.command_preview.text()
        assert not window.start_button.isEnabled()
        window._tick()
        assert not window.start_button.isEnabled()
        window.file_inputs['missions_file'].clear()
        assert window.start_button.isEnabled()
    finally:
        window.close()
        app.processEvents()


def test_external_managed_session_stops_only_after_explicit_click():
    """Consultar ou fechar a tela não toma posse de outra sessão."""
    class OtherSession(FakeController):
        def __init__(self):
            super().__init__()
            self.record = {'mode': 'sim', 'pid': 123}
            self.external_stops = 0

        def active_session(self):
            return self.record

        def stop_active(self, *, wait=False):
            self.external_stops += 1
            self.record = None

    app = QApplication.instance() or QApplication([])
    controller = OtherSession()
    window = StartupWindow(controller, probe=False)
    window.show()
    window._show_environment({})
    window._tick()
    assert not window.start_button.isEnabled()
    assert window.stop_button.isEnabled()
    window._stop()
    assert controller.external_stops == 1
    window.close()
    app.processEvents()
    assert controller.stopped == 0
