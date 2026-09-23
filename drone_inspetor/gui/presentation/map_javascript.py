"""Contrato Python/JavaScript do mapa, sem dependência de Qt ou ROS."""

import json


class MapJavaScript:
    """Centraliza nomes, serialização e fila de marcadores antes do carregamento."""

    FUNCTIONS = frozenset(('setMapCenter', 'updateDronePosition', 'updateDroneStatus',
                           'addHomeMarker', 'removeHomeMarker', 'addInspectionPoint',
                           'clearMissionMarkers', 'zoomIn', 'zoomOut'))

    def __init__(self, page, ready):
        self._page = page
        self._ready = ready
        self._pending = []

    def call(self, function, *values, callback=None, queue=False):
        """Só emite funções conhecidas; rejeita NaN/Inf antes de alterar a fila."""
        if function not in self.FUNCTIONS:
            raise ValueError(f'Função de mapa desconhecida: {function}')
        arguments = ', '.join(json.dumps(value, allow_nan=False) for value in values)
        script = f'{function}({arguments});'
        if queue and not self._ready():
            self._pending.append((script, callback))
        else:
            self._execute(script, callback)

    def _execute(self, script, callback):
        page = self._page()
        if callback is None:
            page.runJavaScript(script)
        else:
            page.runJavaScript(script, callback)

    def flush(self):
        """Entrega comandos pendentes na ordem, após a inicialização do mapa."""
        if not self._ready():
            return
        while self._pending:
            script, callback = self._pending[0]
            self._execute(script, callback)
            self._pending.pop(0)

    def expanded_styles(self):
        """Estilo da janela expandida, mantido junto da integração com a página."""
        self._execute(EXPANDED_STYLES, None)


EXPANDED_STYLES = """
        (function() {
            var style = document.createElement('style');
            style.textContent = `
                /* Ícones 3x maiores */
                .drone-icon {
                    width: 72px !important;
                    height: 72px !important;
                    margin-left: -36px !important;
                    margin-top: -36px !important;
                }
                .home-marker {
                    width: 60px !important;
                    height: 60px !important;
                    font-size: 30px !important;
                    margin-left: -30px !important;
                    margin-top: -30px !important;
                }
                .inspection-marker {
                    width: 45px !important;
                    height: 45px !important;
                    font-size: 21px !important;
                    margin-left: -22px !important;
                    margin-top: -22px !important;
                }
                /* Botões de zoom maiores */
                .leaflet-control-zoom a {
                    width: 60px !important;
                    height: 60px !important;
                    font-size: 36px !important;
                    line-height: 60px !important;
                }
                /* Tooltip/popup fonts 3x */
                .leaflet-popup-content {
                    font-size: 21px !important;
                }
                .leaflet-tooltip {
                    font-size: 18px !important;
                }
            `;
            document.head.appendChild(style);
        })();
        """
