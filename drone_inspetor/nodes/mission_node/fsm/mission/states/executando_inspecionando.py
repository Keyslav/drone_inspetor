# executando_inspecionando.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO
# =================================================================================================
# Percorre os pontos de inspeção da missão, enviando comandos GOTO (com ou sem foco).
# Quando chegado (tempo_de_chegada > 0), verifica se o ponto requer detecção.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class ExecutandoInspecionandoState(BaseState):
    """
    Navega pelos pontos de inspeção da missão, sequencialmente.
    Detecta chegada via callback de action e decide sub-estado seguinte.
    """

    def on_step(self):
        # Aguarda action em andamento
        if self.node._action_in_progress:
            if self.node.check_action_timeout():
                self.node.get_logger().error("Timeout navegando para waypoint. Iniciando retorno.")
                return MS.RETORNANDO
            self.node.get_logger().info(
                "Navegando para ponto de inspeção...",
                throttle_duration_sec=3.0,
            )
            return None

        # Verifica se ainda há missão ativa
        if not self.context.mission:
            self.node.get_logger().warn("Missão não carregada. Iniciando retorno.")
            return MS.RETORNANDO

        # Verifica se todos os waypoints foram visitados
        if self.context.ponto_de_inspecao_indice_atual >= self.context.total_pontos():
            self.node.get_logger().info("Todos os pontos de inspeção concluídos!")
            return MS.INSPECAO_FINALIZADA

        ponto_atual = self.context.get_ponto_atual()
        if ponto_atual is None:
            self.node.get_logger().warn("Ponto de inspeção atual inválido. Iniciando retorno.")
            return MS.RETORNANDO

        # Chegou ao waypoint (tempo_de_chegada foi marcado pelo callback de action)
        if self.context.ponto_de_inspecao_tempo_de_chegada > 0:
            ponto_de_deteccao = ponto_atual.get("ponto_de_deteccao", False)

            if ponto_de_deteccao:
                self.node.get_logger().info(
                    f"Ponto {self.context.ponto_de_inspecao_indice_atual} requer detecção. "
                    "Iniciando fase de detecção."
                )
                return MS.EXECUTANDO_INSPECIONANDO_DETECTANDO
            else:
                self.node.get_logger().info(
                    f"Ponto {self.context.ponto_de_inspecao_indice_atual} concluído (sem detecção). "
                    "Avançando para próximo ponto."
                )
                self.context.ponto_de_inspecao_indice_atual += 1
                self.context.ponto_de_inspecao_tempo_de_chegada = 0.0
                return None

        # Ainda não chegou — envia comando de navegação
        lat = ponto_atual.get("lat")
        lon = ponto_atual.get("lon")
        alt = ponto_atual.get("alt", self.context.takeoff_altitude)
        yaw = ponto_atual.get("yaw", float("nan"))
        focus_lat = ponto_atual.get("focus_lat")
        focus_lon = ponto_atual.get("focus_lon")

        if focus_lat is not None and focus_lon is not None:
            self.node.get_logger().info(
                f"Enviando GOTO (use_focus=True) para ponto {self.context.ponto_de_inspecao_indice_atual} "
                f"({lat}, {lon}, {alt}) foco ({focus_lat}, {focus_lon})."
            )
            self.node.send_drone_action({
                "command": "GOTO",
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "yaw": yaw,
                "use_focus": True,
                "focus_lat": focus_lat,
                "focus_lon": focus_lon,
            })
        else:
            self.node.get_logger().info(
                f"Enviando GOTO para ponto {self.context.ponto_de_inspecao_indice_atual} "
                f"({lat}, {lon}, {alt})."
            )
            self.node.send_drone_action({
                "command": "GOTO",
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "yaw": yaw,
            })

        return None
