# voando_pronto.py
# Estado: VOANDO_PRONTO
# Drone voando estável (hover), aguardando comando.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.waypoint_stack import WaypointStatus


class VoandoProntoState(State):
    """
    Drone em hover estável aguardando comandos de missão.

    Processa pending_command na seguinte ordem de prioridade:
      1. GOTO (use_focus=False) → VOANDO_GIRANDO_INICIO
      2. GOTO (use_focus=True)  → VOANDO_GIRANDO_COM_FOCO
      3. LAND                   → POUSANDO
      4. RTL                    → RETORNANDO_GIRANDO_INICIO
    """

    def on_step(self):
        context = self.context

        # --- GOTO ---
        if context.pending_command == "GOTO":
            use_focus = context.pending_use_focus
            context.on_trajectory = True
            context.trajectory_start_time = context.now()
            context.pending_command = None
            context.pending_use_focus = False
            # Ativa o primeiro waypoint da pilha e carrega variáveis de navegação
            wp = context.waypoint_stack.current
            if wp:
                wp.status = WaypointStatus.ATIVO
                context.apply_waypoint(wp)
            if use_focus:
                return DS.VOANDO_GIRANDO_COM_FOCO
            context.store_static_position()
            return DS.VOANDO_GIRANDO_INICIO

        # --- LAND ---
        if context.pending_command == "LAND":
            context.store_static_position()
            context.on_trajectory = False
            context.pending_command = None
            return DS.POUSANDO

        # --- RTL ---
        if context.pending_command == "RTL":
            context.on_trajectory = True
            context.trajectory_start_time = context.now()
            context.pending_command = None
            # Ativa o primeiro waypoint da pilha e carrega variáveis de navegação
            wp = context.waypoint_stack.current
            if wp:
                wp.status = WaypointStatus.ATIVO
                context.apply_waypoint(wp)
            return DS.RETORNANDO_GIRANDO_INICIO

        return None
