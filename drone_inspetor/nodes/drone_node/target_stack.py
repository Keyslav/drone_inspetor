# =================================================================================================
# TargetStack — Pilha de alvos de navegação do DroneNode
# =================================================================================================
# Substitui a antiga WaypointStack. Diferenças centrais:
#
#   - Cada elemento (Target) representa UM destino de posição+yaw. Pode ser de dois tipos:
#       MISSAO  → destino comandado pelo mission_node (GOTO/RTL).
#       DESVIO  → destino calculado para contornar um obstáculo. Empilhado dinamicamente
#                 SOBRE o target ativo quando a DeslocamentoFSM detecta obstáculo na rota.
#
#   - A pilha funciona como LIFO PURA: a DeslocamentoFSM sempre opera no topo (`current`).
#     Conclusão de um target → pop. Detecção de obstáculo → push DESVIO. Múltiplos
#     desvios encadeados → vários pushes consecutivos; a retomada se dá por pops
#     conforme cada desvio é concluído, retornando naturalmente ao MISSAO original.
#
#   - Não há mais "snapshot do target original": o original permanece embaixo na pilha,
#     preservado simplesmente porque não foi desempilhado.
#
# Exemplo de fluxo com desvio encadeado:
#       push MISSAO(A)                            stack: [A]                  topo: A
#       voa em direção a A; detecta obstáculo
#       push DESVIO(D1)                           stack: [A, D1]              topo: D1
#       voa para D1; detecta novo obstáculo no caminho
#       push DESVIO(D2)                           stack: [A, D1, D2]          topo: D2
#       chega em D2 → pop()                       stack: [A, D1]              topo: D1
#       retoma D1; chega → pop()                  stack: [A]                  topo: A
#       retoma A; chega → pop()                   stack: []                   topo: None
# =================================================================================================

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# =================================================================================================
# Enums auxiliares
# =================================================================================================
class TargetType(Enum):
    """Tipo do target empilhado."""
    # Destino comandado pelo mission_node (GOTO/RTL). Sobrevive até ser explicitamente
    # alcançado ou cancelado por reset.
    MISSAO = auto()
    # Destino calculado dinamicamente para contornar um obstáculo. Tem vida curta:
    # criado quando obstáculo é detectado, removido assim que alcançado.
    DESVIO = auto()


# =================================================================================================
# Dataclass Target
# =================================================================================================
@dataclass
class Target:
    """
    Representa UM destino de navegação na pilha.

    Atributos OBRIGATÓRIOS:
        local_position:  [x, y, z] em coordenadas locais NED.
        target_type:     MISSAO ou DESVIO.

    Atributos OPCIONAIS (todos default None; preenchidos conforme aplicável):
        latitude, longitude, altitude:
            Coordenadas globais correspondentes. Úteis para log/feedback no formato
            usado pela GUI. Não obrigatórias — a navegação interna usa só `local_position`.

        direction_yaw_*:
            Yaw da DIREÇÃO de aproximação (calculado dinamicamente a partir da posição
            atual). Recalculado pelo `apply()` do contexto ao iniciar a manobra.

        final_yaw_*:
            Yaw que o drone deve assumir AO CHEGAR no target. None significa "manter
            o yaw da chegada" (não gira após alcançar a posição).

        focus_local_position, focus_latitude, focus_longitude:
            Coordenadas do ponto de foco. Quando preenchidas, indicam que durante o
            deslocamento o drone deve manter yaw apontando para esse ponto (use_focus=True).
            Apenas relevante para targets MISSAO; DESVIO ignora foco.

        parent_target_id:
            (Apenas para DESVIO) identificador opcional do MISSAO de origem, p/ logs.
    """

    # ---- Obrigatórios ----
    local_position: list
    target_type: TargetType

    # ---- Coordenadas globais (opcionais) ----
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None

    # ---- Yaw de direção (calculado ao aplicar) ----
    direction_yaw_deg: Optional[float] = None
    direction_yaw_deg_normalized: Optional[float] = None
    direction_yaw_rad: Optional[float] = None

    # ---- Yaw final (assumido ao chegar) ----
    final_yaw_deg: Optional[float] = None
    final_yaw_deg_normalized: Optional[float] = None
    final_yaw_rad: Optional[float] = None

    # ---- Foco (apenas para MISSAO com use_focus=True) ----
    focus_local_position: Optional[list] = None
    focus_latitude: Optional[float] = None
    focus_longitude: Optional[float] = None

    # ---- Metadado (DESVIO encadeado) ----
    parent_target_id: Optional[int] = field(default=None)

    @property
    def is_desvio(self) -> bool:
        """True se este target é um desvio temporário (não da missão)."""
        return self.target_type == TargetType.DESVIO


# =================================================================================================
# TargetStack
# =================================================================================================
class TargetStack:
    """
    Pilha LIFO de targets de navegação.

    Operações principais:
        - push_missao(): empilha um destino de missão (GOTO/RTL).
        - push_desvio(): empilha um destino de desvio sobre o target atual.
        - pop():         desempilha o topo (chamado quando o target atual é concluído).
        - current:       propriedade do topo da pilha (ou None se vazia).
        - clear():       limpa toda a pilha (cancelamento total).

    A DeslocamentoFSM consulta exclusivamente `current` — não precisa saber se é
    MISSAO ou DESVIO para executar a manobra. A informação do tipo só importa
    para o detector de obstáculos (que evita encadear desvios em loop) e para logs.
    """

    # Distância mínima (m) para considerar dois pontos como "o mesmo lugar" no
    # critério anti-loop de desvios. Configurável via setter caso necessário.
    LOOP_DISTANCE_THRESHOLD: float = 1.0

    def __init__(self):
        # Pilha interna. Topo = último elemento (`_stack[-1]`).
        self._stack: list[Target] = []
        # Histórico de TODAS as posições de desvio já tentadas no ciclo atual de missão.
        # Limpo junto com `clear()`. Usado por `is_loop_candidate()` para evitar que o
        # algoritmo de evasão proponha um ponto já visitado.
        self._desvio_history: list[tuple[float, float, float]] = []

    # =============================================================================================
    # Propriedades
    # =============================================================================================

    @property
    def current(self) -> Optional[Target]:
        """Target no topo da pilha (alvo ativo da DeslocamentoFSM), ou None se vazia."""
        return self._stack[-1] if self._stack else None

    @property
    def is_empty(self) -> bool:
        """True se não há nenhum target ativo."""
        return not self._stack

    @property
    def size(self) -> int:
        """Quantidade total de targets na pilha (missao + desvios)."""
        return len(self._stack)

    @property
    def has_desvio(self) -> bool:
        """True se há ao menos um DESVIO empilhado (drone em manobra de evasão)."""
        return any(t.is_desvio for t in self._stack)

    # =============================================================================================
    # Push / Pop
    # =============================================================================================

    def push_missao(self, local_pos: list, **kwargs) -> Target:
        """
        Empilha um destino de missão (GOTO/RTL).

        Args:
            local_pos: [x, y, z] em coordenadas locais NED.
            **kwargs:  Demais atributos opcionais de Target (latitude, final_yaw_*, focus_*).

        Returns:
            O Target recém-empilhado (já no topo da pilha).
        """
        target = Target(
            local_position=list(local_pos),
            target_type=TargetType.MISSAO,
            **kwargs,
        )
        self._stack.append(target)
        return target

    def push_desvio(self, local_pos: list, **kwargs) -> Target:
        """
        Empilha um destino de desvio sobre o target atual.

        Chamado pela DeslocamentoFSM quando detecta obstáculo na rota direta para
        `current`. O desvio fica no topo até ser concluído; o pop() subsequente
        retorna naturalmente ao target original (ou ao desvio anterior, se houver
        encadeamento de obstáculos múltiplos).

        Args:
            local_pos: [x, y, z] em coordenadas locais NED — ponto calculado para evitar
                       o obstáculo (tipicamente 90° lateral a uma certa distância).
            **kwargs:  Demais atributos opcionais (latitude/longitude para log).

        Returns:
            O Target de desvio recém-empilhado.
        """
        target = Target(
            local_position=list(local_pos),
            target_type=TargetType.DESVIO,
            parent_target_id=id(self.current) if self.current else None,
            **kwargs,
        )
        self._stack.append(target)
        # Registra no histórico para checagem de loop posterior.
        self._desvio_history.append((local_pos[0], local_pos[1], local_pos[2]))
        return target

    def pop(self) -> Optional[Target]:
        """
        Remove e retorna o target do topo da pilha.

        Chamado pela DeslocamentoFSM ao concluir o target atual (em GIRANDO_FIM,
        após estabilização do yaw final).

        Returns:
            O target removido, ou None se a pilha já estava vazia.
        """
        if not self._stack:
            return None
        return self._stack.pop()

    def clear(self) -> None:
        """
        Limpa completamente a pilha e o histórico de desvios.

        Chamado em cancelamentos: STOP, perda de offboard, emergência, reset geral.
        """
        self._stack.clear()
        self._desvio_history.clear()

    # =============================================================================================
    # Anti-loop de desvios
    # =============================================================================================

    def is_loop_candidate(self, candidate_pos: list) -> bool:
        """
        Verifica se uma posição candidata a desvio já foi tentada no ciclo atual.

        Critério: a candidata está a menos de `LOOP_DISTANCE_THRESHOLD` de algum
        ponto registrado em `_desvio_history`. Usado pelo algoritmo de evasão
        para abortar desvios que estariam revisitando lugares já tentados.

        Args:
            candidate_pos: [x, y, z] da posição candidata.

        Returns:
            True se a candidata caracteriza loop; False caso contrário.
        """
        cx, cy, cz = candidate_pos[0], candidate_pos[1], candidate_pos[2]
        thr = self.LOOP_DISTANCE_THRESHOLD
        for (px, py, pz) in self._desvio_history:
            if math.sqrt((cx - px) ** 2 + (cy - py) ** 2 + (cz - pz) ** 2) < thr:
                return True
        return False

    # =============================================================================================
    # Representação
    # =============================================================================================

    def __repr__(self) -> str:
        if not self._stack:
            return "TargetStack(vazia)"
        lines = [f"TargetStack({self.size} targets, topo→base):"]
        for i, t in enumerate(reversed(self._stack)):
            pos = t.local_position
            tag = "TOPO" if i == 0 else f"  -{i}"
            lines.append(
                f"  [{tag}] {t.target_type.name} "
                f"pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            )
        return "\n".join(lines)
