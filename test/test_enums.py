"""Testes para drone_inspetor.common.enums (state groups, IntEnum semantics)."""

import pytest

from drone_inspetor.common.enums import (
    DroneStateDescription,
    MissionStateDescription,
    DashboardMissionCommandDescription,
    DRONE_STATES_GOTO,
    DRONE_STATES_GOTO_COM_FOCO,
    DRONE_STATES_RTL,
    DRONE_STATES_POUSANDO,
    DRONE_STATES_POUSADO,
    DRONE_STATES_EM_MOVIMENTO,
)


class TestDroneStateDescription:
    def test_is_int_enum(self):
        """Comparações int-direto devem funcionar (telemetria)."""
        assert DroneStateDescription.POUSADO_DESARMADO == 0
        assert DroneStateDescription.VOANDO_PRONTO == 10

    def test_unique_values(self):
        values = [s.value for s in DroneStateDescription]
        assert len(values) == len(set(values))

    def test_emergency_isolated(self):
        """EMERGENCIA deve ter valor distante para destacar como exceção."""
        assert DroneStateDescription.EMERGENCIA == 99


class TestMissionStateDescription:
    def test_is_int_enum(self):
        assert MissionStateDescription.DESATIVADO == 0
        assert MissionStateDescription.PRONTO == 1

    def test_unique_values(self):
        values = [s.value for s in MissionStateDescription]
        assert len(values) == len(set(values))

    def test_inspecionando_subhierarquia(self):
        """Estados sub-hierárquicos devem agrupar pela base (10x para ARMANDO/DECOLANDO/INSPECIONANDO, 20x para sub)."""
        assert 10 <= MissionStateDescription.EXECUTANDO_ARMANDO.value < 20
        assert 10 <= MissionStateDescription.EXECUTANDO_INSPECIONANDO.value < 20
        assert 20 <= MissionStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO.value < 30


class TestDashboardMissionCommandDescription:
    def test_iniciar_cancelar(self):
        assert DashboardMissionCommandDescription.INICIAR_MISSAO == 1
        assert DashboardMissionCommandDescription.CANCELAR_MISSAO == 2


class TestStateGroups:
    def test_goto_membership(self):
        """Estados GOTO devem incluir GIRANDO_INICIO, A_CAMINHO, GIRANDO_FIM."""
        assert DroneStateDescription.VOANDO_GIRANDO_INICIO in DRONE_STATES_GOTO
        assert DroneStateDescription.VOANDO_A_CAMINHO in DRONE_STATES_GOTO
        assert DroneStateDescription.VOANDO_GIRANDO_FIM in DRONE_STATES_GOTO

    def test_goto_com_foco_membership(self):
        assert DroneStateDescription.VOANDO_GIRANDO_COM_FOCO in DRONE_STATES_GOTO_COM_FOCO
        assert DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO in DRONE_STATES_GOTO_COM_FOCO

    def test_rtl_membership(self):
        assert DroneStateDescription.RETORNANDO_GIRANDO_INICIO in DRONE_STATES_RTL
        assert DroneStateDescription.RETORNANDO_A_CAMINHO in DRONE_STATES_RTL
        assert DroneStateDescription.RETORNANDO_GIRANDO_FIM in DRONE_STATES_RTL

    def test_pousado_membership(self):
        assert DroneStateDescription.POUSADO_DESARMADO in DRONE_STATES_POUSADO
        assert DroneStateDescription.POUSADO_ARMADO in DRONE_STATES_POUSADO
        assert DroneStateDescription.VOANDO_PRONTO not in DRONE_STATES_POUSADO

    def test_em_movimento_membership(self):
        """EM_MOVIMENTO deve ser união de GOTO + GOTO_COM_FOCO + RTL."""
        for s in DRONE_STATES_GOTO + DRONE_STATES_GOTO_COM_FOCO + DRONE_STATES_RTL:
            assert s in DRONE_STATES_EM_MOVIMENTO

    def test_em_movimento_excludes_pousado(self):
        for s in DRONE_STATES_POUSADO:
            assert s not in DRONE_STATES_EM_MOVIMENTO

    def test_pousando_singleton(self):
        assert DRONE_STATES_POUSANDO == [DroneStateDescription.POUSANDO]
