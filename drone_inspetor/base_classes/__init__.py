# =================================================================================================
# base_classes
# =================================================================================================
# Classes template (prefixo `Base`) que existem apenas para serem herdadas.
# Cada arquivo contém uma única classe Base*.
#
# Convenção: nenhum código operacional deve instanciar diretamente uma classe deste pacote.
# Subclasses concretas vivem nos pacotes consumidores (ex.: nodes/, gui/).
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.base_classes.base_state_machine import BaseStateMachine

__all__ = ["BaseState", "BaseStateMachine"]
