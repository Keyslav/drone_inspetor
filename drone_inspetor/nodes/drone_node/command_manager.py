"""Registro da operação reservada pelo ActionServer antes de sua execução."""

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class ActiveCommand:
    """Acesso protegido pelo mesmo lock que serializa controle e despacho."""

    sequence: int
    request: Any
    handle: Any = None
    cancel_requested: bool = False
    dispatched: bool = False
