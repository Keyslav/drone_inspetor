"""Criação de artefatos de uma sessão após validar sua definição."""

import tempfile
from datetime import datetime
from pathlib import Path


def create_session_directory(base_directory):
    """Cria uma pasta exclusiva mesmo quando duas sessões iniciam no mesmo segundo."""
    base = Path(base_directory).expanduser()
    base.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now().strftime('mission_%Y%m%d_%H%M%S_')
    session = Path(tempfile.mkdtemp(prefix=prefix, dir=base))
    (session / 'fotos').mkdir()
    (session / 'videos').mkdir()
    return str(session)
