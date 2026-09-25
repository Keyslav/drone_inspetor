"""Registro validado e troca atômica do par de modelos usado por frame."""

from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
from threading import RLock


def resolve_models_directory(configured, package_share):
    """Pesos podem viver fora do build; vazio mantém instalações existentes."""
    return (Path(configured).expanduser() if configured else
            Path(package_share) / 'redes_treinadas').resolve()


class ModelRegistry:
    """Mantém o catálogo do dashboard compatível com formatos plano e agrupado."""

    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        with (self.directory / 'models.json').open(encoding='utf-8') as stream:
            raw_models = json.load(stream).get('models', {})
        if isinstance(raw_models, dict):
            entries = [dict(item, object_type=kind)
                       for kind in ('equipment', 'anomaly')
                       for item in raw_models.get(kind, [])]
        elif isinstance(raw_models, list):
            entries = deepcopy(raw_models)
        else:
            raise ValueError('models.json deve conter uma lista ou grupos de modelos')
        self._entries = entries
        self._by_name = {}
        for entry in entries:
            filename = entry.get('file_name', '')
            if (not filename or Path(filename).name != filename or
                    entry.get('object_type') not in ('equipment', 'anomaly')):
                raise ValueError(f'Entrada inválida no catálogo: {filename!r}')
            if filename in self._by_name:
                raise ValueError(f'Modelo duplicado no catálogo: {filename}')
            self._by_name[filename] = entry

    @property
    def entries(self):
        """Cópia do catálogo; a serialização não modifica o registro."""
        entries = deepcopy(self._entries)
        for entry in entries:
            path = self.directory / entry['file_name']
            entry['storage_directory'] = str(self.directory)
            try:
                entry['available'] = path.is_file()
                entry['size_bytes'] = path.stat().st_size if entry['available'] else None
            except OSError:
                entry['available'], entry['size_bytes'] = False, None
        return entries

    def first(self, kind):
        """Primeiro modelo cadastrado de cada categoria."""
        return next((item['file_name'] for item in self._entries
                     if item['object_type'] == kind), '')

    def path(self, filename, kind):
        """Só permite arquivos locais cadastrados para a categoria solicitada."""
        entry = self._by_name.get(filename)
        if entry is None or entry['object_type'] != kind:
            raise ValueError(f'Modelo {filename!r} não cadastrado como {kind}')
        # O catálogo já exige basename. O colcon --symlink-install aponta os
        # recursos de share para build/source; esse destino externo é válido.
        path = self.directory / filename
        if not path.is_file():
            raise FileNotFoundError(f'Arquivo do modelo não encontrado: {filename}')
        return path


class ModelManager:
    """Carrega candidatos antes de trocar referências; falha preserva o par atual."""

    def __init__(self, registry, loader):
        self.registry = registry
        self._loader = loader
        self._lock = RLock()
        self._models = (None, None)
        self._filenames = ('', '')

    @property
    def filenames(self):
        """Nomes dos modelos carregados com sucesso, na mesma ordem do snapshot."""
        with self._lock:
            return self._filenames

    def replace(self, object_filename='', anomaly_filename=''):
        """Aplica os dois modelos juntos após carregamento bem-sucedido."""
        with self._lock:
            filenames = list(self._filenames)
            models = list(self._models)
            for index, (filename, kind) in enumerate((
                (object_filename, 'equipment'), (anomaly_filename, 'anomaly'),
            )):
                if filename and filename != filenames[index]:
                    models[index] = self._loader(str(self.registry.path(filename, kind)))
                    filenames[index] = filename
            self._models = tuple(models)
            self._filenames = tuple(filenames)

    @contextmanager
    def snapshot(self):
        """Mantém pesos e classes consistentes durante a inferência de um frame."""
        with self._lock:
            yield self._models
