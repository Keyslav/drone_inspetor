"""Propostas limitadas, expiráveis e auditadas antes de delegar à API de missão."""

from collections import OrderedDict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
from threading import Lock, RLock
import time

from drone_inspetor.mobile_gateway.api import GatewayError


BASE_CHOICES = {
    'none': ('Não executar; responder ou solicitar esclarecimento', None, {}),
    'mission.cancel': ('Cancelar a missão atual pela FSM', 'mission.cancel', {}),
    'camera.capture': ('Salvar uma foto da câmera no gateway', 'camera.capture', {}),
    'camera.record.start': ('Iniciar gravação manual da câmera', 'camera.record', {'enabled': True}),
    'camera.record.stop': ('Parar gravação manual da câmera', 'camera.record', {'enabled': False}),
    'cv.record.start': ('Iniciar gravação CV', 'cv.record', {'enabled': True}),
    'cv.record.stop': ('Parar gravação CV', 'cv.record', {'enabled': False}),
    'cv.anomaly.enable': ('Ativar análise de anomalias CV', 'cv.anomaly', {'enabled': True}),
    'cv.anomaly.disable': ('Desativar análise de anomalias CV', 'cv.anomaly', {'enabled': False}),
}


class CopilotService:
    """Inferência nunca publica; somente confirmação posterior passa pela MobileAPI."""

    def __init__(self, api, providers, *, mode='shadow', audit_dir=None, clock=time.monotonic):
        if mode not in ('shadow', 'review'):
            raise ValueError('Modo do copiloto deve ser shadow ou review')
        self.api, self.providers, self.mode, self.clock = api, providers, mode, clock
        self.audit_dir = Path(audit_dir).expanduser() if audit_dir else None
        self.lock, self.inference_lock = RLock(), Lock()
        self.proposals = OrderedDict()
        self.last_request = -math.inf

    def choices(self):
        """Coordenadas/altitudes permanecem no catálogo validado da missão."""
        choices = dict(BASE_CHOICES)
        if len(self.api.missions) > 64:
            raise GatewayError('Catálogo grande demais para o copiloto.', 503)
        for name in self.api.missions:
            if len(name) > 190:
                continue
            choices['start:' + name] = (f'Iniciar a missão cadastrada {name}',
                                        'mission.start', {'mission': name})
        return choices

    def context(self):
        """Fornece resumo sem GPS, imagens, logs completos ou credenciais."""
        snapshot = self.api.state.snapshot()
        fields = {
            'drone': ('state_name', 'is_armed', 'is_landed'),
            'mission': ('state_name', 'on_mission', 'mission_name'),
            'status': ('nav_state_name', 'nav_state', 'arming_state', 'failsafe'),
            'battery': ('remaining', 'connected', 'warning'),
            'lidar': ('minimum_distance',), 'down': ('minimum_distance',),
        }
        telemetry = {}
        for key, keys in fields.items():
            topic = snapshot['topics'][key]
            telemetry[key] = {'health': topic['health'], 'age_s': topic['age_s'],
                              'values': {name: topic['values'].get(name) for name in keys}}
        return {'choices': [{'id': key, 'description': value[0]}
                            for key, value in self.choices().items()],
                'telemetry': telemetry}

    def _signature(self):
        context = self.context()
        stable = {key: context['telemetry'][key]['values']
                  for key in ('drone', 'mission', 'status')}
        data = json.dumps([stable, self.api.missions], sort_keys=True, allow_nan=False)
        return hashlib.sha256(data.encode()).hexdigest()

    def status(self):
        """Disponibiliza propostas do MCP também para revisão na interface."""
        with self.lock:
            return {'enabled': True, 'mode': self.mode,
                    'providers': [{'name': name, 'model': getattr(provider, 'model', ''),
                                   'configured': name == 'demo' or bool(
                                       getattr(provider, 'api_key', '') or
                                       getattr(provider, '_api_key', '')) and bool(
                                       getattr(provider, 'model', ''))}
                                  for name, provider in self.providers.items()],
                    'choices': self.context()['choices'],
                    'proposals': [self._public(item) for item in reversed(
                        list(self.proposals.values()))][-32:]}

    def _public(self, item):
        public = {key: value for key, value in item.items() if not key.startswith('_')}
        public['expires_in_s'] = max(0, 60 - (self.clock() - item['_created']))
        public['can_execute'] = bool(
            self.mode == 'review' and self.api.enabled and item['source'] != 'demo'
            and item['command'] and item['status'] == 'proposed' and public['expires_in_s'] > 0)
        return public

    def _record(self, event, **values):
        if self.audit_dir:
            self.audit_dir.mkdir(parents=True, exist_ok=True)
            path = self.audit_dir / 'events.jsonl'
            data = {'time': datetime.now(timezone.utc).isoformat(), 'event': event, **values}
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            with os.fdopen(descriptor, 'a') as stream:
                stream.write(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n')

    def propose(self, body):
        """Executa uma inferência por vez e guarda somente uma proposta validada."""
        if not isinstance(body, dict) or set(body) != {'provider', 'prompt'}:
            raise GatewayError('Informe provider e prompt.')
        prompt, name = body['prompt'], body['provider']
        if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 2000:
            raise GatewayError('Pedido deve ter entre 1 e 2000 caracteres.')
        if not isinstance(name, str) or name not in self.providers:
            raise GatewayError('Fornecedor de IA desconhecido.')
        if not self.inference_lock.acquire(blocking=False):
            raise GatewayError('Uma consulta já está em andamento.', 429)
        try:
            started = self.clock()
            if started - self.last_request < 2:
                raise GatewayError('Aguarde dois segundos entre consultas.', 429)
            self.last_request = started
            signature = self._signature()
            context = self.context()
            self._record('request', provider=name, prompt=prompt, context=context)
            result = self.providers[name].propose(prompt.strip(), context)
            return self._draft(result, source=name, signature=signature,
                               latency_s=max(0, self.clock() - started))
        except Exception as exc:
            self._record('inference_error', provider=name, error_type=type(exc).__name__)
            raise
        finally:
            self.inference_lock.release()

    def draft(self, body):
        """Recebe escolha estruturada de um cliente MCP sem lhe dar confirmação."""
        if not isinstance(body, dict) or set(body) != {'choice', 'explanation'}:
            raise GatewayError('Informe somente choice e explanation.')
        return self._draft(dict(body, confidence=None), source='mcp', signature=self._signature())

    def _draft(self, result, *, source, signature, latency_s=None):
        if not isinstance(result, dict) or set(result) - {'choice', 'explanation', 'confidence'}:
            raise GatewayError('Proposta fora do contrato.', 502)
        choice = result.get('choice')
        explanation = result.get('explanation')
        if not isinstance(choice, str) or choice not in self.choices():
            raise GatewayError('Ação fora do catálogo.', 422)
        if not isinstance(explanation, str) or not 1 <= len(explanation) <= 1000:
            raise GatewayError('Explicação inválida.', 422)
        confidence = result.get('confidence')
        if confidence is not None and (type(confidence) not in (int, float) or
                                       not math.isfinite(confidence) or not 0 <= confidence <= 1):
            raise GatewayError('Confiança inválida.', 502)
        label, command, args = self.choices()[choice]
        item = {'id': secrets.token_urlsafe(18), 'choice': choice, 'label': label,
                'command': command, 'args': dict(args), 'explanation': explanation,
                'source': source, 'model': getattr(self.providers.get(source), 'model', ''),
                'confidence': confidence, 'latency_s': latency_s,
                'status': 'proposed' if command else 'observation',
                '_created': self.clock(), '_signature': signature}
        with self.lock:
            self._record('proposal', proposal=self._public(item))
            self.proposals[item['id']] = item
            while len(self.proposals) > 32:
                self.proposals.popitem(last=False)
            self.api.state.event(f'Copiloto ({source}): proposta {item["id"]}: {label}')
            return self._public(item)

    def confirm(self, body):
        """Revalida intenção, estado e nonce imediatamente antes de delegar."""
        if (not isinstance(body, dict) or set(body) != {'proposal_id', 'nonce', 'confirmed'} or
                not isinstance(body.get('proposal_id'), str)):
            raise GatewayError('Confirmação de proposta inválida.')
        if body['confirmed'] is not True:
            raise GatewayError('Confirmação explícita necessária.', 409)
        with self.lock:
            item = self.proposals.get(body['proposal_id'])
            if not item:
                raise GatewayError('Proposta inexistente.', 404)
            if '_result' in item:
                return item['_result']
            if not self._public(item)['can_execute']:
                raise GatewayError('Proposta expirada, sem ação ou somente observação.', 409)
            if item['_signature'] != self._signature():
                raise GatewayError('Estado/catálogo mudou. Prepare uma nova proposta.', 409)
            self._record('confirmation', proposal_id=item['id'])
            result = self.api.execute({'id': item['id'], 'nonce': body['nonce'],
                                       'command': item['command'], 'args': item['args'],
                                       'confirmed': True})
            item['_result'], item['status'] = result, result['status']
            self._record('result', proposal_id=item['id'], result=result)
            return result
