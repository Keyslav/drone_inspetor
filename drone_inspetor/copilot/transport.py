"""HTTP de inferência com limites; falhas nunca provocam retentativa automática."""

import json
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, Request, build_opener

from drone_inspetor.mobile_gateway.api import GatewayError


class NoRedirect(HTTPRedirectHandler):
    """Impede transportar credenciais de um fornecedor para um destino redirecionado."""

    def redirect_request(self, *args, **kwargs):
        return None


def post_json(url, api_key, body, *, timeout=8.0):
    """Faz uma única chamada HTTPS e omite chaves/corpos nos erros públicos."""
    if not url.startswith('https://') or not api_key:
        raise GatewayError('Configure a chave do fornecedor de IA.', 503)
    data = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
    if len(data) > 65536:
        raise GatewayError('Contexto de IA excedeu o limite local.', 413)
    request = Request(url, data=data, headers={
        'Authorization': 'Bearer ' + api_key,
        'Content-Type': 'application/json', 'Accept': 'application/json',
    }, method='POST')
    try:
        with build_opener(NoRedirect()).open(request, timeout=timeout) as response:
            raw = response.read(1024 * 1024 + 1)
        if len(raw) > 1024 * 1024:
            raise GatewayError('Resposta da IA excedeu o limite local.', 502)
        result = json.loads(raw)
        if not isinstance(result, dict):
            raise ValueError('objeto esperado')
        return result
    except HTTPError as exc:
        raise GatewayError(f'Fornecedor de IA respondeu HTTP {exc.code}.', 502) from None
    except (TimeoutError, URLError):
        raise GatewayError('Fornecedor indisponível ou sem resposta; não houve reenvio.', 503) from None
    except (ValueError, UnicodeError):
        raise GatewayError('Fornecedor retornou JSON inválido.', 502) from None
