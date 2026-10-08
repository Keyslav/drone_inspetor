"""Avalia classificação em casos offline; jamais publica ROS ou executa propostas."""

import argparse
import json
from pathlib import Path
import time

from .providers import configured_providers


def load_cases(path):
    """Rejeita conjuntos ambíguos antes de fazer qualquer chamada paga."""
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {'choices', 'cases'}:
        raise ValueError('Dataset deve conter choices e cases')
    ids = [entry['id'] for entry in data['choices']]
    if len(ids) != len(set(ids)) or 'none' not in ids:
        raise ValueError('Escolhas devem ser únicas e incluir none')
    if not 1 <= len(data['cases']) <= 100:
        raise ValueError('Use 1 a 100 casos')
    for case in data['cases']:
        if (not isinstance(case['prompt'], str) or not 1 <= len(case['prompt']) <= 2000 or
                case['expected'] not in ids or not isinstance(case['telemetry'], dict)):
            raise ValueError('Caso inválido')
    return data


def main(argv=None):
    """Sem --run, apenas valida; com --run faz uma requisição por caso escolhido."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--provider', choices=('demo', 'jev', 'openai'), default='jev')
    parser.add_argument('--limit', type=int, default=5)
    parser.add_argument('--run', action='store_true', help='Autoriza chamadas à API configurada')
    parser.add_argument('--output', type=Path, default=Path('copilot-evaluation.jsonl'))
    args = parser.parse_args(argv)
    data = load_cases(args.cases)
    if not 1 <= args.limit <= 100:
        parser.error('--limit deve estar entre 1 e 100')
    cases = data['cases'][:args.limit]
    if not args.run:
        print(f'{len(cases)} casos validados. Nenhuma chamada; use --run para avaliar.')
        return 0
    provider = configured_providers()[args.provider]
    correct, errors = 0, 0
    # Não sobrescreve resultados de uma avaliação anterior.
    with args.output.open('x', encoding='utf-8') as output:
        for index, case in enumerate(cases):
            started = time.monotonic()
            result = {'index': index, 'provider': args.provider,
                      'model': getattr(provider, 'model', ''), **case}
            try:
                answer = provider.propose(case['prompt'], {
                    'choices': data['choices'], 'telemetry': case['telemetry']})
                result.update(answer=answer, correct=answer['choice'] == case['expected'])
                correct += result['correct']
            except Exception as exc:
                result.update(error_type=type(exc).__name__, correct=False)
                errors += 1
            result['latency_s'] = time.monotonic() - started
            output.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n')
    print(f'{correct}/{len(cases)} escolhas esperadas; {errors} erros. Resultado: {args.output}')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
