# Retorno da missão

O `mission_node` aguarda o cancelamento do comando anterior antes de solicitar
RTL. O servidor pode continuar ocupado enquanto confirma a frenagem. Se rejeitar
explicitamente o RTL antes de aceitá-lo, a missão pode tentar novamente.

Em `config/param_ros.yaml`, `return_acceptance_timeout` define a janela para novas
tentativas, contada desde a primeira solicitação: **40 segundos** por padrão.
`return_retry_interval` define o intervalo mínimo entre solicitações: **1 segundo**.
Os dois usam tempo monotônico; o prazo continua avançando com `/clock` pausado,
embora uma nova tentativa dependa do próximo ciclo da máquina de missão.

Um RTL aceito nunca é reenviado. Timeout com aceitação desconhecida, aceitação
tardia após timeout, erro de transporte e falha após aceitação não autorizam retentativa.
Ao terminar a janela após rejeições, a missão registra a falha e aguarda pouso ou
intervenção do operador. Os prazos de feedback e cancelamento continuam separados.
