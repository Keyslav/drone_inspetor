# Scripts Legados

Scripts arquivados que **NÃO** fazem parte do sistema atual. Mantidos apenas para referência histórica.

| Arquivo | Descrição | Por que foi arquivado |
|---|---|---|
| `mission_control.py` | Nó standalone de controle de missão (pré-refactor) | Substituído pelo `mission_node` no novo pipeline com Actions |
| `offboard_control.py` | Exemplo simples de controle Offboard PX4 | Substituído pelos mixins do `drone_node` (px4_commands, trajectory) |

⚠️ **Estes scripts referenciam APIs antigas** (MAVROS, tópicos antigos) e **não compilam/funcionam** com a versão atual do código. Não execute.
