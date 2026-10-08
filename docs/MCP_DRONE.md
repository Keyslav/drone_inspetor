# Ponte MCP do Drone Inspetor

Servidor **stdio** opcional, com SDK Python `mcp==1.28.0`. Foi instalado em
`.copilot-venv/`, separado do Python ROS. Não abre uma porta MCP pública.
O cliente de IA lança o subprocesso e conversa por stdin/stdout.

| Ferramenta | Função |
| --- | --- |
| `drone_state` | Consultar telemetria/saúde; omite nonce e credenciais de comandos |
| `mission_catalog` | Consultar missões e seus pontos |
| `pending_proposals` | Consultar propostas, modo e identificadores de escolhas |
| `request_drone_action` | Criar rascunho de uma escolha existente com justificativa |

A ponte não oferece ferramentas para confirmar ou executar. A confirmação ocorre
no [painel do copiloto](COPILOTO.md). O token do gateway fica no subprocesso e
não é argumento de ferramenta nem retornado ao modelo. O cliente MCP recebe o
estado e catálogo consultados, **incluindo coordenadas**; configure o cliente
externo conforme os dados que deseja disponibilizar a ele.

## Instalar em outra máquina

```bash
cd ~/ros2_ws/src/drone_inspetor
python3 -m venv .copilot-venv
# Evita que PYTHONPATH do ROS faça o pip considerar dependências externas ao venv.
env -u PYTHONPATH .copilot-venv/bin/python -m pip install -r requirements-copilot-mcp.txt
```

O gateway deve estar iniciado com `--copilot`, de preferência inicialmente em
`shadow`. Em um cliente que aceite configuração MCP JSON, adapte este exemplo:

```json
{
  "mcpServers": {
    "drone-inspetor": {
      "command": "/home/keyslav/ros2_ws/src/drone_inspetor/.copilot-venv/bin/python",
      "args": [
        "-m", "drone_inspetor.copilot.mcp_server",
        "--gateway", "http://127.0.0.1:8765",
        "--token-file", "/home/keyslav/.config/drone_inspetor/mobile.token"
      ],
      "env": {
        "PYTHONPATH": "/home/keyslav/ros2_ws/src/drone_inspetor"
      }
    }
  }
}
```

O exemplo usa caminhos deste host; outros clientes podem usar formato diferente.
Nenhuma configuração de cliente externo foi instalada automaticamente.
Para gateway em outra estação, HTTP exige IP explícito de rede privada; HTTPS
aceita hostname e exige certificado válido. Redirecionamentos são recusados.

Fluxo sugerido para o cliente: consultar estado e catálogo, consultar `choices`
em `pending_proposals` e propor um `choice` exato, como `start:Flare`. Propor não
significa que o comando ocorreu; aguardar o operador revisar no dashboard.

A validação local abriu uma sessão real do SDK: inicialização, listagem das quatro
ferramentas, consultas e criação de proposta no gateway demo, sem execução ROS.
Referências: [SDK oficial](https://github.com/modelcontextprotocol/python-sdk)
e [transporte stdio](https://modelcontextprotocol.io/specification/2025-11-25/basic/transports).
