# Orientações de desenvolvimento

Leia `README.md` para instalação, execução e verificação. Os dois repositórios usam
`v2.0` e manifesto `2.0.0`; contratos da linha v1 não são intercambiáveis.

Para o ambiente do usuário, leia `INIT_SIMULACAO.md`: modelo `x500_uerj`, mundo
`Plataforma_UERJ`, autostart PX4 4030, caminhos, comandos, massas e limitações
auditadas. As convenções de coordenadas estão em `docs/COORDENADAS.md`.

- Preserve o português dos termos de domínio e nomes públicos ROS.
- `ros_interfaces` centraliza nomes, tipos e QoS; use as fábricas de pub/sub/service/action.
- `drone_node` usa composição (`DroneTrajectory`, controle PX4, servidor de actions).
  As FSMs ficam em `fsm/drone` e `fsm/deslocamento`; cada uma tem seu `description.py`.
  A base fica em `base_classes`, não em `common/state.py`.
- Modelos e algoritmos em `navigation`, `missions` e `gui/presentation` não importam ROS/Qt.
  Passe dados explícitos; mantenha publicação, arquivos e callbacks nos adaptadores.
- Não crie diretórios em validação de missão. `MissionRepository` carrega o catálogo;
  iniciar uma sessão é uma operação separada.
- O controlador de trajetória envia referências de posição/velocidade/aceleração ao PX4.
  Documente metros, segundos, radianos/graus e frame NED em cada fronteira. Z local NED
  é positivo para baixo; preserve o contrato legado de altitude em `DroneStateMSG`.
- Mensagens de detecção viram snapshots imutáveis antes de atravessar sinais Qt. A
  conversão para JSON ocorre apenas em contratos de transporte/arquivo que a exigem.
- `gui/utils.py` mantém reexports de compatibilidade. Novos consumidores usam `widgets`,
  `presentation`, `theme` e `logging` diretamente.
- O pacote usa apenas `ament_python`; não recrie uma instalação CMake alternativa.
  Entry points apontam para módulos explícitos. Não copie código Python para `share`.
- Os launchers têm argumentos `use_sim_time`, `with_*`, `bridges`, `params_file` e
  `missions_file`. Não comente nós para escolher um cenário de execução.
- Teste falhas/limites/transições, não só a implementação feliz. Consulte `test/`.
  Separe testes sem hardware da validação de integração SITL; não apresente um como outro.
- Ruckig é local (`ruckig==0.19.4`), sem API cloud/waypoints intermediários.
  Não instale dependências nem baixe pesos de modelos durante o voo.

A CI exige os testes funcionais e erros estáticos; relatórios de estilo legados são
mantidos visíveis. Não aplique uma reformatação global junto com uma correção funcional.
