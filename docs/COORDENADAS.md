# Coordenadas, origens e altitudes

O projeto usa **NED no controle e planejamento**. ROS 2 transporta mensagens;
ele não converte automaticamente seus números para ENU. Os tópicos `/fmu/*`
preservam os contratos PX4. Converter novamente esses dados inverteria os eixos.

## Convenções encontradas

| Contexto | X | Y | Z | Orientação horizontal |
| --- | --- | --- | --- | --- |
| Mundo Gazebo ENU / mundo ROS convencional | Leste | Norte | Cima | zero Leste, positivo anti-horário |
| PX4 e navegação interna NED | Norte | Leste | Abaixo | zero Norte, positivo horário |
| Corpo/sensor ROS FLU | Frente | Esquerda | Cima | zero frente, positivo para esquerda |
| Corpo PX4 FRD | Frente | Direita | Abaixo | relativo ao veículo |
| Posições locais em DroneStateMSG legado | Norte | Leste | Cima | yaw continua sendo rumo NED |

A última linha é uma **tupla legada de apresentação**, chamada NEU neste projeto.
Não é ENU, não é NED e não deve ser usada como frame TF de mão direita.
Velocidade e aceleração na mesma mensagem permanecem NED: durante uma subida,
`current_local_z` aumenta e `current_velocity_z` é negativo. Os comentários da
mensagem e o monitor agora explicitam essa diferença. Não houve mudança nos
campos ou nos sinais publicados, preservando os consumidores existentes.

## Onde a conversão acontece

1. **Gazebo → PX4:** o `GZBridge.cpp` do firmware local já transforma posição
   ENU para NED (`y, x, -z`) e orientação FLU/ENU para FRD/NED.
2. **PX4 → drone_node:** posição, velocidade e aceleração chegam em NED. A
   atitude usa quaternion `[w, x, y, z]`; mensagens ROS `geometry_msgs/Quaternion`
   apresentam componentes `x, y, z, w`. Não passar uma ordem como se fosse a outra.
3. **Scans → navegação:** ângulos FLU tornam-se rumos NED por
   `yaw_veículo − yaw_montagem − ângulo_scan`. Offsets Frente/Esquerda também
   são rotacionados antes de adicionar a posição local do drone.
4. **Navegação → DroneStateMSG:** somente posições locais têm Z invertido para
   o contrato legado. O monitor usa `/fmu/out/vehicle_local_position` para seus
   vetores NED e avisa sobre o legado ao inspecionar `DroneStateMSG`.

O bridge `ros_gz` configurado no projeto transporta relógio, imagens e scans;
não fornece uma transformação geral entre origem Gazebo, mapa ROS e estimador.
O scan projetado da câmera já incorpora o yaw de montagem; não aplicá-lo duas
vezes. Câmera óptica usa X direita, Y baixo, Z frente antes da projeção para FLU.
O adaptador horizontal atual assume sensores nivelados e considera apenas yaw;
não implementa uma transformação 3D completa de roll/pitch ou gimbal via TF.

## Troca de eixos não é troca de origem

Para origens coincidentes e mundo ENU alinhado ao norte geográfico:

```text
Gazebo/ENU: (Leste=10, Norte=20, Cima=5)
        ↓ troca X/Y e inverte Z
PX4/NED:   (Norte=20, Leste=10, Abaixo=-5)

yaw_NED = normalizar(90° − yaw_ENU)
```

A origem do mundo Gazebo, a origem do estimador PX4 e o HOME podem ser diferentes.
Comparar posições absolutas exige também a translação entre origens; mundos
com orientação geográfica personalizada exigem a rotação correspondente.
Não interpretar uma pose de spawn no Gazebo diretamente como destino NED.
Os helpers ENU/NED fazem apenas mudança de eixos, nunca inventam essa translação.
Após o usuário indicar seu diretório, também foi confirmado ENU no mundo
`/home/keyslav/.simulation-gazebo/worlds/Plataforma_UERJ.sdf`, sem rotação
geográfica explícita. O spawn `(-70,-27,57)` usa esses eixos do mundo; não é um
destino local do estimador. Veja a [referência da simulação](../INIT_SIMULACAO.md).

## Quatro valores que não devem ser chamados apenas de “altitude”

| Valor | Referência | Uso |
| --- | --- | --- |
| `VehicleLocalPosition.z` | origem do estimador; positivo para baixo | controle NED |
| `DroneCommand.alt` / waypoint `alt` | nível médio do mar, AMSL | destino GOTO |
| `DroneCommand.altitude` | altura acima do HOME | TAKEOFF |
| scan inferior | distância ao longo do feixe | proximidade do solo |

Exemplo: HOME a 500 m AMSL, com `home_local_z` **PX4** igual a −7 m.
Um GOTO para 510 m AMSL deve usar `z = −7 − (510 − 500) = −17 m`.
No `DroneStateMSG` legado, o mesmo HOME aparece com `home_local_z = +7 m`;
não usar esse valor de apresentação no cálculo NED.

As conversões GPS locais mantêm a aproximação geográfica já existente. Não são
uma projeção geodésica de alta precisão; usam latitude/longitude em graus e
altitudes com o mesmo datum AMSL, não alturas elipsoidais misturadas com AMSL.

## Regras para novas alterações

- Usar `common/coordinates.py` para conversões explícitas de eixos e altura.
- Usar `global_to_ned_offset()` no controle. A API antiga
  `global_to_local_offset()` foi mantida, mas retorna **Norte/Leste/Cima**.
- Especificar frame, origem, unidade e datum nos novos campos/interfaces.
- Manter posição, velocidade e aceleração na mesma convenção durante cálculos.
- Usar radianos internamente; o campo público `DroneCommand.yaw` aceita graus.
- Não migrar silenciosamente o contrato legado: uma futura mensagem normalizada
  deve ter nome/versão explícitos e migração de todos os consumidores.

Testes cobrem direções cardeais, ida/volta ENU/NED e FLU/FRD, sensores girados,
montagem, HOME deslocado e preservação dos sinais da telemetria legada. Não
substituem calibração dos sensores nem ensaio de um mundo Gazebo customizado.

Referências: [convenções PX4/ROS 2](https://docs.px4.io/main/en/ros2/user_guide#ros-2-px4-frame-conventions),
[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst).
Verificação adicional no firmware local: `src/modules/simulation/gz_bridge/GZBridge.cpp`,
conversão da posição groundtruth e função `rotateQuaternion()`.
