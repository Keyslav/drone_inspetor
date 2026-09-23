# Referência inicial — simulação X500 UERJ

Auditoria em **21/09/2026**, com massa, inércia, motores, herança do autostart e
hashes principais reconferidos em **22/09/2026**. Este arquivo serve como ponto de entrada para futuras
sessões de trabalho. Análise estática dos arquivos locais; não foram iniciados
voos nem alterados modelos, parâmetros PX4 ou plugins nesta auditoria.

## Leitura rápida

- **Massa simulada: 2,3263 kg**, incluindo os acessórios. O centro de massa fica
  8,65 mm à frente e 3,48 mm abaixo da origem de `base_link`.
- **Empuxo máximo ideal: 34,19 N**, contra peso de 22,80 N: relação **1,50**.
  Existe sustentação estática ideal, mas isso não comprova margem em manobra.
- **Hover calculado: aproximadamente 7797 rpm**, com comando linear médio
  de **0,784**. O autostart herda `MPC_THR_HOVER=0,60`; precisa de ensaio e
  verificação do estimador, não de substituição automática pelo cálculo.
- **Potência mecânica inferida: 298 W em hover e 547 W no máximo**, somando
  quatro motores. Não há dados suficientes para potência elétrica/autonomia.
- As principais inconsistências são **geometria dos rotores diferente no PX4**,
  **inércias dos acessórios sem coerência com suas massas/dimensões** e
  **campo magnético puramente vertical declarado** no mundo. O ensaio posterior
  mostrou que alterar esse campo não resolve o erro de yaw; o plugin o recalcula.
  Veja [validação Gazebo](docs/VALIDACAO_GAZEBO.md) para o diagnóstico atual.
- O `4030` seleciona o modelo e herda o gimbal; **não calibra a física da carga
  adicionada**. As próximas ações estão ordenadas ao final deste documento.

## Ambiente e fontes de verdade

- Projeto ROS: `/home/keyslav/ros2_ws/src/drone_inspetor`, branch `v2.0`.
- Interfaces: `/home/keyslav/ros2_ws/src/drone_inspetor_msgs`, branch `v2.0`.
- Recursos utilizados pelo servidor Gazebo: `/home/keyslav/.simulation-gazebo/`.
  Subpastas `models/`, `worlds/`, `plugins/`; arquivo `server.config`.
- Veículo: `models/x500_uerj/model.sdf`; mundo: `worlds/Plataforma_UERJ.sdf`.
- Firmware: `/home/keyslav/PX4-Autopilot`; HEAD local `94bbd2d69a`.
  O checkout tem alterações locais; o hash sozinho não identifica seu conteúdo.
- Gazebo instalado: **8.15.0 / Harmonic**.
- Autostart: `ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_x500_uerj`.
- Executável: `build/px4_sitl_default/bin/px4`.
- Parâmetros persistidos inspecionados:
  `build/px4_sitl_default/rootfs/parameters.bson` — 27 entradas, `SYS_AUTOSTART=4030`.
  Não é um dump completo de parâmetros efetivos de um processo em execução.

Outras referências do projeto: [coordenadas](docs/COORDENADAS.md),
[monitor de estados](docs/MONITOR_DRONE.md), [progresso da v2](docs/PROGRESSO_V2.md).
Os ensaios SIH anteriores da v2 usaram outro airframe; **não validam este veículo
4030 no mundo Plataforma_UERJ**.

## Inicialização informada pelo usuário

Executar em terminais separados, nos diretórios que contêm os executáveis:

```bash
# 1 — servidor
python3 simulation-gazebo --world Plataforma_UERJ --gz_ip 127.0.0.1 --headless

# 2 — PX4, a partir de /home/keyslav/PX4-Autopilot
SIM_GZ_HOME_LAT=-22.633890 SIM_GZ_HOME_LON=-40.093330 SIM_GZ_HOME_ALT=0 \
GZ_IP=127.0.0.1 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=Plataforma_UERJ \
PX4_SYS_AUTOSTART=4030 PX4_GZ_MODEL_POSE="-70,-27,57" \
nvidia-run ./build/px4_sitl_default/bin/px4

# 3 — DDS
nvidia-run MicroXRCEAgent udp4 -p 8888

# 4 — interface Gazebo
sleep 10 && nvidia-run gz sim -g

# 5 — estação de controle
nvidia-run ./QGroundControl-x86_64.AppImage
```

`nvidia-run` apenas seleciona a GPU, conforme informado pelo usuário.

**Dependência de caminho a explicitar:** neste checkout, `px4-rc.gzsim:116`
monta o nome do SDF usando `${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf`.
O comando 2 não define `PX4_GZ_MODELS`; no ambiente desta auditoria ele também
estava ausente. Antes do próximo teste, recomenda-se acrescentar:

```bash
export PX4_GZ_MODELS=/home/keyslav/.simulation-gazebo/models
```

O modo standalone pula o carregamento de `gz_env.sh` existente no ramo que inicia
o servidor. O `gz_env.sh` gerado aponta para `PX4-Autopilot/Tools/simulation/gz/models`,
onde **não existe `x500_uerj`** e a cópia de `x500_base` difere da usada aqui.
Isso é uma dependência identificada, não prova de falha da sessão do usuário:
outro terminal/script pode já definir a variável.

Há duas cópias do launcher `simulation-gazebo`: em `/home/keyslav/PX4-gazebo-models/`
e em `PX4-Autopilot/Tools/simulation/gz/`. A primeira configura
`GZ_SIM_SYSTEM_PLUGIN_PATH` para a pasta local; na segunda essa linha está
comentada. Usar caminhos explícitos evita carregar uma cópia diferente.

## Herança real do autostart

```text
4030_gz_x500_uerj
  └── 4019_gz_x500_gimbal
        └── 4001_gz_x500
              └── rc.mc_defaults
```

O 4030 define o nome do modelo e inclui o 4019. Não ajusta capacidade dos motores,
geometria, massa, inércia ou ganhos para a carga extra. O 4019 configura o gimbal.
As cópias desses três airframes no diretório `build/.../etc` são iguais às fontes.
O arquivo 4030 ainda se descreve como “Lidar L515 and Camera D456”, embora os
includes efetivos sejam `uerj_lidar` e `uerj_OakD-Lite`.

Valores relevantes por herança/default, sem overrides correspondentes no BSON:

| Parâmetro | Valor | Observação |
| --- | ---: | --- |
| `CA_ROTOR_COUNT` | 4 | quadrotor |
| `SIM_GZ_EC_FUNC1..4` | 101..104 | motores 1..4 |
| `SIM_GZ_EC_MIN1..4` | 150 | comando mínimo de rotação |
| `SIM_GZ_EC_MAX1..4` | 1000 | comando máximo de rotação |
| `MPC_THR_HOVER` | 0,60 | imposto pelo 4001; metadata genérica diz 0,50 |
| `MPC_USE_HTE` | 1 | estimador de empuxo de hover habilitado |
| `THR_MDL_FAC` | 0 | mapeamento linear de comando para saída |
| `BAT1_N_CELLS` | 4 | default de `rcS` |
| `BAT1_CAPACITY` | −1 | capacidade não definida |
| `SIM_BAT_DRAIN` | 60 s | escala de descarga artificial |
| `SIM_BAT_MIN_PCT` | 50% | piso da bateria artificial |

O BSON contém `UXRCE_DDS_SYNCT=0` e `EKF2_MAG_DECL≈−23,5013°`.
Com sincronização DDS desligada, manter ROS usando `/clock` simulado quando
interagir com o Gazebo. Defaults, valores persistidos e parâmetros efetivos são
camadas diferentes; conferir estes últimos com `param show` na sessão de voo.

## Massa, centro de massa e inércia

`gz sdf -k` validou o SDF. A soma dos 14 links e `gz sdf --inertial-stats`
concordaram. Includes e `relative_to` foram resolvidos pelo SDFormat, sem somar
novamente a pose de ancoragem das juntas fixas.

| Componente | Massa kg |
| --- | ---: |
| `x500_base/base_link` | 2,000000 |
| 4 rotores, 0,016076923 kg cada | 0,064308 |
| 4 peças do gimbal/câmera, 0,020 kg cada | 0,080000 |
| LiDAR horizontal | 0,050000 |
| OakD-Lite | 0,061000 |
| Optical flow | 0,050000 |
| LW20 | 0,020000 |
| Link adicional do sensor inferior | 0,001000 |
| **Total** | **2,326308** |

Não foram encontradas massas implícitas. Os 2 kg de `base_link` são uma massa
agregada, sem discriminação de estrutura, bateria, eletrônica, motores ou cabos;
portanto não comprovam fidelidade à aeronave física.

- COM no frame do modelo: **(0,008652672; −0,000001205; 0,236522069) m**.
- `base_link` está em `(0; 0; 0,24) m` no modelo.
- COM relativo a `base_link`: **(+8,653; −0,001; −3,478) mm**.
- Diagonal da inércia agregada no COM: **(0,0297781; 0,0324262; 0,0509033) kg·m²**.

Esses números são da configuração inicial do gimbal; sua articulação desloca a
distribuição de massa. Existe carga mais à frente, mesmo sendo pequena em massa.

Para reproduzir a validação e a soma sem iniciar uma simulação:

```bash
SDF_PATH=/home/keyslav/.simulation-gazebo/models \
  gz sdf -k /home/keyslav/.simulation-gazebo/models/x500_uerj/model.sdf

SDF_PATH=/home/keyslav/.simulation-gazebo/models \
  gz sdf --inertial-stats /home/keyslav/.simulation-gazebo/models/x500_uerj/model.sdf
```

Aqui é necessário `SDF_PATH`: definir apenas `GZ_SIM_RESOURCE_PATH` não resolveu
os includes `model://` no utilitário `gz sdf` instalado. Erros de resolução podem
vir acompanhados de uma estatística de massa zero; esse resultado é inválido.
A reconferência retornou `Valid.`, massa 2,32631 kg e o tensor abaixo, no centro
de massa e nos eixos do modelo, em kg·m²:

```text
 0.0297781      0.00000022762   0.000450443
 0.00000022762  0.0324262      -0.000000322824
 0.000450443   -0.000000322824  0.0509033
```

## Motores: empuxo e potência que o modelo representa

O `x500_uerj` inclui `x500`, que inclui `x500_base`; os quatro plugins de motor
vêm de `models/x500/model.sdf`, não de `model_ORI.sdf`.

| Configuração por motor | Valor |
| --- | ---: |
| `motorConstant` | 8,54858 × 10⁻⁶ N/(rad/s)² |
| `momentConstant` | 0,016 m |
| `maxRotVelocity` | 1000 rad/s ≈ 9549 rpm |
| `timeConstantUp` / `Down` | 12,5 / 25 ms |
| `rotorVelocitySlowdownSim` | 10 |
| Sentidos, motores 0..3 | CCW, CCW, CW, CW |

O modelo de motor usa `T=k·ω²`; o fator 10 reduz a velocidade da junta simulada
e é compensado no cálculo do empuxo. Não dividir a rotação física máxima por
10 ao estimar sustentação. Referência: [implementação Gazebo Sim 8](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc).

Com a gravidade de **9,8 m/s²** deste mundo, os cálculos estáticos são:

| Resultado idealizado | Valor |
| --- | ---: |
| Peso total | 22,798 N |
| Empuxo máximo de um rotor | 8,549 N |
| Empuxo máximo total | 34,194 N |
| **Relação empuxo/peso** | **1,500** |
| Rotação média para hover nivelado | 816,53 rad/s ≈ 7797 rpm |
| Fração de empuxo máximo para hover | 66,67% |
| Comando nominal linear `(ω−150)/850` | **0,784** |
| Massa no limite ideal de sustentação | 3,489 kg, sem margem de controle |

O centro de massa deslocado exige forças diferentes entre rotores; a rotação
acima usa distribuição média. Inclinação, aceleração e compensação de momentos
consomem a margem restante. Relação 1,50 permite hover ideal, mas é modesta para
manobras; não considerar a diferença até 3,489 kg como carga útil recomendada.

**Hover 0,60 merece revisão:** com `THR_MDL_FAC=0` e saída 150..1000, esse comando
corresponde aproximadamente a 660 rad/s e 14,895 N totais, abaixo do peso atual.
O HTE e os integradores podem compensar; 0,784 é uma estimativa inicial para
ensaio, não um parâmetro automaticamente aprovado. Ver fontes locais
`FunctionMotors.hpp`, `mixer_module.cpp` e `GZMixingInterfaceESC.cpp`.

Se o torque aerodinâmico `Q=0,016·T` for interpretado como torque no eixo,
`P=Q·ω` fornece cerca de **298 W totais no hover** e **547 W totais no máximo**.
São estimativas mecânicas do modelo, não potência elétrica nominal. Faltam Kv,
resistência, eficiência motor/ESC, curva de hélice e bateria real para obter
corrente, potência elétrica ou autonomia confiáveis.

O simulador de bateria do PX4 reduz a carga pelo tempo armado, limita-a pelo
piso configurado e publica corrente desconhecida (`−1 A`); não usa essas
potências. Fonte: `src/modules/simulation/battery_simulator/BatterySimulator.cpp`.
Além disso, neste `GZMixingInterfaceESC.cpp`, `esc_rpm` recebe diretamente o
comando `Actuators.velocity`, sem conversão rad/s→rpm e sem medição da rotação
filtrada. Não usar esse campo como tacômetro físico sem verificar a unidade.

## Inconsistências a corrigir antes de afinar o controle

### 1. Geometria dos rotores diferente no PX4 e Gazebo

Coordenadas XY relativas a `base_link`, já convertidas de FLU para **FRD**:

| Rotor | Gazebo convertido (m) | PX4 `CA_ROTORn_PX/PY` (m) |
| --- | --- | --- |
| 0 | (+0,174; +0,174) | (+0,13; +0,22) |
| 1 | (−0,174; −0,174) | (−0,13; −0,20) |
| 2 | (+0,174; −0,174) | (+0,13; −0,22) |
| 3 | (−0,174; +0,174) | (−0,13; +0,20) |

Não é apenas a troca de sinal Y: os braços têm comprimentos diferentes. O raio
geométrico do rotor é 0,2461 m. Para uma configuração física coerente, referir
as posições ao COM, considerar seu deslocamento e conferir sinais/sentidos.
Os módulos de `CA_ROTORn_KM` são 0,05, contra `momentConstant=0,016` no SDF.
Essa diferença também merece revisão da alocação de torque; não substituir
valores sem considerar a normalização da matriz do PX4 e testar os eixos.

### 2. Inércias artificiais nos acessórios

- **Gimbal:** cada peça de 20 g usa `Ixx=Iyy=Izz=0,001 kg·m²`. Considerando a
  massa contida nas próprias malhas escaladas, supera até o limite superior
  `massa × raio máximo²` em cerca de **16–28 vezes**. Refazer pelas dimensões
  ou CAD e conferir orientação do tensor. Fonte: `uerj_gimbal/model.sdf:8` e
  demais blocos `inertial`.
- **LiDAR:** massa 50 g com inércias compatíveis com uma caixa de **370 g**,
  dimensões 0,06×0,06×0,087 m; fator **7,4**. Fonte: `uerj_lidar/model.sdf:6`.
- **Hélices:** malha alongada em Y, colisão alongada em X (90° de diferença);
  `Iyy/Izz` são aproximadamente quatro vezes menores que uma caixa uniforme
  com a massa e dimensões da colisão. A caixa não é uma hélice real, mas a
  inconsistência entre visual, colisão e tensor precisa ser resolvida.

### 3. Posição do dispositivo não é posição do emissor

No frame do modelo, após resolver includes:

| Ponto | Posição (m) |
| --- | --- |
| Emissor LiDAR horizontal | (0,005; 0; 0,192) |
| Frame do dispositivo LiDAR | (0,105; 0; 0,242) |
| Emissor depth | (0,17233; 0; 0,23378) |
| Sensor inferior adicional | (0; 0; 0,190) |

O emissor LiDAR está 10 cm atrás e 5 cm abaixo do frame do dispositivo. O
inferior adicional está 29 mm acima do frame LW20: há dois sensores inferiores
distintos. O bridge ROS do projeto usa o adicional `lidar_sensor_link`.
Os offsets de navegação ROS continuam zerados por default; calibrá-los usando
o frame real da pose e do emissor, não apenas a pose do include. Não foram
aplicados automaticamente: o adaptador horizontal não trata offset vertical
ou inclinação completa, e a origem da estimativa PX4 deve ser conferida.

### 4. Campo magnético sem componente horizontal

`Plataforma_UERJ.sdf:15` define **`0 0 5e-5` T**. Campo puramente vertical não
fornece referência horizontal de norte ao magnetômetro. Isso pode prejudicar
inicialização/fusão de yaw, independentemente da conversão ENU/NED. O efeito
efetivo depende das fontes de yaw habilitadas no EKF. Usar um campo geográfico
coerente com a localização e verificar yaw estimado contra groundtruth.

### 5. Passo físico e taxas declaradas

O mundo define passo máximo **0,008 s** (125 passos/s), `real_time_factor=1` e
`real_time_update_rate=250`. Esses valores não significam que a dinâmica roda
a 250 Hz; esse último campo não deve ser tratado como prova da taxa efetiva
no Gazebo Harmonic. O passo é grande frente à constante de subida de motor
de 12,5 ms. Comparar 1–2 ms num ensaio controlado, medindo custo e estabilidade,
antes de atribuir oscilações somente aos ganhos do controlador. O campo
`physics type="ode"` também não identifica sozinho o backend efetivamente
carregado pelo plugin Physics do Harmonic.

## Mundo, coordenadas e plugins

O mundo **declara ENU**, latitude −22,633890°, longitude −40,093330°, elevação
0 m, sem rotação geográfica explícita. Portanto o spawn `(-70,-27,57)` significa
70 m a oeste, 27 m ao sul e 57 m acima da origem do mundo. O vetor equivalente
NED seria `(-27,-70,-57)`, **apenas se as origens coincidissem**. O HOME normalmente
é definido a partir da posição do veículo; 57 m de mundo não é 57 m acima do piso
da plataforma. Yaw de spawn ENU zero aponta para Leste, equivalente a rumo PX4 90°.

Não encontrei leitura de `SIM_GZ_HOME_LAT/LON/ALT` no `src/`/ROMFS deste PX4
nem no launcher inspecionado. Não confiar nessas variáveis para sobrepor a
georreferência; os valores desejados já estão gravados no SDF do mundo.

`server.config` habilita Physics, UserCommands, SceneBroadcaster, Contact, Imu,
AirPressure, AirSpeed, ApplyLinkWrench, NavSat, Magnetometer e Sensors/Ogre2,
além de OpticalFlow, GstCamera e Template personalizados. A biblioteca
MovingPlatformController existe na pasta, mas não é habilitada nesse arquivo.
O código local do Template tem callbacks vazios; não implementa física extra.
Não foi comprovada a identidade binária entre cada `.so` instalado e seu código
fonte atual. Os plugins de sensores não acrescentam automaticamente a massa
de equipamentos: ela vem dos links SDF.

## Próximas ações recomendadas

1. Fixar caminhos de recursos e exportar os parâmetros efetivos da sessão 4030.
2. Corrigir massas/inércias com dados dos componentes; recalcular COM/tensor.
3. Alinhar geometria e coeficientes da alocação PX4 aos rotores reais do modelo.
4. Revisar hover inicial, campo magnético e extrínsecos dos sensores.
5. Ensaiar hover e pequenos comandos em roll/pitch/yaw, registrando saturação,
   atitude, `hover_thrust_estimate`, setpoints e velocidades dos rotores.
6. Só depois afinar cruzeiro, frenagem e desvio do software de inspeção.

Não aplicar todos os ajustes simultaneamente: registrar cada mudança e comparar
com uma execução de referência. Esta auditoria não mede desempenho em voo.

### Identificação dos arquivos auditados

Prefixos SHA-256 para detectar alterações futuras:

| Arquivo | SHA-256, primeiros 16 caracteres |
| --- | --- |
| `x500_uerj/model.sdf` | `4f9d1e5085320956` |
| `x500_base/model.sdf` | `96de2fdfa88f2cb4` |
| `Plataforma_UERJ.sdf` | `fbb29fc69c98b9c4` |
| `4030_gz_x500_uerj` | `b58ad74a7e9b1062` |
