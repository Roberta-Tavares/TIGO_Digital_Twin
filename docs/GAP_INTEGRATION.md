# Integração do Algoritmo Gap Following

## Objetivo

Este documento descreve a integração entre o algoritmo de navegação autônoma do TIGO e o ambiente virtual desenvolvido no Ignition Gazebo.

O objetivo é executar na simulação o nó de navegação ROS 2 que recebe imagens
de profundidade e publica comandos de velocidade.

---

# Arquitetura Geral

Fluxo de dados:

```text
Ignition Gazebo
       │
       ▼
Depth Camera Virtual
       │
       ▼
ros_gz_bridge
       │
       ▼
ROS 2 Humble
       │
       ▼
gap_phase35.py
       │
       ▼
/cmd_vel
       │
       ▼
TIGO Virtual
```

---

# Algoritmo Utilizado

Arquivo:

```text
gap_phase35.py
```

Responsabilidade:

- detectar espaços livres
- identificar gaps navegáveis
- calcular direção segura
- publicar comandos de velocidade

---

# Sensor Utilizado

No robô real:

```text
HP60C (Angstrong)
```

Tipo:

```text
Câmera de Profundidade
```

Dados utilizados:

```text
Mapa de profundidade
```

---

# Tópico de Profundidade

O algoritmo espera receber imagens através de:

```text
/ascamera_hp60c/camera_publisher/depth0/image_raw
```

Tipo:

```text
sensor_msgs/msg/Image
```

---

# Região de Interesse (ROI)

Para reduzir processamento, apenas uma faixa horizontal da imagem é utilizada.

Objetivos:

- reduzir carga computacional
- aumentar FPS
- focar na área de navegação

---

# Segmentação do Espaço Livre

Cada coluna da ROI é analisada.

Uma coluna é considerada livre quando:

```text
profundidade > limiar configurado
```

---

# Detecção de Gaps

Após a segmentação:

```text
livre
ocupado
livre
livre
livre
ocupado
```

o algoritmo procura sequências contínuas de espaço livre.

Essas sequências são classificadas como:

```text
Gaps candidatos
```

---

## Critério de Aceitação Atual

O código atual identifica uma coluna como livre quando o percentil 10 das
profundidades válidas por coluna na faixa central excede `DIST_OBSTACULO`
(0,7 m). Um gap candidato precisa ter pelo
menos `LARGURA_ROBO_PX` (40 pixels). A largura física do gap não é calculada no
momento.

---

# Seleção do Gap

Os candidatos são pontuados pela largura em pixels e pela profundidade média:

```text
score = largura_px * 0.7 + profundidade_media_m * 25
```

O gap de maior score é escolhido; o erro em relação ao centro da imagem define
a correção angular durante a navegação normal.

---

## Controle do Movimento

O algoritmo publica mensagens:

```text
geometry_msgs/msg/Twist
```

no tópico:

```text
/cmd_vel
```

---

# Modos de Operação

## Navegação Normal

Condição:

```text
Gap válido encontrado
```

Comportamento:

- avanço no eixo frontal
- correção angular em direção ao centro do gap

---

## Recuperação de Obstáculos

Quando o percentil 5 das distâncias na região frontal fica abaixo de
`DIST_GATILHO_RECUO` (0,6 m), o robô recua antes de procurar um caminho. O
limiar frontal é separado do limiar de profundidade dos gaps para não iniciar
um ciclo de recuo ao alinhar com uma passagem que ainda tem mais de 0,6 m de
distância frontal. Se não houver gap direto
ou a varredura não encontrar gaps, o ciclo de recuo e busca pode ser repetido
até três tentativas. O recuo começa em 1,8 s e aumenta 0,5 s por tentativa,
até 2,8 s; depois disso, o robô para por segurança.

A detecção frontal usa os 50% centrais da imagem na vertical e na horizontal;
uma faixa central de 40% da largura e o percentil 5 das leituras válidas
representam a distância frontal, reduzindo a dependência de uma faixa estreita
no centro da câmera. Raios infinitos da câmera simulada são convertidos em
10 m e contados como espaço livre; quadros sem nenhuma leitura ainda param o
robô por segurança. Um watchdog também para o robô se a câmera ficar mais de
0,3 s sem enviar imagem válida. Os logs `[SENSOR]` mostram essa distância e a
quantidade de amostras usadas.

Após recuar, o robô usa a pose publicada pelo Gazebo para fazer uma volta
completa de 360° para a esquerda. Durante o giro, registra o gap navegável
mais largo em pixels observado. Ao final, alinha-se com a orientação em que
esse gap foi visto e retoma a navegação.

```text
Recuar ao detectar bloqueio frontal
↓
Varrer 360° à esquerda
↓
Alinhar com o maior gap observado
↓
Retomar navegação
```

---

# Integração com Gazebo

A comunicação ocorre através do:

```text
ros_gz_bridge
```

Bridge utilizada:

```bash
./scripts/run_bridge.sh
```

## Mapeamento exploratório

O script `scripts/tigo_mapper.py` é uma opção separada do nó de navegação
`scripts/gap_phase35.py`. Ele varre 360° em cada posição, registra os gaps
passáveis observados e seleciona gaps pendentes para avançar e explorar. As
posições alcançadas e os gaps registrados são salvos em
`/tmp/tigo_map_gaps.json`.

Para usá-lo, inicie a simulação e a bridge como descrito acima e execute em
outro terminal:

```bash
source /opt/ros/humble/setup.bash
export GZ_WORLD_NAME=lacea_gap_challenge
python3 scripts/tigo_mapper.py
```

Use o mesmo `GZ_WORLD_NAME` da bridge (o padrão é `lacea_world`). O mapeador é
experimental e não modifica a configuração nem o comportamento de
`gap_phase35.py`.

---

# Testes Realizados

Os logs de execução incluem mensagens como:

```text
[NAVEGANDO] Vx=... | Wz=... | Gap=...px | Z=...m
[OBSTÁCULO] Distância: ...m. Varrendo ESQUERDA...
[SEM GAP] Nenhum gap após a varredura. Novo recuo ...
```

Os comandos de velocidade são publicados em `/cmd_vel`. A pose do modelo TIGO
em `/world/$GZ_WORLD_NAME/pose/info` é usada para medir os ângulos da varredura e
alinhar o robô com o maior gap encontrado. Configure `GZ_WORLD_NAME` tanto na
bridge quanto no nó de navegação; o valor padrão é `lacea_world`.

---

# Limitações Atuais

Ainda precisam ser validados:

- publicação da câmera virtual
- profundidade simulada
- resposta dinâmica do robô
- obstáculos virtuais

---

# Próximas Etapas

## Curto Prazo

- validar câmera virtual
- validar tópicos de profundidade

## Médio Prazo

- inserir obstáculos
- validar desvio autônomo

## Longo Prazo

- integração com SLAM
- integração com Nav2
- navegação autônoma completa

---

# Estado Atual

Concluído:

✅ Gap Following funcional

✅ Integração ROS 2

✅ Bridge configurada

✅ Simulação criada

✅ Estrutura pronta para validação completa

Em desenvolvimento:

🔄 Profundidade virtual

🔄 Navegação totalmente autônoma

🔄 Ambiente de testes com obstáculos
