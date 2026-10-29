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

O código atual identifica uma coluna como livre quando a profundidade média da
faixa central excede `DIST_OBSTACULO` (0,75 m). Um gap candidato precisa ter pelo
menos `LARGURA_ROBO_PX` (70 pixels). A largura física do gap não é calculada no
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

Quando a distância mínima na região central fica abaixo de `DIST_CRITICA`
(0,45 m), o robô recua antes da varredura. Se não houver gap direto ou a
varredura não encontrar gaps, o ciclo de recuo e busca pode ser repetido até
três tentativas. O recuo começa em 1,8 s e aumenta 0,5 s por tentativa, até
2,8 s; depois disso, o robô para por segurança.

Quando um obstáculo está entre `DIST_CRITICA` (0,45 m) e
`DIST_OBSTACULO` (0,75 m), o robô inicia a varredura temporizada: gira para a
esquerda por 1,3 s e retorna para a direita por 2,6 s, comparando os scores
dos gaps observados. Essa busca não é uma volta completa de 360 graus.

```text
Recuar quando necessário
↓
Varrer esquerda e direita parado
↓
Escolher o lado com maior score
↓
Contornar e retomar a navegação
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

---

# Testes Realizados

Os logs de execução incluem mensagens como:

```text
[NAVEGANDO] Vx=... | Wz=... | Gap=...px | Z=...m
[OBSTÁCULO] Distância: ...m. Varrendo ESQUERDA...
[SEM GAP] Nenhum gap após a varredura. Novo recuo ...
```

Os comandos de velocidade são publicados em `/cmd_vel`. A odometria está
disponível na bridge para monitoramento, mas o Gap Following atual não a usa
para medir ângulos de varredura.

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
