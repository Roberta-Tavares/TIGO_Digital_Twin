# Histórico de Desenvolvimento

## Projeto

TIGO Digital Twin

## Autora

Roberta Tavares

Laboratório de Controle e Automação (LACEA)

UFCI/RJ

---

# Visão Geral

Este documento registra cronologicamente o processo de construção do gêmeo digital do robô TIGO utilizando ROS 2 Humble e Ignition Gazebo 6.

O objetivo é documentar decisões de projeto, problemas encontrados, soluções implementadas e próximos passos do desenvolvimento.

---

# Etapa 1 - Planejamento do Gêmeo Digital

O projeto teve início com o objetivo de permitir o desenvolvimento remoto do algoritmo de navegação autônoma do TIGO sem a necessidade de acesso físico ao laboratório.

Objetivos iniciais:

- Simular o robô TIGO
- Simular o ambiente do LACEA
- Permitir testes de navegação
- Integrar ROS 2 Humble
- Integrar o algoritmo Gap Following

---

# Etapa 2 - Criação do Repositório

Foi criado o repositório:

```text
TIGO_Digital_Twin
```

Estrutura inicial:

```text
TIGO_Digital_Twin
│
├── worlds
├── urdf
├── models
└── docs
```

---

# Etapa 3 - Construção do Ambiente Virtual

Foi utilizado o modelo tridimensional do laboratório LACEA.

Arquivos utilizados:

```text
LACEA_prototipo_sem_chao_v2.dae
chao_lacea_linha.dae
```

Objetivo:

Representar virtualmente:

- paredes
- corredores
- obstáculos estruturais
- espaço de navegação

---

# Etapa 4 - Remoção da Linha Branca

O modelo original continha uma linha branca utilizada anteriormente em projetos de seguidor de linha.

Problema:

```text
A linha não possui utilidade para navegação baseada em profundidade.
```

Solução:

Substituição do piso original por um plano simples renderizado pelo Gazebo.

Resultado:

```text
Ambiente mais limpo e adequado para testes de desvio de obstáculos.
```

---

# Etapa 5 - Integração do Modelo Real do TIGO

Foi incorporado o modelo CAD exportado em STL.

Arquivo:

```text
Tigo_para_gazebo.stl
```

Localização:

```text
models/meshes/
```

Objetivo:

Substituir o modelo genérico em forma de caixa por uma representação realista do robô.

---

# Etapa 6 - Correção de Escala

Problema identificado:

```text
O STL foi exportado em milímetros.
```

Como o Gazebo trabalha em metros, o modelo aparecia em escala incorreta.

Solução:

```xml
<scale>
0.001 0.001 0.001
</scale>
```

Resultado:

```text
Modelo compatível com as dimensões reais do robô.
```

---

# Etapa 7 - Primeira Inicialização do Ignition Gazebo

Comando utilizado:

```bash
ign gazebo worlds/lacea.sdf
```

Resultado:

- Ambiente carregado
- Estrutura do laboratório carregada
- Modelo do TIGO carregado

---

# Etapa 8 - Problemas Encontrados

## XML_ERROR_MISMATCHED_ELEMENT

Erro:

```text
XML_ERROR_MISMATCHED_ELEMENT
```

Causa:

```text
Tags XML abertas sem fechamento correto.
```

Solução:

```text
Reescrita completa dos arquivos SDF.
```

---

## STL Invisível

Problema:

```text
O robô não aparecia no cenário.
```

Causa:

Nome incorreto do arquivo STL.

Nome utilizado:

```text
tigo_chassi.stl
```

Nome correto:

```text
Tigo_para_gazebo.stl
```

Solução:

Atualização do caminho do modelo.

---

## Caminhos Absolutos

Problema:

Os arquivos continham referências específicas da máquina:

```text
file:///home/betinha/...
```

Impacto:

```text
O projeto funcionava apenas no computador de desenvolvimento.
```

Solução:

Substituição por caminhos relativos.

Exemplo:

```xml
../models/meshes/Tigo_para_gazebo.stl
```

Resultado:

```text
Projeto portátil para qualquer computador.
```

---

# Etapa 9 - Integração ROS 2

Foi instalada a bridge oficial:

```bash
sudo apt install ros-humble-ros-gz -y
```

Objetivo:

Permitir comunicação entre:

- ROS 2 Humble
- Ignition Gazebo

---

# Etapa 10 - Configuração da Bridge

Comando utilizado:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
/ascamera_hp60c/camera_publisher/depth0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image
```

Resultado:

```text
Bridge criada com sucesso.
```

---

# Etapa 11 - Integração Inicial do Gap Following

Foi executado:

```bash
python3 gap_phase35.py
```

Logs observados:

```text
[BLOQUEADO] Nenhum gap direto

Varrendo DIREITA

DECISÃO: Desviar para a ESQUERDA

Contorno concluído
```

Conclusão:

- O algoritmo iniciou corretamente
- Houve publicação em /cmd_vel
- A integração com a simulação ainda requer validação completa da câmera virtual

---

# Etapa 12 - Publicação no GitHub

Foi criado o repositório:

```text
TIGO_Digital_Twin
```

Foi configurado:

```bash
gh auth login
```

Resultado:

```text
Autenticação persistente configurada.
```

Os arquivos do projeto foram enviados para o GitHub.

---

# Etapa 13 - Ajustes de Orientação e Recuperação

Durante os testes, o visual do chassi e o eixo de movimento estavam desalinhados:
a malha STL tem seu comprimento no eixo local `Y`, enquanto o plugin de
movimento e a câmera usam `+X` como frente. O visual foi rotacionado em `-90°`
e a câmera virtual foi reposicionada à frente e acima do chassi.

O Gap Following recebeu recuos progressivos quando não encontra gaps. Após o
recuo, faz uma varredura temporizada para esquerda e direita, compara os scores
dos gaps e mantém o robô parado se não houver uma opção detectada após o limite
de tentativas. Essa varredura é parcial, não uma rotação de 360°.

---

# Etapa 14 - Fusão YOLO + Profundidade + GAP (Navegação Semântica)

Foi implementado o módulo de percepção semântica com Deep Learning (`scripts/yolo_depth_gap.py`),
integrando:
- Inserção de câmera RGB virtual (`ascamera_hp60c_rgb`) no modelo `urdf/tigo.sdf`.
- Atualização da ponte ROS 2 (`scripts/run_bridge.sh`) para imagens RGB.
- Detecção e classificação de obstáculos em tempo real com YOLOv8n.
- Extração de profundidade 3D dos Bounding Boxes e inflação angular de margem de segurança por classe.
- Arquitetura assíncrona multi-thread (loop de controle a 20 Hz e YOLO a 45 FPS com `imgsz=320`).
- Conversão de imagem sem `cv_bridge` para garantir compatibilidade universal entre NumPy 1.x e 2.x.
- Interface gráfica OpenCV (`--gui`) exibindo caixas delimitadoras coloridas, distância em metros e telemetria.

Documentação detalhada:
```text
docs/YOLO_DEPTH_GAP.md
```

---

# Estado Atual

Concluído:

- Ambiente virtual criado com o modelo tridimensional do LACEA
- Modelo STL integrado e visual perfeitamente alinhado ao eixo de movimento (+X)
- Câmera de profundidade virtual posicionada na dianteira operando a 15 Hz
- Câmera RGB virtual integrada e alinhada à profundidade
- ROS 2 Humble e Ignition Gazebo integrados pela ponte `run_bridge.sh`
- Algoritmo Gap Following reativo com recuo progressivo e varredura 360º (`gap_phase35.py`)
- Navegação Semântica com Desvio de Obstáculos YOLOv8n + Profundidade + GAP (`yolo_depth_gap.py`)
- Arquitetura multi-thread assíncrona (inferência a 45 FPS e controle a 20 Hz)
- Conversão de imagem independente de `cv_bridge` para compatibilidade total com NumPy 2.x

Em validação:

- Resposta do robô a diferentes classes de obstáculos semânticos (pessoas, cadeiras, caixas)
- Testes comparativos entre Gap puramente geométrico e Gap semântico em cenários dinâmicos

---

# Síntese: O Que Queremos Fazer, Como Fizemos, Problemas e Soluções

| Pilar | Descrição no Projeto TIGO Digital Twin |
| :--- | :--- |
| **O que quero fazer** | Criar um gêmeo digital do robô omnidirecional TIGO no Ignition Gazebo e ROS 2 para validação de **navegação autônoma puramente reativa com percepção semântica RGB-D**, dispensando deliberadamente SLAM ou mapas prévios do ambiente. O robô navega em tempo real por corredores livres (**Gap Following**) e reconhece classes de obstáculos (**YOLOv8**) para desvios seguros. |
| **Como fiz** | 1. Modelagem física e cinemática SDF com rodas Mecanum (`urdf/tigo.sdf`).<br>2. Cenário tridimensional do laboratório LACEA (`worlds/lacea.sdf`).<br>3. Câmera virtual RGB-D (profundidade métrica 32FC1 + RGB 15 Hz).<br>4. Ponte de comunicação ROS 2 ↔ Gazebo (`scripts/run_bridge.sh`).<br>5. Nó de controle reativo clássico (`scripts/gap_phase35.py`) com máquina de estados de recuo e varredura.<br>6. Nó de percepção semântica multi-thread (`scripts/yolo_depth_gap.py`) com Worker Thread assíncrona e inflação dinâmica de segurança por classe. |
| **Problemas encontrados** | - *Incompatibilidade de ABI:* NumPy 2.x trazido pelo YOLO causava crash com o `cv_bridge` compilado para NumPy 1.x.<br>- *Latência de CPU:* inferência síncrona do YOLO demorava 200–450 ms na CPU, estourando o watchdog de 400 ms e fazendo o robô soluçar/parar.<br>- *Desalinhamento visual do STL:* modelo com comprimento em Y vs cinemática do robô e câmera em +X.<br>- *Filtro de gaps restritivo:* descarte de passagens válidas em corredores estreitos.<br>- *Bloqueios frontais (deadlock):* robô preso de frente para paredes sem rota lateral visível.<br>- *Ruídos e NaNs no sensor virtual:* leituras infinitas quebrando cálculos matemáticos. |
| **Soluções adotadas** | - Conversão de imagem nativa direta via buffer binário (`np.frombuffer`), eliminando o `cv_bridge`.<br>- Worker Thread assíncrona para o YOLO com `imgsz=320` (inferência a 45 FPS sem atrasar o loop de controle a 20 Hz).<br>- Rotação visual de -90° no SDF e reposicionamento da câmera.<br>- Calibração do limiar mínimo para 40 pixels com pontuação ponderada.<br>- Máquina de estados de recuperação: recuo progressivo (1,8 a 2,8 s) seguido de giro 360° para buscar e alinhar com o maior gap.<br>- Conversão de NaNs/infs para teto de 10 m e filtragem por percentil estatístico. |

---

# Próximos Passos

## Curto Prazo

- Inserir pedestres e obstáculos dinâmicos no ambiente virtual
- Ajustar pesos e distâncias de inflação semântica para diferentes classes

## Médio Prazo

- Integrar modo de perseguição de alvos (Target-Driven Gap Following)
- Ensaios experimentais comparativos entre o modelo simulado e o robô físico no laboratório LACEA

## Longo Prazo

- Validação em ambientes industriais simulados complexos com múltiplos robôs
- Transferência de parâmetros calibrados na simulação diretamente para o hardware embarcado do TIGO
