# Histórico de Desenvolvimento

## Projeto

TIGO Digital Twin

## Autora

Roberta Tavares

Laboratório de Controle e Automação (LACEA)

CEFET/RJ

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

# Estado Atual

Concluído:

- Ambiente virtual criado
- Modelo STL integrado
- Estrutura do LACEA carregada
- Repositório GitHub configurado
- ROS 2 Humble integrado
- Ignition Gazebo operacional
- Bridge instalada

---

# Próximos Passos

## Curto Prazo

- Configurar câmera de profundidade virtual
- Publicar tópicos de profundidade
- Validar aquisição de imagens

## Médio Prazo

- Integrar completamente o Gap Following
- Inserir obstáculos virtuais
- Testar desvio autônomo

## Longo Prazo

- SLAM
- Nav2
- Planejamento de trajetórias
- Simulação completa do TIGO
