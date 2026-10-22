# Configuração do Ignition Gazebo

## Objetivo

Este documento descreve a construção do ambiente virtual utilizado pelo projeto TIGO Digital Twin.

O ambiente foi desenvolvido utilizando o Ignition Gazebo 6 com o objetivo de reproduzir o laboratório LACEA e permitir testes do robô TIGO sem necessidade de acesso ao robô físico.

---

# Estrutura do Ambiente

O ambiente virtual é composto pelos seguintes elementos:

```text
worlds/
└── lacea.sdf

urdf/
└── tigo.sdf

models/
├── LACEA_prototipo_sem_chao_v2.dae
├── chao_lacea_linha.dae
└── meshes/
    └── Tigo_para_gazebo.stl
```

---

# Mundo Virtual

Arquivo principal:

```text
worlds/lacea.sdf
```

Responsável por:

- carregar o cenário
- carregar o modelo do TIGO
- configurar iluminação
- configurar física
- configurar sensores

---

# Configuração da Física

Foi utilizada a física padrão do Ignition Gazebo.

Gravidade:

```xml
<gravity>
0 0 -9.8
</gravity>
```

---

# Plugins Utilizados

O mundo utiliza os plugins básicos do Ignition:

```xml
Physics
UserCommands
SceneBroadcaster
Sensors
```

Exemplo:

```xml
<plugin
filename="libignition-gazebo-physics-system.so"
name="ignition::gazebo::systems::Physics"/>
```

---

# Ambiente do LACEA

O ambiente foi construído utilizando o modelo:

```text
LACEA_prototipo_sem_chao_v2.dae
```

Representando:

- paredes
- corredores
- divisórias
- estrutura física do laboratório

Arquivo:

```text
models/LACEA_prototipo_sem_chao_v2.dae
```

---

# Piso da Simulação

Inicialmente foi utilizado:

```text
chao_lacea_linha.dae
```

Este modelo continha uma linha branca utilizada em projetos de seguidor de linha.

Como o objetivo atual é realizar navegação baseada em profundidade e desvio de obstáculos, o piso foi simplificado.

Foi utilizado um plano:

```xml
<plane>
    <normal>0 0 1</normal>
    <size>20 20</size>
</plane>
```

---

# Modelo do TIGO

Arquivo:

```text
urdf/tigo.sdf
```

Responsável por definir:

- estrutura do robô
- massa
- colisão
- visualização
- sensores
- integração ROS

---

# STL do Chassi

O modelo visual do TIGO foi construído utilizando:

```text
Tigo_para_gazebo.stl
```

Localização:

```text
models/meshes/Tigo_para_gazebo.stl
```

---

# Conversão de Escala

O modelo STL foi exportado em milímetros.

O Ignition Gazebo utiliza metros.

Foi aplicada a conversão:

```xml
<scale>
0.001 0.001 0.001
</scale>
```

Resultado:

```text
1 mm = 0.001 m
```

---

# Caminhos Relativos

Durante o desenvolvimento foram utilizados caminhos absolutos:

```xml
file:///home/betinha/...
```

Esses caminhos impediam a execução em outros computadores.

Após a organização do repositório, todos os caminhos foram convertidos para caminhos relativos.

Exemplo:

```xml
../models/meshes/Tigo_para_gazebo.stl
```

e

```xml
../models/LACEA_prototipo_sem_chao_v2.dae
```

---

# Inicialização do Ambiente

Executar:

```bash
ign gazebo worlds/lacea.sdf
```

Resultado esperado:

- ambiente carregado
- modelo do TIGO visível
- iluminação funcional
- física ativa

---

# Problemas Encontrados

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
Reescrita dos arquivos SDF.
```

---

## STL Não Carregado

Problema:

```text
Modelo invisível.
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

---

## Erro de URI

Erro:

```text
Unable to find uri
```

Causa:

Caminhos absolutos ou referências incorretas.

Solução:

```text
Utilização de caminhos relativos.
```

---

# Estado Atual

Concluído:

- Ambiente do LACEA carregado
- Modelo STL integrado
- Física configurada
- Estrutura do projeto organizada
- Repositório preparado para compartilhamento

---

# Próximas Melhorias

- Inserção de obstáculos dinâmicos
- Câmera de profundidade virtual
- Sensor RGB virtual
- Integração completa com Gap Following
- Integração com Nav2
- Integração com SLAM
