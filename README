# Projeto de Navegação Autônoma no Culling Games  
Mapeamento com DFS e Planejamento Ótimo com BFS

## Descrição Geral

Este projeto implementa duas abordagens distintas para navegação autônoma em um labirinto do ambiente Culling Games (ROS 2):

1. Ponderada 1: o robô recebe o mapa completo via serviço e encontra o caminho ótimo utilizando BFS.
2. Ponderada 2: o robô começa sem mapa e explora o ambiente usando DFS com backtracking, reconstrói o labirinto com base nos sensores e então aplica BFS no mapa aprendido para descobrir o caminho mais curto.

O repositório contém tanto o planejador offline (que depende do serviço de mapa), quanto o planejador online (que descobre o mapa por si só usando sensores e movimento real). Também inclui as lógicas de busca utilizadas nas aulas: BFS e DFS.

Um vídeo explicando a atividade está disponibilizado no link:
[Link do Vídeo](https://drive.google.com/file/d/1-BIvvk-8PRoo8BA59zjdraSYEoXRy0pu/view?usp=sharing)

## Conceitos Fundamentais

### BFS (Breadth-First Search)
Na Ponderada 1, o robô já conhece o mapa completo. BFS é usado porque garante a menor distância em número de passos em labirintos sem pesos. O algoritmo expande todos os vizinhos de uma célula antes de avançar para níveis mais profundos, construindo assim o caminho ótimo entre origem e destino.

### DFS (Depth-First Search) 
Na Ponderada 2, o robô não tem acesso ao mapa inicial. DFS com backtracking é usado para exploração completa do ambiente.  
A estratégia funciona da seguinte forma:  
O robô tenta avançar para qualquer vizinho desconhecido; se o movimento falha, marca a célula como bloqueada. Caso consiga entrar, marca como livre, explora recursivamente e depois retorna à célula anterior para continuar a varredura.  
Isso produz um mapa consistente a partir somente dos sensores locais.

### Combinação DFS + BFS  
Após o robô reconstruir o labirinto via DFS, o mapa é convertido para uma grade densa e BFS é aplicado sobre ela.  
A primeira técnica mapeia o ambiente.  
A segunda encontra o caminho ideal até o alvo após o mapa estar completo.

## Estrutura dos Códigos

O repositório contém dois programas principais:

### ```offline_planner.cpp```
Usa o serviço `/get_map` do ROS 2.  
Reconstrói o grid a partir do mapa retornado e aplica BFS para determinar o caminho perfeito até o alvo.  
Executa cada passo chamando `/move_command`.

### ```online_mapper.cpp```
Não conhece o mapa inicial.  
Escuta o tópico `/culling_games/robot_sensors`.  
Faz exploração ativa usando DFS com backtracking.  
Reconstrói o mapa local explorado.  
Depois aplica BFS para gerar a rota ideal.  
Por fim, com o mapa conhecido, executa a rota usando `/move_command`.

## Instalação

### 1. Pré-requisitos  
ROS 2 instalado e configurado.  
Ambiente pixi funcionando, caso esteja usando o setup sugerido no curso.  
Dependências do pacote cg_interfaces disponíveis no workspace.

### 2. Clonar o repositório  
Execute no seu workspace colcon:

```
git clone https://github.com/seu-usuario/seu-repositorio.git
```

### 3. Compilar  
Dentro do workspace:

```
colcon build --packages-select maze_planner
source install/setup.bash
```

### 4. Executar o planejador offline  
O ambiente deve estar rodando com o serviço `/get_map` disponível.

```
ros2 run maze_planner offline_planner
```

### 5. Executar o planejador online  
Certifique-se de que os sensores e o serviço `/move_command` estão disponíveis:

```
ros2 run maze_planner online_mapper
```

## Considerações finais

Este projeto demonstra como algoritmos clássicos de busca podem ser utilizados para navegação autônoma com e sem conhecimento prévio do ambiente.  
DFS permite que o robô construa seu próprio mapa a partir de percepções locais.  
BFS garante a rota ótima após o labirinto ter sido totalmente mapeado.

A estrutura modular permite que o código seja expandido para múltiplos desafios, incluindo heurísticas, navegação com pesos, ou integração com SLAM.
