#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <map>
#include <queue>
#include <algorithm>
#include <climits>

using namespace std::chrono_literals;

// Definições de tipos
using GridPos  = std::pair<int, int>;
using LocalMap = std::map<GridPos, std::string>;

// Direções básicas (ordem: cima, baixo, esquerda, direita)
const std::string MOVE_LABELS[4] = {"up", "down", "left", "right"};
const int DR[4] = {-1, 1, 0, 0};
const int DC[4] = {0, 0, -1, 1};

// Estado global auxiliar

// Chute de onde o alvo está no sistema de coordenadas relativo
GridPos goal_guess = {-999, -999};

// Funções auxiliares
std::string reverse_direction(const std::string &dir)
{
  if (dir == "up")    return "down";
  if (dir == "down")  return "up";
  if (dir == "left")  return "right";
  if (dir == "right") return "left";
  return "";
}

// Converte delta de linha/coluna para string de direção
std::string direction_from_delta(int drow, int dcol)
{
  if (drow == -1 && dcol == 0)  return "up";
  if (drow ==  1 && dcol == 0)  return "down";
  if (drow ==  0 && dcol == -1) return "left";
  if (drow ==  0 && dcol ==  1) return "right";
  return "";
}

// Envia comando de movimento e atualiza a estimativa da posição do target
bool perform_move(
  const rclcpp::Node::SharedPtr &node,
  const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr &client,
  const std::string &direction,
  const GridPos &dest_relative_pos
)
{
  auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
  request->direction = direction;

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success)
    {
      // Posições absolutas no grid do simulador
      int robot_row_abs  = response->robot_pos[0];
      int robot_col_abs  = response->robot_pos[1];
      int target_row_abs = response->target_pos[0];
      int target_col_abs = response->target_pos[1];

      int drow = target_row_abs - robot_row_abs;
      int dcol = target_col_abs - robot_col_abs;

      // Atualiza chute de onde o alvo está no sistema relativo
      goal_guess = {dest_relative_pos.first + drow,
                    dest_relative_pos.second + dcol};

      std::this_thread::sleep_for(100ms);
      return true;
    }
  }

  return false;
}


// Fase 1: Exploração DFS
void depth_first_explore(
  const GridPos &current,
  LocalMap &world_model,
  const rclcpp::Node::SharedPtr &node,
  const rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr &client
)
{
  for (int i = 0; i < 4; ++i)
  {
    GridPos neighbour{ current.first  + DR[i],
                       current.second + DC[i] };

    // Se ainda não temos informação sobre essa célula
    if (world_model.find(neighbour) == world_model.end())
    {
      // Evita entrar diretamente na célula que acreditamos ser o alvo
      if (goal_guess.first != -999 && neighbour == goal_guess)
      {
        world_model[neighbour] = "t";
        continue;
      }

      bool moved = perform_move(node, client, MOVE_LABELS[i], neighbour);

      if (moved)
      {
        // Célula livre visitada
        world_model[neighbour] = "f";

        // Explora recursivamente a partir dela
        depth_first_explore(neighbour, world_model, node, client);

        // Volta para o nó anterior (backtracking)
        perform_move(node, client, reverse_direction(MOVE_LABELS[i]), current);
      }
      else
      {
        // Se não conseguiu entrar, trata como obstáculo
        world_model[neighbour] = "b";
      }
    }
  }
}

// Programa principal
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node   = rclcpp::Node::make_shared("labyrinth_explorer_planner");
  auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

  // Aguarda o serviço de movimento
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
      return 0;

    RCLCPP_INFO(node->get_logger(), "Aguardando disponibilidade do servico 'move_command'...");
  }

  // Etapa 1: Construção do mapa
  LocalMap internal_world;
  GridPos origin{0, 0};
  internal_world[origin] = "f";

  RCLCPP_INFO(node->get_logger(), "Etapa 1: Iniciando varredura (DFS) do labirinto");
  depth_first_explore(origin, internal_world, node, client);
  RCLCPP_INFO(node->get_logger(), "Etapa 1: concluida: mapeamento finalizado");

  // Conversão Map -> Grid denso
  int min_row = INT_MAX, max_row = INT_MIN;
  int min_col = INT_MAX, max_col = INT_MIN;

  GridPos target_rel{-999, -999};

  for (const auto &entry : internal_world)
  {
    const GridPos &pos = entry.first;
    const std::string &cell = entry.second;

    if (pos.first  < min_row) min_row = pos.first;
    if (pos.first  > max_row) max_row = pos.first;
    if (pos.second < min_col) min_col = pos.second;
    if (pos.second > max_col) max_col = pos.second;

    if (cell == "t")
      target_rel = pos;
  }

  // Se não marcou explicitamente "t", usa a estimativa global
  if (target_rel.first == -999 && goal_guess.first != -999)
  {
    target_rel = goal_guess;
  }

  int rows = max_row - min_row + 1;
  int cols = max_col - min_col + 1;

  std::vector<std::vector<std::string>> dense_grid(
      rows, std::vector<std::string>(cols, "b"));

  for (const auto &entry : internal_world)
  {
    const GridPos &pos = entry.first;
    const std::string &cell = entry.second;

    int r = pos.first  - min_row;
    int c = pos.second - min_col;
    dense_grid[r][c] = cell;
  }

  if (target_rel.first != -999)
  {
    int tr = target_rel.first  - min_row;
    int tc = target_rel.second - min_col;
    if (tr >= 0 && tr < rows && tc >= 0 && tc < cols)
      dense_grid[tr][tc] = "t";
  }

  GridPos start_in_grid{0 - min_row, 0 - min_col};
  GridPos target_in_grid{-1, -1};

  if (target_rel.first != -999)
  {
    target_in_grid = {target_rel.first - min_row,
                      target_rel.second - min_col};
  }

  // Etapa 2: BFS sobre o mapa
  if (target_in_grid.first == -1)
  {
    RCLCPP_ERROR(node->get_logger(), "Nao foi possivel localizar o alvo no mapa. Encerrando.");
    rclcpp::shutdown();
    return 0;
  }

  std::queue<GridPos> fringe;
  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
  std::vector<std::vector<GridPos>> parent(rows, std::vector<GridPos>(cols, GridPos{-1, -1}));

  fringe.push(start_in_grid);
  visited[start_in_grid.first][start_in_grid.second] = true;

  bool path_found = false;

  while (!fringe.empty())
  {
    GridPos current = fringe.front();
    fringe.pop();

    if (current == target_in_grid)
    {
      path_found = true;
      break;
    }

    for (int i = 0; i < 4; ++i)
    {
      int nr = current.first  + DR[i];
      int nc = current.second + DC[i];

      if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
        continue;

      if (!visited[nr][nc] && dense_grid[nr][nc] != "b")
      {
        visited[nr][nc] = true;
        parent[nr][nc]  = current;
        fringe.push({nr, nc});
      }
    }
  }

  std::vector<GridPos> path;
  if (path_found)
  {
    GridPos step = target_in_grid;
    while (step != start_in_grid)
    {
      path.push_back(step);
      step = parent[step.first][step.second];
    }
    path.push_back(start_in_grid);
    std::reverse(path.begin(), path.end());

    RCLCPP_INFO(node->get_logger(),
                "Rota otimizada encontrada via BFS com %zu posicoes.", path.size());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Nao ha caminho valido ate o target.");
    rclcpp::shutdown();
    return 0;
  }

  // Etapa 3: Executar a rota
  RCLCPP_INFO(node->get_logger(),
              "=== ETAPA 3: Executando caminho planejado sobre o simulador ===");

  for (size_t i = 1; i < path.size(); ++i)
  {
    GridPos prev = path[i - 1];
    GridPos curr = path[i];

    int drow = curr.first  - prev.first;
    int dcol = curr.second - prev.second;

    std::string direction = direction_from_delta(drow, dcol);
    if (!direction.empty())
    {
      // Converte de volta para o sistema relative (usado no mapeamento)
      GridPos intended_internal{
          curr.first  + min_row,
          curr.second + min_col
      };

      RCLCPP_INFO(node->get_logger(),
                  "Passo %zu/%zu – comando '%s'",
                  i, path.size() - 1, direction.c_str());

      perform_move(node, client, direction, intended_internal);
    }
  }

  RCLCPP_INFO(node->get_logger(),
              "Caminho completo executado. Robo posicionado proximo ao alvo.");

  rclcpp::shutdown();
  return 0;
}
