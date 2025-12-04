#include <memory>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;


// Alias para facilitar leitura
using GetMapSrv = cg_interfaces::srv::GetMap;
using MoveCmdSrv = cg_interfaces::srv::MoveCmd;

// Estrutura simples para representar uma célula no grid
struct Node
{
    int row;
    int col;
};


// Função BFS para encontrar o menor caminho no grid
bool find_shortest_path(
    const std::vector<std::vector<char>> &grid,
    const Node &start,
    const Node &goal,
    std::vector<Node> &out_path)
{
    int rows = grid.size();
    if (rows == 0) return false;
    int cols = grid[0].size();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Node>> parent(rows, std::vector<Node>(cols, {-1, -1}));

    std::queue<Node> q;
    q.push(start);
    visited[start.row][start.col] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    bool reached = false;

    while (!q.empty())
    {
        Node cur = q.front();
        q.pop();

        if (cur.row == goal.row && cur.col == goal.col)
        {
            reached = true;
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            int nr = cur.row + dr[i];
            int nc = cur.col + dc[i];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;

            char cell = grid[nr][nc];

            if (cell == 'b')  // parede
                continue;

            if (!visited[nr][nc])
            {
                visited[nr][nc] = true;
                parent[nr][nc] = cur;
                q.push({nr, nc});
            }
        }
    }

    if (!reached) return false;

    // Reconstrução do caminho
    std::vector<Node> reversed;
    Node cur = goal;

    while (!(cur.row == start.row && cur.col == start.col))
    {
        reversed.push_back(cur);
        cur = parent[cur.row][cur.col];
    }
    reversed.push_back(start);

    std::reverse(reversed.begin(), reversed.end());
    out_path = reversed;
    return true;
}


// Converte pares de coordenadas em comandos de direção
std::vector<std::string> convert_path_to_commands(const std::vector<Node> &path)
{
    std::vector<std::string> cmds;

    if (path.size() < 2)
        return cmds;

    for (size_t i = 0; i + 1 < path.size(); i++)
    {
        int dr = path[i+1].row - path[i].row;
        int dc = path[i+1].col - path[i].col;

        if (dr == -1 && dc == 0)
            cmds.push_back("up");
        else if (dr == 1 && dc == 0)
            cmds.push_back("down");
        else if (dr == 0 && dc == -1)
            cmds.push_back("left");
        else if (dr == 0 && dc == 1)
            cmds.push_back("right");
        else
            cmds.push_back("unknown"); // fallback
    }

    return cmds;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("offline_path_executor");

    RCLCPP_INFO(node->get_logger(), "Planner offline iniciado.");

    
    // Etapa 1: Conexão com serviços
    auto map_client = node->create_client<GetMapSrv>("/get_map");
    auto move_client = node->create_client<MoveCmdSrv>("/move_command");

    RCLCPP_INFO(node->get_logger(), "Aguardando serviço /get_map...");

    while (!map_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "ROS finalizado antes da resposta.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Ainda esperando /get_map...");
    }

   
    // Etapa 2: Solicitar mapa completo
    auto map_request = std::make_shared<GetMapSrv::Request>();
    auto future_map = map_client->async_send_request(map_request);

    if (rclcpp::spin_until_future_complete(node, future_map)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao chamar /get_map.");
        rclcpp::shutdown();
        return 1;
    }

    auto map_response = future_map.get();

    int rows = map_response->occupancy_grid_shape[0];
    int cols = map_response->occupancy_grid_shape[1];

    RCLCPP_INFO(node->get_logger(), "Mapa recebido: %dx%d", rows, cols);

  
    // Etapa 3: Construção do grid e localização de start/goal
    std::vector<std::vector<char>> grid(rows, std::vector<char>(cols));
    Node start{-1, -1};
    Node goal{-1, -1};

    for (int idx = 0; idx < rows * cols; idx++)
    {
        int r = idx / cols;
        int c = idx % cols;

        char ch = map_response->occupancy_grid_flattened[idx].empty()
                  ? 'b'
                  : map_response->occupancy_grid_flattened[idx][0];

        grid[r][c] = ch;

        if (ch == 'r') start = {r, c};
        if (ch == 't') goal = {r, c};
    }

    if (start.row == -1 || goal.row == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Start ou Goal não encontrados.");
        return 1;
    }

    // Etapa 4: Execução BFS
    std::vector<Node> full_path;

    bool has_path = find_shortest_path(grid, start, goal, full_path);

    if (!has_path)
    {
        RCLCPP_ERROR(node->get_logger(), "Nenhum caminho possível até o target.");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
                "Menor rota contém %zu células.", full_path.size());

    auto commands = convert_path_to_commands(full_path);

    RCLCPP_INFO(node->get_logger(),
                "Total de comandos de movimento: %zu", commands.size());

    // Etapa 5: Execução do caminho célula a célula
    while (!move_client->wait_for_service(1s))
    {
        RCLCPP_INFO(node->get_logger(), "Aguardando /move_command...");
    }

    for (size_t i = 0; i < commands.size(); i++)
    {
        RCLCPP_INFO(node->get_logger(),
                    "Executando passo %zu/%zu — direção: %s",
                    i + 1, commands.size(), commands[i].c_str());

        auto req = std::make_shared<MoveCmdSrv::Request>();
        req->direction = commands[i];

        auto fut = move_client->async_send_request(req);

        if (rclcpp::spin_until_future_complete(node, fut)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Erro ao enviar comando %s.",
                         commands[i].c_str());
            break;
        }

        auto resp = fut.get();

        if (!resp->success)
        {
            RCLCPP_WARN(node->get_logger(),
                        "Movimento falhou em %s. Robot=(%d,%d), Target=(%d,%d)",
                        commands[i].c_str(),
                        resp->robot_pos[0], resp->robot_pos[1],
                        resp->target_pos[0], resp->target_pos[1]);
        }

        rclcpp::sleep_for(100ms);
    }

    // Etapa 6 — Finalização
    RCLCPP_INFO(node->get_logger(),
                "Execução do plano offline concluída com sucesso.");

    rclcpp::shutdown();
    return 0;
}
