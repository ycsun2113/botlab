#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <utils/geometric/angle_functions.hpp>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // Set start and goal nodes
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    
    // Print start and goal position
    std::cout << "start position: (" << startCell.x << ", " << startCell.y << ")" << std::endl;
    std::cout << "goal position: (" << goalCell.x << ", " << goalCell.y << ")" << std::endl;
    
    if(!found_path && distances.isCellInGrid(goalCell.x, goalCell.y)){
        // Create open and close lists
        PriorityQueue open_list;
        std::vector<Node*> closed_list;

        startNode->g_cost = 0.0;
        startNode->h_cost = h_cost(startNode, goalNode, distances);
        startNode->parent = nullptr;
        open_list.push(startNode);
        
        double kid_g_cost = 0.0;
        int iter = 0;

        while(!open_list.empty()){


            Node* current_node = open_list.pop();

            // if reach goal
            if(current_node->cell == goalNode->cell){
                found_path = true;
                goalNode = current_node;
                break;
            }

            closed_list.push_back(current_node);

            std::vector<Node*> kids = expand_node(current_node, distances, params);

            for(auto &kid : kids){
                // if kid in closed list (explored), then skip
                if(is_in_list(kid, closed_list)){
                    continue;
                }

                kid_g_cost = g_cost(current_node, kid, distances, params);
                
                // if kid not in open_list, or g_cost is lower: update
                if(!open_list.is_member(kid)  || kid_g_cost < kid->g_cost){
                    kid->parent = current_node;
                    kid->g_cost = kid_g_cost;
                    kid->h_cost = h_cost(kid, goalNode, distances);
                    open_list.push(kid);
                    if(!open_list.is_member(kid)){
                        open_list.push(kid);
                    }
                }
            }
        }
    }

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////

    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);
    h_cost = (dx + dy) + (std::sqrt(2) - 2.0) * std::min(dx, dy);

    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    double obs_cost = 0.0;
    g_cost = from->g_cost;

    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);

    bool diag = (dx==1 && dy==1);
    double move_cost = diag ? std::sqrt(2) : 1.0;
    g_cost += move_cost;

    double goal_to_obs = distances(goal->cell.x, goal->cell.y);

    if(goal_to_obs > params.minDistanceToObstacle*1.01 && goal_to_obs < params.maxDistanceWithCost){
        obs_cost += pow(params.maxDistanceWithCost/goal_to_obs, params.distanceCostExponent);
    }

    g_cost += obs_cost;

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for(int i=0; i<8; i++){
        Node neighborCell(node->cell.x + dx[i], node->cell.y + dy[i]);

        if(distances.isCellInGrid(neighborCell.cell.x, neighborCell.cell.y)){
            auto obs_dist = distances(neighborCell.cell.x, neighborCell.cell.y);

            if(obs_dist > params.minDistanceToObstacle*1.01){
                Node* newNode = new Node(neighborCell.cell.x, neighborCell.cell.y);
                // newNode->parent = node;
                children.push_back(newNode);
            }
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    
    Node* current_node = goal_node;
    
    while(current_node != nullptr){
        path.push_back(current_node);
        current_node = current_node->parent;
    }

    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    
    std::vector<Node*> new_nodes = prune_node_path(nodes);

    for(int i=0; i<new_nodes.size(); i++){
        Point<float> point = grid_position_to_global_position(new_nodes[i]->cell, distances);

        mbot_lcm_msgs::pose2D_t pose;
        pose.x = point.x;
        pose.y = point.y;
        if(i == new_nodes.size()-1){
            pose.theta = path.back().theta;
        }
        else{
            Node* current = new_nodes[i];
            Node* next = new_nodes[i+1];

            double dx = next->cell.x - current->cell.x;
            double dy = next->cell.y - current->cell.y; 

            pose.theta = wrap_to_pi(atan2(dy, dx));
        }

        path.push_back(pose);
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line

    if(nodePath.size() < 3){
        return nodePath;
    }

    new_node_path.push_back(nodePath.front());

    for(int i=1; i<nodePath.size()-1; i++){
        Node* prev = nodePath[i - 1];
        Node* current = nodePath[i];
        Node* next = nodePath[i + 1];

        // Check if the three nodes are collinear
        int dx1 = current->cell.x - prev->cell.x;
        int dy1 = current->cell.y - prev->cell.y;
        int dx2 = next->cell.x - current->cell.x;
        int dy2 = next->cell.y - current->cell.y;

        // Use cross product to check collinearity
        if (dx1 * dy2 != dy1 * dx2) {
            // If not collinear, keep the current node
            new_node_path.push_back(current);
        }
    }

    new_node_path.push_back(nodePath.back());
    
    return new_node_path;

}
