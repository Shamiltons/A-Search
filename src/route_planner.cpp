#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);

}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
   return node->distance(*end_node);
}




void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto i : current_node->neighbors){
        i->parent= current_node;
        i->g_value = current_node->g_value + current_node->distance(*i);
        i->h_value = CalculateHValue(i);
        open_list.push_back(i);
        i->visited = true;
    };
}




RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto &a,const auto &b){
     return (a->h_value+a->g_value)<(b->h_value+b->g_value);
    });
    RouteModel::Node* lowest = open_list.front();
    open_list.erase(open_list.begin());
    return lowest;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);
    while(current_node != start_node){
            auto *new_node = current_node->parent;
            distance += current_node->distance(*new_node);
            path_found.insert(path_found.begin(), *new_node);
            current_node = new_node;
        }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = start_node;
    current_node->visited = true;
    AddNeighbors(current_node);
    while (current_node != end_node){
        // std::cout << current_node->distance(*end_node) << "\n";
        current_node = NextNode();
        AddNeighbors(current_node);
    }
    m_Model.path = ConstructFinalPath(current_node);
}
