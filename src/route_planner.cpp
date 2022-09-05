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


//the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {


return node->distance(*end_node);

}


//the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

current_node->FindNeighbors();

for(auto& i: current_node->neighbors){

i->g_value = (current_node->g_value)+(current_node->distance(*i));
i->h_value = CalculateHValue(i);
i->parent=current_node;
i->visited = true;
open_list.push_back(i);


}

}

// Comparing the F values of two pointer of nodes
bool Compare(RouteModel::Node *node1, RouteModel::Node *node2){

return (node1->g_value+node1->h_value)>(node2->g_value+node2->h_value);

}

//the NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {

std::sort(open_list.begin(),open_list.end(),Compare);

auto *next_node = open_list.back();
open_list.pop_back();

return next_node;
    
}

//the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    

while((current_node->parent!=nullptr)){

path_found.push_back(*current_node);

distance += current_node->distance(*(current_node->parent));

current_node=(current_node->parent);

}
 
 path_found.push_back(*start_node);

//sort the path

 std::reverse(path_found.begin(), path_found.end());
   

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// the A* Search algorithm .

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

current_node=start_node;
current_node->visited=true;
current_node->g_value=0;

open_list.emplace_back(current_node);

while((open_list.size())>0){

AddNeighbors(current_node);
current_node=NextNode();

if (current_node == end_node){

    m_Model.path = ConstructFinalPath(current_node);
    break;
}
}
}