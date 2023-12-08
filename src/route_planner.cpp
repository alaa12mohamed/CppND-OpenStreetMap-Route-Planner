#include "route_planner.h"
#include <algorithm>
#include<iostream>




RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

	start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculate the Euclidean distance from 'node' to 'end_node'
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->h_value = CalculateHValue(neighbor);
            open_list.push_back(neighbor);
            neighbor->visited = true;
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list based on the sum of h_value and g_value
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b) {
        return (a->h_value + a->g_value) > (b->h_value + b->g_value);
    });

    // Get a pointer to the node with the lowest sum
    RouteModel::Node *NextNode = open_list.back();

    // Remove the node from the open_list
    open_list.pop_back();

    return NextNode;

}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // TODO: Implement your solution here.
    while (current_node != nullptr) {
        if (current_node->parent == nullptr)
          break;
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
  	path_found.push_back(*current_node);
    // Reverse the order to start from the beginning
    std::reverse(path_found.begin(), path_found.end());


    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *CurrentNode = nullptr;

      start_node->visited = true;
      open_list.push_back(start_node);

      while (!open_list.empty()) {
          CurrentNode = NextNode();
          if (CurrentNode == end_node) {
              m_Model.path = ConstructFinalPath(CurrentNode);
              return;
          }
		 
          else{
            AddNeighbors(CurrentNode);
          }
      }
}