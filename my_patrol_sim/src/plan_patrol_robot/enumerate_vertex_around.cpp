#include <stdio.h>
#include <string>
#include <set>

#include "../my_patrol_robot/getgraph.h"
#include "../my_patrol_robot/algorithms.h"
#include "../my_patrol_robot/my_algorithms.h"


int main()
{
    std::string graph_name = "/home/ychb/Python_workspace/my_monitor/maps/cumberland/cumberland.graph";
//    std::string graph_name = "/home/turtlebot/Python_workspace/my_monitor/maps/grid/grid.graph";
    
    unsigned int dimension;  // Graph Dimension 
    vertex *vertex_web;
    
    unsigned int current_vertex = 2;
    
    // Check Graph Dimension 
    dimension = GetGraphDimension( graph_name.c_str() );
    // Create Structure to save the Graph Info 
    vertex_web = new vertex[dimension];
    // Get the Graph info from the Graph File 
    GetGraphInfo( vertex_web, dimension, graph_name.c_str() );
    
    // 考虑目标节点的周围节点的情况 
    std::set<unsigned int> vertex_set;
    std::set<unsigned int>::iterator it_set;
    
  /*
    // within 3 hops 
    for(int k1=0; k1<vertex_web[ current_vertex ].num_neigh; ++k1)
    {
        for(int k2=0; k2<vertex_web[ vertex_web[ current_vertex ].id_neigh[k1] ].num_neigh; ++k2)
        {
            // add adjacent vertex to set 
            add_adjacent_vertex(vertex_set, vertex_web[ vertex_web[ current_vertex ].id_neigh[k1] ].id_neigh[k2], vertex_web);
        }
    }
  */
    
//  /*
    // within 2 hops 
    for(int k=0; k<vertex_web[ current_vertex ].num_neigh; ++k)
    {
        // add adjacent vertex to set 
        add_adjacent_vertex(vertex_set, vertex_web[ current_vertex ].id_neigh[k], vertex_web);
    }
//  */
    
    printf("vertexs around current vertex %i :", current_vertex);
    // 枚举(遍历)并输出目标节点的周围节点 
    for(it_set = vertex_set.begin(); it_set != vertex_set.end(); ++it_set)
    {
        printf(" %i", (*it_set));
    }
    printf("\n");
    
    return 0;
}
