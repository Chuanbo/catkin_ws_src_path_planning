#include <stdio.h>
#include <string>

#include "../my_patrol_robot/getgraph.h"
#include "../my_patrol_robot/algorithms.h"


int main()
{
//    std::string graph_name = "/home/ychb/Python_workspace/my_monitor/maps/cumberland/cumberland.graph";
    std::string graph_name = "/home/ychb/Python_workspace/my_monitor/maps/grid/grid.graph";
    
    unsigned int dimension;  // Graph Dimension 
    vertex *vertex_web;
    
    unsigned int current_vertex;
    
    // Check Graph Dimension 
    dimension = GetGraphDimension( graph_name.c_str() );
    // Create Structure to save the Graph Info 
    vertex_web = new vertex[dimension];
    // Get the Graph info from the Graph File 
    GetGraphInfo( vertex_web, dimension, graph_name.c_str() );
    
    int *path;
    int path_elements;
    
    //robot's cyclic path:
    path = new int[4*dimension];
  
    //get cyclic path:
    path_elements = cyclic(dimension, vertex_web, path);
    
    current_vertex = 0;
    
    //Shift the cyclic path to start at the current vertex:
    shift_cyclic_path (current_vertex, path, path_elements);
    
    printf("\nFinal Path: ");
    for(int i=0; i<path_elements; i++){
        if(i==path_elements-1){ printf("%i\n", path[i]); }else{ printf("%i, ", path[i]); }
    }
    printf("Number of elements = %i\n", path_elements);
    
    return 0;
}
