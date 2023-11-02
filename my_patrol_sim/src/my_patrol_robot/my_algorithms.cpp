#include <ctime>
#include <climits>
#include <cmath>
#include <iostream>
#include <set>

#include "getgraph.h"
#include "my_algorithms.h"

#include "algorithms.h"


/*
  The header <climits> (limits.h) defines constants with the limits of fundamental integral types for the specific system and compiler implementation used. 
  The limits for fundamental floating-point types are defined in <cfloat> (<float.h>). 
  The limits for width-specific integral types and other typedef types are defined in <cstdint> (<stdint.h>). 
*/


unsigned int expected_reactive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg)
{
    // number of neighbors of current vertex (number of existing possibilites) 
    unsigned int num_neighs = vertex_web[current_vertex].num_neigh;
    unsigned int next_vertex;
    
    if (num_neighs > 1)
    {
        double decision_table [num_neighs];
        unsigned int neighbors [num_neighs];
        unsigned int possibilities [num_neighs];
        
        unsigned int hits = 0;
        unsigned int i = 0;
        
        double max_utility = -1;  // 由于 decision_table 中的值已经过 fabs 运算，所以 -1 一定小于这些值 
        
        double x_target, y_target, distance_linear;
        double delta_time;
        double estimate_time;
        for (i=0; i<num_neighs; i++)
        {
            neighbors[i] = vertex_web[current_vertex].id_neigh[i];  // neighbors table 
            
            // 估计到达目标所需的时间 
            x_target = ( vertex_web[ neighbors[i] ].x ) * RESOLUTION;
            y_target = ( vertex_web[ neighbors[i] ].y ) * RESOLUTION;
            distance_linear = sqrt(pow((x_target - x_robot), 2) + pow((y_target - y_robot), 2));  // linear distance 
            if(distance_linear < edge_avg)  distance_linear = edge_avg;
            delta_time = distance_linear / vel_robot;
            estimate_time = current_time + delta_time;  // 对到达目标的时间的估计 
            
            // estimated idleness table 
//            decision_table[i] = estimate_time - estimate_last_visit[ neighbors[i] ];  // 未取绝对值 
            decision_table[i] = fabs( estimate_time - estimate_last_visit[ neighbors[i] ] ) / delta_time;  // 已取绝对值 
            
            // choose the one with maximum estimated idleness 
            if (decision_table[i] > max_utility)
            {
                max_utility = decision_table[i];  // maximum idleness 
                hits = 0;
                possibilities[hits] = neighbors[i];
            }
            else if (decision_table[i] == max_utility)
            {
                hits++;
                possibilities[hits] = neighbors[i];
            }
        }
        
        if (hits > 0)
        {
            // more than one possibility (choose at random) 
            srand ( time(NULL) );
            i = rand() % (hits+1) + 0;  // 0, ... ,hits 
            next_vertex = possibilities[i];  // random vertex with higher idleness 
        }
        else
        {
            // only one possibility 
            next_vertex = possibilities[hits];  // vertex with higher idleness 
        }
    }
    else
    {
        next_vertex = vertex_web[current_vertex].id_neigh[0];  // only one neighbor 
    }
    
    return next_vertex;
}


unsigned int expected_cognitive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension, double *instantaneous_idleness, unsigned int & goal_vertex, double & goal_time)
{
    unsigned int next_vertex;
    unsigned int far_vertex;
    double estimate_time;
    
    unsigned int elem_s_path;
    int *shortest_path = new int[dimension];
    while(true)
    {
        // 找出所有节点中空闲率最大的节点 
        far_vertex = 0;
        double idleness_max = instantaneous_idleness[0];
        for(int i=1; i<dimension; ++i)
        {
            if( instantaneous_idleness[i] > idleness_max )
            {
                far_vertex = i;
                idleness_max = instantaneous_idleness[i];
            }
        }
        // 计算到目标节点(far_vertex)的距离 
        double distance_far = 0;
        elem_s_path = 0;  // 每次循环都置零，用以判断空闲率最大的节点是否是当前节点或与当前节点相邻 (dijkstra算法是否被调用) 
        if(far_vertex != current_vertex)
        {
            int id_neigh = is_neigh(current_vertex, far_vertex, vertex_web, dimension);  // in algorithms.h 
            if(id_neigh >= 0)
            {
                // 如果两节点相邻，则可直接得到路径长度 (neighbors) 
                distance_far = vertex_web[ current_vertex ].cost[id_neigh];
            }
            else
            {
                // 如果两节点不相邻，则使用dijkstra算法计算最短路径 
                dijkstra( current_vertex, far_vertex, shortest_path, elem_s_path, vertex_web, dimension);
                distance_far = 0;
                for(unsigned int j=0; j<(elem_s_path-1); ++j)
                {
                    id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
                    distance_far += vertex_web[shortest_path[j]].cost[id_neigh];
                }
            }
        }
        // 对到达目标节点的时间的估计 
        estimate_time = current_time + distance_far / vel_robot;
        
        if( ( estimate_last_visit[far_vertex] > current_time ) && ( estimate_time >= estimate_last_visit[far_vertex] ) )
        {
            // 将当前选择节点的idleness置零，再重新选择idleness最大的节点 
            instantaneous_idleness[far_vertex] = 0;
        }
        else
        {
            // 已找到合适的目标节点，跳出选择循环 
            break;
        }
    }
    
    if(elem_s_path == 0)
    {
        // dijkstra算法未被调用，空闲率最大的节点是当前节点或与当前节点相邻 
        goal_vertex = far_vertex;  // 远处的目标节点，使用引用参数返回 
        goal_time = estimate_time;  // 估计到达目标节点所需的时间，使用引用参数返回 
        next_vertex = far_vertex;  // 下一步要到达的节点，作为函数的返回参数 
    }
    else
    {
        goal_vertex = far_vertex;  // 远处的目标节点，使用引用参数返回 
        goal_time = estimate_time;  // 估计到达目标节点所需的时间，使用引用参数返回 
        next_vertex = shortest_path[1];  // 计划的第一步，下一步要到达的节点，作为函数的返回参数 
        
        if( estimate_last_visit[next_vertex] > current_time )
        {
            // 如果下一个目标点与其他机器人有冲突，则使用 expected_reactive 算法重新选择以避免冲突 
            next_vertex = expected_reactive(current_vertex, vertex_web, current_time, x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg);
        }
    }
    delete [] shortest_path;
    
    return next_vertex;
}


unsigned int expected_cognitive_around (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension, double *instantaneous_idleness, unsigned int & goal_vertex, double & goal_time)
{
    unsigned int next_vertex;
    unsigned int far_vertex;
    double estimate_time;
    
    // 考虑目标节点的周围节点的情况 
    std::set<unsigned int> vertex_set;
    std::set<unsigned int>::iterator it_set;
    
    for(int k1=0; k1<vertex_web[ current_vertex ].num_neigh; ++k1)
    {
        for(int k2=0; k2<vertex_web[ vertex_web[ current_vertex ].id_neigh[k1] ].num_neigh; ++k2)
        {
            // add adjacent vertex to set 
            add_adjacent_vertex(vertex_set, vertex_web[ vertex_web[ current_vertex ].id_neigh[k1] ].id_neigh[k2], vertex_web);
        }
    }
    
    unsigned int elem_s_path;
    int *shortest_path = new int[dimension];
    while(true)
    {
        // 找出目标节点的周围节点中空闲率最大的节点 
        far_vertex = *(vertex_set.begin());
        double idleness_max = instantaneous_idleness[far_vertex];
        for(it_set = vertex_set.begin(); it_set != vertex_set.end(); ++it_set)
        {
            if(instantaneous_idleness[*it_set] > idleness_max)
            {
                far_vertex = *it_set;
                idleness_max = instantaneous_idleness[*it_set];
            }
        }
        
        // 计算到目标节点(far_vertex)的距离 
        double distance_far = 0;
        elem_s_path = 0;  // 每次循环都置零，用以判断空闲率最大的节点是否是当前节点或与当前节点相邻 (dijkstra算法是否被调用) 
        if(far_vertex != current_vertex)
        {
            int id_neigh = is_neigh(current_vertex, far_vertex, vertex_web, dimension);  // in algorithms.h 
            if(id_neigh >= 0)
            {
                // 如果两节点相邻，则可直接得到路径长度 (neighbors) 
                distance_far = vertex_web[ current_vertex ].cost[id_neigh];
            }
            else
            {
                // 如果两节点不相邻，则使用dijkstra算法计算最短路径 
                dijkstra( current_vertex, far_vertex, shortest_path, elem_s_path, vertex_web, dimension);
                distance_far = 0;
                for(unsigned int j=0; j<(elem_s_path-1); ++j)
                {
                    id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
                    distance_far += vertex_web[shortest_path[j]].cost[id_neigh];
                }
            }
        }
        // 对到达目标节点的时间的估计 
        estimate_time = current_time + distance_far / vel_robot;
        
        if( ( estimate_last_visit[far_vertex] > current_time ) && ( estimate_time >= estimate_last_visit[far_vertex] ) )
        {
            // 将当前选择节点的idleness置零，再重新选择idleness最大的节点 
            instantaneous_idleness[far_vertex] = 0;
        }
        else
        {
            // 已找到合适的目标节点，跳出选择循环 
            break;
        }
    }
    
    if(elem_s_path == 0)
    {
        // dijkstra算法未被调用，空闲率最大的节点是当前节点或与当前节点相邻 
        goal_vertex = far_vertex;  // 远处的目标节点，使用引用参数返回 
        goal_time = estimate_time;  // 估计到达目标节点所需的时间，使用引用参数返回 
        next_vertex = far_vertex;  // 下一步要到达的节点，作为函数的返回参数 
    }
    else
    {
        goal_vertex = far_vertex;  // 远处的目标节点，使用引用参数返回 
        goal_time = estimate_time;  // 估计到达目标节点所需的时间，使用引用参数返回 
        next_vertex = shortest_path[1];  // 计划的第一步，下一步要到达的节点，作为函数的返回参数 
        
        if( estimate_last_visit[next_vertex] > current_time )
        {
            // 如果下一个目标点与其他机器人有冲突，则使用 expected_reactive 算法重新选择以避免冲突 
            next_vertex = expected_reactive(current_vertex, vertex_web, current_time, x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg);
        }
    }
    delete [] shortest_path;
    
    return next_vertex;
}


unsigned int mixed_reactive (unsigned int current_vertex, vertex *vertex_web, double current_time, double x_robot, double y_robot, double vel_robot, double *estimate_last_visit, double edge_avg, unsigned int dimension)
{
    // number of neighbors of current vertex (number of existing possibilites) 
    unsigned int num_neighs = vertex_web[current_vertex].num_neigh;
    unsigned int next_vertex;
    
    if (num_neighs > 1)
    {
        double decision_table [num_neighs];
        unsigned int neighbors [num_neighs];
        unsigned int possibilities [num_neighs];
        
        unsigned int hits = 0;
        unsigned int i = 0;
        
        double max_utility = -1;  // 由于 decision_table 中的值已经过 fabs 运算，所以 -1 一定小于这些值 
        
        double x_target, y_target, distance_linear;
        double delta_time;
        double estimate_time;
        for (i=0; i<num_neighs; i++)
        {
            neighbors[i] = vertex_web[current_vertex].id_neigh[i];  // neighbors table 
            
            // 估计到达目标所需的时间 
            x_target = ( vertex_web[ neighbors[i] ].x ) * RESOLUTION;
            y_target = ( vertex_web[ neighbors[i] ].y ) * RESOLUTION;
            distance_linear = sqrt(pow((x_target - x_robot), 2) + pow((y_target - y_robot), 2));  // linear distance 
            if(distance_linear < edge_avg)  distance_linear = edge_avg;
            delta_time = distance_linear / vel_robot;
            estimate_time = current_time + delta_time;  // 对到达目标的时间的估计 
            
            // 考虑目标节点的周围节点的情况 
            std::set<unsigned int> vertex_set;
            std::set<unsigned int>::iterator it_set;
            
            for(int k1=0; k1<vertex_web[ neighbors[i] ].num_neigh; ++k1)
            {
                for(int k2=0; k2<vertex_web[ vertex_web[ neighbors[i] ].id_neigh[k1] ].num_neigh; ++k2)
                {
                    // add adjacent vertex to set 
                    add_adjacent_vertex(vertex_set, vertex_web[ vertex_web[ neighbors[i] ].id_neigh[k1] ].id_neigh[k2], vertex_web);
                }
            }
            
            // 找出目标节点的周围节点中空闲率最大的节点 
            unsigned int far_vertex = *(vertex_set.begin());
            double idleness_max = current_time - estimate_last_visit[far_vertex];
            double idleness_tmp;
            for(it_set = vertex_set.begin(); it_set != vertex_set.end(); ++it_set)
            {
                idleness_tmp = current_time - estimate_last_visit[*it_set];
                if(idleness_tmp > idleness_max)
                {
                    far_vertex = *it_set;
                    idleness_max = idleness_tmp;
                }
            }
            
/*
            // 找出所有节点中空闲率最大的节点  find global max idleness 
            far_vertex = 0;
            idleness_max = current_time - estimate_last_visit[far_vertex];
            for(int k=1; k<dimension; ++k)
            {
                idleness_tmp = current_time - estimate_last_visit[k];
                if(idleness_tmp > idleness_max)
                {
                    far_vertex = k;
                    idleness_max = idleness_tmp;
                }
            }
*/
            
            double distance_far = 0;
            if(far_vertex != neighbors[i])
            {
                int id_neigh = is_neigh(neighbors[i], far_vertex, vertex_web, dimension);  // in algorithms.h 
                if(id_neigh >= 0)
                {
                    // 如果两节点相邻，则可直接得到路径长度 (neighbors) 
                    distance_far = vertex_web[ neighbors[i] ].cost[id_neigh];
                }
                else
                {
                    // 如果两节点不相邻，则使用dijkstra算法计算最短路径 
                    unsigned int elem_s_path;
                    int *shortest_path = new int[dimension];
                    dijkstra( neighbors[i], far_vertex, shortest_path, elem_s_path, vertex_web, dimension);
                    distance_far = 0;
                    for(unsigned int j=0; j<(elem_s_path-1); ++j)
                    {
                        id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
                        distance_far += vertex_web[shortest_path[j]].cost[id_neigh];
                    }
                    delete [] shortest_path;
                }
            }
            
            // estimated idleness table 
            decision_table[i] = fabs( estimate_time - estimate_last_visit[ neighbors[i] ] ) / delta_time;  // 已取绝对值 
            if(distance_far <= 0)
            {
                if(far_vertex == neighbors[i])
                {
                    decision_table[i] += decision_table[i];
                }
                else
                {
                    std::cout << "distance_far error !" << std::endl;
                }
            }
            else
            {
                decision_table[i] += (idleness_max * vel_robot / distance_far);
            }
            
            
            // choose the one with maximum estimated idleness 
            if (decision_table[i] > max_utility)
            {
                max_utility = decision_table[i];  // maximum idleness 
                hits = 0;
                possibilities[hits] = neighbors[i];
            }
            else if (decision_table[i] == max_utility)
            {
                hits++;
                possibilities[hits] = neighbors[i];
            }
        }
        
        if (hits > 0)
        {
            // more than one possibility (choose at random) 
            srand ( time(NULL) );
            i = rand() % (hits+1) + 0;  // 0, ... ,hits 
            next_vertex = possibilities[i];  // random vertex with higher idleness 
        }
        else
        {
            // only one possibility 
            next_vertex = possibilities[hits];  // vertex with higher idleness 
        }
    }
    else
    {
        next_vertex = vertex_web[current_vertex].id_neigh[0];  // only one neighbor 
    }
    
    return next_vertex;
}


void add_adjacent_vertex(std::set<unsigned int> & vertex_set, unsigned int current_vertex, vertex *vertex_web)
{
    vertex_set.insert(current_vertex);
    
    unsigned int num_neighs = vertex_web[current_vertex].num_neigh;
    unsigned int neighbors [num_neighs];
    
    for(int i=0; i<num_neighs; ++i)
    {
        neighbors[i] = vertex_web[current_vertex].id_neigh[i];  // neighbors table 
        vertex_set.insert( neighbors[i] );
    }
}


unsigned int my_conscientious_reactive (unsigned int current_vertex, vertex *vertex_web, double *instantaneous_idleness)
{
    // number of neighbors of current vertex (number of existing possibilites) 
    unsigned int num_neighs = vertex_web[current_vertex].num_neigh;
    unsigned int next_vertex;
    
    if (num_neighs > 1)
    {
        double decision_table [num_neighs];
        unsigned int neighbors [num_neighs];
        unsigned int possibilities [num_neighs];
        
        unsigned int hits = 0;
        unsigned int i = 0;
        double max_utility = -1;
        
        for (i=0; i<num_neighs; i++)
        {
            neighbors[i] = vertex_web[current_vertex].id_neigh[i];  // neighbors table 
            decision_table[i] = instantaneous_idleness [ neighbors[i] ];  // corresponding idleness table 
            
            // choose the one with maximum idleness 
            if (decision_table[i] > max_utility)
            {
                max_utility = decision_table[i];  // maximum idleness 
                hits = 0;
                possibilities[hits] = neighbors[i];
            }
            else if (decision_table[i] == max_utility)
            {
                hits++;
                possibilities[hits] = neighbors[i];
            }
        }
        
        if (hits > 0)
        {
            // more than one possibility (choose at random) 
            srand ( time(NULL) );
            i = rand() % (hits+1) + 0;  // 0, ... ,hits 
            next_vertex = possibilities[i];  // random vertex with higher idleness 
        }
        else
        {
            // only one possibility 
            next_vertex = possibilities[hits];  // vertex with higher idleness 
        }
    }
    else
    {
        next_vertex = vertex_web[current_vertex].id_neigh[0];  // only one neighbor 
    }
    
    return next_vertex;
}
