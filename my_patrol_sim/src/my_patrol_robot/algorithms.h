/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014)
*********************************************************************/


#ifndef OTHER_ALGORITHMS_H_
#define OTHER_ALGORITHMS_H_


typedef unsigned int uint;


typedef struct {
  int id, dist, elem_path;
  int path[1000];
  bool visit;
}s_path;

typedef struct {
  int id, elem_path;
  double dist;
  int path[1000];
  bool visit;
}s_path_mcost;


uint random (uint current_vertex, vertex *vertex_web);

uint conscientious_reactive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness);

uint heuristic_conscientious_reactive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness);

uint greedy_bayesian_strategy (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, double G1, double G2, double edge_min);

int count_intention (uint vertex, int *tab_intention, int nr_robots);

uint state_exchange_bayesian_strategy (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, int *tab_intention, int nr_robots, double G1, double G2, double edge_min);

void dijkstra( uint source, uint destination, int *shortest_path, uint &elem_s_path, vertex *vertex_web, uint dimension);

int is_neigh(uint vertex1, uint vertex2, vertex *vertex_web, uint dimension);

void dijkstra_mcost( uint source, uint destination, int *shortest_path, uint &elem_s_path, vertex *vertex_web, double new_costs[][8], uint dimension);

uint heuristic_pathfinder_conscientious_cognitive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, uint dimension, uint *path);

//Check if an element belongs to a table
bool pertence (int elemento, int *tab, int tam_tab);

int pertence_idx (int elemento, int *tab, int tam_tab);

bool UHC (vertex *vertex_web, int v1, int *caminho_principal, uint dimension);

void clear_visited (vertex *vertex_web, uint dimension);

bool procurar_ciclo (vertex *vertex_web, uint dimension, int *caminho_principal, int &elem_caminho, int &custo_max, int seed);

int devolve_viz_unicos (vertex *vertex_web, int vertice);

int longest_path (vertex *vertex_web, int origem, int destino, int *lista_v1v, int i_list, int dimension, int seed, int *caminho_parcial, int &elem_cp);

bool caminho_apartir_vizinhos_unicos (vertex *vertex_web, int dimension, int *caminho_principal, int &elem_max, int &custo_max, int seed);

bool verificar_arco_cp (int no_1, int no_2, int *caminho_principal, int elem_cp);

bool check_visited (vertex *vertex_web, int dimension);

int devolve_vizinhos_nao_visitados (vertex *vertex_web, int dimension, int vertice);

bool computar_caminho_de_ida (vertex *vertex_web, int dimension, int *caminho_de_ida, int &elem_c_ida, int *caminho_principal, int elem_cp, int num_arcos);

int computar_custo_caminho_final (vertex *vertex_web, int *caminho_final, int elem_caminho_final);

int cyclic (uint dimension, vertex *vertex_web, int *caminho_final);

void shift_cyclic_path (uint start_vertex, int *caminho_final, int elem_caminho_final);

uint get_MSP_dimension (const char* msp_file);

void get_MSP_route (uint *route, uint dimension, const char* msp_file);


#endif  // OTHER_ALGORITHMS_H_
