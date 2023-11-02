/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014)
*********************************************************************/

// 修改之处： 
// 全局变量 RESOLUTION,WIDTH_M,HEIGHT_M 的数据类型由float改为double，相应的fscanf函数参数由"%f"改为"%lf" 
// vertex 的成员变量 x,y 的数据类型由 double 改为 unsigned int，单位为像素 
// 注释掉了自定义函数itoa()，需要基本数据类型转换时，使用 boost::lexical_cast 
// 使用输出函数printf代替ROS_INFO，以使这个文件与ROS独立 


#ifndef __GETGRAPH_H__
#define __GETGRAPH_H__

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

//File Line of the First Vertex ID to read (Protection) - fscanf() ignores blank lines
#define FIRST_VID 5

typedef unsigned int uint;

typedef struct {
  uint id, num_neigh;
  unsigned int x, y;
  uint id_neigh[8], cost[8];
  bool visited[8];
  char dir [8][3];  //table of 8 strings with 3 chars max ("N","NE","E","SE","S","SW","W","NW")
}vertex;

extern uint WIDTH_PX;
extern uint HEIGHT_PX;
extern double RESOLUTION;
extern double WIDTH_M;
extern double HEIGHT_M;

uint GetGraphDimension (const char* graph_file);

void GetGraphInfo (vertex *vertex_web, uint dimension, const char* graph_file);

// uint IdentifyVertex (vertex *vertex_web, uint size, double x, double y);

//integer to array (itoa for linux c)
// char* itoa(int value, char* str, int radix);


#endif
