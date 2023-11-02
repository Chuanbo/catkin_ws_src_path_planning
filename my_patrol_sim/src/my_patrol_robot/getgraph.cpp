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


#include "getgraph.h"


uint WIDTH_PX;
uint HEIGHT_PX;
double RESOLUTION;
double WIDTH_M;
double HEIGHT_M;


uint GetGraphDimension (const char* graph_file){
  
   FILE *file;
   file = fopen (graph_file,"r");
   uint dimension;
   
   if(file == NULL){
      printf("Can not open filename %s", graph_file);
      return 0;  // 返回 dimension = 0 
   }
   else{
      int r;
      r=fscanf (file, "%u", &dimension);
      
      //Initialize other dimension variables:
      r=fscanf (file, "%u", &WIDTH_PX);
      r=fscanf (file, "%u", &HEIGHT_PX);
      r=fscanf (file, "%lf", &RESOLUTION);
      WIDTH_M = (double) WIDTH_PX * RESOLUTION;
      HEIGHT_M = (double) HEIGHT_PX * RESOLUTION;
   }
   fclose(file);
   return dimension;
}


void GetGraphInfo (vertex *vertex_web, uint dimension, const char* graph_file){
   
   FILE *file;
   file = fopen (graph_file,"r");
   
   if(file == NULL){
      printf("Can not open filename %s", graph_file);
      return;  // 返回 
   }
   else{
      uint i,j;
      double temp;
      int r;
      
      //Start Reading the File from FIRST_VID On:
      for (i=0; i<FIRST_VID-1; i++){
        r=fscanf (file, "%lf", &temp);
      }      

      for (i=0;i<dimension;i++){
        r=fscanf (file, "%u", &vertex_web[i].id);
        r=fscanf (file, "%u", &vertex_web[i].x);
        r=fscanf (file, "%u", &vertex_web[i].y);
        r=fscanf (file, "%u", &vertex_web[i].num_neigh);
        
        for (j=0;j<vertex_web[i].num_neigh; j++){
          r=fscanf (file, "%u", &vertex_web[i].id_neigh[j]);
          r=fscanf (file, "%s", &vertex_web[i].dir[j]);  // 注意：此处在编译时有一个warning，但实为指针的灵活应用，并无问题 
          r=fscanf (file, "%u", &vertex_web[i].cost[j]);
        }
      }
   }
   fclose(file);
}


/*

uint IdentifyVertex (vertex *vertex_web, uint size, double x, double y){
  
  uint i, v=0;
  double dif_x, dif_y, result=INFINITY;
  
  for (i=0; i<size; i++){
    dif_x = vertex_web[i].x - x;
    dif_y = vertex_web[i].y - y;
    
    if( result > fabs (dif_x) + fabs (dif_y) ){ //Identify the Vertex closer to the initial coordinates x & y
      result = fabs (dif_x) + fabs (dif_y);
      v = i;
    }
  }
  return v;  
}


// integer to array (itoa for linux c)
char* itoa(int value, char* str, int radix) {
    static char dig[] =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";
    int n = 0, neg = 0;
    unsigned int v;
    char* p, *q;
    char c;

    if (radix == 10 && value < 0) {
        value = -value;
        neg = 1;
    }
    v = value;
    do {
        str[n++] = dig[v%radix];
        v /= radix;
    } while (v);
    if (neg)
        str[n++] = '-';
    str[n] = '\0';

    for (p = str, q = p + (n-1); p < q; ++p, --q)
        c = *p, *p = *q, *q = c;
    return str;
}

*/

