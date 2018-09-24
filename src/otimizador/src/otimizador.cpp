#include <cstdio>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <time.h>


#include "Firefly.h"
#include "AStar.h"
#include "OptimizationMethods.h"


//########################################### DEFINES ############################################
#define _USE_MATH_DEFINES

#define NUMERO_MINIMO_PFRONTEIRA 10
#define DIST_MIN 5 // 5 // Distância minina do alvo ao robô, para que seja gerado uma caminho

//#define GENETICO
//#define FIREFLY
//#define FIREFLY2
//#define ENUMERATIVO
#define SIMPLEXALG

//#define GERAFUNCAO

//#define _USE_MATH_DcEFINES

#define ESTRATEGIA_1 // Espera robô chegar ao objetivo para otimizar novamente

#define sind(x) (sin(fmod((x), 360) * M_PI / 180))
#define cosd(x) (cos(fmod((x), 360) * M_PI / 180))

#define GARE "GAv06"
#define SIMPLEND
#define SIMPLEXON
#define NOSIMPLEN

//#define FUNCAO1
//#define FUNCAO2
//#define FUNCAO3
//#define FUNCAO4

#define FUNCAOPP

//#define AMB1
#define AMB2

#define NOLINHA
#define XLS
#define NOPLOT
#define NODUMP
#define CONSO
#define PASSO
#define NOTESTE
#define ANSI

#ifndef ANSI
#pragma hdrstop
#include <condefs.h>
#endif

/* Problema */
#define FUNCAO f12
#define MAXVAR 2
#define SOLUCAO -1000000.0F

#define PI 3.14159265358979F

/* Persistencia */
#define MAXGER 1000 // 1000
#define NUMCRU 1
#define TAXERR 1.0e-10
#define MAXAVA 1000000 // 10000000

/* Populacao*/
#define PATUAL 20 // 10
#define MAXPOP 1024

/* Selecao e Cruzamento */
#define FATOR 2000

#define XALFA 0.25

/* Mutacao */
#define MNUNI 7
#define PMUTAC 10

/* Parametros do Simplex */
#define VLALFA 1.0
#define VLBETA 0.5
#define VLGAMA 2.0
#define PASSOS 1000
#define SCALE 1.0
//################################################################################################

using namespace std;
using namespace grid_map;

//=== PARAMETROS DO ALGORITMO DE VAGALUMES ====
int MaxEpocas = 250; // 150
int NumFireflies = 40;

int NumClusters = 0;

int _step = 0;
int firststep = 0;
double percent = 0.0;
int first = NumFireflies * percent / 100.0;

double *dmin;
double *dmax;

int maxiEpocas = 0;
int minEpocas = 1000000;

vector<Firefly *> populacao(NumFireflies);
//=============================================


//PARAMETROS DO FUNCAO OBJETIVO:==========

int Aval = 0;          // Numero de chamadas função objetivo
double avg_aval = 0.0; // Tempo médio de execução do algoritmo
double avg_time = 0.0; // Tempo médio de execução do algoritmo
int qtdeCA = 0;        // Qtde de vezes que o alg. foi chamado

#ifdef FUNCAO1
float alpha = 50, w1 = 1.5; // 5000
float beta = 100, w2 = 4;   // 10000
float rho = 3, w3 = 3.45;   // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAO2
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAO3
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 6;    //float beta = 10000, w2 = 4;
float rho = 300, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAO4
float alpha = 500, w1 = 1.5;
float beta = 10, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPP
float alpha = 500, w1 = 1.5;
float beta = 150, w2 = 4; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45; // 300  w3 = 3.45
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPP_2
float alpha = 500, w1 = 3.5;
float beta = 150, w2 = 15; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45;  // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPL
float alpha = 5000, w1 = 1.5;
float beta = 1000, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 2000, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPC
float alpha1 = 1500, alpha2 = 500, w1 = 1.5;
float beta = 30, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 10, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif
//========================================

using namespace std;

/************************* Variáveis de estado *****************************/
int **_map;      // Guarda estado do mapa
float **_mapa;   // Guarda estado do mapa
float **_mapobj; // Guarda estado da função objetivo
bool **pr_map;   // Guarda posições visitadas pelo robô
int **dilatedMap;

int **TargetMatrix;   // Guarda os alvos gerados a cada passo da exploração
int **PositionMatrix; // Guarda as posições alcançadas pelo robô a cada passo da exploração
int **StepMatrix;     // Guarda as posições que deveriam ser alcançadas pelo robô a cada passo da exploração

int **fronteira;     // Lista de pontos do mapa da fronteira
int **regiao_perigo; // Lista de pontos do mapa proximos ao obstaculo
int **visitados;     // Lista de pontos do mapa visitados pelo robô
int **desconhecidos; // Lista de pontos do mapa desconhecidos
/***************************************************************************/

/*************************** Variáveis axiliares ***********************************/
double pixel_size = 0.1;    // Tamanho de um pixel do mapa (resolução do mapa)
bool has_map = false;       // Indica se um mapa parcial do ambiente já foi obtido
bool has_goal = false;      // Indica se há um alvo novo para alcaçar
bool first_time = true;     //
bool gera_new_goal = false; // flag para controlar a geração de alvos
double distanceToGoal = 0;  // Distância entre o robô e o objetivo
int fi = 0;
int ri = 0;
int vi = 0;
int di = 0; // contadores das listas de pontos dos mapas
bool initVst = true;
/***********************************************************************************/

/******************** Variáveis Globais **********************/
int w; // Largura do mapa
int h; // Altura do mapa

int origin_x;
int origin_y;

double pose_x; // Posição x da pose do robô
double pose_y; // Posição y da pose do robô

double bpos[2];        // melhor posição do mapa encontrada no processo de otimização
int robot_position[2]; // TODO: Descobrir a posicao do robo;
int destino_corrente[2];

vector<double *> _path; // Caminho do robô ao alvo

/**************************************************************/

/************************** Publishers *****************************/
ros::Publisher target_pub; // Publica um novo alvo para o robô
ros::Publisher origin_pub; // Posicão do robô
ros::Publisher path_pub;   // Publica o caminho para o alvo
ros::Publisher finalize_pub;
ros::Publisher ori_target_pub;
ros::Publisher grid_cells_pub;
ros::Publisher gc_path_pub;
ros::Publisher gb_publisher;
/*******************************************************************/

/************************* CALLBACKS ****************************/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &cmap);
void currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &curr_pose);
/****************************************************************/

/************************** Funcões ****************************/

double Distance(double *p1, double *p2);
double FuncaoObjetivo(double position[2], int step);
double _FuncaoObjetivo(double position[2], int step);
void GeraMapaObjetivo(int w, int h);

double LevyFlight(int dim);
void SortPop();
void ReplacePop();
double *FireflyAlgorithm(); // Implementa o Algoritmo de Vagalumes
double *GeneticAlgorithm(int numFireflies, int dim, double *dmin, double *dmax, int seed, int maxEpocas, int step); // Implementa o Algoritmo Genético
double *SimplexAlgorithm();
void heapify(int n, int i);
void heapSort();
/***************************************************************/
int main(int argc, char **argv)
{
  /**************** Bloco de Inicialização ****************/
  dmin = (double *)malloc(2 * sizeof(double));
  dmax = (double *)malloc(2 * sizeof(double));

  ros::init(argc, argv, "otimizador");
  ros::NodeHandle n;

  cout << "Otimizador iniciou sua execução." << endl;

  //###################### Subscribers ########################
  ros::Subscriber map_sub = n.subscribe("/map", 1, mapCallback);                                // Subscreve no topico que recebe o OG do ambiente
  ros::Subscriber curr_pose_sub = n.subscribe("/vrep_ros_interface/pose", 1, currPoseCallback); // Subscreve tópico que recebe a posição corrente do robô
  ros::Subscriber costmap_sub = n.subscribe("/move_base_node/global_costmap/costmap", 1, costmapCallback);
  //################ Publishers (Definição) ###################
  path_pub = n.advertise<std_msgs::String>("/path_plan", 1, true); // Publica o caminho para o alvo
  //target_pub = n.advertise<geometry_msgs::PoseStamped>("/goalPose", 1, true);   // Publica o caminho para o alvo
  target_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true); // Publica o caminho para o alvo
  origin_pub = n.advertise<geometry_msgs::PoseStamped>("/robot_position", 1, true);        // Publica o caminho para o alvo
  finalize_pub = n.advertise<std_msgs::String>("/finalize_mapping", 1, true);              // Publica flag que para o mapeamento

  grid_cells_pub = n.advertise<nav_msgs::GridCells>("/grid_cells", 1, true);
  gc_path_pub = n.advertise<nav_msgs::GridCells>("/path_cells", 1, true);

  gb_publisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  //ori_target_pub = n.advertise<geometry_msgs::PoseStamped>("/ori_goal", 1, true);
  ros::Rate r(1);

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

/* #################### Definicão das funções ##################### */
void currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &curr_pose)
{
  if (has_goal)
  {
    geometry_msgs::PoseStamped _msg;
    //msg.header.stamp = ros::Time();
    std::string frame = "/map";
    _msg.header.frame_id = frame.c_str();

    _msg.pose.position.x = destino_corrente[0] * pixel_size + origin_x;
    _msg.pose.position.y = destino_corrente[1] * pixel_size + origin_y;
    _msg.pose.position.z = 0;
    _msg.pose.orientation = curr_pose->pose.orientation;

    //ori_target_pub.publish(_msg);
    target_pub.publish(_msg);
  }
}

int **costmap;
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &cmap)
{
  int _w = cmap->info.width;  // Largura do mapa
  int _h = cmap->info.height; // Altura do mapa

  double pixel_size = cmap->info.resolution;
  int origin_x = cmap->info.origin.position.x;
  int origin_y = cmap->info.origin.position.y;

  costmap = (int **)malloc(_h * sizeof(int *));
  for (int i = 0; i < _h; i++)
    costmap[i] = (int *)malloc(_w * sizeof(int));

  for (int i = 0; i < _h; i++)
    for (int j = 0; j < _w; j++)
      costmap[i][j] = cmap->data[i * w + j];
}

void inicilizaVariaeis(const nav_msgs::OccupancyGrid::ConstPtr &map){

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{

      // TODO: carregar variaveis locais necessárias
  w = map->info.width;  // Largura do mapa
  h = map->info.height; // Altura do mapa

  pixel_size = map->info.resolution;
  cout << "resolucao: " << pixel_size << endl;
  origin_x = map->info.origin.position.x;
  origin_y = map->info.origin.position.y;

  dmin[0] = 0;
  dmin[1] = 0;
  dmax[0] = pixel_size * h;
  dmax[1] = pixel_size * w;

  // - Inicialização das matrizes de estado -
  // => Realocação todas as vezes é necessária ?????

  if (first_time) {
    _map = (int **)malloc(h * sizeof(int *));
    _mapobj = (float **)malloc(h * sizeof(float *));
    _mapa = (float **)malloc(h * sizeof(float *));
    dilatedMap = (int **)malloc(h * sizeof(int *));

    for (int i = 0; i < h; i++) {
      _map[i] = (int *)malloc(w * sizeof(int));
      _mapobj[i] = (float *)malloc(w * sizeof(float));
      _mapa[i] = (float *)malloc(w * sizeof(float));
      dilatedMap[i] = (int *)malloc(w * sizeof(int));
    }

    // - Atualização das listas de pontos -
    fronteira = new int *[w * h];     // Lista de pontos de fronteira
    regiao_perigo = new int *[w * h]; // Lista de pontos da regiao de perigo (ocupados ou próximos de ocupados)
    visitados = new int *[w * h];
    vi = 0;                           // Lista de pontos visitados
    desconhecidos = new int *[w * h]; // Lista de pontos desconhecidos

    cout << "FIRST TIME\n"
         << endl;
  }

  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      int pixel = map->data[i * w + j];

      _map[i][j] = pixel;
      dilatedMap[i][j] = pixel;
    }
  }

  fi = 0;
  ri = 0;
  di = 0;

  nav_msgs::GridCells rdanger;
  std::string frameid = "/map";
  rdanger.header.frame_id = frameid.c_str();
  //rdanger.header.stamp = ros::Time::now();
  rdanger.cell_width = pixel_size;
  rdanger.cell_height = pixel_size;
  rdanger.cells.clear();

  int pixel;
  int aux;
  int max;
  int ci = 0;
  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      pixel = _map[i][j];
      aux = 4;

      max = 0;
      double dis;
      for (int ei = i - aux; ei <= i + aux; ++ei)
      {
        for (int ej = j - aux; ej <= j + aux; ++ej)
        {
          if (ei > 0 && ej > 0 && ei < h && ej < w)
          {
            if (_map[ei][ej] > max)
            {
              dis = sqrt((ei - i) * (ei - i) + (ej - j) * (ej - j));
              if (dis < aux)
              {
                max = _map[ei][ej];
              }
            }
          }
        }
      }

      dilatedMap[i][j] = max;

      if (_map[i][j] > 0)
      {
        dilatedMap[i][j] = _map[i][j];
      }

      if (_map[i][j] > 0)
      {
        geometry_msgs::Point c;
        //c.x = (float)i; c.y = (float)j; c.z = 0;

        c.y = (float)i * pixel_size + origin_x; //path[l][0];
        c.x = (float)j * pixel_size + origin_y; //path[l][1];
        c.z = 0;

        rdanger.cells.push_back(c);
      }
      //else dilatedMap[i][j] = pixel;

      if (_map[i][j] < 0)
        dilatedMap[i][j] = _map[i][j];
    }
  }

  grid_cells_pub.publish(rdanger);

  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      //int pixel = dilatedMap[i][j];

      int pixel = _map[i][j];

      //_map[i][j] = pixel;
      //dilatedMap[i][j] = pixel;

      if (pixel == -1)
      {
        desconhecidos[di] = new int[2]{i, j};
        di++;
      }
      else
      {
        if (pixel > 0)
        {
          regiao_perigo[ri] = new int[2]{i, j};
          ri++;
        }
        else
        { // Procura pontos desconhecidos adjacentes
          int l = j > 0 ? map->data[i * w + (j - 1)] : 0;
          int r = j < h - 1 ? map->data[i * w + (j + 1)] : 0;
          int u = i > 0 ? map->data[(i - 1) * w + j] : 0;
          int d = i < w - 1 ? map->data[(i + 1) * w + j] : 0;

          if (l == -1 || r == -1 || u == -1 || d == -1)
          {
            int p[2] = {i, j};
            fronteira[fi] = new int[2]{i, j};
            fi++;
          }
        }
      }
    }
  }

  cout << "\nTESTE DE PARADA.\n";
  cout << "numero de desconhecidos: " << di << endl;

  int NoDesc;
  #ifdef AMB1
    NoDesc = 27270;
  #endif

  #ifdef AMB2
    NoDesc = 27890;
  #endif

  if (di <= NoDesc) // amb 1: 27250 / 27270 -- amb 2: 27890
  {
    cout << "Publica finalização; " << endl;
    std_msgs::String finalize;
    finalize.data = "1";
    finalize_pub.publish(finalize);

    FILE *fpath = fopen("path.txt", "a");

    fprintf(fpath, "\n");
    fprintf(fpath, "[");
    for (int i = 0; i < vi; i++)
    {
      fprintf(fpath, "%d, %d; ", visitados[i][0], visitados[i][1]);
    }
    fprintf(fpath, "]\n");
    fclose(fpath);

    FILE *fpath2 = fopen("aval.txt", "a");
    fprintf(fpath2, "Exp 1 => pathlenght: %d \t function calls: %d \n", vi, Aval);
    fclose(fpath2);

    FILE *ftime = fopen("avg_time.txt", "a");
    fprintf(ftime, "%lf\n", avg_time / (double)qtdeCA);
    fclose(ftime);

    FILE *faval = fopen("avg_aval.txt", "a");
    fprintf(faval, "%lf\n", avg_aval / (double)qtdeCA);
    fclose(faval);

    cout << "\nO MAPEAMENTO FINALIZOU.\n";
    int aux = system("rosnode kill SLAM");
    ros::shutdown();
    return;
  }
  else
  {
    std_msgs::String finalize;
    finalize.data = "0";
    finalize_pub.publish(finalize);
  }

  //fclose(fo);

  /*
    for(int i=0; i<h; i++)
    {
        for(int j=0; j<w; j++)
        {
            cout << _map[i][j] << " ";
        }
        cout << endl;
    }
    */

  // - Pega posição do robô no OG - (TODO: Entender isso!)
  tf::TransformListener listener;
  tf::StampedTransform transform;

  try
  {
    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Twist vel_msg;

  pose_x = transform.getOrigin().getX();
  pose_y = transform.getOrigin().getY();

  //int grid_x = ((pose_x - origin_x) / pixel_size);  // posição x no Ocuppancy Grid
  //int grid_y = ((pose_y - origin_y) / pixel_size);  // posição y no Ocuppancy Grid

  int grid_x = ((pose_x - origin_x) / pixel_size); // posição x no Ocuppancy Grid
  int grid_y = ((pose_y - origin_y) / pixel_size); // posição y no Ocuppancy Grid

  robot_position[0] = grid_x;
  robot_position[1] = grid_y;

  //pr_map[grid_x][grid_y] = true;

  visitados[vi] = new int[2]{robot_position[1], robot_position[0]};
  vi++; // Adiciona um ponto a lista de visitados

  //cout << " marco 4"  << endl;
  //#define GERAFUNCAO
  #ifdef GERAFUNCAO
    FILE *fo = fopen("fobj.txt", "w");
    GeraMapaObjetivo(w, h);

    fprintf(fo, "[");
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
        fprintf(fo, "\t %.4f", _mapobj[i][j]);
      }

      if (i < h - 1)
      {
        fprintf(fo, ";\n");
      }
    }
    fprintf(fo, "]");
    fclose(fo);
  #endif

  //#define SALVAMAPA
  #ifdef SALVAMAPA
    cout << "salva mapa" << endl;
    FILE *fo = fopen("_map_.txt", "w");

    fprintf(fo, "[");
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
        if (_map[i][j] > 0)
          fprintf(fo, " %d", 1);
        else
          fprintf(fo, " %d", 0);

        //fprintf(fo, "\t %.4f", _mapobj[i][j]);
      }

      if (i < h - 1)
      {
        fprintf(fo, ";\n");
      }
    }
    fprintf(fo, "]");
    fclose(fo);
  #endif

  int destino[2];
  double dis = 0.0;

  if (!first_time)
    dis = sqrt((destino_corrente[0] - robot_position[0]) * (destino_corrente[0] - robot_position[0]) + (destino_corrente[1] - robot_position[1]) * (destino_corrente[1] - robot_position[1])); // distância entre o robô e o alvo

  // -> Se a distância é menor que a distancia minima, o alvo não é atualizado
  if (dilatedMap[destino_corrente[1]][destino_corrente[0]] != 0)
    has_goal = false;

  cout << "Distancia Alvo-Robo: " << dis << endl;
  cout << "Distancia Alvo-Robo: " << has_goal << endl;
  if (dis < DIST_MIN)
    has_goal = false;
  else
    has_goal = true;

  cout<< "1>";
  first_time = false;
  if (dilatedMap[destino_corrente[1]][destino_corrente[0]] != 0)
    has_goal = false;

  // - Gera um alvo -
  //has_goal = false;
  cout<< "1>";
  if (!has_goal)
  {
    double *s;

  #ifdef ENUMERATIVO
      /*
      GridMap gmap({"elevation"});
      gmap.setFrameId("map");
      gmap.setGeometry(Length(h*pixel_size, w*pixel_size), pixel_size);
      ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
      gmap.getLength().x(), gmap.getLength().y(),
      gmap.getSize()(0), gmap.getSize()(1));
      */

      //rOS_INFO("w*h: %i", w*h);

      clock_t start, end;
      double cpu_time_used;

      //struct timeval t1, t2;
      //double elapsedTime;

      // start timer
      // gettimeofday(&t1, NULL);
      start = clock();
      GeraMapaObjetivo(w, h);
      end = clock();

      //gettimeofday(&t2, NULL);
      //elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
      cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

      avg_time += cpu_time_used;
      qtdeCA++;

      FILE *ftime = fopen("time.txt", "a");

      fprintf(ftime, "time(ms): %lf;\n ", cpu_time_used);
      fclose(ftime);

      /*
      int l=0,c=0;
      float max = 0;
      for (GridMapIterator it(gmap); !it.isPastEnd(); ++it) {
        //Position position;
        //gmap.getPosition(*it, position);

        if(_mapobj[l][c] == 100000000) 
          gmap.at("elevation", *it) = 0;
        else
        { 
          gmap.at("elevation", *it) = _mapobj[l][c]/100;
          if(_mapobj[l][c] > max)
            max = _mapobj[l][c];
        }
        c++;

        if(c==w)
        {
          l++;
          c=0;
        }
      }

      ROS_INFO("max: %i", max);

      ros::Time t = ros::Time::now();
      gmap.setTimestamp(t.toNSec());
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(gmap, message);
      gb_publisher.publish(message);
      */
  #endif

  #ifdef FIREFLY
      cout << "Chama Firefly" << endl;
      clock_t start, end;
      double cpu_time_used;

      start = clock();
      s = FireflyAlgorithm();
      end = clock();

      FILE *faval = fopen("faval.txt", "a");
      fprintf(faval, "%d\n", Aval);
      fclose(faval);

      cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

      avg_aval += Aval;
      Aval = 0;
      avg_time += cpu_time_used;
      qtdeCA++;

      //FILE *ftime = fopen("time.txt", "a");

      //fprintf(ftime, "%lf\n", cpu_time_used);
      //fclose(ftime);

      if (bpos[0] != s[0] / pixel_size && bpos[1] != s[1] / pixel_size)
      {
        bpos[0] = s[0] / pixel_size;
        bpos[1] = s[1] / pixel_size;
      }

      destino[0] = (int)bpos[1];
      destino[1] = (int)bpos[0];

      destino_corrente[0] = destino[0];
      destino_corrente[1] = destino[1];
  #endif

  #ifdef GENETICO
      clock_t start, end;
      double cpu_time_used;

      start = clock();
      s = GeneticAlgorithm(40, 2, dmin, dmax, 0, 100, 0);
      end = clock();

      cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

      avg_time += cpu_time_used;
      qtdeCA++;

      //FILE *ftime = fopen("time.txt", "a");

      //fprintf(ftime, "%lf\n", cpu_time_used);
      //fclose(ftime);

      if (bpos[0] != s[0] / pixel_size && bpos[1] != s[1] / pixel_size)
      {
        bpos[0] = s[0] / pixel_size;
        bpos[1] = s[1] / pixel_size;
      }

      destino[0] = (int)bpos[1];
      destino[1] = (int)bpos[0];

      destino_corrente[0] = destino[0];
      destino_corrente[1] = destino[1];
  #endif

  cout << "2>";
  #ifdef SIMPLEXALG
    s = SimplexAlgorithm();

    if (bpos[0] != s[0] / pixel_size && bpos[1] != s[1] / pixel_size)
      {
        bpos[0] = s[0] / pixel_size;
        bpos[1] = s[1] / pixel_size;
      }

      destino[0] = (int)bpos[1];
      destino[1] = (int)bpos[0];

      destino_corrente[0] = destino[0];
      destino_corrente[1] = destino[1];
  #endif

  #ifdef FIREFLY2
      cout << "Chama Firefly" << endl;
      clock_t start, end;
      double cpu_time_used;

      start = clock();
      s = FireflyAlgorithm();
      end = clock();

      FILE *faval = fopen("faval.txt", "a");
      fprintf(faval, "%d\n", Aval);
      fclose(faval);

      cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

      avg_aval += Aval;
      Aval = 0;
      avg_time += cpu_time_used;
      qtdeCA++;

      //FILE *ftime = fopen("time.txt", "a");

      //fprintf(ftime, "%lf\n", cpu_time_used);
      //fclose(ftime);

      if (bpos[0] != s[0] / pixel_size && bpos[1] != s[1] / pixel_size)
      {
        bpos[0] = s[0] / pixel_size;
        bpos[1] = s[1] / pixel_size;
      }

      destino[0] = (int)bpos[1];
      destino[1] = (int)bpos[0];

      destino_corrente[0] = destino[0];
      destino_corrente[1] = destino[1];
  #endif
  }

  geometry_msgs::PoseStamped _msg;
  //msg.header.stamp = ros::Time();
  std::string frame = "/map";
  _msg.header.frame_id = frame.c_str();

  _msg.pose.position.x = destino_corrente[0] * pixel_size + origin_x; //path[l][0];
  _msg.pose.position.y = destino_corrente[1] * pixel_size + origin_y; //path[l][1];
  _msg.pose.position.z = 0;
  _msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  target_pub.publish(_msg);

  geometry_msgs::PoseStamped _msg2;
  //msg.header.stamp = ros::Time();
  frame = "/map";
  _msg2.header.frame_id = frame.c_str();

  _msg2.pose.position.x = robot_position[0] * pixel_size + origin_x; //path[l][0];
  _msg2.pose.position.y = robot_position[1] * pixel_size + origin_y; //path[l][1];
  _msg2.pose.position.z = 0;
  _msg2.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  origin_pub.publish(_msg2);
}

bool hc = false;
Firefly *campeao = new Firefly(2);

enum
{
  f12 = 0,
  gol,
  fea,
  sph,
  ros2,
  sch,
  ras,
  gri,
  mic,
  lan,
  she,
  ack
} enumFunc;
static double (*funccod[])(double[], int n) =
    {_FuncaoObjetivo};
struct DadosFuncoes
{
  char nom[5];
  double inf;
  double sup;
} limFixos[] = {{"F12", 0.0F, 19.1F}};

int funcao = FUNCAO;

void nu_mutate(double *indiv, int tamind, int tampop, int ger, int expo)
{
  int i, j;
  float fRandVal, fFactor;
  float fNewt, fNewT;
  int iExponent, iIndex;

  fRandVal = (rand() % 101 / 100.);
  iIndex = rand() % tamind;
  /* pick either the max or min. limit */
  if (fRandVal < 0.5) /* lower */
  {
    fNewt = ger;
    fNewT = MAXGER;
    //                iExponent = (int) (6*randgen(0.0, 1.0)) + 1;
    fRandVal = (rand() % 101 / 100.);
    fFactor = pow((1.0F - (fNewt / fNewT)), expo) * fRandVal;

    if (fFactor < TAXERR)
      fFactor = TAXERR;
    fFactor = fFactor * (indiv[iIndex] - limFixos[funcao].inf);
    indiv[iIndex] = indiv[iIndex] - fFactor;
  }
  else
  {
    fNewt = ger;
    fNewT = MAXGER;
    //                iExponent = (int) (6*randgen(0.0, 1.0)) + 1;
    fRandVal = (rand() % 101 / 100.);

    fFactor = pow((1.0F - (fNewt / fNewT)), expo) * fRandVal;

    if (fFactor < TAXERR)
      fFactor = TAXERR;
    fFactor = fFactor * (limFixos[funcao].sup - indiv[iIndex]);
    indiv[iIndex] = indiv[iIndex] + fFactor;
  }
}

typedef struct
{
  double var[MAXVAR];
  double fit;
} Cromossomo;

typedef struct
{
  Cromossomo indiv[MAXPOP];
  double sumFit;
  int tamPop;
  int tamInd;
  int melhor;
  int pior;
  int numMuta;
  int iguais;
  int gerMelhor;
} Populacao;

double Simplex(double (*func)(double[], int n), double start[], int n, double EPSILON,
               double scale, int MAX_IT, double ALPHA, double BETA, double GAMMA)
{

  int vs; /* vertex with smallest value */
  int vh; /* vertex with next smallest value */
  int vg; /* vertex with largest value */

  int i, j, m, row;
  int k;   /* track the number of function evaluations */
  int itr; /* track the number of iterations */

  double **v;    /* holds vertices of simplex */
  double pn, qn; /* values used to create initial simplex */
  double *f;     /* value of function at each vertex */
  double fr;     /* value of function at reflection point */
  double fe;     /* value of function at expansion point */
  double fc;     /* value of function at contraction point */
  double *vr;    /* reflection - coordinates */
  double *ve;    /* expansion - coordinates */
  double *vc;    /* contraction - coordinates */
  double *vm;    /* centroid - coordinates */
  double min;

  double fsum, favg, s, cent;

  /* dynamically allocate arrays */

  /* allocate the rows of the arrays */
  v = (double **)malloc((n + 1) * sizeof(double *));
  f = (double *)malloc((n + 1) * sizeof(double));
  vr = (double *)malloc(n * sizeof(double));
  ve = (double *)malloc(n * sizeof(double));
  vc = (double *)malloc(n * sizeof(double));
  vm = (double *)malloc(n * sizeof(double));

  /* allocate the columns of the arrays */
  for (i = 0; i <= n; i++)
  {
    v[i] = (double *)malloc(n * sizeof(double));
  }

  /* create the initial simplex */
  /* assume one of the vertices is 0,0 */

  pn = scale * (sqrt((double)n + 1) - 1 + n) / (n * sqrt((double)2));
  qn = scale * (sqrt((double)n + 1) - 1) / (n * sqrt((double)2));

  for (i = 0; i < n; i++)
  {
    v[0][i] = start[i];
  }

  for (i = 1; i <= n; i++)
  {
    for (j = 0; j < n; j++)
    {
      if (i - 1 == j)
      {
        v[i][j] = pn + start[j];
      }
      else
      {
        v[i][j] = qn + start[j];
      }
    }
  }

  /* find the initial function values */
  for (j = 0; j <= n; j++)
  {
    f[j] = func(v[j], n);
  }

  k = n + 1;

  /* print out the initial values
  fprintf("Initial Values\n");
  for (j=0;j<=n;j++) {
  fprintf("%f %f %f\n",v[j][0],v[j][1],f[j]);
  }                               */

  /* begin the main loop of the minimization */
  for (itr = 1; itr <= MAX_IT; itr++)
  {
    /* find the index of the largest value */
    vg = 0;
    for (j = 0; j <= n; j++)
    {
      if (f[j] > f[vg])
      {
        vg = j;
      }
    }

    /* find the index of the smallest value */
    vs = 0;
    for (j = 0; j <= n; j++)
    {
      if (f[j] < f[vs])
      {
        vs = j;
      }
    }

    /* find the index of the second largest value */
    vh = vs;
    for (j = 0; j <= n; j++)
    {
      if (f[j] > f[vh] && f[j] < f[vg])
      {
        vh = j;
      }
    }

    /* calculate the centroid */
    for (j = 0; j <= n - 1; j++)
    {
      cent = 0.0;
      for (m = 0; m <= n; m++)
      {
        if (m != vg)
        {
          cent += v[m][j];
        }
      }
      vm[j] = cent / n;
    }

    /* reflect vg to new vertex vr */
    for (j = 0; j <= n - 1; j++)
    {
      /*vr[j] = (1+ALPHA)*vm[j] - ALPHA*v[vg][j];*/
      vr[j] = vm[j] + ALPHA * (vm[j] - v[vg][j]);
    }
    fr = func(vr, n);
    k++;

    if (fr < f[vh] && fr >= f[vs])
    {
      for (j = 0; j <= n - 1; j++)
      {
        v[vg][j] = vr[j];
      }
      f[vg] = fr;
    }

    /* investigate a step further in this direction */
    if (fr < f[vs])
    {
      for (j = 0; j <= n - 1; j++)
      {
        /*ve[j] = GAMMA*vr[j] + (1-GAMMA)*vm[j];*/
        ve[j] = vm[j] + GAMMA * (vr[j] - vm[j]);
      }
      fe = func(ve, n);
      k++;

      /* by making fe < fr as opposed to fe < f[vs],
      Rosenbrocks function takes 63 iterations as opposed
      to 64 when using double variables. */

      if (fe < fr)
      {
        for (j = 0; j <= n - 1; j++)
        {
          v[vg][j] = ve[j];
        }
        f[vg] = fe;
      }
      else
      {
        for (j = 0; j <= n - 1; j++)
        {
          v[vg][j] = vr[j];
        }
        f[vg] = fr;
      }
    }

    /* check to see if a contraction is necessary */
    if (fr >= f[vh])
    {
      if (fr < f[vg] && fr >= f[vh])
      {
        /* perform outside contraction */
        for (j = 0; j <= n - 1; j++)
        {
          /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
          vc[j] = vm[j] + BETA * (vr[j] - vm[j]);
        }
        fc = func(vc, n);
        k++;
      }
      else
      {
        /* perform inside contraction */
        for (j = 0; j <= n - 1; j++)
        {
          /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
          vc[j] = vm[j] - BETA * (vm[j] - v[vg][j]);
        }
        fc = func(vc, n);
        k++;
      }

      if (fc < f[vg])
      {
        for (j = 0; j <= n - 1; j++)
        {
          v[vg][j] = vc[j];
        }
        f[vg] = fc;
      }
      /* at this point the contraction is not successful,
      we must halve the distance from vs to all the
      vertices of the simplex and then continue.
      10/31/97 - modified to account for ALL vertices.
      */
      else
      {
        for (row = 0; row <= n; row++)
        {
          if (row != vs)
          {
            for (j = 0; j <= n - 1; j++)
            {
              v[row][j] = v[vs][j] + (v[row][j] - v[vs][j]) / 2.0;
            }
          }
        }
        f[vg] = func(v[vg], n);
        k++;
        f[vh] = func(v[vh], n);
        k++;
      }
    }

    /* print out the value at each iteration
    fprintf("Iteration %d\n",itr);
    for (j=0;j<=n;j++) {
    fprintf("%f %f %f\n",v[j][0],v[j][1],f[j]);
    }                                        */

    /* test for convergence */
    fsum = 0.0;
    for (j = 0; j <= n; j++)
    {
      fsum += f[j];
    }
    favg = fsum / (n + 1);
    s = 0.0;
    for (j = 0; j <= n; j++)
    {
      s += pow((f[j] - favg), 2.0) / (n);
    }
    s = sqrt(s);
    if (s < EPSILON)
      break;
  }
  /* end main loop of the minimization */

  /* find the index of the smallest value */
  vs = 0;
  for (j = 0; j <= n; j++)
  {
    if (f[j] < f[vs])
    {
      vs = j;
    }
  }

  //  fprintf("The minimum was found at\n");
  for (j = 0; j < n; j++)
  {
    //    fprintf("%e\n",v[vs][j]);
    start[j] = v[vs][j];
  }
  min = func(v[vs], n);
  k++;
  //  fprintf("%d Function Evaluations\n",k);
  //  fprintf("%d Iterations through program\n",itr);

  for (i = 0; i <= n; i++)
  {
    free(v[i]);
  }

  free(f);
  free(vr);
  free(ve);
  free(vc);
  free(vm);
  free(v);
  return min;
}

float randgen(float fLlim, float fUlim)
{
  float fRandomVal;

  fRandomVal = rand() % 101 / 100.; // rand entre 0 e 1

  return (fLlim + (float)(fRandomVal * (fUlim - fLlim)));
}

void IniciaPop(Populacao *p, int m, int n, int nfun)
{
  int i, j, pior, melhor;
  double soma, fit;

  //std::cout << "MAX: " << limFixos[nfun].sup << std::endl;
  //std::cout << "3:" ;
  for (i = 0, soma = 0, pior = 0, melhor = 0; i < m; i++)
  {
    //std::cout << "3:" ;
    for (j = 0; j < n; j++)
    {
      p->indiv[i].var[j] = (double)randgen(limFixos[nfun].inf, limFixos[nfun].sup);
    }

    fit = funccod[nfun](p->indiv[i].var, n);

    p->indiv[i].fit = fit;
    if (fit > p->indiv[pior].fit)
      pior = i;
    if (fit < p->indiv[melhor].fit)
      melhor = i;
    soma += (fit); //fabs
    //d::cout << ":3" ;
  }

  //std::cout << "MAX: " << limFixos[nfun].sup << std::endl;
  p->tamPop = m;
  p->tamInd = n;
  p->sumFit = soma;
  p->melhor = melhor;
  p->pior = pior;
  p->numMuta = 0;
  p->iguais = 0;
}

void IniciaPSc(Populacao *p, int m, int n, int nfun)
{
  int i, j, k;
  int pior, melhor, ipp;
  float inf, sup, pas, par, npa;
  double soma, fit;

  npa = log((double)m) / log((double)2);
  pas = (limFixos[nfun].sup - limFixos[nfun].inf) / npa;
  inf = limFixos[nfun].inf;
  sup = ((inf + pas) < limFixos[nfun].sup ? (inf + pas) : limFixos[nfun].sup);
  ipp = m / npa;

  for (i = 0, soma = 0, pior = 0, melhor = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
      p->indiv[i].var[j] = (double)randgen(inf, sup);
    fit = funccod[nfun](p->indiv[i].var, n);
    p->indiv[i].fit = fit;
    if (fit > p->indiv[pior].fit)
      pior = i;
    if (fit < p->indiv[melhor].fit)
      melhor = i;
    soma += (fit); //fabs
    if ((i + 1) % ipp == 0)
    {
      inf = sup;
      sup = ((inf + pas) < limFixos[nfun].sup ? (inf + pas) : limFixos[nfun].sup);
    }
  }
  p->tamPop = m;
  p->tamInd = n;
  p->sumFit = soma;
  p->melhor = melhor;
  p->pior = pior;
  p->numMuta = 0;
  p->iguais = 0;
}

int Selecao(Populacao *p)
{
  int i;
  double val, spin_val;

  val = 0.0;
  spin_val = (rand() % 101) / 100. * p->sumFit;
  i = rand() % p->tamPop;

  do
  {
    i = (i < p->tamPop - 1) ? i + 1 : 0;
    val = val + fabs(FATOR * p->sumFit) / (1 + p->indiv[i].fit - p->indiv[p->melhor].fit);
  } while (val < spin_val);
  return i; // posição do estouro
}

void CruzaBlend(Populacao *p, int pai, int mae, int filho, float alfa)
{
  float a, b, r;
  int i;
  double ajuste, nureal, interv;
  int nuinte, voltas;

  a = -alfa;
  b = 1 + alfa;
  r = rand() % 101 / 100.;
  r = a + r * (b - a);

  for (i = 0; i < p->tamInd; i++)
  {
    p->indiv[filho].var[i] =
        p->indiv[pai].var[i] + r * (p->indiv[mae].var[i] - p->indiv[pai].var[i]);
    if (p->indiv[filho].var[i] < limFixos[FUNCAO].inf)
    {
      interv = limFixos[FUNCAO].sup - limFixos[FUNCAO].inf;
      nureal = (limFixos[FUNCAO].inf - p->indiv[filho].var[i]) / interv;
      nuinte = nureal;
      ajuste = (nureal - nuinte) * interv;
      voltas = nuinte % 2;
      p->indiv[filho].var[i] = limFixos[FUNCAO].inf + voltas * interv - (2 * voltas - 1) * ajuste;
    }
    else if (p->indiv[filho].var[i] > limFixos[FUNCAO].sup)
    {
      interv = limFixos[FUNCAO].sup - limFixos[FUNCAO].inf;
      nureal = (p->indiv[filho].var[i] - limFixos[FUNCAO].sup) / interv;
      nuinte = nureal;
      ajuste = (nureal - nuinte) * interv;
      voltas = 1 - nuinte % 2;
      p->indiv[filho].var[i] = limFixos[FUNCAO].inf + voltas * interv - (2 * voltas - 1) * ajuste;
    }
  }
}
void CruzaGeom(Populacao *p, int pai, int mae, int filho)
{
  int i;
  float alfa;

  alfa = rand() % 101 / 100.;

  for (i = 0; i < p->tamInd; i++)
  {
    p->indiv[filho].var[i] =
        pow((double)p->indiv[pai].var[i], (double)alfa) * pow(p->indiv[mae].var[i], 1 - (double)alfa);
  }
}

void CruzaEsfe(Populacao *p, int pai, int mae, int filho)
{
  int i;
  float alfa;

  alfa = rand() % 101 / 100.;

  for (i = 0; i < p->tamInd; i++)
  {
    p->indiv[filho].var[i] =
        sqrt(alfa * pow(p->indiv[pai].var[i], 2) + (1 - alfa) * pow(p->indiv[mae].var[i], 2));
  }
}

void AtualizaPop(Populacao *p, int pos, double fit, int ger)
{
  int i, j, max;

  p->sumFit -= (p->indiv[pos].fit); //fabs
  p->sumFit += (fit);               // fabs
  p->indiv[pos].fit = fit;

  if (fit < p->indiv[p->melhor].fit)
  {
    p->melhor = pos;
    p->gerMelhor = ger;
  }
  /* ***** procura um outro pior ******** */
  max = PATUAL * p->tamPop / 100;
  //        p->pior=p->melhor == p->tamPop ? p->melhor - 1: p->melhor +1;
  for (i = 0; i < max; i++)
  {
    j = rand() % p->tamPop;
    if (j == pos || j == p->melhor)
      continue;
    if (p->indiv[j].fit > p->indiv[p->pior].fit)
    {
      p->pior = j;
      i += PATUAL;
    }
  }
}

double SimplexFixo(double *indiv, int tam, int passos, int funcao)
{
  double erro = TAXERR;
  //double Alfa=1.1, Beta=0.5, Gama=2.2;       /* expansion coefficient */
  double Alfa = VLALFA, Beta = VLBETA, Gama = VLGAMA;
  double min;

  min = Simplex(funccod[funcao], indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
  return min;
}

double SimplexRand(double *indiv, int tam, int passos, int funcao) {
  double erro = TAXERR;
  double Alfa, Beta, Gama;
  double min;

  Alfa = randgen(VLALFA - 0.5, VLALFA + 0.5);
  Beta = randgen(VLBETA - 0.5, VLBETA + 0.5);
  Gama = randgen(VLGAMA - 0.5, VLGAMA + 0.5);

  min = Simplex(funccod[funcao], indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
  return min;
}

double* SimplexAlgorithm() {
  double erro = TAXERR;
  double Alfa, Beta, Gama;
  double min;

  Alfa = randgen(VLALFA - 0.5, VLALFA + 0.5);
  Beta = randgen(VLBETA - 0.5, VLBETA + 0.5);
  Gama = randgen(VLGAMA - 0.5, VLGAMA + 0.5);

  cout << "erro aqui";
  double * start; start[0] = 0.0; start[1] = 0.0;

  cout << "=> Executando Simplex ...";
  double* best = SimplexMethod(funccod[funcao], start, 2, erro, SCALE, 1000, Alfa, Beta, Gama);
  cout << "=> Simplex terminou a execução.";
  return best;
}

/****/
void IniciaSct(Populacao *p, int m, int n, int nfun)
{
  int i, j, k;
  int pior, melhor;
  int ipd, dim;
  double inf, sup, pas;
  double soma, fit;
  static double lista[MAXVAR];

  inf = limFixos[nfun].inf;
  sup = limFixos[nfun].sup;
  ipd = pow(m, 1.0F / (n));
  pas = (double)(sup - inf) / ipd;
  for (i = 0; i < n; i++)
    lista[i] = inf;
  dim = 0;
  i = 0;
  soma = 0;
  pior = 0;
  melhor = 0;
  while (i < m)
  {
    for (j = 0; j < n; j++)
      p->indiv[i].var[j] = (double)randgen(lista[j], lista[j] + pas);
    lista[dim] = (lista[dim] + 2 * pas > sup ? sup - pas : lista[dim] + pas);
    fit = funccod[nfun](p->indiv[i].var, n);
    p->indiv[i].fit = fit;
    if (fit > p->indiv[pior].fit)
      pior = i;
    if (fit < p->indiv[melhor].fit)
      melhor = i;
    soma += (fit);
    if ((++i % ipd) == 0)
    {
      do
      {
        lista[dim] = limFixos[nfun].inf;
        dim++;
        lista[dim] = lista[dim] + pas;
      } while (lista[dim] + pas > sup && dim < MAXVAR - 1);
      if (lista[dim] + pas > sup)
        lista[dim] = sup - pas;
      dim = 0;
    }
  }
  p->tamPop = m;
  p->tamInd = n;
  p->sumFit = soma;
  p->melhor = melhor;
  p->pior = pior;
  p->numMuta = 0;
  p->iguais = 0;
}

int n = 0;
double *GeneticAlgorithm(int numFireflies, int dim, double *dmin, double *dmax, int seed, int maxEpocas, int step)
{

  std::cout << "1:";

  double bestFitness = 10000000.0;
  double *bestPosition;

  bestPosition = (double *)malloc(2 * sizeof(double));
  clock_t start, end;

  FILE *saida = NULL, *arq = NULL;
  char nomarq[50];

  int MaxIt = PASSOS, numGeracoes = MAXGER, numCruza;
  float pMuta = PMUTAC, xalfa = XALFA;
  double erro;

  Populacao P;
  int pa1, pa2;
  double fit, dvp, med;

  int i, j;

  int semente = 0;

  #ifdef LINHA
    funcao = atoi(argv[2]);
    strcpy(nomarq, GARE);
    strcat(nomarq, limFixos[funcao].nom);
    strcat(&nomarq[strlen(limFixos[funcao].nom)], argv[1]);
    if (!(saida = fopen(nomarq, "w")))
    {
      perror("");
      exit(-1);
    }

    funcao = atoi(argv[2]);
    semente = atoi(argv[3]);
  #endif

  #ifdef NOLINHA
    saida = stdout;
  #endif

  /* randomico ou não */
  srand((unsigned)time(0) + semente);

  IniciaPop(&P, MAXPOP, MAXVAR, funcao);
  erro = (double)P.indiv[P.melhor].fit - SOLUCAO;

  std::cout << "2:";

  printf("\n***   (%s) por ACMO/CAP/LAC/INPE    ***", GARE);
  printf("\n      Evoluindo %s com %d vars", limFixos[funcao].nom, MAXVAR);
  start = clock();
  while (numGeracoes-- && Aval < MAXAVA && erro > (double)0.0F)
  {
    numCruza = NUMCRU;
    while (numCruza--)
    {
      pa1 = Selecao(&P);
      pa2 = Selecao(&P);
      if (pa1 != pa2)
        CruzaBlend(&P, pa1, pa2, P.pior, xalfa);
      //                        CruzaEsfe(&P, pa1, pa2, P.pior);
      //                        CruzaGeom(&P, pa1, pa2, P.pior);
      else
        P.iguais++;

  #ifdef SIMPLEXO
        if (pMuta > rand() % 100 && (MAXGER - numGeracoes) > (0.95 * MAXGER))
        {
          //                        fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
          fit = SimplexFixo(P.indiv[P.pior].var, P.tamInd, MaxIt, funcao);
          P.numMuta++;
        }
        else
        {
          fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
        }
  #endif
  #ifdef SIMPLEND
        if (pMuta > rand() % 100)
        {
          fit = SimplexRand(P.indiv[P.pior].var, P.tamInd, MaxIt, funcao);
          P.numMuta++;
        }
        else
        {
          fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
        }
  #endif
  #ifdef NOSIMPLE
        if (pMuta > rand() % 100)
        {
          nu_mutate(P.indiv[P.pior].var, P.tamInd, P.tamPop, MAXGER - numGeracoes, MNUNI);
          P.numMuta++;
        }
        //                P.indiv[P.pior].var[0]=2.25;
        //                P.indiv[P.pior].var[1]=1.6;

        fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
  #endif
      AtualizaPop(&P, P.pior, fit, MAXGER - numGeracoes);
  #ifdef PASSO
        if (!(numGeracoes % 20))
          printf("\r\t %d) Minimo = %.16f ", MAXGER - numGeracoes, P.indiv[P.melhor].fit);
  #endif
    }
    erro = (double)P.indiv[P.melhor].fit - SOLUCAO;
  }

  std::cout << "2\n";
  end = clock();

  /* *******************RESULTADOS ******************** */

  med = P.sumFit / P.tamPop;
  dvp = 0;
  for (i = 0; i < P.tamPop; i++)
    dvp += pow(P.indiv[i].fit - med, 2);
  dvp = sqrt(dvp / (P.tamPop - 1));

  bestPosition[0] = P.indiv[P.melhor].var[0];
  bestPosition[1] = P.indiv[P.melhor].var[1];

  #ifdef XLS

    fprintf(saida, "\n%s)Min = %.10f; Aval= %d; Tempo = %.4f; Med = %.4f; Dpd = %.4f; Ger = %d",
            nomarq, P.indiv[P.melhor].fit, Aval, (double)(end - start) / 118, med, dvp, P.gerMelhor);

  #endif

  #ifdef CONSO
    printf("\n\t Variaveis ...");
    for (i = 0; i < P.tamInd; i++)
    {
      printf("\n\t\t%.6f", P.indiv[P.melhor].var[i]);
    }

    printf("\n\t Minimo = %.16f; \n\t Na ger = %d; \n\t mutacoes = %d \n\t ; aval = %d\n\t; erro = %f\n\t; (%.4f, %.4f)\n",
          P.indiv[P.melhor].fit, P.gerMelhor, P.numMuta, Aval, erro, med, dvp);       
    printf("\n\t em tempo = %.4f, ", (double) (end - start) / CLOCKS_PER_SEC);
    //getchar();
  #endif

  std::cout << "1\n";

  return bestPosition;
}

static unsigned int g_seed;

inline void fast_srand(int seed)
{
  g_seed = seed;
}

inline int fastrand()
{
  g_seed = (214013 * g_seed + 2531011);
  return (g_seed >> 16) & 0x7FFF;
}

// Implementa Firefly's Algorithm
double *FireflyAlgorithm()
{
  double B0 = 1;      // 1
  double gamma = 0.4; // 0.4
  double alpha = 0.3; // 0.1

  double bestFitness = 10000000.0;
  double *bestPosition;

  bestPosition = (double *)malloc(2 * sizeof(double));

  fast_srand(time(NULL));
  double r01;

  int ini = 0, i, j, k;

  bool hasChange;
  double beta;

  struct timeval inicio, final;
  double tmili;

  gettimeofday(&inicio, NULL);

  double newpos[2];

  #ifdef FIREFLY2
    ini = NumClusters;
  #endif

  for (i = 0; i < NumFireflies; i++)
  {
    if (i >= ini)
    {
      populacao[i] = new Firefly(2);
      for (j = 0; j < 2; j++)
      {
        r01 = ((double)(fastrand() % 101) / 100);
        newpos[j] = (dmax[j] - dmin[j]) * r01 + dmin[j]; // inicializando as posicoes aleatoriamente
      }

      populacao[i]->SetPosicao(newpos);
      populacao[i]->bposition = newpos;
    }

    //populacao[i]->fitness = _FuncaoObjetivo(populacao[i]->GetPosicao(), _step); n++;
    //populacao[i]->bfitness = populacao[i]->fitness;

    //populacao[i]->intensity = -1 / (populacao[i]->fitness + 1);

    //if (populacao[i]->fitness < bestFitness)
    //{
    //  bestFitness = populacao[i]->fitness;
    //  for (k = 0; k < 2; k++)
    //    bestPosition[k] = populacao[i]->GetPosicao()[k];
    //}
  }

  if (hc)
  {
    populacao[NumFireflies] = campeao;
  }

  gettimeofday(&final, NULL);
  tmili = (1 * (final.tv_sec - inicio.tv_sec) * 1000 + (final.tv_usec - inicio.tv_usec) / 1);

  int bepoca = 0;
  int epoca = 0;
  n = 0;

  double r;
  double Ii;
  double Ij;

  gettimeofday(&inicio, NULL);

  Firefly *atual;

  cout << "entrou em FireflyAlgorithm..." << endl;

  //#define TESTCONVERGENCE
  #ifdef TESTCONVERGENCE
    FILE *fpath2 = fopen("fireflies.txt", "w");
    fprintf(fpath2, "[");
    for (i = 0; i < NumFireflies; i++)
    {
      fprintf(fpath2, "%f, %f;", populacao[i]->GetPosicao()[1] / 0.1, populacao[i]->GetPosicao()[0] / 0.1);
    }
    fprintf(fpath2, "]\n");
    fclose(fpath2);
  #endif

  while (epoca < MaxEpocas)
  {
    hasChange = false;

    // Evaluate:
    for (i = 0; i < NumFireflies; i++)
      populacao[i]->fitness = _FuncaoObjetivo(populacao[i]->GetPosicao(), _step);
    n++;

    /*  bubbleSort:
    //  Sort:
    SortPop();

    // Replace:
    ReplacePop();
    */

    heapSort();

    // Move:
    for (i = 0; i < NumFireflies; i++)
    {
      double *ipos = populacao[i]->GetPosicao();
      for (j = 0; j < NumFireflies; j++)
      {
        double *jpos = populacao[j]->GetPosicao();

        r = (ipos[0] - jpos[0]) * (ipos[0] - jpos[0]) +
            (ipos[1] - jpos[1]) * (ipos[1] - jpos[1]);

        r = sqrt(r) / pixel_size;

        Ii = populacao[i]->fitness;
        Ij = populacao[j]->fitness;

        if (Ij < Ii)
        {
          hasChange = true;

          beta = B0 * exp(-gamma * r * r);

          for (k = 0; k < 2; k++)
          {
            r01 = ((double)(fastrand() % 101) / 100);
            ;

            newpos[k] = ipos[k] + beta * (jpos[k] - ipos[k]) + alpha * (r01 - 0.5) * (dmax[k] - dmin[k]);

            if (newpos[k] < dmin[k])
              newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand() % 101) / 100) + dmin[k];
            if (newpos[k] > dmax[k])
              newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand() % 101) / 100) + dmin[k];
          }
          populacao[i]->SetPosicao(newpos);
        }

        if (populacao[i]->bfitness > populacao[i]->fitness)
        {
          populacao[i]->bfitness = populacao[i]->fitness;
          populacao[i]->bposition = populacao[i]->GetPosicao();
        }
      }

      if (populacao[i]->fitness < bestFitness)
      {
        bestFitness = populacao[i]->fitness;
        for (k = 0; k < 2; k++)
          bestPosition[k] = populacao[i]->GetPosicao()[k];

        campeao = populacao[i];
        hc = true;
        bepoca = epoca;
      }
    }
    epoca++;
  }

  #ifdef TESTCONVERGENCE
    fpath2 = fopen("fireflies.txt", "a");
    fprintf(fpath2, "[");
    for (i = 0; i < NumFireflies; i++)
    {
      fprintf(fpath2, "%f, %f;", populacao[i]->GetPosicao()[1] / 0.1, populacao[i]->GetPosicao()[0] / 0.1);
    }
    fprintf(fpath2, "]\n");
    fclose(fpath2);
  #endif

  if (bepoca > maxiEpocas)
  {
    maxiEpocas = bepoca;
  }

  if (bepoca < minEpocas)
  {
    minEpocas = bepoca;
  }

  #ifdef FIREFLY2
    vector<Firefly *> newpop(NumFireflies);

    // Evaluate:
    for (i = 0; i < NumFireflies; i++)
      populacao[i]->fitness = _FuncaoObjetivo(populacao[i]->GetPosicao(), _step);
    n++;

    //  Sort:
    SortPop();

    newpop[0] = populacao[0];
    k = 0;
    double RC = 130.0;
    for (i = 1; i < NumFireflies; i++)
    {
      if (newpop[k]->GetPosicao()[0] != populacao[i]->GetPosicao()[0] ||
          newpop[k]->GetPosicao()[1] != populacao[i]->GetPosicao()[1])
      {
        if (Distance(newpop[k]->GetPosicao(), populacao[i]->GetPosicao()) > RC)
        {
          k++;
          newpop[k] = populacao[i];
        }
      }
    }

    for (i = 0; i < k; i++)
    {
      populacao[i] = newpop[i];
    }

    NumClusters = k;

    cout << "NumClusters: " << NumClusters << endl;
  #endif

  n = 0;
  return bestPosition;
}

int _Index[1000];

void heapify(int n, int i)
{
  int largest = i;   //initiaze largest as root
  int l = 2 * i + 1; // left = 2*i + 1
  int r = 2 * i + 2; // right = 2*i + 2

  // If right child is larger than largest so far
  if (r < n && populacao[r]->fitness > populacao[largest]->fitness)
    largest = r;

  // If largest is not root
  if (largest != i)
  {
    swap(populacao[i], populacao[largest]);

    // Recursively heapify the affected sub-tree
    heapify(n, largest);
  }
}

void heapSort()
{
  // Build heap (rearrange array)
  for (int i = NumFireflies / 2 - 1; i >= 0; i--)
    heapify(NumFireflies, i);

  // One by one extract an element from heap
  for (int i = NumFireflies - 1; i >= 0; i--)
  {
    // Move current root to end
    swap(populacao[0], populacao[i]);

    // call max heapify on the reduced heap
    heapify(i, 0);
  }
}

void SortPop()
{
  int i, j;

  // initialization of indexes
  for (i = 0; i < NumFireflies; i++)
    _Index[i] = i;

  // Bubble sort
  for (i = 0; i < NumFireflies - 1; i++)
  {
    for (j = i + 1; j < NumFireflies; j++)
    {
      if (populacao[i]->fitness > populacao[j]->fitness)
      {
        double z = populacao[i]->fitness;
        populacao[i]->fitness = populacao[j]->fitness;
        populacao[i]->fitness = z;

        int k = _Index[i]; // exchange indexes
        _Index[i] = _Index[j];
        _Index[j] = k;
      }
    }
  }
}

void ReplacePop()
{
  int i, j;
  vector<Firefly *> popTmp = populacao;

  for (i = 0; i < NumFireflies; i++)
  {
    populacao[i] = popTmp[_Index[i]];
  }
}

double Distance(double *p1, double *p2)
{
  double ssd = 0.0;
  for (int i = 0; i < 2; i++)
    ssd += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  return sqrt(ssd) / pixel_size;
}

double FuncaoObjetivo(double posicao[2], int step)
{
  Aval++;
  int x = posicao[0] / pixel_size;
  int y = posicao[1] / pixel_size;
  return _mapobj[x][y];
}

double _FuncaoObjetivo(double posicao[], int step)
{
  Aval++;
  int i = posicao[0] / pixel_size; //cout << "i -> " << i << endl;
  int j = posicao[1] / pixel_size; //cout << "j -> " << j << endl;

  double pixel[2] = {(double)i, (double)j};

  if (i >= h || i < 0 || j >= w || j < 0)
  {
    return 100000000000;
  }

  if (_map[i][j] > -1)
  {
    double distF = 0;
    double rp[2] = {(double)robot_position[1], (double)robot_position[0]};
    double dp[2] = {(double)destino_corrente[1], (double)destino_corrente[0]};
    double dr = Distance(pixel, rp);

    for (int f = 0; f < fi; ++f)
    {
      double pf[2] = {(double)fronteira[f][0], (double)fronteira[f][1]};
      double d = Distance(pixel, pf);
      distF += exp(-d / (2 * w1 * w1));
    }

    double distR = 0;
    for (int r = 0; r < ri; ++r)
    {
      double pr[2] = {(double)regiao_perigo[r][0], (double)regiao_perigo[r][1]};
      double d = Distance(pixel, pr);
      distR += exp(-d / (2 * w2 * w2));
    }

    double distV = 0;
    //cout << "\n=> VI => " << vi << endl;
    for (int v = 0; v < vi; ++v)
    {
      double pv[2] = {(double)visitados[v][0], (double)visitados[v][1]};
      double d = Distance(pixel, pv);
      distV += exp(-d / (2 * w3 * w3));
    }
    /*
        double distD = 0;

        //cout << "\nhere" << endl;
        for (int ii = pixel[0] - 5; ii<pixel[0] + 5; ii++)
        { 
            if (ii < 0 || ii >= h)  
              continue;
            //cout << "\nii -> " << ii << endl;
            for (int jj = pixel[1] - 5; jj<pixel[1] + 5; jj++)
            {

                if (jj < 0 || jj >= w)
                  continue;
                //cout << "\njj -> " << jj << endl;
                if (_mapa[ii][jj] == 0)
                  distD += 1;
                else if (_mapa[ii][jj] < 0)
                  distD -= 1;
            } 
        }
        */
    //distD = exp(-distD / (2 * w3*w3));

    //cout << "distF: " << distF << endl;
    //cout << "distR: " << distR << endl;
    //cout << "F : " << -alpha*distF + beta*distR + 100* rho*distV  << endl;
    //cout << "distD: " << distD << endl;

  #ifdef FUNCAOPP
      float lambda = 0.1;
      return (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
  #endif

  #ifdef FUNCAOPC
      double k = 20;
      if (dr < k)
        return -alpha1 * distF + beta * distR + rho * distV;
      else
        return -alpha2 * distF + beta * distR + rho * distV;
  #endif

  #ifdef FUNCAOPL
      float lambda = 10;
      return -alpha * distF + beta * distR + rho * distV + lambda * dr;
  #endif
  }
  else
  {
    return 1000000000000000;
  }
}

// TODO: Considerar distancia para o robô
void GeraMapaObjetivo(int w, int h)
{
  cout << ">>> GeraMapaObjetivo\n";
  // Função Objetivo
  float melhor = 1000000;

  /*if (!has_goal)
  {
      destino_corrente[0] = robot_position[0];
      destino_corrente[1] = robot_position[1];
  }*/

  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      double pixel[2] = {(double)i, (double)j};

      if (_map[i][j] != -1)
      {
        float distF = 0;
        double rp[2] = {(double)robot_position[1], (double)robot_position[0]};
        double dp[2] = {(double)destino_corrente[1], (double)destino_corrente[0]};
        float dr = Distance(pixel, rp);
        //std::cout << "DISTANCIA " << dr << std::endl;

        for (int f = 0; f < fi; ++f)
        {
          double pf[2] = {(double)fronteira[f][0], (double)fronteira[f][1]};
          double d = Distance(pixel, pf);
          distF += exp(-d / (2 * w1 * w1));
        }

        float distR = 0;
        for (int r = 0; r < ri; ++r)
        {
          double pr[2] = {(double)regiao_perigo[r][0], (double)regiao_perigo[r][1]};
          float d = Distance(pixel, pr);
          distR += exp(-d / (2 * w2 * w2));
        }

        float distV = 0;
        for (int v = 0; v < vi; ++v)
        {
          double pv[2] = {(double)visitados[v][0], (double)visitados[v][1]};
          float d = Distance(pixel, pv);
          distV += exp(-d / (2 * w3 * w3));
        }

  #ifdef FUNCAOPP
          float lambda = 0.1;
          _mapobj[i][j] = (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
  #endif

  #ifdef FUNCAOPP_2
          float lambda = 0.1;
          _mapobj[i][j] = (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
  #endif

  #ifdef FUNCAOPC
          double k = 30;
          if (dr < k)
            _mapobj[i][j] = -alpha1 * distF + beta * distR + rho * distV;
          else
            _mapobj[i][j] = -alpha2 * distF + beta * distR + rho * distV;

  #endif

  #ifdef FUNCAOPL

          float lambda = 10;
          _mapobj[i][j] = -alpha * distF + beta * distR + rho * distV + lambda * dr;

  #endif
        }
        else
        {
          _mapobj[i][j] = 100000000;
        }

        if (melhor > _mapobj[i][j])
        {
          melhor = _mapobj[i][j];

  #ifdef ENUMERATIVO
          destino_corrente[0] = j;
          destino_corrente[1] = i;
  #endif
      }
    }
  }

  cout << endl
       << "############ MELHOR #############" << endl
       << melhor << endl
       << "#################################" << endl;
}
