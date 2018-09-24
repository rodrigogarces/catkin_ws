// otimizador.hpp
// Data: 11/09/2018
// Editado por Raphael Gomes Santos

#ifndef TARGET_GENERATOR_H_ 
#define TARGET_GENERATOR_H_

#include <math.h>
#include <time.h>
#include <cstdio>

#include <ros/ros.h>

#include <stds_msgs/Int8.h>
#include <stds_msgs/String.h>
#include <stds_msgs/Float64.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <tf/transform_listener.h>

#include "Parameters.h"
#include "Firefly.h"
#include "AStar.h"

class TargetGenerator
{
public:
    TargetGenerator(ros::NodeHandle* nodehandle);

private:
    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;       // Subscreve no topico que recebe o OG do ambiente
    ros::Subscriber curr_pose_sub_; // Subscreve tópico que recebe a posição corrente do robô
    ros::Subscriber costmap_sub_;   //

    ros::Publisher path_pub_;       // Publica o caminho para o alvo
    ros::Publihser target_pub_;     // Publica o próximo alvo do robô
    ros::Publisher finalize_pub_;   // Publica o comando de finalização da exploração

    //=== PARAMETROS DO MAPA ======================
    int w;                      // Largura do mapa
    int h;                      // Altura do mapa

    int origin_x;               //
    int origin_y;               //

    double pose_x;              // Posição x da pose do robô
    double pose_y;              // Posição y da pose do robô

    double bpos[2];             // melhor posição do mapa encontrada no processo de otimização
    int robot_position[2];      // TODO: Descobrir a posicao do robo;
    int destino_corrente[2];    //

    vector<double *> _path;     // Caminho do robô ao alvo
    //=============================================

    //=== VARIAVEIS AUXILIARES ====================
    double pixel_size;      // Tamanho de um pixel do mapa (resolução do mapa)
    bool has_map;           // Indica se um mapa parcial do ambiente já foi obtido
    bool has_goal;          // Indica se há um alvo novo para alcaçar
    bool first_time;        //
    bool gera_new_goal;     // flag para controlar a geração de alvos
    double distanceToGoal;  // Distância entre o robô e o objetivo
    int fi;                 // contadores das listas de pontos dos mapas
    int ri;
    int vi;
    int di;                 
    bool initVst;
    //=============================================

    //=== PARAMETROS DO ALGORITMO DE VAGALUMES ====
    int MaxEpocas;                              //
    int NumFireflies;                           //
    int NumClusters;                            //

    int _step;                                  //
    int firststep;                              //
    double percent;                             //    

    double *dmin;
    double *dmax;                               //

    int maxiEpocas;                             //
    int minEpocas;                              //

    vector<Firefly *> populacao(NumFireflies);  //  
    //=============================================

    //=== PARÂMETROS DO ALGORITMO GÉNETICO ========
    //=============================================

    //=== PARÂMETROS DO ALGORITMO SIMPLEX =========
    //=============================================

    //=== PARAMETROS DO FUNCAO OBJETIVO ===========
    int Aval;          // Numero de chamadas função objetivo
    double avg_aval; // Tempo médio de execução do algoritmo
    double avg_time; // Tempo médio de execução do algoritmo
    int qtdeCA;        // Qtde de vezes que o alg. foi chamado
    //=============================================

    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &cmap);
    void currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &curr_pose);

    // Algoritmos de Otimização:
    double *FireflyAlgorithm(); // Implementa o Algoritmo de Vagalumes
    double *GeneticAlgorithm(); // Implementa o Algoritmo Genético
    double *SimplexAlgorithm(); // Implementa o Algoritmo Simplex

    // Metódos auxiliares:
    double Distance(double *p1, double *p2);
    
    void SortPop();
    void ReplacePop();

    void heapify(int n, int i);
    void heapSort();
};

#endif 