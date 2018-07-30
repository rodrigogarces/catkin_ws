#include <cstdio>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include "Firefly.h"
#include "AStar.h"

using namespace std;

//PARAMETROS DO ALGORITMO DE VAGALUMES:================
int MaxEpocas = 500; // 150
int NumFireflies = 40;

int _step = 0;
int firststep = 0;
double percent = 0.0;
int first = NumFireflies * percent / 100.0;

double *dmin;
double *dmax;

int maxiEpocas = 0;
int minEpocas = 1000000;

vector<Firefly*> populacao(NumFireflies);
//========================================

//########################################### DEFINES ############################################
#define _USE_MATH_DEFINES

#define NUMERO_MINIMO_PFRONTEIRA 10
#define DIST_MIN  10          // Distância minina do alvo ao robô, para que seja gerado uma caminho 

//#define GENETICO
#define FIREFLY
//#define ENUMERATIVO

//#define FUNCAO1
//#define FUNCAO2
//#define FUNCAO3
//#define FUNCAO4

#define FUNCAOPP

//#define GERAFUNCAO

#define _USE_MATH_DcEFINES

#define ESTRATEGIA_1 // Espera robô chegar ao objetivo para otimizar novamente

#define sind(x) (sin(fmod((x),360) * M_PI/180))
#define cosd(x) (cos(fmod((x),360) * M_PI/180))

#define GARE "GAv06"
#define SIMPLEND
#define SIMPLEXON
#define NOSIMPLEN

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
#define FATOR  2000

#define XALFA  0.25

/* Mutacao */
#define MNUNI  7
#define PMUTAC 10

/* Parametros do Simplex */
#define VLALFA 1.0
#define VLBETA 0.5
#define VLGAMA 2.0
#define PASSOS 1000
#define SCALE  1.0
//################################################################################################

//PARAMETROS DO FUNCAO OBJETIVO:==========

#ifdef FUNCAO1
float alpha = 50, w1 = 1.5; // 5000
float beta = 100, w2 = 4; // 10000
float rho = 3, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif


#ifdef FUNCAO2
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 4; //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAO3
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 6; //float beta = 10000, w2 = 4;
float rho = 300, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAO4
float alpha = 500, w1 = 1.5;
float beta = 10, w2 = 4; //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPP
float alpha = 500, w1 = 1.5;
float beta = 300, w2 = 4; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPP_2
float alpha = 500, w1 = 1.5;
float beta = 600, w2 = 4; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPL
float alpha = 5000, w1 = 1.5;
float beta = 1000, w2 = 4; //float beta = 10000, w2 = 4;
float rho = 2000, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef FUNCAOPC
float alpha1 = 1500, alpha2 = 500, w1 = 1.5;
float beta = 30, w2 = 4; //float beta = 10000, w2 = 4;
float rho = 10, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif
//========================================

using namespace std;

/************************* Variáveis de estado *****************************/
int **_map;                 // Guarda estado do mapa
float **_mapa;              // Guarda estado do mapa
float **_mapobj;            // Guarda estado da função objetivo
bool ** pr_map;             // Guarda posições visitadas pelo robô
int ** dilatedMap;

int ** TargetMatrix;        // Guarda os alvos gerados a cada passo da exploração
int ** PositionMatrix;      // Guarda as posições alcançadas pelo robô a cada passo da exploração
int ** StepMatrix;          // Guarda as posições que deveriam ser alcançadas pelo robô a cada passo da exploração

int ** fronteira;           // Lista de pontos do mapa da fronteira
int ** regiao_perigo;       // Lista de pontos do mapa proximos ao obstaculo
int ** visitados;           // Lista de pontos do mapa visitados pelo robô
int ** desconhecidos;       // Lista de pontos do mapa desconhecidos
/***************************************************************************/

/*************************** Variáveis axiliares ***********************************/
double pixel_size = 10;                 // Tamanho de um pixel do mapa (resolução do mapa)
bool has_map = false;                   // Indica se um mapa parcial do ambiente já foi obtido
bool has_goal = false;                  // Indica se há um alvo novo para alcaçar
bool first_time = true;                // 
bool gera_new_goal = false;             // flag para controlar a geração de alvos
double distanceToGoal = 0;              // Distância entre o robô e o objetivo
int fi = 0;
int ri = 0;
int vi = 0;
int di = 0;     // contadores das listas de pontos dos mapas
bool initVst = true;
/***********************************************************************************/

/******************** Variáveis Globais **********************/
int w; // Largura do mapa
int h; // Altura do mapa

int origin_x;
int origin_y;

double pose_x;      // Posição x da pose do robô
double pose_y;      // Posição y da pose do robô

double bpos[2];     // melhor posição do mapa encontrada no processo de otimização
int robot_position[2]; // TODO: Descobrir a posicao do robo;
int destino_corrente[2];

vector<double*> _path;  // Caminho do robô ao alvo


/**************************************************************/

/************************** Publishers *****************************/
ros::Publisher target_pub;      // Publica um novo alvo para o robô
ros::Publisher path_pub;        // Publica o caminho para o alvo
ros::Publisher finalize_pub;
ros::Publisher ori_target_pub;
/*******************************************************************/

/************************* CALLBACKS ****************************/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
/****************************************************************/

/************************** Funcões ****************************/
double LevyFlight(int dim);
void GeraMapaObjetivo(int w, int h);
double Distance(double* p1, double* p2);
double FuncaoObjetivo(double position[2], int step);
double _FuncaoObjetivo(double position[2], int step);
double *Solve();    // Implementa o Algoritmo de Vagalumes
void SortPop();
void ReplacePop();
double *SolveG(int numFireflies, int dim, double *dmin, double *dmax, int seed, int maxEpocas, int step);   // Implementa o Algoritmo Genético
void currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_pose);
void planPath();
/***************************************************************/

void planPath()
{
    if(dilatedMap != NULL)
    {
        AStar *pfinder = new AStar(w, h, dilatedMap);

        // - Pega posição do robô no OG - (TODO: Entender isso!)
        try{
          listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
          listener.lookupTransform("/odom", "/base_link",
            ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
        }

        pose_x = transform.getOrigin().getX();
        pose_y = transform.getOrigin().getY();

        grid_x = ((pose_x - origin_x) / pixel_size);  // posição x no Ocuppancy Grid
        grid_y = ((pose_y - origin_y) / pixel_size);  // posição y no Ocuppancy Grid

        robot_position[0] = grid_x;
        robot_position[1] = grid_y;

        visitados[vi] = new int[2] { robot_position[1], robot_position[0] }; vi++; // Adiciona um ponto a lista de visitados

        vector< vector<int> > path = pfinder->FindPath(robot_position, destino_corrente); // Executa o A*

        // -- Pós-processamento do caminho gerado para que seja enviado como uma mensagem via ROS --
        nav_msgs::Path posepath;
        posepath.poses.resize(path.size());
        int i = 0;

        std::reverse(path.begin(), path.end()); // O path é gerado de trás para frente(destino -> origem). Por isso é invertido.

        if(path.size() < 1)
        {
            has_goal = false;
        }
        else 
        {
            double ant[2];  // gambiarra
            ant[0] = path[0][0] * pixel_size + origin_x;
            ant[1] = path[0][1] * pixel_size + origin_y;

            double ant2[2]; // gambiarra
            ant2[0] = ant[0];
            ant2[1] = ant[1];

            double dif[2];
            _path.clear();

            int ini;// = 2;

            if(path.size() > 4)
              ini = 2;
            else ini = 0; 
            
            _path.push_back(new double[2] {path[ini][0] * pixel_size + origin_x, path[ini][1] * pixel_size + origin_y});
        
            std_msgs::String msg; // Mensagem que conterá o path
            std::stringstream ss;

            float floatoutarray[3];
            char* string;
            double pose[2];

            int k = 0;

            cout << "processa caminho" << endl;
            for (i = ini + 1; i < path.size() ; i++)
            {
                pose[0] = path[i][0] * pixel_size + origin_x;
                pose[1] = path[i][1] * pixel_size + origin_y;

                dif[0] = ant[0] - (path[i][0] * pixel_size + origin_x);
                dif[1] = ant[1] - (path[i][1] * pixel_size + origin_y);

                ant2[0] = ant2[0] + dif[0];
                ant2[1] = ant2[1] - dif[1];

                _path.push_back(new double[2] {pose[0], ant2[1]});

                if (abs(pose[0] - robot_position[0]) <= 2 && abs(pose[1] - robot_position[1]) <= 2)
                    k = _path.size()-1;

                ant[0] = path[i][0] * pixel_size + origin_x;
                ant[1] = path[i][1] * pixel_size + origin_y;
            }

            for(;k < _path.size(); k++)
            {
                floatoutarray[0] = _path[k][0];
                floatoutarray[1] = _path[k][1];
                floatoutarray[2] = 0.0;
                string = (char*) floatoutarray;
                
                ss.write(string, 3*sizeof(float));
            }

            cout << "Publica caminho" << endl;

            msg.data = ss.str();
            path_pub.publish(msg); // Mensagem (com o path) é publicada!
        }
    }
}

int main(int argc, char **argv)
{
    /**************** Bloco de Inicialização ****************/
    dmin = (double*)malloc(2 * sizeof(double));
    dmax = (double*)malloc(2 * sizeof(double));

    ros::init(argc, argv, "otimizador");
    ros::NodeHandle n;

    cout << "Otimizador iniciou sua execução." << endl;

    //###################### Subscribers ########################
    ros::Subscriber map_sub = n.subscribe("/map", 1, mapCallback);          // Subscreve no topico que recebe o OG do ambiente
    ros::Subscriber curr_pose_sub = n.subscribe("/vrep_ros_interface/pose", 1, currPoseCallback); // Subscreve tópico que recebe a posição corrente do robô
    //################ Publishers (Definição) ###################
    path_pub = n.advertise<std_msgs::String>("/path_plan", 1, true);   // Publica o caminho para o alvo
    //target_pub = n.advertise<geometry_msgs::PoseStamped>("/goalPose", 1, true);   // Publica o caminho para o alvo
    target_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);   // Publica o caminho para o alvo
    finalize_pub = n.advertise<std_msgs::String>("/finalize_mapping", 1, true);   // Publica flag que para o mapeamento


    //ori_target_pub = n.advertise<geometry_msgs::PoseStamped>("/ori_goal", 1, true); 
    ros::Rate r(10);

    while(n.ok())
    {
      ros::spinOnce();
      planPath();
      r.sleep();
    }

    //ros::spin();
    return 0;
}


/* #################### Definicão das funções ##################### */
void currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_pose)
{
    if (has_goal)
    {
        geometry_msgs::PoseStamped _msg;
        //msg.header.stamp = ros::Time();
        std::string frame = "/map";
        _msg.header.frame_id = frame.c_str();

        _msg.pose.position.x =  destino_corrente[0] * pixel_size + origin_x;
        _msg.pose.position.y =  destino_corrente[1] * pixel_size + origin_y;
        _msg.pose.position.z = 0;
        _msg.pose.orientation = curr_pose->pose.orientation;

        //ori_target_pub.publish(_msg);
        target_pub.publish(_msg);
    }
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    cout << "################ Map Callback ################\n";

    // TODO: carregar variaveis locais necessárias
    w = map->info.width; // Largura do mapa
    h = map->info.height; // Altura do mapa
    
    pixel_size = map->info.resolution;
    origin_x = map->info.origin.position.x;
    origin_y = map->info.origin.position.y;
    
    dmin[0] = 0; dmin[1] = 0;
    dmax[0] = pixel_size*h; dmax[1] = pixel_size*w;

    // - Inicialização das matrizes de estado -
        // => Realocação todas as vezes é necessária ?????

    if(first_time)
    {
        _map = (int**)malloc(h*sizeof(int*));
        _mapobj = (float**)malloc(h*sizeof(float*));
        _mapa = (float**)malloc(h*sizeof(float*));
        dilatedMap = (int**)malloc(h*sizeof(int*));

        for (int i = 0; i<h; i++)
        {
            _map[i] = (int*)malloc(w*sizeof(int));
            _mapobj[i] = (float*)malloc(w*sizeof(float));
            _mapa[i] = (float*)malloc(w*sizeof(float));
            dilatedMap[i] = (int*)malloc(w*sizeof(int));
        }

        // - Atualização das listas de pontos -
        fronteira = new int*[w*h];         // Lista de pontos de fronteira
        regiao_perigo = new int*[w*h];     // Lista de pontos da regiao de perigo (ocupados ou próximos de ocupados)
        visitados = new int*[w*h]; vi = 0; // Lista de pontos visitados
        desconhecidos = new int*[w*h];     // Lista de pontos desconhecidos

        cout << "FIRST TIME\n" << endl;
    }
    
    /*if(initVst)
    {
        visitados = new int*[w*h]; vi = 0;
        initVst = false;
    }*/
    //has_map = true;

    fi = 0;
    ri = 0;
    di = 0;

    for (int i = 0; i<h; i++) 
    {
        for (int j = 0; j<w; j++)
        {
            int pixel = map->data[i*w + j];

            _map[i][j] = pixel;
            dilatedMap[i][j] = pixel;
        }
    }


    int pixel;
    int aux;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            pixel = _map[i][j];
            aux = 8;
            int mei = i, mej = j;

            for (int ei = i - aux; ei <= i + aux; ++ei)
            {
                for (int ej = j - aux; ej <= j + aux; ++ej)
                {
                    if (ei > 0 && ej > 0 && ei < h && ej < w)
                    {
                        if (_map[ei][ej] > 0)
                        {
                            pixel = -2;

                            break;
                        }
                    }
                }
            }

            if( _map[i][j] > 0)
                dilatedMap[i][j] = _map[i][j];
            else dilatedMap[i][j] = pixel;


            if (_map[i][j] < 0)
              dilatedMap[i][j] = _map[i][j];

            //if(pixel > 0 &&  _map[i][j] == 0)
            //    dilatedMap[i][j] = -2;
            //else 
            //    
        }
    }

    for (int i = 0; i<h; i++) 
    {
        for (int j = 0; j<w; j++)
        {
            int pixel = dilatedMap[i][j];

            //_map[i][j] = pixel;
            //dilatedMap[i][j] = pixel;

            if (pixel == -1)
            {
                desconhecidos[di] = new int[2] { i, j }; di++;
            }
            else
            {
                if (pixel > 0)
                {
                    regiao_perigo[ri] = new int[2] { i, j }; ri++;
                }
                else
                {   // Procura pontos desconhecidos adjacentes
                    int l = j > 0 ? map->data[i*w + (j - 1)] : 0;
                    int r = j < h - 1 ? map->data[i*w + (j + 1)] : 0;
                    int u = i > 0 ? map->data[(i - 1)*w + j] : 0;
                    int d = i < w - 1 ? map->data[(i + 1)*w + j] : 0;

                    if (l == -1 || r == -1 || u == -1 || d == -1) 
                    {
                        int p[2] = { i, j };
                        fronteira[fi] = new int[2] { i, j }; fi++;
                    }
                }
            }   
        } 
    }

    cout << "\nTESTE DE PARADA.\n";
    cout << "numero de desconhecidos: " << di << endl;
    if(di <= 27270) // 109400 
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

        cout << "\nO MAPEAMENTO FINALIZOU.\n";
        system("rosnode kill SLAM");
        ros::shutdown();
        return;
    }
    else 
    {
        cout << "Publica não finalização; " << endl;
        std_msgs::String finalize;
        finalize.data = "0";
        finalize_pub.publish(finalize); 
    }

    cout << "marco 1"  << endl;

    

    cout << " marco 2"  << endl;
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
    
    //int a;
    //cin >> a;

cout << " marco 3"  << endl;
    // - Pega posição do robô no OG - (TODO: Entender isso!)
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try{
	//cout << " marco 3.1"  << endl;
      listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
//cout << " marco 3.2"  << endl;
      listener.lookupTransform("/odom", "/base_link",
        ros::Time(0), transform);
//cout << " marco 3.3"  << endl;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }


    geometry_msgs::Twist vel_msg;

    pose_x = transform.getOrigin().getX();
    pose_y = transform.getOrigin().getY();

    //int grid_x = ((pose_x - origin_x) / pixel_size);  // posição x no Ocuppancy Grid
    //int grid_y = ((pose_y - origin_y) / pixel_size);  // posição y no Ocuppancy Grid

    int grid_x = ((pose_x - origin_x) / pixel_size);  // posição x no Ocuppancy Grid
    int grid_y = ((pose_y - origin_y) / pixel_size);  // posição y no Ocuppancy Grid

    robot_position[0] = grid_x;
    robot_position[1] = grid_y;

    //pr_map[grid_x][grid_y] = true;

    visitados[vi] = new int[2] { robot_position[1], robot_position[0] }; vi++; // Adiciona um ponto a lista de visitados

cout << " marco 4"  << endl;
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

        if (i < h-1)
        {
            fprintf(fo, ";\n");
        }
    }
    fprintf(fo, "]");
    fclose(fo);
#endif

//#define SALVAMAPA
#ifdef SALVAMAPA
    FILE *fo = fopen("_map.txt", "w");

    fprintf(fo, "[");
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if(_map[i][j] > 0)
              fprintf(fo, " %d", 1);
            else fprintf(fo, " %d", 0);

            //fprintf(fo, "\t %.4f", _mapobj[i][j]);
        }

        if (i < h-1)
        {
            fprintf(fo, ";\n");
        }
    }
    fprintf(fo, "]");
    fclose(fo);
#endif

cout << " marco 5"  << endl;

    int destino[2];
    double dis = 0.0;

    cout << "first_time" << first_time << endl;
    if(!first_time)
    {
        dis = sqrt( (destino_corrente[0] - robot_position[0])*(destino_corrente[0] - robot_position[0]) 
          + (destino_corrente[1] - robot_position[1])*(destino_corrente[1] - robot_position[1])); // distância entre o robô e o alvo
    }    

    // -> Se a distância é menor que a distancia minima, o alvo não é atualizado
    if(dis < DIST_MIN)
    {
        has_goal = false;   
        cout << "dis: " << dis << endl;
        cout << "has_goal = false " << endl;
    }
    else
    {
        has_goal = true; 
        cout << "dis: " << dis << endl;
        cout << "has_goal = true " << endl;
    }

     first_time = false;

    // - Gera um alvo - 
    if (!has_goal)
    { 
        double *s;

#ifdef ENUMERATIVO
        GeraMapaObjetivo(w,h);
#endif

#ifdef FIREFLY
            cout << "Chama Firefly" << endl;
            s = Solve();

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
            s = SolveG(40, 2, dmin, dmax, 0, 100, 0);

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

    _msg.pose.position.x =  destino_corrente[0] * pixel_size + origin_x;//path[l][0];
    _msg.pose.position.y =  destino_corrente[1] * pixel_size + origin_y;//path[l][1];
    _msg.pose.position.z = 0;
    _msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    cout << "Publica alvo." << endl;
    target_pub.publish(_msg);

    cout << "\n" << robot_position[0] << ", " << robot_position[1] << endl;
    cout << "\n" << destino_corrente[0] << ", " << destino_corrente[1] << endl;
    cout << " valor: " << _map[ destino_corrente[1] ][ destino_corrente[0] ] << endl;
 
    
    // -- Gera caminho --
    AStar *pfinder = new AStar(w, h, dilatedMap);

    // - Pega posição do robô no OG - (TODO: Entender isso!)
    try{
      listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("/odom", "/base_link",
        ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    pose_x = transform.getOrigin().getX();
    pose_y = transform.getOrigin().getY();

    grid_x = ((pose_x - origin_x) / pixel_size);  // posição x no Ocuppancy Grid
    grid_y = ((pose_y - origin_y) / pixel_size);  // posição y no Ocuppancy Grid

    robot_position[0] = grid_x;
    robot_position[1] = grid_y;

    visitados[vi] = new int[2] { robot_position[1], robot_position[0] }; vi++; // Adiciona um ponto a lista de visitados

    vector< vector<int> > path = pfinder->FindPath(robot_position, destino_corrente); // Executa o A*

    cout << "retornou path!!!! " << endl;

    // -- Pós-processamento do caminho gerado para que seja enviado como uma mensagem via ROS --
    nav_msgs::Path posepath;
    posepath.poses.resize(path.size());
    int i = 0;

    std::reverse(path.begin(), path.end()); // O path é gerado de trás para frente(destino -> origem). Por isso é invertido.

    if(path.size() <= 1)
    {
        cout << "Caminho não gerado!" << endl;

        has_goal = false;
        //_map[ destino_corrente[1] ][ destino_corrente[0] ] = -2;
        /*std_msgs::String _msg; // Mensagem que conterá o path
        std::stringstream _ss;

        float _floatoutarray[3];
        char* _string;


        _floatoutarray[0] = robot_position[0] * pixel_size + origin_x;
        _floatoutarray[1] = robot_position[1] * pixel_size + origin_y;
        _floatoutarray[2] = 0.0;
        _string = (char*) _floatoutarray;
            
        _ss.write(_string, 3*sizeof(float));

        _msg.data = _ss.str();
        path_pub.publish(_msg); */
    }
    else 
    {
        double ant[2];  // gambiarra
        ant[0] = path[0][0] * pixel_size + origin_x;
        ant[1] = path[0][1] * pixel_size + origin_y;

        double ant2[2]; // gambiarra
        ant2[0] = ant[0];
        ant2[1] = ant[1];

        double dif[2];
        _path.clear();

        int ini;// = 2;

        if(path.size() > 4)
          ini = 2;
        else ini = 0; 
        
        _path.push_back(new double[2] {path[ini][0] * pixel_size + origin_x, path[ini][1] * pixel_size + origin_y});
    
        std_msgs::String msg; // Mensagem que conterá o path
        std::stringstream ss;

        float floatoutarray[3];
        char* string;
        double pose[2];

        int k = 0;

        cout << "processa caminho" << endl;
        for (i = ini + 1; i < path.size() ; i++)
        {
            pose[0] = path[i][0] * pixel_size + origin_x;
            pose[1] = path[i][1] * pixel_size + origin_y;

            dif[0] = ant[0] - (path[i][0] * pixel_size + origin_x);
            dif[1] = ant[1] - (path[i][1] * pixel_size + origin_y);

            ant2[0] = ant2[0] + dif[0];
            ant2[1] = ant2[1] - dif[1];

            _path.push_back(new double[2] {pose[0], ant2[1]});

            if (abs(pose[0] - robot_position[0]) <= 2 && abs(pose[1] - robot_position[1]) <= 2)
                k = _path.size()-1;

            ant[0] = path[i][0] * pixel_size + origin_x;
            ant[1] = path[i][1] * pixel_size + origin_y;

            /*
            floatoutarray[0] = pose[0];
            floatoutarray[1] = ant2[1];
            floatoutarray[2] = 0.0;
            string = (char*) floatoutarray;
            
            ss.write(string, 3*sizeof(float));*/
        }

        //if(!has_goal)
        //{
            for(;k < _path.size(); k++)
            {
                floatoutarray[0] = _path[k][0];
                floatoutarray[1] = _path[k][1];
                floatoutarray[2] = 0.0;
                string = (char*) floatoutarray;
                
                ss.write(string, 3*sizeof(float));
            }

            cout << "Publica caminho" << endl;

            msg.data = ss.str();
            path_pub.publish(msg); // Mensagem (com o path) é publicada!
        //}
        // TODO: Publicação do alvo.
        /*std_msgs::String _msg; // Mensagem que conterá o path
        std::stringstream _ss;

        float _floatoutarray[3];
        char* _string;
        
        _floatoutarray[0] = path[l][0];
        _floatoutarray[1] = path[l][1];
        _floatoutarray[2] = 0.0;
        _string = (char*) _floatoutarray;
        
        _ss.write(string, 3*sizeof(float));
        _msg.data = _ss.str();*/
    }
}


int Aval = 0;

bool hc = false;
Firefly *campeao = new Firefly(2);

enum { f12 = 0, gol, fea, sph, ros2, sch, ras, gri, mic, lan, she, ack } enumFunc;
static double(*funccod[])(double[], int n) =
{ _FuncaoObjetivo };
struct DadosFuncoes{
  char    nom[5];
  double  inf;
  double  sup;
}limFixos[] = { { "F12", 0.0F,  19.1F}};


int funcao = FUNCAO;

  void nu_mutate(double * indiv, int tamind, int tampop, int ger, int expo)
  {
    int i, j;
    float fRandVal, fFactor;
    float fNewt, fNewT;
    int   iExponent, iIndex;

    fRandVal = (rand() % 101 / 100.);
    iIndex = rand() % tamind;
    /* pick either the max or min. limit */
    if (fRandVal < 0.5)   /* lower */
    {
      fNewt = ger;
      fNewT = MAXGER;
      //                iExponent = (int) (6*randgen(0.0, 1.0)) + 1;
      fRandVal = (rand() % 101 / 100.);
      fFactor = pow((1.0F - (fNewt / fNewT)), expo) * fRandVal;

      if (fFactor < TAXERR) fFactor = TAXERR;
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

      if (fFactor < TAXERR) fFactor = TAXERR;
      fFactor = fFactor * (limFixos[funcao].sup - indiv[iIndex]);
      indiv[iIndex] = indiv[iIndex] + fFactor;
    }
  }

typedef struct {
  double          var[MAXVAR];
  double          fit;
} Cromossomo;

typedef struct {
  Cromossomo      indiv[MAXPOP];
  double          sumFit;
  int             tamPop;
  int             tamInd;
  int             melhor;
  int             pior;
  int             numMuta;
  int             iguais;
  int             gerMelhor;
} Populacao;



double Simplex(double(*func)(double[], int n), double start[], int n, double EPSILON,
  double scale, int MAX_IT, double ALPHA, double BETA, double GAMMA)
{

  int vs;        /* vertex with smallest value */
  int vh;        /* vertex with next smallest value */
  int vg;        /* vertex with largest value */

  int i, j, m, row;
  int k;    /* track the number of function evaluations */
  int itr;    /* track the number of iterations */

  double **v;          /* holds vertices of simplex */
  double pn, qn;        /* values used to create initial simplex */
  double *f;           /* value of function at each vertex */
  double fr;           /* value of function at reflection point */
  double fe;           /* value of function at expansion point */
  double fc;           /* value of function at contraction point */
  double *vr;         /* reflection - coordinates */
  double *ve;         /* expansion - coordinates */
  double *vc;         /* contraction - coordinates */
  double *vm;         /* centroid - coordinates */
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
  for (i = 0; i <= n; i++) {
    v[i] = (double *)malloc(n * sizeof(double));
  }


  /* create the initial simplex */
  /* assume one of the vertices is 0,0 */

  pn = scale*(sqrt((double)n + 1) - 1 + n) / (n*sqrt((double)2));
  qn = scale*(sqrt((double)n + 1) - 1) / (n*sqrt((double)2));

  for (i = 0; i<n; i++) {
    v[0][i] = start[i];
  }

  for (i = 1; i <= n; i++) {
    for (j = 0; j<n; j++) {
      if (i - 1 == j) {
        v[i][j] = pn + start[j];
      }
      else {
        v[i][j] = qn + start[j];
      }
    }
  }

  /* find the initial function values */
  for (j = 0; j <= n; j++) {
    f[j] = func(v[j], n);
  }

  k = n + 1;

  /* print out the initial values
  fprintf("Initial Values\n");
  for (j=0;j<=n;j++) {
  fprintf("%f %f %f\n",v[j][0],v[j][1],f[j]);
  }                               */


  /* begin the main loop of the minimization */
  for (itr = 1; itr <= MAX_IT; itr++) {
    /* find the index of the largest value */
    vg = 0;
    for (j = 0; j <= n; j++) {
      if (f[j] > f[vg]) {
        vg = j;
      }
    }

    /* find the index of the smallest value */
    vs = 0;
    for (j = 0; j <= n; j++) {
      if (f[j] < f[vs]) {
        vs = j;
      }
    }

    /* find the index of the second largest value */
    vh = vs;
    for (j = 0; j <= n; j++) {
      if (f[j] > f[vh] && f[j] < f[vg]) {
        vh = j;
      }
    }

    /* calculate the centroid */
    for (j = 0; j <= n - 1; j++) {
      cent = 0.0;
      for (m = 0; m <= n; m++) {
        if (m != vg) {
          cent += v[m][j];
        }
      }
      vm[j] = cent / n;
    }

    /* reflect vg to new vertex vr */
    for (j = 0; j <= n - 1; j++) {
      /*vr[j] = (1+ALPHA)*vm[j] - ALPHA*v[vg][j];*/
      vr[j] = vm[j] + ALPHA*(vm[j] - v[vg][j]);
    }
    fr = func(vr, n);
    k++;

    if (fr < f[vh] && fr >= f[vs]) {
      for (j = 0; j <= n - 1; j++) {
        v[vg][j] = vr[j];
      }
      f[vg] = fr;
    }

    /* investigate a step further in this direction */
    if (fr <  f[vs]) {
      for (j = 0; j <= n - 1; j++) {
        /*ve[j] = GAMMA*vr[j] + (1-GAMMA)*vm[j];*/
        ve[j] = vm[j] + GAMMA*(vr[j] - vm[j]);
      }
      fe = func(ve, n);
      k++;

      /* by making fe < fr as opposed to fe < f[vs],
      Rosenbrocks function takes 63 iterations as opposed
      to 64 when using double variables. */

      if (fe < fr) {
        for (j = 0; j <= n - 1; j++) {
          v[vg][j] = ve[j];
        }
        f[vg] = fe;
      }
      else {
        for (j = 0; j <= n - 1; j++) {
          v[vg][j] = vr[j];
        }
        f[vg] = fr;
      }
    }

    /* check to see if a contraction is necessary */
    if (fr >= f[vh]) {
      if (fr < f[vg] && fr >= f[vh]) {
        /* perform outside contraction */
        for (j = 0; j <= n - 1; j++) {
          /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
          vc[j] = vm[j] + BETA*(vr[j] - vm[j]);
        }
        fc = func(vc, n);
        k++;
      }
      else {
        /* perform inside contraction */
        for (j = 0; j <= n - 1; j++) {
          /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
          vc[j] = vm[j] - BETA*(vm[j] - v[vg][j]);
        }
        fc = func(vc, n);
        k++;
      }


      if (fc < f[vg]) {
        for (j = 0; j <= n - 1; j++) {
          v[vg][j] = vc[j];
        }
        f[vg] = fc;
      }
      /* at this point the contraction is not successful,
      we must halve the distance from vs to all the
      vertices of the simplex and then continue.
      10/31/97 - modified to account for ALL vertices.
      */
      else {
        for (row = 0; row <= n; row++) {
          if (row != vs) {
            for (j = 0; j <= n - 1; j++) {
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
    for (j = 0; j <= n; j++) {
      fsum += f[j];
    }
    favg = fsum / (n + 1);
    s = 0.0;
    for (j = 0; j <= n; j++) {
      s += pow((f[j] - favg), 2.0) / (n);
    }
    s = sqrt(s);
    if (s < EPSILON) break;
  }
  /* end main loop of the minimization */

  /* find the index of the smallest value */
  vs = 0;
  for (j = 0; j <= n; j++) {
    if (f[j] < f[vs]) {
      vs = j;
    }
  }

  //  fprintf("The minimum was found at\n");
  for (j = 0; j<n; j++) {
    //    fprintf("%e\n",v[vs][j]);
    start[j] = v[vs][j];
  }
  min = func(v[vs], n);
  k++;
  //  fprintf("%d Function Evaluations\n",k);
  //  fprintf("%d Iterations through program\n",itr);

  for (i = 0; i <= n; i++) {
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

  fRandomVal = rand() % 101 / 100.;        // rand entre 0 e 1

  return(fLlim + (float)(fRandomVal * (fUlim - fLlim)));

}

void IniciaPop(Populacao * p, int m, int n, int nfun){
  int i, j, pior, melhor;
  double soma, fit;

  //std::cout << "MAX: " << limFixos[nfun].sup << std::endl;
 //std::cout << "3:" ; 
  for (i = 0, soma = 0, pior = 0, melhor = 0; i<m; i++)
  {
    //std::cout << "3:" ; 
    for (j = 0; j<n; j++){
      p->indiv[i].var[j] = (double)randgen(limFixos[nfun].inf, limFixos[nfun].sup);

    }
    
    fit = funccod[nfun](p->indiv[i].var, n);
    
    p->indiv[i].fit = fit;
    if (fit > p->indiv[pior].fit) pior = i;
    if (fit < p->indiv[melhor].fit) melhor = i;
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

void IniciaPSc(Populacao * p, int m, int n, int nfun){
  int i, j, k;
  int pior, melhor, ipp;
  float inf, sup, pas, par, npa;
  double soma, fit;

  npa = log((double)m) / log((double)2);
  pas = (limFixos[nfun].sup - limFixos[nfun].inf) / npa;
  inf = limFixos[nfun].inf;
  sup = ((inf + pas) < limFixos[nfun].sup ? (inf + pas) : limFixos[nfun].sup);
  ipp = m / npa;

  for (i = 0, soma = 0, pior = 0, melhor = 0; i<m; i++)
  {
    for (j = 0; j < n; j++)
      p->indiv[i].var[j] = (double)randgen(inf, sup);
    fit = funccod[nfun](p->indiv[i].var, n);
    p->indiv[i].fit = fit;
    if (fit > p->indiv[pior].fit) pior = i;
    if (fit < p->indiv[melhor].fit) melhor = i;
    soma += (fit); //fabs
    if ((i + 1) % ipp == 0) {
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

int  Selecao(Populacao *p)
{
  int   i;
  double val, spin_val;

  val = 0.0;
  spin_val = (rand() % 101) / 100. * p->sumFit;
  i = rand() % p->tamPop;

  do {
    i = (i < p->tamPop - 1) ? i + 1 : 0;
    val = val + fabs(FATOR*p->sumFit) / (1 + p->indiv[i].fit - p->indiv[p->melhor].fit);
  } while (val < spin_val);
  return i;        // posição do estouro
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
  r = a + r*(b - a);

  for (i = 0; i < p->tamInd; i++) {
    p->indiv[filho].var[i] =
      p->indiv[pai].var[i] + r*(p->indiv[mae].var[i] - p->indiv[pai].var[i]);
    if (p->indiv[filho].var[i] < limFixos[FUNCAO].inf){
      interv = limFixos[FUNCAO].sup - limFixos[FUNCAO].inf;
      nureal = (limFixos[FUNCAO].inf - p->indiv[filho].var[i]) / interv;
      nuinte = nureal;
      ajuste = (nureal - nuinte) * interv;
      voltas = nuinte % 2;
      p->indiv[filho].var[i] = limFixos[FUNCAO].inf + voltas * interv - (2 * voltas - 1)*ajuste;
    }
    else if (p->indiv[filho].var[i] > limFixos[FUNCAO].sup) {
      interv = limFixos[FUNCAO].sup - limFixos[FUNCAO].inf;
      nureal = (p->indiv[filho].var[i] - limFixos[FUNCAO].sup) / interv;
      nuinte = nureal;
      ajuste = (nureal - nuinte) * interv;
      voltas = 1 - nuinte % 2;
      p->indiv[filho].var[i] = limFixos[FUNCAO].inf + voltas * interv - (2 * voltas - 1)*ajuste;
    }
  }

}
void CruzaGeom(Populacao *p, int pai, int mae, int filho)
{
  int i;
  float alfa;

  alfa = rand() % 101 / 100.;

  for (i = 0; i < p->tamInd; i++){
    p->indiv[filho].var[i] =
      pow((double)p->indiv[pai].var[i], (double)alfa) * pow(p->indiv[mae].var[i], 1 - (double)alfa);
  }

}

void CruzaEsfe(Populacao *p, int pai, int mae, int filho)
{
  int i;
  float alfa;

  alfa = rand() % 101 / 100.;

  for (i = 0; i < p->tamInd; i++){
    p->indiv[filho].var[i] =
      sqrt(alfa*pow(p->indiv[pai].var[i], 2) + (1 - alfa)*pow(p->indiv[mae].var[i], 2));
  }

}

void AtualizaPop(Populacao *p, int pos, double fit, int ger)
{
  int i, j, max;

  p->sumFit -= (p->indiv[pos].fit); //fabs
  p->sumFit += (fit);               // fabs
  p->indiv[pos].fit = fit;

  if (fit < p->indiv[p->melhor].fit){
    p->melhor = pos;
    p->gerMelhor = ger;
  }
  /* ***** procura um outro pior ******** */
  max = PATUAL*p->tamPop / 100;
  //        p->pior=p->melhor == p->tamPop ? p->melhor - 1: p->melhor +1;
  for (i = 0; i < max; i++){
    j = rand() % p->tamPop;
    if (j == pos || j == p->melhor)
      continue;
    if (p->indiv[j].fit > p->indiv[p->pior].fit) {
      p->pior = j;
      i += PATUAL;
    }
  }

}

double SimplexFixo(double * indiv, int tam, int passos, int funcao)
{
  double erro = TAXERR;
  //double Alfa=1.1, Beta=0.5, Gama=2.2;       /* expansion coefficient */
  double Alfa = VLALFA, Beta = VLBETA, Gama = VLGAMA;
  double min;

  min = Simplex(funccod[funcao], indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
  return min;

}

double SimplexRand(double * indiv, int tam, int passos, int funcao)
{
  double erro = TAXERR;
  double Alfa, Beta, Gama;
  double min;

  Alfa = randgen(VLALFA - 0.5, VLALFA + 0.5);
  Beta = randgen(VLBETA - 0.5, VLBETA + 0.5);
  Gama = randgen(VLGAMA - 0.5, VLGAMA + 0.5);

  min = Simplex(funccod[funcao], indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
  return min;

}
/****/
void IniciaSct(Populacao * p, int m, int n, int nfun){
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
  for (i = 0; i<n; i++)
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
    if ((++i % ipd) == 0){
      do{
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
double * SolveG(int numFireflies, int dim, double *dmin, double *dmax, int seed, int maxEpocas, int step)
{

  std::cout << "1:" ; 

    double bestFitness = 10000000.0;
    double *bestPosition;

    bestPosition = (double*)malloc(2 * sizeof(double));
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
    if (!(saida = fopen(nomarq, "w"))){
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

    std::cout << "2:" ; 

    printf("\n***   (%s) por ACMO/CAP/LAC/INPE    ***", GARE);
    printf("\n      Evoluindo %s com %d vars", limFixos[funcao].nom, MAXVAR);
    start = clock();
    while (numGeracoes--  && Aval < MAXAVA  && erro >(double) 0.0F){
      numCruza = NUMCRU;
      while (numCruza--){
        pa1 = Selecao(&P);
        pa2 = Selecao(&P);
        if (pa1 != pa2)
          CruzaBlend(&P, pa1, pa2, P.pior, xalfa);
        //                        CruzaEsfe(&P, pa1, pa2, P.pior);
        //                        CruzaGeom(&P, pa1, pa2, P.pior);
        else
          P.iguais++;

#ifdef SIMPLEXO
        if (pMuta > rand() % 100 && (MAXGER - numGeracoes)> (0.95*MAXGER)){
          //                        fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
          fit = SimplexFixo(P.indiv[P.pior].var, P.tamInd, MaxIt, funcao);
          P.numMuta++;
        }
        else{
          fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
        }
#endif
#ifdef SIMPLEND
        if (pMuta > rand() % 100){
          fit = SimplexRand(P.indiv[P.pior].var, P.tamInd, MaxIt, funcao);
          P.numMuta++;
        }
        else{
          fit = funccod[funcao](P.indiv[P.pior].var, P.tamInd);
        }
#endif
#ifdef NOSIMPLE
        if (pMuta > rand() % 100){
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

    std::cout << "2\n" ; 
    end = clock();

    /* *******************RESULTADOS ******************** */

    med = P.sumFit / P.tamPop;
    dvp = 0;
    for (i = 0; i<P.tamPop; i++)
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
    for (i = 0; i<P.tamInd; i++) {
      printf("\n\t\t%.6f", P.indiv[P.melhor].var[i]);
    }

    printf("\n\t Minimo = %.16f; \n\t Na ger = %d; \n\t mutacoes = %d \n\t ; aval = %d\n\t; (%.4f, %.4f)\n",
      P.indiv[P.melhor].fit, P.gerMelhor, P.numMuta, Aval, med, dvp);
    printf("\n\t em tempo = %.4f, ", (end - start) / CLOCKS_PER_SEC);
    //getchar();
#endif

    std::cout << "1\n" ;

    return bestPosition;
}

static unsigned int g_seed;

inline void fast_srand( int seed ){
    g_seed = seed;
}

inline int fastrand() { 
    g_seed = (214013*g_seed+2531011); 
    return (g_seed>>16)&0x7FFF; 
} 

// Implementa Firefly's Algorithm
double * Solve()
{
  double B0 = 1; // 1 
  double gamma = 0.4; // 0.0
  double alpha = 0.1; // 0.3

  double bestFitness = 10000000.0;
  double *bestPosition;

  bestPosition = (double*)malloc(2 * sizeof(double));

  fast_srand(time(NULL));
  double r01;

  int ini = 0, i, j, k;

  bool hasChange;
  double beta;

  struct timeval inicio, final;
  double tmili;

  gettimeofday(&inicio, NULL);

  double newpos[2];

  for (i = 0; i < NumFireflies; i++)
  {
    if (i >= ini)
    {
      populacao[i] = new Firefly(2);
      for (j = 0; j < 2; j++)
      {
        r01 = ((double)(fastrand()% 101) / 100);
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

  Firefly* atual;

  cout << "entrou em solve..." << endl; 

  while (epoca < MaxEpocas)
  {
    hasChange = false;

    // Evaluate:
    for(i=0;i<NumFireflies;i++)
        populacao[i]->fitness = _FuncaoObjetivo(populacao[i]->GetPosicao(), _step); n++;

    //  Sort:
    SortPop();

    // Replace:
    ReplacePop();

    // Move:
    for (i = 0; i < NumFireflies; i++)
    {
      double *ipos = populacao[i]->GetPosicao();
      for (j = 0; j < NumFireflies; j++)
      {
        double * jpos = populacao[j]->GetPosicao();

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
              r01 = ((double)(fastrand()% 101) / 100);;

              newpos[k] = ipos[k]
                + beta * (jpos[k] - ipos[k])
                + alpha * (r01 - 0.5) * (dmax[k] - dmin[k]);

              if (newpos[k] < dmin[k])
                newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand()% 101) / 100) + dmin[k];
              if (newpos[k] > dmax[k])
                newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand()% 101) / 100) + dmin[k];

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

    if (bepoca > maxiEpocas)
    {
        maxiEpocas = bepoca;
    }

    if (bepoca < minEpocas)
    {
        minEpocas = bepoca;
    }

    n = 0;
    return bestPosition;
}

int Index[1000];

void SortPop()
{
    int i, j;

	  // initialization of indexes
	  for(i=0;i<NumFireflies;i++)
		    Index[i] = i;

	  // Bubble sort
	  for(i=0;i<NumFireflies-1;i++)
    {
        for(j=i+1;j<NumFireflies;j++)
        {
            if(populacao[i]->fitness > populacao[j]->fitness)
            {
                double z = populacao[i]->fitness;
                populacao[i]->fitness = populacao[j]->fitness;
                populacao[i]->fitness = z;

                int k = Index[i];	// exchange indexes
                Index[i] = Index[j];
                Index[j] = k;
            }
        }
    }   
}

void ReplacePop()
{
    int i, j;
    vector<Firefly*> popTmp = populacao; 

    for(i=0;i<NumFireflies;i++)
    {
        populacao[i] = popTmp[Index[i]];
    }
    
}

double Distance(double* p1, double* p2)
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
    
    double pixel[2] = { i, j };

    if (i >= h || i < 0 || j >= w || j < 0)
    {
        return 100000000000;
    }

    if (_mapa[i][j] > -1)
    {
        double distF = 0;
        double rp[2] = { robot_position[1], robot_position[0] };
        double dp[2] = { destino_corrente[1], destino_corrente[0] };
        double dr = Distance(pixel, rp);

        for (int f = 0; f<fi; ++f)
        {
            double pf[2] = { fronteira[f][0], fronteira[f][1] };
            double d = Distance(pixel, pf);
            distF += exp(-d / (2 * w1 * w1));
        }

        double distR = 0;
        for (int r = 0; r<ri; ++r)
        {
            double pr[2] = { regiao_perigo[r][0], regiao_perigo[r][1] };
            double d = Distance(pixel, pr);
            distR += exp(-d / (2 * w2*w2));
        }

        double distV = 0;
        //cout << "\n=> VI => " << vi << endl;
        for (int v = 0; v<vi; ++v)
        { 
            double pv[2] = { visitados[v][0], visitados[v][1] };
            double d = Distance(pixel, pv); 
            distV += exp(-d / (2 * w3*w3));
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
        return (-alpha*distF + beta*distR + rho*distV ) / (lambda * dr);
#endif         

#ifdef FUNCAOPC
      double k = 30;
      if (dr < k)
          return  -alpha1*distF + beta*distR + rho*distV;
      else return  -alpha2*distF + beta*distR + rho*distV;

#endif

#ifdef FUNCAOPL
        float lambda = 10;
        return -alpha*distF + beta*distR + rho*distV + lambda * dr;
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
      double pixel[2] = { i, j };

      if (_mapa[i][j] != -1)
      {
        float distF = 0;
        double rp[2] = { robot_position[1], robot_position[0] };
        double dp[2] = { destino_corrente[1], destino_corrente[0] };
        float dr = Distance(pixel, rp);
        //std::cout << "DISTANCIA " << dr << std::endl;

        for (int f = 0; f<fi; ++f)
        {
            double pf[2] = { fronteira[f][0], fronteira[f][1] };
            double d = Distance(pixel, pf);
            distF += exp(-d / (2 * w1 * w1));
        }

        float distR = 0;
        for (int r = 0; r<ri; ++r)
        {
          double pr[2] = { regiao_perigo[r][0], regiao_perigo[r][1] };
          float d = Distance(pixel, pr);
          distR += exp(-d / (2 * w2*w2));
        }

        float distV = 0;
        for (int v = 0; v<vi; ++v)
        {
          double pv[2] = { visitados[v][0], visitados[v][1] };
          float d = Distance(pixel, pv); 
          distV +=  exp(-d / (2 * w3*w3));
        }

        int distD = 0;

        //cout << "\nhere" << endl;
        for (int ii = pixel[0] - 5; ii<pixel[0] + 5; ii++)
        { 
            if (ii < 0 || ii >= h)
              continue;
            //cout << "\nii" << endl;
            for (int jj = pixel[1] - 5; jj<pixel[1] + 5; jj++)
            {

                if (jj < 0 || jj >= w)
                  continue;
                //cout << "\njj" << endl;
                distD += _mapa[ii][jj] == 0 ? 1 : (_mapa[ii][jj] > 0 ? 0 : -1); 
            } 
        }
      

#ifdef FUNCAOPP
        float lambda = 0.1;
        _mapobj[i][j] =  (-alpha*distF + beta*distR + rho*distV - 100 * distD ) / (lambda * dr);
#endif         

#ifdef FUNCAOPP_2
        float lambda = 0.1;
        _mapobj[i][j] =  (-alpha*distF + beta*distR + rho*distV) / (lambda * dr);
#endif 


#ifdef FUNCAOPC
      double k = 30;
      if (dr < k)
          _mapobj[i][j] =  -alpha1*distF + beta*distR + rho*distV;
      else _mapobj[i][j] =  -alpha2*distF + beta*distR + rho*distV;

#endif

#ifdef FUNCAOPL

        float lambda = 10;
        _mapobj[i][j] =  -alpha*distF + beta*distR + rho*distV + lambda * dr;

#endif

      
      }
      else
      {
        _mapobj[i][j] = 100000000;

        
      }

      if (melhor  >_mapobj[i][j])
      {
          melhor = _mapobj[i][j];

#ifdef ENUMERATIVO
          destino_corrente[0] = j;
          destino_corrente[1] = i;
#endif

      }
    }
  }

  cout << endl << "############ MELHOR #############" << endl << melhor << endl << "##############################" << endl;
}
