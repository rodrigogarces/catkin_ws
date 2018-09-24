#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#define _USE_MATH_DEFINES

#define NUMERO_MINIMO_PFRONTEIRA 10
#define DIST_MIN 5 // 5 // Distância minina do alvo ao robô, para que seja gerado uma caminho

#define sind(x) (sin(fmod((x), 360) * M_PI / 180))
#define cosd(x) (cos(fmod((x), 360) * M_PI / 180))

//=== PARAMETROS DO FUNCAO OBJETIVO ==========
#ifdef PARAM_F1
float alpha = 50, w1 = 1.5; // 5000
float beta = 100, w2 = 4;   // 10000
float rho = 3, w3 = 3.45;   // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_F2
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_F3
float alpha = 50, w1 = 1.5;
float beta = 10, w2 = 6;    //float beta = 10000, w2 = 4;
float rho = 300, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_F4
float alpha = 500, w1 = 1.5;
float beta = 10, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 30, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_PLF
float alpha = 500, w1 = 1.5;
float beta = 150, w2 = 4; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45; // 300  w3 = 3.45
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_PLF_2
float alpha = 500, w1 = 3.5;
float beta = 150, w2 = 15; //float beta = 150, w2 = 4;
float rho = 3, w3 = 3.45;  // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_LLF
float alpha = 5000, w1 = 1.5;
float beta = 1000, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 2000, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif

#ifdef PARAM_CLF
float alpha1 = 1500, alpha2 = 500, w1 = 1.5;
float beta = 30, w2 = 4;   //float beta = 10000, w2 = 4;
float rho = 10, w3 = 3.45; // 300
float phi = 0, w4 = 1;
float delta = 0, w5 = 1;
#endif
//========================================

#endif