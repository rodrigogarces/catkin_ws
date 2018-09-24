#ifndef OPTMIZATION_METHODS_H_
#define OPTMIZATION_METHODS_H_

double* SimplexMethod(double (*func)(double[], int n), double start[], int n, double EPSILON,
               double scale, int MAX_IT, double ALPHA, double BETA, double GAMMA);

#endif