#pragma once

#define MAX_FFA	1000
#define MAX_D	1000

class FireflyAlgorithm
{
private:
    int D = 2;			    // dimension of the problem
    int n = 20;			    // number of fireflies
    int MaxGeneration;		// number of iterations
    int NumEval;			// number of evaluations
    int Index[MAX_FFA];		// sort of fireflies according to fitness values

public:

	FireflyAlgorithm(int dim);
	~FireflyAlgorithm();
	double alpha_new(double alpha, int NGen);
	void InicializaPopulacao();
	void OrdenaPopulacao();
    void Selecao();
    void MovimentaPopulacao();
};

