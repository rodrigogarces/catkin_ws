#pragma once
class Firefly
{
private:
	double position[2];

public:
	double bfitness;
	double* bposition;
	double fitness;
	double intensity;

	Firefly(int dim);
	~Firefly();
	int CompareTo(Firefly other);
	void SetPosicao(double pos[2]);
	double* GetPosicao();
};

