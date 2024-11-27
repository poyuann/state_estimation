#include "HEIF_self.h"

HEIF_self::HEIF_self(int x_size=6) : HEIF(x_size)
{
	fusionNum = 0;
}

HEIF_self::~HEIF_self(){}

void HEIF_self::setNeighborEstData(std::vector<EIF_data> est_Data)
{
	fusionNum = 0;
	self_est_neighbor.insert(self_est_neighbor.end(), est_Data.begin(), est_Data.end());
	fusionNum = self_est_neighbor.size();
    weight.resize(fusionNum);
}
void HEIF_self::set_passiveEstData(std::vector<EIF_data> est_Data, int self_id)
{
	for(int i=0; i< est_Data.size(); i++)
	{
		for(int j=0; j< est_Data[i].passive_id.size(); j++)
		{
			if (est_Data[i].passive_id[j] == self_id)
			{	
				est_Data[i].s = est_Data[i].passive_s[j];
				est_Data[i].y = est_Data[i].passive_y[j];

				self_est_neighbor.push_back(est_Data[i]);
				
				break;
			}
		}
	}
	fusionNum = self_est_neighbor.size();
    weight.resize(fusionNum);

}
void HEIF_self::setSelfEstData(EIF_data self)
{
	self_est = self;
	self_est_neighbor.clear();
}

void HEIF_self::eighborEstDataCI()
{
	double trace_sum = 0.0;

	//////////////////////////// s, y ////////////////////////////
	for(int i=0; i<fusionNum; i++)
		trace_sum += self_est_neighbor[i].s.trace();
	for(int i=0; i<fusionNum; i++)
	{
		if(trace_sum !=0)
		{
			weight[i] = self_est_neighbor[i].s.trace()/trace_sum;
			weightedS += weight[i]*self_est_neighbor[i].s;
			weightedY += weight[i]*self_est_neighbor[i].y;
		}
	}
}

void HEIF_self::CI_combination()
{
	weightedOmega_hat += self_est.P_hat.inverse();
	weightedXi_hat += self_est.P_hat.inverse()*self_est.X_hat;
	weightedS += self_est.s;
	weightedY += self_est.y;
	fusedP = (weightedOmega_hat + weightedS).inverse();
	fusedX = fusedP*(weightedXi_hat + weightedY);
}

void HEIF_self::process()
{
	initialize();
	if(fusionNum > 0)
		eighborEstDataCI();
	CI_combination();
}

Eigen::MatrixXd HEIF_self::getS(){ return weightedS;}