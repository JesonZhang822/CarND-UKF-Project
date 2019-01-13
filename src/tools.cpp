#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;
    
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size

	if (estimations.size() == 0 || ground_truth.size() == 0){
	    
	    cout << "the estimation vector size is zero"<< endl;
	    
	    return rmse;
	 
	}
	if (estimations.size() != ground_truth.size()){
	    
	    cout << "estimations size not equal ground_truth size" << endl;
	    return rmse;
	}
	
	VectorXd x(4);
	
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
             
        x = estimations[i] - ground_truth[i];
        x = x.array()*x.array();

       	rmse += x;
	}

	//calculate the mean
	rmse = rmse / estimations.size();
	

	//calculate the squared root
	rmse = rmse.array().sqrt();

	cout << "The rmse is : " << rmse << endl;


	return rmse;
}

double Tools::NormalizeAngle (double angle ){

    while (angle > M_PI) angle -= 2.* M_PI;
    while (angle < -M_PI) angle += 2.* M_PI;

    return angle;
}