#ifndef POLYNOMIAL_SPLINE_H
#define POLYNOMIAL_SPLINE_H

#include <cmath> 
#include <math.h> 
#include <typeinfo>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <iostream>

namespace polynomial_spline
{
	class PolynomialSpline
	{
	public:
		PolynomialSpline();

		void generate_spline(const Eigen::MatrixXd x,
								const Eigen::VectorXd t, 
								const float tint);
	private:
		
	};
}

#endif