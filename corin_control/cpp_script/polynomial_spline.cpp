#include "polynomial_spline.h"

using namespace Eigen;

namespace polynomial_spline
{
	PolynomialSpline::PolynomialSpline()
	{
		int x = 0;
	}

	void PolynomialSpline::generate_spline(const Eigen::MatrixXd x,
																					const Eigen::VectorXd t, 
																					const float tint)
	{
		unsigned int ns = x.rows()-1;	// number of splines
		unsigned int sz = x.rows()-2;	// number of via points excl. start & final
		MatrixXd A(sz,sz);	// time
		MatrixXd tv(sz,3);
		MatrixXd v(x.rows(),3); 	// intermediate velocity
		MatrixXd C(sz,3); 	// intermediate position & time
		MatrixXd vo(3,1);		// initial velocity
		MatrixXd vf(3,1);		// final velocity
		
  	// determine number of intervals on spline 
  	int tns = int(ceil((t(t.rows()-1) - t(0))/tint)) + 1;
  	VectorXd ct(tns); 	// output timing array
		MatrixXd cp(tns,3); 	// output position array
		MatrixXd cv(tns,3); 	// output velocity array
		MatrixXd ca(tns,3); 	// output array array
		// std::cout << (t(t.rows()-1) - t(0))/tint << " " << int((t(t.rows()-1) - t(0))/tint)<< std::endl;
  	// Populate matrix for calculating intermediate velocity with via points (pg. 171)
		if (sz>0)
		{
			std::cout<< "in sz>" << sz << std::endl;
			for (int i=0; i<sz; i++) {
				unsigned int T0 = t(i+1)-t(i);
				unsigned int T1 = t(i+2)-t(i+1);

				// Compute A
				if (i-1+0 >= 0)
				{
					A(i,i-1+0) = T1; 	// skip first item
				}
				A(i,i-1+1) = 2*(T0+T1);
				if (i-1+2 != sz)
				{
					A(i,i-1+2) = T0;
				}
				// try{
				// 	A(i,i-1+2) = T0;
				// }
				// catch (...){
				// 	;
				// } // skip last item

				// Compute C - change to 3D
				for (int j=0; j<3; j++) {
					C(i,j) = 3*( T0*T0*(x(i+2,j)-x(i+1,j)) + T1*T1*(x(i+1,j)-x(i,j)) )/(T0*T1);

					if (i==0)
						C(i,j) = C(i,j) - T1*vf(j,0);
					else if (i==sz-1)
						C(i,j) = C(i,j) - T0*vf(j,0);
				}
			}
			// Solve for v
			tv = A.ldlt().solve(C);
			std::cout<< A <<std::endl;
			std::cout<< C <<std::endl;
			// std::cout<< tv <<std::endl;
		}
		v.block(1,0,sz,3) = tv;
		std::cout<< v <<std::endl;

		int sc = 0;
		// Compute coefficients
		for (int i=0; i<sz+1; i++) {
			float tk = t(i+1)-t(i);
			float qkx   = x(i,0);		float qky   = x(i,1);		float qkz   = x(i,2);
			float qkx_1 = x(i+1,0);	float qky_1 = x(i+1,1);	float qkz_1 = x(i+1,2);
			float vkx   = v(i,0);		float vky   = v(i,1);		float vkz   = v(i,2);
			float vkx_1 = v(i+1,0);	float vky_1 = v(i+1,1);	float vkz_1 = v(i+1,2);

			// compute spline coefficients for each dimension
			float a0x = qkx;
			float a1x = vkx;
			float a2x = (1/tk)*( 3*(qkx_1-qkx)/tk - 2*vkx - vkx_1 );
			float a3x = (1/(tk*tk))*( 2*(qkx-qkx_1)/tk + vkx + vkx_1 );
			// std::cout << "======================="  << std::endl;
			// std::cout << a0x << " " << a1x << " " << a2x << " " << a3x << std::endl;
			float a0y = qky;
			float a1y = vky;
			float a2y = (1/tk)*( 3*(qky_1-qky)/tk - 2*vky - vky_1 );
			float a3y = (1/(tk*tk))*( 2*(qky-qky_1)/tk + vky + vky_1 );

			float a0z = qkz;
			float a1z = vkz;
			float a2z = (1/tk)*( 3*(qkz_1-qkz)/tk - 2*vkz - vkz_1 );
			float a3z = (1/(tk*tk))*( 2*(qkz-qkz_1)/tk + vkz + vkz_1 );

			// compute trajectory values
			float lv = 0; //tint
			// float nv = 0.;
			// if (t(i+1)==t(t.rows()-1))
			// {
			// 	lv = 0.;
			// 	nv = 1;
			// }
			// Interpolate spline intervals
			for (float tq=t(i); tq<=t(i+1)-lv; tq+=tint) { 
				float td = tq - t[i];
				// std::cout << td << "\t" << t(i+1)-lv << " " << sc << " " << tns << std::endl;		
				float qpx = a0x + a1x*td + a2x*td*td + a3x*td*td*td;
				float qvx = a1x + 2*a2x*td + 3*a3x*td*td;
				float qax = 2*a2x + 6*a3x*td;

				float qpy = a0y + a1y*td + a2y*td*td + a3y*td*td*td;
				float qvy = a1y + 2*a2y*td + 3*a3y*td*td;
				float qay = 2*a2y + 6*a3y*td;

				float qpz = a0z + a1z*td + a2z*td*td + a3z*td*td*td;
				float qvz = a1z + 2*a2z*td + 3*a3z*td*td;
				float qaz = 2*a2z + 6*a3z*td;
				
				// concatenate segments together - size is <no_rows> by 3
				ct(sc) = tq;
				cp(sc,0) = qpx;	cv(sc,0) = qvx;	ca(sc,0) = qax;
				cp(sc,1) = qpy;	cv(sc,1) = qvy;	ca(sc,1) = qay;
				cp(sc,2) = qpz;	cv(sc,2) = qvz;	ca(sc,2) = qaz;
				sc++;
				
			}
		}
		std::cout << ct << std::endl;
		// std::cout << cp << std::endl;
	}
} 	// namespace polynomial_spline

int main () {
  polynomial_spline::PolynomialSpline* pspline = new polynomial_spline::PolynomialSpline();
	  
  // MatrixXd m(3,3);
  MatrixXd m(4,3);
  VectorXd t(4);

  m << 1,1,1,2,2,2,3,3,3,4,4,4;
  t(0) = 0;
  t(1) = 1;
  t(2) = 2;
  t(3) = 3;

  pspline -> generate_spline(m, t, 0.1);

  // rect.set_values (3,4);
  
  
  return 0;
}