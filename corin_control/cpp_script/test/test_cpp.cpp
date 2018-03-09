#include "iostream"
#include "stdio.h"
#include <Eigen/Dense>

using namespace std;

int main()
{
	int m1[2][2], m2[2][2], m3[2][2]; 		// create two 2x2 matrix
	/*
	m1[0] = 1;
	m1[1] = 2;
	m1[2] = 3;
	cout<< m1[0] << endl;
	cout<< m1[1] << endl;
	cout<< m1[2] << endl;
	cout<< m1[3] << endl;
	cout<< m1[4] << endl;
	cout<< m1[5] << endl;
	*/
	m1[0][0] = 1;
	m1[0][1] = 2;
	m1[1][0] = 3;
	m1[1][1] = 4;

	m2[0][0] = 1;
	m2[0][1] = 2;
	m2[1][0] = 3;
	m2[1][1] = 4;

	m3[0][0] = m1[0][0]*m2[1][1];
	// multiplication can only be done element-wise
	
	cout << m3[0][0] << endl;
	cout << m3[0][1] << endl;
	cout << m3[1][0] << endl;
	cout << m3[1][1] << endl;
}