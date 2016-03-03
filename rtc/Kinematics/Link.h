/*
 * @file		Link.h
 * @brief		Link Information
 * @author	Ryu Yamamoto
 * @date		2016/02/26
 */

#ifndef _LINK_
#define _LINK_

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "LinkParameter.h"

using namespace std;
using namespace Eigen;

namespace MotionControl
{
	struct Link
	{
		string joint_name;
		int sister;
		int child;
		int parent;
		Matrix<double,3,1> p;
		Matrix<double,3,3> R;
		Matrix<double,3,1> a;
		Matrix<double,3,1> b;	
		double q;						
		double dq;						
		double ddq;

		Link() : 
			p(Matrix<double,3,1>::Zero()),
			R(Matrix<double,3,3>::Identity()),
			a(Matrix<double,3,1>::Zero()),
			b(Matrix<double,3,1>::Zero()),
			q(0.0),
			dq(0.0),
			ddq(0.0)
		{
		}
	};
	extern "C" void SetJointInfo(struct Link *link);
};

#endif
