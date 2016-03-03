/*
 * @file		ForwardKinematics.h
 * @brief		Calculation ForwardKinematics
 * @author	Ryu Yamamoto
 * @date		2016/03/03
 */

#ifndef _FORWARD_KINEMATICS_
#define _FORWARD_KINEMATICS_

#include "Link.h"

namespace MotionControl
{
	class ForwardKinematics
	{
	public:
		struct Link *ulink;
	public:
		ForwardKinematics(Link *ulink)
		{
			this->ulink = ulink;
		}
		void calcForwardKinematics(int rootlink);
		vector<int> FindRoute(int to);
		Matrix<double,3,3> Rodrigues(Matrix<double,3,1> a, double q);
	};
};

#endif
