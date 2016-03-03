#include "ForwardKinematics.h"

using namespace MotionControl;

Matrix<double,3,3> ForwardKinematics::Rodrigues(Matrix<double,3,1> a, double q)
{
	return AngleAxisd(q,a).toRotationMatrix();
}

vector<int> ForwardKinematics::FindRoute(int to)
{
	vector<int> idx;
	int link_num = to;

	while(link_num != 0)
	{
		idx.push_back(link_num);
		link_num = ulink[link_num].parent;
	}
	reverse(idx.begin(), idx.end());
	return idx;
}

void ForwardKinematics::calcForwardKinematics(int rootlink)
{
	if(rootlink == -1)
		return ;
	if(rootlink != 0)
	{
		int parent = ulink[rootlink].parent;
		ulink[rootlink].p = ulink[parent].R * ulink[rootlink].b + ulink[parent].p;
		ulink[rootlink].R = ulink[parent].R * Rodrigues(ulink[rootlink].a, ulink[rootlink].q);
	}
	calcForwardKinematics(ulink[rootlink].sister);
	calcForwardKinematics(ulink[rootlink].child);
}
