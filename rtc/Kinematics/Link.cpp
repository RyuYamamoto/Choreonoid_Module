#include "Link.h"

using namespace MotionControl;

extern "C" void SetJointInfo(struct Link *link)
{
	for(int i=0;i<LEG_JOINT_NUM;i++)
	{
		link[i].joint_name = joint_name[i];
		link[i].parent = parent[i];
		link[i].child = child[i];
		link[i].sister = sister[i];
		for(int j=0;j<3;j++)
		{
			link[i].a(j) = LinkAxis[i][j];
			link[i].b(j) = LinkPos[i][j];
		}
	}
}