#include "ForwardKinematics.h"

using namespace std;
using namespace MotionControl;

int main(int argc, char* argv[])
{
	vector<int> idx;
	Link ulink[LEG_JOINT_NUM];
	SetJointInfo(ulink);

	ForwardKinematics fk(ulink);

	idx = fk.FindRoute(LINK_2);

	for(size_t i=0;i<idx.size();i++)
		cout<<idx[i]<<endl;

	return 0;
}
