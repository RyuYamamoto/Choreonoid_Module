#include <iostream>
#include <vector>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/Body>
#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

class Sample1Plugin : public Plugin
{
public:
    
    Sample1Plugin() : Plugin("Debug")
    {
		require("Body");
    }
    
    virtual bool initialize()
    {    
		ToolBar* bar = new ToolBar("Debug");

        addToolBar(bar);
		bar->addButton("Number Joints")->sigClicked().connect(bind(&Sample1Plugin::onButtonNumJoints,this));
		bar->addButton("Waist Pos_Rot")->sigClicked().connect(bind(&Sample1Plugin::onButtonPosRot,this));
        bar->addButton("Invese Kinematics")->sigClicked().connect(bind(&Sample1Plugin::onButtonInverseKinematics,this));
		return true;
    }

	void onButtonNumJoints()
	{
		ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
		BodyPtr body = bodyItems[0]->body();
		
		unsigned int n;
		n = body->numJoints();
		cout<<"Joint Number:"<<n<<endl;
	}

	void onButtonPosRot()
	{
		ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
		BodyPtr body = bodyItems[0]->body();
		
		Vector3 p(body->link("WAIST")->p());
		Matrix3 R(body->link("WAIST")->R());
		cout<<"x:"<<p(0)<<" y:"<<p(1)<<" z:"<<p(2)<<endl;
		cout<<"R:"<<R(0,0)<<" P:"<<R(1,1)<<" Y:"<<R(2,2)<<endl;
	}

	void onButtonInverseKinematics()
	{
		ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
		BodyPtr body = bodyItems[0]->body();
		
		//JointPath rarm = body->getJointPath(body->link("CHEST_JOINT2"),body->link("RARM_JOINT7"));
		//JointPath larm = body->getJointPath(bdoy->link("CHEST_JOINT2"),body->link("LARM_JOINT7"));
		JointPathPtr l_footlink = getCustomJointPath(body,body->link("WAIST"),body->link("LLEG_JOINT5"));
		JointPathPtr r_footlink = getCustomJointPath(body,body->link("WAIST"),body->link("RLEG_JOINT5"));

		//rarm->calcFowardKinematics();
		//l_link->calcForwardKinematics();
		l_footlink->calcForwardKinematics();
		r_footlink->calcForwardKinematics();

		Vector3 ltarget_p(body->link("LLEG_JOINT5")->p());
		Matrix3 ltarget_R(body->link("LLEG_JOINT5")->R());
		Vector3 rtarget_p(body->link("RLEG_JOINT5")->p());
		Matrix3 rtarget_R(body->link("RLEG_JOINT5")->R());

		ltarget_p(2) = ltarget_p(2) + 0.05;
		rtarget_p(2) = rtarget_p(2) + 0.05;

		if(l_footlink->calcInverseKinematics(ltarget_p,ltarget_R) && r_footlink->calcInverseKinematics(rtarget_p,rtarget_R))
		{
			bodyItems[0]->notifyKinematicStateChange(true);
		}
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)
