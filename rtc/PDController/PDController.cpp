#include "PDController.h"

using namespace std;
using namespace cnoid;

namespace{
	const char* PDController_spec[] =
	{
		"implementation_id", "PDController",
		"type_name",         "PDController",
		"description",       "Robot TorqueController component",
		"version",           "0.1",
		"vendor",            "CIT",
		"category",          "Generic",
		"activity_type",     "DataFlowComponent",
		"max_instance",      "10",
		"language",          "C++",
		"lang_type",         "compile",
		""
	};
}

PDController::PDController(RTC::Manager* manager)
	: RTC::DataFlowComponentBase(manager),
	m_angleIn("q", m_angle),
	m_torqueOut("u", m_torque),
	dt(0.001),
	dq_old(0.0)
{
}

PDController::~PDController()
{
}

void PDController::ReadGain(size_t numJoints,std::vector<double> &pgain,std::vector<double> &dgain)
{
	FILE *PDGain;
	float temp_pgain,temp_dgain;
	char joint_name[20];
	PDGain = fopen("/usr/lib/choreonoid-1.5/rtc/pdgain.txt","r");

	pgain.clear(); dgain.clear();

	pgain.resize(numJoints);
	dgain.resize(numJoints);
	angleRef.resize(numJoints);
	q_old.resize(numJoints);
	q_old_ref.resize(numJoints);
	m_torque.data.length(numJoints);

	int i=0;
	while(fscanf(PDGain,"%f %f",&temp_pgain,&temp_dgain) != EOF)
	{
		pgain[i] = temp_pgain;
		dgain[i] = temp_dgain;
		//cout<<pgain[i]<<" "<<dgain[i]<<endl;
		i++;
	}
}

RTC::ReturnCode_t PDController::onInitialize()
{
	// Set InPort buffers
	addInPort("q", m_angleIn);
	//addInPort("refq",m_anglelRefIn);

	// Set OutPort buffenr
	addOutPort("u", m_torqueOut);

	return RTC::RTC_OK;
}

RTC::ReturnCode_t PDController::onActivated(RTC::UniqueId ec_id)
{
	if(m_angleIn.isNew()){
		m_angleIn.read();
		ReadGain(m_angle.data.length(),pgain,dgain);
	}

	for(size_t i=0;i<m_angle.data.length();i++){
		angleRef[i] = q_old[i] = q_old_ref[i] = m_angle.data[i];
	}

	return RTC::RTC_OK;
}

RTC::ReturnCode_t PDController::onDeactivated(RTC::UniqueId ec_id)
{
	cout<<"Deactivated"<<endl;
	return RTC::RTC_OK;
}

RTC::ReturnCode_t PDController::onExecute(RTC::UniqueId ec_id)
{
	if(m_angleIn.isNew()){
		m_angleIn.read();
	}

	for(size_t i=0; i < m_angle.data.length(); i++){
		double q_ref = angleRef[i];
		double q = m_angle.data[i];
		double dq_ref = (q_ref - q_old_ref[i]) / dt;
		double dq = (q - q_old[i]) / dt;
		m_torque.data[i] = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
		if(i == 37){
			double ddq = (dq - dq_old)/dt;
			m_torque.data[i] = (1.0 - dq) * 100 - 0.2 * ddq;
			dq_old = dq;
		}
		q_old[i] = q;
	}
	q_old_ref = angleRef;
	m_torqueOut.write();

	return RTC::RTC_OK;
}

extern "C"
{
	DLL_EXPORT void PDControllerInit(RTC::Manager* manager)
	{
		coil::Properties profile(PDController_spec);
		manager->registerFactory(profile,
				RTC::Create<PDController>,
				RTC::Delete<PDController>);
	}
};
