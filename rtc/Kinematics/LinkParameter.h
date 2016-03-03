/*
 * @file		LinkParameter.h
 * @brief		Constant Value of Link for Accelite
 * @author	Ryu Yamamoto
 * @date		2016/02/26
 */

#ifndef _LINK_PARAMETER_
#define _LINK_PARAMETER_

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <limits>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;

static const double eps = numeric_limits<double>::epsilon();
static const double pi = boost::math::constants::pi<double>();

static const string joint_name[] = {
	"WAIST",
	"LINK_1",
	"LINK_2",	
};

enum
{
	WAIST=0,
	LINK_1,
	LINK_2,
	LEG_JOINT_NUM
};

static const int parent[LEG_JOINT_NUM] = {-1,0,1};
static const int child[LEG_JOINT_NUM] = {1,2,-1};
static const int sister[LEG_JOINT_NUM] = {-1,-1,-1};

static const double LinkAxis[LEG_JOINT_NUM][3] = {
	{0,0,0},	//WASIT
	{0,1,0},	//LINK_1
	{0,1,0}		//LINK_2
};

//For test
static const double LinkPos[LEG_JOINT_NUM][3] = {
	{0.0f, 0.0f, 0.0f},		//WAIST
	{0.0f, -50.0f, 0.0f},	//LINK_1
	{0.0f, 0.0f, -40.0f},	//LINK_2
};

#endif
