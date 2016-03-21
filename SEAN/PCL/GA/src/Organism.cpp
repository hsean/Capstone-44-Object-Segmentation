#include "Organism.h"


COrganism::COrganism()
{
	mBinaryString = "";
	mParameters = nullptr;
}


COrganism::~COrganism()
{
	mBinaryString = "";

	// check if parameters exist
	if (mParameters != nullptr)
	{
		mParameters = nullptr;
	}
}


/**
* desc: convert parameters in OrganismParameters structure into a single
*       binary string to be used in genetic algorithm operations
* param:
* ret: -1 if an error occured,
*       0 if successful
*/
int COrganism::ConvertParametersToString()
{
	// TODO: implement function
	return 1000; // dummy value
}


/**
* desc: parse binary string and place parsed data into OrganismParameters
*       structure. These parameters will be used to test the fitness
*       value of the organism.
* param:
* ret: -1 if an error occured,
*       0 if successful
*/
int COrganism::ConvertStringToParameters()
{
	// TODO: implement function
	return 1000; // dummy value
}


/**
* desc: pass pointer to COrganism structure used to store parameters
* param: parameters - structure holding all application specific parameters.
*                     In this case, parameters are for object segmentation
*                     using the PCL library.
*/
void COrganism::setParameters(OrganismParameters* parameters)
{
	mParameters = parameters;
}


/**
* desc: get a pointer to the OrganismParameters structure
* ret: pointer to mParameters. May be nullptr if no parameters set
*/
OrganismParameters* COrganism::getParameters()
{
	return mParameters;
}


/**
* desc: display organism data to standard out
*/
void COrganism::displayOrganism()
{
	std::cout << "Genome: " << mBinaryString << std::endl;
}


/**
* desc: save organism data to a log file
* param: filename - name of file to save data to
* post-cond: if file has not been created, this function will create the
*            file. Information will be appended to the specified file if
*            it already exists.
*/
int COrganism::saveOrganism(std::string filename)
{
	// TODO: implement file IO
	return 1000; // dummy value
}

