#pragma once
#include <string>
#include <iostream>


//*****************************************************************************
// STRUCTURE: OrganismParameters
// desc: This structure contains the parameters for a pcl dataflow.
//       This structure needs to be modified if more parameters are added
//       to a dataflow.
//*****************************************************************************
struct OrganismParameters
{
	double leafSize;
};


//*****************************************************************************
// CLASS: COrganism
// desc: This class contains all information for a single pcl dataflow.
//       This includes the parameters for functions in the dataflow, as well
//       as the execution time of the dataflow.
//*****************************************************************************
class COrganism
{
public:
	COrganism();
	~COrganism();

	/**
	* desc: convert parameters in OrganismParameters structure into a single
	*       binary string to be used in genetic algorithm operations
	* param: 
	* ret: -1 if an error occured,
	*       0 if successful
	*/
	int ConvertParametersToString();

	/**
	* desc: parse binary string and place parsed data into OrganismParameters 
	*       structure. These parameters will be used to test the fitness
	*       value of the organism.
	* param:
	* ret: -1 if an error occured,
	*       0 if successful
	*/
	int ConvertStringToParameters();

	/**
	* desc: pass pointer to COrganism structure used to store parameters
	* param: parameters - structure holding all application specific parameters.
	*                     In this case, parameters are for object segmentation
	*                     using the PCL library.
	*/
	void setParameters(OrganismParameters* parameters);

	/**
	* desc: get a pointer to the OrganismParameters structure
	* ret: pointer to mParameters. May be nullptr if no parameters set
	*/
	OrganismParameters* getParameters();

	/**
	* desc: display organism data to standard out
	*/
	void displayOrganism();

	/**
	* desc: save organism data to a log file
	* param: filename - name of file to save data to
	* post-cond: if file has not been created, this function will create the 
	*            file. Information will be appended to the specified file if
	*            it already exists.
	*/
	int saveOrganism(std::string filename);

private:
	std::string mBinaryString;
	OrganismParameters* mParameters;
};

