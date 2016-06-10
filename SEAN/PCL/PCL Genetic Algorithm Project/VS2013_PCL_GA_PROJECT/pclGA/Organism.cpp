#include "Organism.h"

// TODO: add set and get functions for each parameter
// TODO: think about making parameters part of organism
//       arguments instead of being separate

COrganism::COrganism()
{
	mBinaryString = "";
	mParameters = nullptr;
}


COrganism::~COrganism()
{
	mBinaryString = "";
	deleteParameters();
}

/**
* desc: delete parameters and set pointer to NULL
*/
void COrganism::deleteParameters()
{
	// check if parameters exist and delete
	if (mParameters != nullptr)
	{
		mParameters->leafSize = 0;
		delete mParameters;
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
	// TODO: update function once more than 1 parameter is added
	mBinaryString = convertIntToBinary(mParameters->leafSize);
	return 0;
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
	// TODO: modify when more parameters added
	// NOTE: potential problem if stoi() returns error
	mParameters->leafSize = std::stoi(mBinaryString, nullptr, 2);
	return 0; 
}


/**
* desc: sets binary string to passed value
* param: data - string of binary information
* ret: -1 if an error occured,
*       0 if successful
*/
int COrganism::setBinaryString(std::string data)
{
	mBinaryString = data;
	return 0;
}


/**
* desc: gets binary string of organism
* ret: string of binary data if it exists,
*      empty string if no data
*/
std::string COrganism::getBinaryString()
{
	return mBinaryString;
}


/**
* desc: add parameters object
* param: pParameters - parameters pointer
* ret: -1 if passed invalid data, 0 if sucessfull
*/
int COrganism::setParameters(OrganismParameters* pParameters)
{
	// that a valid object was passed
	if (pParameters == nullptr)
	{
		return -1;
	}

	// check if organism has parameters
	if (mParameters == nullptr)
	{   // add new parameters
		mParameters = pParameters;
		return 0;
	}
	else
	{   // need to delete original parameters
		deleteParameters();
		mParameters = pParameters;
		return 0;
	}
}

/**
* desc: returns Parameters object
* ret: will return nullptr if no object exists
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


std::string COrganism::convertIntToBinary(unsigned int num)
{
	// store num as 8 bit value
	std::bitset<BITSET_SIZE> bits = num;
	return bits.to_string();
}


