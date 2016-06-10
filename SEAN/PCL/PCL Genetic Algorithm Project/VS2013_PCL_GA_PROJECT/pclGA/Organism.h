#pragma once
#include <string>
#include <iostream>
#include <bitset>

#define BITSET_SIZE 8
#define STRING_LENGTH 8

//*****************************************************************************
// STRUCT: Parameters
// desc: This structure contains all parameters for PCL functions used
//       by genetic algorithm
//*****************************************************************************
struct OrganismParameters
{
	int leafSize;
};

//*****************************************************************************
// CLASS: COrganism
// desc: This class contains all information for a single pcl dataflow.
//       This includes the parameters for functions in the dataflow, as well
//       as the binary string needed for a genetic algorithm
//*****************************************************************************
class COrganism
{
public:
	COrganism();
	~COrganism();

	/**
	* desc: delete parameters and set pointer to NULL
	*/
	void deleteParameters();

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
	* ret: -1 if an error occured,
	*       0 if successful
	*/
	int ConvertStringToParameters();

	/**
	* desc: sets binary string to passed value
	* param: data - string of binary information
	* ret: -1 if an error occured,
	*       0 if successful
	*/
	int setBinaryString(std::string data);


	/**
	* desc: gets binary string of organism
	* ret: string of binary data if it exists,
	*      empty string if no data
	*/
	std::string getBinaryString();

	/**
	* desc: add parameters object
	* param: pParameters - parameters pointer
	* ret: -1 if passed invalid data, 0 if sucessfull
	*/
	int setParameters(OrganismParameters* pParameters);

	/**
	* desc: returns Parameters object
	* ret: will return nullptr if no object exists
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

	/**
	* desc: save organism data to a log file
	* param: filename - name of file to save data to
	* post-cond: if file has not been created, this function will create the
	*            file. Information will be appended to the specified file if
	*            it already exists.
	*/
	std::string convertIntToBinary(unsigned int num);
};

