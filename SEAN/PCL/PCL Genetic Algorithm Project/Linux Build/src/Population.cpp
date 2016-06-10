#include "Population.h"

// TODO: add check if [id] is within array bounds.
//       e.g. id >= LIST_SIZE return -1
// TODO: change int id to unsigned int id

CPopulation::CPopulation()
{
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		mValid[i] = false;
		mExecutionTime[i] = -1;
		mFitnessList[i] = 0;
		mOrganismList[i] = nullptr;
	}
}


CPopulation::~CPopulation()
{
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		mValid[i] = false;
		mExecutionTime[i] = -1;
		mFitnessList[i] = 0;
		
		// take care of organisms
		if (mOrganismList[i] != nullptr)
		{
			delete mOrganismList[i];
			mOrganismList[i] = nullptr;
		}
	}
}


/**
* desc: add organism to population
* param: id - index of organism
*        organism - ptr to new organism to add
* pre-cond: organism must already be initialized
* post-cond: organism added to population.
* ret: 0 if organism added sucessfully,
*     -1 if an organism already exists at index id
*/
int CPopulation::addOrganism(int id, COrganism* organism)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	// check if index already contains an organism
	if (mOrganismList[id] == nullptr)
	{   // add new organism
		mOrganismList[id] = organism;
		return 0;
	}

	return -1; // index already has data
}


/**
* desc: remove organism from population
* param: id - index of organism
* post-cond: organism removed from population
* ret: 0 if successful,
*     -1 if index is out of array bounds
*/
int CPopulation::removeOrganism(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	// check if id already empty
	if (mOrganismList[id] != nullptr)
	{   // remove organism
		delete mOrganismList[id];
		mOrganismList[id] = nullptr;

		// clear other data
		mValid[id] = false;
		mExecutionTime[id] = -1;
		mFitnessList[id] = 0;
		return 0;
	}

	return 0;
}


/**
* desc: get a pointer to a specific organism
* param: id - index of organism
* pre-cond: organism must exist
* ret: nullptr is returned if id is outside array bounds
*      or if no organism exists at specified index; otherwise,
*      a ptr to the organism is returned.
*/
COrganism* CPopulation::getOrganism(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return nullptr;
	}

	return mOrganismList[id];
}


/**
* desc: display organisms in population. This includes
*       valid, execution time, and parameters of population
* param: id - index of organism to be displayed
* post-cond: organism data displayed to standard out
* ret: 0 if organism was found,
*     -1 if organism was not found
*/
int CPopulation::displayOrganism(int id)
{
	// TODO: implement function
	return 1000; // dummy value
}


/**
* desc: save organism to a file. This includes valid,
*       execution time, and parameters of organism
* param: id - index of organism to be saved
*        filename - name of file to append data too
* post-cond: data appended to file
* ret: 0 if list is not empty, and
*     -1 if list is empty
*/
int CPopulation::saveOrganism(int id, std::string filename)
{
	// TODO: implement function
	return 1000; // dummy value
}


/**
* desc: set the fitness of an organism
* param: id - index of organism
*        fitness - fitness to set organism too
* ret: 0 if successful,
*     -1 if index is out of array bounds
*/
int CPopulation::setOrganismFitness(int id, int fitness)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}
	
	// set fitness
	mFitnessList[id] = fitness;
	return 0; 
}


/**
* desc: get the fitness value of an organism
* param: id - index of organism
* ret: returns fitness of organism 
*/
int CPopulation::getOrganismFitness(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	return mFitnessList[id];
}


/**
* desc: add/replace parameters of specified organism
* param: id - index of organism
*        parameters - ptr to parameter to add to organism
* pre-cond: organism must already be initialized.
* post-cond: parameters added to specified organism
* ret: 0 on success,
*     -1 if an error occured
*/
int CPopulation::setOrganismParameters(int id, OrganismParameters* parameters)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	// check if organism exists
	if (mOrganismList[id] != nullptr)
	{
		mOrganismList[id]->setParameters(parameters);
		return 0;
	}

	return -1;  // no organism exists
}


/**
* desc: get parameters of specified organism
* param: id - index of organism
* pre-cond: organism must have initialized parameters
* post-cond: parameters added to specified organism
* ret: nullptr if no parameters, else
*      ptr to parameters is returned
*/
OrganismParameters* CPopulation::getOrganismParameters(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return nullptr;
	}

	// check if organism exists
	if (mOrganismList[id] != nullptr)
	{   // get the parameters
		OrganismParameters *result;  // catch returned value

		// return parameters. may be nullptr if none exist
		return mOrganismList[id]->getParameters();
	}

	return nullptr;
}


/**
* desc: set if organism is valid
* param: id - index of organism
* pre-cond: organism must exist
* post-cond: organism valid flag is set
* ret: 0 if successful
*     -1 if id is out of array bounds
*/
int CPopulation::setOrganismValid(int id, bool valid)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	// set organism to valid if organism exists
	if (mOrganismList[id] != nullptr)
	{
		mValid[id] = valid;
		return 0;
	}

	return -1;  // organism does not exist
}


/**
* desc: get if organism is valid
* param: id - index of organism
* pre-cond: organism must exist
* ret: value of valid flag or false if
*      out of bounds
*/
bool CPopulation::getOrganismValid(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return false;
	}

	return mValid[id];
}


/**
* desc: set organism execution time
* param: id - index of organism
*        executionTime - time needed for organism to execute
* pre-cond: organism must exist
* ret: 0 if successful
*     -1 if id is out of array bounds
*/
int CPopulation::setOrganismExecutionTime(int id, int executionTime)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	// check if organism exists
	if (mOrganismList[id] != nullptr)
	{
		mExecutionTime[id] = executionTime;
		return 0;
	}

	return -1;  // organism does not exist
}


/**
* desc: get organism execution time
* param: id - index of organism
* pre-cond: organism must exist
* ret: execution time in milliseconds. result of -1
*      means organism has not had its execution time
*      set yet or organism does not exist.
*/
int CPopulation::getOrganismExecutionTime(int id)
{
	// check array bounds
	if (id >= LIST_SIZE)
	{
		return -1;
	}

	return mExecutionTime[id];
}


/**
* desc: calculate the fitness value of each organism in the population.
*       fitness is based upon the validity and execution time of an
*       organism.
* post-cond: fitness values are calculated and added to fitness list
* ret: 0 if successful, else
*     -1 if an error occured
*/
int CPopulation::calculatePopulationFitness()
{
	// TODO: implement fitness function
	return 1000;
}


/**
* desc: display organisms in population. This includes
*       valid, execution time, and parameters of population
* param: organism - ptr to organism to be removed
* post-cond: organism removed from population
* ret: -1 if list is empty, 0 if list is not empty
*/
int CPopulation::displayPopulation()
{
	// TODO: implement function
	return 1000;
}


/**
* desc: save population to a file. This includes valid,
*       execution time, and parameters of organism
* param: filename - name of file to append data too
* post-cond: data appended to file
* ret: 0 if list is not empty, and
*     -1 if list is empty or an error occured
*/
int CPopulation::savePopulation(std::string filename)
{
	// TODO: implement this function
	return 1000; // dummy value
}
