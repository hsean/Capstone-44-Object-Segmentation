#include "GeneticAlgorithm.h"


CGeneticAlgorithm::CGeneticAlgorithm()
{
	mParent1 = nullptr;
	mParent2 = nullptr;
	mPopulation = nullptr;
	mCurrentOrganism = 0;
	mCurrentGeneration = 0;
}


CGeneticAlgorithm::~CGeneticAlgorithm()
{
	// manage pointers
	if (mParent1 != nullptr)
	{
		delete mParent1;
		mParent1 = nullptr;
	}

	if (mParent2 != nullptr)
	{
		delete mParent2;
		mParent2 = nullptr;
	}

	// delete populations
	deleteAllPopulations();
	
	// set to safe values
	mCurrentOrganism = 0;
	mCurrentGeneration = 0;
}


/**
* desc: compute the fitness value of all organisms in a population
* param: generation - population to to find fitness falues for
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::computePopulationFitness(int generation)
{
	// TODO: implement function
	return 1000; // dummy value
}


/**
* desc: set first parent organism
* param: parent1 - ptr to organism with highest fitness value
* pre-cond: fitness values must already be generated for a population
* ret: 0 if successful,
*     -1 if an error occured
*/
int CGeneticAlgorithm::setParent1(COrganism* parent1)
{
	mParent1 = parent1;
	return 0;
}


/**
* desc: get ptr to first parent organism
* ret: ptr to parent 1 if successful,
*      nullptr if parent 1 does not yet exist
*/
COrganism* CGeneticAlgorithm::getParent1()
{
	return mParent1;
}


/**
* desc: set second parent organism
* param: parent2 - ptr to organism with second highest fitness value
* pre-cond: fitness values must already be generated for a population
* ret: 0 if successful,
*     -1 if an error occured
*/
int CGeneticAlgorithm::setParent2(COrganism* parent2)
{
	mParent2 = parent2;
}


/**
* desc: get ptr to second parent organism
* ret: ptr to parent 2 if successful,
*      nullptr if parent 2 does not yet exist
*/
COrganism* CGeneticAlgorithm::getParent2()
{
	return mParent2;
}


/**
* desc: set the current organism in current population
* param: num - value to set current organism as
*/
void CGeneticAlgorithm::setCurrentOrganism(int num)
{
	mCurrentOrganism = num;
}


/**
* desc: get the value of current organism
* ret: value of current organism
*/
int CGeneticAlgorithm::getCurrentOrganism()
{
	return mCurrentOrganism;
}


/**
* desc: increment the value of the current organism variable
* ret: value of current organism after being incremented
*/
int CGeneticAlgorithm::incrementCurrentOrganism()
{
	return (++mCurrentOrganism);
}


/**
* desc: set the current generation
* param: num - value to set generation as
*/
void CGeneticAlgorithm::setGeneration(int num)
{
	mCurrentGeneration = num;
}


/**
* desc: get the value of current generation
* ret: value of current generation
*/
int CGeneticAlgorithm::getGeneration()
{
	return mCurrentGeneration;
}


/**
* desc: increment the value of the current generation variable
* ret: value of current generation after being incremented
*/
int CGeneticAlgorithm::incrementGeneration()
{
	return (++mCurrentGeneration);
}


/**
* desc: return index of the two organisms with the highest fitness values
* param: parent1 - ptr to organism with the highest fitness value
*        parent2 - ptr to organism with the second highest fitness value
* pre-cond: fitness values for current generation must already be set
* post-cond: index of both organisms are passed back through function arguments
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::getParents(COrganism* parent1, COrganism* parent2)
{
	// TODO: implement function
	return 1000; // dummy variable
}


/**
* desc: create a brand new population using the current population
* param: parent1 - ptr to first organism to be used to generate new population
*        parent2 - ptr to second organism to be used to generate new population
* pre-cond: fitness values for current generation must already be set
* post-cond: current generation will be incremented
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::generateNewPopulation(COrganism* parent1, COrganism* parent2)
{
	// TODO: implement function
	return 1000; // dummy variable
}


/**
* desc: delete all populations in list
*/
void CGeneticAlgorithm::deleteAllPopulations()
{
	// step through each population node and delete everything
	PopulationNode* current = mPopulation;
	PopulationNode* previous = current;

	// delete each node in list if head is not nullptr
	while (current != nullptr)
	{
		current = current->next;  // hold next node
		previous->next = nullptr;
		previous->prev = nullptr;
		
		// check if population data is already empty
		if (previous->data != nullptr)
		{
			delete previous->data;
			previous->data = nullptr;
		}

		// check if population is already null
		if (previous->population != nullptr)
		{
			delete previous->population;
			previous->population = nullptr;
		}

		// set generation to safe value
		previous->generation = 0;

		// prepare for next node
		previous = current;
	}

	// make sure the head of the linked list is nullptr
	if (mPopulation != nullptr)
	{
		mPopulation = nullptr;
	}
}


/**
* desc: display all data for current population
* param: generation to display
* ret: 0 if successful
*     -1 if current generation does not exist
*/
int CGeneticAlgorithm::displayPopulation(int generation)
{
	// TODO: implement function
	return 1000; // dummy variable
}


/**
* desc: save current population to a file
* param: filename - name of file to save data to
*        generation - generation to save
* recommendation: have fitness values already set
* post-cond: data appended to file if it already exists
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::savePopulation(int generation, std::string filename)
{
	// TODO: implement function
	return 1000; // dummy variable
}


/**
* desc: display statistics for population as specified by the generation
* param: generation - generation to save
* post-cond: data displayed to standard out
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::displayPopulationData(int generation)
{
	// TODO: implement function
	return 1000; // dummy variable
}


/**
* desc: save population statistics to a file
* param: filename - name of file to save data to
*        generation - generation to save
* recommendation: have fitness values already set
* post-cond: data appended to file if it already exists
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::savePopulationData(int generation, std::string filename)
{
	// TODO: implement function
	return 1000; // dummy variable
}

