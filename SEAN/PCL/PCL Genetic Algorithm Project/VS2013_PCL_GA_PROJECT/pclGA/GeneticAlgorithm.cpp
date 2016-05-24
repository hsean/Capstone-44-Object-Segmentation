#include "GeneticAlgorithm.h"


CGeneticAlgorithm::CGeneticAlgorithm()
{
	mParent1 = nullptr;
	mParent2 = nullptr;
	mGenerationList = nullptr;
	mCurrentOrganism = 0;
	mCurrentGeneration = 0;
}


CGeneticAlgorithm::~CGeneticAlgorithm()
{
	// manage pointers
	mParent1 = nullptr;
	mParent2 = nullptr;

	// delete populations
	deleteAllPopulations();
	
	// set to safe values
	mCurrentOrganism = 0;
	mCurrentGeneration = 0;
}


/**
* desc: creates first generation of organisms each with a random binary
*       string to determine the parameters.
* pre-cond: class must not yet be initialized
* post-cond: generation 1 created
* ret: 0 on success,
*     -1 if failed
*/
int CGeneticAlgorithm::init()
{
	//TODO: finish method
	// check that class has not already been initialized
	if (mGenerationList != nullptr)
	{   // already been initialized
		return -1;
	}

	// add first node to generation list
	PopulationNode* pNode = new PopulationNode;
	CPopulation* pPopulation = new CPopulation;
	CPopulationData* pPopulationData = new CPopulationData;
	mCurrentGeneration = 0;             // first generation
	mGenerationList = pNode;            // set head of generation list
	pNode->population = pPopulation;    // hold population
	pNode->data = pPopulationData;      // hold population data
	pNode->generation = 0;              // set nodes generation id
	pNode->next = nullptr;
	pNode->prev = nullptr;

	// create first population
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		// create new organism
		COrganism* pOrganism = new COrganism;
		OrganismParameters* pParameters = new OrganismParameters;

		// generate random string for organism
		std::string rndString = rndBinaryString(STRING_LENGTH);

		// initialize parameters
		pParameters->leafSize = 0;

		// add parameters to organism
		if (pOrganism->setParameters(pParameters) < 0)
		{   // failed to set parameters
			return -1;
		}

		// initialize the organism
		if (pOrganism->setBinaryString(rndString) < 0)
		{   // failed to set string
			return -1;
		}

		// TODO: think about making it so that only the string
		//       can be manipulated outsite of COrganism
		if (pOrganism->ConvertStringToParameters() < 0)
		{   // failed to convert string to parameters
			return -1;
		}

		if (pNode->population->addOrganism(i, pOrganism) < 0)
		{   // failed to add organism
			return -1;
		}
	}

	return 0;
}


/**
* desc: compute the fitness value of all organisms in a population
* param: generation - population to to find fitness falues for
* post-cond: parent1 and parent2 are updated along with population data
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::computePopulationFitness(int generation)
{
	PopulationNode* holdGeneration;

	// check if a population can be found
	if ((holdGeneration = findPopulation(generation)) == nullptr)
	{   // generation not found
		return -1;
	}

	// compute fitness and check for success
	if (holdGeneration->population->calculatePopulationFitness() < 0)
	{   
		return -1;
	}
	else
	{   // update population data
		if (updatePopulationData(holdGeneration->population, holdGeneration->data) < 0)
		{   // an error occured when updating population data
			return -1;
		}
	}
	// set parents now that fitness has been calculated
	if (setParent1(holdGeneration->population->getOrganism(holdGeneration->data->getParent1Index())) < 0)
	{   // failed to set parent 1
		return -1;
	}
	if (setParent2(holdGeneration->population->getOrganism(holdGeneration->data->getParent2Index())) < 0)
	{   // failed to set parent 2
		return -1;
	}
	return 0;  // everything set
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
	// check that data was passed
	if (parent1 == nullptr)
	{
		return -1;
	}

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
	// check that data was passed
	if (parent2 == nullptr)
	{
		return -1;
	}

	mParent2 = parent2;
	return 0;
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
* desc: get a pointer to an organism in the population
* param: id - index of organism to fetch
*        generation - generation to get organism from
* ret: returns pointer to organism if it exists,
*      else NULL is returned
*/
COrganism* CGeneticAlgorithm::getOrganism(int id, int generation)
{
	PopulationNode* pNode;

	// check that id is not out of bounds
	if (id >= LIST_SIZE)
	{   // out of bounds
		return nullptr;
	}

	// find population
	if ((pNode = findPopulation(generation)) == nullptr)
	{   // population not found
		return nullptr;
	}

	// return organism
	return pNode->population->getOrganism(id);
}


/**
* desc: get a pointer to a population
* param: generation - generation to get population from
* ret: returns pointer to population if it exists,
*      else NULL is returned
*/
CPopulation* CGeneticAlgorithm::getPopulation(int generation)
{
	PopulationNode* pNode;

	// find population
	if ((pNode = findPopulation(generation)) == nullptr)
	{   // population not found
		return nullptr;
	}

	// return organism
	return pNode->population;
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
	// check if parents have been set
	if ((parent1 = mParent1) == nullptr)
	{   // parent1 not yet set
		return -1;
	}
	if ((parent2 = mParent2) == nullptr)
	{   // parent2 not yet set
		return -1;
	}

	return 0;
}


/**
* desc: create a brand new population using the current population
* param: parent1 - binary string of parent 1
*        parent2 - binary string of parent 2
*        generation - generation id of new generation
*        mutationRate - rate int which mutation mill effect children. (e.g. 0.05 = 5%)
* pre-cond: fitness values for current generation must already be set
* post-cond: current generation will be incremented
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::addNewPopulation(std::string parent1, std::string parent2, 
	                                    int generation, double mutationRate)
{
	// check that generation list has been initialized
	if (mGenerationList == nullptr)
	{
		return -1;
	}

	// check that passed strings are same length
	if (parent1.length() != parent2.length())
	{
		return -1;
	}

	// create new population node
	PopulationNode* node (new PopulationNode);

	// set up node
	node->generation = generation;
	node->population = new CPopulation;
	node->data = new CPopulationData;
	node->next = nullptr;
	node->prev = nullptr;

	// initialize each organism in the population
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		COrganism* pChild = new COrganism;
		OrganismParameters* pParam = new OrganismParameters;
		std::string childString;
		int crosspnt = crosspoint(parent1.length());
		
		// create new child and check for errors
		if (createChild(parent1, parent2, childString, crosspnt) < 0)
		{
			return -1;
		}

		// mutate the child string
		std::string mutatedChild = mutateString(childString, mutationRate);

		// set up new child to be added to population
		pChild->setBinaryString(mutatedChild);
		pChild->setParameters(pParam);
		if (pChild->ConvertStringToParameters() < 0)
		{
			return -1;
		}
	
		// add child to population
		if (node->population->addOrganism(i, pChild) < 0)
		{   // failed to add organism
			return -1;
		}
	}

	// add new generation to end of population list
	PopulationNode* current = mGenerationList;

	// TODO: add pointer to last node in GeneticAlgorithm class
	while (current != nullptr)
	{
		// check if current generation matches current
		if ((current->generation) == (node->generation))
		{   // not a unique generation id
			return -1;
		}

		// check if at end of list
		if (current->next == nullptr)
		{   // add new node to list
			current->next = node;
			current->prev = current;
			current = nullptr;  // set NULL to end loop
		}
		else
		{   // move to next node
			current = current->next;
		}
	}

	// newest generation added
	setGeneration(generation);
	return 0;
}


/**
* desc: create a brand new population using the current population
* param: mutationRate - rate int which mutation mill effect children. (e.g. 0.05 = 5%)
* pre-cond: fitness values for current generation must already be set.
*           parents from previous generation must already be set.
* post-cond: current generation will be incremented
* ret: 0 if successful
*     -1 if an error occured
*/
int CGeneticAlgorithm::addNewPopulation(double mutationRate)
{
	// check that generation list has been initialized
	if (mGenerationList == nullptr)
	{
		return -1;
	}

	// create new population node
	PopulationNode* pNode = new PopulationNode;

	// set up node
	pNode->generation = mCurrentGeneration + 1;
	pNode->population = new CPopulation;
	pNode->data = new CPopulationData;
	pNode->next = nullptr;
	pNode->prev = nullptr;

	// initialize each organism in the population
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		COrganism* pChild = new COrganism;
		OrganismParameters* pParam = new OrganismParameters;
		std::string childString;
		int crosspnt = crosspoint((mParent1->getBinaryString()).length());

		// create new child and check for errors
		if (createChild(mParent1->getBinaryString(), 
		                mParent2->getBinaryString(), 
		                childString, crosspnt) < 0)
		{
			return -1;
		}

		// mutate the child string
		std::string mutatedChild = mutateString(childString, mutationRate);

		// set up new child to be added to population
		pChild->setBinaryString(mutatedChild);
		pChild->setParameters(pParam);
		if (pChild->ConvertStringToParameters() < 0)
		{
			return -1;
		}

		// add child to population
		if (pNode->population->addOrganism(i, pChild) < 0)
		{   // failed to add organism
			return -1;
		}
	}

	// add new generation to end of population list
	PopulationNode* current = mGenerationList;

	// TODO: add tail pointer to last node in GeneticAlgorithm class
	while (current != nullptr)
	{
		// check if current generation matches current
		if ((current->generation) == (pNode->generation))
		{   // not a unique generation id
			return -1;
		}

		// check if at end of list
		if (current->next == nullptr)
		{   // add new node to list
			current->next = pNode;
			current->prev = current;
			current = nullptr;  // set NULL to end loop
		}
		else
		{   // move to next node
			current = current->next;
		}
	}

	// new generation added correctly, so increment
	incrementGeneration();
	return 0;
}


/**
* desc: delete all populations in list
*/
void CGeneticAlgorithm::deleteAllPopulations()
{
	// step through each population node and delete everything
	PopulationNode* current = mGenerationList;
	PopulationNode* previous = current;

	// delete each node in list if head is not nullptr
	while (current != nullptr)
	{
		current = current->next;  // hold next node
		previous->next = nullptr; // disconnect from next pointer
		previous->prev = nullptr; // disconnect from prev pointer
		previous->generation = 0; // set to safe value


		// check if population data is already empty
		if (previous->data != nullptr)
		{
			delete previous->data;
			(previous->data) = nullptr;
		}

		// check if population is already null
		if (previous->population != nullptr)
		{
			delete previous->population;
			previous->population = nullptr;
		}

		// delete the population node
		delete previous;

		// prepare for next node
		previous = current;
	}

	// make sure the head of the linked list is nullptr
	if (mGenerationList != nullptr)
	{
		mGenerationList = nullptr;
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
	PopulationNode* current;
	current = findPopulation(generation);

	// check that matching population was found
	if (current == nullptr)
	{
		return -1;
	}

	// display each organism in the population
	if (current->population->displayPopulation() < 0)
	{   // list is empty
		return -1;
	}

	return 0;
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
	PopulationNode* current;
	current = findPopulation(generation);
	
	// check that matching population was found
	if (current == nullptr)
	{
		return -1;
	}

#ifndef ALTERNATE_POPDATA_DISPLAY
	// display data for current population
	std::cout << "organisms: " << current->data->getNumOrganisms() << "t "
		      << current->data->getNumValidOrganisms() << "v" << std::endl;
	std::cout << "parents: " << current->data->getParent1Index() << " "
		      << current->data->getParent2Index() << std::endl;
	std::cout << "fitness: " << current->data->getLowestFitness() << " "
		                     << current->data->getAverageFitness() << " "
		                     << current->data->getHighestFitness() << std::endl;
	std::cout << "execution time: " << current->data->getLowestExecutionTime() << " "
		                            << current->data->getAverageExecutionTime() << " "
		                            << current->data->getHighestExecutionTime() << std::endl;
#endif
	
#ifdef ALTERNATE_POPDATA_DISPLAY
	// display data for current population in a single line
	std::cout << "org: " << current->data->getNumOrganisms() << "t "
		<< current->data->getNumValidOrganisms() << "v" << 
		 " p: " << current->data->getParent1Index() << " "
		<< current->data->getParent2Index() << " "
		<< " f: " << current->data->getLowestFitness() << "l "
		<< current->data->getAverageFitness() << "a "
		<< current->data->getHighestFitness() << "h "
		<< " exe: " << current->data->getLowestExecutionTime() << "l "
		<< current->data->getAverageExecutionTime() << "a "
		<< current->data->getHighestExecutionTime() << "h" << std::endl;
#endif

	return 0;
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


/**
* desc: find a population given its generation
* param: generation - generation to find
* ret: ptr to population if population found,
*      nullptr if population not found
*/
PopulationNode* CGeneticAlgorithm::findPopulation(int generation)
{
	// check if population list exists
	if (mGenerationList != nullptr)
	{
		// search through generations until match is found or list ends
		PopulationNode* current = mGenerationList;
		PopulationNode* previous = current;
		bool generationFound = false;
		while ((current != nullptr) && (generationFound != true))
		{
			current = current->next;  // hold next item in list
			if (previous->generation == generation)
			{   // match found so return ptr to population node
				return previous;
			}
			// prepare to check next generation
			previous = current;
		}
		// if loop exited it means match was not found
		return nullptr;
	}
	else
	{   // generation list is empty
		return nullptr;
	}
}


/**
* desc: fill in population data based on passed population
* param: population - population to extract data from
*        data - place to store population statistics
* pre-cond: execution time for all existing organisms must be set.
* ret: 0 if everything was successful,
*     -1 if an error occurred
*/
int CGeneticAlgorithm::updatePopulationData(CPopulation* population, CPopulationData* data)
{
	bool lowestFitnessSet = false;
	bool lowestExecutionTimeSet = false;
	int fitnessTally = 0;
	int executionTimeTally = 0;

	// check that neither population or data are empty
	if ((population == nullptr) || (data == nullptr))
	{   // passed empty parameter
		return -1;
	}

	// step through each organism and update stats as necessary
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		// check if organism exists
		if (population->getOrganism(i) != nullptr)
		{
			int currentOrganismFitness = population->getOrganismFitness(i);
			int currentOrganismExecutionTime = population->getOrganismExecutionTime(i);

			// check if execution time is set
			if (currentOrganismExecutionTime >= 0)
			{   // execution time not set so add to statistics
				data->setNumOrganisms(data->getNumOrganisms() + 1);  // increment organism counter

				// check if organism is valid
				if (population->getOrganismValid(i) == true)
				{   // increment valid counter
					data->setNumValidOrganisms(data->getNumValidOrganisms() + 1);
				}

				// check if organism should be a parent by comparing fitness values
				if (currentOrganismFitness > population->getOrganismFitness(data->getParent1Index()))
				{   // current organism has higher fitness than current parent 1
					// check if parent 1 should become parent 2
					if (currentOrganismFitness > population->getOrganismFitness(data->getParent2Index()))
					{   // parent 1 has higher fitness than parent 2
						// so, replace parent 2 with parent 1
						data->setParent2Index(data->getParent1Index());
					}

					// replace parent 1
					data->setParent1Index(i);
				}
				else if (currentOrganismFitness > population->getOrganismFitness(data->getParent2Index()))
				{   // current organism has higher fitness than parent 2 but less than parent 1
					data->setParent2Index(i);
				}
				
				// add executionTime and fitness to running tallies
				// check if these are the lowest or highest values thus far
				if (lowestFitnessSet == false)
				{
					data->setLowestFitness(static_cast<float>(currentOrganismFitness));
					lowestFitnessSet = true;
				}
				if (lowestExecutionTimeSet == false)
				{
					data->setLowestExecutionTime(currentOrganismExecutionTime);
					lowestExecutionTimeSet = true;
				}

				// check if curent fitness is smallest or largest thus far
				if (currentOrganismFitness < data->getLowestFitness())
				{
					data->setLowestFitness(static_cast<float>(currentOrganismFitness));
				}
				if (currentOrganismFitness > data->getHighestFitness())
				{
					data->setHighestFitness(static_cast<float>(currentOrganismFitness));
				}

				// check if current exTime is smallest or largest thus far
				if (currentOrganismExecutionTime < data->getLowestExecutionTime())
				{
					data->setLowestExecutionTime(currentOrganismExecutionTime);
				}
				if (currentOrganismExecutionTime > data->getHighestExecutionTime())
				{
					data->setHighestExecutionTime(currentOrganismExecutionTime);
				}

				// add fitness and exTime to running tallies
				fitnessTally += currentOrganismFitness;
				executionTimeTally += currentOrganismExecutionTime;
			}
		}
	}
	// use tallies to find average fitness and execution times
	// check that at least two organisms existed
	if (data->getNumOrganisms() > 1)
	{   // can divide now
		data->setAverageExecutionTime(static_cast<float>(executionTimeTally / (data->getNumOrganisms())));
		data->setAverageFitness(static_cast<float>(fitnessTally / (data->getNumOrganisms())));
		return 0;
	}
	else
	{   // need at least two organisms as parents
		return -1;
	}
}


/**
* desc: returns a random integer between 0 and stringLength
* param: stringLength - the number of characters in an organisms binary string
* pre-cond: must have stringLength >= 2
* ret: returns crosspoint on success
*      -1 if failed
*/
int CGeneticAlgorithm::crosspoint(unsigned int stringLength)
{
	// must have at least length 2
	if (stringLength < 2)
	{
		return -1;
	}

	// get random number between 0 and numBits
	int crossPoint = rand() % stringLength;

	// make sure crosspoint is at least 1, no parent clones
	if (crossPoint < 1)
	{
		crossPoint = 1;
	}

	return crossPoint;
}


/**
* desc: Takes left side of one parent and right side of other parent and
*       combines them into a new binary string of the same length.  The
*       left and right sides of the parents is desided by the crosspoint.
* param: parent1 - first parent of new child
*        parent2 - second parent of new child
*        child - string of new child
*        crosspoint - position to devide parent strings at
* pre-cond: parent1 and parent2 must be strings of the same length
* ret: 0 if successful, -1 if failed
*/
int CGeneticAlgorithm::createChild(std::string parent1, std::string parent2, 
	                               std::string &child, int crosspoint)
{
	std::string tempStr; // string to hold child data

	// check that parents are same length
	if (parent1.length() != parent2.length())
	{   // strings not same length
		return -1;
	}

	// get the left and right halves of both parents
	std::string parent1Left = parent1.substr(0, crosspoint);
	std::string parent1Right = parent1.substr(crosspoint);
	std::string parent2Left = parent2.substr(0, crosspoint);
	std::string parent2Right = parent2.substr(crosspoint);

	// random number used to choose parent combination
	int combination = rand() % 2;  // output 0 or 1

	// combine parent's two different ways to create a child
	switch (combination)
	{
	  case 0:  tempStr = parent1Left + parent2Right; 
		       break;
	  case 1:  tempStr = parent2Left + parent1Right;
		       break;
	  default: return -1;
	}
	child = tempStr;

	return 0;
}


/**
* desc: generates a random string of binary characters.
* param: stringLength - the number of characters in generated string
* ret: new string if successful,
*      empty string if stringLength is 0
*/
std::string CGeneticAlgorithm::rndBinaryString(unsigned int stringLength)
{
	int value = 0;         // stores value of random number
	std::string finalString;

	// generate random string
	for (unsigned int i = 0; i < stringLength; ++i)
	{
		value = rand() % 2;  // give 1 or 0
		if (value == 1)
		{
			finalString += '1';
		}
		else
		{
			finalString += '0';
		}
	}

	return finalString;
}


/**
* desc: ransomly flips binary characters in list based on mutation rate
* param: binaryString - string to be mutated
*        mutationRate - rate of mutation per character
* ret: newly mutated string
*/
std::string CGeneticAlgorithm::mutateString(std::string binaryString, double mutationRate)
{
	// TODO: test this function because I do not remember how it works
	int mutate = 0;  // used to flip bit

	for (unsigned int i = 0; i < binaryString.length(); ++i)
	{
		// find if a mutation occurs
		mutate = rand() % 100;

		// mutate
		if (mutate < mutationRate)
		{
			if (binaryString[i] == '1')
			{
				// flip bit
				binaryString[i] = '0';
			}
			else if (binaryString[i] == '0')
			{
				binaryString[i] = '1';
			}
			else  // should be binary string so condition not possible
			{
				return "";
			}
		}
	}

	// mutation complete
	return binaryString;
}