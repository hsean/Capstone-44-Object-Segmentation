/******************************************************************************
 * author: Sean Hendrickson
 * created: 21 March 2016
 * desc: This file tests the function needed for a genetic algorithm.
 ******************************************************************************/
#include "Organism.h"
#include "Population.h"
#include "PopulationData.h"
#include "GeneticAlgorithm.h"
#include <ctime>

using std::cout;
using std::endl;
using std::cin;

//#define ORGANISM_TESTS
//#define POPULATION_TESTS
//#define POPULATION_DATA_TESTS
#define GA_TESTS

void organismTests();
void populationTests();
void popDataTests();
void gaTests();

int main()
{
	#ifdef ORGANISM_TESTS
	organismTests();
	#endif

	#ifdef POPULATION_TESTS
	populationTests();
	#endif

	#ifdef POPULATION_DATA_TESTS
	popDataTests();
	#endif

	#ifdef GA_TESTS
	gaTests();
	#endif

	cout << "Press any key to end program...";
	cin.get();

	return 0;
}

// tests functions in Organism.h
void organismTests()
{
	// TODO: change test once more parameters are added

	COrganism organism;
	OrganismParameters* pParameters = new OrganismParameters;
	OrganismParameters* holdReturn = nullptr;
	std::string binaryString;
	int holdValue = 0;
	
	cout << "BEGIN ORGANISM TESTS" << endl;

	// initialize leaf size to 1 (1cm)
	pParameters->leafSize = 1;
	
	// set and check parameters
	if (organism.setParameters(pParameters) < 0)
	{
		cout << "ERROR: organism parameters not passed correctly" << endl;
		return;
	}

	// set and check binary string
	organism.ConvertParametersToString();
	binaryString = organism.getBinaryString();
	if (binaryString.empty())
	{
		cout << "ERROR: parameters not converted to binary string" << endl;
		return;
	}
	organism.displayOrganism();
	
	// change string and set parameters
	binaryString = "00000100";  // set leafsize to 4cm
	organism.setBinaryString(binaryString);
	cout << "binary string set to: "
		 << (binaryString = organism.getBinaryString()) 
		 << endl;
	int result = std::stoi(binaryString, nullptr, 2);
	cout << "After conversion, leafsize = " << result << endl;
	if (result != 4)
	{
		cout << "ERROR: failed to set new string" << endl;
		return;
	}
	organism.ConvertStringToParameters();
	holdValue = pParameters->leafSize;
	if (holdValue != 4)
	{
		cout << "ERROR: string to parameters conversion failed" << endl;
		return;
	}

	// test saving organism
	// TODO: implement this feature at a later point
	if (organism.saveOrganism("log_organism.txt") < 0)
	{
		cout << "ERROR: failed to save organism" << endl;
		return;
	}
}


// tests functions in Population.h
void populationTests()
{
	CPopulation* pPopulation (new CPopulation);
	int testExecutionTime = 50;  // 400 ms

	cout << "BEGIN POPULATION TESTS" << endl;
	// add organisms to population
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		//TODO: add pOrganism->init(paramlist/pParamlist)  // to reduce clutter  // two overloaded functions
		// set up organism
		COrganism* pOrganism = new COrganism;
		OrganismParameters* pParameters = new OrganismParameters;
		pParameters->leafSize = 1;  // 1 cm
		pOrganism->setParameters(pParameters);
		pOrganism->ConvertParametersToString();

		// add the now set up organism to population
		if (pPopulation->addOrganism(i, pOrganism) < 0)
		{
			cout << "ERROR: organism already exists at index" << i << endl;
			return;
		}

		// add execution time to organism
		pPopulation->setOrganismExecutionTime(i, testExecutionTime);
	}
	cout << "POPULATION TESTS: CALCULATE FITNESS AND DISPLAY" << endl;
	// find fitness values of population
	pPopulation->calculatePopulationFitness();
	pPopulation->displayPopulation();  // fitness should not be above 500

	// set first 5 organisms to valid
	for (int i = 0; i < 5; ++i)
	{
		if (pPopulation->setOrganismValid(i, true) < 0)
		{
			cout << "ERROR: index out of bounds" << endl;
			return;
		}
	}

	// check that fitness values changed
	if (pPopulation->calculatePopulationFitness() < 0)
	{   // update fitness of population
		cout << "ERROR: failed to update fitness values" << endl;
		return;
	}

	cout << "POPULATION TESTS: SET VALID AND DISPLAY" << endl;
	if (pPopulation->displayPopulation() < 0)
	{   // display to check for updated fitness values
		cout << "ERROR: setting valid flag failed" << endl;
		return;
	}

	// remove 3rd organism in population
	if (pPopulation->removeOrganism(2) < 0)
	{
		cout << "ERROR: index out of bounds" << endl;
		return;
	}

	// check that third organism was removed
	if (pPopulation->getOrganism(2) != nullptr)
	{
		cout << "ERROR: failed to remove organism" << endl;
		return;
	}

	// display organism 4
	cout << "POPULATION TESTS: DISPLAY 4TH ORGANISM" << endl;
	if (pPopulation->displayOrganism(3) < 0)
	{
		cout << "ERROR: failed to display organism" << endl;
		return;
	}

	// save organism 4
	if (pPopulation->saveOrganism(3, "organism.txt") < 0)
	{
		cout << "ERROR: failed to save organism" << endl;
		return;
	}

	cout << "POPULATION TESTS: TESTING GET FUNCTIONS" << endl;
	// check that remaining get functions work
	int index = 4;
	cout << "org:" << index 
		 << " v:" << pPopulation->getOrganismValid(index)
		 << " t:" << pPopulation->getOrganismExecutionTime(index)
		 << " f:" << pPopulation->getOrganismFitness(index) << endl;

	// save population
	if (pPopulation->savePopulation("population.txt") < 0)
	{
		cout << "ERROR: failed to save population" << endl;
		return;
	}
}


// tests functions in PopulationData.h
void popDataTests()
{
	CPopulationData data;
	cout << "BEGIN POPULATION DATA TESTS" << endl;
	// set and check each argument in CPopulation data
	data.setNumOrganisms(10);
	if (data.getNumOrganisms() != 10)
	{
		cout << "ERROR: failed to set number of organisms" << endl;
		return;
	}

	data.setNumValidOrganisms(11);
	if (data.getNumValidOrganisms() != 11)
	{
		cout << "ERROR: failed to set number of valid organisms" << endl;
		return;
	}

	data.setParent1Index(2);
	if (data.getParent1Index() != 2)
	{
		cout << "ERROR: failed to get parent 1 index" << endl;
		return;
	}

	data.setParent2Index(3);
	if (data.getParent2Index() != 3)
	{
		cout << "ERROR: failed to get parent 2 index" << endl;
		return;
	}

	data.setLowestFitness(100);
	if (data.getLowestFitness() != 100)
	{
		cout << "ERROR: failed to get lowest fitness" << endl;
		return;
	}

	data.setHighestFitness(300);
	if (data.getHighestFitness() != 300)
	{
		cout << "ERROR: failed to get highest fitness" << endl;
		return;
	}

	data.setAverageFitness(200);
	if (data.getAverageFitness() != 200)
	{
		cout << "ERROR: failed to get average fitness" << endl;
		return;
	}

	data.setLowestExecutionTime(30);
	if (data.getLowestExecutionTime() != 30)
	{
		cout << "ERROR: failed to get lowest execution time" << endl;
		return;
	}

	data.setHighestExecutionTime(500);
	if (data.getHighestExecutionTime() != 500)
	{
		cout << "ERROR: failed to get highest execution time" << endl;
		return;
	}

	data.setAverageExecutionTime(50);
	if (data.getAverageExecutionTime() != 50)
	{
		cout << "ERROR: failed to get average execution time" << endl;
		return;
	}
}


// tests functions in GeneticAlgorithm.h
void gaTests()
{
	cout << "BEGIN GENETIC ALGORITHM TESTS" << endl;
	CGeneticAlgorithm myGA;
	CPopulation* pPopulation;
	//int generation = 0;
	int rndNumber = 0;
	double mutationRate = 0.05;

	/* initialize random seed: */
	srand(time(NULL));
	
	/* ADD INITIAL POPULATION */
	cout << "POPULATION 0" << endl;

	// initialization creates first population with random organisms
	myGA.init();  // initialize GA
	
	// get the first population at index 0
	if ((pPopulation = myGA.getPopulation(myGA.getGeneration())) == nullptr)
	{
		cout << "ERROR: could not get population" << endl;
		return;
	}

	// set random execution time for all organisms
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		rndNumber = rand() % 1000 + 1;  // time between 1 and 1000
		if ((pPopulation->setOrganismExecutionTime(i, rndNumber)) < 0)
		{
			cout << "ERROR: could not set execution time" << endl;
			return;
		}
	}

	// set fitness values of population
	if (myGA.computePopulationFitness(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not calculate fitness values" << endl;
		return;
	}

	// display population and data
	if (myGA.displayPopulation(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << endl;
		return;
	}

	// display population data
	if (myGA.displayPopulationData(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << "'s data" << endl;
		return;
	}

	/* ADDING SECOND POPULATION */
	cout << "POPULATION 1" << endl;

	// create a new population using highest fitness parents
	if (myGA.addNewPopulation(myGA.getParent1()->getBinaryString(),
		                      myGA.getParent2()->getBinaryString(),
		                      myGA.getGeneration() + 1,
		                      mutationRate) < 0)
	{
		cout << "ERROR: failed to add generation 1" << endl;
		return;
	}

	// get the second population at index 1
	if ((pPopulation = myGA.getPopulation(myGA.getGeneration())) == nullptr)
	{
		cout << "ERROR: could not get population" << endl;
		return;
	}

	// set random execution time for all organisms
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		rndNumber = rand() % 1000 + 1;  // time between 1 and 1000
		if ((pPopulation->setOrganismExecutionTime(i, rndNumber)) < 0)
		{
			cout << "ERROR: could not set execution time" << endl;
			return;
		}
	}

	// set fitness values of population
	if (myGA.computePopulationFitness(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not calculate fitness values" << endl;
		return;
	}

	// display population and data
	if (myGA.displayPopulation(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << endl;
		return;
	}

	// display population data
	if (myGA.displayPopulationData(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << "'s data" << endl;
		return;
	}

	/* ADDING THIRD POPULATION */
	cout << "POPULATION 2" << endl;

	// create a new population using highest fitness parents
	if (myGA.addNewPopulation(mutationRate) < 0)
	{
		cout << "ERROR: failed to add generation 2" << endl;
		return;
	}

	// get the third population at index 2
	if ((pPopulation = myGA.getPopulation(myGA.getGeneration())) == nullptr)
	{
		cout << "ERROR: could not get population" << endl;
		return;
	}

	// set random execution time for all organisms
	for (int i = 0; i < LIST_SIZE; ++i)
	{
		rndNumber = rand() % 1000 + 1;  // time between 1 and 1000
		if ((pPopulation->setOrganismExecutionTime(i, rndNumber)) < 0)
		{
			cout << "ERROR: could not set execution time" << endl;
			return;
		}
	}

	// set fitness values of population
	if (myGA.computePopulationFitness(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not calculate fitness values" << endl;
		return;
	}

	// display population and data
	if (myGA.displayPopulation(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << endl;
		return;
	}

	// display population data
	if (myGA.displayPopulationData(myGA.getGeneration()) < 0)
	{
		cout << "ERROR: could not display generation " << myGA.getGeneration() << "'s data" << endl;
		return;
	}

	/* END OF ADDING POPULATIONS */

	// TODO: implement save function at some later point
	// test save function
	if (myGA.savePopulation(myGA.getGeneration(), "foo.txt") < 0)
	{
		cout << "ERROR: failed to save population" << endl;
		return;
	}

	if (myGA.savePopulationData(myGA.getGeneration(), "foo.txt") < 0)
	{
		cout << "ERROR: failed to save population data" << endl;
		return;
	}
}



