#pragma once
#include "Organism.h"
#include "Population.h"
#include "PopulationData.h"

//#define ALTERNATE_POPDATA_DISPLAY
// TODO: add getPopulationData function

//*****************************************************************************
// STRUCTURE: PopulationNode
// desc: This structure contains the node for a linear linked 
//       list of populations
//*****************************************************************************
struct PopulationNode
{
	CPopulation* population;
	CPopulationData* data;
	int generation;
	PopulationNode* next;
	PopulationNode* prev;
};


//*****************************************************************************
// CLASS: CGeneticAlgorithm
// desc: This class contains all methods needed for a genetic algorithm
//       to be used with pcl. The genetic algorithm uses genomes to manage
//       each organism in a population.
//*****************************************************************************
class CGeneticAlgorithm
{
public:
	CGeneticAlgorithm();
	~CGeneticAlgorithm();

	/**
	* desc: creates first generation of organisms each with a random binary
	*       string to determine the parameters.
	* pre-cond: class must not yet be initialized
	* post-cond: generation 1 created
	* ret: 0 on success,
	*     -1 if failed
	*/
	int init();

	/**
	* desc: compute the fitness value of all organisms in a population
	* param: generation - population to to find fitness falues for
	* post-cond: parent1 and parent2 are updated along with population data
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int computePopulationFitness(int generation);

	/**
	* desc: set first parent organism
	* param: parent1 - ptr to organism with highest fitness value
	* pre-cond: fitness values must already be generated for a population
	* ret: 0 if successful,
	*     -1 if an error occured
	*/
	int setParent1(COrganism* parent1);

	/**
	* desc: get ptr to first parent organism
	* ret: ptr to parent 1 if successful,
	*      nullptr if parent 1 does not yet exist
	*/
	COrganism* getParent1();

	/**
	* desc: set second parent organism
	* param: parent2 - ptr to organism with second highest fitness value
	* pre-cond: fitness values must already be generated for a population
	* ret: 0 if successful,
	*     -1 if an error occured
	*/
	int setParent2(COrganism* parent2);

	/**
	* desc: get ptr to second parent organism
	* ret: ptr to parent 2 if successful,
	*      nullptr if parent 2 does not yet exist
	*/
	COrganism* getParent2();

	/**
	* desc: set the current organism in current population
	* param: num - value to set current organism as
	*/
	void setCurrentOrganism(int num);

	/**
	* desc: get the value of current organism
	* ret: value of current organism
	*/
	int getCurrentOrganism();

	/**
	* desc: get a pointer to an organism in the population
	* param: id - index of organism to fetch
	*        generation - generation to get organism from
	* ret: returns pointer to organism if it exists,
	*      else NULL is returned
	*/
	COrganism* getOrganism(int id, int generation);

	/**
	* desc: get a pointer to a population
	* param: generation - generation to get population from
	* ret: returns pointer to population if it exists,
	*      else NULL is returned
	*/
	CPopulation* getPopulation(int generation);

	/**
	* desc: increment the value of the current organism variable
	* ret: value of current organism after being incremented
	*/
	int incrementCurrentOrganism();

	/**
	* desc: set the current generation
	* param: num - value to set generation as
	*/
	void setGeneration(int num);

	/**
	* desc: get the value of current generation
	* ret: value of current generation
	*/
	int getGeneration();

	/**
	* desc: increment the value of the current generation variable
	* ret: value of current generation after being incremented
	*/
	int incrementGeneration();

	/**
	* desc: return index of the two organisms with the highest fitness values
	* param: parent1 - ptr to organism with the highest fitness value
	*        parent2 - ptr to organism with the second highest fitness value
	* pre-cond: fitness values for current generation must already be set
	* post-cond: index of both organisms are passed back through function arguments
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int getParents(COrganism* parent1, COrganism* parent2);

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
	int addNewPopulation(std::string parent1, std::string parent2,
		                 int generation, double mutationRate);

	/**
	* desc: create a brand new population using the current population
	* param: mutationRate - rate int which mutation mill effect children. (e.g. 0.05 = 5%)
	* pre-cond: fitness values for current generation must already be set.
	*           parents from previous generation must already be set.
	* post-cond: current generation will be incremented
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int addNewPopulation(double mutationRate);
	
	/**
	* desc: delete all generations in list
	*/
	void deleteAllPopulations();

	/**
	* desc: display all data for current population
	* param: generation to display
	* ret: 0 if successful
	*     -1 if current generation does not exist
	*/
	int displayPopulation(int generation);

	/**
	* desc: save current population to a file
	* param: filename - name of file to save data to
	*        generation - generation to save
	* recommendation: have fitness values already set
	* post-cond: data appended to file if it already exists
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int savePopulation(int generation, std::string filename);

	/**
	* desc: display statistics for population as specified by the generation
	* param: generation - generation to save
	* post-cond: data displayed to standard out
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int displayPopulationData(int generation);

	/**
	* desc: save population statistics to a file
	* param: filename - name of file to save data to
	*        generation - generation to save
	* recommendation: have fitness values already set
	* post-cond: data appended to file if it already exists
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int savePopulationData(int generation, std::string filename);

private:
	COrganism* mParent1;
	COrganism* mParent2;
	PopulationNode* mGenerationList;
	int mCurrentOrganism;
	int mCurrentGeneration;

	/**
	* desc: find a population given its generation
	* param: generation - generation to find
	* ret: ptr to population if population found,
	*      nullptr if population not found
	*/
	PopulationNode* findPopulation(int generation);

	/**
	* desc: fill in population data based on passed population
	* param: population - population to extract data from
	*        data - place to store population statistics
	* pre-cond: execution time for all existing organisms must be set.
	*           need at least two existing organisms to set as parents
	* ret: 0 if everything was successful,
	*     -1 if an error occurred
	*/
	int updatePopulationData(CPopulation* population, CPopulationData* data);

	/**
	* desc: returns a random integer between 0 and stringLength
	* param: stringLength - the number of characters in an organisms binary string
	* ret: position of crosspoint if everything was successful,
	*     -1 if 0 is entered as stringLength
	*/
	int crosspoint(unsigned int stringLength);

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
	int createChild(std::string parent1, std::string parent2,
		            std::string &child, int crosspoint);

	/**
	* desc: generates a random string of binary characters.
	* param: stringLength - the number of characters in generated string
	* ret: new string if successful,
	*      empty string if stringLength is 0
	*/
	std::string rndBinaryString(unsigned int stringLength);
	
	/**
	* desc: ransomly flips binary characters in list based on mutation rate
	* param: binaryString - string to be mutated
	*        mutationRate - rate of mutation per character
	* ret: newly mutated string
	*/
	std::string mutateString(std::string binaryString, double mutationRate);
};

