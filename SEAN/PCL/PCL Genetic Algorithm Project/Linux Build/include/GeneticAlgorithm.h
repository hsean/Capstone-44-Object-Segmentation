#pragma once
#include "Organism.h"
#include "Population.h"
#include "PopulationData.h"

class CGeneticAlgorithm
{
public:
	CGeneticAlgorithm();
	~CGeneticAlgorithm();

	/**
	* desc: compute the fitness value of all organisms in a population
	* param: generation - population to to find fitness falues for
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
	* param: parent1 - ptr to first organism to be used to generate new population
	*        parent2 - ptr to second organism to be used to generate new population
	* pre-cond: fitness values for current generation must already be set
	* post-cond: current generation will be incremented
	* ret: 0 if successful
	*     -1 if an error occured
	*/
	int generateNewPopulation(COrganism* parent1, COrganism* parent2);
	
	/**
	* desc: delete all populations in list
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
	PopulationNode* mPopulation;
	int mCurrentOrganism;
	int mCurrentGeneration;
};

