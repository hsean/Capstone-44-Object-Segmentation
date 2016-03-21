#pragma once
#include "Organism.h"
#define LIST_SIZE 10

class CPopulation
{
public:
	CPopulation();
	~CPopulation();

	/**
	* desc: add organism to population
	* param: id - index of organism 
	*        organism - ptr to new organism to add
	* pre-cond: organism must already be initialized
	* post-cond: organism added to population. 
	* ret: 0 if organism added sucessfully,
	*     -1 if an organism already exists at index id
	*/
	int addOrganism(int id, COrganism* organism);

	/**
	* desc: remove organism from population
	* param: id - index of organism
	* post-cond: organism removed from population
	* ret: 0 if successful,
	*     -1 if index is out of array bounds
	*/
	int removeOrganism(int id);

	/**
	* desc: get a pointer to a specific organism
	* param: id - index of organism
	* pre-cond: organism must exist
	* ret: nullptr if organism does not exist, else
	*      a ptr to the organism is returned.
	*/
	COrganism* getOrganism(int id);

	/**
	* desc: display organisms in population. This includes
	*       valid, execution time, and parameters of population
	* param: id - index of organism to be displayed
	* post-cond: organism data displayed to standard out
	* ret: 0 if organism was found,
	*     -1 if organism was not found
	*/
	int displayOrganism(int id);

	/**
	* desc: save organism to a file. This includes valid,
	*       execution time, and parameters of organism
	* param: id - index of organism to be saved
	*        filename - name of file to append data too
	* post-cond: data appended to file
	* ret: 0 if list is not empty, and
	*     -1 if list is empty
	*/
	int saveOrganism(int id, std::string filename);

	/**
	* desc: set the fitness of an organism
	* param: id - index of organism
	*        fitness - fitness to set organism too
	* ret: 0 if successful,
	*     -1 if index is out of array bounds
	*/
	int setOrganismFitness(int id, int fitness);

	/**
	* desc: get the fitness value of an organism
	* param: id - index of organism
	* ret: returns fitness of organism
	*/
	int getOrganismFitness(int id);
	

	/**
	* desc: add/replace parameters of specified organism
	* param: id - index of organism
	*        parameters - ptr to parameter to add to organism
	* pre-cond: organism must already be initialized.
	* post-cond: parameters added to specified organism
	* ret: 0 on success,
	*     -1 if an error occured
	*/
	int setOrganismParameters(int id, OrganismParameters* parameters);

	/**
	* desc: get parameters of specified organism
	* param: id - index of organism
	* pre-cond: organism must have initialized parameters
	* post-cond: parameters added to specified organism
	* ret: nullptr if no parameters, else
	*      ptr to parameters is returned
	*/
	OrganismParameters* getOrganismParameters(int id);

	/**
	* desc: set if organism is valid
	* param: id - index of organism
	*        valid - state of valid flag
	* pre-cond: organism must exist
	* post-cond: organism valid flag is set
	* ret: 0 if successful
	*     -1 if id is out of array bounds
	*/
	int setOrganismValid(int id, bool valid);

	/**
	* desc: get if organism is valid
	* param: id - index of organism
	* pre-cond: organism must exist
	* ret: value of valid flag or false if 
	*      out of bounds
	*/
	bool getOrganismValid(int id);

	/**
	* desc: set organism execution time
	* param: id - index of organism
	*        executionTime - time needed for organism to execute
	* pre-cond: organism must exist
	* ret: 0 if successful
	*     -1 if id is out of array bounds
	*/
	int setOrganismExecutionTime(int id, int executionTime);

	/**
	* desc: get organism execution time
	* param: id - index of organism
	* pre-cond: organism must exist
	* ret: execution time in milliseconds. result of -1
	*      means organism has not had its execution time
	*      set yet or organism does not exist.
	*/
	int getOrganismExecutionTime(int id);

	/**
	* desc: calculate the fitness value of each organism in the population.
	*       fitness is based upon the validity and execution time of an 
	*       organism.
	* post-cond: fitness values are calculated and added to fitness list
	* ret: 0 if successful, else
	*     -1 if an error occured
	*/
	int calculatePopulationFitness();

	/**
	* desc: display organisms in population. This includes
	*       valid, execution time, and parameters of population
	* param: organism - ptr to organism to be removed
	* post-cond: organism removed from population
	* ret: -1 if list is empty, 0 if list is not empty
	*/
	int displayPopulation();

	/**
	* desc: save population to a file. This includes valid,
	*       execution time, and parameters of organism
	* param: filename - name of file to append data too
	* post-cond: data appended to file
	* ret: 0 if list is not empty, and
	*     -1 if list is empty or an error occured
	*/
	int savePopulation(std::string filename);

private:
	bool mValid[LIST_SIZE];              // valid flags
	int mExecutionTime[LIST_SIZE];       // execution times
	int mFitnessList[LIST_SIZE];         // fitness values
	COrganism* mOrganismList[LIST_SIZE]; // organisms in population
};

