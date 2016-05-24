#pragma once


//*****************************************************************************
// STRUCTURE: CPopulationData
// desc: This class contains statistics for the population
//*****************************************************************************
class CPopulationData
{
public:
	CPopulationData();
	~CPopulationData();

	/**
	* desc: set the number of organisms in population
	* param: num - set number of organisms
	*/
	void setNumOrganisms(int num);

	/**
	* desc: get the number of organisms in population
	* ret: number of organisms in population
	*/
	int getNumOrganisms();

	/**
	* desc: set the number of valid organisms in population
	* param: num - number to set
	*/
	void setNumValidOrganisms(int num);

	/**
	* desc: get the number of valid organisms in population
	* ret: number of valid organisms in population
	*/
	int getNumValidOrganisms();

	/**
	* desc: set the index of the organism with the highest fitness
	* param: id - index of parent 1 in population
	*/
	void setParent1Index(unsigned int id);

	/**
	* desc: get the index of the organism with the highest fitness
	* ret: index of parent 1 if it has been set,
	*     -1 if parent 1 has not be set
	*/
	int getParent1Index();

	/**
	* desc: set the index of the organism with the second highest fitness
	* param: id - index of parent 2 in population
	*/
	void setParent2Index(unsigned int id);

	/**
	* desc: get the index of the organism with the second highest fitness
	* ret: index of parent 2 if it has been set,
	*     -1 if parent 2 has not be set
	*/
	int getParent2Index();

	/**
	* desc: set the lowest fitness value in population
	* param: num - lowest fitness value
	*/
	void setLowestFitness(float num);

	/**
	* desc: get the lowest fitness value in population
	* ret: lowest fitness value
	*/
	float getLowestFitness();

	/**
	* desc: set the highest fitness value in population
	* param: num - highest fitness value
	*/
	void setHighestFitness(float num);

	/**
	* desc: get the highest fitness value in population
	* ret: highest fitness value
	*/
	float getHighestFitness();

	/**
	* desc: set the average fitness value in population
	* param: num - average fitness value
	*/
	void setAverageFitness(float num);

	/**
	* desc: get the average fitness value in population
	* ret: average fitness value
	*/
	float getAverageFitness();

	/**
	* desc: set the lowest execution time in population
	* param: num - lowest execution time
	*/
	void setLowestExecutionTime(int num);

	/**
	* desc: get the lowest execution time in population
	* ret: lowest execution time
	*/
	int getLowestExecutionTime();

	/**
	* desc: set the highest execution time in population
	* param: num - highest execution time
	*/
	void setHighestExecutionTime(int num);

	/**
	* desc: get the highest execution time in population
	* ret: highest execution time
	*/
	int getHighestExecutionTime();

	/**
	* desc: set the average execution time in population
	* param: num - average execution time
	*/
	void setAverageExecutionTime(float num);

	/**
	* desc: get the average execution time in population
	* ret: average execution time
	*/
	float getAverageExecutionTime();

private:
	int mNumOrganisms;
	int mNumValidOrganisms;
	int mParent1Index;
	int mParent2Index;
	float mLowestFitness;
	float mHighestFitness;
	float mAverageFitness;
	int mLowestExecutionTime;
	int mHighestExecutionTime;
	float mAverageExecutionTime;
};

