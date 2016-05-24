#include "PopulationData.h"


CPopulationData::CPopulationData()
{
	mNumOrganisms = 0;
	mNumValidOrganisms = 0;
	mParent1Index = -1;
	mParent2Index = -1;
	mLowestFitness = 0;
	mHighestFitness = 0;
	mAverageFitness = 0;
	mLowestExecutionTime = 0;
	mHighestExecutionTime = 0;
	mAverageExecutionTime = 0;
}


CPopulationData::~CPopulationData()
{
	mNumOrganisms = 0;
	mNumValidOrganisms = 0;
	mParent1Index = 0;
	mParent2Index = 0;
	mLowestFitness = 0;
	mHighestFitness = 0;
	mAverageFitness = 0;
	mLowestExecutionTime = 0;
	mHighestExecutionTime = 0;
	mAverageExecutionTime = 0;
}


/**
* desc: set the number of organisms in population
* param: num - set number of organisms
*/
void CPopulationData::setNumOrganisms(int num)
{
	mNumOrganisms = num;
}


/**
* desc: get the number of organisms in population
* ret: number of organisms in population
*/
int CPopulationData::getNumOrganisms()
{
	return mNumOrganisms;
}


/**
* desc: set the number of valid organisms in population
* param: num - number to set
*/
void CPopulationData::setNumValidOrganisms(int num)
{
	mNumValidOrganisms = num;
}


/**
* desc: get the number of valid organisms in population
* ret: number of valid organisms in population
*/
int CPopulationData::getNumValidOrganisms()
{
	return mNumValidOrganisms;
}

/**
* desc: set the index of the organism with the highest fitness
* param: id - index of parent 1 in population
*/
void CPopulationData::setParent1Index(unsigned int id)
{
	mParent1Index = id;
}


/**
* desc: get the index of the organism with the highest fitness
* ret: index of parent 1 if it has been set,
*     -1 if parent 1 has not be set
*/
int CPopulationData::getParent1Index()
{
	return mParent1Index;
}


/**
* desc: set the index of the organism with the second highest fitness
* param: id - index of parent 2 in population
*/
void CPopulationData::setParent2Index(unsigned int id)
{
	mParent2Index = id;
}


/**
* desc: get the index of the organism with the second highest fitness
* ret: index of parent 2 if it has been set,
*     -1 if parent 2 has not be set
*/
int CPopulationData::getParent2Index()
{
	return mParent2Index;
}


/**
* desc: set the lowest fitness value in population
* param: num - lowest fitness value
*/
void CPopulationData::setLowestFitness(float num)
{
	mLowestFitness = num;
}


/**
* desc: get the lowest fitness value in population
* ret: lowest fitness value
*/
float CPopulationData::getLowestFitness()
{
	return mLowestFitness;
}


/**
* desc: set the highest fitness value in population
* param: num - highest fitness value
*/
void CPopulationData::setHighestFitness(float num)
{
	mHighestFitness = num;
}


/**
* desc: get the highest fitness value in population
* ret: highest fitness value
*/
float CPopulationData::getHighestFitness()
{
	return mHighestFitness;
}


/**
* desc: set the average fitness value in population
* param: num - average fitness value
*/
void CPopulationData::setAverageFitness(float num)
{
	mAverageFitness = num;
}


/**
* desc: get the average fitness value in population
* ret: average fitness value
*/
float CPopulationData::getAverageFitness()
{
	return mAverageFitness;
}


/**
* desc: set the lowest execution time in population
* param: num - lowest execution time
*/
void CPopulationData::setLowestExecutionTime(int num)
{
	mLowestExecutionTime = num;
}


/**
* desc: get the lowest execution time in population
* ret: lowest execution time
*/
int CPopulationData::getLowestExecutionTime()
{
	return mLowestExecutionTime;
}


/**
* desc: set the highest execution time in population
* param: num - highest execution time
*/
void CPopulationData::setHighestExecutionTime(int num)
{
	mHighestExecutionTime = num;
}


/**
* desc: get the highest execution time in population
* ret: highest execution time
*/
int CPopulationData::getHighestExecutionTime()
{
	return mHighestExecutionTime;
}


/**
* desc: set the average execution time in population
* param: num - average execution time
*/
void CPopulationData::setAverageExecutionTime(float num)
{
	mAverageExecutionTime = num;
}


/**
* desc: get the average execution time in population
* ret: average execution time
*/
float CPopulationData::getAverageExecutionTime()
{
	return mAverageExecutionTime;
}
