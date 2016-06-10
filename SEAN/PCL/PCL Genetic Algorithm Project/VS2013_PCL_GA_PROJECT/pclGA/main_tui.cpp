/******************************************************************************
* Author: Sean Hendrickson
* File: main_tui.cpp
* Desc: this file contains a Text User Interface (TUI) that allows a user to 
*       interact with the PCL Genetic Algorithm
*******************************************************************************/
#include "GeneticAlgorithm.h"

#define MUTATION_RATE 0.05

using std::cout;
using std::cin;
using std::endl;


/* prototypes */
int mainMenu(CGeneticAlgorithm* pclGA);
int generationMenu(CGeneticAlgorithm* pclGA, int generation);
unsigned int getUserInput();


int main()
{
	CGeneticAlgorithm pclGA;  // genetic algorithm object
	// initialize GA and check for errors
	if (pclGA.init() < 0)
	{
		cout << "ERROR: Genetic Algorithm failed to initialize" << endl;
	}
	else
	{
		mainMenu(&pclGA);
	}
	return 0;
}


/* implementation*/
int mainMenu(CGeneticAlgorithm* pclGA)
{
	bool fExitMenu = false;  // exit menu flag
	bool fCurrentGenerationValidated = false;
	bool fSaved = false;
	unsigned int option = 0;  // used to get user response

	// main menu welcome text goes here
	cout << "THANK YOU FOR USING PCL GENETIC ALGORITHM" << endl;

	do
	{
		cout << "\n-------------------------------------" << endl
			 << "1. Generate Next Generation          " << endl
			 << "2. View Current Generation           " << endl
			 << "3. View a Generation                 " << endl
			 << "4. Display Available Generations     " << endl
			 << "5. Display Statistics                " << endl
			 << "6. Save                              " << endl
			 << "7. Exit Program                      " << endl
			 << "-------------------------------------" << endl;
		option = getUserInput();

		// check option
		switch (option)
		{
			case 1:  // Generate Next Generation
			{
				// create next generation if validated
				if (!fCurrentGenerationValidated)
				{   // note yet validated
					cout << "Cannot create new generation. Current generation not yet validated" << endl;
				}
				else
				{   // validated
					int fFailed = false;  // flag for errors

					// add new generation
					if (pclGA->addNewPopulation(MUTATION_RATE) < 0)
					{
						cout << "ERROR: failed to create new generation" << endl;
						fFailed = true;
					}

					// check for errors
					if (fFailed == false)
					{
						// reset flag for new generation
						fCurrentGenerationValidated = false;
					}
				}
				break;
			}
			case 2:  // View Current Generation Menu
			{
				generationMenu(pclGA, pclGA->getGeneration());
				break;
			}
			case 3:  // View a Generations' Menu
			{
				unsigned int generationChoice = 0;
				generationChoice = getUserInput();

				// check that the chosen generation exists
				if (pclGA->getPopulation(generationChoice) == nullptr)
				{
					cout << "Cannot open menu. Generation [" << generationChoice << "] could not be found" << endl;
				}
				else
				{   // generation exists, so open its menu
					generationMenu(pclGA, pclGA->getGeneration());
				}
				break;
			}
			case 4:  // Display Available Generations
			{
				cout << "[Generations Available as Options]: 0 - " << pclGA->getGeneration();	 
				break;
			}
			case 5:  // Display Statistics
			{
				// display stats for all generation
				for (int i = 0; i < pclGA->getGeneration(); ++i)
				{
					// display statistics for generation and check for error
					if (pclGA->displayPopulationData(pclGA->getGeneration()) < 0)
					{
						cout << "ERROR: failed to display generation " << pclGA->getGeneration() << endl;
					}
				}		 
				break;
			}
			case 6:  // Save
			{
				cout << "Save functionality not yet implemented" << endl;
				break;
			}
			case 7:  // Exit Program
			{
				fExitMenu = true;
				break;
			}
			default:
			{
				cout << "Option [" << option << "] not supported" << endl;
			}
		}
	} while (!fExitMenu);

	return 0;
}

/**
* desc: Asks a user to provide an integer and returns the passed integer
* ret: passed integer,
*     else -1 if some error occured
*/
unsigned int getUserInput()
{
	int value;  // user passed value

	// get user input
	cout << "Enter an integer: ";

	// get input until correct
	while (!(cin >> value))
	{   // incorrect input, clear errors and re-prompt user
		cin.clear();
		cin.ignore();
		cout << "Please enter a positive integer: ";
	}
	
	return value;
}


/**
* desc: This function allows a user to interact with contents of a single generation
*       of the PCL GA. This function also allows validation of each organisms output
*       which is required to create a new generation.
* param: pclGA - pointer to genetic algorithm object
* ret: -1 if an error occured, else
*       0 on success
*/
int generationMenu(CGeneticAlgorithm* pclGA, int generation)
{
	bool fExitMenu = false;  // exit menu flag
	unsigned int option = 0;  // used to get user response
	unsigned int organismChoice = 0;  // catch user input

	// generation menu welcome goes here
	cout << "GENERATION [" << generation << "] MENU" << endl;

	do
	{
		cout << "\n-------------------------------------" << endl
			<< "1. Start Validation                  " << endl
			<< "2. Invalidate an Organism            " << endl
			<< "3. Validate an Organism              " << endl
			<< "4. Display All Organisms             " << endl
			<< "5. View Organism Image               " << endl
			<< "6. Exit Generation Menu              " << endl
			<< "-------------------------------------" << endl;
		option = getUserInput();

		// implement menu options
		switch (option)
		{
		case 1: // Start Validation
		{
			// TODO: this will involve VTK visualization
			cout << "Validation Process not yet implemented" << endl;
			break;
		}
		case 2: // Invalidate an Organism
		{
			// choose an organism to invalidate
			organismChoice = getUserInput();
			if (pclGA->getPopulation(generation)->setOrganismValid(organismChoice, false) < 0)
			{   // organism could not be found
				cout << "Organism [" << organismChoice << "] could not be found" << endl;
			}
			// TODO: update fitness values
			break;
		}
		case 3: // Validate an Organism
		{
			// choose an organism to validate
			organismChoice = getUserInput();
			if (pclGA->getPopulation(generation)->setOrganismValid(organismChoice, true) < 0)
			{   // organism could not be found
				cout << "Organism [" << organismChoice << "] could not be found" << endl;
			}
			// TODO: update fitness values
			break;
		}
		case 4: // Display All Organisms
		{
			// TODO: uncomment after new function is added
			cout << "Function not yet implemented" << endl;
			//cout << "[Organisms Available as Options]: 0 - " << pclGA->getPopulationData()->getNumOrganisms();
			break;
		}
		case 5: // View Organism Image
		{
			// TODO: add PCD Visualization
			cout << "PCD image viewer not yet implemented" << endl;
			break;
		}
		case 6: // Exit Generation Menu
		{
			fExitMenu = true;
			break;
		}
		default:
		{
			cout << "Option [" << option << "] not supported" << endl;
		}
		}
	} while (fExitMenu == false);

	return 0;
}