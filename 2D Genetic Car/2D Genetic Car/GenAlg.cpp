#include "genAlg.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>

GenAlg::GenAlg(int populationSize, int genomeLength)
{
	m_mutationRate = 0;
	m_crossoverRate = 0;
	m_survivalRate = 1.0;
	m_genomeLength = genomeLength;
	m_populationSize = populationSize;

	m_population.resize(m_populationSize);

	for(int i = 0; i < m_populationSize; i++)
	{
		m_population[i].m_genome.resize(m_genomeLength);
		for(int j = 0; j < m_genomeLength; j++)
		{
			m_population[i].m_genome[j] = rand() / (float)RAND_MAX; 
		}

		m_population[i].m_fitness = 0;
	}
}

void GenAlg::newGeneration()
{
	int numSurvivors = (int)(m_populationSize * m_survivalRate);

	if(numSurvivors < 2)
	{
		std::cout << "ERROR: Too few survivors, increase survival rate." << std::endl;
	}
	else
	{
		std::sort(m_population.begin(), m_population.end());
		std::vector<Genome> survivors(m_population.end() - numSurvivors, m_population.end());
	
		for(int i = 0; i < survivors.size(); i++)
		{
			std::cout << "SURVIVOR " << i << " " << survivors[i].m_fitness << std::endl;
		}

		Genome *curParent, *otherParent;

		for ( int i = 0; i < m_populationSize; i++ )
		{
			m_population[i].m_fitness = 0;

			int tempRandIndex = rand() % survivors.size();
			curParent = &survivors[tempRandIndex];
			// Select a parent that isn't parent 1.
			otherParent = &survivors[(tempRandIndex + rand() % (survivors.size()-1)) % survivors.size()]; 

			// Form the child's genes.
			for ( int j = 0; j < m_genomeLength; j++ )
			{
				// Randomly switch the parent we're currently taking genes from 
				if ( rand() / (float)RAND_MAX <= m_crossoverRate )
				{
					Genome *temp = curParent;
					curParent = otherParent;
					otherParent = temp;
				}

				// Randomly mutate gene
				if ( rand() / (float)RAND_MAX <= m_mutationRate )
				{
					// Mutation
					m_population[i].m_genome[j] = (rand() / (float)RAND_MAX);
				}
				else
				{
					// No Mutation
					m_population[i].m_genome[j] = (curParent->m_genome[j]);
				}
			}
		}
	}
}

unsigned GenAlg::getPopulationSize() { return m_populationSize; }

unsigned GenAlg::getGenomeLength() { return m_genomeLength; }

float GenAlg::getMutationRate() { return m_mutationRate; }

float GenAlg::getCrossoverRate() { return m_crossoverRate; }

float GenAlg::getSurvivalRate() { return m_survivalRate; }

std::vector<float> GenAlg::getGenome(int index) 
{
	if(index <= m_populationSize)
	{
		return m_population[index].m_genome;
	}
	else
	{
		std::vector<float> empty;
		return empty;
	}
}

float GenAlg::getFitness(int index)
{
	if(index <= m_populationSize)
	{
		return m_population[index].m_fitness;
	}
	else
	{
		return 0.0f;
	}
}

void GenAlg::setFitness(unsigned index, float fitness)
{
	if(index <= m_populationSize)
	{
		m_population[index].m_fitness = fitness;
	}
}

void GenAlg::setMutationRate(float rate) { m_mutationRate = rate; }

void GenAlg::setCrossoverRate(float rate)  { m_crossoverRate = rate; }

void GenAlg::setSurvivalRate(float rate) { m_survivalRate = rate; }
