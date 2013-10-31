#ifndef H_GENALG
#define H_GENALG

#include <vector>

struct Genome
{
	std::vector<float> m_genome;
	float m_fitness;
	bool operator <(const Genome &lhs) {return m_fitness < lhs.m_fitness; }
};

class GenAlg
{
public:
	GenAlg(int populationSize, int genomeLength);
	
	void newGeneration();

	unsigned getPopulationSize(); // Returns the size of the population
	unsigned getGenomeLength(); // Returns the length of the genomes
	float getMutationRate();
	float getCrossoverRate();
	float getSurvivalRate();
	std::vector<float> getGenome(int index);
	float getFitness(int index);

	void setFitness(unsigned index, float fitness); // Sets the fitness of a gene.
	void setMutationRate(float rate);
	void setCrossoverRate(float rate);
	void setSurvivalRate(float rate);

private:
	float m_mutationRate;
	float m_crossoverRate;
	float m_survivalRate;
	unsigned m_genomeLength;
	unsigned m_populationSize;
	std::vector<Genome> m_population;
};

#endif // H_GENALG