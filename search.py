
import os
import numpy as np
import constants as c
import sys

from hillclimber import HILL_CLIMBER
from parallelHillClimber import PARALLEL_HILL_CLIMBER

import matplotlib.pyplot as plt

if __name__ == '__main__':

    for i in range(1, 2):
        #np.random.seed(i)
        phc = PARALLEL_HILL_CLIMBER()
        phc.Evolve()
        phc.Show_Best()

        plt.plot([j + 1 for j in range(c.numberOfGenerations)], phc.best_fitnesses, label="Random Seed: {}".format(i))


    plt.title("Peak Fitness at Each Generation")
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.legend()
    plt.grid()

    plt.savefig("best_fitnesses.jpg")