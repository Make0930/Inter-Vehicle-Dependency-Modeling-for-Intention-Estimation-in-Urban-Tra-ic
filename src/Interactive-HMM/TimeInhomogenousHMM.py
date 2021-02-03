import numpy as np
import pandas as pd
from probability_matrix import ProbabilityMatrix
from incremental_markov_chain import IncrementalHiddenMarkovChainFP

class TimeInhomogenousHMM :

    def __init__(self, markovChain):
        self.markovChain = markovChain

    @classmethod
    def initialize(cls, states : list, observables: list) :
        chain = IncrementalHiddenMarkovChainFP.initialize(states, observables)
        return cls(chain)
