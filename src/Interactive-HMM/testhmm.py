from incremental_markov_chain import IncrementalHiddenMarkovChainFP
from probability_matrix import ProbabilityMatrix
from probability_vector import ProbabilityVector
from itertools import product
from hidden_markov_chain_reference import HiddenMarkovChain
from hidden_markov_chain_reference import HiddenMarkovChain_FP


def main() :
    a1 = ProbabilityVector({'1H': 0.7, '2C': 0.3})
    a2 = ProbabilityVector({'1H': 0.4, '2C': 0.6})

    b1 = ProbabilityVector({'1S': 0.1, '2M': 0.4, '3L': 0.5})
    b2 = ProbabilityVector({'1S': 0.7, '2M': 0.2, '3L': 0.1})

    A = ProbabilityMatrix({'1H': a1, '2C': a2}) # state transition matrix
    B = ProbabilityMatrix({'1H': b1, '2C': b2}) # Emission Probability Matrix
    pi = ProbabilityVector({'1H': 0.6, '2C': 0.4}) # initial state

    hmcL = IncrementalHiddenMarkovChainFP(A, B, pi, useLogarithm=True)
    hmcL.incScore('1S')
    hmcL.incScore('2M')
    hmcL.incScore('3L')
    hmcL.incScore('2M')
    print ("Score for 1S when using Logarithms is {:f}.".format(hmcL.incScore('1S')))

    hmc = IncrementalHiddenMarkovChainFP(A, B, pi, useLogarithm=False)
    hmc.incScore('1S')
    hmc.incScore('2M')
    hmc.incScore('3L')
    hmc.incScore('2M')
    print("Score for 1S when not using Logarithms is {:f}.".format(hmc.incScore('1S')))

    observations = ['1S', '2M', '3L', '2M', '1S']
    nmc = HiddenMarkovChain(A, B, pi)
    print("Score for {} with reference implementation is {:f}.".format(observations, nmc.score(observations)))

    fpmc = HiddenMarkovChain_FP(A, B, pi)
    print("Score for {} with reference implementation with forwards pass is {:f}.".format(observations, fpmc.score(observations)))

    hmc2 = IncrementalHiddenMarkovChainFP(A, B, pi)
    all_possible_observations = {'1S', '2M', '3L'}
    chain_length = 3  # any int > 0
    all_observation_chains = list(product(*(all_possible_observations,) * chain_length))
    all_possible_scores = list(map(lambda obs: hmc2.score(obs), all_observation_chains))
    print("All possible scores added: {}.".format(sum(all_possible_scores)))


if __name__=="__main__":
    main()