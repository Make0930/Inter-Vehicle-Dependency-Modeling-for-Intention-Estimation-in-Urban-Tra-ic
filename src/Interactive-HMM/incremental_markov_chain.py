from hidden_markov_chain import HiddenMarkovChain
import numpy as np
import math
import copy


class IncrementalHiddenMarkovChainFP(HiddenMarkovChain):

    def __init__(self, T, E, pi, useLogarithm=True):
        """ The constructor for this class

        :param T: State Transition matrix
        :param E: Emission Probability matrix
        :param pi: initialization vector of state probabilities
        :param useLogarithm: should alphas be stored as their logarithms? Default = True
        """
        HiddenMarkovChain.__init__(self, T, E, pi)
        self.alphas = np.zeros((0, len(pi)))
        self.usingLogarithm = useLogarithm
        self.currentProbabilities = copy.deepcopy(pi)
        self.currentObservationProbability = 0

    def incScore(self, observation) -> float:
        """ Get the score of the current observation given the previous state of the HMM

        :param observation: An observation which is possible according to the emission matrix E
        :return: the score = Sum of the last Alphas for the observation in the current state of the HMM
        """
        if self.usingLogarithm:
            if len(self.alphas) == 0:
                self.alphas = np.array([math.log(x) for x in (self.pi.values * self.E[observation].T)[0]]).reshape(
                    1, -1)
                sumOfAlphas = sum([math.exp(x) for x in self.alphas[-1]])
                self.currentProbabilities.values = np.array(
                    [math.exp(x) / sumOfAlphas for x in self.alphas[-1]]).reshape(
                    1, -1)
                self.currentObservationProbability = math.log(sumOfAlphas)
            else:
                self.currentProbabilities.values = np.dot(self.currentProbabilities.values, self.T.values)
                self.currentProbabilities.values = self.currentProbabilities.values * self.E[observation].T

                newalphas = np.zeros(((len(self.alphas) + 1), len(self.currentProbabilities)))
                newalphas[:-1, :] = self.alphas
                newalphas[-1, :] = [math.log(x) + self.currentObservationProbability for x in
                                    self.currentProbabilities.values[0]]
                self.alphas = newalphas
                f_new = np.sum(self.currentProbabilities.values)
                self.currentProbabilities.values = self.currentProbabilities.values / f_new
                self.currentObservationProbability = self.currentObservationProbability + math.log(f_new)
            return float(np.sum([math.exp(x) for x in self.alphas[-1]]))
        else:
            if len(self.alphas) == 0:
                self.alphas = self.pi.values * self.E[observation].T
            else:
                newalphas = np.zeros(((len(self.alphas) + 1), len(self.alphas[0])))
                newalphas[:-1, :] = self.alphas
                newalphas[-1, :] = (newalphas[-2, :].reshape(1, -1) @ self.T.values) * self.E[observation].T
                self.alphas = newalphas
        sumOfAlphas = sum(self.alphas[-1])
        #normalization
        self.currentProbabilities.values = np.array([float(x) / sumOfAlphas for x in self.alphas[-1]])
        return float(self.alphas[-1].sum())
