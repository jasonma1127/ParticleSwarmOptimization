import numpy as np
import random as rand
import math
import numpy as np 
from matplotlib import pyplot as plt 

class ParticleSwarmOptimization:
    
    def __init__(self, x_Lb, x_Ub, y_Lb, y_Ub, n, velocityRange, w, c1, c2, iterMax, func, cons):
#         print("__init__")
        self.x_Lb = x_Lb
        self.x_Ub = x_Ub
        self.y_Lb = y_Lb
        self.y_Ub = y_Ub
        self.n = n
        self.velocityRange = velocityRange
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.iterMax = iterMax
        self.func = func
        self.cons = cons
        self.locationArray = np.ones((self.n,2))
        self.velocityArray = np.ones((self.n,2))
        self.fitnessArray = np.ones(self.n)
        self.pBestArray = np.ones(self.n) * -float("inf")
        self.pBestLocArray = np.ones((self.n,2))
        self.gBest = -float("inf")
        self.gBestLocArray = np.ones(2)
        self.optSolution = -float("inf")
        self.optSolutionArray = np.ones(2)
        
    def initialzation(self):
#         print("initialzation===============")
        for i in range(self.n):
            self.locationArray[i][0] = rand.uniform(self.x_Lb, self.x_Ub)
            self.locationArray[i][1] = rand.uniform(self.y_Lb, self.y_Ub)
            #constrain handling
            while(not self.cons(self.locationArray[i])):
                self.locationArray[i][0] = rand.uniform(self.x_Lb, self.x_Ub)
                self.locationArray[i][1] = rand.uniform(self.y_Lb, self.y_Ub)
        
        for i in range(self.n):
            self.velocityArray[i][0] = rand.uniform(-(self.velocityRange), self.velocityRange)
            self.velocityArray[i][1] = rand.uniform(-(self.velocityRange), self.velocityRange)
                
#         print("locationArray:\n", self.locationArray)
#         print("velocityArray:\n", self.velocityArray)
    
    def calculateFitness(self):
#         print("calculateFitness===============")
        for i in range(self.n):
            self.fitnessArray[i] = self.func(self.locationArray[i])
        
#         print("fitnessArray:\n", self.fitnessArray)
    
    def update_pBest_pBestLoc(self):
#         print("update_pBest_pBestLoc===============")
        for i in range(self.n):
            if(self.fitnessArray[i] > self.pBestArray[i]):
                self.pBestArray[i] = self.fitnessArray[i]
                self.pBestLocArray[i][0] = self.locationArray[i][0]
                self.pBestLocArray[i][1] = self.locationArray[i][1]
                
#         print("pBestArray:", self.pBestArray)
#         print("pBestLocArray:\n", self.pBestLocArray)
    
    def determine_gBest_gBestLoc(self):
        self.gBest = -float("inf")
#         print("determine_gBest_gBestLoc===============")
        for i in range(self.n):
            if(self.fitnessArray[i] > self.gBest):
                self.gBest = self.fitnessArray[i]
                self.gBestLocArray[0] = self.locationArray[i][0]
                self.gBestLocArray[1] = self.locationArray[i][1]
                
#         print("gBest:", self.gBest)
#         print("gBestLocArray:\n", self.gBestLocArray)

    def determineSolution(self):
        if(self.gBest > self.optSolution):
            self.optSolution = self.gBest
            self.optSolutionArray[0] = self.gBestLocArray[0]
            self.optSolutionArray[1] = self.gBestLocArray[1]
    
    def calculateVelocities(self):
#         print("calculateVelocities===============")
        for i in range(self.n):
            r1 = rand.random()
            r2 = rand.random()
            self.velocityArray[i] = (self.w * self.velocityArray[i]) + (r1*self.c1*(self.pBestLocArray[i]-self.locationArray[i])) + (r2*self.c2*(self.gBestLocArray-self.locationArray[i]))
        
#         print("velocityArray:\n", self.velocityArray)
            
    def damping(self):
#         print("damping===============")
        for i in range(self.n):
            for j in range(2):
                if(self.velocityArray[i][j] > self.velocityRange):
                    self.velocityArray[i][j] = self.velocityRange
                elif(self.velocityArray[i][j] < -self.velocityRange):
                    self.velocityArray[i][j] = -self.velocityRange
        
#         print("velocityArray:\n", self.velocityArray)
        
    def calculateLocations(self):
#         print("calculateLocations===============")
        for i in range(self.n):
            self.locationArray[i] = self.locationArray[i] + self.velocityArray[i]
            
#         print("locationArray:\n", self.locationArray)
        
    def repairLocations(self):
#         print("repairLocations===============")
        for i in range(self.n):
            if(self.locationArray[i][0] < self.x_Lb):
                self.locationArray[i][0] = self.x_Lb
            elif(self.locationArray[i][0] > self.x_Ub):
                self.locationArray[i][0] = self.x_Ub
            
            if(self.locationArray[i][1] < self.y_Lb):
                self.locationArray[i][1] = self.y_Lb
            elif(self.locationArray[i][1] > self.y_Ub):
                self.locationArray[i][1] = self.y_Ub
                
            #constrain handling
            while(not self.cons(self.locationArray[i])):
                self.locationArray[i][0] = rand.uniform(self.x_Lb, self.x_Ub)
                self.locationArray[i][1] = rand.uniform(self.y_Lb, self.y_Ub)
                
#         print("locationArray:\n", self.locationArray)
    
    def doRun(self):
        gbest_Y = []
        optSolution_Y = []
        self.initialzation()
        
        for gen in range(self.iterMax):
            print("Generation:", gen+1)
            self.calculateFitness()
            self.update_pBest_pBestLoc()
            self.determine_gBest_gBestLoc()
            print("gBest:", self.gBest)
            self.determineSolution()
            print("optSolution:", self.optSolution)
            self.calculateVelocities()
            self.damping()
            self.calculateLocations()
            self.repairLocations()
            gbest_Y.append(self.gBest)
            optSolution_Y.append(self.optSolution)
        
        x = np.arange(1, self.iterMax + 1)
        plt.figure(figsize=(15,5))
        plt.title("ParticleSwarmOptimization Demo") 
        plt.xlabel("Iter") 
        plt.ylabel("optSolution") 
        plt.plot(x, gbest_Y, color = 'red') 
        plt.plot(x, optSolution_Y, color = 'blue') 
        plt.show()
        print("===========ANS============")
        print("optSolution:", self.optSolution)
        print("optSolutionArray:\n", self.optSolutionArray)
