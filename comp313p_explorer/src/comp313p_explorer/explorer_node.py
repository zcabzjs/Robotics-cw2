import rospy
from math import pow,atan2,sqrt

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []
        self.frontierList = []
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    self.frontierList.append(candidate)
        
        self.current = (0, 0)

    def updateFrontiers(self):
        self.frontierList = []
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.isFrontierCell(x, y) is True:
                    coordinate = (x, y)
                    #IF IT EXPLODES!!!!
                    #UNCOMMENT THIS
                    #self.frontierList.append(coordinate)
                    # AND COMMENT THE IF BLOCK BELOW!!!
                    if coordinate not in self.blackList:
                        self.frontierList.append(coordinate)
        #print(self.frontierList)
        return (len(self.frontierList) != 0) 

    def chooseNewDestination(self):
        

#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

#        for x in range(0, self.occupancyGrid.getWidthInCells()):
#            for y in range(0, self.occupancyGrid.getHeightInCells()):
#                candidate = (x, y)
#                if self.isFrontierCell(x, y) is True:
#                    candidateGood = True
#                    for k in range(0, len(self.blackList)):
#                        if self.blackList[k] == candidate:
#                            candidateGood = False
#                            break

#                    if candidateGood is True:                       
#                        return True, candidate

#        return False, None

        # Going to do this based on shortest distance from current spot
        if(len(self.frontierList)) > 0:
            resultCoord = None
            minimum = 99999
            for k in range(0, len(self.frontierList)):
                diffX = abs(self.current[0] - self.frontierList[k][0])
                diffY = abs(self.current[1] - self.frontierList[k][1])
                distance = max(diffX, diffY) + (sqrt(2) - 1)*min(diffX, diffY)
                #distance = sqrt(pow((self.current[0] - self.frontierList[k][0]), 2) + pow((self.current[1] - self.frontierList[k][1]), 2))
                if distance < minimum:
                    resultCoord = self.frontierList[k]
                    minimum = distance
            
            if(resultCoord != None):
                return True, resultCoord
            return False, None
        
        return False, None



    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            if goal in self.frontierList:
                self.frontierList.remove(goal)
        else:
            self.current = goal
            print("Current coordinate: " + str(self.current))
            
