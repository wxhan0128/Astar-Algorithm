'''
Created on 9/6/2016

@author: Wenxuan Han
@contact: wenxhan@umail.iu.edu

Python version:3.5.2
'''

import math
import yaml

class CityNode:
    def __init__(self, no, cX, cY):
        self.no = no
        self.x = cX
        self.y = cY
        self.hdis = 0
        self.gdis = 0
        self.parent = self
        self.windaffect = False
    
    def getCityNo(self):
        return self.no
    
    def getCityX(self):
        return self.x
    
    def getCityY(self):
        return self.y
    
    def setHDis(self, hdis):
        self.hdis = hdis
    
    def getHDis(self):
        return self.hdis
    
    def setGDis(self, gdis):
        self.gdis = gdis
        
    def getGDis(self):
        return self.gdis        
    
    def setParent(self, pp):
        self.parent = pp
    
    def getParent(self):
        return self.parent
    
    def setWindAffect(self, flag):
        self.windaffect = flag
    
    def getWindAffect(self):
        return self.windaffect

class AStar:        
    def __init__(self, start_no, goal_no):
        self.start_no = start_no
        self.goal_no = goal_no
        self.start_x = cities.get(self.start_no)[0]
        self.start_y = cities.get(self.start_no)[1]
        self.goal_x = cities.get(self.goal_no)[0]
        self.goal_y = cities.get(self.goal_no)[1]
        
        self.citiesdict = cities
        self.fringe = []
        self.path = []
        self.finDis = 0.0
        
        self.try_times = 0
        self.erro_code = 1
    
    def startFinding(self):
        scitynode = CityNode(self.start_no, self.start_x, self.start_y)
        
        if self.testGoal(scitynode):
            self.updateGDis(scitynode)
            self.updatePath(scitynode.getCityNo())
            return scitynode
        else:
            self.fringe.append(scitynode)
            self.prepared(scitynode)     
    
    def prepared(self, p):
        citynode = p
        self.fringe.remove(citynode)
        
        if self.testGoal(citynode):
            self.updateGDis(citynode)
            self.updatePath(citynode.getCityNo())
            return self.path           
        else:
            self.findnextp(citynode)
        
    def findnextp(self, p):
        parentp = p 
        
        for i in highways:
            if i[0] == parentp.getCityNo():
                childp = CityNode(i[1], self.citiesdict.get(i[1])[0], self.citiesdict.get(i[1])[1])
                childp.setParent(parentp)
                
                if childp.getCityY() < parentp.getCityY():
                    childp.setWindAffect(True)
                else:
                    childp.setWindAffect(False)
                    
                self.fringe.append(childp)
            elif i[1] == parentp.getCityNo():
                childp = CityNode(i[0], self.citiesdict.get(i[0])[0], self.citiesdict.get(i[0])[1])
                childp.setParent(parentp)
                
                if childp.getCityY() < parentp.getCityY():
                    childp.setWindAffect(True)
                else:
                    childp.setWindAffect(False)
                    
                self.fringe.append(childp)                    
                
        self.choosebestp()       
        
    def choosebestp(self):
        value = 0.0

        if not len(self.fringe) == 0:
            for i in self.fringe:
                gdis = self.updateGDis(i)
                if not i.getWindAffect():
                    hdis = self.calcCost(i.getCityX(), self.goal_x, i.getCityY(), self.goal_y)
                else:
                    hdis = float('inf')
                i.setHDis(hdis)
                i.setGDis(gdis)
            
            for j in self.fringe:
                value = j.getHDis() + j.getGDis()
                for k in self.fringe:
                    if value > (k.getHDis() + k.getGDis()):
                        value = k.getHDis() + k.getGDis()    
                            
            for l in self.fringe:
                if (l.getHDis() + l.getGDis()) == value:
                    self.updatePath(l.getParent().getCityNo())
                    self.expandbestp(l)
    
        else:
            self.erro_code = 0
            erro_str = 'Find Failed'
            print(erro_str)
            return erro_str
        
    def expandbestp(self, bp):
        if self.try_times < 50:
            self.try_times = self.try_times + 1
            self.erro_code = 1
            self.prepared(bp)
        else:
            self.erro_code = 0
            return
    
    def calcCost(self, x1, x2, y1, y2):
        cost = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
        return cost
    
    def updateGDis(self, p):
        gdis = p.getParent().getGDis() + \
        self.calcCost(p.getCityX(), p.getParent().getCityX(),
                     p.getCityY(), p.getParent().getCityY())
        p.setGDis(gdis)
        self.finDis = gdis
        return gdis
    
    def updatePath(self, cno):
        self.path.append(cno)
        return self.path
    
    def testGoal(self, tp):
        findgoal = False
        if tp.getCityNo() == self.goal_no:
            findgoal = True
        else:
            findgoal = False
            
        return findgoal
        
    def displayFindingResult(self):
        if self.erro_code == 1:
            print('PATH RECORD: ', self.path)
            print('TOTAL DISTANCE: ', self.finDis)
        elif self.erro_code == 0:
            print('PATH RECORD: ', self.path)
            print('This is the', self.try_times, 'times try,'\
                  ' and still not reach the accessible node. Program exit.')

if __name__ == '__main__':
    f = open('sample-input.yaml', 'r')
    data = yaml.load(f)
    f.close()
    
    cities = data['cities']
    highways = data['highways']
    start = data['start']
    goal = data['end']
    
    astarsort = AStar(start, goal)
    astarsort.startFinding()
    astarsort.displayFindingResult()
