import datetime
import math

class localization:
	
	# this is our last position
	position = [0, 0, 0]
	beaconPosition = []
	
	# uncertainties
	positionUncert
	
	
	lastTime = 0
	deltaT = 0
	
	
	def __init__(self, beaconList):
		lastTime = datetime.now()
		# TODO - INITIALIZE BEACONS
		
	
	def kalmanStep(self):
		
		# deltaT from last step
		deltaT = datetime.now() - lastTime
		
		# location estimate with basic equation of motion
		inertialEst = self.position + getfromDVL()*deltaT + 0.5*getFromIMU()*deltaT*deltaT
	
		# relative estimation
		relativeEst = getFromLBL()
		
		
	
	def GetFromIMU(self):
	
	def GetFromDVL(self):
	
	def getFromLBL(self):
		
		distance = []
		for b in beaconPosition:
			distance.append( (b[1], calcDist(b[0]) )
			
		
		
	def calcDist(self, beacon):
	
	
	def 