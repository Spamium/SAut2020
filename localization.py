import datetime
import math

class localization:
	
	# this is our last position
	position = [0, 0, 0]
	# List of beacon tuples = ( beaconID, beaconPosition )
	# investigate possibility of identifying beacon by position
	beaconList = []
	
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
		relativeEst = multilat()
		
		
	

	
	def Multilat(self):
	
		distance = []
		# TODO - grab algorithm for multilateration
		for b in self.beaconList:
			distance.append( (b[1], calcDist(b[0]) ) )
	
	
	
	def GetFromIMU(self):
	
	def GetFromDVL(self):
	
	def GetFromLBL(self, beacon):
		
