# This is the main program to run the simulator with the given design.

# Author : Dilip C Kavarthapu


import os
import sys
import subprocess
import random
import optparse
import random
import time
import math
import numpy as np

# Need to import python modules from the $SUMO_HOME/tools directory.
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci


# IDs of lanes

listLanes = ['8949170_0', '8949170_1', \
			'-164126513_0', '-164126513_1',\
			'52016249_0', '52016249_1',\
			'-164126511_0', '-164126511_1']

# IDs of edges

edges = ['8949170 -52016249', '8949170 164126513', '8949170 164126511', \
							'-164126513 164126511', '-164126513 -8949170', '-164126513 -52016249',\
							'52016249 164126513', '52016249 164126511', '52016249 -8949170',\
							'-164126511 -8949170', '-164126511 -52016249', '-164126511 164126513']

# Hour to record arrival data

dfArrivalData = ['hour', '0', '1', '2', \
							'3', '4', '5',\
							'6', '7', '8',\
							'9', '10', '11']

colNames = list(dfArrivalData)

SL = "65546898" # ID of stoplight

random.seed(int(time.time()))


# Route map to match inflow road to outflow road.

routemap = [[-1,7,6,8], [10,-1,11,9], [5,3,-1,4], [0,2,1,-1]]



# This function generates traffic and creates an xml file that contains details about the traffic
# including time, direction, type, etc

def write_routes():

	arrivals = []

	secs = 0
	hrs = 0

	# Variables initialization for simulation purposes.

	x1 = random.uniform(10,11)
	x2 = random.uniform(18.5,19.5)

	v1 = random.uniform(0.5,1.5)
	v2 = random.uniform(0.5,1.5)
	v3 = random.uniform(5,7)

	# Iterating in a day

	while secs < 60*60*24:
		if secs > 60*60*24:
			break	
		hrs = secs/(60*60)

		pro = random.uniform(0,1)

		# Generating traffic according to the traffic density function

		if (pro < traffic_density(x1,x2,3.5,2.5,secs/(60*60.0))*7):


			# Deciding From and To destinations using the probability distribution

			total = poisson_prob(x1, hrs) + poisson_prob(x2,hrs) + 0.04 + gaussian_prob(x1,v3,hrs)
			fromm = np.random.choice(4,p=[poisson_prob(x1,hrs)/total, poisson_prob(x2,hrs)/total, 0.04/total, gaussian_prob(x1,v3,hrs)/total])
			to = fromm

			total = poisson_prob(x2, hrs) + poisson_prob(x1,hrs) + 0.04 + gaussian_prob(x2,v3,hrs)
			while (to == fromm):
				to = np.random.choice(4,p=[poisson_prob(x2,hrs)/total, poisson_prob(x1,hrs)/total, 0.04/total, gaussian_prob(x2,v3,hrs)/total])

			route = routemap[fromm][to]

			arrivals.append([secs, route, random.randint(1,2)])

		secs += 2

		# Writing the traffic into an xml file to read.

	with open('palm.rand.rou.xml', 'w') as routes:
		routes.write("""<?xml version="1.0"?>""" + '\n' + '\n')
		routes.write("""<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">""" + '\n')
		routes.write('\n')
		routes.write("""<vType id="type1" accel="0.5" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" color="255,105,180"/>""" + '\n')
		routes.write("""<vType id="type0" accel="0.3" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" color="0,255,255"/>""" + '\n')
		routes.write('\n')	
		for i in range(12):
			routes.write("""<route id=\"""" + str(i) + """\"""" + """ edges=\"""" + edges[i] + """\"/> """ + '\n')
			#    <route id="route0" color="1,1,0" edges="beg middle end rend"/>

		routes.write('\n')
		idCounter = 0
		for i in arrivals:
			if (i[2]==1):
				color = """ color=\"255,105,180\""""
				vType = """\" type=\"type1"""
			else:
				color = """ color=\"0,255,255\""""
				vType = """\" type=\"type0"""
			routes.write("""<vehicle id=\"""" + str(idCounter) + """\" depart=\"""" + str(round(i[0],2)) + """\" route=\"""" + str(i[1]) + vType + """\"/>""" + '\n')
			# routes.write("""     <route edges=\"""" + str(i[1]) + """\"""" + """/>""" + '\n')
			# routes.write("""</vehicle>""" + '\n')
			idCounter += 1
		routes.write("""</routes>""")	


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def gaussian_prob(mu, sigma, x):

	prob = 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (x - mu)**2 / (2 * sigma**2) )
	return (prob)

def poisson_prob(mu, x):

	prob = ((mu**x)*math.exp(-mu))/math.factorial(int(x))
	return (prob)

def traffic_density(x1,x2,v1,v2,time):

	probab = (gaussian_prob(x1,v1,time) + 3*gaussian_prob(x2,v2,time)/4)/2

	return (probab)

def importance_sum(x1,x2,v1,v2,v3,x):

	summ = poisson_prob(x1, x) + poisson_prob(x2,24-x) + pow(24,-1) + gaussian_prob(x1,v3,x)

	return (summ)



def train_Q(state, weights, time, epsilon, retu): #(state, weights, time)


	M = 16
	yel = 1

	# Get the state 

	st_psi = np.zeros(len(state))
	for i in range(len(state)):
		st_psi[i] = state[i]

	# Create probability distribution and pick an action (a_t)

	rand = random.uniform(0,1)
	prob = [0 for i in range(M)]
	if (rand<epsilon):
		prob = [1.0/M for i in range (M)]
	else:
		dot = [np.dot(weights[x],st_psi) for x in range(M)]
		A = np.where(dot==max(dot))[0]
		for i in A:
			prob[i] = 1.0/len(A)



	at = np.random.choice(M, p = prob)

	if (at/4 == 0):
		traci.trafficlights.setPhase(SL,0)
		yel = 1
	elif(at/4 == 1):
		traci.trafficlights.setPhase(SL,2)
		yel = 3
	elif(at/4 == 2):
		traci.trafficlights.setPhase(SL,4)
		yel = 5
	elif(at/4 == 3):
		traci.trafficlights.setPhase(SL,6)
		yel = 7

	timee = 12 + (at%4)*8


	# Rewards!!

	reward = 0

	for i in range(timee):

		traci.simulationStep()
		for lane in listLanes:
			reward = reward - traci.lane.getWaitingTime(str(lane))/60
			#reward = reward + (traci.lane.getLastStepMeanSpeed(str(lane))*traci.lane.getLastStepVehicleNumber(str(lane)))/(max_No_cars[lane]*max_Mean_speed[lane]) #- traci.lane.getWaitingTime(str(lane))


	traci.trafficlights.setPhase(SL,yel)

	for i in range(4):

		traci.simulationStep()
		for lane in listLanes:
			reward = reward - traci.lane.getWaitingTime(str(lane))/60
			#reward = reward + (traci.lane.getLastStepMeanSpeed(str(lane))*traci.lane.getLastStepVehicleNumber(str(lane)))/(max_No_cars[lane]*max_Mean_speed[lane]) #- traci.lane.getWaitingTime(str(lane))


	reward = reward/(timee+4)
	reward = reward

	retu = retu + reward

	# New state now!!

	state = []
	for lane in listLanes:
		Halted_cars[lane] = traci.lane.getLastStepHaltingNumber(str(lane))/max_Halted_cars[lane]
		No_cars[lane] = traci.lane.getLastStepVehicleNumber(str(lane))/max_No_cars[lane]
		Waiting_time[lane] = (traci.lane.getWaitingTime(str(lane))/60)/max_Waiting_time[lane]
		state.append(Halted_cars[lane])
		state.append(No_cars[lane])
		state.append(Waiting_time[lane])
	
	stt_psi = np.zeros(len(state))
	for i in range(len(state)):
		stt_psi[i] = state[i]



	# Compute delta and update the weights

	max_q = max([np.dot(weights[x] , stt_psi) for x in range(M)])
	deltat = reward + gamma * max_q - np.dot(weights[at] , st_psi)

	weights[at] = weights[at] + alpha * deltat * st_psi

	return (state,weights,timee+4,retu)



def train_SARSA(state, weights, time, epsilon, retu): #(state, weights, time)


	M = 16
	yel = 1

	# Get the state

	st_psi = np.zeros(len(state))
	for i in range(len(state)):
		st_psi[i] = state[i]

	# Create probability distribution and pick an action (a_t)

	rand = random.uniform(0,1)
	prob = [0 for i in range(M)]
	if (rand<epsilon):
		prob = [1.0/M for i in range (M)]
	else:
		dot = [np.dot(weights[x],st_psi) for x in range(M)]
		A = np.where(dot==max(dot))[0]
		for i in A:
			prob[i] = 1.0/len(A)


	at = np.random.choice(M, p = prob)

	if (at/4 == 0):
		traci.trafficlights.setPhase(SL,0)
		yel = 1
	elif(at/4 == 1):
		traci.trafficlights.setPhase(SL,2)
		yel = 3
	elif(at/4 == 2):
		traci.trafficlights.setPhase(SL,4)
		yel = 5
	elif(at/4 == 3):
		traci.trafficlights.setPhase(SL,6)
		yel = 7

	timee = 12 + (at%4)*8


	# Rewards!!

	reward = 0

	for i in range(timee):

		traci.simulationStep()
		for lane in listLanes:
			reward = reward - traci.lane.getWaitingTime(str(lane))/60
			#reward = reward + (traci.lane.getLastStepMeanSpeed(str(lane))*traci.lane.getLastStepVehicleNumber(str(lane)))/(max_No_cars[lane]*max_Mean_speed[lane]) #- traci.lane.getWaitingTime(str(lane))


	traci.trafficlights.setPhase(SL,yel)

	for i in range(4):

		traci.simulationStep()
		for lane in listLanes:
			reward = reward - traci.lane.getWaitingTime(str(lane))/60
			#reward = reward + (traci.lane.getLastStepMeanSpeed(str(lane))*traci.lane.getLastStepVehicleNumber(str(lane)))/(max_No_cars[lane]*max_Mean_speed[lane]) #- traci.lane.getWaitingTime(str(lane))


	reward = reward/(timee+4)
	reward = reward

	retu = retu + reward

	# New state now!!

	state = []
	for lane in listLanes:
		Halted_cars[lane] = traci.lane.getLastStepHaltingNumber(str(lane))/max_Halted_cars[lane]
		No_cars[lane] = traci.lane.getLastStepVehicleNumber(str(lane))/max_No_cars[lane]
		Waiting_time[lane] = (traci.lane.getWaitingTime(str(lane))/60)/max_Waiting_time[lane]
		state.append(Halted_cars[lane])
		state.append(No_cars[lane])
		state.append(Waiting_time[lane])
	
	stt_psi = np.zeros(len(state))
	for i in range(len(state)):
		stt_psi[i] = state[i]


	# Next action!!!!!

	rand = random.uniform(0,1)
	prob = [0 for i in range(M)]
	if (rand<epsilon):
		prob = [1.0/M for i in range (M)]
	else:
		dot = [np.dot(weights[x],stt_psi) for x in range(M)]
		A = np.where(dot==max(dot))[0]
		for i in A:
			prob[i] = 1.0/len(A)

	att = np.random.choice(M, p = prob)



	# Compute delta and update the weights

	deltat = reward + gamma * np.dot(weights[att] , stt_psi) - np.dot(weights[at] , st_psi)

	weights[at] = weights[at] + alpha * deltat * st_psi

	return (state,weights,timee+4,retu)



# States

No_cars = {}
Halted_cars = {}
Waiting_time = {}
Mean_speed = {}
max_No_cars = {}
max_Halted_cars = {}
max_Waiting_time = {}
max_Mean_speed = {}
gamma = 1
alpha = 0.005
epsilon = 0.05


for lane in listLanes:
	No_cars[lane] = 0
	Halted_cars[lane] = 0
	Waiting_time[lane] = 0




if __name__ == "__main__":


	# First Simulation for some parameters to scale down with

	write_routes()

	options = get_options()
	if options.nogui:
	        sumoBinary = checkBinary('sumo')
	else:
	        sumoBinary = checkBinary('sumo-gui')
	sumoBinary = checkBinary('sumo')

	traci.start([sumoBinary, "-c", "palm.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])

	max_time = 86400

	for lane in listLanes:
		max_Halted_cars[lane] = 1
		max_No_cars[lane] = 1
		max_Waiting_time[lane] = 1
		max_Mean_speed[lane] = 1

	time = 0
	rr = 0
	rew = 0

	while (time<max_time):

		if (time%21==0):
			rew = rew +rr/20
			traci.trafficlights.setPhaseDuration(SL, 20)
			rr = 0

		for lane in listLanes:
			max_Halted_cars[lane] = max(max_Halted_cars[lane],traci.lane.getLastStepHaltingNumber(str(lane)))
			max_No_cars[lane] = max(max_No_cars[lane],traci.lane.getLastStepVehicleNumber(str(lane)))
			max_Waiting_time[lane] = max(max_No_cars[lane],traci.lane.getWaitingTime(str(lane))/60)
			max_Mean_speed[lane] = max(max_Mean_speed[lane], traci.lane.getLastStepMeanSpeed(str(lane)))

		traci.simulationStep()
		for lane in listLanes:
			rr = rr - traci.lane.getWaitingTime(str(lane))/60
		time+=1
	
	print ("Returns: ", rr)
	traci.close()


	# Start actual train


	stlen = 24
	no_actions = 16
	N = 2
	weights = np.zeros((no_actions, stlen))
	decide_time = -4
	state = []
	episodes = 5
	trials = 50
	returns = []


	for j in range(trials):
		ret = [] 
		weights = np.zeros((no_actions, stlen))
		print ("Trial No: ",j)
		for i in range(episodes):
			print ("Episode No: ", i)
			write_routes()
			if options.nogui:
			        sumoBinary = checkBinary('sumo')
			else:
			        sumoBinary = checkBinary('sumo-gui')
			if (i!=2):
				sumoBinary = checkBinary('sumo')


			traci.start([sumoBinary, "-c", "palm.sumocfg",
		                             "--tripinfo-output", "tripinfo.xml"])

			retu = 0
			time = 0

			while (time<86400):
				#print (time)
				state = []
				for lane in listLanes:
					Halted_cars[lane] = traci.lane.getLastStepHaltingNumber(str(lane))/max_Halted_cars[lane]
					No_cars[lane] = traci.lane.getLastStepVehicleNumber(str(lane))/max_No_cars[lane]
					Waiting_time[lane] = (traci.lane.getWaitingTime(str(lane))/60)/max_Waiting_time[lane]
					state.append(Halted_cars[lane])
					state.append(No_cars[lane])
					state.append(Waiting_time[lane])

				if (epsilon>0.05):
					epsilon = 0.99 * epsilon

				state, weights, timee, retu = train_Q(state,weights,time,epsilon, retu)

				time += timee
			print ("Episode Returns: ", retu)
			ret.append(retu)
			traci.close()
		returns.append(ret)
		Ret = np.mean(returns, axis=0)
		np.save('q', returns)
















