
#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import threading
import time
import qi
import argparse
import sys
import random

def tablette(session): # thread pour la tablette
	tabletService = session.service("ALTabletService")
	tabletService.hideImage()
	
def lumiere(session): # thread pour la lumiere
	leds_service = session.service("ALLeds")
	leds_service.off("FaceLeds")
	leds_service.off("EarLeds")
	leds_service.off("ChestLeds")
	
	
def mouvements(session): # thread pour les mouvements
	motion_service  = session.service("ALMotion")
	robot_posture_service = session.service("ALRobotPosture")
	robot_posture_service.goToPosture("StandInit", 0.5)
	motion_service.setStiffnesses("Head", 1.0)
	
	motion_service.openHand('RHand')
	motion_service.openHand('LHand')
	
	i=1
	# [A MODIFIER] a revoir en fonction de la methode d'arret
	while (i<4):
		a=(random.random()/10)
		l=(random.random()/10)
		r=(random.random()/10)
		names = ["HeadPitch", "KneePitch","HipRoll","HipPitch","LWristYaw", "RWristYaw", "LElbowYaw","RElbowYaw","LElbowRoll","RElbowRoll","LShoulderPitch","RShoulderPitch","LShoulderRoll","RShoulderRoll"]
		angles  = [a,a,a,a,l,r,(3*l-1.5),(3*r+1.5),(l-0.1),(r+0.1),(l+1.3),(r+1.3),(l+0.1),(r-0.1)]
		fractionMaxSpeed  = 0.01
		motion_service.setAngles(names, angles, fractionMaxSpeed)
		time.sleep(5.0)
		i=i+1
		
	motion_service.setStiffnesses("Head", 0.0)

def trans() :
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="172.30.4.30",
						help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
	parser.add_argument("--port", type=int, default=9559,
						help="Naoqi port number")
	args = parser.parse_args()
	session = qi.Session()
	try:
		session.connect("tcp://" + args.ip + ":" + str(args.port))
	except RuntimeError:
		print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
			   "Please check your script arguments. Run with -h option for help.")
		sys.exit(1)
		
	thread_1 = threading.Thread(target = tablette, args=(session,))
	thread_2 = threading.Thread(target = lumiere, args=(session,))
	thread_3 = threading.Thread(target = mouvements, args=(session,))
		
	# Start threads
	thread_1.start()
	thread_2.start()
	thread_3.start()
