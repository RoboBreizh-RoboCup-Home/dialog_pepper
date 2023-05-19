#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import threading
import time
import qi
import argparse
import sys
import transition

def lumiere(session): # thread pour la lumière
	leds_service = session.service("ALLeds")
	duration = 3.0
	leds_service.fadeRGB("FaceLeds",1, 0, 0,duration)
	leds_service.fadeRGB("EarLeds",1, 0, 0,duration)
	leds_service.fadeRGB("ChestLeds",1, 0, 0,duration)


def mouvements(session): # thread pour les mouvements
	motion_service  = session.service("ALMotion")
	robot_posture_service = session.service("ALRobotPosture")
	
	robot_posture_service.goToPosture("StandInit", 0.5)
	motion_service.setStiffnesses("Head", 1.0)
	
	# Fermer les mains
	motion_service.closeHand('RHand')
	motion_service.closeHand('LHand')
	
	# Mouvements de la tête et du buste
	names  = ["HeadYaw", "HeadPitch", "KneePitch","HipRoll","HipPitch","LWristYaw", "RWristYaw", "LElbowYaw","RElbowYaw","LElbowRoll","RElbowRoll","LShoulderPitch","RShoulderPitch","LShoulderRoll","RShoulderRoll"]
	angles  = [0.,0.1,0.05,0.,-0.1,-1,1,0.,0.,-1.5,1.5,1.5,1.5,0.9,-0.9]
	fractionMaxSpeed  = 0.25
	motion_service.setAngles(names, angles, fractionMaxSpeed)
	
	time.sleep(5.0)
	
	motion_service.setStiffnesses("Head", 0.0)
	robot_posture_service.goToPosture("StandInit", 0.5)

def main(session):
	
	# Init threads
	thread_1 = threading.Thread(target = lumiere, args=(session,))
	thread_2 = threading.Thread(target = mouvements, args=(session,))
    
	# Start threads
	thread_1.start()
	thread_2.start()
	thread_1.join()
	thread_2.join()
	
	# Transition
	transition.trans()

if __name__ == "__main__":
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
	main(session)


