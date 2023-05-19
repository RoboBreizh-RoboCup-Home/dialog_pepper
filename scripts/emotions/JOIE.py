#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import threading
import time
import qi
import argparse
import sys
import transition

def mouvements(session): # thread pour les mouvements
	motion_service  = session.service("ALMotion")
	robot_posture_service = session.service("ALRobotPosture")

	robot_posture_service.goToPosture("StandInit", 0.5)
	motion_service.setStiffnesses("Head", 1.0)
	
	motion_service.openHand('RHand')
	motion_service.openHand('LHand')
	
	names  = ["HeadYaw", "HeadPitch", "HipPitch","LShoulderPitch", "RShoulderPitch", "LElbowYaw","RElbowYaw","LElbowRoll","RElbowRoll"]
	angles  = [-0.3, -0.1, 0.1,-1,-1,-1.8,1.8,-0.5,0.2]
	fractionMaxSpeed  = 0.3
	motion_service.setAngles(names, angles, fractionMaxSpeed)
	
	time.sleep(3.0)
	
	motion_service.setStiffnesses("Head", 0.0)
	robot_posture_service.goToPosture("StandInit", 0.5)
	
def lumiere(session): # thread pour la lumière
	leds_service = session.service("ALLeds")
	duration = 7.0
	leds_service.rasta(duration)


def main(session):
	# Init threads
	thread_2 = threading.Thread(target = mouvements, args=(session,))
	thread_3 = threading.Thread(target = lumiere, args=(session,))
    
	# Start threads
	thread_2.start()
	thread_3.start()
	thread_2.join()
	thread_3.join()
	
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


