#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

from geometry_msgs.msg import Twist, Vector3
import smach
import smach_ros

from std_msgs.msg import Empty, UInt8
import math

# from sensor_msgs.msg import Image #, CompressedImage

################################################
# from cv_bridge import CvBridge, CvBridgeError
# import numpy as np
# import tf
# import cv2 
# import time
# import encontra_objeto

# bridge = CvBridge()

# cv_image = None

# # Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
# media = []
# centro = []
# area = 0.0

# # tolerancia_x = 20
# # tolerancia_y = 20
# # ang_speed = 0.1
# # area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
# # tolerancia_area = 20000

# # Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
# atraso = 0.3E9
# check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

# delay = 0.04

# def roda_todo_frame(imagem):
# 	print("frame")
# 	global cv_image
# 	global media
# 	global centro
# 	global area

# 	now = rospy.get_rostime()
# 	imgtime = imagem.header.stamp
# 	lag = now-imgtime
# 	delay = lag.secs
# 	if delay > atraso and check_delay==True:
# 		return 
# 	try:
# 		antes = time.clock()
# 		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

# 		media, centro, area = encontra_objeto.identifica_cor(cv_image)
		
# 		depois = time.clock()
# 		cv2.imshow("Camera", cv_image)
# 	except CvBridgeError as e:
# 		print('ex', e)
################################################

take_off = None
landing = None
flipping = None
empty_msg = Empty()

#Variáveis para fazer o drone andar em espiral
teta = None
incremento_angular = 0.1


class Takeoff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['andar'])

	def execute(self, userdata):
		global take_off
		global empty_msg

		rospy.sleep(0.1)#Ver se esse tempo é OK
		take_off.publish(empty_msg)
		rospy.sleep(0.1)#Ver se esse tempo é OK

		print("takeoff")
		teta = 0

		return 'andar'

class Andar(smach.State):
	def __init__(self):
    	#alinhou1 = referente ao objeto1
		smach.State.__init__(self, outcomes=['pousar','andar'])

	def execute(self, userdata):
		global velocidade_saida
		global flipping
		global teta
		global incremento_angular
		
		# rospy.sleep(0.1)
		# vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
		# velocidade_saida.publish(vel)
		# rospy.sleep(0.05) #Ver se é necessário, se não der problema então deixar sem.

		print(teta)
		if teta <= 4*math.pi:

			x_linear = -(math.sin(teta))/2
			y_linear = (math.cos(teta))/2

			# z_angular = incremento_angular #Usar isso para que a frente do drone também vire.

			rospy.sleep(0.1)
			vel = Twist(Vector3(x_linear, y_linear, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			# rospy.sleep(0.05) #Ver se é necessário, se não der problema então deixar sem.

			print('rodando')
			teta += incremento_angular

			return 'andar'
		
		else:
			rospy.sleep(0.5)#Ver se esse tempo é OK
			flipping.publish(2) #0 - Forward; 1 - Backward; 2 - Right; 3 - Left
			rospy.sleep(0.5)#Ver se esse tempo é OK
			
			return 'pousar'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['parar'])

	def execute(self, userdata):
		global landing
		global empty_msg
		rospy.sleep(0.1)#Ver se esse tempo é OK
		landing.publish(empty_msg)
		
		print('pousa')
		rospy.sleep(0.1)#Ver se esse tempo é OK
		
		return 'parar'

# main
def main():
	global velocidade_saida
	global take_off
	global landing
	global flipping

	global buffer

	rospy.init_node('drone_drive')

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)

	take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
	landing = rospy.Publisher('bebop/land', Empty, queue_size = 1)
	flipping = rospy.Publisher('bebop/flip', UInt8, queue_size = 1)

	#Ver para o bebop
	# recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

    # Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['fim'])

	# Open the container
	with sm:
	    # Add states to the container
		smach.StateMachine.add('TAKEOFF', Takeoff(),
	    	transitions={'andar':'ANDAR'})

		smach.StateMachine.add('ANDAR', Andar(),
	    	transitions={'pousar':'LAND', 'andar':'ANDAR'})

		smach.StateMachine.add('LAND', Land(),
	    	transitions={'parar':'fim'})




	# Execute SMACH plan
	outcome = sm.execute()


if __name__ == '__main__':
	main()
