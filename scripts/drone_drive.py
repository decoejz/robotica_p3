#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

from geometry_msgs.msg import Twist, Vector3
import smach
import smach_ros

from std_msgs.msg import Empty, UInt8
import time

take_off = None
landing = None
empty_msg = Empty()

flipping = None

velocidade_teste = Twist()

antes = None
t = 0

class Takeoff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['andar'])

	def execute(self, userdata):
		global take_off
		global empty_msg
		global antes

		rospy.sleep(1.)
		take_off.publish(empty_msg)
		rospy.sleep(1.)

		print("takeoff")

		antes = time.clock()
		t = 0
		return 'andar'

class Andar(smach.State):
	def __init__(self):
    	#alinhou1 = referente ao objeto1
		smach.State.__init__(self, outcomes=['pousar','andar'])

	def execute(self, userdata):
		global velocidade_saida
		global velocidade_teste
		global t
		global flipping

		rospy.sleep(0.1)
		vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		# rospy.sleep(0.05)

		# depois = time.clock()
		# tempo = depois - antes

		# if tempo <= 1:
		print(t)
		if t <= 25:
			# depois = time.clock()
			print('anda')
			t += 1
			return 'andar'
		else:
			rospy.sleep(1.)
			flipping.publish(2)
			rospy.sleep(2.)
			return 'pousar'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['parar'])

	def execute(self, userdata):
		global landing
		global empty_msg
		rospy.sleep(1.)
		landing.publish(empty_msg)
		print('pousa')
		rospy.sleep(1.)
		t = 0
		return 'parar'

# main
def main():
	global velocidade_saida
	global take_off
	global landing
	global flipping

	global buffer

	rospy.init_node('drone_drive')

	# Para usar a webcam
	# recebedor = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)

	take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
	landing = rospy.Publisher('bebop/land', Empty, queue_size = 1)
	
	flipping = rospy.Publisher('bebop/flip', UInt8, queue_size = 1)

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


	#while not rospy.is_shutdown():
	#	take_off.publish(empty_msg)
	#	rospy.sleep(15.)
	#	print("takeoff")
	#	landing.publish(empty_msg)
	#	rospy.sleep(15.)
	#	print('landing')
		# outcome = sm.execute()


if __name__ == '__main__':
	main()
