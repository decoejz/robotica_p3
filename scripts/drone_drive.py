#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

from geometry_msgs.msg import Twist, Vector3
import smach
import smach_ros

from std_msgs.msg import Empty
import time

take_off = None
landing = None
empty_msg = Empty()

class Takeoff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['andar'])

	def execute(self, userdata):
		global take_off
		global empty_msg

		take_off.publish(Empty())
		rospy.sleep(15.)

		print("takeoff")

		return 'andar'

class Andar(smach.State):
	def __init__(self):
    	#alinhou1 = referente ao objeto1
		smach.State.__init__(self, outcomes=['pousar'])

	def execute(self, userdata):
		# global velocidade_saida

		# vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
		# velocidade_saida.publish(vel)

		# antes = time.clock()
		# depois = time.clock()
		# tempo = depois - antes

		# while tempo <= 3:
		# 	depois = time.clock()
		# 	tempo = depois - antes
		print('anda')
		return 'pousar'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['parar'])

	def execute(self, userdata):
		global landing
		global empty_msg
		landing.publish(empty_msg)
		print('pousa')
		rospy.sleep(15.)
		return 'parar'

# main
def main():
	global velocidade_saida
	global take_off
	global landing

	global buffer

	rospy.init_node('drone_drive')

	# Para usar a webcam
	# recebedor = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)

	take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
	landing = rospy.Publisher('bebop/land', Empty, queue_size = 1)
	
    # Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['fim'])

	# Open the container
	with sm:
	    # Add states to the container
		smach.StateMachine.add('TAKEOFF', Takeoff(),
	    	transitions={'andar':'ANDAR'})

		smach.StateMachine.add('ANDAR', Andar(),
	    	transitions={'pousar':'LAND'})

		smach.StateMachine.add('LAND', Land(),
	    	transitions={'parar':'fim'})




	# Execute SMACH plan
	# outcome = sm.execute()


	if not rospy.is_shutdown():
		take_off.publish(empty_msg)
		rospy.sleep(4.)
		print("takeoff")
		landing.publish(empty_msg)
		rospy.sleep(4.)
		print('landing')
		# outcome = sm.execute()


if __name__ == '__main__':
	main()
