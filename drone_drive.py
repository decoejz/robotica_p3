#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

from geometry_msgs.msg import Twist
import smach
import smach_ros

take_off = 

class Takeoff(smach.State):
    def __init__(self):
    	#alinhou1 = referente ao objeto1
        smach.State.__init__(self, outcomes=['andar'])

    def execute(self, userdata):
		global velocidade_saida

		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)

		

# main
def main():
	global velocidade_saida
	
	global buffer
	
	rospy.init_node('cor_maq_est')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('TAKEOFF', Takeoff(),
	                            transitions={'girando': 'TAKEOFF',
	                            'alinhou1':'REACAO1','enxergou2':'REACAO2','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO1', Reage1(),
	                            transitions={'centralizado': 'REACAO1',
	                            'alinhando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO2', Reage2(),
	                            transitions={'centralizado': 'REACAO2',
	                            'procurando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('PERIGOSO', Parar(),
	                            transitions={'perigo': 'PERIGOSO',
	                            'seguro':'GIRANDO'})


	# Execute SMACH plan
	outcome = sm.execute()


if __name__ == '__main__':
    main()

