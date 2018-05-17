#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

from geometry_msgs.msg import Twist, Vector3
import smach
import smach_ros

from std_msgs.msg import Empty
import time

class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['andar'])

    def execute(self, userdata):
        take_off.publish(empty_msg)
        return 'andar'

class Andar(smach.State):
    def __init__(self):
    	#alinhou1 = referente ao objeto1
        smach.State.__init__(self, outcomes=['pousar'])

    def execute(self, userdata):
		global velocidade_saida

		vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)

        antes = time.clock()
        depois = time.clock()
        tempo = depois - antes

        while tempo <= 6:
            depois = time.clock()
            tempo = depois - antes

        return 'pousar'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parar'])

    def execute(self, userdata):
        landing.publish(empty_msg)
        return 'parar'

# main
def main():
	global velocidade_saida

	global buffer

	rospy.init_node('drone_drive')

	# Para usar a webcam
	# recebedor = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    take_off = rospy.Publisher('drone/takeoff', Empty, queue_size = 1)
    landing = rospy.Publisher('drone/land', Empty, queue_size = 1)
    empty_msg = Empty()

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
	outcome = sm.execute()


if __name__ == '__main__':
    main()
