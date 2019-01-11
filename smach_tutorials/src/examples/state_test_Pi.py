#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy


# define state Foo
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found'])
        self.motion_search = [0,0,0,0,0,0.5]

    	self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('chatter',
                                          bool,queue_size=1000)

    def execute(self, userdata):
        rospy.loginfo('Searching')
        while self.sub_gate == 0:
            # search for gate
            self.pub_motion.publish(self.motion_search)
        return 'Found'



# define state Bar
class Attack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Goal', 'Lost_target'])
        self.motion_attack = [1,0,0,0,0,0]
        self.motion_goal = [0,0,0,0,0,0]
        
        self.pub_motion = rospy.Publisher('propulsion_command',
                                        PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('chatter',
                                          bool,queue_size=1000)
        #self.sub_gate_pos = rospy.Subscriber('GateCenter', 
         #                               CenterOfGate, queue_size=10000)

    #def goal_center_pid(self):
        #some PI
        

    def execute(self, userdata):
        self.pub_motion.publish(self.motion_attack)

	if self.gate_passed == 1:
        self.pub_motion.publish(self.motion_goal)
        rospy.loginfo('Attacking')
        return 'Goal'

	if self.sub_gate == 0:
	    rospy.loginfo('Gate lost')
	    return 'Lost_target'


def main():
    rospy.init_node('Testing_manta_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Done'])

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('Super_duper_test_server', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Search', Search(), 
                               transitions={'Found':'Attack'})
        smach.StateMachine.add('Attack', Attack(), 
                               transitions={'Goal':'Done', 'Lost_target':'Search'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
