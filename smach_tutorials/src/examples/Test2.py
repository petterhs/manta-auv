#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Foo
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Searching')
        while gate == 0:
            # search for gate 
        return'Found'



# define state Bar
class Attack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Goal', 'Lost_target'])

    def execute(self, userdata):
	# make the damn thing move
	if gate_passed == 1:
            rospy.loginfo('Attacking')
            return'Goal'
	if gate == 0:
	    rospy.loginfo('Gate lost')
	    return'Lost_target'


def main():
    rospy.init_node('Testing_manta_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Done'])

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('Super_test_server', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Searching', Search(), 
                               transitions={'Found':'Attack'})
        smach.StateMachine.add('Attacking', Attack(), 
                               transitions={'Lost_target':'Seaching', 'Goal':'Done'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
