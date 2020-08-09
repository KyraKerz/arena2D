#!/usr/bin/env python2
import rospy
import unittest
import sys
import numpy as np
import numpy.random
from arena2d.srv import interactionDiscActs


class TestArenaInteractionSev(unittest.TestCase):

    def test_getting_response(self):
        service_name = "area2d/interaction"
        try:
            rospy.wait_for_service(service_name, 10)
        except rospy.ROSException() as e:
            print("time out for waiting a the interaction service:\t"+str(e))
            sys.exit(-1)
        send_actions = rospy.ServiceProxy(service_name,
                                          interactionDiscActs)
        action_range_min = 0
        action_range_max = 6
        num_envs = 4
        for i in range(8):
            actions = np.random.randint(action_range_min, action_range_max+1, num_envs)
            resp = send_actions(actions)
            print(resp)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("arena2d", 'teset_interaction_service', TestArenaInteractionSev)
