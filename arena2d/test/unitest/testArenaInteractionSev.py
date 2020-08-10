#!/usr/bin/env python2
import rospy
import unittest
import sys
import numpy as np
import numpy.random
from arena2d.srv import InteractionDiscActs, InteractionDiscActsRequest

print(sys.path)


class TestArenaInteractionSev(unittest.TestCase):
    def setUp(self):
        service_name = "area/interaction"
        try:
            rospy.wait_for_service(service_name, 10)
        except rospy.ROSException() as e:
            print("time out for waiting a the interaction service:\t"+str(e))
            sys.exit(-1)
        self.send_actions = rospy.ServiceProxy(service_name,
                                               InteractionDiscActs)
        self.action_range_min = 0
        self.action_range_max = 4
        self.num_envs = 4

    def test_sending_response(self):
        for i in range(100):
            actions = np.random.randint(self.action_range_min, self.action_range_max+1, self.num_envs)
            res = InteractionDiscActsRequest(actions)
            self.send_actions(res)

    def test_action_not_in_range(self):
        for i in range(8):
            actions = np.random.randint(self.action_range_min, self.action_range_max+1, self.num_envs)
            actions[np.random.randint(0, self.num_envs, 1)] = 100
            res = InteractionDiscActsRequest(actions)
            self.assertRaises(rospy.ServiceException, self.send_actions, res)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("arena2d", 'teset_interaction_service', TestArenaInteractionSev)
