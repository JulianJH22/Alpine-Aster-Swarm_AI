import unittest
from enjambre.control import (RobotCommand,CellObservation,ScriptedController,FutureAIController)

class ControllerTest(unittest.TestCase):
    def test_contract(self):
        c=ScriptedController()
        obs=CellObservation(0,'test',{}, {},0,[],{})
        self.assertEqual(c.decide(obs),())
        action=RobotCommand('brazo_1',(0.0,0.0,.25),1,1.0)
        c.set_commands([action])
        self.assertEqual(tuple(c.decide(obs)),(action,))
        with self.assertRaises(NotImplementedError):
            FutureAIController().decide(obs)

if __name__=='__main__':unittest.main()
