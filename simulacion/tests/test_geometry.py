import unittest
from enjambre.geometry import (validate_essential_geometry, polar,
                              tray_angle, mix, ease,radial_reach)

class GeometryTest(unittest.TestCase):
    def test_reach(self):
        self.assertLess(max(validate_essential_geometry().values()),.43)
    def test_belt_alignment(self):
        self.assertAlmostEqual(tray_angle(120,150),270)
        self.assertAlmostEqual(tray_angle(300,150),90)
    def test_radial_placement(self):
        self.assertAlmostEqual(polar(1,90,0)[1],1)
    def test_smooth_movement(self):
        self.assertEqual(ease(-1),0)
        self.assertEqual(ease(2),1)
        self.assertAlmostEqual(ease(.5),.5)
        self.assertEqual(mix((0,0),(1,2),.5),(.5,1))

if __name__=='__main__':unittest.main()
