import unittest
import math
from sim_car.core import Simulator, demo_command
from sim_car.model import generate
import xml.etree.ElementTree as ET

class KinematicsTests(unittest.TestCase):
    def test_inverse_reconstructs_rigid_velocity(self):
        s=Simulator()
        for cmd in [(1,0,0),(0,1,0),(.3,.2,.8),(-1,0,0),(0,0,1)]:
            for (x,y),(angle,speed) in zip(s.positions,s.inverse(cmd)):
                self.assertAlmostEqual(speed*math.cos(angle),cmd[0]-cmd[2]*y)
                self.assertAlmostEqual(speed*math.sin(angle),cmd[1]+cmd[2]*x)
    def test_demo_limits_and_no_lateral_slip(self):
        s=Simulator(); prev=[0]*3; angles=s.angles[:]
        for _ in range(1600):
            s.step(demo_command(s.time),.02)
            vx,vy,w=s.velocity
            self.assertLessEqual(math.hypot(vx-prev[0],vy-prev[1]),.02000001)
            self.assertLessEqual(abs(w-prev[2]),s.cfg['angular_acceleration']*.02+1e-8)
            for (x,y),a,old,v in zip(s.positions,s.angles,angles,s.speeds):
                self.assertAlmostEqual(-(vx-w*y)*math.sin(a)+(vy+w*x)*math.cos(a),0,places=7)
                self.assertLessEqual(abs(v),1.500001)
                self.assertLessEqual(abs(a-old),s.cfg['steer_rate']*.02+1e-8)
            prev=s.velocity[:]; angles=s.angles[:]
    def test_straight_distance(self):
        s=Simulator()
        for _ in range(200):s.step((.5,0,0),.01)
        self.assertAlmostEqual(s.pose[0],.8775,places=6)
        self.assertAlmostEqual(s.pose[1],0)
    def test_rotation_no_translation(self):
        s=Simulator()
        for _ in range(300):s.step((0,0,.7),.02)
        self.assertAlmostEqual(s.pose[0],0); self.assertAlmostEqual(s.pose[1],0)
        self.assertGreater(s.pose[2],1)
    def test_model_tree(self):
        root=ET.fromstring(generate()); links={l.attrib['name'] for l in root.findall('link')}; children=[]
        for joint in root.findall('joint'):
            self.assertIn(joint.find('parent').attrib['link'],links)
            children.append(joint.find('child').attrib['link'])
        self.assertEqual(len(children),len(set(children)))
        self.assertEqual(links-set(children),{'base_footprint'})
        self.assertEqual(len(root.findall("joint[@type='continuous']")),8)
if __name__=='__main__':unittest.main()
