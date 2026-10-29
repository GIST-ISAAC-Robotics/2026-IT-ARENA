import math
import unittest
import numpy as np
from rounded_sweep import rounded_swept_limit, sampled_rounded_limit


class RoundedTests(unittest.TestCase):
    def test_axial_collision_analytic_lower_bound(self):
        # 후륜/차체 중심이 직진하므로 앞쪽 첫 접촉은1-.1-.055-.06=.785 m다.
        value=rounded_swept_limit([[1.,0.]],0.,wheelbase=.135,horizon=1.5,margin=.115)
        self.assertLessEqual(value,.785)
        self.assertGreaterEqual(value,.765)

    def test_existing_fixed_box_stays_box(self):
        self.assertEqual(rounded_swept_limit([[.15,.12]],0.,margin=.055),0.)
        self.assertEqual(rounded_swept_limit([[0.,0.]],0.,margin=.055),0.)

    def test_diagonal_speed_disk_differs_from_axis_square(self):
        # 각 축50 mm의 동시 이동은 이동량70.7 mm다. 60 mm 이동 원판 밖이다.
        self.assertGreater(rounded_swept_limit([[.205,.18]],0.,margin=.115,step=.005),0.)

    def test_reflection(self):
        points=np.array([[.4,.2],[.8,-.12],[.25,.5]])
        a=rounded_swept_limit(points,.2,wheelbase=.135,margin=[.1,.12,.055])
        b=rounded_swept_limit(points*[1.,-1.],-.2,wheelbase=.135,margin=[.1,.12,.055])
        self.assertEqual(a,b)

    def test_arc_bound_does_not_cross_dense_contact(self):
        for steering in (-.3,.0,.3):
            curve=math.tan(steering)/.135
            s=np.linspace(0,1.5,15001)
            yaw=curve*s
            rx=s-.0675 if not curve else np.sin(yaw)/curve-.0675
            ry=np.zeros_like(s) if not curve else (1-np.cos(yaw))/curve
            for px,py in ((.4,.2),(.7,0.),(.3,-.3)):
                dx,dy=px-rx,py-ry
                x=np.cos(yaw)*dx+np.sin(yaw)*dy-.0675
                y=-np.sin(yaw)*dx+np.cos(yaw)*dy
                d=np.hypot(np.maximum(abs(x)-.155,0.),np.maximum(abs(y)-.13,0.))
                hit=np.flatnonzero(d<=.06)
                result=rounded_swept_limit([[px,py]],steering,wheelbase=.135,horizon=1.5,margin=.115)
                if len(hit):self.assertLessEqual(result,float(s[hit[0]])+1e-12)

    def test_invalid_input(self):
        for kwargs in ({'margin':.03},{'margin':float('nan')},{'step':0.},{'wheelbase':0.}):
            with self.assertRaises(ValueError):rounded_swept_limit([[1.,0.]],0.,**kwargs)

    def test_same_sampling_disk_not_larger_than_axis_obstacle(self):
        points=np.array([[.35,.2],[.75,-.14],[.28,.4]])
        for steering in (-.3,0.,.3):
            axis=rounded_swept_limit(points,steering,wheelbase=.135,margin=[.11,.12,.055],motion_shape='axis')
            disk=rounded_swept_limit(points,steering,wheelbase=.135,margin=[.11,.12,.055])
            self.assertGreaterEqual(disk,axis)

    def test_sampled_axis_reconstructs_original_rule(self):
        def old(points,angle,padding):
            curve=math.tan(angle)/.135
            for distance in np.arange(0.,2.501,.04):
                yaw=curve*distance
                rx=distance-.0675 if abs(curve)<1e-5 else math.sin(yaw)/curve-.0675
                ry=0. if abs(curve)<1e-5 else (1-math.cos(yaw))/curve
                dx,dy=points[:,0]-rx,points[:,1]-ry
                x=math.cos(yaw)*dx+math.sin(yaw)*dy-.0675
                y=-math.sin(yaw)*dx+math.cos(yaw)*dy
                if np.any((abs(x)<=.1+padding)&(abs(y)<=.075+padding)):
                    return max(0.,float(distance)-.04)
            return 2.5
        rng=np.random.default_rng(18)
        for angle in np.linspace(-.37,.37,17):
            points=rng.uniform([-.3,-.5],[2.,.5],(40,2));padding=rng.uniform(.055,.15,40)
            axis=sampled_rounded_limit(points,angle,wheelbase=.135,horizon=2.5,margin=padding,motion_shape='axis')
            disk=sampled_rounded_limit(points,angle,wheelbase=.135,horizon=2.5,margin=padding)
            self.assertEqual(axis,old(points,angle,padding))
            self.assertGreaterEqual(disk,axis)


if __name__=='__main__':unittest.main()
