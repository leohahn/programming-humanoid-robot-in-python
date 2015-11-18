'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interpolation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
import keyframes 
import numpy as np
import sys
from os import listdir
from matplotlib.cbook import flatten

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip,
                                                      simspark_port,
                                                      teamname,
                                                      player_id,
                                                      sync_mode)
        self.keyframes = ([], [], [])
        self.keyframe_name = None
        self.keyframe_init = None
        self.keyframe_end = None  # specify when the keyframe ends
                                  # relative to self.keyframe_time
        self.keyframe_time = None # specify the time on current keyframe

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def set_keyframe(self, name):
        if self.keyframe_name == name:
            return 
        
        keyframe_names = [s for s in listdir('keyframes')
                          if (s.endswith('py')
                              and s != '__init__.py')]
        if keyframe_names.count(name + '.py') == 0:
            print "error: keyframe not found"
            sys.exit(1) 

        keyframe_method = getattr(keyframes, name)
        self.keyframes = keyframe_method()
        self.keyframe_init = self.perception.time
        self.keyframe_end = self.get_end_keyframe(self.keyframes)
        self.keyframe_name = name

    def get_end_keyframe(self, keyframe):
        (_, times, _) = keyframe
        return max(list(flatten(times)))

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        (names, times, keys) = keyframes

        if (len(times) == 0):
            # the keyframe is empty, does not modify joint angles
            self.keyframes = ([], [], [])
            return {}
        
        self.keyframe_time = self.perception.time - self.keyframe_init
        if (self.keyframe_time > self.keyframe_end):
            # if keyframe is over, does not try to interpolate
            self.keyframe_name = None
            return {}

        # proceeds to the actual interpolation
        for index, name in enumerate(names):
            points = zip(times[index], keys[index])
            angle = self.get_angle(points, self.keyframe_time)
            if angle != None and self.target_joints.has_key(name):
                target_joints[name] = angle

        return target_joints

    def get_angle(self, points, time):
        if time < points[0][0]:
            return None
        elif time > points[-1][0]:
            return None

        times = [i for (i, _) in points]
        ci = self.get_curve_index(times, time)

        (t0, [ang0, [_, dt0, dAng0], [_, dt1, dAng1]]) = points[ci]
        (t1, [ang1, [_, dt2, dAng2], [_, dt3, dAng3]]) = points[ci+1]
        p0 = [t0, ang0]
        p1 = [t0+dt1, ang0+dAng1]
        p2 = [t1+dt2, ang1+dAng2]
        p3 = [t1, ang1]
        control_points = [p0, p1, p2, p3]
        t = self.t_from_x(control_points, time)
        
        return self.y_t(control_points, t) 

    def t_from_x(self, control_points, target_x):
        [p0, p1, p2, p3] = control_points

        x_tolerance = 0.0001
        lower = 0
        upper = 1
        percent = (upper + lower) / 2.
        x = self.x_t(control_points, percent)

        while abs(target_x - x) > x_tolerance:
            if target_x > x:
                lower = percent
            else:
                upper = percent
            percent = (upper + lower) / 2.
            x = self.x_t(control_points, percent)
        return percent
            
    def x_t(self, control_points, t):
        [p0, p1, p2, p3] = control_points
        return (((1 - t)**3)*p0[0] +
                3*t*((1-t)**2)*p1[0] +
                3*(t**2)*(1-t)*p2[0] +
                (t**3)*p3[0])

    def y_t(self, control_points, t):
        [p0, p1, p2, p3] = control_points
        return (((1 - t)**3)*p0[1] +
                3*t*((1-t)**2)*p1[1] +
                3*(t**2)*(1-t)*p2[1] +
                (t**3)*p3[1])

    def get_curve_index(self, times, time):
        for i in range(len(times)-1):
            if (time >= times[i]) and (time <= times[i+1]) :
                return i

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.set_keyframe('hello')
    agent.run()
