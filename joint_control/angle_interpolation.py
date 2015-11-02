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
from keyframes import * 
import numpy as np

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
        self.time_created = self.perception.time
        
    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        (names, times, keys) = keyframes
        current_time = self.perception.time - self.time_created
        for index, name in enumerate(names):
            points = zip(times[index], keys[index])
            angle = self._get_angle(points, current_time)
            if angle != None and self.target_joints.has_key(name):
                target_joints[name] = angle

        return target_joints

    def _get_angle(self, points, time):
        if time < points[0][0]:
            return None
        elif time > points[-1][0]:
            return None

        times = [i for (i, _) in points]
        ci = self._get_curve_index(times, time)

        #print 'curve index is: ' + str(ci)
        (t0, [ang0, [_, dt0, dAng0], [_, dt1, dAng1]]) = points[ci]
        (t1, [ang1, [_, dt2, dAng2], [_, dt3, dAng3]]) = points[ci+1]
        p0 = np.array([t0, ang0])
        p1 = np.array([t0+dt1, ang0+dAng1])
        p2 = np.array([t1+dt2, ang1+dAng2])
        p3 = np.array([t1, ang1])
        t = (time - t0)/(t1-t0)
        #print 'p0 ' + str(p0) + ' p1 ' + str(p1) + ' p2 ' + str(p2) + ' p3 ' + str(p3)
        #print 't: ' + str(t)
        bezier = ((1-t)**3)*p0 + \
                 3*t*((1-t)**2)*p1 + \
                 3*(t**2)*(1-t)*p2 + \
                 (t**3)*p3
        return bezier[1]

    def _get_curve_index(self, times, time):
        for i in range(len(times)-1):
            if (time >= times[i]) and (time <= times[i+1]) :
                return i

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBellyToStand()
    agent.run()
