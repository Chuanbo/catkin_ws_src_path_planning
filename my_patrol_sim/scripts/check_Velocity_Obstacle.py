import numpy as np

class RobotforVO(object):
    def __init__(self, robot_radius, rx, ry, vx, vy):
        self.rx = rx
        self.ry = ry
        self.vx = vx
        self.vy = vy
        self.radius = robot_radius
        self.velocity = vx,vy
        self.position = rx,ry

class ObstacleforVO(object):
    def __init__(self, obs_radius, rx, ry, vx, vy):
        self.radius = obs_radius 
        self.rx = rx
        self.ry = ry
        self.vx = vx
        self.vy = vy
        self.velocity = vx,vy
        self.position = rx,ry

def collision_cone_val(robot, obstacle):
    rx = robot.rx
    ry = robot.ry
    vrx = robot.vx
    vry = robot.vy

    obx = obstacle.rx
    oby = obstacle.ry
    vobx = obstacle.vx
    voby = obstacle.vy
    R = robot.radius + obstacle.radius
    # if constraint_val >= 0, no collision , else there will be a collision in the future
    constraint_val = -((rx - obx) * (vrx - vobx) + (ry - oby) * (vry - voby)) ** 2 + ( -R ** 2 + ( rx- obx) ** 2 + (ry - oby) ** 2) * ((vrx - vobx)**2 + (vry - voby)**2)
    return constraint_val

def Collision_check(robot, obstacle):
    collision = np.linalg.norm(obstacle.position - robot.position) < (robot.radius + obstacle.radius)
    if collision:
        return True
    return False

