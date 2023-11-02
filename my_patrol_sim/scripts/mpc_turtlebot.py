import rospy
import copy
import tf
import math
import numpy as np
from scipy import spatial
from scipy import optimize
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt

plt.ion()

wps = np.loadtxt("wps.txt")
x = wps[:,0]
y = wps[:,1]
t = np.linspace(0, 1, num=len(x))
f1 = interp1d(t,x,kind='cubic')
f2 = interp1d(t,y,kind='cubic')
newt = np.linspace(0,1,100)
nwps = np.zeros((100, 2))
nwps[:,0] = f1(newt)
nwps[:,1] = f2(newt)
wpstree = spatial.KDTree(nwps)

def getcwps(rp):
    _, nindex = wpstree.query(rp)
    cwps = np.zeros((5,2))
    for i in range(5):
        cwps[i] = nwps[(nindex+i)%len(nwps)]
        
#     if (nindex + 5) >= 100:
#         cwps[0:100-nindex-1] = nwps[nindex:-1]
#         cwps[100-nindex-1:-1] = nwps[0:nindex+5-100]        
#     else:
#         cwps = nwps[nindex:nindex+5]
    return cwps    

def cubic_fun(coeffs, x):
    return coeffs[0]*x**3+coeffs[1]*x**2+coeffs[2]*x+coeffs[3]    
        
def plot_durations(cwps, prex, prey):
    plt.figure(2)
    plt.clf()
    plt.plot(cwps[:,0],cwps[:,1])
    plt.plot(prex, prey)
    plt.scatter(x, y)


N = 19  # forward predict steps
ns = 5  # state numbers / here: 1: x, 2: y, 3: psi, 4: cte, 5: epsi
na = 2  # actuator numbers /here: 1: steering angle, 2: omega


class MPC(object):
    def __init__(self):
        # MPC parameter
        self.forward_step = 4


    @staticmethod
    def vehicle_kinematics_model(current_x, current_y, current_yaw, velocity, steer_angle):
        dt = 4.0
        car_l = 4.0

        next_x = current_x - velocity * math.cos(current_yaw) * dt
        next_y = current_y + velocity * math.sin(current_yaw) * dt
        next_yaw = current_yaw + velocity * math.sin(steer_angle) * dt / car_l

        return next_x, next_y, next_yaw


    def MPC_forward(self, cmd_list):
        start_x = 65
        start_y = 48
        start_yaw = 0

        predict_x = []
        predict_y = []

        current_x = start_x
        current_y = start_y
        current_yaw = start_yaw

        for i in range(int(len(cmd_list) / 2)):
            cmd_v, cmd_s = cmd_list[2 * i], cmd_list[2 * i + 1]
            next_x, next_y, next_yaw = self.vehicle_kinematics_model(current_x, current_y, current_yaw, cmd_v, cmd_s)
            predict_x.append(next_x)
            predict_y.append(next_y)
            current_x = next_x
            current_y = next_y
            current_yaw = next_yaw

        return predict_x, predict_y


    def MPC_optimize_func(self, cmd_list):
        # cmd_list = [v0, s0, v1, s1, ......]
        pre_x, pre_y = self.MPC_forward(cmd_list)

        ref_error = 0
        sum_s = 0
        sum_centripetal_acc = 0
        sum_acc = 0
        sum_df = 0

        for i in range(int(len(cmd_list) / 2) - 1):
            sum_acc += abs(cmd_list[2 * i] - cmd_list[2 * i + 2])
            sum_df += abs(cmd_list[2 * i + 1] - cmd_list[2 * i + 3])

        last_x, last_y = 65, 48
        for i, [x, y] in enumerate(zip(pre_x, pre_y)):
            distance, _ = self.ref_kd_tree.query([x, y], k=1)
            ref_error += distance
            sum_s += math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
            sum_centripetal_acc += cmd_list[2 * i] * math.tan(cmd_list[2 * i + 1])
            last_x, last_y = x, y

        return 1 * ref_error - 1 * sum_s + 0.01 * sum_centripetal_acc + 300 * sum_acc + 10 * sum_df


    def MPC_solve(self, state, coeffs):
        # scipy.optimize.minimize can only optimize one-dimensional vectors
        init_cmd = [2.0, 0.0]
        cmd_bound = [(0.5, 5), (-math.pi / 4, math.pi / 4)]
        cmd_0 = np.array([init_cmd[i % 2] for i in range(2 * self.forward_step)])
        bound = [cmd_bound[i % 2] for i in range(2 * self.forward_step)]

        test = optimize.minimize(self.MPC_optimize_func, cmd_0,
                                 bounds=bound,
                                 method='L-BFGS-B',
                                 options={'maxiter': 6})

        x_pred_vals = [self.iN.s[0,k]() for k in self.iN.sk]
        y_pred_vals = [self.iN.s[1,k]() for k in self.iN.sk]

        pre_path = np.zeros((self.forward_step, 2))
        pre_path[:,0] = np.array(x_pred_vals)
        pre_path[:,1] = np.array(y_pred_vals)        
        v = test.x[0]
        w = test.x[1]
        return pre_path, v, w


class Turtlebot_core():
    def __init__(self):
        rospy.init_node("Turtlebot_core", anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/robot_1/odom", Odometry, self.odomCallback)
        self.pub_refpath = rospy.Publisher("/ref_path", Path, queue_size=1)
        self.pub_prepath = rospy.Publisher("/pre_path", Path, queue_size=1)
        self.pub_cmd = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size=1)
        self.rp = np.zeros(3)
        self.crv = 0.0
        self.crw = 0.0
        self.mpc = MPC() 
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.getrobotpose()  # update self.rp
            cwps = getcwps(self.rp[0:2])
            # self.crv and self.crw are updated in odomCallback
            px = self.rp[0] + self.crv*np.cos(self.rp[2])*0.1
            py = self.rp[1] + self.crw*np.sin(self.rp[2])*0.1
            psi = self.rp[2] + self.crw*0.1
            
            self.rp[0] = px
            self.rp[1] = py
            self.rp[2] = psi
            
            cwps_robot = np.zeros((len(cwps), 2))
            
            for i in range(len(cwps)):
                dx = cwps[i,0] - px
                dy = cwps[i,1] - py
                
                cwps_robot[i,0] = dx*np.cos(psi) + dy*np.sin(psi)
                cwps_robot[i,1] = dy*np.cos(psi) - dx*np.sin(psi)
                
            coeffs = np.polyfit(cwps_robot[:,0], cwps_robot[:,1], 3)
            cte = cubic_fun(coeffs, 0)
            
            f_prime_x = coeffs[2]
            epsi = np.arctan(f_prime_x)
            s0 = np.array([0.0, 0.0, 0.0, cte, epsi])


            pre_path, v, w = self.mpc.MPC_solve(s0, coeffs)

            self.pub_ref_path(cwps_robot)
            self.pub_pre_path(pre_path)
            self.pub_Command(v, w)
            print(v,w)
#             plot_durations(cwps, x_pred_vals, y_pred_vals)
            rate.sleep()        
        rospy.spin()            
            
    def getrobotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot_1/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return   
        self.rp[0] = trans[0]
        self.rp[1] = trans[1]
        r,p,y = tf.transformations.euler_from_quaternion(rot)
        self.rp[2] = y
        
    def odomCallback(self, data):
        self.crv = data.twist.twist.linear.x
        self.crw = data.twist.twist.angular.z
        
    def pub_ref_path(self, ref_path):        
        msg_ref_path = Path()
        msg_ref_path.header.stamp = rospy.Time.now()
        msg_ref_path.header.frame_id = "base_link"
        for i in range(len(ref_path)):
            pose = PoseStamped()
            pose.pose.position.x = ref_path[i,0]
            pose.pose.position.y = ref_path[i,1]
            msg_ref_path.poses.append(copy.deepcopy(pose))
            
        self.pub_refpath.publish(msg_ref_path)
        
    def pub_pre_path(self, pre_path):
        msg_pre_path = Path()
        msg_pre_path.header.stamp = rospy.Time.now()
        msg_pre_path.header.frame_id = "base_link"
        for i in range(len(pre_path)):
            pose = PoseStamped()
            pose.pose.position.x = pre_path[i,0]
            pose.pose.position.y = pre_path[i,1]
            msg_pre_path.poses.append(copy.deepcopy(pose))    
        self.pub_prepath.publish(msg_pre_path)
        
    def pub_Command(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)


if __name__ == "__main__":
    turtlebot_core = Turtlebot_core()
