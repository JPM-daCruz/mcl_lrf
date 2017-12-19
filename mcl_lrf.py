import random
import tf
import numpy as np
import math
import rospy
import threading
import scipy.integrate as integrate
import scipy.special as special
from decimal import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

init=0
old_pose=list()
map_info=[]
n_particulas=500
map_width=324
map_height=323
resolution=0.05
xorigin=-8.2
yorigin=-8.1
alpha1=0.2
alpha2=0.2
alpha3=0.2
alpha4=0.2
laser_min_range=0.0
laser_max_range=8.0
min_angle=0
max_angle=180
pose=(-7.0, -7.0, 45)
z_hit=0.95
z_short=0.1
z_max=0.05
z_rand=0.05
sigma_hit=0.2
lambda_short=0.1

def map_calc_range(ox, oy, oa):
    x0, x1, y0, y1 = 0, 0, 0, 0
    x, y = 0, 0
    xstep, ystep = 0, 0
    tmp = 0
    deltax, deltay, error, deltaerr = 0, 0, 0, 0
    (x0,y0) = xy_to_wh(ox,oy)   
    (x1,y1) = xy_to_wh(ox + laser_max_range * math.cos(oa),oy + laser_max_range * math.sin(oa))
    if (abs(y1 - y0) > abs(x1 - x0)):
        steep = 1
    else:
        steep = 0
    if(steep==1):
        tmp = x0
        x0 = y0
        y0 = tmp
        tmp = x1
        x1 = y1
        y1 = tmp
    deltax = abs(x1-x0)
    deltay = abs(y1-y0)
    error = 0
    deltaerr = deltay
    x = x0
    y = y0
    if(x0 < x1):
        xstep = 1
    else:
        xstep = -1
    if(y0 < y1):
        ystep = 1
    else:
        ystep = -1
    if (steep==1): 
        if valid_map(y, x)==0 or map_info.data[wh_to_map(y, x)] == -1 or map_info.data[wh_to_map(y, x)] == 100:
           return math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution
    else:
        if valid_map(x,y)==0 or map_info.data[wh_to_map(x, y)]== -1 or map_info.data[wh_to_map(x, y)] == 100:
            return math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution
    while x != (x1 + xstep * 1):
        x += xstep
        error += deltaerr
        if (2*error >= deltax) :
            y += ystep
            error -= deltax
        if (steep==1):
            if valid_map(y,x)==0 or map_info.data[wh_to_map(y, x)] == -1 or map_info.data[wh_to_map(y, x)] == 100:
                return math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution
        else:
            if valid_map(x,y)==0 or map_info.data[wh_to_map(x, y)] == -1 or map_info.data[wh_to_map(x, y)] == 100:
                return math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution
    return laser_max_range


def map_to_xy(index):
    (h,w)=divmod(index,map_width)
    x=w*resolution+xorigin
    x=round(x,2)
    y=h*resolution+yorigin
    y=round(y,2)
    return x,y

def map_to_wh(index):
    (h,w)=divmod(index,map_width)
    return w,h

def xy_to_wh(x,y):
    w=math.floor(round((x-xorigin),2)/resolution)
    h=math.floor(round((y-yorigin),2)/resolution)
    return w,h
    
def wh_to_map(w,h):
    index=int(h*(map_width)+w)
    return index
    
def valid_map(w,h):
    r=0
    if((w>=0) and (w < map_width) and (h>=0) and (h<map_height)):
        r=1
    return r


def sample(b):
    b=math.sqrt(b)
    sum1=0
    for i in range(0,12):
        sum1+=random.uniform(-b,b)
    return 0.5*sum1


def normalize(teta):
    return math.atan2(math.sin(teta),math.cos(teta))

def angle_diff(t1,t2):
    t1=normalize(t1)
    t2=normalize(t2)
    d1=t1-t2
    d2=2*math.pi-abs(d1)
    if d1>0:
        d2*=-1.0
    if abs(d1)<abs(d2):
        return d1
    else:
        return d2 
    
def create_samples(M,W,H):
    particulas=list()
    for i in range(0,M):
        x=(random.gauss(-6.92,0.1))
        y=(random.gauss(-6.92,0.1))
        (w,h)=xy_to_wh(x,y)
        while(map_info.data[int(wh_to_map(w,h))]!=0):	
            x=(random.gauss(-6.92,0.1))
            y=(random.gauss(-6.92,0.1))
            (w,h)=xy_to_wh(x,y) 
        teta=(random.gauss(math.pi/4,0.1))
        weight=float(float(1)/float(M))
        n_part=0
	particula=list([x,y,teta,weight,n_part])
	particulas.append(particula)
    return particulas


def create_pose(particula):
    msg=Pose()
    msg.position.x=particula[0]
    msg.position.y=particula[1]
    cy=math.cos((particula[2])*0.5)
    sy=math.sin((particula[2])*0.5)
    cr=math.cos(0)
    sr=math.sin(0)
    cp=math.cos(0)
    sp=math.sin(0)
    msg.orientation.z=sy*cr*cp-cy*sr*sp
    msg.orientation.w=cy*cr*cp+sy*sr*sp
    return msg	    


def sample_motion_model(msg):
    global init
    global old_pose
    if init==0:
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.orientation.z
        w=msg.pose.pose.orientation.w
        (t1,t2,teta)=tf.transformations.euler_from_quaternion((0,0,z,w))
        old_pose=list([x,y,teta])
        init=1
    if init==1:
        (t1,t2,teta_atual)=tf.transformations.euler_from_quaternion((0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
        delta_x=(msg.pose.pose.position.x-old_pose[0])
        delta_y=(msg.pose.pose.position.y-old_pose[1])
        delta_teta=angle_diff(teta_atual,old_pose[2])
        if math.sqrt(delta_x**2+delta_y**2)<0.01:        
	    delta_rot1=0
        else:
            delta_rot1=angle_diff(math.atan2(delta_y,delta_x),old_pose[2])
        delta_trans=math.sqrt(delta_x**2+delta_y**2)
        delta_rot2=angle_diff(delta_teta,delta_rot1)
        delta_rot1_noise=min(abs(angle_diff(delta_rot1,0.0)),abs(angle_diff(delta_rot1,math.pi)))
        delta_rot2_noise=min(abs(angle_diff(delta_rot2,0.0)),abs(angle_diff(delta_rot2,math.pi)))
        group=PoseArray()
        group.header.frame_id="map"
        for i in range(0,n_particulas):       
            delta_rot1_hat=angle_diff(delta_rot1,random.gauss(0.0,alpha1*(delta_rot1_noise**2)+alpha2*(delta_trans**2)))
            delta_trans_hat=delta_trans-random.gauss(0.0,alpha3*(delta_trans**2)+alpha4*(delta_rot1_noise**2)+alpha4*(delta_rot2_noise**2))
            delta_rot2_hat=angle_diff(delta_rot2,random.gauss(0.0,alpha1*(delta_rot2_noise**2)+alpha2*(delta_trans**2)))
      	    particulas[i][0]+=delta_trans_hat*math.cos(particulas[i][2]+delta_rot1_hat)
            particulas[i][1]+=delta_trans_hat*math.sin(particulas[i][2]+delta_rot1_hat)
            particulas[i][2]+=delta_rot1_hat+delta_rot2_hat
            part_array=create_pose(particulas[i])
            group.poses.append(part_array)
        part_array_pub.publish(group)
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.orientation.z
        w=msg.pose.pose.orientation.w
        (t1,t2,teta)=tf.transformations.euler_from_quaternion((0,0,z,w))
        old_pose=list([x,y,teta])
        
    
    
def measurement_model(msg):
    increment=6
    total_weight=0
    pesos=[] 
    k=0   
    teste=0
    particulas[:][0]
    for j in range(0,n_particulas):
        q=1.0
        p=0
        pz=0 
        for i in range(0,len(msg.ranges),increment):
            angle=i-min_angle
            zteste=map_calc_range(particulas[j][0], particulas[j][1], (particulas[j][2]+math.radians(angle)-(math.pi/2)))
            z=msg.ranges[i]-zteste
            pz+=z_hit*math.exp(-(z*z)/(2 * sigma_hit *sigma_hit))     
            if(msg.ranges[i]<laser_max_range):         
                pz+=z_rand*1.0/laser_max_range
            if(z<0):
                p_short=1.0/(1-math.exp(-lambda_short*zteste)      *lambda_short*math.exp(-lambda_short*msg.ranges[i]))
            if(msg.ranges[i]==laser_max_range):
                pz+=z_max*1.0
            p+=pz
            particulas[j][3]=q*p
            total_weight+=particulas[j][3]
    #resampling
    for i in range(0,n_particulas):
        particulas[i][3]=particulas[i][3]/total_weight
        pesos.append(particulas[i][3])
    cum_matrix=np.cumsum(pesos)
    for j in range(0,n_particulas):
		i=0
		x=random.uniform(0,1)
		while x>cum_matrix[i] and i<99:
			i+=1
                particulas[i][4]+=1
                teste+=1
    pteste=particulas
    for i in range(0,n_particulas-1):
        for j in range(0,pteste[i][4]): 
            x=pteste[i][0]
            y=pteste[i][1]
            teta=pteste[i][2]
            weight=float(float(1)/float(n_particulas))
            n_part=0        
	    particula=list([x,y,teta,weight,n_part])
	    particulas[k]=particula
            k+=1     
    print "move"
       


class InfoGetter(object):
    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._msg = None

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._msg


if __name__ == '__main__':
    #initializes a node
    rospy.init_node('particles')
    #Get the map info
    get_map = InfoGetter()
    rospy.Subscriber("map", OccupancyGrid, get_map)
    #ig.get_msg() Blocks until message is received
    map_info = get_map.get_msg()
    #create a variable type PoseArray to represent the particles on Rviz
    group=PoseArray()
    group.header.frame_id="map"
    #randomly create particles only in the free space of the map
    particulas=create_samples(n_particulas,map_width,map_height)
    part_array_pub = rospy.Publisher("partarray", PoseArray, queue_size=1)
    for i in range(0,n_particulas):
        part_array=create_pose(particulas[i])
        group.poses.append(part_array)
    rospy.Subscriber("odom", Odometry, sample_motion_model)
    rospy.Subscriber("base_scan_1", LaserScan, measurement_model)
    rospy.spin()  
