import numpy as np

def calcTrajectory(start, stop, num):
    X, Y, Z = np.mgrid[start[0]:stop[0]:num, start[1]:stop[1]:num\
        , start[2]:stop[2]:num]
    return np.vstack((X.flatten(), Y.flatten(), Z.flatten())).T

def calcTrajAngles(traj, param):
    _, i = shape(traj)
    

def inverseKinematics(r, param):
    # parameters
    x = r[0]
    y = r[1]
    z = r[2]
    t1 = 0
    t2 = 0
    t3 = 0
    pi = Math.pi
    # calculations
    t1, s = calcInverse(r, param)
    newR = np.array([x*Math.cos(120*pi/180) + y*Math.sin(120*pi/180)\
            , y*Math.cos(120*pi/180)-x*Math.sin(120*pi/180), z])
    if (s == 0):
        t2, s = calcInverse(r2 ,param)
    if (s == 0):
        t3, s = calcInverse(r2 ,param)
    if (s == 0):
        return np.array([t1,t2,t3])
    else:
        return np.array([0, 0, 0])
    
def calcInverse(r, param):
    theta = np.array([0, 0, 0])
    # parameters
    x = r[0]
    y = r[1]
    z = r[2]
    #rod length
    r_f = param[0]
    r_e = param[1]
    #triangular side length:
    f = param[3]
    e = param[4]
    #inverse kinematics
    y1 = -f/(2*sqrt(3))
    k = e/(2*sqrt(3))
    y = y-k

    a=(x**2 + y**2 + z**2 +r_f**2 - r_e**2 - y1**2)/(2*z)
    b=(y1-y)/z

    d=-(a+b*y1)**2+r_f*(b^2*r_f+r_f); 
    if(d<0):
        print("Pose not in range! Choose other pose that is not a singularity!")
        s=1
    else:
        yj = (y1-a*b-sqrt(d))/(b**2 + 1)
        zj = a + b*yj
        theta = 180*math.atan(-zj/(y1-yj))/math.pi
        if (yj>y1):
            theta=theta+180
        s=0
    return theta, s

def forwardKinematics(t1, t2, t3, param):
