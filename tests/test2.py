import pybullet as p
from Indy7  import *
#--------------------------------------------
## Env Setup
CONTROL_FREQ = 100.0
endTime = 50;
dt = 1/CONTROL_FREQ;
urdf_name = "model/indy7.urdf"
indy7 = Indy7(p,urdf_name = urdf_name, CONTROL_FREQ=CONTROL_FREQ)
indy7.resetJointStates(np.array([ 0.   ,  0,-np.pi/2.0,0,-np.pi/2.0,0]).T);
p.loadURDF("model/plane.urdf")
p.loadURDF("model/axis.urdf")
qddot = np.zeros([6,1])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]
t = 0
while(1):
	q,qdot = indy7.getJointStates();
	tau = np.array([100,100,100,100,100,100]);
	indy7.setTorques(tau)
	qT = np.array([np.sin(2*np.pi*t),np.sin(2*np.pi*t),np.sin(2*np.pi*t),np.sin(2*np.pi*t),np.sin(2*np.pi*t),np.sin(2*np.pi*t)]);
	indy7.setJointStates(qT);
	t = t+dt;
#	indy7.step();
