import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import *
import json
import math
import modern_robotics as mr
import matplotlib.pyplot as plt

from Indy7  import *
def flip(lambda_):
	return np.array([lambda_[3],lambda_[4],lambda_[5],lambda_[0],lambda_[1],lambda_[2]])


def getRayFromTo(mouseX, mouseY):
  width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
  )
  camPos = [
      camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
      camTarget[2] - dist * camForward[2]
  ]
  farPlane = 10000
  rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
  invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                      rayForward[1] + rayForward[2] * rayForward[2]))
  rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
  rayFrom = camPos
  oneOverWidth = float(1) / float(width)
  oneOverHeight = float(1) / float(height)
  dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
  dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
  rayToCenter = [
      rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
  ]
  rayTo = [
      rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
      float(mouseY) * dVer[0], rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
      float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayFrom[2] + rayForward[2] -
      0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
  ]
  return rayFrom, rayTo

#--------------------------------------------
## Env Setup
CONTROL_FREQ = 1000.0
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

# READ MR_INFO.JSON
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data
    
json_file_path = 'MR_info.json'
data_from_json = read_json_file(json_file_path)


M=data_from_json['M']
Slist = np.array(data_from_json['baseSlist'])
Blist = np.array(data_from_json['baseBlist'])
Mlist_ = np.array(data_from_json['Mlist'])
Mlist = np.zeros([4,4,7]);
Glist = np.zeros([6,6,6]);
Glist=[]
Mlist=[]
for i in range(Mlist_.shape[0]):
	Mlist.append(Mlist_[i,0:4,0:4]);
Glist_ = np.array(data_from_json['Glist'])

for i in range(Glist_.shape[0]):
	Glist.append(Glist_[i,0:6,0:6]);



print_cnt = 0;


Xd_list=[]
Vd_list=[]
dVd_list=[]
q,qdot = indy7.getJointStates();
T = FKinSpace(M,Slist,q);
X0= T
XT= T@mr.MatrixExp6(mr.VecTose3(np.array([0.1,0.1,0.1,0.1,0.1,0.1])))
V0 = np.zeros(6)
VT = np.zeros(6)
dV0 = np.zeros(6)
dVT = np.zeros(6)
N = int(endTime/dt)
print(XT)
for _ in range(N + 1):
    Xd_list.append(np.zeros((4, 4)))
    Vd_list.append(np.zeros(6))
    dVd_list.append(np.zeros(6))
i = 0;
Xd_list,Vd_list,dVd_list=mr.LieScrewTrajectory(X0,XT,V0,VT,dV0,dVT,1,int(1/dt));
x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
							lineToXYZ=[0, 0, 0],
							lineColorRGB=[1,0,0],
							lineWidth=0.1,
							lifeTime=0)
y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
							lineToXYZ=[0, 0, 0],
							lineColorRGB=[0,1,0],
							lineWidth=0.1,
							lifeTime=0)
z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
							lineToXYZ=[0, 0, 0],
							lineColorRGB=[0,1,0],
							lineWidth=0.1,
							lifeTime=0)     
t = 0
prev_qdot = np.zeros(6)
q,qdot = indy7.getJointStates();
qddot = (qdot-prev_qdot)/dt
T = FKinSpace(M,Slist,q);
Xd=T
Vd = np.zeros(6);
dVd = np.zeros(6);
prev_lambda = np.zeros(6);

time_data = []
V0_data=[]
V1_data=[]
V2_data=[]
V3_data=[]
V4_data=[]
V5_data=[]

dV0_data=[]
dV1_data=[]
dV2_data=[]
dV3_data=[]
dV4_data=[]
dV5_data=[]


fig, ax = plt.subplots(2,1)
ax[0].set_ylim(-1, 1)
Vline0, = ax[0].plot([], [], lw=2)
Vline1, = ax[0].plot([], [], lw=2)
Vline2, = ax[0].plot([], [], lw=2)
Vline3, = ax[0].plot([], [], lw=2)
Vline4, = ax[0].plot([], [], lw=2)
Vline5, = ax[0].plot([], [], lw=2)

dVline0, = ax[1].plot([], [], lw=2)
dVline1, = ax[1].plot([], [], lw=2)
dVline2, = ax[1].plot([], [], lw=2)
dVline3, = ax[1].plot([], [], lw=2)
dVline4, = ax[1].plot([], [], lw=2)
dVline5, = ax[1].plot([], [], lw=2)
ax[0].grid(True)
ax[1].grid(True)
qddot_ref = np.zeros(6)

print(Blist)
clicked_index = 0
while(t<endTime):
	q,qdot = indy7.getJointStates();
	qddot = (qdot-prev_qdot)/dt
	T = FKinSpace(M,Slist,q);
	Js,Jb,Ja,pinvJs,pinvJb, pinvJa = indy7.getJacobian(M,Slist,Blist,q);
	Jb = mr.JacobianBody(Blist,q)
	dJb = dJacobianBody(Jb ,qdot)


	Kp=np.diag([2000,2000,2000,2000,2000,2000])
	Kv=np.diag([200,200,200,200,200,200])
	Hinf_K_gamma = np.array([50/100.0,50/100.0,50/100.0,50/100.0,50/100.0,50/100.0])
	V = Jb@qdot
	#V = Vd
	Xe = TransInv(T)@Xd
	invXe = TransInv(Xe)
	Ve = Vd-Adjoint(invXe)@V
	dV = dJb@qdot + Jb@qddot_ref;
	#dV = dVd;
	dVe = dVd-Adjoint(invXe)@dV + ad(Ve)@Vd
	lambda_ = flip(se3ToVec(MatrixLog6(Xe)))
	dlambda = dlog6(-lambda_)@flip(Ve)
	ddlambda_ref = -Kv@dlambda -Kp@lambda_
	#mouse Event
	mouseEvents = p.getMouseEvents()
	for e in mouseEvents:
		clicked_index = clicked_index+1
		if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
			i=0
			XT= T@mr.MatrixExp6(mr.VecTose3(np.array([(np.random.rand(1)-0.5)*0.2,(np.random.rand(1)-0.5)*0.2,(np.random.rand(1)-0.5)*0.2,(np.random.rand(1)-0.5)*0.2,(np.random.rand(1)-0.5)*0.2,(np.random.rand(1)-0.5)*0.2])))
			
			Xd_list,Vd_list,dVd_list=mr.LieScrewTrajectory(T,XT,(Vd),VT,(dVd),dVT,2,2000);
			data_dict = {
				"X0": T.tolist(),
				"XT": XT.tolist(),
				"V0": Vd.tolist(),
				"VT": VT.tolist(),
				"dV0": dVd.tolist(),
				"dVT": VT.tolist()
			}

			file_path = "numpy_arrays"+str(clicked_index)+".json"
			with open(file_path, 'w') as json_file:
				json.dump(data_dict, json_file)
			#print("V0 : ",V)
			#print("dV0 : ",dV)
			#print("V0_ : ",(Vd_list[0]))
			#print("dV0_ : ",(dVd_list[0]))
			print("Clicked")
	if(i>=len(Xd_list)):
		Xd = Xd_list[-1]
		Vd = flip(Vd_list[-1])
		dVd = flip(dVd_list[-1])
	else:
		Xd = Xd_list[i]
		Vd = flip(Vd_list[i])
		dVd = flip(dVd_list[i])
	dV_ref = Adjoint(Xe)@(dVd - flip(dexp6(-lambda_)@ddlambda_ref) + ad(Ve)@Vd - flip(ddexp6(-lambda_,-dlambda)@dlambda))
	qddot_ref = Jb.T@np.linalg.inv(Jb@Jb.T+np.eye(6)*0.001)@(dV_ref-dJb@qdot)
	tau =mr.InverseDynamics(q, qdot, qddot_ref, g, Ftip, Mlist,  Glist, Slist,0);	

	#print
	print_cnt =print_cnt+1
	if(print_cnt>100):
		print_cnt = 0
		#print(dlambda)
		#print((lambda_-prev_lambda)/dt)
		ax[0].set_xlim(np.min(time_data), np.max(time_data))
		ax[1].set_xlim(np.min(time_data), np.max(time_data))
		ax[0].set_ylim(np.min(V0_data), np.max(V0_data))
		ax[1].set_ylim(np.min(dV0_data), np.max(dV0_data))
		Vline0.set_data(time_data, V0_data)		
		Vline1.set_data(time_data, V1_data)		
		Vline2.set_data(time_data, V2_data)		
		Vline3.set_data(time_data, V3_data)		
		Vline4.set_data(time_data, V4_data)		
		Vline5.set_data(time_data, V5_data)		
		dVline0.set_data(time_data, dV0_data)		
		dVline1.set_data(time_data, dV1_data)		
		dVline2.set_data(time_data, dV2_data)		
		dVline3.set_data(time_data, dV3_data)		
		dVline4.set_data(time_data, dV4_data)		
		dVline5.set_data(time_data, dV5_data)		
		plt.pause(dt);
	prev_lambda = lambda_

	i = i+1;
	prev_qdot = qdot;
	indy7.drawT(XT,0.1,x_lineId,y_lineId,z_lineId,1)
	indy7.setTorques(tau)
	indy7.drawEEF();
	indy7.step()
	time.sleep(dt)
	t = t+dt;
	time_data.append(t)
	V0_data.append(Vd[0])
	V1_data.append(Vd[1])
	V2_data.append(Vd[2])
	V3_data.append(Vd[3])
	V4_data.append(Vd[4])
	V5_data.append(Vd[5])
	dV0_data.append(dVd[0])
	dV1_data.append(dVd[1])
	dV2_data.append(dVd[2])
	dV3_data.append(dVd[3])
	dV4_data.append(dVd[4])
	dV5_data.append(dVd[5])	
	if(len(time_data)>1000):
		time_data=time_data[1:-1]
		V0_data=V0_data[1:-1]
		V1_data=V1_data[1:-1]
		V2_data=V2_data[1:-1]
		V3_data=V3_data[1:-1]
		V4_data=V4_data[1:-1]
		V5_data=V5_data[1:-1]
		dV0_data=dV0_data[1:-1]
		dV1_data=dV1_data[1:-1]
		dV2_data=dV2_data[1:-1]
		dV3_data=dV3_data[1:-1]
		dV4_data=dV4_data[1:-1]
		dV5_data=dV5_data[1:-1]
	
	
exit(0)
	

