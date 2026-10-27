"""Live automatic entry, floor change and reverse exit through Pilot HTTP.

Run inside the Pilot container with its ROS environment. Requires an idle robot
in the elevator scene, front/rear depth and the simulator on localhost:8092.
This test moves the robot and doors. JSON lines record stages and velocity checks.
"""
import asyncio,json,math,time,urllib.request,threading
import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
import websockets
rclpy.init(); node=rclpy.create_node('elevator_validation'); pose=None
def on_pose(m):
 global pose
 for t in m.transforms:
  if t.child_frame_id=='go2':
   p,q=t.transform.translation,t.transform.rotation
   pose=(p.x,p.y,2*math.atan2(q.z,q.w),time.monotonic())
node.create_subscription(TFMessage,'/sim/world_pose',on_pose,qos_profile_sensor_data)
spin_thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);spin_thread.start()
def req(path,body=None,port=8000):
 data=None if body is None else json.dumps(body).encode()
 return json.load(urllib.request.urlopen(urllib.request.Request(f'http://127.0.0.1:{port}'+path,data=data,headers={'Content-Type':'application/json'}),timeout=4))
def emit(**kw):print(json.dumps(kw),flush=True)
async def waitfor(fn,timeout=100):
 start=time.monotonic()
 while time.monotonic()-start<timeout:
  v=fn()
  if v:return v
  await asyncio.sleep(.2)
 raise RuntimeError('timeout')
async def position(x,y,yaw):
 if pose and pose[0]>2.65 and (y!=0 or yaw!=0):
  await position(1.25,0,0)
 async with websockets.connect('ws://127.0.0.1:8000/ws/teleop') as ws:
  start=time.monotonic()
  while time.monotonic()-start<65:
   assert pose and time.monotonic()-pose[3]<1,'stale pose'
   px,py,a,_=pose; dx,dy=x-px,y-py; da=math.atan2(math.sin(yaw-a),math.cos(yaw-a))
   if math.hypot(dx,dy)<.035 and abs(da)<.025:break
   vx=max(-.45,min(.45,1.5*(math.cos(a)*dx+math.sin(a)*dy)))
   vy=max(-.45,min(.45,1.5*(-math.sin(a)*dx+math.cos(a)*dy)))
   await ws.send(json.dumps(dict(linear_x=vx,linear_y=vy,angular_z=max(-.6,min(.6,da*2)))))
   await asyncio.sleep(.06)
  else:raise RuntimeError('position timeout')
  await ws.send(json.dumps(dict(linear_x=0,linear_y=0,angular_z=0)))
 await asyncio.sleep(1)
from geometry_msgs.msg import Twist
commands=[]
node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),10)
async def stage(target,timeout=100):
 start=time.monotonic();last=None
 while time.monotonic()-start<timeout:
  d=req('/device/status')['elevatorEntry'];s=req('/status',port=8092)
  if d['stage']!=last:emit(event='STAGE',entry=d,lift=s,pose=pose[:3]);last=d['stage']
  if d['stage']=='stopped':raise RuntimeError(d['error'])
  if d['stage']==target:return s
  await asyncio.sleep(.2)
 raise RuntimeError('stage timeout '+str(req('/device/status')))
async def run_exit():
 await waitfor(lambda:pose,15)
 destination = 1-req('/status',port=8092)['floor']
 req('/nav/runtime-mode',{'mode':'off'});await asyncio.sleep(2)
 req('/nav/elevator/operation',{'operation':'entry'})
 req('/command',{'action':'open'},8092);await waitfor(lambda:req('/status',port=8092)['door_open'],15)
 await position(1.25,0,0)
 req('/command',{'action':'close'},8092);await waitfor(lambda:req('/status',port=8092)['door_fraction']==0,15)
 req('/nav/runtime-mode',{'mode':'elevator_manual'});await waitfor(lambda:req('/device/status').get('navModeRunning')=='elevator_manual',45)
 await asyncio.sleep(2);req('/nav/elevator/command',{'action':'open'})
 await stage('inside_door_closed')
 req('/nav/elevator/command',{'action':'floor','floor':destination})
 await waitfor(lambda:req('/status',port=8092)['floor']==destination and req('/status',port=8092)['motion']=='idle',45)
 req('/nav/elevator/operation',{'operation':'exit'})
 await stage('waiting_for_open_door',15)
 before=req('/status',port=8092)['robot'];await asyncio.sleep(2);assert math.dist(before,req('/status',port=8092)['robot'])<.02
 started=time.monotonic();req('/nav/elevator/command',{'action':'open'})
 end=await stage('outside_door_closed',120)
 moving=[c for c in commands if c[0]>started and abs(c[1])>.01]
 assert moving and all(c[1]<0 for c in moving),moving[-10:]
 assert end['robot'][0]<2.1 and abs(end['robot'][2]-3*destination)<.08 and end['door_fraction']==0,end
 emit(event='AUTO_EXIT_PASS',lift=end,pose=pose[:3],velocity_range=[min(c[1] for c in moving),max(c[1] for c in moving)])
try:asyncio.run(run_exit())
except Exception as e:
 emit(event='FAIL',error=str(e),device=req('/device/status'),lift=req('/status',port=8092));req('/nav/elevator/stop',{});raise
finally:rclpy.shutdown();spin_thread.join(timeout=3);node.destroy_node()
