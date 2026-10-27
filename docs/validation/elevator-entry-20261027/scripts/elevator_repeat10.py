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
async def matrix_round():
 await waitfor(lambda:pose,15)
 for name,y,angle in [('center',0,0),('left',.35,0),('right',-.35,0),('angled',.20,20)]:
  req('/nav/runtime-mode',{'mode':'off'});await asyncio.sleep(2)
  req('/command',{'action':'open'},8092); await waitfor(lambda:req('/status',port=8092)['door_open'],15)
  await position(1.25,y,math.radians(angle))
  req('/command',{'action':'close'},8092);await waitfor(lambda:req('/status',port=8092)['door_fraction']==0,15)
  req('/nav/runtime-mode',{'mode':'elevator_manual'})
  await waitfor(lambda:req('/device/status').get('navModeRunning')=='elevator_manual',45)
  await asyncio.sleep(2)
  before=req('/status',port=8092)['robot'];await asyncio.sleep(2)
  after=req('/status',port=8092)['robot']; assert math.dist(before,after)<.03,'moved through closed door'
  emit(case=name,event='start',pose=pose[:3])
  req('/nav/elevator/command',{'action':'open'})
  start=time.monotonic();previous=None;events=[]
  while time.monotonic()-start<140:
   d=req('/device/status');entry=d['elevatorEntry'];stage=entry['stage'];s=req('/status',port=8092)
   if stage!=previous:
    emit(case=name,event='stage',stage=stage,elapsed=round(time.monotonic()-start,1),robot=s['robot'],error=entry['error']);events.append(stage);previous=stage
   if stage=='stopped':raise RuntimeError(name+': '+entry['error'])
   if stage=='inside_door_closed':
    assert s['door_fraction']==0 and 3.6<s['robot'][0]<5.4 and abs(s['robot'][1])<.35
    await asyncio.sleep(2);assert math.dist(s['robot'],req('/status',port=8092)['robot'])<.02
    emit(case=name,event='PASS',events=events,robot=s['robot']);break
   await asyncio.sleep(.25)
  else:raise RuntimeError(name+' entry timeout')
 req('/nav/runtime-mode',{'mode':'off'})
 emit(event='ROUND_PASS')

async def check():
 await waitfor(lambda:pose,15)
 req('/nav/runtime-mode',{'mode':'off'});await asyncio.sleep(2)
 req('/command',{'action':'open'},8092);await waitfor(lambda:req('/status',port=8092)['door_open'],15)
 await position(1.25,0,0)
 req('/command',{'action':'close'},8092);await waitfor(lambda:req('/status',port=8092)['door_fraction']==0,15)
 req('/nav/runtime-mode',{'mode':'elevator_manual'});await waitfor(lambda:req('/device/status').get('navModeRunning')=='elevator_manual',45)
 await asyncio.sleep(2)
 emit(event='SAFETY_START',device=req('/device/status'),lift=req('/status',port=8092))
 emit(event='SAFETY_OPEN',response=req('/nav/elevator/command',{'action':'open'}))
 await waitfor(lambda:req('/device/status')['elevatorEntry']['stage']=='entering',50)
 req('/command',{'action':'close'},8092)
 await waitfor(lambda:req('/device/status')['elevatorEntry']['stage']=='stopped',10)
 await asyncio.sleep(1);a=req('/status',port=8092)['robot'];await asyncio.sleep(3);b=req('/status',port=8092)['robot']
 assert math.dist(a,b)<.02,(a,b)
 emit(event='DOOR_INTERRUPTION_PASS',robot=b,status=req('/device/status')['elevatorEntry'])
 req('/command',{'action':'open'},8092);await waitfor(lambda:req('/status',port=8092)['door_open'],15)
 await asyncio.sleep(3);assert req('/device/status')['elevatorEntry']['stage']=='stopped'
 assert math.dist(b,req('/status',port=8092)['robot'])<.02
 emit(event='NO_UNEXPECTED_RETRY_PASS')
 req('/nav/runtime-mode',{'mode':'off'});await asyncio.sleep(2)
 await position(1.25,0,0)
 req('/command',{'action':'close'},8092);await waitfor(lambda:req('/status',port=8092)['door_fraction']==0,15)
 emit(event='READY_FOR_USER',robot=req('/status',port=8092)['robot'])

async def repeat():
 for iteration in range(2,11):
  emit(event='ROUND_START',iteration=iteration)
  await matrix_round()
  await check()
  emit(event='ITERATION_PASS',iteration=iteration)
 emit(event='ALL_10_ROUNDS_PASS')
try:asyncio.run(repeat())
except Exception as e:
 emit(event='FAIL',error=str(e),device=req('/device/status'),lift=req('/status',port=8092))
 req('/nav/elevator/stop',{});req('/nav/runtime-mode',{'mode':'off'})
 raise
finally:
 rclpy.shutdown();spin_thread.join(timeout=3);node.destroy_node()
