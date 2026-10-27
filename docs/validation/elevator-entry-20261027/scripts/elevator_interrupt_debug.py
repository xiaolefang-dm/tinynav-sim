exec(open('/runtime/elevator_live_matrix.py').read().split('try:asyncio.run(run())')[0])
async def check():
 await waitfor(lambda:pose,15)
 req('/nav/runtime-mode',{'mode':'off'});await asyncio.sleep(2)
 req('/command',{'action':'open'},8092);await waitfor(lambda:req('/status',port=8092)['door_open'],15)
 await position(1.25,0,0)
 req('/command',{'action':'close'},8092);await waitfor(lambda:req('/status',port=8092)['door_fraction']==0,15)
 req('/nav/runtime-mode',{'mode':'elevator_manual'});await waitfor(lambda:req('/device/status').get('navModeRunning')=='elevator_manual',45)
 emit(event='BEFORE_OPEN',device=req('/device/status'),lift=req('/status',port=8092));emit(event='OPEN_RESPONSE',response=req('/nav/elevator/command',{'action':'open'}))

 for i in range(250):
  d=req('/device/status');ls=req('/status',port=8092)
  if i%5==0:emit(event='WAIT_ENTER',device=d,lift=ls)
  if d['elevatorEntry']['stage']=='entering':break
  await asyncio.sleep(.2)
 else:raise RuntimeError('wait entering timeout')
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
try:asyncio.run(check())
finally:
 rclpy.shutdown();spin_thread.join(timeout=3);node.destroy_node()
