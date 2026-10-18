"""One bounded writer process for independent log streams; callbacks only enqueue immutable records."""
import json,multiprocessing as mp,queue,time,os
from pathlib import Path

def worker(q,out):
 files={};count=0;max_delay=0.;last=time.monotonic()
 try:
  while True:
   item=q.get()
   if item is None:break
   name,kind,data,stamp=item
   if name not in files:files[name]=(Path(out)/name).open('w',buffering=262144)
   max_delay=max(max_delay,(time.monotonic_ns()-stamp)/1e6)
   text=data if kind=='text' else json.dumps(data,default=lambda x:x.item(),allow_nan=True)+'\n'
   files[name].write(text);count+=1
   if time.monotonic()-last>.05:
    for f in files.values():f.flush()
    last=time.monotonic()
  for f in files.values():f.flush();os.fsync(f.fileno());f.close()
  result=dict(passed=True,written=count,max_queue_delay_ms=max_delay)
 except BaseException as e:result=dict(passed=False,written=count,error=repr(e))
 (Path(out)/'async_log_result.json').write_text(json.dumps(result,indent=2))

class LogMux:
 def __init__(self,out):
  self.ctx=mp.get_context('spawn');self.q=self.ctx.Queue(8192);self.failed=False;self.max_backlog=0;self.count=0;self.out=Path(out);self.p=self.ctx.Process(target=worker,args=(self.q,str(out)));self.p.start()
 def stream(self,name):return Stream(self,name)
 def put(self,name,kind,data):
  if not self.p.is_alive():self.failed=True;raise RuntimeError('async_log_worker_dead')
  try:self.q.put_nowait((name,kind,data,time.monotonic_ns()));self.count+=1;self.max_backlog=max(self.max_backlog,self.q.qsize())
  except queue.Full:self.failed=True;raise RuntimeError('async_log_overflow')
 def close(self):
  if self.p.is_alive():
   try:self.q.put(None,timeout=2)
   except queue.Full:self.failed=True
  self.p.join(10)
  if self.p.is_alive():self.p.terminate();self.p.join(2);self.failed=True
  self.q.close()
  if self.failed:self.q.cancel_join_thread()
  else:self.q.join_thread()
  result=json.loads((self.out/'async_log_result.json').read_text()) if (self.out/'async_log_result.json').exists() else {'passed':False}
  result.update(enqueued=self.count,max_backlog=self.max_backlog);result['passed']=result['passed'] and not self.failed and result['written']==self.count
  (self.out/'async_log_result.json').write_text(json.dumps(result,indent=2));return result
class Stream:
 def __init__(self,owner,name):self.owner=owner;self.name=name
 def submit(self,record):self.owner.put(self.name,'json',record)
 def write(self,text):self.owner.put(self.name,'text',text)
 def close(self):pass
