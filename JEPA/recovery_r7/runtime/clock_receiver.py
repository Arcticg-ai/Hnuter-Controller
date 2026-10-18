"""Dedicated clock receiver process: parsing/JSON/ULog cannot hold its Python GIL."""
import multiprocessing as mp,time,json

def receive(shared,stop,path):
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    rows=[]
    def callback(msg):
        ns=int(msg.sim.sec)*10**9+int(msg.sim.nsec);now=time.monotonic_ns()
        with shared.get_lock():
            if shared[2] and ns<shared[0]:shared[3]+=1
            shared[0]=ns;shared[1]=now;shared[2]+=1
        rows.append((ns,now))
    node=Node();ok=node.subscribe(Clock,'/world/default/clock',callback)
    with shared.get_lock():shared[4]=int(ok)
    stop.wait(180)
    if path:
        with open(path,'w') as f:
            f.write('source_ns,receive_monotonic_ns\n');f.writelines(f'{a},{b}\n' for a,b in rows)

class ClockReceiver:
    def __init__(self,path=None):
        self.ctx=mp.get_context('spawn');self.shared=self.ctx.Array('q',5);self.stop=self.ctx.Event();self.proc=self.ctx.Process(target=receive,args=(self.shared,self.stop,str(path) if path else None));self.proc.start()
    def sample(self):
        with self.shared.get_lock():ns,wall,count,rev,sub=list(self.shared)
        now=time.monotonic_ns();return dict(ns=ns,wall_ns=wall,count=count,reversals=rev,subscribed=bool(sub),fresh=bool(count and sub and not rev and 0<=now-wall<200_000_000))
    @property
    def ns(self):
        s=self.sample()
        return s['ns'] if s['fresh'] else None
    def after(self,computation_end_ns,timeout=.2):
        # Require a real new receipt after heavy work. No extrapolated/future clock.
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            s=self.sample()
            if s['fresh'] and s['wall_ns']>=computation_end_ns:return s
            time.sleep(.001)
        raise TimeoutError('no fresh post-computation clock receipt')
    def close(self):
        self.stop.set();self.proc.join(5)
        if self.proc.is_alive():self.proc.terminate();self.proc.join(2);raise RuntimeError('clock receiver shutdown timeout')
