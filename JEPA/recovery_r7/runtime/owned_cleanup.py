"""Bounded cleanup of Popen process groups created by this launcher only."""
import json,os,signal,subprocess,time,threading
from pathlib import Path

def cleanup_owned(procs, timeout=3.):
    events=[]
    def event(name, action, error=None):
        events.append(dict(name=name,action=action,monotonic_ns=time.monotonic_ns(),error=error))
    node=procs.get('controller')
    if node is not None and node.poll() is None:
        try:
            os.killpg(node.pid,signal.SIGINT);event('controller','SIGINT_for_result_flush')
            node.wait(timeout=timeout)
        except (ProcessLookupError,subprocess.TimeoutExpired,OSError) as e:event('controller','interrupt_wait',str(e))
    px=procs.get('px4')
    if px is not None and px.poll() is None:
        try:
            # Close only the audit stream; retain the bridge clock for normal PX4 shutdown.
            px.stdin.write('gz_bridge gps_audit_close\nshutdown\n');px.stdin.flush();event('px4','audit_close_then_shutdown_request')
            px.wait(timeout=max(timeout, 7.))
        except (BrokenPipeError,subprocess.TimeoutExpired,OSError,ValueError) as e:event('px4','shutdown_wait',str(e))
    for name,proc in reversed(list(procs.items())):
        try:os.killpg(proc.pid,signal.SIGTERM);event(name,'SIGTERM_owned_group')
        except ProcessLookupError:event(name,'group_already_exited')
        except OSError as e:event(name,'SIGTERM_error',str(e))
    for name,proc in procs.items():
        try:proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL);event(name,'SIGKILL_owned_group')
            except ProcessLookupError:pass
            except OSError as e:event(name,'SIGKILL_error',str(e))
            try:proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:event(name,'unreaped_after_kill')
    return dict(events=events,processes={n:p.poll() for n,p in procs.items()},
                all_launched_parents_exited=all(p.poll() is not None for p in procs.values()),
                control_result_inferred_from_cleanup=False)

class RunMonitor:
    def __init__(self,path,procs):
        self.path=Path(path);self.procs=procs;self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        with self.path.open('w',buffering=1) as f:
            while not self.stop_event.is_set():
                r=dict(monotonic_ns=time.monotonic_ns(),boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),wall_ns=time.time_ns(),processes={n:p.poll() for n,p in list(self.procs.items())})
                for name in ['cpu.stat','memory.current','cpu.max']:
                    try:r[name]=(Path('/sys/fs/cgroup')/name).read_text().strip()
                    except OSError:r[name]=None
                f.write(json.dumps(r)+'\n');self.stop_event.wait(.25)
    def stop(self):self.stop_event.set();self.thread.join(timeout=2)
