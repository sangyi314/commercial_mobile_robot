"""Optional GUI entry into the ORIGINAL run_trial.main; frozen runner stays byte-identical.

Only scoped adapter classes are substituted while that same main owns the trial.
No launch, recorder, action, pause, result or cleanup lifecycle is duplicated here.
"""
import argparse,contextlib,io,json,sys,time
from pathlib import Path
import run_trial as runner
from runner_core import DeadlineBudget
from runner_gui import GUIOptions,GUIClient

@contextlib.contextmanager
def gui_adapters(options):
 BaseSupervisor=runner.ProcessSupervisor;BaseLink=runner.WorkerLink;session={'gui':None}
 class Supervisor(BaseSupervisor):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,**kwargs);session['gui']=GUIClient(self,options,self.emit)
  def spawn(self,role,argv,env,cwd,**kwargs):
   item=super().spawn(role,argv,env,cwd,**kwargs)
   if role=='gazebo':session['gui'].start(env,cwd)
   return item
  def close(self,item,budget,**kwargs):
   gui=session['gui']
   if item.role in ('gazebo','navigation','worker'):gui.close(budget)
   return super().close(item,budget,**kwargs)
 class Link(BaseLink):
  def pump(self,*args,**kwargs):
   gui=session['gui']
   if self.item.role=='worker':gui.tick()
   return super().pump(*args,**kwargs)
  def rpc(self,op,data,budget):
   gui=session['gui']
   if op in ('manager','shutdown_actual') and data.get('command')=='SHUTDOWN':gui.close(budget)
   value=super().rpc(op,data,budget)
   if self.item.role=='worker' and op=='nav_ready':
    gui.await_ready(self)
    # Independent wait deadline; never extend the original run work limit.
    profile=json.loads((gui.run/'artifacts/profile.json').read_text())
    remaining=DeadlineBudget(session['entry']+profile['budgets']['work'])
    gui.await_operator(self,remaining)
   return value
 session['entry']=time.monotonic();runner.ProcessSupervisor=Supervisor;runner.WorkerLink=Link
 try:yield session
 finally:runner.ProcessSupervisor=BaseSupervisor;runner.WorkerLink=BaseLink

def main(argv=None):
 parser=argparse.ArgumentParser(description=__doc__,epilog='All original --scenario/--profile/--method/--run-id/--check/fault options are forwarded to run_trial unchanged.')
 parser.add_argument('--gazebo-gui',action=argparse.BooleanOptionalAction,default=False)
 parser.add_argument('--wait-for-operator',action=argparse.BooleanOptionalAction,default=False)
 parser.add_argument('--operator-timeout',type=float,default=60.,metavar='SECONDS')
 parser.add_argument('--gui-startup-timeout',type=float,default=20.,metavar='SECONDS')
 args,forwarded=parser.parse_known_args(argv);saved=sys.argv
 try:
  options=GUIOptions(args.gazebo_gui,args.wait_for_operator,args.operator_timeout,args.gui_startup_timeout).validate();sys.argv=[str(Path(runner.__file__)),*forwarded]
  if '--check' in forwarded:
   output=io.StringIO()
   with contextlib.redirect_stdout(output):code=runner.main()
   if code==0:
    plan=json.loads(output.getvalue());plan['observation']=options.plan();plan['processes']+=options.plan()['process_roles'];print(json.dumps(plan,ensure_ascii=False))
   return code
  if not options.gazebo_gui:return runner.main()
  if options.wait_for_operator and not sys.stdin.isatty():raise ValueError('wait_for_operator requires a terminal; use ssh -t')
  with gui_adapters(options):return runner.main()
 except ValueError as exc:print(json.dumps({'preflight_rejected':str(exc)}),file=sys.stderr);return 2
 finally:sys.argv=saved

if __name__=='__main__':raise SystemExit(main())
