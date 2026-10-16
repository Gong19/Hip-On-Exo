import ast,unittest,tempfile,os,time,subprocess,fcntl
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
source=Path(__file__).with_name('hipexo_monitor.py').read_text();tree=ast.parse(source);node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_ensure_motor_mode_for_ports');ns={'os':os,'time':time};exec(compile(ast.Module(body=[node],type_ignores=[]),'<startup>','exec'),ns);prepare=ns[node.name]
class StartupTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory(prefix='motor (copy) ');self.binary=Path(self.tmp.name)/'swmotor';self.binary.touch();self.port='/dev/TEST_motor_startup_'+str(os.getpid())
 def tearDown(self):self.tmp.cleanup();Path('/tmp/hipexo-motor-'+os.path.basename(self.port)+'.lock').unlink(missing_ok=True)
 def test_exact_arguments_and_deduplicate(self):
  with patch('subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run,patch('time.sleep'):
   prepare([self.port,self.port],str(self.binary));run.assert_called_once_with(['sudo','-n',str(self.binary),self.port],capture_output=True,text=True,timeout=8)
 def test_failure_detail(self):
  with patch('subprocess.run',return_value=SimpleNamespace(returncode=1,stdout='',stderr='adapter error')):
   with self.assertRaisesRegex(RuntimeError,'adapter error'):prepare([self.port],str(self.binary))
 def test_timeout(self):
  with patch('subprocess.run',side_effect=subprocess.TimeoutExpired('test',8)):
   with self.assertRaisesRegex(RuntimeError,'timeout'):prepare([self.port],str(self.binary))
 def test_busy_port_never_runs_tool(self):
  with open('/tmp/hipexo-motor-'+os.path.basename(self.port)+'.lock','a') as lease,patch('subprocess.run') as run:
   fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
   with self.assertRaisesRegex(RuntimeError,'already in use'):prepare([self.port],str(self.binary))
   run.assert_not_called()
if __name__ == '__main__': unittest.main()
