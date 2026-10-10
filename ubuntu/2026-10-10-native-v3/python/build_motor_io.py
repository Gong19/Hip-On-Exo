"""Build a GIL-releasing wrapper from the user's existing Unitree SDK sources.
No vendor source or binaries are downloaded or redistributed.
"""
import argparse,pathlib,subprocess,sysconfig,tempfile

def main():
    p=argparse.ArgumentParser();p.add_argument('--sdk-root',required=True);a=p.parse_args()
    root=pathlib.Path(a.sdk_root).resolve()
    source=(root/'thirdparty/python_wrapper/wrapper.cpp').read_text()
    old='.def("sendRecv", py::overload_cast<MotorCmd*, MotorData*>(&SerialPort::sendRecv));'
    if old not in source:raise RuntimeError('Unsupported SDK wrapper; inspect source before patching')
    source=source.replace('PYBIND11_MODULE(unitree_actuator_sdk, m)','PYBIND11_MODULE(hipexo_unitree_sdk, m)').replace(old,old[:-2]+', py::call_guard<py::gil_scoped_release>());')
    target=pathlib.Path(__file__).resolve().parent/('hipexo_unitree_sdk'+sysconfig.get_config_var('EXT_SUFFIX'))
    with tempfile.TemporaryDirectory() as d:
        src=pathlib.Path(d)/'wrapper.cpp';src.write_text(source)
        subprocess.run(['g++','-O2','-shared','-std=c++14','-fPIC',str(src),'-I'+str(root/'include'),'-I'+str(root/'thirdparty/pybind11/include'),'-I'+sysconfig.get_paths()['include'],str(root/'lib/libUnitreeMotorSDK_Arm64.so'),'-Wl,-rpath,'+str(root/'lib'),'-o',str(target)],check=True)
    print(target)
if __name__=='__main__':main()
