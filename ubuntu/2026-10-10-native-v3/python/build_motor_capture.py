"""Build the zero-output capture helper against a user's existing Unitree SDK."""
import argparse,pathlib,subprocess
p=argparse.ArgumentParser();p.add_argument('--sdk-root',required=True);a=p.parse_args();root=pathlib.Path(a.sdk_root).resolve();here=pathlib.Path(__file__).resolve().parent
subprocess.run(['g++','-O2','-std=c++14',str(here/'hipexo_motor_capture.cpp'),'-I'+str(root/'include'),str(root/'lib/libUnitreeMotorSDK_Arm64.so'),'-Wl,-rpath,'+str(root/'lib'),'-o',str(here/'hipexo_motor_capture')],check=True)
