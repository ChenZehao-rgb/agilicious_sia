#!/usr/bin/env python3
"""Build the recorded timer baseline against current message types for local A/B.

Only baseline ControlNode/HardwarePilot are compiled into this executable. Other
unchanged components and generated ROS types come from the tested local build.
No git checkout, firmware or installed executable is modified.
"""
import json
from pathlib import Path
import shlex
import subprocess
import sys

root=Path(__file__).resolve().parents[2]
base=Path(sys.argv[1]).resolve()
head='87b0f5e20f6f256fa5b10a519cbdf4cc10f9c615'
paths=['agi_ros2/src/control_main.cpp','agi_ros2/src/control_node.cpp','agi_ros2/include/agi_ros2/control_node.h',
       'agilib/src/pilot/hardware_pilot.cpp','agilib/include/agilib/pilot/hardware_pilot.hpp']
for path in paths:
    target=base/path
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_bytes(subprocess.check_output(['git','show',f'{head}:{path}'],cwd=root))
commands=json.loads((root/'build/agi_ros2/compile_commands.json').read_text())
objects=[]
for path in [p for p in paths if p.endswith('.cpp')]:
    c=next((c for c in commands if Path(c['file']).resolve()==root/path), None)
    if c is None:
        flags={}
        for line in (root/'build/agi_ros2/CMakeFiles/control_node.dir/flags.make').read_text().splitlines():
            if ' = ' in line:
                key,value=line.split(' = ',1); flags[key]=value
        c={'directory':str(root/'build/agi_ros2'), 'command':'/usr/bin/c++ '+flags['CXX_DEFINES']+' '+flags['CXX_INCLUDES']+' '+flags['CXX_FLAGS']+' -o unused.o -c '+str(root/path)}
    args=shlex.split(c['command'])
    obj=base/(Path(path).stem+'.o')
    args[args.index('-o')+1]=str(obj)
    args[args.index('-c')+1]=str(base/path)
    args[1:1]=['-I'+str(base/'agi_ros2/include'),'-I'+str(base/'agilib/include')]
    subprocess.run(args,cwd=c['directory'],check=True)
    objects.append(str(obj))
link=shlex.split((root/'build/agi_ros2/CMakeFiles/control_node.dir/link.txt').read_text())
link=[arg for arg in link if not arg.endswith('.o')]
link[link.index('-o')+1]=str(base/'control_node')
link[1:1]=objects
subprocess.run(link,cwd=root/'build/agi_ros2',check=True)
print(base/'control_node')
