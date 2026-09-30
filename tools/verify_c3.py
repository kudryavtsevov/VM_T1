"""SPDX-License-Identifier: Apache-2.0
Run with KiCad Python: verify_c3.py path/to/exported-kicadxml-netlist.xml
Checks actual PCB geometry, schematic connectivity and firmware assignments.
"""
from pathlib import Path
import re, sys, xml.etree.ElementTree as E
import pcbnew as p
root=Path(__file__).resolve().parents[1]
b=p.LoadBoard(str(root/'hardware/c3-supermini-v1/vm-t1-f1q-c3-v1.kicad_pcb'))
fps={f.GetReference():f for f in b.GetFootprints()}
u={int(q.GetNumber()):q for q in fps['U1'].Pads()}
assert len(u)==16
for i in range(1,9):
 assert abs(p.ToMM(u[i+8].GetPosition().x-u[i].GetPosition().x)-15.24)<1e-5
 assert u[i+8].GetPosition().y==u[i].GetPosition().y
 if i<8:assert abs(p.ToMM(u[i+1].GetPosition().y-u[i].GetPosition().y)-2.54)<1e-5
gpio_to_pad={5:1,6:2,7:3,8:4,9:5,10:6,20:7,21:8,4:12,3:13,2:14,1:15,0:16}
pin_to_net={'SERVO_S':'SERVO_S','SERVO_R':'SERVO_R','SERVO_W':'SERVO_W','ESC':'ESC','ARM':'ARM','START':'START','RC_DT':'RC_DT','OFF_BUTTON':'OFF','BUZZER':'BUZZER','LED':'LED','SDA':'SDA','SCL':'SCL'}
assign=dict((n,int(v)) for n,v in re.findall(r'\bPIN_(\w+)\s*=\s*(\d+)',(root/'VM_T1/BoardPins.h').read_text()))
assert len(assign)==len(pin_to_net) and len(set(assign.values()))==len(assign)
for name,net in pin_to_net.items():assert u[gpio_to_pad[assign[name]]].GetNetname()==net,(name,net)
assert [u[i].GetNetname() for i in (9,10,11)]==['+5V','GND','+3V3']
ignore={('U1','4'),('U1','5')} # onboard LED, BOOT; no external connections
actual={(f.GetReference(),q.GetNumber()):q.GetNetname() for f in b.GetFootprints() for q in f.Pads() if (f.GetReference(),q.GetNumber()) not in ignore}
schematic={}
for n in E.parse(sys.argv[1]).findall('./nets/net'):
 for q in n.findall('node'):
  k=(q.get('ref'),q.get('pin'))
  if k not in ignore:schematic[k]=n.get('name').lstrip('/')
assert actual==schematic,(actual,schematic)
print(f'C3 check passed: 2x8 geometry, {len(assign)} unique GPIO, {len(actual)} connected schematic/PCB pins')
