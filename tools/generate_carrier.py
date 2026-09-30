"""SPDX-License-Identifier: Apache-2.0
Generate the VM-T1 wired-module carrier with KiCad 8 pcbnew.
Run with KiCad's Python; no files from the separate closed project are used.
"""
from pathlib import Path
import json, uuid, html
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'hardware'/'carrier-v1'; OUT.mkdir(parents=True,exist_ok=True)
NAME='vm-t1-carrier-v1'
b=p.BOARD(); b.SetCopperLayerCount(2); b.GetDesignSettings().SetBoardThickness(p.FromMM(1.6))
def xy(x,y): return p.VECTOR2I(p.FromMM(x+100),p.FromMM(y+100))
def uid(): return str(uuid.uuid4())
nets={}; components=[]; routes=[]
def net(name):
 if name not in nets:
  n=p.NETINFO_ITEM(b,name);b.Add(n);nets[name]=n
 return nets[name]
def text(t,x,y,size=.8,layer=p.F_SilkS):
 size=max(size,.8)
 q=p.PCB_TEXT(b);q.SetText(t);q.SetPosition(xy(x,y));q.SetTextSize(p.VECTOR2I(p.FromMM(size),p.FromMM(size)));q.SetTextThickness(p.FromMM(.12));q.SetLayer(layer);b.Add(q)
def terminal(ref,value,pins):
 f=p.FOOTPRINT(b); f.SetReference(ref);f.SetValue(value); f.SetPosition(xy(0,0));f.SetAttributes(p.FP_THROUGH_HOLE)
 f.SetFPID(p.LIB_ID('VM_T1',ref));f.Reference().SetVisible(False);f.Value().SetVisible(False)
 for number,(x,y,n) in enumerate(pins,1):
  pad=p.PAD(f);pad.SetNumber(str(number));pad.SetShape(p.PAD_SHAPE_CIRCLE);pad.SetAttribute(p.PAD_ATTRIB_PTH)
  pad.SetSize(p.VECTOR2I(p.FromMM(1.8),p.FromMM(1.8)));pad.SetDrillSize(p.VECTOR2I(p.FromMM(.9),p.FromMM(.9)))
  layers=p.LSET.AllCuMask();layers.AddLayer(p.F_Mask);layers.AddLayer(p.B_Mask)
  pad.SetLayerSet(layers);pad.SetPosition(xy(x,y));pad.SetNet(net(n));f.Add(pad)
 b.Add(f);components.append(dict(ref=ref,value=value,pins=[dict(number=i+1,x=x,y=y,net=n) for i,(x,y,n) in enumerate(pins)]))
 return f
def route(n,points,layer=p.F_Cu,width=.3):
 for a,c in zip(points,points[1:]):
  if a==c: continue
  t=p.PCB_TRACK(b);t.SetStart(xy(*a));t.SetEnd(xy(*c));t.SetLayer(layer);t.SetWidth(p.FromMM(width));t.SetNet(net(n));b.Add(t)
  routes.append(dict(net=n,a=a,c=c,layer='F' if layer==p.F_Cu else 'B',width=width))
def via(n,x,y):
 v=p.PCB_VIA(b);v.SetPosition(xy(x,y));v.SetWidth(p.FromMM(.7));v.SetDrill(p.FromMM(.3));v.SetViaType(p.VIATYPE_THROUGH);v.SetLayerPair(p.F_Cu,p.B_Cu);v.SetNet(net(n));b.Add(v)

rows=[('5V','+5V','BEC +5V'),('GND','GND','BEC GND'),('3V3','+3V3','3V3 OUT'),
 ('4','SERVO_S','SERVO S'),('5','SERVO_R','SERVO R'),('6','SERVO_W','SERVO W'),
 ('7','BACK','BACK'),('8','FRONT','FRONT'),('3','RC_DT','RC DT'),('18','OFF','OFF'),
 ('10','START','START'),('2','ARM','ARM'),('17','ESC','ESC'),('9','BUZZER','BUZZER'),
 ('16','LED','LED'),('11','SDA','SDA'),('12','SCL','SCL')]
terminal('J1','ESP32-S3-Zero WIRES',[(4,10+4*i,n) for i,(_,n,_) in enumerate(rows)])
for i,(gpio,n,label) in enumerate(rows):
 y=10+4*i
 pins=[(26.5,y,n)]
 if i>=3 and i<=14:
  pins.append((21.42,y,'GND'))
  if i in (3,4,5): pins.append((23.96,y,'+5V'))
 terminal('J'+str(i+2),label,pins)
 route(n,[(4,y),(4,y-1.7),(26.5,y-1.7),(26.5,y)],width=.6 if i<3 else .3)
 text(gpio,7,y,.75);text(label,14,y,.75)
 if 3<=i<=14: text('G',21.42,y+1.55,.65)
 if i in (3,4,5): text('+',23.96,y+1.55,.65)

# Servo supply: BEC rail only. NO battery voltage or motor current on this board.
route('+5V',[(26.5,10),(28.5,10),(28.5,20),(23.96,20),(23.96,30)],p.B_Cu,1.3)
route('GND',[(26.5,14),(21.42,14),(21.42,80.54)],p.B_Cu,1.3)
# 3.3V supply for the two module harnesses.
via('+3V3',17,16.3);route('+3V3',[(17,16.3),(17,78)],p.B_Cu,.6)
for ref,x,value in [('J19',10,'INA226 I2C'),('J20',26.5,'BMP280 I2C')]:
 terminal(ref,value,[(x,78,'+3V3'),(x,80.54,'GND'),(x,83.08,'SDA'),(x,85.62,'SCL')])
 text(value,x-2,76.3,.7)
route('+3V3',[(10,78),(26.5,78)],width=.6)
via('+3V3',17,78)
route('GND',[(10,80.54),(26.5,80.54)],p.B_Cu,.6)
route('SDA',[(26.5,70),(28.5,70),(28.5,83.08),(10,83.08)])
via('SCL',6,72.3);route('SCL',[(6,72.3),(6,85.62),(26.5,85.62)],p.B_Cu,.3)
# 47k pulldown holds ESC low before PWM is configured.
terminal('R1','47k axial',[(10,92,'ESC'),(17.62,92,'GND')])
route('ESC',[(4,58),(1.7,58),(1.7,92),(10,92)])
route('GND',[(26.5,80.54),(28.5,80.54),(28.5,92),(17.62,92)],p.B_Cu,.3)
text('R1 47k',14,94,.8)
text('VM-T1 CARRIER v1',15,3,1)
text('ESP WIRES     OUTPUTS',15,6,.75)
text('3V3 GND SDA SCL',15,88,.65)
text('BEC 5V ONLY / NO MOTOR CURRENT',15,97,.65)
for a,c in [((0,0),(30,0)),((30,0),(30,100)),((30,100),(0,100)),((0,100),(0,0))]:
 e=p.PCB_SHAPE();e.SetShape(p.SHAPE_T_SEGMENT);e.SetStart(xy(*a));e.SetEnd(xy(*c));e.SetLayer(p.Edge_Cuts);e.SetWidth(p.FromMM(.05));b.Add(e)
footprints=OUT/'VM_T1.pretty';footprints.mkdir(exist_ok=True)
plugin = p.PCB_IO_MGR.PluginFind(p.PCB_IO_MGR.KICAD_SEXP)
for f in b.GetFootprints(): plugin.FootprintSave(str(footprints),f)
(OUT/'fp-lib-table').write_text('(fp_lib_table (lib (name "VM_T1")(type "KiCad")(uri "${KIPRJMOD}/VM_T1.pretty")(options "")(descr "VM-T1 solder terminals")))')
p.SaveBoard(str(OUT/(NAME+'.kicad_pcb')),b)
(OUT/(NAME+'.kicad_pro')).write_text(json.dumps({'board':{'design_settings':{'rules':{'min_clearance':.2,'min_track_width':.25,'min_hole_clearance':.25,'min_copper_edge_clearance':.3}}}},indent=2))
(OUT/'connectivity.json').write_text(json.dumps(dict(board_mm=[30,100,1.6],components=components,routes=routes),indent=2))

# Native KiCad schematic, passive harness connectors with net labels on all pins.
root=uid(); libs=[]; inst=[]
for idx,c in enumerate(components):
 count=len(c['pins']); key=c['ref']; height=(count-1)*2.54
 if count==17: x,y=40,35
 else: j=idx-1;x,y=110+(j//7)*65,30+(j%7)*23
 x=round(x/1.27)*1.27;y=round(y/1.27)*1.27
 pinsexpr=''.join(f'(pin passive line (at -10.16 {-i*2.54:g} 0) (length 2.54) (name "{pin["net"]}" (effects (font (size 1 1)))) (number "{i+1}" (effects (font (size 1 1)))))' for i,pin in enumerate(c['pins']))
 libs.append(f'(symbol "VM_T1:{key}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes) (property "Reference" "J" (at 0 4 0) (effects (font (size 1 1)))) (property "Value" "{c["value"]}" (at 0 2 0) (effects (font (size 1 1)))) (symbol "{key}_0_1" (rectangle (start -7.62 1.27) (end 12.7 {-height-1.27:g}) (stroke (width .254) (type default)) (fill (type background)))) (symbol "{key}_1_1" {pinsexpr}))')
 inst.append(f'(symbol (lib_id "VM_T1:{key}") (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}") (property "Reference" "{key}" (at {x} {y-5} 0) (effects (font (size 1 1)))) (property "Value" "{c["value"]}" (at {x} {y-2.5} 0) (effects (font (size 1 1)))) (property "Footprint" "VM_T1:{key}" (at {x} {y} 0) (effects (font (size 1 1)) hide)) (instances (project "{NAME}" (path "/{root}" (reference "{key}") (unit 1)))))')
 for i,pin in enumerate(c['pins']):
  inst.append(f'(label "{pin["net"]}" (at {x-10.16:g} {y+i*2.54:g} 0) (effects (font (size .8 .8)) (justify right bottom)) (uuid "{uid()}"))')
(OUT/(NAME+'.kicad_sch')).write_text(f'(kicad_sch (version 20231120) (generator eeschema) (uuid "{root}") (paper "A4") (lib_symbols {"".join(libs)}) {"".join(inst)})',encoding='utf-8')
(OUT/'VM_T1.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator kicad_symbol_editor) '+''.join(libs).replace('VM_T1:', '')+')',encoding='utf-8')
(OUT/'sym-lib-table').write_text('(sym_lib_table (lib (name "VM_T1")(type "KiCad")(uri "${KIPRJMOD}/VM_T1.kicad_sym")(options "")(descr "VM-T1 connectors")))')
# Standalone assembly preview, generated from actual board coordinates.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="420" height="1150" viewBox="-4 -5 38 110">','<rect x="0" y="0" width="30" height="100" rx="1" fill="#16453d"/>']
for r in routes:
 a,c=r['a'],r['c'];color='#dfac56' if r['layer']=='F' else '#467d9a'
 svg.append(f'<path d="M{a[0]},{a[1]} L{c[0]},{c[1]}" stroke="{color}" stroke-width="{r["width"]}" fill="none" opacity=".6"/>')
for c in components:
 for pin in c['pins']:
  x,y=pin['x'],pin['y'];svg.append(f'<circle cx="{x}" cy="{y}" r=".9" fill="#dab76a"/><circle cx="{x}" cy="{y}" r=".45" fill="#111"/>')
for i,(gpio,n,label) in enumerate(rows):
 y=10+i*4
 svg.append(f'<text x="6" y="{y+.3}" fill="white" font-size=".85" font-family="sans-serif">{gpio}</text><text x="9" y="{y+.3}" fill="white" font-size=".85" font-family="sans-serif">{html.escape(label)}</text>')
for x in (10,26.5):
 for i,label in enumerate(['3V3','GND','SDA','SCL']):svg.append(f'<text x="{x-5}" y="{78+i*2.54+.3}" fill="white" font-size=".85">{label}</text>')
svg.append('<g fill="white" font-family="sans-serif" text-anchor="middle"><text x="15" y="4" font-size="1.6">VM-T1 CARRIER v1</text><text x="15" y="6.5" font-size="1">ESP wires | peripherals</text><text x="9" y="76.5" font-size="1">INA226</text><text x="24" y="76.5" font-size="1">BMP280</text><text x="14" y="95" font-size="1">R1 47k / ESC pulldown</text><text x="15" y="98" font-size=".8">5V BEC ONLY — NO MOTOR CURRENT</text></g></svg>')
(OUT/'assembly.svg').write_text(''.join(svg),encoding='utf-8')
print(OUT)

