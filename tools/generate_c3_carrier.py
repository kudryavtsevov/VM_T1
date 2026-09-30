"""SPDX-License-Identifier: Apache-2.0
Independent F1Q carrier for the two 1x8 ESP32-C3 Super Mini headers.
KiCad 8 Python. Top view, USB toward top edge; 15.24 mm row separation.
"""
from pathlib import Path
import json, uuid, heapq, math, html
import pcbnew as p
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'hardware'/'c3-supermini-v1';OUT.mkdir(parents=True,exist_ok=True)
NAME='vm-t1-f1q-c3-v1'
b=p.BOARD();b.SetCopperLayerCount(2);b.GetDesignSettings().SetBoardThickness(p.FromMM(1.6))
nets={};components=[];routes=[];pads=[]
def uid():return str(uuid.uuid4())
def xy(x,y):return p.VECTOR2I(p.FromMM(100+x),p.FromMM(100+y))
def net(n):
 if n not in nets:
  q=p.NETINFO_ITEM(b,n);b.Add(q);nets[n]=q
 return nets[n]
def text(t,x,y,size=.85,layer=p.F_SilkS):
 q=p.PCB_TEXT(b);q.SetText(t);q.SetPosition(xy(x,y));q.SetTextSize(p.VECTOR2I(p.FromMM(size),p.FromMM(size)));q.SetTextThickness(p.FromMM(.12));q.SetLayer(layer);b.Add(q)
def component(ref,value,pts):
 f=p.FOOTPRINT(b);f.SetReference(ref);f.SetValue(value);f.SetPosition(xy(0,0));f.SetFPID(p.LIB_ID('VM_T1_C3',ref));f.SetAttributes(p.FP_THROUGH_HOLE)
 f.Reference().SetVisible(False);f.Value().SetVisible(False)
 for i,(x,y,n) in enumerate(pts,1):
  q=p.PAD(f);q.SetNumber(str(i));q.SetShape(p.PAD_SHAPE_CIRCLE);q.SetAttribute(p.PAD_ATTRIB_PTH);q.SetPosition(xy(x,y));q.SetSize(p.VECTOR2I(p.FromMM(1.8),p.FromMM(1.8)));q.SetDrillSize(p.VECTOR2I(p.FromMM(1),p.FromMM(1)));ls=p.LSET.AllCuMask();ls.AddLayer(p.F_Mask);ls.AddLayer(p.B_Mask);q.SetLayerSet(ls);q.SetNet(net(n));f.Add(q);pads.append((x,y,n))
 b.Add(f);components.append(dict(ref=ref,value=value,pins=[dict(number=i+1,x=x,y=y,net=n) for i,(x,y,n) in enumerate(pts)]));return f
left=['SERVO_R','SERVO_W','RC_DT','LED','BOOT','SCL','OFF','BUZZER']
right=['+5V','GND','+3V3','SERVO_S','ESC','SDA','START','ARM']
component('U1','ESP32-C3 Super Mini',[(10.16,5.08+i*2.54,n) for i,n in enumerate(left)]+[(25.4,5.08+i*2.54,n) for i,n in enumerate(right)])
for i,(a,c) in enumerate(zip(['5','6','7','8','9','10','20','21'],['5V','G','3V3','4','3','2','1','0'])):
 text(a,7.9,5.08+i*2.54);text(c,28.1,5.08+i*2.54)
text('USB UP',17.78,2.54,1)
text('C3 SUPER MINI',17.78,12.7,1)
text('2 x 8 / 15.24mm',17.78,16.5,.8)
text('ANTENNA / NO COPPER',17.78,29.21,.8)
for ref,x,label in [('J1',5.08,'INA226'),('J2',25.4,'BMP280')]:
 component(ref,label,[(x+i*2.54,38.1,n) for i,n in enumerate(['+3V3','GND','SDA','SCL'])]);text(label,x+3.81,35.56);text('3V G D C',x+3.81,40.1,.8)
for ref,x,y,n,label in [('J3',5.08,45.72,'SERVO_S','SERVO S'),('J4',25.4,45.72,'SERVO_R','SERVO R'),('J5',5.08,53.34,'SERVO_W','SERVO W'),('J6',25.4,53.34,'ESC','ESC')]:
 pts=[(x,y,'GND'),(x+2.54,y,'+5V'),(x+5.08,y,n)] if n!='ESC' else [(x,y,'GND'),(x+5.08,y,n)]
 component(ref,label,pts);text(label,x+2.54,y-2);text('G + S' if n!='ESC' else 'G   S',x+2.54,y+2,.8)
for ref,x,y,n,label in [('J7',5.08,60.96,'ARM','ARM'),('J8',5.08,68.58,'START','START'),('J9',17.78,60.96,'OFF','OFF'),('J10',17.78,68.58,'RC_DT','RC DT'),('J11',30.48,60.96,'BUZZER','BUZZ'),('J12',30.48,68.58,'+5V','BEC 5V')]:
 component(ref,label,[(x,y,'GND'),(x+2.54,y,n)]);text(label,x+1.27,y-2);text('G S' if n!='+5V' else 'G +',x+1.27,y+2,.8)
component('R1','47k ESC pulldown',[(2.54,73.66,'ESC'),(10.16,73.66,'GND')]);text('R1 47k',6.35,75.3,.8)
component('R2','4k7 SDA pullup',[(15.24,73.66,'SDA'),(22.86,73.66,'+3V3')]);text('R2 4k7',19.05,75.3,.8)
component('R3','4k7 SCL pullup',[(27.94,73.66,'SCL'),(35.56,73.66,'+3V3')]);text('R3 4k7',31.75,75.3,.8)
# Copper exclusion between headers and under/beyond ceramic antenna.
zone=p.ZONE(b);zone.SetIsRuleArea(True);zone.SetLayerSet(p.LSET.AllCuMask());zone.SetDoNotAllowTracks(True);zone.SetDoNotAllowVias(True);zone.SetDoNotAllowCopperPour(True)
poly=zone.Outline();poly.NewOutline()
for x,y in [(11.5,0),(24.06,0),(24.06,34),(11.5,34)]:
 v=xy(x,y);poly.Append(v.x,v.y)
b.Add(zone)

# Grid router: each electrical net becomes a tree, copper clearance obstacles
# include pads, traces and vias; KiCad DRC remains the authoritative check.
STEP=.254;NX=160;NY=300
blocked=[{},{}];nodes={};segments=[];vias=[]
def idx(x,y):return round(x/STEP),round(y/STEP)
def mark(layer,x,y,r,n):
 ix,iy=idx(x,y);rr=math.ceil(r/STEP)
 for a in range(max(0,ix-rr),min(NX,ix+rr)+1):
  for c in range(max(0,iy-rr),min(NY,iy+rr)+1):
   if math.hypot(a*STEP-x,c*STEP-y)<=r:blocked[layer].setdefault((a,c),set()).add(n)
for x,y,n in pads:
 for l in (0,1):mark(l,x,y,1.32,n)
 nodes.setdefault(n,[]).append(idx(x,y))
def free(x,y,l,n):
 if x<3 or y<3 or x>NX-3 or y>NY-3:return False
 if 11.2<=x*STEP<=24.36 and y*STEP<=34.3:return False
 if n in ('+5V','GND','+3V3'):
  return not any(blocked[l].get((x+dx,y+dy),set())-{n} for dx in (-1,0,1) for dy in (-1,0,1))
 return not(blocked[l].get((x,y),set())-{n})
def track(n,a,c,l):
 q=p.PCB_TRACK(b);q.SetStart(xy(a[0]*STEP,a[1]*STEP));q.SetEnd(xy(c[0]*STEP,c[1]*STEP));q.SetLayer(p.F_Cu if l==0 else p.B_Cu);q.SetWidth(p.FromMM(.8 if n in ('+5V','GND','+3V3') else .3));q.SetNet(net(n));b.Add(q)
 routes.append(dict(net=n,a=[a[0]*STEP,a[1]*STEP],c=[c[0]*STEP,c[1]*STEP],layer='F' if l==0 else 'B',width=.8 if n in ('+5V','GND','+3V3') else .3))
def addvia(n,x,y):
 if any(math.hypot(x*STEP-a,y*STEP-c)<.01 for a,c,_ in pads+vias):return
 q=p.PCB_VIA(b);q.SetPosition(xy(x*STEP,y*STEP));q.SetWidth(p.FromMM(.7));q.SetDrill(p.FromMM(.3));q.SetViaType(p.VIATYPE_THROUGH);q.SetLayerPair(p.F_Cu,p.B_Cu);q.SetNet(net(n));b.Add(q);vias.append((x*STEP,y*STEP,n))
 for l in (0,1):mark(l,x*STEP,y*STEP,.75,n)
def route_net(n,pts):
 tree={(pts[0][0],pts[0][1],l) for l in (0,1)}
 for end in pts[1:]:
  goals={(end[0],end[1],l) for l in (0,1)}
  if tree & goals:continue
  # A* from the new pad to any existing point of the same net.
  xmin=min(x for x,y,l in tree);xmax=max(x for x,y,l in tree);ymin=min(y for x,y,l in tree);ymax=max(y for x,y,l in tree)
  def h(s):x,y,l=s;return max(xmin-x,0,x-xmax)+max(ymin-y,0,y-ymax)
  cost={s:0 for s in goals};prev={};heap=[(h(s),0,s) for s in goals];heapq.heapify(heap);last=None
  while heap:
   _,g,s=heapq.heappop(heap)
   if cost[s]!=g:continue
   if s in tree:last=s;break
   x,y,l=s
   for a,c,k,w in [(x+1,y,l,1),(x-1,y,l,1),(x,y+1,l,1),(x,y-1,l,1),(x,y,1-l,12)]:
    if not free(a,c,k,n):continue
    if k!=l:
     if any(.01<math.hypot(a*STEP-px,c*STEP-py)<.92 for px,py,_ in pads):continue
     if any(.01<math.hypot(a*STEP-px,c*STEP-py)<.56 for px,py,_ in vias):continue
     # Wider via needs extra distance from foreign copper on BOTH layers.
     if any(blocked[j].get((a+dx,c+dy),set())-{n} for j in (0,1) for dx in (-1,0,1) for dy in (-1,0,1)):continue
    t=(a,c,k);ng=g+w
    if ng<cost.get(t,1e20):cost[t]=ng;prev[t]=s;heapq.heappush(heap,(ng+h(t),ng,t))
  if last is None:raise RuntimeError('No route for '+n+' to '+str(end))
  path=[last]
  while path[-1] in prev:path.append(prev[path[-1]])
  tree.update(path)
  for x,y,l in path:mark(l,x*STEP,y*STEP,.85 if n in ('+5V','GND','+3V3') else .58,n)
  start=path[0];lastdir=None;old=start
  for pt in path[1:]:
   if pt[2]!=old[2]:
    if start!=old:track(n,start,old,old[2])
    addvia(n,pt[0],pt[1]);start=pt;lastdir=None
   else:
    direction=(pt[0]-old[0],pt[1]-old[1])
    if lastdir is not None and direction!=lastdir:
     track(n,start,old,old[2]);start=old
    lastdir=direction
   old=pt
  if start!=old:track(n,start,old,old[2])
# Fan out each C3 pin before routing the longer connections.
for x,y,n in pads[:16]:
 if n in ('BOOT','LED'): continue
 endx=5.08 if x<17 else 30.48
 a=idx(x,y);c=idx(endx,y);track(n,a,c,0);addvia(n,*c)
 for ix in range(min(a[0],c[0]),max(a[0],c[0])+1):mark(0,ix*STEP,y,.85 if n in ('+5V','GND','+3V3') else .58,n)
 nodes[n][0]=c
for n in sorted(nodes,key=lambda k: (3 if k=='GND' else 2 if k=='+3V3' else 1 if k=='+5V' else 0, -len(nodes[k]), k)):
 print('Route',n,flush=True);route_net(n,nodes[n])
# Remove fanout vias that ultimately remained on the front layer only.
for v in list(b.GetTracks()):
 if not isinstance(v,p.PCB_VIA):continue
 pos=v.GetPosition();layers=set()
 for t in b.GetTracks():
  if isinstance(t,p.PCB_VIA) or t.GetNetCode()!=v.GetNetCode():continue
  a=t.GetStart();c=t.GetEnd();dx=c.x-a.x;dy=c.y-a.y;den=dx*dx+dy*dy
  f=max(0,min(1,((pos.x-a.x)*dx+(pos.y-a.y)*dy)/den)) if den else 0
  if math.hypot(pos.x-a.x-f*dx,pos.y-a.y-f*dy)<10:layers.add(t.GetLayer())
 if len(layers)<2:b.Remove(v)
for a,c in [((0,0),(40.64,0)),((40.64,0),(40.64,76.2)),((40.64,76.2),(0,76.2)),((0,76.2),(0,0))]:
 q=p.PCB_SHAPE();q.SetShape(p.SHAPE_T_SEGMENT);q.SetStart(xy(*a));q.SetEnd(xy(*c));q.SetLayer(p.Edge_Cuts);q.SetWidth(p.FromMM(.05));b.Add(q)
text('F1Q C3 / R010',20.32,32.5,.85)
lib=OUT/'VM_T1_C3.pretty';lib.mkdir(exist_ok=True);plugin=p.PCB_IO_MGR.PluginFind(p.PCB_IO_MGR.KICAD_SEXP)
for f in b.GetFootprints():plugin.FootprintSave(str(lib),f)
(OUT/'fp-lib-table').write_text('(fp_lib_table (lib (name "VM_T1_C3")(type "KiCad")(uri "${KIPRJMOD}/VM_T1_C3.pretty")(options "")(descr "")))')
p.SaveBoard(str(OUT/(NAME+'.kicad_pcb')),b)
(OUT/(NAME+'.kicad_pro')).write_text(json.dumps({'board':{'design_settings':{'rules':{'min_clearance':.2,'min_track_width':.25,'min_hole_clearance':.25,'min_copper_edge_clearance':.3}}}},indent=2))
(OUT/'connectivity.json').write_text(json.dumps(dict(board_mm=[40.64,76.2,1.6],components=components,routes=routes,vias=vias),indent=2))
# Net-labelled passive connector schematic, internal module circuitry is external.
root=uid();libs=[];inst=[]
for index,c in enumerate(components):
 key=c['ref'];height=(len(c['pins'])-1)*2.54;x,y=(38.1,30.48) if index==0 else (101.6+63.5*((index-1)//6),25.4+27.94*((index-1)%6))
 pins=''.join(f'(pin passive line (at -10.16 {-i*2.54:g} 0) (length 2.54) (name "{v["net"]}" (effects (font (size 1 1)))) (number "{i+1}" (effects (font (size 1 1)))))' for i,v in enumerate(c['pins']))
 libs.append(f'(symbol "VM_T1_C3:{key}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes) (property "Reference" "{key[0]}" (at 0 4 0) (effects (font (size 1 1)))) (property "Value" "{c["value"]}" (at 0 2 0) (effects (font (size 1 1)))) (symbol "{key}_0_1" (rectangle (start -7.62 1.27) (end 12.7 {-height-1.27:g}) (stroke (width .254) (type default)) (fill (type background)))) (symbol "{key}_1_1" {pins}))')
 inst.append(f'(symbol (lib_id "VM_T1_C3:{key}") (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid "{uid()}") (property "Reference" "{key}" (at {x} {y-5} 0) (effects (font (size 1 1)))) (property "Value" "{c["value"]}" (at {x} {y-2.5} 0) (effects (font (size 1 1)))) (property "Footprint" "VM_T1_C3:{key}" (at {x} {y} 0) (effects (font (size 1 1)) hide)) (instances (project "{NAME}" (path "/{root}" (reference "{key}") (unit 1)))))')
 for i,v in enumerate(c['pins']):
  if v['net'] in ('BOOT','LED'):
   inst.append(f'(no_connect (at {x-10.16:g} {y+i*2.54:g}) (uuid "{uid()}"))');continue
  inst.append(f'(label "{v["net"]}" (at {x-10.16:g} {y+i*2.54:g} 0) (effects (font (size .8 .8)) (justify right bottom)) (uuid "{uid()}"))')
(OUT/(NAME+'.kicad_sch')).write_text(f'(kicad_sch (version 20231120) (generator eeschema) (uuid "{root}") (paper "A4") (lib_symbols {"".join(libs)}) {"".join(inst)})')
(OUT/'VM_T1_C3.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator kicad_symbol_editor) '+''.join(libs).replace('VM_T1_C3:','')+')')
(OUT/'sym-lib-table').write_text('(sym_lib_table (lib (name "VM_T1_C3")(type "KiCad")(uri "${KIPRJMOD}/VM_T1_C3.kicad_sym")(options "")(descr "")))')
print(OUT)

# Assembly preview uses the actual KiCad pad and text coordinates.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="650" height="1170" viewBox="-1 -1 42.64 78.2"><rect width="40.64" height="76.2" fill="#173f38"/>', '<rect x="8.78" y="3.71" width="18" height="22.52" fill="#30433e" stroke="#aec4b7" stroke-width=".12" stroke-dasharray=".5 .4"/>', '<rect x="13.5" y="1.5" width="8.6" height="5" fill="#899497"/>']
for t in b.GetTracks():
 if isinstance(t,p.PCB_VIA):continue
 a=t.GetStart();c=t.GetEnd();color='#cf9e5d' if t.GetLayer()==p.F_Cu else '#52859b'
 svg.append(f'<path d="M{p.ToMM(a.x)-100},{p.ToMM(a.y)-100} L{p.ToMM(c.x)-100},{p.ToMM(c.y)-100}" fill="none" stroke="{color}" stroke-width="{p.ToMM(t.GetWidth())}" opacity=".5"/>')
for f in b.GetFootprints():
 for q in f.Pads():
  a=q.GetPosition();x=p.ToMM(a.x)-100;y=p.ToMM(a.y)-100
  svg.append(f'<circle cx="{x}" cy="{y}" r=".9" fill="#d9b56e"/><circle cx="{x}" cy="{y}" r=".5" fill="#111"/>')
for q in b.GetDrawings():
 if isinstance(q,p.PCB_TEXT):
  a=q.GetPosition();svg.append(f'<text x="{p.ToMM(a.x)-100}" y="{p.ToMM(a.y)-100+.3}" text-anchor="middle" fill="#fff" font-size="{p.ToMM(q.GetTextSize().x)}" font-family="sans-serif">{html.escape(q.GetText())}</text>')
svg.append('</svg>');(OUT/'assembly.svg').write_text(''.join(svg),encoding='utf-8')
