from pathlib import Path
import sys,csv
import argparse
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description='Write an EPC2218 replay netlist from saved gate and current records.')

parser.add_argument('--inputs',type=Path,default=ROOT/'Results')
parser.add_argument('--output',type=Path,default=ROOT/'output')
parser.add_argument('--duration',type=float,default=.02)
parser.add_argument('--reltol',type=float,default=1e-4)
args=parser.parse_args()
R=args.inputs;method=ROOT.parent.name;duration=args.duration;tol=args.reltol
assert 0<duration<=.02, 'The recorded input contains one 20 ms cycle.'
assert tol>0, 'Relative tolerance must be positive.'
tag=method+'_devices_cycle'
args.output.mkdir(parents=True,exist_ok=True)
import shutil
shutil.copyfile(ROOT/'EPC2218V5.net',args.output/'EPC2218V5.net')
pre=50e-6;end=pre+duration
gate=[[float(x) for x in row] for row in csv.reader((R/(method+'_gates.csv')).open())]
curr=[[float(x) for x in row] for row in csv.reader((R/(method+'_bridge_current.csv')).open())]
lines=[f'EPC2218 device loss replay {method}', '.include EPC2218V5.net','.temp 27',f'.options reltol={tol} abstol=1e-9 vntol=1e-7 method=gear maxord=2 klu','Vdc bus 0 60']

def shifted(rows):
    return [[max(0,r[0]-.02+pre),*r[1:]] for r in rows if .02-pre-1e-13<=r[0]<.02-1e-13]+[[r[0]+pre,*r[1:]] for r in rows if r[0]<=duration]

def pwl(name,positive,negative,points):
    points=sorted({round(t,14):v for t,v in points if t<=end+1e-13}.items())
    lines.append(f'{name} {positive} {negative} PWL(')
    lines.extend('+ '+' '.join(f'{t:.15g} {v:.15g}' for t,v in points[k:k+5]) for k in range(0,len(points),5));lines.append('+ )')

for ph in range(3):
    p='abc'[ph];hi=ph+1;lo=ph+4
    lines.extend([f'Xq{hi} bus g{hi} {p} EPC2218',f'Xq{lo} {p} g{lo} 0 EPC2218'])
    records=shifted(curr);pwl('Iload'+p,p,'0',[(r[0],r[ph+1]) for r in records])
    for j,col,ref in [(hi,2*ph+1,p),(lo,2*ph+2,'0')]:
        rows=shifted(gate);last=next(r[col] for r in reversed(gate) if r[0]<=.02-pre)
        points=[(0,5*last)]
        for r in rows:
            t=r[0];value=r[col]
            if t>0 and value!=last:
                points.extend([(max(points[-1][0],t-1e-10),5*last),(t,5*value)]);last=value
        points.append((end,5*last));pwl('Vg'+str(j),'g'+str(j),ref,points)
# Initial operating-point hint used for the saved SVPWM calculation.
if method == 'SVPWM':
    nodes = {f'V({phase})': 60 for phase in 'abc'}
    for node, value in [('drain', 60), ('source', 60), ('gate', 65)]:
        nodes.update({f'V(xq{j}.{node})': value for j in range(1, 4)})
    for node, value in [('drain', 60), ('source', 0), ('gate', 0)]:
        nodes.update({f'V(xq{j}.{node})': value for j in range(4, 7)})
    lines.append('.nodeset ' + ' '.join(f'{key}={value}' for key, value in nodes.items()))
save=['v(bus)','i(vdc)']+[f'v({p})' for p in 'abc']
for j in range(1,7):save += [f'v(xq{j}.{n})' for n in ['drain','source','gate']]+[f'v(g{j})',f'i(vg{j})',f'@b.xq{j}.bswitch[i]']
lines+=['.save '+' '.join(save),f'.tran 0.5u {end:.12g} 0 0.5u','.end']
(args.output/(tag+'.cir')).write_text('\n'.join(lines)+'\n')
print(tag,'generated',len(lines),'lines')
