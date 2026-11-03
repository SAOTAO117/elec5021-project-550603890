from pathlib import Path
import json
import numpy as np

ROOT=Path(__file__).resolve().parent
R=ROOT/"output"
OLD=ROOT/"Inputs"

def generate(method,dead,spacing,tag):
    gate=np.loadtxt(OLD/(method+'_gates.csv'),delimiter=',')
    curr=np.loadtxt(OLD/(method+'_bridge_current.csv'),delimiter=',')
    duration=.02;pre=50e-6;end=duration+pre
    ix=np.arange(0,len(curr),int(round(spacing/.5e-6)))
    if ix[-1]!=len(curr)-1:ix=np.r_[ix,len(curr)-1]
    curr=curr[ix]
    lines=[f'EPC2218 device replay {tag}', '.include EPC2218V5.net','.temp 27','.options reltol=1e-4 abstol=1e-9 vntol=1e-7 method=gear maxord=2 klu','Vdc bus 0 60']
    def pwl(name,pos,neg,points):
        points=sorted({round(t,14):v for t,v in points if -1e-13<=t<=end+1e-13}.items())
        lines.append(f'{name} {pos} {neg} PWL(')
        lines.extend('+ '+' '.join(f'{t:.15g} {v:.15g}' for t,v in points[k:k+5]) for k in range(0,len(points),5));lines.append('+ )')
    currents=np.vstack([np.column_stack([curr[:,0]-.02+pre,curr[:,1:]])[curr[:,0]>=.02-pre-1e-13],np.column_stack([curr[:,0]+pre,curr[:,1:]])])
    events={};states={}
    initial=gate[gate[:,0]<=.02-pre+1e-13][-1].copy();initial[0]=0
    tail=gate[(gate[:,0]>.02-pre+1e-13)&(gate[:,0]<.02-1e-13)].copy();tail[:,0]+=-.02+pre
    main=gate.copy();main[:,0]+=pre
    records=np.vstack([initial,tail,main])
    for ph in range(3):
        p='abc'[ph];hi=ph+1;lo=ph+4
        lines += [f'Xq{hi} bus g{hi} {p} EPC2218',f'Xq{lo} {p} g{lo} 0 EPC2218']
        pwl('Iload'+p,p,'0',[(r[0],r[ph+1]) for r in currents])
        for j,col,ref in [(hi,2*ph+1,p),(lo,2*ph+2,'0')]:
            changes=np.flatnonzero(np.diff(records[:,col])!=0)+1
            t=records[changes,0].copy();v=records[changes,col]
            matecol=col+1 if col%2 else col-1
            falls=records[np.flatnonzero(np.diff(records[:,matecol])<0)+1,0]
            for k in np.flatnonzero(v>0):
                preceding=falls[falls<=t[k]+1e-13]
                if len(preceding):t[k]=preceding[-1]+dead
                else:t[k]+=dead-1e-6
            t=np.round(t,14)
            order=np.argsort(t);t=t[order];v=v[order]
            before=np.flatnonzero(t<=0);last=v[before[-1]] if len(before) else records[0,col]
            points=[(0,5*last)];kept=[]
            for tt,vv in zip(t,v):
                if 0<tt<=end:
                    points.extend([(max(points[-1][0],tt-1e-10),5*last),(tt,5*vv)]);last=vv;kept.append(float(tt))
            points.append((end,5*last));pwl(f'Vg{j}',f'g{j}',ref,points);events[str(j)]=kept
            states[j]=[(0,float(initial[col]))]+[(float(tt),float(vv)) for tt,vv in zip(t,v) if 0<tt<=end]
    # Validate the entire replay, including the pre-roll/record boundary.
    for hi,lo in [(1,4),(2,5),(3,6)]:
        merged=sorted([(t,j,v) for j in [hi,lo] for t,v in states[j]],key=lambda e:(e[0],e[2]))
        current={hi:states[hi][0][1],lo:states[lo][0][1]};fall={};blanking=[]
        for t,j,v in merged:
            other=lo if j==hi else hi
            current[j]=v
            assert not (current[hi] and current[lo]),f'{tag}: overlapping pair {hi}/{lo} at {t}'
            if v==0:fall[j]=t
            elif other in fall and pre+dead<t<end-dead:blanking.append(t-fall[other])
        assert blanking and np.allclose(blanking,dead,rtol=0,atol=1e-12),(tag,hi,min(blanking),max(blanking))
    save=['v(bus)','i(vdc)']+[f'v({p})' for p in 'abc']
    for j in range(1,7):save += [f'v(xq{j}.{n})' for n in ['drain','source','gate']]+[f'v(g{j})',f'i(vg{j})',f'@b.xq{j}.bswitch[i]']
    # Same operating-point initial guess used by the completed baseline replay.
    if method=='SVPWM':
        lines.extend(x for x in (OLD/'nodeset.sp').read_text().splitlines() if x.startswith('.nodeset'))
    lines+=['.save '+' '.join(save),f'.tran 0.5u {end:.12g} 0 0.5u','.end']
    (R/(tag+'.cir')).write_text('\n'.join(lines)+'\n')
    (R/(tag+'.json')).write_text(json.dumps({'method':method,'deadtime':dead,'current_spacing':spacing,'events':events}))
    np.savetxt(R/(tag+'_current.csv'),currents,delimiter=',')
