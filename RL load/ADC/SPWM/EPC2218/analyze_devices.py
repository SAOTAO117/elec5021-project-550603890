from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent
import numpy as np
from device_math import read_raw,energy_device
import argparse
parser=argparse.ArgumentParser(description='Integrate EPC2218 device heat from a completed ngspice binary record.')

parser.add_argument('raw',type=Path)
parser.add_argument('--inputs',type=Path,default=ROOT/'Results')
parser.add_argument('--output',type=Path,default=ROOT/'output')
parser.add_argument('--duration',type=float,default=.02)
args=parser.parse_args()
method=ROOT.parent.name;duration=args.duration;R=args.inputs
assert 0 < duration <= .02, 'Duration must be greater than zero and no longer than the recorded 20 ms cycle.'
args.output.mkdir(parents=True,exist_ok=True)
outtag=method+'_devices_cycle'
raw=read_raw(args.raw);start=50e-6;end=start+duration;t0=raw['time']
assert t0[0] <= start and t0[-1] >= end-1e-12, f'{method}: the binary record does not cover {start:.8g} to {end:.8g} s.'
t=np.unique(np.r_[start,t0[(t0>start)&(t0<end)],end]);d={n:np.interp(t,t0,v) for n,v in raw.items()}
avg=lambda v:float(np.trapezoid(v,t)/duration)
z=np.zeros_like(t);dc=-60*d['i(vdc)'];gate=z.copy();add=z.copy();terminal=z.copy();path=[];gates=[];vdslist=[];device_energy=0.
rd=(1-.3229)*.001975*1.015;rs=.3229*.001975*1.015
diode=lambda v:.125*1545/1077*(4.3e-7*np.expm1(np.minimum(v,10)/.8)+2.6e-13*np.expm1(np.minimum(v,10)/.23))
for j in range(1,7):
    ph='abc'[(j-1)%3];vs=d[f'v({ph})'] if j<=3 else z;vd=d['v(bus)'] if j<=3 else d[f'v({ph})']
    vi=d[f'v(xq{j}.drain)'];si=d[f'v(xq{j}.source)'];gi=d[f'v(xq{j}.gate)'];ge=d[f'v(g{j})'];vds=vi-si;vgs=gi-si;vgd=gi-vi
    p=vds*d[f'i(@b.xq{j}.bswitch[i])']+(vd-vi)**2/rd+(vs-si)**2/rs;path.append(p)
    add+=(ge-gi)**2/.4+vgs*diode(vgs)+vgd*diode(vgd)+(vds**2+vgs**2+vgd**2)/(1e11/1545)
    gate-=(ge-vs)*d[f'i(vg{j})'];gates.append(ge-vs);vdslist.append(vds)
    terminal+=(vd-vs)*(vd-vi)/rd-(ge-vs)*d[f'i(vg{j})']
    device_energy+=energy_device(vgs[-1],vgd[-1],-vds[-1])-energy_device(vgs[0],vgd[0],-vds[0])
path=np.array(path).T;loss=path.sum(axis=1)+add
curr=np.loadtxt(R/(method+'_bridge_current.csv'),delimiter=',');currents=np.stack([np.interp(t-start,curr[:,0],curr[:,i]) for i in [1,2,3]],axis=1)
transfer=sum(d[f'v({ph})']*currents[:,k] for k,ph in enumerate('abc'))
records=np.loadtxt(R/(method+'_gates.csv'),delimiter=',');base=records[:,0]+start;cols=[1,3,5,2,4,6];splits=[]
for width in [25e-9,50e-9,100e-9]:
    totals={'on_W':0.,'reverse_W':0.,'switching_W':0.,'off_W':0.}
    for j in range(6):
        events=base[1:][(np.diff(records[:,cols[j]])!=0)|(np.diff(records[:,cols[(j+3)%6]])!=0)]
        events=events[(events>=start-width)&(events<=end)]
        tt=np.unique(np.r_[t,events[(events>=start)&(events<=end)],(events+width)[(events+width>=start)&(events+width<=end)]])
        mid=(tt[1:]+tt[:-1])/2;en=.5*(np.interp(tt[:-1],t,path[:,j])+np.interp(tt[1:],t,path[:,j]))*np.diff(tt)
        k=np.searchsorted(events,mid,side='right')-1;sw=(k>=0)&(mid-events[np.maximum(k,0)]<width)
        on=(np.interp(mid,t,gates[j])>2.5)&~sw;rev=(np.interp(mid,t,vdslist[j])<0)&~sw&~on
        for name,mask in [('on_W',on),('reverse_W',rev),('switching_W',sw),('off_W',~(on|rev|sw))]:totals[name]+=float(en[mask].sum()/duration)
    splits.append({'window_ns':width*1e9,**totals})
load=json.loads((R/(method+'_cycle_load.json')).read_text())
result={'method':method,'duration':duration,'samples':len(t),'device_loss_W':avg(loss),'additional_W':avg(add),'load_W':load['load_W'],'filter_W':load['filter_W'],'efficiency_budget_pct':100*load['load_W']/(load['load_W']+load['filter_W']+avg(loss)),'dc_replay_W':avg(dc),'gate_W':avg(gate),'current_source_power_W':avg(transfer),'device_storage_change_W':device_energy/duration,'energy_balance_residual_W':avg(dc+gate-transfer-loss)-device_energy/duration,'terminal_balance_residual_W':avg(terminal-loss)-device_energy/duration,'splits':splits,'bridge_rms_A':list(np.sqrt([avg(currents[:,j]**2) for j in range(3)]))}
(args.output/(outtag+'_metrics.json')).write_text(json.dumps(result,indent=2));print(f'{method}: loss={result["device_loss_W"]:.9g} W, estimated efficiency={result["efficiency_budget_pct"]:.6f}%')
# Save sampled waveforms for plotting.
ix=np.unique(np.r_[np.arange(0,len(t),max(1,len(t)//12000)),len(t)-1])
np.savez_compressed(args.output/(outtag+'_overview.npz'),t=t[ix]-start,device_loss=loss[ix],bridge_current=currents[ix],gate_power=gate[ix])
