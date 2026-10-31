from pathlib import Path
import re
import numpy as np

def read_raw(path):
    b=Path(path).read_bytes();pos=b.index(b'Binary:\n')+8;header=b[:pos].decode()
    names=re.findall(r'^\s*\d+\s+(\S+)\s+\S+',header,re.M)
    data=np.frombuffer(b[pos:],dtype='<f8').reshape(-1,len(names)).copy()
    return {n:data[:,i] for i,n in enumerate(names)}

def energy_device(gs,gd,sd):
    si=1545*.00075;so=1545*.0039;sr=1545
    sig=lambda x:1/(1+np.exp(-np.clip(x,-700,700)))
    cgs=lambda u:1.023e-9*si+.5*5.465e-10*si*sig((u-1.604)/.1861)
    cgd=lambda u:1.942e-15*sr+.5*5.465e-10*si*sig((u-1.604)/.1861)+1.178e-13*sr*sig((u+5)/4.314)+7.786e-15*sr*sig((u+17.26)/15.29)+1.742e-15*sr*sig((u+33.26)/.89)
    csd=lambda u:8.330e-11*so+4.297e-11*so*sig((u+14.35)/1.287)+1.155e-10*so*sig((u+20.14)/9.467)+4.647e-12*so*sig((u+81.25)/5.665)
    return sum(np.trapezoid((u:=np.linspace(0,z,20001))*c(u),u) for c,z in [(cgs,gs),(cgd,gd),(csd,sd)])
