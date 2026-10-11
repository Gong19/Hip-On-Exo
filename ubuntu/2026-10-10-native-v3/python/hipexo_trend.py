"""Display-only extrema envelope. Never used by native capture or CSV recording."""
import numpy as np

def trend_points(x,y,limit=160):
    x=np.asarray(x);y=np.asarray(y,dtype=float)
    if len(x)!=len(y):raise ValueError('Display axes must have equal lengths')
    if limit<6:raise ValueError('At least six display points required')
    n=len(y)
    if n<=limit:return x,y
    width=int(np.ceil((n-2)/((limit-2)//2)))
    indices=[]
    # Preserve both chronological extrema in each bucket, and explicit gaps.
    for start in range(1,n-1,width):
        stop=min(n-1,start+width);a=y[start:stop]
        if not np.isfinite(a).all():
            indices.append(start+int(np.flatnonzero(~np.isfinite(a))[0]))
        else:
            indices.extend(sorted(set([start+int(np.argmin(a)),start+int(np.argmax(a))])))
    idx=np.asarray([0,*indices,n-1],dtype=int)
    return x[idx],y[idx]
