"""Tangent-continuous corner fillets and continuous arc-length lookahead."""
import math
import numpy as np
from isv_water_trials.codex_nav.fast_corridor import compiled_within_corridor, compiled_corridor_candidate


def samples_on_line(a,b,spacing=.04):
    return np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/spacing))+1))


def smooth_path(raw, safe):
    """Shorten only validated chords, then quadratic Bezier tangent fillets.

    safe checks dense samples with an extra half-spacing clearance reserve.
    Each corner is reduced independently if its curve leaves the safe corridor.
    A failed corner rejects the control path, rather than silently following an
    unsafe curve or a sharp raw corner. Curves have matching tangent directions
    with adjoining straight sections (G1 / arc-length tangent continuity).
    """
    raw=np.asarray(raw)
    if len(raw)<2:return None
    # Keep the route's corridor decision: a globally clear shortcut must not
    # erase the Gap preference or switch to a different side of an obstacle.
    validate=safe
    delta=np.diff(raw,axis=0)
    segment_length_sq=np.maximum(np.sum(delta*delta,axis=1),1e-12)
    raw_segment_starts=raw[:-1]
    # Conservative spatial rejection only: every point within the exact
    # 0.40 m corridor belongs to at least one expanded segment cell.
    cell_size=.2
    corridor_cells=set()
    for a,b in (zip(raw[:-1],raw[1:]) if compiled_corridor_candidate is None else ()):
        x0=math.floor((min(a[0],b[0])-.400000001)/cell_size)
        x1=math.floor((max(a[0],b[0])+.400000001)/cell_size)
        y0=math.floor((min(a[1],b[1])-.400000001)/cell_size)
        y1=math.floor((max(a[1],b[1])+.400000001)/cell_size)
        corridor_cells.update((x,y) for x in range(x0,x1+1) for y in range(y0,y1+1))
    def maybe_within_corridor(points):
        return all((math.floor(x/cell_size),math.floor(y/cell_size)) in corridor_cells for x,y in points)
    def within_corridor(points):
        if compiled_within_corridor is not None:
            return bool(compiled_within_corridor(np.asarray(points),raw_segment_starts,
                                                 delta,segment_length_sq))
        offset=np.asarray(points)[:,None,:]-raw_segment_starts[None,:,:]
        fraction=np.clip(np.sum(offset*delta,axis=2)/segment_length_sq,0,1)
        distance=np.linalg.norm(offset-fraction[:,:,None]*delta,axis=2)
        return bool(np.all(distance.min(axis=1)<=.40))
    def safe(points):
        if compiled_corridor_candidate is not None:
            return bool(within_corridor(points) and validate(points))
        # A subset of the exact same samples can reject an impossible chord.
        # Any passing candidate still runs the full original safety checks.
        probe=points[np.linspace(0,len(points)-1,min(8,len(points)),dtype=int)]
        if not maybe_within_corridor(probe) or not within_corridor(probe):return False
        return bool(validate(points) and within_corridor(points))
    def safe_chord(a,b):
        if compiled_corridor_candidate is not None:
            return safe(samples_on_line(a,b))
        # Most proposed shortcuts fail the corridor test. Probe the same
        # eight line-sample indices before allocating every dense sample.
        count=max(2,int(np.ceil(np.linalg.norm(b-a)/.04))+1)
        probe_count=min(8,count)
        dx=(b[0]-a[0])/(count-1)
        dy=(b[1]-a[1])/(count-1)
        for k in range(probe_count):
            index=k*(count-1)//(probe_count-1)
            cell=(math.floor((a[0]+dx*index)/cell_size),
                  math.floor((a[1]+dy*index)/cell_size))
            if cell not in corridor_cells:return False
        probe_idx=np.linspace(0,count-1,probe_count,dtype=int)
        probe=a+(b-a)*(probe_idx/(count-1))[:,None]
        if not within_corridor(probe):return False
        return safe(samples_on_line(a,b))
    nodes=[raw[0]];i=0
    while i<len(raw)-1:
        j=len(raw)-1
        if compiled_corridor_candidate is not None:
            while j>i:
                j=compiled_corridor_candidate(raw,raw_segment_starts,delta,segment_length_sq,i,j)
                if safe_chord(raw[i],raw[j]):break
                j-=1
            if j<=i:return None
        else:
            while j>i+1 and not safe_chord(raw[i],raw[j]):j-=1
            if not safe_chord(raw[i],raw[j]):return None
        nodes.append(raw[j]);i=j
    nodes=np.array(nodes);parts=[];last=nodes[0]
    for i in range(1,len(nodes)-1):
        a,b,c=nodes[i-1:i+2];incoming=b-a;outgoing=c-b
        lin,lout=np.linalg.norm(incoming),np.linalg.norm(outgoing)
        trim=min(.8,.45*lin,.45*lout)
        accepted=None
        for _ in range(12):
            entry=b-incoming/lin*trim;exit=b+outgoing/lout*trim
            t=np.linspace(0,1,max(8,int(np.ceil(2*trim/.025))))[:,None]
            curve=(1-t)**2*entry+2*(1-t)*t*b+t*t*exit
            if safe(curve):accepted=curve;break
            trim*=.5
        if accepted is None:return None
        parts.extend([samples_on_line(last,accepted[0])[:-1],accepted[:-1]])
        last=accepted[-1]
    parts.append(samples_on_line(last,nodes[-1]))
    path=np.concatenate(parts)
    path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-9]]
    return path if safe(path) else None


def path_geometry(path):
    delta=np.diff(path,axis=0)
    length=np.linalg.norm(delta,axis=1)
    arc=np.r_[0.,np.cumsum(length)]
    return delta,length,arc


def lookahead(path,position,distance,progress=0.,geometry=None):
    """Project onto segments; monotonically advance by arc length, not index."""
    delta,length,arc=path_geometry(path) if geometry is None else geometry
    fraction=np.clip(np.sum((position-path[:-1])*delta,axis=1)/np.maximum(length**2,1e-12),0,1)
    projection=path[:-1]+fraction[:,None]*delta
    candidate=arc[:-1]+fraction*length
    error=np.linalg.norm(projection-position,axis=1)
    error[candidate<progress-.15]=np.inf
    idx=int(np.argmin(error));progress=max(progress,float(candidate[idx]))
    target_s=min(progress+distance,arc[-1]);idx=min(int(np.searchsorted(arc,target_s,side='right'))-1,len(length)-1)
    target=path[idx]+delta[idx]*(target_s-arc[idx])/max(length[idx],1e-12)
    return target,progress
