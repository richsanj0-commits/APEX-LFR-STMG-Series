import sys
f=sys.argv[1]
X=[-60.24,-50.24,-40.24,-30.24,-22.62,-12.62,-5.0,5.0,12.62,22.62,30.24,40.24,50.24,60.24]
DEAD={0,8}; FLOOR=250
rows=[list(map(int,l.split(',')[1:])) for l in open(f) if l.startswith('RAW')]
t=[r[0] for r in rows]; v=[r[1:] for r in rows]
lo=[min(c) for c in zip(*v)]; hi=[max(c) for c in zip(*v)]
print('per-sensor min/max:'); print(' '.join(f'S{i}:{lo[i]}-{hi[i]}' for i in range(14)))
def norm(r): return [0 if i in DEAD else max(0,min(1000,((r[i]-lo[i])*1000//max(1,hi[i]-lo[i])-FLOOR)*1000//(1000-FLOOR))) for i in range(14)]
def pos(n):
    s=sum(n); return (sum(n[i]*X[i] for i in range(14))/s if max(n)>400 else None), s
def pos_bin(n):
    a=[i for i in range(14) if n[i]>500]; return sum(X[i] for i in a)/len(a) if a else None
out=[]
for ti,r in zip(t,v):
    n=norm(r); p,s=pos(n); out.append((ti,p,pos_bin(n),s,n))
t0=t[0]
print('\n  t(s)  analog  binary  sum  | normalized bars (0-9)')
for k,(ti,p,pb,s,n) in enumerate(out):
    if k%3: continue
    bars=''.join('x' if i in DEAD else str(min(9,x//100)) for i,x in enumerate(n))
    ps=f'{p:7.1f}' if p is not None else '   lost'; pbs=f'{pb:7.1f}' if pb is not None else '   lost'
    print(f'{(ti-t0)/1000:6.2f} {ps} {pbs} {s:5d} | {bars}')
