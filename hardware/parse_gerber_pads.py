import re,sys,collections
pads=collections.defaultdict(list)
for fn in sys.argv[1:]:
    txt=open(fn).read()
    cur={}
    for stmt in re.findall(r'%[^%]*%|[^%*]+\*',txt):
        s=stmt.strip()
        m=re.match(r'%TO\.(\w+),(.*)\*%',s)
        if m: cur[m.group(1)]=m.group(2); continue
        if s.startswith('%TD'):
            k=re.match(r'%TD\.?(\w*)',s).group(1)
            if k: cur.pop(k,None)
            else: cur={}
            continue
        m=re.match(r'(?:X(-?\d+))?(?:Y(-?\d+))?D03\*',s)
        if m and 'P' in cur:
            x=int(m.group(1) or 0)/1e6; y=int(m.group(2) or 0)/1e6
            ref,pin,*f=cur['P'].split(',')
            pads[ref].append((pin,f[0] if f else '',cur.get('N',''),x,y,fn.split('-')[-1]))
for ref in sorted(pads,key=lambda r:(re.sub(r'\d','',r),int(re.sub(r'\D','',r) or 0))):
    p=pads[ref]; cx=sum(a[3] for a in p)/len(p); cy=sum(a[4] for a in p)/len(p)
    print(f"{ref:6} c=({cx:7.2f},{cy:7.2f}) layer={p[0][5]} "+" | ".join(f"{a[0]}:{a[1]}={a[2]}" for a in p))
