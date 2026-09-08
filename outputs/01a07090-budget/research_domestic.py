import urllib.request, json, re, html, sys, concurrent.futures
from pathlib import Path
root=Path(__file__).parent/'sources_2026_09_08'
root.mkdir(exist_ok=True)
def get(pair):
    name,url=pair
    try:
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        raw=opener.open(urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0'}),timeout=30).read()
        if raw.startswith(b'%PDF'):
            from pypdf import PdfReader
            p=root/(name+'.pdf');p.write_bytes(raw)
            txt='\n'.join('PAGE '+str(i+1)+'\n'+p.extract_text(extraction_mode='layout') for i,p in enumerate(PdfReader(p).pages))
            links=[]
        else:
            h=raw.decode('utf-8','replace');(root/(name+'.html')).write_text(h,encoding='utf-8')
            links=[(html.unescape(re.sub('<[^>]+>',' ',b)).strip(),html.unescape(a)) for a,b in re.findall(r'<a[^>]*href=[\"\x27]([^\"\x27]+)[\"\x27][^>]*>(.*?)</a>',h,re.S|re.I)]
            h=re.sub(r'<(script|style)\b[^>]*>.*?</\1>','',h,flags=re.S|re.I)
            txt=html.unescape(re.sub('<[^>]+>',' ',h));txt=re.sub(r'[ \t]+',' ',txt);txt=re.sub(r'\n\s*\n','\n',txt)
        (root/(name+'.txt')).write_text(txt,encoding='utf-8')
        (root/(name+'_links.json')).write_text(json.dumps(links,ensure_ascii=False,indent=2),encoding='utf-8')
        return name,len(raw)
    except Exception as e:return name,str(e)
items=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
    for r in pool.map(get,items.items()): print(r)
