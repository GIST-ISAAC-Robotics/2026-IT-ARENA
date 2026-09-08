import urllib.request,re,concurrent.futures
from pathlib import Path
from pypdf import PdfReader
out=Path(__file__).parent/'sources'
urls={
 'depth_v3':'https://docs.luxonis.com/overview/toplevel-features/depth',
 'ov9782':'https://docs.luxonis.com/hardware/sensors/OV9782',
 'imx214':'https://docs.luxonis.com/hardware/sensors/IMX214'
}
def fetch(item):
 name,url=item
 try:
  op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
  raw=op.open(urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0'}),timeout=35).read()
  if url.endswith('.pdf'):
   path=out/(name+'.pdf');path.write_bytes(raw)
   reader=PdfReader(path);text='\n'.join(f'\nPAGE {i+1}\n'+p.extract_text(extraction_mode='layout') for i,p in enumerate(reader.pages))
  else:
   import html
   text=raw.decode('utf-8','replace')
   text=re.sub(r'<(script|style)\b[^>]*>.*?</\1>','',text,flags=re.S|re.I)
   text=html.unescape(re.sub('<[^>]+>',' ',text))
   text=re.sub(r'[ \t]+',' ',text)
  (out/(name+'.txt')).write_text(text,encoding='utf-8')
  return name,len(raw)
 except Exception as e:return name,str(e)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
 for r in pool.map(fetch,urls.items()):print(r)
