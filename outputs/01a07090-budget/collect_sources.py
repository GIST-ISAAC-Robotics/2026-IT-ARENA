import urllib.request,re,html,json,concurrent.futures
from pathlib import Path
out=Path(__file__).parent/'sources';out.mkdir(exist_ok=True)
sources={
 'gemini2_devicemart':'https://www.devicemart.co.kr/goods/view?no=15013145',
 'gemini335l_devicemart':'https://www.devicemart.co.kr/goods/view?no=15465324',
 'oakpro_ff_devicemart':'https://www.devicemart.co.kr/goods/view?no=15073612',
 'oakpro_gs_devicemart':'https://www.devicemart.co.kr/goods/view?no=15539204',
 'orbbec_icbanq_catalog':'https://www.icbanq.com/A05_templete/templeteList.do?catg_code=101144&order=&t_idx=28',
 'd455_really':'https://reallystore.net/goods/catalog?code=0001',
 'oaks2_mouser':'https://www.mouser.kr/ProductDetail/Luxonis/OAK-D-S2-FF?qs=Znm5pLBrcAIcaIkFS8zqUw%3D%3D'
}
def fetch(kv):
 k,u=kv
 try:
  op=urllib.request.build_opener(urllib.request.ProxyHandler({}));raw=op.open(urllib.request.Request(u,headers={'User-Agent':'Mozilla/5.0'}),timeout=25).read().decode('utf-8','replace')
  (out/(k+'.html')).write_text(raw,encoding='utf-8')
  clean=re.sub(r'<(script|style)\b[^>]*>.*?</\1>','',raw,flags=re.S|re.I);clean=html.unescape(re.sub('<[^>]+>',' ',clean));clean=re.sub(r'\s+',' ',clean)
  (out/(k+'.txt')).write_text(clean,encoding='utf-8')
  if 'devicemart' in k:
   at=clean.find('판매가 (VAT');text=clean[max(0,at-200):at+1100]
  elif 'icbanq' in k:
   text='\n'.join(re.sub('<[^>]+>',' ',raw[max(0,m.start()-500):m.end()+500]) for m in re.finditer('GEMINI 335 L|Gemini 2 L',raw))
  elif 'really' in k:text=clean[-18000:]
  else:text=clean[-8000:]
  return {'name':k,'url':u,'text':text}
 except Exception as e:return {'name':k,'error':str(e)}
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
 results=list(pool.map(fetch,sources.items()))
(out/'retailer_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
for r in results:print(json.dumps(r,ensure_ascii=False))
