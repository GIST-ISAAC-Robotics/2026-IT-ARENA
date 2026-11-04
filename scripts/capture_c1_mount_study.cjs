// node scripts/capture_c1_mount_study.cjs <playwright module path> <chromium executable>
const path = require('node:path');
const {pathToFileURL} = require('node:url');
const assert = require('node:assert/strict');
const {chromium} = require(process.argv[2] || 'playwright');
(async () => {
  const out=path.join(__dirname,'../artifacts/validation/2026-11-03/lidar_mount_study_v1');
  const browser=await chromium.launch({executablePath:process.argv[3],headless:true});
  try {
    const page=await browser.newPage({viewport:{width:1280,height:900},deviceScaleFactor:1.35});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.goto(pathToFileURL(path.join(out,'index.html')).href);
    assert.equal(await page.locator('#cards .card').count(),6);
    assert.equal(await page.locator('#gridstats tr').count(),15);
    for (const r of [...require(path.join(out,'results.json')).layouts,...require(path.join(out,'stack.json')).layouts]){
      await page.selectOption('#preset',r.id);
      const actual=await page.locator('#metrics').innerText();
      assert.equal(actual,'전체 사각 '+r.total_deg.toFixed(1)+'°　/　전방 ±100° 안 '+(r.sector_deg?.front100 ?? r.front100).toFixed(1)+'°');
    }
    await page.selectOption('#preset','E2');
    await page.locator('#diam').fill('10');
    assert.match(await page.locator('#metrics').innerText(),/24\.3°/);
    await page.selectOption('#preset','J0');
    await page.locator('#pos').fill('50');
    assert.match(await page.locator('#metrics').innerText(),/48\.9°/);
    await page.locator('#printplate').screenshot({path:path.join(out,'designs.png')});
    assert.equal(await page.locator('#jcards .card').count(),6);
    assert.equal(await page.locator('#packplate svg').count(),2);
    for(const [id,basename] of [['jplate','j_forward_comparison'],['packplate','low_c1_pdb_stack']])
      await page.locator('#'+id).screenshot({path:path.join(out,basename+'.png')});
    await page.setViewportSize({width:390,height:850});
    assert(await page.evaluate(()=>document.body.scrollWidth<=window.innerWidth));
    assert.deepEqual(errors,[]);
    console.log('21 presets match Python; diameter and fixed rear face checked; 3 sheets captured; mobile width fits.');
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
