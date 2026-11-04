// 실제 Chrome에서 운영 화면을 열어 표시·버튼을 확인한다. 시험 전용, 저장소 코드가 아니다.
// node page_check.mjs <url> <vehicle> <token> <steps.json> <output-prefix>
import { spawn } from 'node:child_process'
import { mkdtempSync, readFileSync, writeFileSync, rmSync } from 'node:fs'
import { tmpdir } from 'node:os'
import { join } from 'node:path'

const [url, vehicle, token, stepsPath, prefix] = process.argv.slice(2)
const steps = JSON.parse(readFileSync(stepsPath, 'utf8'))
const chrome = 'C:/Program Files/Google/Chrome/Application/chrome.exe'
const profile = mkdtempSync(join(tmpdir(), 'arena-page-'))
const port = 9333
const browser = spawn(chrome, ['--headless=new', '--disable-gpu', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`,
  '--window-size=1100,900', 'about:blank'], { stdio: 'ignore' })
const sleep = ms => new Promise(r => setTimeout(r, ms))
async function target() {
  for (let i = 0; i < 100; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json`)).json()
      const page = list.find(t => t.type === 'page')
      if (page) return page.webSocketDebuggerUrl
    } catch { /* 시작 대기 */ }
    await sleep(100)
  }
  throw new Error('chrome not reachable')
}
const ws = new WebSocket(await target())
await new Promise((ok, no) => { ws.onopen = ok; ws.onerror = no })
let id = 0
const pending = new Map()
const errors = []
ws.onmessage = event => {
  const message = JSON.parse(event.data)
  if (message.id && pending.has(message.id)) { pending.get(message.id)(message); pending.delete(message.id) }
  if (message.method === 'Runtime.exceptionThrown') errors.push(message.params.exceptionDetails.text)
  if (message.method === 'Log.entryAdded' && message.params.entry.level === 'error') errors.push(message.params.entry.text)
}
const send = (method, params = {}) => new Promise(ok => { const n = ++id; pending.set(n, ok); ws.send(JSON.stringify({ id: n, method, params })) })
const evaluate = async expression => {
  const reply = await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true })
  if (reply.result?.exceptionDetails) throw new Error(JSON.stringify(reply.result.exceptionDetails))
  return reply.result?.result?.value
}
const shot = async name => {
  const reply = await send('Page.captureScreenshot', { format: 'png' })
  writeFileSync(`${prefix}_${name}.png`, Buffer.from(reply.result.data, 'base64'))
}
const read = () => evaluate(`({badge:document.getElementById('badge').textContent,cls:document.getElementById('badge').className,
  conn:document.getElementById('conn').textContent,why:document.getElementById('why').textContent,
  race:document.getElementById('race').textContent,
  disabled:Object.fromEntries(['stop','reset','start','prepare','newrace'].map(k=>[k,document.getElementById(k).disabled])),
  commands:document.getElementById('commands').innerText,driving:document.getElementById('driving').innerText,
  speeds:document.getElementById('speeds').innerText,restoreHidden:document.getElementById('restorebox').hidden})`)
const log = []
try {
  await send('Page.enable'); await send('Runtime.enable'); await send('Log.enable')
  await send('Page.navigate', { url })
  await sleep(1200)
  await evaluate(`document.getElementById('vehicle').value=${JSON.stringify(vehicle)};document.getElementById('token').value=${JSON.stringify(token)};document.getElementById('login').click()`)
  await sleep(900)
  for (const step of steps) {
    if (step.click) await evaluate(`document.getElementById(${JSON.stringify(step.click)}).click()`)
    if (step.eval) log.push({ eval: step.eval, value: await evaluate(step.eval) })
    await sleep(step.wait ?? 800)
    const state = await read()
    log.push({ name: step.name, state })
    if (step.shot) await shot(step.name)
  }
} finally {
  writeFileSync(`${prefix}.json`, JSON.stringify({ steps: log, errors }, null, 1))
  ws.close(); browser.kill()
  await sleep(300)
  try { rmSync(profile, { recursive: true, force: true }) } catch { /* 잠금 무시 */ }
}
console.log('errors', errors.length, 'steps', log.length)
