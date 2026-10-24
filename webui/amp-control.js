/* Optional core0 amplifier control. Separate from cfg, DSP presets and A5.
 * UART and BLE use the same B8 protocol and ordered sendFrame queue. */
(() => {
  'use strict';
  const MAX_BYTES=8192, encoder=new TextEncoder(), decoder=new TextDecoder('utf-8',{fatal:true});
  const statusNames=['Off','Starting','Ready','I2C fault'];
  const errorNames=['OK','Incorrect length','Invalid value','Busy','Stale transaction','NVS write failed','I2C operation failed','Insufficient memory'];
  const defaults={version:1,bus:{enabled:false,sda:9,scl:10,hz:100000},ports:[0,1].map(port=>({
    port,enabled:false,model:port?'TAS5825M':'TAS5828M',address:76+port,
    functions:tasFunctions()
  }))};
  function tasFunctions(){return {
    startup:[{write:[0,0]},{write:[127,0]},{update:[3,27,10]},{delay_ms:5},{update:[3,3,3]}],
    mute:[{write:[0,0]},{write:[127,0]},{update:[3,8,8]}],
    unmute:[{write:[0,0]},{write:[127,0]},{update:[3,8,0]}]
  };}
  const clone=v=>JSON.parse(JSON.stringify(v));
  let profile=clone(defaults), generation=null, busy=false, nextToken=0, waiting=null;
  let dirty=false, timer=null, status=null;
  const root=document.createElement('section'); root.className='card amp-panel';root.id='ampPanel';
  root.innerHTML=`<div class="amp-heading"><h4>I²C amplifier configuration</h4><span class="amp-badge">Optional</span></div>
    <p>Read the device profile, edit the bus and amplifiers, then apply or save.</p>
    <section class="amp-section" aria-labelledby="ampBusTitle"><h5 id="ampBusTitle">1. Bus connection</h5>
      <p>Set the bus on only for an amplifier wired in I2C mode. The default is off.</p>
      <div class="amp-bus"><label class="amp-enable"><input id="ampBusOn" type="checkbox"> Bus on</label>
        <label>SDA GPIO <input id="ampSda" type="number" min="0" max="63"></label>
        <label>SCL GPIO <input id="ampScl" type="number" min="0" max="63"></label>
        <label>Clock <select id="ampHz"><option value="100000">100 kHz</option><option value="400000">400 kHz</option></select></label></div>
      <p>Default: SDA GPIO9, SCL GPIO10. Use external pull-up resistors at the correct bus voltage.</p>
    </section>
    <section class="amp-section" aria-labelledby="ampPortsTitle"><h5 id="ampPortsTitle">2. Amplifiers</h5>
      <div id="ampPortCards" class="amp-grid"></div>
    </section>
    <details class="amp-advanced"><summary>Custom amplifier JSON</summary>
      <p>Use write, update, check, and delay_ms operations. Values are decimal integers. Addresses are 7-bit.</p>
      <p>Each port needs startup, mute, and unmute functions. Add up to four named user functions.</p>
      <p>The TAS templates set basic startup and mute registers. Add board-specific coefficients from the amplifier configuration.</p>
      <textarea id="ampJson" rows="16" spellcheck="false" aria-label="Amplifier JSON"></textarea></details>
    <section class="amp-section amp-device-actions" aria-labelledby="ampApplyTitle"><h5 id="ampApplyTitle">3. Apply to the device</h5>
      <div class="btnrow"><button id="ampRead">Read amplifier configuration</button><button id="ampApply">Apply temporarily</button>
        <button id="ampSave" class="amp-primary">Save for startup</button></div>
      <p>Apply temporarily changes RAM only. Save for startup also saves the amplifier profile to NVS.</p>
      <p id="ampStatus" role="status">Disconnected. No amplifier configuration has been read.</p>
    </section>
    <div class="amp-file-actions"><span>Amplifier files</span><div class="btnrow"><button id="ampExport">Export to file</button>
      <button id="ampImport">Import from file</button><input id="ampFile" type="file" accept=".json,application/json" hidden></div></div>
    <p>Amplifier settings and files are separate from DSP settings and presets.</p>
    <details class="amp-storage"><summary>Reset saved amplifier settings</summary>
      <p>Remove only the saved amplifier profile. The current RAM settings stay unchanged.</p>
      <button id="ampClear">Remove saved amplifier profile</button></details>
    <p>Automatic mute follows UAC or Bluetooth PCM activity and the mute delays above. Digital-zero PCM is active.</p>`;
  document.getElementById('tp-ports').appendChild(root);
  const style=document.createElement('style');style.textContent=`
    .amp-panel{margin-top:28px;padding:22px}.amp-heading{display:flex;align-items:center;gap:12px;flex-wrap:wrap}.amp-heading h4{font-size:18px;margin:0}
    .amp-badge{font-size:11px;color:var(--acc);background:var(--tabind-bg);padding:3px 9px;border-radius:6px}
    .amp-panel p{font-size:12px;line-height:1.7;color:var(--dim);margin:8px 0}
    .amp-section{margin-top:20px;padding-top:20px;border-top:1px solid var(--line)}.amp-section h5{font-size:13px;margin:0 0 10px;color:var(--txt)}
    .amp-bus{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;margin:16px 0 10px}.amp-bus label{display:flex;flex-direction:column;gap:7px;align-items:stretch;font-size:12px}
    .amp-bus label.amp-enable{grid-column:1/-1;flex-direction:row;align-items:center;font-weight:650}
    .amp-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}.amp-grid>article{min-width:0;padding:18px;border:1px solid var(--line);border-radius:12px;background:var(--glass)}
    .amp-port-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:14px}.amp-port-head h4{font-size:14px;margin:0}
    .amp-port-head label{display:flex;gap:8px;align-items:center;font-size:12px}
    .amp-fields{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,.65fr);gap:12px}.amp-fields label{display:flex;flex-direction:column;gap:6px;font-size:12px;min-width:0}
    .amp-template{display:flex;align-items:flex-end;gap:8px;margin:16px 0}.amp-template label{display:flex;flex-direction:column;gap:6px;flex:1;min-width:0;font-size:12px}
    .amp-panel input:not([type=checkbox]),.amp-panel select,.amp-panel textarea{min-width:0;width:100%;box-sizing:border-box;border:1px solid var(--line-hi);border-radius:9px;background-color:var(--inp-bg);color:var(--txt);padding:8px 10px;font:inherit;font-size:13px}
    .amp-panel button{border:1px solid var(--line-hi);border-radius:9px;background:var(--btn-bg);color:var(--txt);padding:9px 12px;font:inherit;font-size:12px;cursor:pointer}
    .amp-panel button:hover{background:var(--btn-bg-hi)}.amp-panel button:disabled{opacity:.5;cursor:default}
    .amp-panel button.amp-primary{background:var(--acc);border-color:var(--acc);color:var(--bg0)}[data-theme=light] .amp-panel button.amp-primary{color:#fff}
    #ampJson{box-sizing:border-box;width:100%;font:12px/1.5 monospace;margin:10px 0;resize:vertical}
    .amp-panel .btnrow{display:flex;flex-wrap:wrap;gap:8px;margin:12px 0}.amp-panel button{white-space:normal;min-height:38px}
    .amp-panel details{margin-top:18px}.amp-panel summary{cursor:pointer;font-size:12px;color:var(--acc)}.amp-panel details[open] summary{margin-bottom:12px}
    #ampStatus{padding:12px 14px;border:1px solid var(--line);border-radius:9px;background:var(--soft)}
    .amp-file-actions{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-top:18px;border-top:1px solid var(--line);padding-top:8px;font-size:12px;color:var(--dim)}
    @media(max-width:700px){.amp-panel{padding:17px 13px}.amp-grid{grid-template-columns:minmax(0,1fr)}.amp-fields{grid-template-columns:minmax(0,1fr) 100px}.amp-template{align-items:stretch;flex-wrap:wrap}.amp-template label{flex-basis:100%}.amp-template button{width:100%}.amp-file-actions{display:block}.amp-file-actions .btnrow>button{flex:1}.amp-device-actions .btnrow>button{flex:1 1 120px}}
    @media(max-width:420px){.amp-bus{grid-template-columns:repeat(2,minmax(0,1fr))}.amp-bus>label:last-child{grid-column:1/-1}}`;
  document.head.appendChild(style);
  const el=id=>document.getElementById(id);
  function message(text,error=false){el('ampStatus').textContent=text;el('ampStatus').style.color=error?'var(--err)':'';}
  function lockUi(value){
    busy=value;
    for(const e of root.querySelectorAll('input,button,select,textarea'))e.disabled=value;
    if(!value) for(const b of root.querySelectorAll('[data-amp-run]'))b.disabled=!transportActive()||generation===null;
  }
  function render(){
    el('ampBusOn').checked=profile.bus.enabled;el('ampSda').value=profile.bus.sda;
    el('ampScl').value=profile.bus.scl;el('ampHz').value=profile.bus.hz;
    el('ampJson').value=JSON.stringify(profile,null,2);
    const cards=el('ampPortCards');cards.replaceChildren();
    for(const port of profile.ports){
      const p=document.createElement('article');
      p.innerHTML=`<div class="amp-port-head"><h4>Port ${port.port} / I2S${port.port}</h4>
        <label>Amplifier on <input type="checkbox" data-field="enabled"></label></div>
        <div class="amp-fields"><label>Model <input type="text" data-field="model" maxlength="23"></label>
          <label>Address (decimal) <input type="number" data-field="address" min="8" max="119"></label></div>
        <div class="amp-template"><label>Basic template <select data-template-select><option>TAS5828M</option><option>TAS5825M</option></select></label><button type="button" data-load-template>Load template</button></div>
        <div class="amp-functions"></div><p id="ampPortState${port.port}">No device status.</p>`;
      for(const field of ['enabled','model','address']){
        const input=p.querySelector(`[data-field="${field}"]`);
        if(field==='enabled')input.checked=port[field];else input.value=port[field];
        input.onchange=()=>{port[field]=field==='enabled'?input.checked:field==='address'?+input.value:input.value;changed();};
      }
      const template=p.querySelector('[data-template-select]');
      if(['TAS5828M','TAS5825M'].includes(port.model))template.value=port.model;
      p.querySelector('[data-load-template]').onclick=()=>{
        port.model=template.value;port.functions=tasFunctions();changed();render();
      };
      const names=Object.keys(port.functions).filter(k=>!['startup','mute','unmute'].includes(k));
      names.forEach((name,index)=>{
        const b=document.createElement('button');b.type='button';b.dataset.ampRun='1';b.textContent='Run '+name;
        b.onclick=()=>operation(async()=>{
          if(dirty)throw Error('Apply or read the amplifier profile before you run a function.');
          const token=tokenId();await request(5,new Uint8Array([port.port,index+3]),token);
          const end=Date.now()+8000;
          for(;;){const s=await readStatus();if(s.lastToken===token){if(s.lastResult)throw Error(errorNames[s.lastResult]);break;}
            if(Date.now()>end)throw Error('The function result is not confirmed. Read device status.');await delay(150);}
          message('Function completed. Automatic PCM mute was restored.');
        });p.querySelector('.amp-functions').appendChild(b);
      });cards.appendChild(p);
    }
    renderStatus();lockUi(busy);
  }
  function changed(){dirty=true;el('ampJson').value=JSON.stringify(profile,null,2);message('Local amplifier changes. Select Apply temporarily or Save for startup.');}
  function parseEditor(){
    const p=S31Config.parseJson(el('ampJson').value);
    if(p.version!==1||!p.bus||!Array.isArray(p.ports)||p.ports.length!==2)throw Error('Invalid amplifier schema.');
    if(typeof p.bus.enabled!=='boolean'||![p.bus.sda,p.bus.scl,p.bus.hz].every(Number.isInteger))throw Error('Invalid bus values.');
    if(![100000,400000].includes(p.bus.hz)||p.bus.sda===p.bus.scl)throw Error('Invalid I2C clock or pins.');
    if(new Set(p.ports.map(x=>x.port)).size!==2||!p.ports.every(x=>[0,1].includes(x.port)&&typeof x.enabled==='boolean'&&
      typeof x.model==='string'&&/^[A-Za-z0-9_-]{1,23}$/.test(x.model)&&Number.isInteger(x.address)&&x.address>=8&&x.address<=119&&
      x.functions&&['startup','mute','unmute'].every(k=>Array.isArray(x.functions[k])&&x.functions[k].length)))throw Error('Invalid port or function.');
    if(p.ports.every(x=>x.enabled)&&p.ports[0].address===p.ports[1].address)throw Error('The two active amplifiers need different addresses.');
    const bytes=encoder.encode(JSON.stringify(p));if(bytes.length>MAX_BYTES)throw Error('Amplifier JSON exceeds 8192 bytes.');
    return p;
  }
  function tokenId(){nextToken=nextToken%255+1;return nextToken;}
  function request(op,data=new Uint8Array(),token=tokenId()){
    if(waiting)return Promise.reject(Error('Another amplifier request is pending.'));
    if(!transportActive())return Promise.reject(Error('Connect serial or BLE first.'));
    const p=new Uint8Array(3+data.length);p.set([0xB8,op,token]);p.set(data,3);
    return new Promise((resolve,reject)=>{
      const timeout=setTimeout(()=>{if(waiting?.token===token){waiting=null;reject(Error('No amplifier reply. Read the device configuration before you try again.'));}},4000);
      waiting={op,token,resolve,reject,timeout};
      if(!sendFrame(p)){clearTimeout(timeout);waiting=null;reject(Error('The amplifier request was not queued.'));}
    });
  }
  function receive(p){
    if(p[0]!==0xB8||p.length<4)return false;
    if(waiting&&p[1]===(waiting.op|0x80)&&p[2]===waiting.token){
      const w=waiting;waiting=null;clearTimeout(w.timeout);
      if(p[3]!==0)w.reject(Error(errorNames[p[3]]||'Unknown amplifier error'));else w.resolve(p.slice());
    }return true;
  }
  async function readStatus(){
    const p=await request(0);if(p.length!==55||p[4]!==1)throw Error('Unsupported amplifier status.');
    const v=new DataView(p.buffer,p.byteOffset,p.byteLength);
    const s={generation:v.getUint32(5,true),applied:v.getUint32(9,true),busy:!!p[13],lastToken:p[14],lastResult:p[15],playing:!!p[16],muted:!!p[17],persisted:!!p[54],ports:[]};
    for(let i=0;i<2;i++){const o=18+i*18;s.ports.push({state:p[o],muted:p[o+1],error:v.getInt32(o+2,true),writes:v.getUint32(o+6,true),reads:v.getUint32(o+10,true),jobs:v.getUint32(o+14,true)});}
    status=s;renderStatus();return s;
  }
  function renderStatus(){if(!status)return;status.ports.forEach((s,i)=>{
    const line=el('ampPortState'+i);if(line)line.textContent=(statusNames[s.state]||'Unknown')+' / mute '+(s.muted===2?'unknown':s.muted?'on':'off')+
      ' / error '+s.error+' / writes '+s.writes+' / reads '+s.reads;
  });}
  async function readProfile(){
    let offset=0,bytes=null,gen=null;
    do{
      const arg=new Uint8Array([offset&255,offset>>8]);const p=await request(1,arg);
      if(p.length<12)throw Error('Short amplifier configuration reply.');
      const v=new DataView(p.buffer,p.byteOffset,p.byteLength),total=v.getUint16(4,true),off=v.getUint16(6,true),g=v.getUint32(8,true);
      if(total>MAX_BYTES||!total||off!==offset||(!bytes?false:total!==bytes.length||gen!==g)||p.length-12>total-off)throw Error('Inconsistent amplifier configuration.');
      if(!bytes){bytes=new Uint8Array(total);gen=g;}
      if(p.length===12&&offset<total)throw Error('Empty amplifier configuration chunk.');
      bytes.set(p.slice(12),offset);offset+=p.length-12;
    }while(offset<bytes.length);
    el('ampJson').value=decoder.decode(bytes);profile=parseEditor();generation=gen;dirty=false;render();
    const s=await readStatus();
    if(s.lastToken===0&&s.lastResult)message('Saved amplifier profile was not loaded. Disabled defaults are active. Check NVS or profile data.',true);
    else message('Amplifier configuration read. DSP settings are unchanged.');
  }
  async function apply(persist){
    if(generation===null)throw Error('Read the amplifier configuration first.');
    profile=parseEditor();const bytes=encoder.encode(JSON.stringify(profile)),token=tokenId();
    const arg=new Uint8Array(6);const v=new DataView(arg.buffer);v.setUint16(0,bytes.length,true);v.setUint32(2,generation,true);
    await request(2,arg,token);
    for(let off=0;off<bytes.length;off+=240){const chunk=bytes.slice(off,off+240),data=new Uint8Array(2+chunk.length);
      data[0]=off&255;data[1]=off>>8;data.set(chunk,2);await request(3,data,token);}
    const committed=await request(4,new Uint8Array([persist?1:0]),token);
    if(committed.length!==8)throw Error('Invalid commit reply.');
    generation=new DataView(committed.buffer,committed.byteOffset).getUint32(4,true);dirty=false;render();
    const end=Date.now()+8000;
    for(;;){const s=await readStatus();if(!s.busy){
      if(s.applied!==generation||s.ports.some(x=>x.state===3||x.error!==0))throw Error('The profile was accepted, but amplifier I2C setup failed. Check device status.');break;}
      if(Date.now()>end)throw Error('The profile was queued. Application is not confirmed. Read device status.');await delay(150);}
    message(persist?'Amplifier profile applied and saved. DSP settings are unchanged.':'Amplifier profile applied to RAM. NVS is unchanged.');
  }
  const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
  async function operation(fn){if(busy)return;lockUi(true);try{await fn();}catch(e){message(e.message,true);}finally{lockUi(false);}}
  function cancel(){
    if(waiting){clearTimeout(waiting.timeout);const w=waiting;waiting=null;w.reject(Error('The control connection changed.'));}
    generation=null;status=null;message('Control connection changed. Read the amplifier configuration.');
  }
  for(const [id,field] of [['ampBusOn','enabled'],['ampSda','sda'],['ampScl','scl'],['ampHz','hz']])el(id).onchange=()=>{
    profile.bus[field]=field==='enabled'?el(id).checked:+el(id).value;changed();
  };
  el('ampJson').onchange=()=>{try{profile=parseEditor();dirty=true;render();message('Local amplifier JSON loaded. Device settings are unchanged.');}catch(e){message(e.message,true);}};
  el('ampRead').onclick=()=>operation(readProfile);
  el('ampApply').onclick=()=>operation(()=>apply(false));el('ampSave').onclick=()=>operation(()=>apply(true));
  el('ampClear').onclick=()=>operation(async()=>{
    await request(6);await readStatus();message('Saved amplifier profile removed. RAM is unchanged. The next start uses disabled defaults.');
  });
  el('ampExport').onclick=()=>{try{const p=parseEditor(),a=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(p,null,2)],{type:'application/json'}));
    a.href=url;a.download='s31-amplifiers.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),0);}catch(e){message(e.message,true);}};
  el('ampImport').onclick=()=>el('ampFile').click();el('ampFile').onchange=async e=>{
    const f=e.target.files[0];if(!f)return;if(f.size>MAX_BYTES){message('The file exceeds 8192 bytes.',true);return;}
    try{el('ampJson').value=await f.text();profile=parseEditor();dirty=true;render();message('Local amplifier file loaded. Device settings are unchanged.');}catch(err){message(err.message,true);}finally{e.target.value='';}
  };
  window.ampUI={receive,cancel,read:()=>operation(readProfile),defaults:clone(defaults),parseEditor};
  render();
})();
