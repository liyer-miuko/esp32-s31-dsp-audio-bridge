/* BULK v5 decoder. Pure: validate every record before publishing any UI state.
 * Source contract: main/dsp_config.c serialize/deserialize, DSP_CONFIG_BLOB_MAX.
 * Reserved fields are skipped, not repurposed. No transport or parameter writes.
 */
(() => {
  'use strict';
  function decode(bytes,current){
    const v=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),n=bytes.byteLength;
    const fail=why=>{throw Error('Invalid device configuration: '+why);};
    if(n<298||n>2304)fail('length');
    if(v.getUint32(0,true)!==0x44535043)fail('signature');
    const version=v.getUint16(4,true);if(version!==5)fail('unsupported version');
    const preset=v.getUint16(6,true),sampleRate=v.getUint32(8,true);
    if(sampleRate<8000||sampleRate>384000)fail('sample rate');
    const next=JSON.parse(JSON.stringify(current));let off=16;
    const need=count=>{if(off+count>n)fail('record length');};
    const f=at=>{const x=v.getFloat32(at,true);if(!Number.isFinite(x))fail('non-finite value');return x;};
    const boolean=at=>{const x=v.getUint8(at);if(x>1)fail('boolean value');return x;};
    const count=at=>{const x=v.getUint8(at);if(x>15)fail('band count');return x;};
    function bands(number){
      const result=[];
      for(let i=0;i<number;i++){
        need(16);const b={f:f(off),g:f(off+4),q:f(off+8),type:v.getUint8(off+12),on:boolean(off+13)};
        if(b.type>7||b.q<=0||b.f<=0)fail('band value');off+=16;result.push(b);
      }
      return result;
    }
    function xo(){
      need(8);const x={f:f(off),type:v.getUint8(off+4),slope:v.getUint8(off+5),on:boolean(off+6)};
      if(x.f<=0||x.type>1||x.slope>2)fail('crossover value');off+=8;return x;
    }
    for(let i=0;i<2;i++){
      need(8);const lin=f(off);if(lin<0)fail('input gain');
      const input=next.inputs[i];input.gain=Math.round(Math.max(-72,Math.min(12,20*Math.log10(Math.max(lin,1e-6))))*100)/100;
      input.mute=boolean(off+4);input.phase=boolean(off+5);
      const eq=count(off+6),req=count(off+7);off+=8;input.eq=bands(eq);input.req=bands(req);
    }
    for(let i=0;i<2;i++)for(let j=0;j<4;j++){
      need(8);const r={on:boolean(off),src:v.getUint8(off+1),gain:f(off+4)};
      if(r.src>3||r.gain<0||r.gain>1)fail('route value');next.route[i][j]=r;off+=8;
    }
    for(let i=0;i<4;i++){
      need(8);const eq=count(off+2);off+=8;next.outs[i].eq=bands(eq);next.outs[i].hp=xo();next.outs[i].lp=xo();
    }
    need(106);if(off+106!==n)fail('trailing or missing bytes');
    next.master=f(off);off+=4;
    for(let i=0;i<2;i++){
      const port=next.ports[i];port.mode=v.getUint8(off);port.format=v.getUint8(off+1);port.enabled=boolean(off+2);
      port.gain=f(off+4);port.delay=f(off+8);off+=16;port.hp=xo();port.lp=xo();
      if(port.format>2||port.delay<0)fail('port value');
      // Preserve a legacy I2S1 RJ readback for correction; applying it is refused.
    }
    const tail=off;
    next.lim={on:boolean(tail),thresh:f(tail+4),release:f(tail+8)};
    const loud={on:boolean(tail+12),amount:f(tail+16),ref:f(tail+20)};
    // Older v5 peers used an all-zero loudness reservation. A real zero boost
    // with a nonzero reference is valid and must not revert to the default 10 dB.
    if(loud.on||loud.amount||loud.ref)next.loud=loud;
    const comp={on:boolean(tail+24),th:v.getInt8(tail+26),ratio:v.getUint8(tail+27),rel:v.getUint16(tail+28,true)};
    if(comp.on||comp.th||comp.ratio||comp.rel){
      if(comp.th>0||comp.th< -60||comp.ratio<10||comp.ratio>255||comp.rel<10||comp.rel>2000)fail('compressor value');
      next.comp=comp;
    }
    next.precision=v.getUint8(tail+30);if(next.precision>1)fail('precision');
    if(v.getUint8(tail+37)){
      next.hw={delayA:v.getUint16(tail+31,true),delayB:v.getUint16(tail+33,true),pol:boolean(tail+36)};
      if(next.hw.delayA>2000||next.hw.delayB>2000)fail('mute delay');
    }
    return {config:next,sampleRate,preset,version};
  }
  function parseJson(text){
    if(new TextEncoder().encode(text).length>8192)throw Error('Amplifier JSON exceeds 8192 bytes.');
    const parsed=JSON.parse(text);let i=0;
    const ws=()=>{while(/\s/.test(text[i]||'')&&i<text.length)i++;};
    function string(){const start=i++;while(i<text.length){if(text[i]==='\\'){i+=2;continue;}if(text[i++]==='"')return JSON.parse(text.slice(start,i));}throw Error('Invalid amplifier schema.');}
    function value(depth){
      if(depth>12)throw Error('Amplifier JSON is too deeply nested.');ws();
      if(text[i]==='{'){
        i++;ws();const keys=new Set();
        while(text[i]!=='}'){
          const key=string();if(keys.has(key))throw Error('Amplifier JSON contains duplicate keys.');keys.add(key);ws();i++;value(depth+1);ws();
          if(text[i]!==',')break;i++;ws();
        }i++;
      }else if(text[i]==='['){i++;ws();while(text[i]!==']'){value(depth+1);ws();if(text[i]!==',')break;i++;ws();}i++;}
      else if(text[i]==='"')string();
      else while(i<text.length&&!/[\s,}\]]/.test(text[i]))i++;
    }
    value(0);return parsed;
  }
  window.S31Config={decode,parseJson};
})();
