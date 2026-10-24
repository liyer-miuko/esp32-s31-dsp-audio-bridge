/* Presentation only. Never translate protocol keys, JSON, input values, or raw
 * device logs. Keep each original English text node so language changes neither
 * rebuild forms nor send commands. New dynamic nodes use the same catalog. */
(() => {
  'use strict';
  const KEY='esp32s31.dspui.language', originals=new WeakMap(),attributes=new WeakMap();
  const normalize=s=>s.replace(/\s+/g,' ').trim();
  const catalog=new Map(Object.entries(window.S31_ZH||{}).map(([a,b])=>[normalize(a),b]));
  const phrases=[...catalog].filter(([a])=>a.length>1&&!/[<>${}]/.test(a)).sort((a,b)=>b[0].length-a[0].length);
  const escape=s=>s.replace(/[.*+?^${}()|[\]\\]/g,'\\$&');
  const pattern=new RegExp(phrases.map(([s])=>escape(s)).join('|'),'g');
  let language='en';
  try {const saved=localStorage.getItem(KEY);language=saved==='en'||saved==='zh'?saved:(navigator.language.toLowerCase().startsWith('zh')?'zh':'en');}catch(_){}
  function t(s){
    s=String(s);if(language==='en')return s;
    const n=normalize(s);if(catalog.has(n))return (s.match(/^\s*/)||[''])[0]+catalog.get(n)+(s.match(/\s*$/)||[''])[0];
    // Explicit catalog phrases preserve variable numbers, IDs, units and names.
    return s.replace(pattern,(match,offset,all)=>{
      if(/[A-Za-z0-9_]/.test(match[0])&&offset&&/[A-Za-z0-9_]/.test(all[offset-1]))return match;
      if(/[A-Za-z0-9_]/.test(match.at(-1))&&/[A-Za-z0-9_]/.test(all[offset+match.length]||''))return match;
      return catalog.get(match)||match;
    });
  }
  const skip=e=>!e||!!e.closest('script,style,textarea,pre,code,[data-no-translate]');
  function text(node){
    if(skip(node.parentElement)||!node.data.trim())return;
    let state=originals.get(node);
    if(!state||node.data!==state.last){state={source:node.data,last:node.data};originals.set(node,state);}
    const translated=t(state.source);state.last=translated;if(node.data!==translated)node.data=translated;
  }
  function attrs(el){
    if(skip(el))return;
    let state=attributes.get(el);if(!state){state={};attributes.set(el,state);}
    for(const key of ['title','placeholder','aria-label'])if(el.hasAttribute(key)){
      const value=el.getAttribute(key);let item=state[key];
      if(!item||value!==item.last)item=state[key]={source:value,last:value};
      const translated=t(item.source);item.last=translated;if(value!==translated)el.setAttribute(key,translated);
    }
  }
  function walk(root){
    if(root.nodeType===3){text(root);return;}
    if(root.nodeType!==1||skip(root))return;
    attrs(root);const walker=document.createTreeWalker(root,NodeFilter.SHOW_ELEMENT|NodeFilter.SHOW_TEXT);let n;
    while((n=walker.nextNode())){if(n.nodeType===3)text(n);else attrs(n);}
  }
  function setLanguage(value,persist=true){
    if(!['en','zh'].includes(value))return;
    language=value;document.documentElement.lang=value==='zh'?'zh-CN':'en';
    const button=document.getElementById('btnLanguage');
    if(button){
      button.textContent=value==='zh'?'简体中文':'English';
      button.lang=value==='zh'?'zh-CN':'en';
      const next=value==='zh'?'Switch to English':'切换为简体中文';
      button.setAttribute('aria-label',next);button.title=next;
    }
    if(persist)try{localStorage.setItem(KEY,value);}catch(_){}
    walk(document.body);document.title=t('Audio Bridge DSP Console');
    window.dispatchEvent(new CustomEvent('s31languagechange',{detail:value}));
  }
  window.S31I18N={t,setLanguage,get language(){return language;}};
  document.addEventListener('DOMContentLoaded',()=>{
    document.getElementById('btnLanguage').addEventListener('click',()=>setLanguage(language==='zh'?'en':'zh'));
    setLanguage(language,false);
    const observer=new MutationObserver(records=>{
      for(const record of records){
        if(record.type==='characterData')text(record.target);
        else if(record.type==='attributes')attrs(record.target);
        else for(const node of record.addedNodes)walk(node);
      }
    });
    observer.observe(document.body,{subtree:true,childList:true,characterData:true,attributes:true,attributeFilter:['title','placeholder','aria-label']});
  },{once:true});
})();
