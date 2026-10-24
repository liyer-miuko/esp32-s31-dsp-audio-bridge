/* Build an offline single-file UI from local sources. No downloads or devices. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),crypto=require('node:crypto');
const root=__dirname,files=new Set(['console.source.html']);
function local(name){
  const p=path.resolve(root,name);
  if(path.dirname(p)!==root)throw Error('External asset is not allowed: '+name);
  files.add(name);return fs.readFileSync(p,'utf8');
}
let html=local('console.source.html');
html=html.replace(/<link rel="stylesheet" href="([^"]+)">/g,(_,name)=>'<style>\n'+local(name)+'\n</style>');
html=html.replace(/<script src="([^"]+)"><\/script>/g,(_,name)=>{
  const code=local(name);new vm.Script(code,{filename:name});
  if(/<\/script/i.test(code))throw Error('Unsafe inline script text in '+name);
  return '<script>\n'+code+'\n</script>';
});
let scripts=0;for(const s of html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/g)){new vm.Script(s[1]);scripts++;}
if(scripts!==6||/<script[^>]+src=|<link[^>]+href=/.test(html))throw Error('Incomplete standalone build');
// Both public entry points are complete HTML files. The modular template is
// explicitly named *.source.html so it cannot be mistaken for the delivery.
fs.writeFileSync(path.join(root,'dsp webui.html'),html,'utf8');
fs.mkdirSync(path.join(root,'dist'),{recursive:true});fs.writeFileSync(path.join(root,'dist','dsp webui.html'),html,'utf8');
files.add('dsp webui.html');
files.add('dist/dsp webui.html');
const identities=[...files].sort().map(name=>{const b=fs.readFileSync(path.join(root,name));return{path:name,bytes:b.length,sha256:crypto.createHash('sha256').update(b).digest('hex').toUpperCase()};});
const manifest={format:'offline standalone HTML',languages:['en','zh'],scripts,files:identities};
fs.writeFileSync(path.join(root,'dist','build_manifest.json'),JSON.stringify(manifest,null,2)+'\n');
console.log(JSON.stringify({ok:true,languages:manifest.languages,scripts,artifact:identities.find(f=>f.path==='dist/dsp webui.html')}));
