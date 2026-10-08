const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync('drone_inspetor/mobile_web/video.js', 'utf8');
const flush = async () => { for(let i=0;i<15;i++) await Promise.resolve(); };
function fixture(handler, preference='auto') {
  const elements = new Map(), calls=[], peers=[], timers=new Map(), revoked=[];
  let now=0;
  const el=(id)=>{
    if(!elements.has(id)) elements.set(id, {hidden:true,textContent:'',value:preference,
      srcObject:null,readyState:0,currentTime:0,open:false,
      addEventListener(name,callback){this[name]=callback;},
      removeAttribute(name){delete this[name];},play:async()=>{}});
    return elements.get(id);
  };
  class Peer {
    constructor(){ peers.push(this); this.iceGatheringState='complete'; }
    addTransceiver(kind,opts){assert.equal(kind,'video'); assert.equal(opts.direction,'recvonly');}
    async createOffer(){return {type:'offer',sdp:'SDP'};}
    async setLocalDescription(value){this.localDescription=value;}
    async setRemoteDescription(value){this.remoteDescription=value;}
    close(){this.closed=true;}
  }
  const context={active:true,stream:'camera',frames:{camera:{available:true}}};
  const notices=[];
  const scope={window:{RTCPeerConnection:Peer},document:{getElementById:el},
    performance:{now:()=>now},URL:{createObjectURL:()=> 'blob:test',revokeObjectURL:u=>revoked.push(u)},
    MediaStream:class{},setTimeout:cb=>{const id={};timers.set(id,cb);return id;},clearTimeout:id=>timers.delete(id)};
  vm.runInNewContext(source,scope);
  const video=new scope.window.DroneVideo({getContext:()=>context,getRequest:()=>async(path,opts)=>{
    calls.push({path,opts});return handler(path,opts);
  },notice:text=>notices.push(text)});
  return {video,el,context,calls,peers,timers,revoked,notices,advance:n=>now+=n};
}
const response=path=> path.endsWith('/video') ? {webrtc:true} : path.endsWith('/offer') ? {session_id:'one',type:'answer',sdp:'answer'} : {closed:true};
test('JPEG consulta apenas a câmera ativa e oculta respostas atrasadas',async()=>{
  let resolve;
  const f=fixture(()=>new Promise(r=>resolve=r),'jpeg');
  f.video.tick();
  assert.deepEqual(f.calls.map(x=>x.path),['/api/v1/frame/camera']);
  f.context.active=false;f.video.tick();
  resolve({type:'image/jpeg'});await flush();
  assert.equal(f.el('frame-camera').hidden,true);
  assert.equal(f.video.session,null);
});
test('automático recua para JPEG; escolha explícita WebRTC mantém erro visível',async()=>{
  const handler=path=>path.endsWith('/video')?{webrtc:false,reason:'Desativado'}:{type:'image/jpeg'};
  for(const preference of ['auto','webrtc']) {
    const f=fixture(handler,preference);f.video.tick();await flush();
    assert.equal(f.calls.some(x=>x.path.includes('/frame/')),preference==='auto');
    assert.equal(f.el('frame-camera').hidden,preference==='webrtc');
    f.video.stop();
  }
});
test('trocar câmera fecha peer e sessão; oferta atrasada também é encerrada',async()=>{
  let offer;
  const f=fixture(path=>path.endsWith('/offer')?new Promise(r=>offer=r):response(path));
  f.video.tick();await flush();
  f.context.active=false;f.video.tick();
  assert.equal(f.peers[0].closed,true);
  offer({session_id:'late',type:'answer',sdp:'SDP'});await flush();
  const close=f.calls.find(c=>c.path.endsWith('/close'));
  assert.equal(JSON.parse(close.opts.body).session_id,'late');
  assert.equal(f.video.session,null);
});
test('fonte vencida e vídeo congelado não são exibidos como imagem atual',async()=>{
  const f=fixture(response,'webrtc');f.video.tick();await flush();
  const video=f.el('video-camera');video.readyState=2;video.currentTime=1;
  f.video.tick();assert.equal(video.hidden,false);
  f.context.frames.camera.available=false;
  f.video.tick();assert.equal(video.hidden,true);
  f.context.frames.camera.available=true;
  f.advance(3100);f.video.tick();
  assert.equal(video.hidden,true);
  assert.equal(f.peers[0].closed,true);
  assert.match(f.el('frame-camera-note').textContent,/sem atualização/);
  f.video.stop();
});
test('ampliação compartilha o mesmo stream sem uma segunda negociação',async()=>{
  const f=fixture(response);f.video.tick();await flush();
  const media={};
  f.peers[0].ontrack({streams:[media]});
  const video=f.el('video-camera');video.readyState=2;video.currentTime=1;
  f.video.tick();f.el('video-dialog').open=true;f.video.expanded();
  assert.equal(f.el('expanded-video').srcObject,media);
  assert.equal(f.el('expanded-video').hidden,false);
  assert.equal(f.peers.length,1);
  f.video.stop();
});
