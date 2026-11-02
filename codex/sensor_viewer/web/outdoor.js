/* GPS view is independent of indoor mapping. All command acknowledgements are
   from the ROS owner; a successful HTTP send is not an execution acknowledgement. */
(() => {
'use strict';
const $=id=>document.getElementById(id), C=['#48e5c2','#ffad56','#f1dc6c'], V=['#81b6ff','#e995f1'];
let data=null,lastReceived=0,pending=null,draft=null,adding=false,follow=true,center=null,zoom=19;
const map=$('map'),tiles=new Map();let scheduled=false;
const layerSpec=[['hits','현재 히트포인트'],['accumulated','누적 관측'],['raw','A* 원본'],['smooth','평활 경로'],['prediction','회피 예측'],['preview','현재 출력 예측'],['obstacles','장애물 예측'],['trail','이동 경로'],['boat','현재 위치'],['waypoints','목적지']];
let layers={};try{layers=JSON.parse(localStorage.getItem('outdoor.layers.v1')||'{}')||{};}catch(_){}
const enabled=key=>layers[key]!==false;
for(const [key,label] of layerSpec){const el=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.checked=enabled(key);input.dataset.layer=key;input.onchange=()=>{layers[key]=input.checked;try{localStorage.setItem('outdoor.layers.v1',JSON.stringify(layers));}catch(_){}soon();};el.append(input,document.createTextNode(label));$('layers').append(el);}
const project=(ll,z=zoom)=>{const n=256*2**z,s=Math.sin(ll[0]*Math.PI/180);return [(ll[1]+180)/360*n,(.5-Math.log((1+s)/(1-s))/(4*Math.PI))*n];};
const unproject=(p,z=zoom)=>{const n=256*2**z;return [Math.atan(Math.sinh(Math.PI*(1-2*p[1]/n)))*180/Math.PI,p[0]/n*360-180];};
function canvas(id){const c=$(id),r=c.getBoundingClientRect(),d=Math.min(2,devicePixelRatio||1);if(c.width!==Math.round(r.width*d)||c.height!==Math.round(r.height*d)){c.width=Math.round(r.width*d);c.height=Math.round(r.height*d);}const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return [g,r.width,r.height];}
function soon(){if(!scheduled){scheduled=true;requestAnimationFrame(()=>{scheduled=false;drawMap();});}}
let basemap='satellite';
try{basemap=localStorage.getItem('outdoor.basemap')||'satellite';}catch(_){}
if(!['satellite','street'].includes(basemap))basemap='satellite';
$('basemap').value=basemap;
function attribution(){
 $('mapAttribution').innerHTML=basemap==='satellite'?'Imagery © Esri, Vantor, Earthstar Geographics, GIS User Community · <a href="https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer" target="_blank" rel="noopener">출처</a>':'© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap contributors</a>';
}
$('basemap').onchange=()=>{basemap=$('basemap').value;try{localStorage.setItem('outdoor.basemap',basemap);}catch(_){}attribution();soon();};attribution();
function tile(x,y,z){
 const n=2**z;if(y<0||y>=n)return null;const xx=(x%n+n)%n,key=`${basemap}/${z}/${xx}/${y}`;
 if(!tiles.has(key)){const im=new Image();im.onload=soon;im.onerror=()=>{im.failed=true;soon();};
  im.src=basemap==='satellite'?`https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${xx}?blankTile=false`:`https://tile.openstreetmap.org/${z}/${xx}/${y}.png`;
  tiles.set(key,im);if(tiles.size>384)tiles.delete(tiles.keys().next().value);
 }return tiles.get(key);
}
function drawTile(g,x,y,z,dx,dy,size){
 // Request only visible tiles. While loading or unavailable, crop a parent tile.
 for(let level=z;level>=Math.max(10,z-5);level--){
  const factor=2**(z-level),px=Math.floor(x/factor),py=Math.floor(y/factor),im=tile(px,py,level);
  if(im?.complete&&im.naturalWidth&&!im.failed){const unit=im.naturalWidth/factor;
   g.drawImage(im,(x-px*factor)*unit,(y-py*factor)*unit,unit,unit,dx,dy,size+.5,size+.5);return true;}
 }return false;
}
function drawMap(){const [g,w,h]=canvas('map');if(!center){g.fillStyle='#d0d0d0';g.fillText('브링업 후 GPS 위치가 잡히면 표시됩니다.',24,h/2);return;}const base=project(center),pixel=ll=>{const p=project(ll);return [p[0]-base[0]+w/2,p[1]-base[1]+h/2];};
 if($('tiles').checked){
  const native=Math.min(zoom,basemap==='satellite'?21:19),factor=2**(zoom-native),size=256*factor;
  const sx=Math.floor((base[0]-w/2)/size),sy=Math.floor((base[1]-h/2)/size);let visible=0;
  g.globalAlpha=basemap==='satellite'?.88:.72;
  for(let x=sx;x<=(base[0]+w/2)/size;x++)for(let y=sy;y<=(base[1]+h/2)/size;y++)if(drawTile(g,x,y,native,x*size-base[0]+w/2,y*size-base[1]+h/2,size))visible++;
  g.globalAlpha=1;map.dataset.tiles=visible;map.dataset.zoom=zoom;map.dataset.basemap=basemap;
  if(!visible){g.fillStyle='#bbb';g.font='12px system-ui';g.fillText('배경 지도 수신 대기 · 위치와 경로는 계속 표시',16,20);}
 }
 const metersPerPixel=156543.03392*Math.cos(center[0]*Math.PI/180)/2**zoom,spacing=10/metersPerPixel;g.strokeStyle='#88888830';g.lineWidth=.5;g.beginPath();for(let x=w/2%spacing;x<w;x+=spacing){g.moveTo(x,0);g.lineTo(x,h);}for(let y=h/2%spacing;y<h;y+=spacing){g.moveTo(0,y);g.lineTo(w,y);}g.stroke();
 function line(points,color,dash=[],segmented=false){g.strokeStyle=color;g.lineWidth=2;g.setLineDash(dash);g.beginPath();let prev=null;for(const p of points||[]){const [x,y]=pixel(p);if(!prev||(segmented&&p[2]!==prev[2]))g.moveTo(x,y);else g.lineTo(x,y);prev=p;}g.stroke();g.setLineDash([]);}
 if(data?.origin){const originPixel=pixel(data.origin),mp=156543.03392*Math.cos(data.origin[0]*Math.PI/180)/2**zoom;const point=(p,size)=>{const x=originPixel[0]+p[0]/mp,y=originPixel[1]-p[1]/mp;if(x>=0&&x<w&&y>=0&&y<h)g.rect(x-size/2,y-size/2,size,size);};
  if(enabled('accumulated')){const ttl=data.map_layers?.ttl_s||20;for(let band=0;band<4;band++){g.fillStyle=`rgba(180,180,180,${.55-band*.12})`;g.beginPath();for(const p of data.map_layers?.accumulated||[])if(Math.min(3,Math.floor(p[2]/ttl*4))===band)point(p,2);g.fill();}}
  if(enabled('hits')&&performance.now()-lastReceived<3000){g.fillStyle='#f2e8c9';g.beginPath();for(const p of data.map_layers?.hits||[])point(p,3);g.fill();}
 }
 if(enabled('raw'))line(data?.guidance_raw,'#c5a772',[3,4]);if(enabled('smooth'))line(data?.guidance,'#929292');if(enabled('trail'))line(data?.trail,C[0],[],true);if(enabled('preview'))line(data?.preview,'#b9aaca',[6,5]);if(enabled('prediction'))line(data?.prediction,C[1],[8,5]);
 if(enabled('obstacles')&&performance.now()-lastReceived<3000)for(const track of data?.obstacle_tracks||[]){
  if(!track.path?.length)continue;const [x,y]=pixel(track.path[0]);
  line(track.path,'#ee8f9e',[3,4]);g.strokeStyle='#ee8f9e';g.lineWidth=1;g.beginPath();g.arc(x,y,Math.max(3,track.radius_m/metersPerPixel),0,Math.PI*2);g.stroke();
  const end=pixel(track.path.at(-1)),angle=Math.atan2(end[1]-y,end[0]-x);
  if(track.speed_m_s>.1){g.beginPath();g.moveTo(...end);g.lineTo(end[0]-7*Math.cos(angle-.5),end[1]-7*Math.sin(angle-.5));g.moveTo(...end);g.lineTo(end[0]-7*Math.cos(angle+.5),end[1]-7*Math.sin(angle+.5));g.stroke();}
  g.font='10px system-ui';g.fillStyle='#ee8f9e';g.fillText(track.confirmed?`${track.speed_m_s.toFixed(1)} m/s`:'추적 중',x+7,y-7);
 }
 const points=enabled('waypoints')?(draft??data?.route?.points??[]):[];line(points.length>1?[...points,points[0]]:points,'#b0b0b0',[3,7]);points.forEach((p,i)=>{const [x,y]=pixel(p);g.fillStyle=i===data?.goal&&data?.execute?'#f3b16b':'#e5e5e5';g.beginPath();g.arc(x,y,11,0,Math.PI*2);g.fill();g.fillStyle='#222222';g.font='bold 12px system-ui';g.textAlign='center';g.fillText(String(i+1),x,y+4);});g.textAlign='left';
 const s=data?.state;if(s&&enabled('boat')){const [x,y]=pixel([s.lat,s.lon]);g.save();g.translate(x,y);g.rotate((s.heading_deg||0)*Math.PI/180);g.globalAlpha=performance.now()-lastReceived<3000?1:.35;g.fillStyle=C[0];g.strokeStyle='#202020';g.lineWidth=1.5;g.beginPath();if(s.heading_deg===null){g.arc(0,0,6,0,Math.PI*2);}else{g.moveTo(0,-14);g.lineTo(8,11);g.lineTo(0,6);g.lineTo(-8,11);g.closePath();}g.fill();g.stroke();g.restore();}
 const scale=Math.max(1,Math.round(90*metersPerPixel/5)*5);g.strokeStyle='#eeeeee';g.lineWidth=3;g.beginPath();g.moveTo(20,h-22);g.lineTo(20+scale/metersPerPixel,h-22);g.stroke();g.fillStyle='#eeeeee';g.font='12px system-ui';g.fillText(`${scale} m`,20,h-30);
}
let drag=null;
map.addEventListener('pointerdown',e=>{if(!center)return;map.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,base:project(center),moved:false};});
map.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(Math.hypot(dx,dy)>4)drag.moved=true;if(drag.moved){follow=false;center=unproject([drag.base[0]-dx,drag.base[1]-dy]);soon();}});
map.addEventListener('pointerup',e=>{if(!drag)return;if(!drag.moved&&adding){const r=map.getBoundingClientRect(),b=project(center);draft=draft??structuredClone(data?.route?.points??[]);if(draft.length<100)draft.push(unproject([b[0]+e.clientX-r.left-r.width/2,b[1]+e.clientY-r.top-r.height/2]));renderPoints();soon();}drag=null;});
map.addEventListener('pointercancel',()=>drag=null);
map.addEventListener('wheel',e=>{if(!center)return;e.preventDefault();const r=map.getBoundingClientRect(),offset=[e.clientX-r.left-r.width/2,e.clientY-r.top-r.height/2],b=project(center),at=unproject([b[0]+offset[0],b[1]+offset[1]]);zoom=Math.max(14,Math.min(23,zoom+(e.deltaY<0?1:-1)));const p=project(at);center=unproject([p[0]-offset[0],p[1]-offset[1]]);follow=false;soon();},{passive:false});
function renderPoints(){const root=$('points');root.replaceChildren();(draft??data?.route?.points??[]).forEach((p,i)=>{const s=document.createElement('span');s.className='point';s.textContent=`${i+1} · ${p[0].toFixed(6)}, ${p[1].toFixed(6)}`;root.append(s);});$('addPoint').classList.toggle('active',adding);}
function plot(id,rows,series){const [g,w,h]=canvas(id),pad=40,top=12,bot=h-26;let values=[];for(const row of rows)for(const s of series){const v=s.value(row);if(Number.isFinite(v))values.push(v);}let lo=Math.min(0,...values),hi=Math.max(.1,...values);const margin=(hi-lo)*.1;lo-=margin;hi+=margin;const end=rows.at(-1)?.t||0,start=end-60;const px=t=>pad+(w-pad-10)*(t-start)/60,py=v=>bot-(bot-top)*(v-lo)/(hi-lo);g.font='12px system-ui';g.fillStyle='#d0d0d0';g.strokeStyle='#454545';g.setLineDash([]);for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,y=py(v);g.beginPath();g.moveTo(pad,y);g.lineTo(w-10,y);g.stroke();g.fillText(v.toFixed(2),1,y+4);}for(const t of [-60,-45,-30,-15,0]){const x=px(end+t);g.fillText(String(t),x-6,h-5);g.strokeStyle='#383838';g.beginPath();g.moveTo(x,top);g.lineTo(x,bot);g.stroke();}g.fillText('t [s]',w-36,top+8);
 for(const s of series){g.strokeStyle=s.color;g.lineWidth=2.3;g.setLineDash(s.predicted?[6,4]:[]);g.beginPath();let last=null;for(const row of rows){const v=s.value(row);if(row.t<start)continue;if(!Number.isFinite(v)){last=null;continue;}if(!last||row.t-last.t>1.6)g.moveTo(px(row.t),py(v));else g.lineTo(px(row.t),py(v));last=row;}g.stroke();}g.setLineDash([]);}
function bars(id,items){const [g,w,h]=canvas(id);g.font='12px system-ui';const max=Math.max(.001,...items.map(x=>x.value)),labelWidth=Math.min(w*.58,Math.max(92,...items.map(x=>g.measureText(x.label).width+8)));const step=Math.min(28,(h-16)/Math.max(1,items.length));items.forEach((item,i)=>{const y=10+i*step;g.fillStyle='#d0d0d0';let label=item.label;while(label.length>2&&g.measureText(label).width>labelWidth-5)label=label.slice(0,-2)+'…';g.fillText(label,2,y+12);g.fillStyle=item.color||C[0];const length=Math.max(0,w-labelWidth-48)*item.value/max;g.fillRect(labelWidth,y,Math.max(0,length),Math.max(4,step-10));g.fillStyle='#eeeeee';g.fillText(item.value.toFixed(3),labelWidth+5+length,y+12);});}
function drawOutputBands(){const bands=data?.model?.metrics?.speed_bands||[],best=data?.model?.metrics?.best_speed_band;bars('outputBands',bands.filter(b=>Number.isFinite(b.score)).map(b=>({label:`${b.low_m_s.toFixed(1)}–${b.high_m_s.toFixed(1)} m/s`,value:b.score,color:b.windows<5?'#555':b.low_m_s===best?.low_m_s?V[0]:'#999'})));$('bestOutput').textContent=best?`오차 최소 ${best.low_m_s.toFixed(1)}–${best.high_m_s.toFixed(1)} m/s · 검증 ${best.windows}구간`:'속도 구간별 비중복 검증 5회부터 비교';}
function modelName(m){if(!m)return '미지정';return m.display_name||m.candidate_name||(m.variant!=='기준 보존'&&m.variant)||'기존 모델';}
function speedText(m){const mean=m?.speed_coverage?.mean_m_s,b=m?.metrics?.best_speed_band;return [Number.isFinite(mean)?`${mean.toFixed(2)} m/s`:'자료 부족',b?`${b.low_m_s.toFixed(1)}–${b.high_m_s.toFixed(1)} m/s`:'자료 부족'];}
function modelSpec(m,prefix){const root=$(prefix+'Speed'),values=speedText(m);root.replaceChildren();['평균속도','최저 오차 속도'].forEach((label,i)=>{const cell=document.createElement('div'),small=document.createElement('span'),strong=document.createElement('b');small.textContent=label;strong.textContent=values[i];cell.append(small,strong);if(i===1)cell.title='실측 평면속도 기준 0.2 m/s 구간. 겹치지 않는 3초 검증 5회 이상에서 2초 예측 오차 비교.';else cell.title='모델 비교에 사용한 유효 연속 기록의 평면속도 평균 (전진·횡방향 합성)';root.append(cell);});
 const details=[];if(m){details.push(`지연 ${(m.delay_steps*m.dt_s).toFixed(1)} s`);if(Number.isFinite(m.recording_segments))details.push(`${m.recording_segments}기록 구간`);if(Number.isFinite(m.training_samples))details.push(`${m.training_samples.toLocaleString()}표본`);const e=m.metrics?.horizons?.['2']?.velocity_rmse;if(e)details.push(`2초 RMSE 전진 ${e[0].toFixed(3)} · 횡 ${e[1].toFixed(3)} m/s`);details.push(m.qualified?'검증 통과':'기록 보강');}$(prefix+'Details').textContent=details.join(' · ');}
let lastCatalog='';
function renderModels(m,vm){$('modelState').textContent=modelName(m);$('validationState').textContent=modelName(vm);modelSpec(m,'learning');modelSpec(vm,'validation');const count=m?.candidate_count;$('delay').textContent=Number.isFinite(count)?`비교 후보 ${count}개`:'후보 비교 대기';
 const entries=m?.candidate_catalog||[],key=JSON.stringify(entries);$('candidateSummary').textContent=`비교 후보 ${entries.length||count||0}개 · 전체 / 최근 / 균형 × 선형 / 비선형 × 지연시간`;
 if(key!==lastCatalog){lastCatalog=key;const root=$('candidateRows');root.replaceChildren();if(!entries.length){const row=document.createElement('tr'),cell=document.createElement('td');cell.colSpan=4;cell.textContent=m?'다음 학습부터 전체 후보 표시':'후보 비교 대기';row.append(cell);root.append(row);}for(const entry of entries){const row=document.createElement('tr');row.classList.toggle('selected',entry.selected);for(const text of [entry.name+(entry.incumbent?' · 기존 계수':''),entry.delay_s.toFixed(1),entry.score.toFixed(4),entry.selected?'현재 선택':'']){const cell=document.createElement('td');cell.textContent=text;row.append(cell);}root.append(row);}}
}
function rankingRows(model){const all=model?.ranking||[],rows=all.slice(0,4),base=all.find(x=>x.label.startsWith('기준 보존'));if(base&&!rows.includes(base))return [...rows.slice(0,3),base];return rows;}
function drawCharts(){drawOutputBands();const rows=data?.history||[];plot('inputs',rows,[0,1].map(i=>({color:C[i],value:r=>r.command?.[i]*100})));plot('speed',rows,[0,1].flatMap(i=>[{color:V[i],value:r=>r.measured?.[i]},{color:V[i],predicted:true,value:r=>r.predicted?.[i]}]));plot('yaw',rows,[{color:C[2],value:r=>r.measured?.[2]*180/Math.PI},{color:C[2],predicted:true,value:r=>r.predicted?.[2]*180/Math.PI}]);const m=data?.model;bars('ranking',rankingRows(m).map((x,i)=>({label:x.label,value:x.score,color:i===0?C[0]:'#666666'})));bars('errorsSpeed',[1,2,3].map(s=>({label:`${s}초 전진`,value:m?.metrics?.horizons?.[String(s)]?.velocity_rmse?.[0]||0,color:V[0]})));bars('errorsHeading',[1,2,3].map(s=>({label:`${s}초 방향`,value:m?.metrics?.horizons?.[String(s)]?.heading_rmse_deg||0,color:C[2]})));}
async function request(action,extra={}){try{const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(4000)});const value=await response.json();if(!response.ok)throw Error(value.error||'요청 실패');pending={id:value.request_id,at:performance.now(),action};$('notice').textContent='적용 대기';}catch(e){$('notice').textContent=e.message;}}
for(const id of ['pause','resume','start','stop'])$(id).onclick=()=>request(id,id==='start'?{percent:Number($('percent').value)}:{});
$('applyModel').onclick=()=>request('apply_model');$('clearTrail').onclick=()=>request('clear_trail');$('follow').onclick=()=>{follow=true;if(data?.state)center=[data.state.lat,data.state.lon];soon();};$('tiles').onchange=soon;
$('addPoint').onclick=()=>{adding=!adding;renderPoints();};$('undoPoint').onclick=()=>{draft=draft??structuredClone(data?.route?.points??[]);draft.pop();renderPoints();soon();};$('clearPoints').onclick=()=>{draft=[];renderPoints();soon();};$('savePoints').onclick=()=>request('route',{points:draft??data?.route?.points??[]});
function render(){const s=data.state,m=data.model;$('position').textContent=s?`${s.lat.toFixed(6)}, ${s.lon.toFixed(6)} · ${(s.heading_deg===null?'—':s.heading_deg.toFixed(1))}° · ±${s.position_sigma_m.toFixed(1)} m`:data.state_reason;$('learningState').textContent=data.learning?'학습 중':'학습 중지';const summary=data.recording_summary;$('recordCount').textContent=summary?`누적 ${summary.segments}구간 · ${summary.samples.toLocaleString()}표본`:`기록 ${data.native_frames}표본`;$('bag').textContent=`원본 ${data.bag}`;$('fitState').textContent=data.fit_running?'후보 비교 중':data.learning?'자동 비교':'갱신 중지';$('trial').textContent=data.execute?`실행 중 · 다음 ${data.goal+1} · ${data.arrivals}회 도착`:'정지';$('start').disabled=!(data.validation_model?.qualified&&data.route);$('applyModel').disabled=!m?.qualified||data.execute;const vm=data.validation_model;renderModels(m,vm);
 const speeds=speedText(vm);$('coverage').textContent=vm?`검증 모델 평균 ${speeds[0]} · 최저 오차 ${speeds[1]}`:'검증 모델 미지정';
 if(pending&&data.ack?.id===pending.id){$('notice').textContent=data.ack.ok?'적용됨':data.ack.message;if(data.ack.ok&&pending.action==='route'){draft=null;adding=false;}pending=null;}else if(pending&&performance.now()-pending.at>7000){$('notice').textContent='적용 응답을 확인하지 못했습니다. 현재 상태를 확인하세요.';pending=null;}else if(!pending&&data.error)$('notice').textContent=data.error;
 if(s&&(follow||!center))center=[s.lat,s.lon];renderPoints();soon();drawCharts();}
async function poll(){try{const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(4500)});if(response.ok){const body=await response.json(),next=body.values?.outdoor;if(next&&next.age_s<3){data=next;lastReceived=performance.now();render();}}}catch(_){}if(performance.now()-lastReceived>3500){$('start').disabled=true;soon();}setTimeout(poll,document.hidden?1500:500);}
window.addEventListener('message',e=>{
 if(e.origin!==location.origin)return;
 const frame=['sensorFrame','topicFrame'].map(id=>$(id)).find(f=>e.source===f.contentWindow);if(!frame)return;
 if(e.data?.kind==='sensor-height'&&Number.isFinite(e.data.height))frame.style.height=Math.max(160,Math.min(5000,e.data.height+4))+'px';
 if(frame.id==='sensorFrame'&&e.data?.kind==='sensor-graph')$('topicFrame').contentWindow.postMessage(e.data,location.origin);
});
new ResizeObserver(()=>{soon();drawCharts();}).observe(document.querySelector('main'));
poll();
})();
