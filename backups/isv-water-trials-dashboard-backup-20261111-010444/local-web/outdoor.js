// This function is embedded in outdoor.js and runs only in a browser Worker.
function kabotPcPlannerWorker(){
 'use strict';
 const step=.5,range=12,horizon=10,reserve=.05;
 const rad=Math.PI/180,R=6371000;
 const buffers={free:new Uint8Array(49*49),cost:new Float64Array(49*49),parent:new Int32Array(49*49),closed:new Uint8Array(49*49)};
 const inside=(p,poly)=>{let yes=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const a=poly[i],b=poly[j];if((a[1]>p[1])!==(b[1]>p[1])&&p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])yes=!yes;}return yes;};
 self.onmessage=({data:q})=>{
  const began=performance.now(),halfWidth=(q.dimensions?.[1]||.5)/2,halfLength=(q.dimensions?.[0]||1.1)/2;
  try{
   const lat=q.pose[0],lon=q.pose[1],cosLat=Math.cos(lat*rad),yaw=(90-q.scanPose.heading_deg)*rad,c=Math.cos(yaw),s=Math.sin(yaw);
   const xy=p=>[(p[1]-lon)*rad*R*cosLat,(p[0]-lat)*rad*R];
   const ll=p=>[lat+p[1]/R/rad,lon+p[0]/(R*cosLat)/rad];
   const scanOrigin=xy(q.scanPose.point);
   const target=xy(q.goal),distance=Math.hypot(...target),limited=distance>horizon;
   const goal=limited?target.map(v=>v*horizon/distance):target;
   const result={id:q.id,key:q.key,routeKey:q.routeKey,sentAt:q.sentAt,raw:[],smooth:[],hits:[],status:'',limited,resolution_m:step,range_m:range,input_points:q.rays??q.scan.length,used_points:0,cells:0};
   const finish=status=>{result.status=status;result.compute_ms=performance.now()-began;self.postMessage(result);};
   if(distance<.1){finish('목표 도착');return;}
   // Keep all observed near returns; never average away a nearer obstacle.
   // /scan is already one 2D projection, so no 16-ring cloud is transferred here.
   const obstacles=[],buckets=new Map();
   for(const p of q.scan){const a=p[0]*rad,r=p[1];if(!(r>0&&r<range))continue;
    const aMount=(q.mount?.[2]||0)*rad,bx=r*Math.cos(a+aMount)+(q.mount?.[0]||0),by=r*Math.sin(a+aMount)+(q.mount?.[1]||0);
    if(Math.abs(bx)<halfLength&&Math.abs(by)<halfWidth)continue;
    const v=[scanOrigin[0]+c*bx-s*by,scanOrigin[1]+s*bx+c*by];obstacles.push(v);
    const key=Math.floor(v[0])+','+Math.floor(v[1]);if(!buckets.has(key))buckets.set(key,[]);buckets.get(key).push(v);
   }
   result.used_points=obstacles.length;
   const regions=(q.regions||[]).map(r=>({kind:r.kind,points:r.points.map(xy)})),include=regions.filter(r=>r.kind==='include'),exclude=regions.filter(r=>r.kind==='exclude');
   const regionClear=p=>{
    if(include.length&&!include.some(r=>inside(p,r.points)))return false;
    if(exclude.some(r=>inside(p,r.points)))return false;
    return true;
   };
   const clear=p=>{
    if(!regionClear(p))return false;
    const ix=Math.floor(p[0]),iy=Math.floor(p[1]),limit=(halfWidth+reserve)**2;
    for(let x=ix-1;x<=ix+1;x++)for(let y=iy-1;y<=iy+1;y++)for(const v of buckets.get(x+','+y)||[])if((v[0]-p[0])**2+(v[1]-p[1])**2<limit)return false;
    return true;
   };
   const radius=halfWidth+reserve,limit2=radius*radius;
   const segment=(a,b)=>{
    const dx=b[0]-a[0],dy=b[1]-a[1],length2=dx*dx+dy*dy;
    if(regions.length){const count=Math.max(1,Math.ceil(Math.sqrt(length2)/.1));for(let i=0;i<=count;i++)if(!regionClear([a[0]+dx*i/count,a[1]+dy*i/count]))return false;}
    // Exact point-to-segment clearance replaces repeated 0.1 m radius queries.
    for(let x=Math.floor(Math.min(a[0],b[0])-radius);x<=Math.floor(Math.max(a[0],b[0])+radius);x++)
     for(let y=Math.floor(Math.min(a[1],b[1])-radius);y<=Math.floor(Math.max(a[1],b[1])+radius);y++)
      for(const v of buckets.get(x+','+y)||[]){const t=length2?Math.max(0,Math.min(1,((v[0]-a[0])*dx+(v[1]-a[1])*dy)/length2)):0;
       if((v[0]-a[0]-t*dx)**2+(v[1]-a[1]-t*dy)**2<limit2)return false;}
    return true;
   };
   const start=[0,0];
   if(!clear(start)){finish('출발점 주변 장애물 또는 구역 밖');return;}
   if(!clear(goal)){finish('목표점 주변 장애물 또는 구역 밖');return;}
   if(segment(start,goal)){result.raw=[ll(start),ll(goal)];result.smooth=result.raw;finish('PC 근거리 경로');return;}
   // Fixed 49 x 49 grid; all buffers live off the UI thread.
   const n=49,mid=24,N=n*n,coord=i=>[(i%n-mid)*step,(Math.floor(i/n)-mid)*step];
   const index=p=>Math.round(p[0]/step)+mid+(Math.round(p[1]/step)+mid)*n,si=index(start),gi=index(goal);
   const {free,cost,parent,closed}=buffers;
   cost.fill(Infinity);parent.fill(-1);closed.fill(0);
   for(let i=0;i<N;i++)free[i]=regions.length?(regionClear(coord(i))?1:0):1;
   // Rasterize each obstacle into only its affected grid centres.
   for(const v of obstacles){const x0=Math.max(0,Math.ceil((v[0]-radius)/step)+mid),x1=Math.min(n-1,Math.floor((v[0]+radius)/step)+mid);
    const y0=Math.max(0,Math.ceil((v[1]-radius)/step)+mid),y1=Math.min(n-1,Math.floor((v[1]+radius)/step)+mid);
    for(let y=y0;y<=y1;y++)for(let x=x0;x<=x1;x++)if(((x-mid)*step-v[0])**2+((y-mid)*step-v[1])**2<limit2)free[x+y*n]=0;
   }
   free[si]=1;result.cells=N;
   if(!free[gi]||!segment(coord(gi),goal)){finish('목표 격자 통과 불가');return;}
   const heap=[];
   function push(id,f){const item=[id,f];let i=heap.length;heap.push(item);while(i){const p=(i-1)>>1;if(heap[p][1]<=f)break;heap[i]=heap[p];i=p;}heap[i]=item;}
   function pop(){const root=heap[0],last=heap.pop();if(heap.length){let i=0;while(i*2+1<heap.length){let j=i*2+1;if(j+1<heap.length&&heap[j+1][1]<heap[j][1])j++;if(heap[j][1]>=last[1])break;heap[i]=heap[j];i=j;}heap[i]=last;}return root[0];}
   cost[si]=0;push(si,distance);
   let reached=false;
   while(heap.length){const i=pop();if(closed[i])continue;if(i===gi){reached=true;break;}closed[i]=1;const x=i%n,y=Math.floor(i/n),a=coord(i);
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if(!dx&&!dy)continue;const nx=x+dx,ny=y+dy;if(nx<0||ny<0||nx>=n||ny>=n)continue;const j=nx+ny*n;if(!free[j]||closed[j])continue;
     if(dx&&dy&&(!free[x+dx+y*n]||!free[x+(y+dy)*n]))continue;
     const candidate=cost[i]+Math.hypot(dx,dy)*step;if(candidate>=cost[j])continue;const b=coord(j);if(!segment(a,b))continue;
     cost[j]=candidate;parent[j]=i;push(j,candidate+Math.hypot(b[0]-goal[0],b[1]-goal[1]));
    }
   }
   if(!reached){finish('근거리 통과 경로 없음');return;}
   const path=[];for(let i=gi;i>=0;i=parent[i]){path.push(coord(i));if(i===si)break;}path.reverse();path[0]=start;path[path.length-1]=goal;
   const smooth=[start];let i=0;
   while(i<path.length-1){let j=path.length-1;while(j>i+1&&!segment(path[i],path[j]))j--;if(!segment(path[i],path[j])){finish('격자 연결 통과 불가');return;}smooth.push(path[j]);i=j;}
   result.raw=path.map(ll);result.smooth=smooth.map(ll);finish('PC 근거리 A* 경로');
  }catch(error){self.postMessage({id:q.id,key:q.key,routeKey:q.routeKey,sentAt:q.sentAt,raw:[],smooth:[],status:'PC 경로 오류: '+error.message,compute_ms:performance.now()-began});}
 };
}
/* End PC planner worker. */

/* GPS view is independent of indoor mapping. All command acknowledgements are
   from the ROS owner; a successful HTTP send is not an execution acknowledgement. */
(() => {
'use strict';
const $=id=>document.getElementById(id), C=['#48e5c2','#ffad56','#f1dc6c'], V=['#81b6ff','#e995f1'];
let data=null,lastReceived=0,pending=null,draft=null,adding=true,follow=true,center=null,zoom=19;
let sensorValues={},sensorValuesAt=0,selectedCoordinate=null,pcPosition=null,lastCenterSaved=0;
let pcTelemetry=null,pcTelemetryAt=0,pcPath=null,pcPathAt=0,pcWorker=null,pcWorkerBusy=false,pcQueued=null,pcRequestId=0,pcLastKey='',pcError='PC 센서 연결 대기',pcSnapshot=null;
let pcHits=null,pcHitsKey='';
const pcPlanTimes=[],pcSensorTimes=[];
const pcPaintTimes=[];let pcDirty=false,pcEvents=null,pcSentSequence=-1,pcRouteKey='';
let mapClickMode='route',mapClickQueue=Promise.resolve(),routeDraftCache='',routePointRenderKey='';
try{const saved=JSON.parse(localStorage.getItem('outdoor.route.draft.v1')||'null');if(Array.isArray(saved?.points)&&saved.points.length<=100&&saved.points.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)))draft=saved.points;}catch(_){}
try{const p=JSON.parse(localStorage.getItem('outdoor.map.center')||'null');if(Array.isArray(p)&&p.length===2&&p.every(Number.isFinite))center=p;}catch(_){}
const captureView=new URLSearchParams(location.search).get('capture')==='1';
if(captureView){$('captureTools').hidden=true;for(const id of ['sensorFrame','topicFrame'])$(id).src+='&capture=1';}
const map=$('map'),tiles=new Map();let scheduled=false;
$('start').disabled=true;$('startModelTest').disabled=true;
const pointFollowButton=document.createElement('button');pointFollowButton.id='startPointFollow';pointFollowButton.className='primary';pointFollowButton.textContent='지점 추종 시작';pointFollowButton.disabled=true;$('startModelTest').before(pointFollowButton);
pointFollowButton.onclick=()=>request('start_point_follow',{percent:Number($('percent').value),position_source:virtualGps()?'virtual':'gps'});
const pathFollowButton=document.createElement('button');pathFollowButton.id='startPathFollow';pathFollowButton.className='primary';pathFollowButton.textContent='경로 추종 시작';pathFollowButton.disabled=true;pointFollowButton.after(pathFollowButton);
pathFollowButton.onclick=()=>request('start_point_follow',{tracking:'path',percent:Number($('percent').value),position_source:virtualGps()?'virtual':'gps'});
$('percent').addEventListener('change',()=>{if(data?.execute&&data.navigation_mode==='point_follow')request('point_follow_limit',{percent:Number($('percent').value)});});
document.querySelector('.map-legend').textContent='번호순 경유 · 파랑: 현재 구간 경로 · 점선: 경유 순서 · 빨강: 라이다 · 주황: MPC · 보라: 명령 예측';
const layerSpec=[['boat','현재 위치·방향'],['trail','실제 이동 경로'],['waypoints','목적지'],['hits','라이다 히트'],['memory','누적 히트'],['astar','A* 원본'],['smooth','목표 경로'],['feasible','MPC 목표 궤적'],['prediction','현재 명령 예측']];
let layers={};try{layers=JSON.parse(localStorage.getItem('outdoor.layers.core.v2')||'{}')||{};}catch(_){}
const enabled=key=>layerSpec.some(([name])=>name===key)&&layers[key]!==false;
if(layers.memory===undefined)layers.memory=false;
for(const [key,label] of layerSpec){const el=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.checked=enabled(key);input.dataset.layer=key;input.onchange=()=>{layers[key]=input.checked;try{localStorage.setItem('outdoor.layers.core.v2',JSON.stringify(layers));}catch(_){}soon();};el.append(input,document.createTextNode(label));$('layers').append(el);}
const locationTools=document.createElement('div');locationTools.className='toolbar';locationTools.innerHTML='<button id="pcLocation">PC 위치</button><button id="virtualGps">현재 위치 지정</button><button id="routePoints" class="active">경유지 추가</button><button id="targetPoint">한 지점 교체</button><button id="stopVirtualGps" disabled>가상 GPS 종료</button><span id="virtualGpsInfo">지도 클릭으로 1 → 2 → 3 경유지 추가</span>';
document.querySelector('.map-stage').before(locationTools);
const mapOverlayStatus=document.createElement('span');mapOverlayStatus.id='mapOverlayStatus';mapOverlayStatus.className='map-overlay-status';document.querySelector('#layers').append(mapOverlayStatus);
const pcPlannerStatus=document.createElement('span');pcPlannerStatus.id='pcPlannerStatus';pcPlannerStatus.className='map-overlay-status';pcPlannerStatus.setAttribute('role','status');pcPlannerStatus.textContent='PC 빠른 경로 준비 중';document.querySelector('#layers').append(pcPlannerStatus);
pcPlannerStatus.title='PC 표시 경로 · 2D 스캔 · 반경 12 m · 격자 0.5 m · 다음 목표 최대 10 m · 선박 제어에는 전달하지 않음';
const mapAdvanced=document.createElement('details');mapAdvanced.className='map-advanced';
mapAdvanced.innerHTML='<summary>주행 구역 설정</summary>';
for(const part of [document.querySelector('.test-tools'),$('testRegions')])mapAdvanced.append(part);
const routeEditor=document.createElement('div');routeEditor.className='route-editor';
const routeTools=$('addPoint').closest('.toolbar');routeTools.querySelector('span').id='routeProgress';routeTools.querySelector('span').setAttribute('role','status');
routeEditor.append(routeTools,$('points'));locationTools.after(routeEditor);
$('clearPoints').textContent='새 경로';
document.querySelector('.gps-map').append(mapAdvanced);
document.querySelector('.map-stage').before($('stop').closest('.toolbar'));
function virtualGps(){const v=sensorValues.virtual_gps;return v?.active&&v.age_s<2?v:null;}
function gpsBoat(live,heading){
 const gps=live.gps,velocity=live.gps_velocity;
 if(!gps?.valid_fix||gps.age_s>=3||!Number.isFinite(gps.latitude_deg)||!Number.isFinite(gps.longitude_deg))return null;
 let lat=gps.latitude_deg,lon=gps.longitude_deg,compensation=0;
 const age=gps.age_s+gps.measurement_age_s,velocityAge=velocity?.age_s+velocity?.measurement_age_s;
 // Use Jetson ROS measurement age, not the difference between PC/Jetson clocks.
 // Only bridge a short, fresh GNSS gap; keep recorded fixes unchanged.
 if(Number.isFinite(age)&&age>=0&&age<=1.5&&Number.isFinite(velocityAge)&&velocityAge>=0&&velocityAge<=1.5&&
    Math.abs(gps.measurement_stamp_s-velocity.measurement_stamp_s)<=.25&&
    Number.isFinite(velocity.east_m_s)&&Number.isFinite(velocity.north_m_s)&&
    velocity.velocity_sigma_m_s>0&&velocity.velocity_sigma_m_s<=.5&&Math.hypot(velocity.east_m_s,velocity.north_m_s)<=5){
  compensation=Math.min(age,.5);
  lat+=velocity.north_m_s*compensation/6378137*180/Math.PI;
  lon+=velocity.east_m_s*compensation/(6378137*Math.cos(gps.latitude_deg*Math.PI/180))*180/Math.PI;
 }
 return {lat,lon,heading_deg:heading,position_sigma_m:gps.horizontal_sigma_m??0,measurement_age_s:age,position_compensation_s:compensation};
}
function mapBoat(){
 const live=pcLiveValues()||Object.fromEntries(Object.entries(sensorValues).map(([k,v])=>[k,{...v,age_s:(v.age_s||0)+(performance.now()-sensorValuesAt)/1000}]));
 const h=live.heading,heading=h?.age_s<2&&Number.isFinite(h.heading_deg)?h.heading_deg:null,v=live.virtual_gps?.active&&live.virtual_gps.age_s<2?live.virtual_gps:null;
 if(v?.point)return {lat:v.point[0],lon:v.point[1],heading_deg:heading,position_sigma_m:.5,virtual:true};
 const gps=gpsBoat(live,heading);if(gps)return gps;
 if(data?.state&&performance.now()-lastReceived<3500)return data.state;
 return null;
}
function publishYawTarget(){
 const boat=mapBoat(),points=savedRoutePoints(),index=Math.max(0,Math.min(points.length-1,data?.goal||0)),goal=points[index];
 let target=null;
 if(boat&&goal&&performance.now()-lastReceived<3500){
  const north=(goal[0]-boat.lat)*Math.PI/180*6371000,east=(goal[1]-boat.lon)*Math.PI/180*6371000*Math.cos(boat.lat*Math.PI/180);
  if(Math.hypot(east,north)>.01)target={bearing_deg:(Math.atan2(east,north)*180/Math.PI+360)%360,distance_m:Math.hypot(east,north),index};
 }
 $('sensorFrame').contentWindow.postMessage({kind:'yaw-target',target},location.origin);
}
function renderLocation(redraw=true){
 publishYawTarget();
 const v=virtualGps();$('virtualGps').disabled=false;$('stopVirtualGps').disabled=!v;$('virtualGps').classList.toggle('active',mapClickMode==='position');$('targetPoint').classList.toggle('active',mapClickMode==='target');$('routePoints').classList.toggle('active',mapClickMode==='route');
 $('virtualGps').setAttribute('aria-pressed',String(mapClickMode==='position'));$('targetPoint').setAttribute('aria-pressed',String(mapClickMode==='target'));
 const selected=mapClickMode==='position'?'클릭 즉시 가상 GPS 발행':mapClickMode==='target'?'클릭으로 경로를 한 지점으로 교체 · 저장 후 적용':'지도 클릭으로 경유지 추가 · 번호 순서대로 추종';
 $('virtualGpsInfo').textContent=v?`${selected} · 발행 ${v.point[0].toFixed(6)}, ${v.point[1].toFixed(6)} · 5 Hz`:selected;
 $('virtualGpsInfo').title='/gps/virtual/fix';
 if(!center){const p=savedRoutePoints()[0]||sensorValues.map_reference?.point;if(p)center=[...p];}
 const s=mapBoat();if(s&&(follow||!center))center=[s.lat,s.lon];
 $('position').title=s&&Number.isFinite(s.measurement_age_s)?`GPS 측정 지연 ${(s.measurement_age_s*1000).toFixed(0)} ms · ENU 시각 보정 ${((s.position_compensation_s||0)*1000).toFixed(0)} ms`:'';
 $('position').textContent=s?`${s.virtual?'가상 GPS · ':''}${s.lat.toFixed(6)}, ${s.lon.toFixed(6)} · ${Number.isFinite(s.heading_deg)?s.heading_deg.toFixed(1):'—'}° · ±${Number(s.position_sigma_m||0).toFixed(1)} m`:pcPosition?'PC 위치로 지도 표시 · 배 GPS 대기':center?'이전 지도 위치 · 배 GPS 대기':'PC 위치 확인 중 · 배 GPS 대기';
 if(redraw)soon();
}
function locatePc(){
 if(!navigator.geolocation){$('pcLocation').title='브라우저 위치 서비스를 사용할 수 없습니다';renderLocation();return;}
 $('pcLocation').disabled=true;
 navigator.geolocation.getCurrentPosition(p=>{pcPosition=[p.coords.latitude,p.coords.longitude];if(!mapBoat()){center=[...pcPosition];follow=false;}try{localStorage.setItem('outdoor.map.center',JSON.stringify(pcPosition));}catch(_){}$('pcLocation').disabled=false;$('pcLocation').title=`PC 위치 정확도 ±${Math.round(p.coords.accuracy)} m`;renderLocation();},()=>{$('pcLocation').disabled=false;$('pcLocation').title='브라우저·Windows 위치 권한을 허용하면 Wi-Fi 기반 위치를 사용할 수 있습니다';renderLocation();},{enableHighAccuracy:false,timeout:8000,maximumAge:60000});
}
$('pcLocation').onclick=locatePc;
function setMapClickMode(mode){mapClickMode=mode;adding=mode==='route';definingTestArea=false;selectedCoordinate=null;renderLocation();renderPoints();renderTest();soon();}
$('virtualGps').onclick=()=>setMapClickMode('position');
$('targetPoint').onclick=()=>setMapClickMode('target');
$('routePoints').onclick=()=>setMapClickMode('route');
$('stopVirtualGps').onclick=()=>request('virtual_gps_stop');
const project=(ll,z=zoom)=>{const n=256*2**z,s=Math.sin(ll[0]*Math.PI/180);return [(ll[1]+180)/360*n,(.5-Math.log((1+s)/(1-s))/(4*Math.PI))*n];};
const unproject=(p,z=zoom)=>{const n=256*2**z;return [Math.atan(Math.sinh(Math.PI*(1-2*p[1]/n)))*180/Math.PI,p[0]/n*360-180];};
const mapMetersPerPixel=(latitude,z)=>2*Math.PI*6371000*Math.cos(latitude*Math.PI/180)/(256*2**z);
function canvas(id){const c=$(id),r=c.getBoundingClientRect(),d=Math.min(2,devicePixelRatio||1);if(c.width!==Math.round(r.width*d)||c.height!==Math.round(r.height*d)){c.width=Math.round(r.width*d);c.height=Math.round(r.height*d);}const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return [g,r.width,r.height];}
function soon(){if(!scheduled){scheduled=true;requestAnimationFrame(()=>{scheduled=false;if(pcDirty){pcDirty=false;renderLocation(false);pcStatus();pcShareSensors();pcPaintTimes.push(performance.now());}drawMap();});}}
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
function drawMap(){const [g,w,h]=canvas('map');if(!center){g.fillStyle='#d0d0d0';g.fillText('PC 위치 권한을 허용하면 지도가 표시됩니다.',24,h/2);return;}if(performance.now()-lastCenterSaved>5000){lastCenterSaved=performance.now();try{localStorage.setItem('outdoor.map.center',JSON.stringify(center));}catch(_){}}const base=project(center),pixel=ll=>{const p=project(ll);return [p[0]-base[0]+w/2,p[1]-base[1]+h/2];};
 if($('tiles').checked){
  const native=Math.min(zoom,basemap==='satellite'?21:19),factor=2**(zoom-native),size=256*factor;
  const sx=Math.floor((base[0]-w/2)/size),sy=Math.floor((base[1]-h/2)/size);let visible=0;
  g.globalAlpha=basemap==='satellite'?.88:.72;
  for(let x=sx;x<=(base[0]+w/2)/size;x++)for(let y=sy;y<=(base[1]+h/2)/size;y++)if(drawTile(g,x,y,native,x*size-base[0]+w/2,y*size-base[1]+h/2,size))visible++;
  g.globalAlpha=1;map.dataset.tiles=visible;map.dataset.zoom=zoom;map.dataset.basemap=basemap;
  if(!visible){g.fillStyle='#bbb';g.font='12px system-ui';g.fillText('배경 지도 수신 대기 · 위치와 경로는 계속 표시',16,20);}
 }
 const metersPerPixel=mapMetersPerPixel(center[0],zoom),spacing=10/metersPerPixel;g.strokeStyle='#88888830';g.lineWidth=.5;g.beginPath();for(let x=w/2%spacing;x<w;x+=spacing){g.moveTo(x,0);g.lineTo(x,h);}for(let y=h/2%spacing;y<h;y+=spacing){g.moveTo(0,y);g.lineTo(w,y);}g.stroke();
 function line(points,color,dash=[],segmented=false,width=2){g.strokeStyle=color;g.lineWidth=width;g.setLineDash(dash);g.beginPath();let prev=null;for(const p of points||[]){const [x,y]=pixel(p);if(!prev||(segmented&&p[2]!==prev[2]))g.moveTo(x,y);else g.lineTo(x,y);prev=p;}g.stroke();g.setLineDash([]);}
 const fresh=performance.now()-lastReceived<2500,origin=data?.map_origin||data?.origin;
 if(fresh&&origin){
  const toLatlon=p=>[origin[0]+p[1]/6371000*180/Math.PI,origin[1]+p[0]/(6371000*Math.cos(origin[0]*Math.PI/180))*180/Math.PI];
  function hits(points,color,size,memory=false){g.fillStyle=color;for(const p of points||[]){const [x,y]=pixel(toLatlon(p));if(x<0||y<0||x>w||y>h)continue;g.globalAlpha=memory?Math.max(.05,.35*(1-p[2]/(data.map_layers.ttl_s||20))):1;g.fillRect(x-size/2,y-size/2,size,size);}g.globalAlpha=1;}
  if(enabled('memory'))hits(data?.map_layers?.accumulated,'#ffab85',2,true);
  if(!pcHitsFresh()&&enabled('hits'))hits(data?.map_layers?.hits,'#ff6767',3);
 }
 if(fresh){
  if(!pcWorker&&enabled('astar'))line(data?.display_guidance_raw?.length?data.display_guidance_raw:data?.guidance_raw,'#d6d6d6',[4,5]);
  if(!pcWorker&&enabled('smooth'))line(data?.point_follow_path?.length?data.point_follow_path:data?.display_guidance?.length?data.display_guidance:data?.guidance,'#64a8ff',[],false,2.5);
  if(pcWorker&&data?.execute&&enabled('smooth'))line(data?.point_follow_path?.length?data.point_follow_path:data?.guidance,'#d0d0d0',[7,5],false,1.5);
  if(enabled('feasible'))line(data?.prediction?.length?data.prediction:data?.feasible_path,'#ffad56',[],false,3.5);
  if(enabled('prediction'))line(data?.preview,'#e995f1',[6,4],false,2.5);
 }
 if(pcPath&&performance.now()-pcPathAt<750&&pcLiveValues()){
  if(enabled('astar'))line(pcPath.raw,'#d6d6d6',[4,5]);
  if(enabled('smooth'))line(pcPath.smooth,'#64a8ff',[],false,2.5);
 }
 if(enabled('hits')&&pcHitsFresh()){g.fillStyle='#ff6767';for(const p of pcHits.points){const v=pixel(p);g.fillRect(v[0]-1.5,v[1]-1.5,3,3);}}
 const followControl=data?.execute&&data.navigation_mode==='point_follow'&&data.point_follow?.control;
 $('mapOverlayStatus').textContent=fresh?(followControl?`${data.point_follow.maneuver} · ${followControl}`:data?.feasible_status||data?.preview_status||data?.display_planning?.status||'경로 계산 중'):'지도 데이터 대기';
 $('mapOverlayStatus').title='주황: 현재 자세·전진·횡방향 속도·회전속도·최근 추진 명령에서 모델로 계산한 MPC 후보 궤적 · 보라: 현재 명령 유지 시 3초 예측 · 점선: 기하학적 기준 경로';
 if(data?.planner_compute){const compute=data.planner_compute;$('mapOverlayStatus').textContent+=` · 선박 ${compute.backend}`;$('mapOverlayStatus').title+=` · ${compute.device||compute.reason||compute.backend} · GPU ${compute.gpu_calls||0}회`;}
 if(data?.execute&&data.anticipation){const a=data.anticipation;$('mapOverlayStatus').textContent+=a.available?` · 모델 예측 ${a.horizon_s.toFixed(1)}초${a.intervened?' · 선제 회피':''}`:' · 모델 선제 회피 대기';$('mapOverlayStatus').title+=a.available?` · 모델 ${a.model_id} · 입력 지연 ${a.model_delay_s.toFixed(2)}초 · 관성 응답 ${a.inertia_tau_s.toFixed(2)}초`:` · ${a.reason||'실측 모델/속도 대기'}`;}
 if(enabled('trail'))line(data?.trail,C[0],[],true);
 for(const [i,r] of (definingTestArea||regionDraft!==null?selectedRegions():[]).entries()){
  const color=r.kind==='exclude'?'#ffad56':'#48e5c2';line([...r.points,r.points[0]],color,r.kind==='exclude'?[5,4]:[]);
  g.beginPath();r.points.forEach((p,j)=>{const v=pixel(p);if(j)g.lineTo(...v);else g.moveTo(...v);});g.closePath();g.fillStyle=r.kind==='exclude'?'#ffad5635':'#48e5c20a';g.fill();
  g.fillStyle=color;g.font='12px system-ui';g.fillText(`${r.kind==='exclude'?'제외':'이동'} ${i+1}`,...pixel(r.points[0]));
 }
 if(testArea.length){line(testArea.length>2?[...testArea,testArea[0]]:testArea,'#81b6ff',[4,5]);g.fillStyle='#81b6ff';for(const p of testArea){g.beginPath();g.arc(...pixel(p),4,0,Math.PI*2);g.fill();}}
 const points=enabled('waypoints')?editorPoints():[];line(points,'#b0b0b0',[3,7]);points.forEach((p,i)=>{const [x,y]=pixel(p);g.fillStyle=draft===null&&i===data?.goal&&data?.execute?'#ffc66b':'#ffffff';g.strokeStyle='#111';g.lineWidth=2;g.beginPath();g.arc(x,y,6,0,Math.PI*2);g.fill();g.stroke();g.font='bold 12px system-ui';g.lineWidth=3;g.strokeText(String(i+1),x+8,y-7);g.fillText(String(i+1),x+8,y-7);});
 if(selectedCoordinate){g.save();g.globalAlpha=.6;g.fillStyle='#ffffff';g.strokeStyle='#111';g.lineWidth=1.5;g.beginPath();g.arc(...pixel(selectedCoordinate),8,0,Math.PI*2);g.fill();g.stroke();g.restore();}
 const s=mapBoat();if(s&&enabled('boat')){g.save();g.translate(...pixel([s.lat,s.lon]));g.rotate((s.heading_deg||0)*Math.PI/180);g.globalAlpha=s.virtual||sensorValues.gps?.age_s<3||performance.now()-lastReceived<3000?1:.35;g.fillStyle=s.virtual?'#ffe36b':'#48ffd0';g.strokeStyle='#101010';g.lineWidth=2;g.beginPath();if(s.heading_deg===null){g.arc(0,0,6,0,Math.PI*2);}else{g.moveTo(0,-14);g.lineTo(8,10);g.lineTo(0,6);g.lineTo(-8,10);g.closePath();}g.fill();g.stroke();g.restore();}
 const scale=Math.max(1,Math.round(90*metersPerPixel/5)*5);g.strokeStyle='#eeeeee';g.lineWidth=3;g.beginPath();g.moveTo(20,h-22);g.lineTo(20+scale/metersPerPixel,h-22);g.stroke();g.fillStyle='#eeeeee';g.font='12px system-ui';g.fillText(`${scale} m`,20,h-30);
}
let testArea=[],definingTestArea=false,regionDraft=null,editingRegion=null,lastRegions='';
const selectedRegions=()=>regionDraft??data?.regions??[];
function editRegions(){if(regionDraft===null)regionDraft=structuredClone(data?.regions??[]);}
let drag=null,hoverCoordinate=null,coordinateCopiedAt=0;
function mapCoordinate(e){if(!center)return null;const r=map.getBoundingClientRect(),b=project(center);return unproject([b[0]+e.clientX-r.left-r.width/2,b[1]+e.clientY-r.top-r.height/2]);}
function showCoordinate(p){if(!p)return;hoverCoordinate=p;$('copyCoordinate').disabled=false;$('copyCoordinate').textContent=`${p[0].toFixed(7)}, ${p[1].toFixed(7)} · 복사`;}
async function copyCoordinate(p){
 if(!p)return;const text=`${p[0].toFixed(7)}, ${p[1].toFixed(7)}`,request=++coordinateCopiedAt;
 try{await navigator.clipboard.writeText(text);if(request===coordinateCopiedAt)$('copyCoordinate').textContent=text+' · 복사됨';}
 catch(_){$('copyCoordinate').textContent=text+' · 좌표 선택 후 복사';window.prompt('위도, 경도',text);}
}
$('copyCoordinate').onclick=()=>copyCoordinate(hoverCoordinate);
map.addEventListener('pointerdown',e=>{if(!center||e.button!==0)return;map.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,base:project(center),moved:false};});
map.addEventListener('pointermove',e=>{if(drag){const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(Math.hypot(dx,dy)>4)drag.moved=true;if(drag.moved){follow=false;center=unproject([drag.base[0]-dx,drag.base[1]-dy]);soon();}}showCoordinate(mapCoordinate(e));});
map.addEventListener('pointerup',e=>{if(!drag)return;if(!drag.moved){const p=mapCoordinate(e);showCoordinate(p);copyCoordinate(p);if(definingTestArea){if(testArea.length<32)testArea.push(p);renderTest();soon();}else if(adding){draft=structuredClone(editorPoints());if(draft.length<100)draft.push(p);renderPoints();soon();}else if(mapClickMode==='position'){selectedCoordinate=null;mapClickQueue=mapClickQueue.then(()=>request('virtual_gps_start',{point:p}));}else{selectedCoordinate=null;draft=[p];renderPoints();soon();}renderLocation();}drag=null;});
map.addEventListener('pointercancel',()=>drag=null);
function changeZoom(next,offset=[0,0]){
 if(!center)return;
 const b=project(center),at=unproject([b[0]+offset[0],b[1]+offset[1]]);
 zoom=Math.max(14,Math.min(23,next));const p=project(at);
 center=unproject([p[0]-offset[0],p[1]-offset[1]]);follow=false;
 $('zoomIn').disabled=zoom===23;$('zoomOut').disabled=zoom===14;soon();
}
$('zoomIn').onclick=()=>changeZoom(zoom+1);
$('zoomOut').onclick=()=>changeZoom(zoom-1);
map.addEventListener('wheel',e=>{if(!center)return;e.preventDefault();const r=map.getBoundingClientRect();changeZoom(zoom+(e.deltaY<0?1:-1),[e.clientX-r.left-r.width/2,e.clientY-r.top-r.height/2]);},{passive:false});
function sameRoute(a,b){return Array.isArray(a)&&Array.isArray(b)&&a.length===b.length&&a.every((p,i)=>p.length===2&&b[i].length===2&&p.every((v,j)=>Math.abs(v-b[i][j])<1e-9));}
function savedRoutePoints(){return (data&&performance.now()-lastReceived<3500?data.route?.points:null)??sensorValues.saved_route?.route?.points??pcSnapshot?.route?.points??[];}
function editorPoints(){return draft??savedRoutePoints();}
function routeIssue(points){
 if(!points.length)return '지도에 경유지를 추가하세요';if(points.length>100)return '경유지는 최대 100개입니다';
 for(let i=1;i<points.length;i++){const a=points[i-1],b=points[i],dy=(b[0]-a[0])*Math.PI/180*6371000,dx=(b[1]-a[1])*Math.PI/180*6371000*Math.cos(a[0]*Math.PI/180),d=Math.hypot(dx,dy);if(d<1||d>200)return `${i} ↔ ${i+1}번 경유지 거리를 1~200m로 지정하세요`;}
 return '';
}
function routeDirty(){return draft!==null&&!sameRoute(draft,savedRoutePoints());}
function renderRouteControls(){
 const saved=savedRoutePoints(),dirty=routeDirty(),fresh=lastReceived>0&&performance.now()-lastReceived<3500&&!!data;
 const routeReady=fresh||(sensorValues.saved_route?.available&&sensorValues.saved_route.age_s<3&&performance.now()-sensorValuesAt<3500);
 const reason=!fresh?'젯슨 상태 수신 대기':data?.execute?'주행 중':virtualGps()?'가상 GPS 종료 후 실제 위치로 시작하세요':dirty?'편집한 경로를 저장하세요':saved.length<2?'웨이포인트를 2개 이상 저장하세요':!data?.validation_model?'현재 모델을 적용하세요':'';
 for(const id of ['start','startModelTest']){$(id).disabled=!!reason;$(id).title=reason||'저장 경로로 주행 시작';}
 const followReason=!fresh?'젯슨 상태 수신 대기':data?.execute?'주행 중':dirty?'편집한 경로를 저장하세요':!saved.length?'목적지를 추가하고 경로 저장을 누르세요':'';
 $('startPointFollow').disabled=!!followReason;$('startPointFollow').title=followReason||`${virtualGps()?'가상 GPS':'실제 GPS'}·IMU로 연속 추종 · 위치·목표 변경 즉시 반영 · 주행 정지까지 유지`;
 $('startPathFollow').disabled=!!followReason;$('startPathFollow').title=followReason||'파란 목표 경로를 따라 좌우 출력 제어 · 출력 상한 공유';
 const slamRunning=!!(data?.execute&&data?.model_test?.running);
 const pointRunning=!!(data?.execute&&data?.navigation_mode==='point_follow');
 if(pointRunning&&Number.isFinite(data.point_follow_limit_percent)&&document.activeElement!==$('percent')&&pending?.action!=='point_follow_limit')$('percent').value=data.point_follow_limit_percent;
 const pathRunning=pointRunning&&data.point_follow_tracking==='path';
 $('startPointFollow').textContent=pointRunning&&!pathRunning?(data.point_follow_position_source==='virtual'?'가상 지점 추종 중':'지점 추종 중'):'지점 추종 시작';$('startPointFollow').classList.toggle('running',pointRunning&&!pathRunning);
 $('startPathFollow').textContent=pathRunning?'경로 추종 중':'경로 추종 시작';$('startPathFollow').classList.toggle('running',pathRunning);
 $('startModelTest').textContent=slamRunning?'SLAM 주행 중':'SLAM 주행 시작';
 $('start').textContent=data?.execute&&!slamRunning&&!pointRunning?'순환 주행 중':'순환 주행 시작';
 $('startModelTest').classList.toggle('running',slamRunning);
 $('start').classList.toggle('running',!!data?.execute&&!slamRunning&&!pointRunning);
 if(data?.execute)for(const id of ['start','startModelTest'])$(id).title='현재 주행을 마친 뒤 다시 시작할 수 있습니다. 중지하려면 주행 정지를 누르세요.';
 $('savePoints').textContent=dirty?'경로 저장':saved.length?`경로 저장됨 · ${saved.length}점`:'경로 저장';
 const issue=routeIssue(editorPoints());$('savePoints').disabled=!!data?.execute||!routeReady||!!issue||pending?.action==='route';$('savePoints').title=data?.execute?'현재 주행을 정지한 뒤 새 경로를 저장하세요':issue||(!routeReady?'젯슨 연결 대기':fresh?'번호 순서대로 선박에 저장':'젯슨에 저장 · 주행 노드 실행 시 불러오기');
 $('routeProgress').textContent=dirty?`편집 ${editorPoints().length}지점 · ${data?.execute?'현재 주행 정지 후 저장':issue||'저장 후 경로 추종 시작'}`:pointRunning?(data.point_follow?.status?.startsWith('목표 도착')?`전체 ${saved.length}지점 도착 · 마지막 위치 유지`:`경유지 ${Math.min(saved.length,(data.goal||0)+1)} / ${saved.length} 추종 중`):saved.length?`${saved.length}지점 · 1 → ${saved.length} 순서 · 마지막 위치 유지`:'지도 클릭으로 경유지 추가';
}
function editPoint(index,action){draft=structuredClone(editorPoints());if(action==='remove')draft.splice(index,1);else{const next=index+(action==='up'?-1:1);if(next<0||next>=draft.length)return;[draft[index],draft[next]]=[draft[next],draft[index]];}renderPoints();pcSubmit();soon();}
function renderPoints(){
 const root=$('points'),points=editorPoints(),key=JSON.stringify(points);
 if(key!==routePointRenderKey){routePointRenderKey=key;root.replaceChildren();
 points.forEach((p,i)=>{const row=document.createElement('span');row.className='point';row.dataset.index=i;const label=document.createElement('span');label.textContent=`${i+1} · ${p[0].toFixed(6)}, ${p[1].toFixed(6)}`;row.append(label);
  for(const [action,text,title,disabled] of [['up','↑','앞으로',i===0],['down','↓','뒤로',i===points.length-1],['remove','×','삭제',false]]){const b=document.createElement('button');b.type='button';b.textContent=text;b.dataset.action=action;b.disabled=disabled;b.setAttribute('aria-label',`${i+1}번 경유지 ${title}`);b.onclick=()=>editPoint(i,action);row.append(b);}root.append(row);
 });}
 const encoded=JSON.stringify(draft);if(encoded!==routeDraftCache){routeDraftCache=encoded;try{if(draft===null)localStorage.removeItem('outdoor.route.draft.v1');else localStorage.setItem('outdoor.route.draft.v1',JSON.stringify({points:draft,saved_at:Date.now()}));}catch(_){}}
 $('addPoint').classList.toggle('active',adding);renderRouteControls();
}
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
 const entries=m?.candidate_catalog||[],key=JSON.stringify(entries);$('candidateSummary').textContent=`비교 후보 ${entries.length||count||0}개 · 전체 / 최근 / 균형 / 강건 × 선형 / 비선형 × 지연시간`;
 if(key!==lastCatalog){lastCatalog=key;const root=$('candidateRows');root.replaceChildren();if(!entries.length){const row=document.createElement('tr'),cell=document.createElement('td');cell.colSpan=4;cell.textContent=m?'다음 학습부터 전체 후보 표시':'후보 비교 대기';row.append(cell);root.append(row);}for(const entry of entries){const row=document.createElement('tr');row.classList.toggle('selected',entry.selected);for(const text of [entry.name+(entry.incumbent?' · 기존 계수':''),entry.delay_s.toFixed(1),entry.score.toFixed(4),entry.selected?'현재 선택':'']){const cell=document.createElement('td');cell.textContent=text;row.append(cell);}root.append(row);}}
}
function rankingRows(model){const all=model?.ranking||[],rows=all.slice(0,4),base=all.find(x=>x.label.startsWith('기준 보존'));if(base&&!rows.includes(base))return [...rows.slice(0,3),base];return rows;}
function drawCharts(){drawOutputBands();const rows=data?.history||[];plot('inputs',rows,[0,1].map(i=>({color:C[i],value:r=>r.command?.[i]*100})));plot('speed',rows,[0,1].flatMap(i=>[{color:V[i],value:r=>r.measured?.[i]},{color:V[i],predicted:true,value:r=>r.predicted?.[i]}]));plot('yaw',rows,[{color:C[2],value:r=>r.measured?.[2]*180/Math.PI},{color:C[2],predicted:true,value:r=>r.predicted?.[2]*180/Math.PI}]);const m=data?.model;bars('ranking',rankingRows(m).map((x,i)=>({label:x.label,value:x.score,color:i===0?C[0]:'#666666'})));bars('errorsSpeed',[1,2,3].map(s=>({label:`${s}초 전진`,value:m?.metrics?.horizons?.[String(s)]?.velocity_rmse?.[0]||0,color:V[0]})));bars('errorsHeading',[1,2,3].map(s=>({label:`${s}초 방향`,value:m?.metrics?.horizons?.[String(s)]?.heading_rmse_deg||0,color:C[2]})));}
async function request(action,extra={}){try{const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(4000)});const value=await response.json();if(!response.ok)throw Error(value.error||'요청 실패');if(value.saved&&action==='route'){sensorValues.saved_route={...(sensorValues.saved_route||{}),route:value.route,available:true,age_s:0};draft=null;pending=null;renderPoints();pcSubmit();soon();$('notice').textContent='경로 '+value.route.points.length+'지점 · 젯슨에 저장됨';return;}if(value.simulation){if(value.active&&selectedCoordinate&&extra.point&&sameRoute([selectedCoordinate],[extra.point]))selectedCoordinate=null;renderLocation();$('notice').textContent=value.active?'가상 GPS 발행 중 · 지점 추종에 사용 가능':'가상 GPS 발행 종료';return;}pending={id:value.request_id,at:performance.now(),action};$('notice').textContent='적용 대기';}catch(e){$('notice').textContent=e.message;}}
for(const id of ['pause','resume','start','stop'])$(id).onclick=()=>request(id,id==='start'?{percent:Number($('percent').value)}:{});
function renderTest(){
 $('defineTestArea').classList.toggle('active',definingTestArea);
 const regions=selectedRegions(),dirty=regionDraft!==null;
 $('addTestRegion').disabled=testArea.length<3;
 $('saveTestRegions').disabled=!dirty||!regions.some(r=>r.kind==='include')||!!data?.execute;
 renderRouteControls();
 const t=data?.model_test,m=t?.metrics;
 $('testInfo').textContent=dirty||testArea.length?`편집 중 · ${testArea.length}점 · 구역 저장 후 적용`:t?`${t.running?'시험 중':'시험 종료'} · ${m?.samples||0}개 비교 · 계획 대비 위치 RMSE ${Number.isFinite(m?.rmse_m)?m.rmse_m.toFixed(2)+'m':'대기'} · 저장 경로 · 실제 장애물 회피`:`이동 ${regions.filter(r=>r.kind==='include').length}개 · 제외 ${regions.filter(r=>r.kind==='exclude').length}개 · 저장된 구역`;
 const key=JSON.stringify(regions);if(key!==lastRegions){lastRegions=key;const root=$('testRegions');root.replaceChildren();regions.forEach((r,i)=>{
  const item=document.createElement('div');item.className='region-item '+r.kind;const label=document.createElement('span');label.textContent=`${r.kind==='exclude'?'제외':'이동'} ${i+1} · ${r.points.length}점`;
  const edit=document.createElement('button');edit.textContent='수정';edit.onclick=()=>{testArea=structuredClone(selectedRegions()[i].points);$('regionKind').value=r.kind;editingRegion=i;definingTestArea=true;adding=false;renderTest();soon();};
  const remove=document.createElement('button');remove.textContent='삭제';remove.onclick=()=>{editRegions();regionDraft.splice(i,1);editingRegion=null;testArea=[];renderTest();soon();};item.append(label,edit,remove);root.append(item);
 });}
}
$('defineTestArea').onclick=()=>{definingTestArea=!definingTestArea;adding=false;renderPoints();renderTest();};
$('undoTestArea').onclick=()=>{testArea.pop();renderTest();soon();};$('clearTestArea').onclick=()=>{testArea=[];editingRegion=null;definingTestArea=false;renderTest();soon();};
$('addTestRegion').onclick=()=>{if(testArea.length<3)return;editRegions();const r={kind:$('regionKind').value,points:structuredClone(testArea)};if(editingRegion===null){if(regionDraft.length>=16){$('notice').textContent='구역은 16개까지 지정하세요';return;}regionDraft.push(r);}else regionDraft[editingRegion]=r;testArea=[];editingRegion=null;definingTestArea=false;renderTest();soon();};
$('saveTestRegions').onclick=()=>request('regions',{regions:selectedRegions()});
$('startModelTest').onclick=()=>request('start_model_test',{percent:Number($('percent').value)});

$('applyModel').onclick=()=>request('apply_model');$('clearTrail').onclick=()=>request('clear_trail');$('follow').onclick=()=>{follow=true;if(data?.state)center=[data.state.lat,data.state.lon];soon();};$('tiles').onchange=soon;
$('addPoint').textContent='지도에 경유지 추가';$('addPoint').onclick=()=>setMapClickMode('route');$('undoPoint').onclick=()=>{draft=structuredClone(editorPoints());draft.pop();renderPoints();soon();};$('clearPoints').onclick=()=>{draft=[];setMapClickMode('route');};$('savePoints').onclick=()=>{const points=editorPoints(),issue=routeIssue(points);if(data?.execute||issue){$('notice').textContent=data?.execute?'주행 정지 후 새 경로를 저장하세요':issue;return;}request('route',{points});};
function render(){if(draft!==null&&sameRoute(draft,data.route?.points??[])){draft=null;adding=mapClickMode==='route';}const s=data.state,m=data.model;$('position').textContent=s?`${s.lat.toFixed(6)}, ${s.lon.toFixed(6)} · ${(s.heading_deg===null?'—':s.heading_deg.toFixed(1))}° · ±${s.position_sigma_m.toFixed(1)} m`:data.state_reason;$('learningState').textContent=data.learning?'학습 중':'학습 중지';const summary=data.recording_summary;$('recordCount').textContent=summary?`누적 ${summary.segments}구간 · ${summary.samples.toLocaleString()}표본`:`기록 ${data.native_frames}표본`;$('bag').textContent=`원본 ${data.bag}`;$('fitState').textContent=data.fit_running?'후보 비교 중':data.learning?'자동 비교':'갱신 중지';$('trial').textContent=data.execute?`실행 중 · 다음 ${data.goal+1} · ${data.arrivals}회 도착`:'정지';$('applyModel').disabled=!m||data.execute;$('applyModel').title=!m?'학습 모델 생성 대기':data.execute?'주행 정지 후 적용':m.qualified?'현재 모델을 주행에 적용':(m.reasons||['검증 기록 부족']).join(' · ')+' · 수동 적용 가능';const vm=data.validation_model;renderModels(m,vm);
 const speeds=speedText(vm);$('coverage').textContent=vm?`검증 모델 평균 ${speeds[0]} · 최저 오차 ${speeds[1]}`:'검증 모델 미지정';
 renderTest();if(pending&&data.ack?.id===pending.id){$('notice').textContent=data.ack.ok?'적용됨':data.ack.message;if(data.ack.ok&&pending.action==='regions'){regionDraft=null;renderTest();}if(data.ack.ok&&pending.action==='start_model_test')definingTestArea=false;pending=null;}else if(pending&&performance.now()-pending.at>7000){$('notice').textContent='적용 응답을 확인하지 못했습니다. 현재 상태를 확인하세요.';pending=null;}else if(!pending&&data.error)$('notice').textContent=data.error;
 renderLocation();renderPoints();soon();drawCharts();}
async function poll(){const began=performance.now();try{const response=await fetch('/api/state'+(captureView?'?capture=1':''),{cache:'no-store',signal:AbortSignal.timeout(4500)});if(response.ok){const body=await response.json(),next=body.values?.outdoor;sensorValues=body.values||{};sensorValuesAt=performance.now();if(next)pcSnapshot=next;if(next&&next.age_s<3){data=next;lastReceived=performance.now();render();}else{data=null;renderLocation();renderPoints();}}}catch(_){}if(performance.now()-lastReceived>3500){$('start').disabled=true;$('startModelTest').disabled=true;$('startPointFollow').disabled=true;$('startPathFollow').disabled=true;soon();}setTimeout(poll,Math.max(10,(document.hidden?1500:500)-(performance.now()-began)));}
function pcLiveValues(){
 if(!pcTelemetry||pcTelemetry.error||pcTelemetry.age_s===null)return null;
 const elapsed=(performance.now()-pcTelemetryAt)/1000+pcTelemetry.age_s+(pcTelemetry.request_ms||0)/1000;
 if(elapsed>.65)return null;
 const values={};for(const [key,v] of Object.entries(pcTelemetry.values||{}))values[key]={...v,age_s:v.age_s+elapsed};return values;
}
function pcHz(times,now){while(times.length&&times[0]<now-5000)times.shift();return times.length>1?(times.length-1)/Math.max(1000,now-times[0])*1000:0;}
function pcHitsFresh(){return pcHits&&performance.now()-pcHits.at<750&&(pcLiveValues()?.scan?.age_s??Infinity)<.65;}
function pcUpdateHits(){
 const v=pcLiveValues(),scan=v?.scan,virtual=v?.virtual_gps?.active?v.virtual_gps:null;
 const pose=virtual&&Number.isFinite(scan?.scan_heading_deg)?{point:virtual.point,heading_deg:scan.scan_heading_deg}:scan?.scan_pose;
 if(!pose?.point?.every(Number.isFinite)||!Number.isFinite(pose.heading_deg)||!Array.isArray(scan.polar_deg_m)||scan.age_s>.65){pcHits=null;return;}
 const key=JSON.stringify([scan.stamp_s,virtual?.point]);if(key===pcHitsKey&&pcHits)return;pcHitsKey=key;
 const yaw=(90-pose.heading_deg)*Math.PI/180,c=Math.cos(yaw),s=Math.sin(yaw),mount=pcTelemetry.mount||[0,0,0],aMount=(mount[2]||0)*Math.PI/180;
 const halfLength=(pcTelemetry.dimensions?.[0]||1.1)/2,halfWidth=(pcTelemetry.dimensions?.[1]||.5)/2,R=6371000,rad=Math.PI/180,points=[];
 for(const p of scan.polar_deg_m){if(!(p[1]>0&&p[1]<12))continue;const a=p[0]*rad+aMount,x=p[1]*Math.cos(a)+mount[0],y=p[1]*Math.sin(a)+mount[1];
  if(Math.abs(x)<halfLength&&Math.abs(y)<halfWidth)continue;
  const east=c*x-s*y,north=s*x+c*y;points.push([pose.point[0]+north/R/rad,pose.point[1]+east/(R*Math.cos(pose.point[0]*rad))/rad]);
 }
 pcHits={points,at:performance.now()};
}
function pcStatus(){
 const now=performance.now(),hz=pcHz(pcPlanTimes,now),received=pcHz(pcSensorTimes,now),current=pcPath&&now-pcPathAt<750&&pcLiveValues();
 const paint=pcHz(pcPaintTimes,now);
 pcPlannerStatus.textContent=current?`${pcPath.status} · 계산 ${hz.toFixed(1)} Hz · 표시 ${paint.toFixed(1)} Hz · ${pcPath.compute_ms.toFixed(1)} ms · 수신 ${received.toFixed(1)} Hz · ${pcPath.used_points??0}/${pcPath.input_points??0}점${pcPath.limited?' · 앞 10m':''}${performance.now()-lastReceived>3500?' · 저장 목표':''}`:pcError||'PC 경로 입력 대기';
 // Expose measured counters to capture/diagnostics; repeated renders do not count.
 window.kabotPcPlannerStats={planning_hz:hz,paint_hz:paint,received_hz:received,compute_ms:pcPath?.compute_ms??null,status:pcPlannerStatus.textContent,age_s:pcPath?(now-pcPathAt)/1000:null,sequence:pcTelemetry?.sequence,transport:pcEvents?'push':'poll',visualization_only:true};
}
function pcSubmit(){
 if(!pcWorker)return;
 const v=pcLiveValues(),snapshot=data||pcSnapshot,points=editorPoints(),goal=points[draft!==null||!data?0:Math.min(points.length-1,Math.max(0,snapshot?.goal||0))];
 if(!v||!goal){pcRouteKey='';pcQueued=null;pcPath=null;pcError=!v?'PC 센서 수신 대기':'PC 경로 · 목표 지점 지정 대기';pcStatus();return;}
 const gps=v.virtual_gps?.active?v.virtual_gps:v.gps,h=v.heading,scan=v.scan,boat=gpsBoat(v,h?.heading_deg),pose=gps?.point||(boat?[boat.lat,boat.lon]:null);
 if(!gps||(gps.active!==true&&!gps.valid_fix)||gps.age_s>2||!pose?.every(Number.isFinite)||!Number.isFinite(h?.heading_deg)||h.age_s>.75||!Array.isArray(scan?.polar_deg_m)||scan.age_s>.65){pcError='PC 경로 · GPS/IMU/라이다 최신 데이터 대기';pcPath=null;pcStatus();soon();return;}
 const scanPose=gps.active&&Number.isFinite(scan.scan_heading_deg)?{point:pose,heading_deg:scan.scan_heading_deg}:scan.scan_pose;
 if(!scanPose){pcError='라이다·GPS·IMU 측정 시각 정렬 대기';pcPath=null;pcStatus();soon();return;}
 const regions=snapshot?.regions||[];pcRouteKey=JSON.stringify([goal,regions]);
 const key=JSON.stringify([pcTelemetry.instance_id,gps.stamp_s,h.stamp_s,scan.stamp_s,pcRouteKey]);
 if(key===pcLastKey){pcStatus();return;}
 pcLastKey=key;pcError='';
 pcQueued={id:++pcRequestId,key,routeKey:pcRouteKey,sentAt:performance.now(),pose,heading:h.heading_deg,scanPose,scan:scan.polar_deg_m,rays:scan.rays,goal,regions,mount:pcTelemetry.mount,dimensions:pcTelemetry.dimensions};
 pcDispatch();
}
function pcDispatch(){if(pcWorkerBusy||!pcQueued||!pcWorker)return;pcWorkerBusy=true;const q=pcQueued;pcQueued=null;pcWorker.postMessage(q);}
function pcStart(){
 try{
  const url=URL.createObjectURL(new Blob(['('+kabotPcPlannerWorker.toString()+')()'],{type:'text/javascript'}));pcWorker=new Worker(url);URL.revokeObjectURL(url);
  pcWorker.onmessage=({data:next})=>{pcWorkerBusy=false;if(next.routeKey===pcRouteKey&&performance.now()-next.sentAt<650&&pcLiveValues()){pcPath=next;pcPathAt=performance.now();pcPlanTimes.push(pcPathAt);pcDirty=true;soon();}pcDispatch();};
  pcWorker.onerror=()=>{pcError='PC 작업 스레드 오류 · 새로고침 필요';pcPath=null;pcWorker?.terminate();pcWorker=null;pcStatus();};
 }catch(e){pcError='PC 경로 시작 실패: '+e.message;pcStatus();return;}
 if(typeof EventSource==='function'){
  pcEvents=new EventSource('http://127.0.0.1:18766/events');
  pcEvents.onmessage=e=>{try{pcAccept(JSON.parse(e.data));}catch(_){pcError='PC 센서 데이터 오류';}};
  pcEvents.onerror=()=>{pcError='PC 센서 스트림 재연결 중';pcPath=null;pcHits=null;pcDirty=true;soon();};
 }else pcPoll();
 setInterval(()=>{if(!pcLiveValues()){pcPath=null;pcDirty=true;soon();}},250);
}
function pcAccept(next){
 if(next.age_s!==null&&Number.isFinite(next.pc_sent_at_s))next.age_s+=Math.max(0,Date.now()/1000-next.pc_sent_at_s);
 const changed=next.sequence!==pcTelemetry?.sequence;
 if(changed)pcSensorTimes.push(performance.now());
 pcTelemetry=next;pcTelemetryAt=performance.now();
 if(next.error){pcError='PC 센서 연결 대기';pcPath=null;pcHits=null;pcDirty=true;soon();return;}
 if(document.hidden&&!captureView)return;
 pcUpdateHits();pcSubmit();if(changed){pcDirty=true;soon();}
}
function pcShareSensors(){
 const values=pcLiveValues();
 if(!values){if(pcSentSequence!==-1){pcSentSequence=-1;$('sensorFrame').contentWindow.postMessage({kind:'pc-fast-sensors',values:null},location.origin);}return;}
 if(pcSentSequence===pcTelemetry.sequence)return;
 pcSentSequence=pcTelemetry.sequence;
 $('sensorFrame').contentWindow.postMessage({kind:'pc-fast-sensors',values,sequence:pcTelemetry.sequence},location.origin);
}
document.addEventListener('visibilitychange',()=>{if(!document.hidden&&pcTelemetry){pcSentSequence=-1;pcSubmit();pcDirty=true;soon();}});
async function pcPoll(){
 const began=performance.now();
 if(!document.hidden||captureView){
  try{
   const r=await fetch('http://127.0.0.1:18766/state',{cache:'no-store',signal:AbortSignal.timeout(800)});
   if(!r.ok)throw Error('피드 HTTP '+r.status);const next=await r.json();
   pcAccept(next);
  }catch(e){pcError='PC 빠른 센서 연결 대기';pcPath=null;pcStatus();soon();}
 }
 setTimeout(pcPoll,document.hidden&&!captureView?500:Math.max(5,15-(performance.now()-began)));
}
window.addEventListener('message',e=>{
 if(e.origin!==location.origin)return;
 const frame=['sensorFrame','topicFrame'].map(id=>$(id)).find(f=>e.source===f.contentWindow);if(!frame)return;
 if(!window.kabotCaptureLocked&&e.data?.kind==='sensor-height'&&Number.isFinite(e.data.height))frame.style.height=Math.max(160,Math.min(5000,e.data.height+4))+'px';
 if(frame.id==='sensorFrame'&&e.data?.kind==='sensor-graph')$('topicFrame').contentWindow.postMessage(e.data,location.origin);
});
new ResizeObserver(()=>{soon();drawCharts();}).observe(document.querySelector('main'));

window.kabotCaptureView=(v)=>{
 if(!v)return {width:innerWidth,height:innerHeight,scale:Math.min(devicePixelRatio||1,2),center,zoom,follow,layers,basemap,routeDraft:draft,mapClickMode,tiles:$('tiles').checked,candidates:$('candidateList').open,sensors:$('sensorFrame').contentWindow.kabotCaptureView?.(),topics:$('topicFrame').contentWindow.kabotCaptureView?.(),testArea,regions:selectedRegions(),selectedCoordinate};
 if(v.selectedCoordinate===null)selectedCoordinate=null;
 else if(Array.isArray(v.selectedCoordinate)&&v.selectedCoordinate.length===2&&v.selectedCoordinate.every(Number.isFinite))selectedCoordinate=[...v.selectedCoordinate];
 if(Array.isArray(v.testArea)&&v.testArea.length<=32&&v.testArea.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)))testArea=v.testArea;
 if(Array.isArray(v.regions)&&v.regions.length<=16)regionDraft=v.regions;
 if(Object.hasOwn(v,'routeDraft')&&(v.routeDraft===null||Array.isArray(v.routeDraft)&&v.routeDraft.length<=100&&v.routeDraft.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite))))draft=structuredClone(v.routeDraft);
 if(['position','target','route'].includes(v.mapClickMode)){mapClickMode=v.mapClickMode;adding=mapClickMode==='route';}
 if(Array.isArray(v.center)&&v.center.length===2&&v.center.every(Number.isFinite))center=v.center;
 if(Number.isFinite(v.zoom))zoom=Math.max(10,Math.min(23,v.zoom));
 follow=v.follow!==false;
 for(const [key] of layerSpec)if(typeof v.layers?.[key]==='boolean')layers[key]=v.layers[key];
 for(const input of document.querySelectorAll('[data-layer]'))input.checked=enabled(input.dataset.layer);
 if(['satellite','street'].includes(v.basemap)){basemap=v.basemap;$('basemap').value=basemap;attribution();}
 if(typeof v.tiles==='boolean')$('tiles').checked=v.tiles;
 $('candidateList').open=!!v.candidates;
 $('sensorFrame').contentWindow.kabotCaptureView?.(v.sensors||{});$('topicFrame').contentWindow.kabotCaptureView?.(v.topics||{});renderPoints();soon();drawCharts();
};
window.kabotCaptureDraw=()=>{drawMap();drawCharts();};
async function captureAction(action){
 try{const response=await fetch('/api/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,view:window.kabotCaptureView()})});const body=await response.json();if(!response.ok)throw Error(body.error||'촬영 요청 실패');$('captureStatus').textContent=action==='stop'?'영상 저장 중':'촬영 준비 중';}
 catch(e){$('captureStatus').textContent=e.message;}
}
$('takeScreenshot').onclick=()=>captureAction('screenshot');$('startVideo').onclick=()=>captureAction('start');$('stopVideo').onclick=()=>captureAction('stop');
async function captureStatus(){
 try{const r=await fetch('/api/capture',{signal:AbortSignal.timeout(3000)});if(r.ok){const s=await r.json(),busy=s.phase!=='idle';$('startVideo').disabled=busy;$('stopVideo').disabled=!(s.phase==='recording'||(s.phase==='preparing'&&s.operation==='start'));$('takeScreenshot').disabled=busy&&!s.recording;
 const label={idle:'',preparing:'촬영 준비 중',recording:`녹화 ${s.elapsed_s||0}s · 실제 ${s.capture_fps||0}fps`,saving:'전체·패널 영상 저장 중',unavailable:'촬영 기능 준비 필요'}[s.phase]||'';
 $('captureStatus').textContent=s.error||label||(s.last_path?'저장 완료':'');$('captureStatus').title=s.last_path||'';}}
 catch(_){}setTimeout(captureStatus,1000);
}
if(!captureView)captureStatus();

if(!captureView)locatePc();
pcStart();
poll();
})();
