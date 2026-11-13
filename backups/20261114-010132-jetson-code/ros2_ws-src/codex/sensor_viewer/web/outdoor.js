/* GPS view is independent of indoor mapping. All command acknowledgements are
   from the ROS owner; a successful HTTP send is not an execution acknowledgement. */
(() => {
'use strict';
const $=id=>document.getElementById(id), C=['#48e5c2','#ffad56','#f1dc6c'], V=['#81b6ff','#e995f1'];
let draftTypes=null,draftHold=null,draftOutputs=null,draftMission=null,draftOrbits=null,lastPointsKey='',selectedWaypoint=-1;
let placingWaypoint=-1;
let insertionIndex=null;
const waypointMoveHistory=[];
let data=null,lastReceived=0,pending=null,draft=null,adding=false,follow=true,center=null,zoom=19,captureUi=null;
let sensorValues={},selectedCoordinate=null,lastCenterSaved=0,heldMapBoat=null,heldMapHeading=null;
let trailAfter=0;
try{const value=Number(localStorage.getItem('outdoor.trail.after.v1'));if(Number.isFinite(value))trailAfter=value;}catch(_){}
function setTrailAfter(value){
 if(!Number.isFinite(value)||value<=trailAfter)return;
 trailAfter=value;try{localStorage.setItem('outdoor.trail.after.v1',String(value));}catch(_){}
}
function visibleTrail(rows){return trailAfter?rows.filter(p=>p.length>3&&Number.isFinite(p[3])&&p[3]>trailAfter):rows;}
let gpsPaths={active:null,reference:null,error:null},gpsPathBusy=false,gpsPathNotice='';
const gpsPathTools=document.createElement('div');gpsPathTools.className='toolbar gps-path-tools';
gpsPathTools.innerHTML='<button id="newGpsPath">새 GPS 경로 생성</button><button id="saveGpsPath" disabled>GPS 경로 저장</button><select id="gpsPathFiles" aria-label="저장 GPS 경로"><option value="">저장 GPS 경로 선택</option></select><button id="loadGpsPath" disabled>GPS 경로 불러오기</button><button id="hideGpsPath" disabled>참고 경로 숨기기</button><span id="gpsPathStatus" class="gps-path-status" role="status"></span><span id="gpsPathNotice" role="status"></span>';
document.querySelector('.gps-map>.toolbar').after(gpsPathTools);
const stopGpsPath=document.createElement('button');stopGpsPath.id='stopGpsPath';stopGpsPath.textContent='GPS 기록 중지';
$('saveGpsPath').before(stopGpsPath);
function renderGpsPaths(value){
 if(value)gpsPaths=value;
 const active=gpsPaths.active,reference=gpsPaths.reference;
 $('newGpsPath').disabled=gpsPathBusy;$('saveGpsPath').disabled=gpsPathBusy||!active;
 $('stopGpsPath').disabled=gpsPathBusy||active?.recording!==true;
 $('loadGpsPath').disabled=gpsPathBusy||!$('gpsPathFiles').value;$('hideGpsPath').disabled=gpsPathBusy||!reference;
 $('newGpsPath').classList.toggle('running',active?.recording===true);
 $('gpsPathStatus').textContent=active?`GPS 경로 ${active.recording===true?'기록 중':'기록 중지'} · ${Number(active.point_count||0).toLocaleString()}점 · ${active.started_kst?.replace('T',' ').slice(0,19)||''} 시작`:'GPS 기록 꺼짐 · 새 GPS 경로 생성 시 시작';
 $('gpsPathNotice').textContent=gpsPaths.error||gpsPathNotice|| (reference?`노란 참고 경로 · ${reference.saved_kst?.replace('T',' ').slice(0,19)||reference.name}`:'');
 if(!center&&reference?.points?.length)center=reference.points[0].slice(0,2);
}
function fitGpsReference(points){
 if(!points.length)return;
 const lat=points.map(p=>p[0]),lon=points.map(p=>p[1]);
 center=[(Math.min(...lat)+Math.max(...lat))/2,(Math.min(...lon)+Math.max(...lon))/2];follow=false;
 const rect=map.getBoundingClientRect();zoom=19;
 if(points.length>1)for(let z=23;z>=10;z--){const pixels=points.map(p=>project(p,z)),xs=pixels.map(p=>p[0]),ys=pixels.map(p=>p[1]);if(Math.max(...xs)-Math.min(...xs)<rect.width*.8&&Math.max(...ys)-Math.min(...ys)<rect.height*.8){zoom=z;break;}}
}
async function refreshGpsPaths(){
 try{
  const response=await fetch('/api/gps-paths',{cache:'no-store',signal:AbortSignal.timeout(6000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'GPS 경로 목록 수신 실패');
  const select=$('gpsPathFiles'),selected=value.reference?.name||select.value;
  select.replaceChildren(new Option('저장 GPS 경로 선택',''));
  for(const file of value.files||[])select.add(new Option(`${file.saved_kst.replace('T',' ').slice(0,19)} · ${file.point_count.toLocaleString()}점`,file.name));
  if([...select.options].some(option=>option.value===selected))select.value=selected;
  renderGpsPaths(value);soon();
 }catch(error){gpsPathNotice=error.message;renderGpsPaths();}
}
async function gpsPathAction(action,extra={}){
 if(gpsPathBusy)return;
 gpsPathBusy=true;gpsPathNotice=action==='save'?'GPS 경로 저장 중':'GPS 경로 처리 중';renderGpsPaths();
 try{
  const response=await fetch('/api/gps-paths',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(action==='save'?60000:12000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'GPS 경로 요청 실패');
  if(Number.isFinite(value.trail_after_t)){setTrailAfter(value.trail_after_t);if(data)data={...data,trail:[]};}
  renderGpsPaths(value.gps_paths);
  if(action==='load'&&value.reference){
   layers.reference=true;const input=document.querySelector('[data-layer="reference"]');if(input)input.checked=true;
   try{localStorage.setItem('outdoor.layers.core.v2',JSON.stringify(layers));}catch(_){}
   fitGpsReference(value.reference.points||[]);
  }
  gpsPathNotice=value.saved?`저장됨 · 기록 종료 · ${value.saved.name}`:action==='start'?'새 GPS 경로 기록 시작':action==='stop'?'GPS 기록 중지 · 기존 경로는 저장 가능':action==='hide'?'참고 경로 숨김':'노란 참고 경로를 불러왔습니다.';
  if(action==='save')await refreshGpsPaths();
  soon();
 }catch(error){gpsPathNotice=error.message;}
 finally{gpsPathBusy=false;renderGpsPaths();}
}
$('newGpsPath').onclick=()=>gpsPathAction('start');$('saveGpsPath').onclick=()=>gpsPathAction('save');
$('stopGpsPath').onclick=()=>gpsPathAction('stop');
$('loadGpsPath').onclick=()=>gpsPathAction('load',{name:$('gpsPathFiles').value});
$('hideGpsPath').onclick=()=>gpsPathAction('hide');$('gpsPathFiles').onchange=()=>renderGpsPaths();
const mapSubscribers=new Set();
let mapClickMode='browse';
try{const p=JSON.parse(localStorage.getItem('outdoor.map.center')||'null');if(Array.isArray(p)&&p.length===2&&p.every(Number.isFinite))center=p;}catch(_){}
const captureView=new URLSearchParams(location.search).get('capture')==='1';
if(captureView){for(const id of ['sensorFrame'])$(id).src+='&capture=1';}
const map=$('map'),tiles=new Map();let scheduled=false;
const gpsArrivalLabel=document.createElement('label');
gpsArrivalLabel.innerHTML='GPS 도달 범위 <input id="gpsArrivalRadius" type="number" min="0.01" step="0.1" aria-label="GPS 도달 범위 m" title="목표 지점 중심에서 도달로 판정하는 반경" style="width:46px"> m';
$('percent').closest('label').after(gpsArrivalLabel);
const driveFields={point:'percent',dock:'dockPercent',path:'pathPercent'};
const driveDefaults={point:Number($('percent').value)||20,dock:Number($('percent').value)||20,path:Number($('percent').value)||20};
const pointLimitLabel=$('percent').closest('label');pointLimitLabel.firstChild.textContent='주행 ';pointLimitLabel.classList.add('drive-limit');
$('percent').min='0';$('percent').max='100';$('percent').step='1';$('percent').setAttribute('aria-label','주행 출력 %');
for(const [key,label] of [['dock','도킹'],['path','경로추종']]){
 const element=document.createElement('label');element.className='drive-limit';
 element.innerHTML=`${label} <input id="${driveFields[key]}" type="number" min="0" max="100" step="1" value="${driveDefaults[key]}" aria-label="${label} 출력 %"> %`;
 gpsArrivalLabel.before(element);
}
let driveLimitsDirty=false,driveLimitsReady=false,driveLimitsTimer=null,driveLimitsBusy=false,driveLimitsQueued=false;
try{
 const saved=JSON.parse(localStorage.getItem('isv.drive-limits.v2')||localStorage.getItem('isv.drive-limits.v1')||'null');
 if(saved&&Object.keys(driveFields).every(key=>Number.isFinite(saved[key])&&saved[key]>=0&&saved[key]<=100)){
  for(const [key,id] of Object.entries(driveFields))$(id).value=String(saved[key]);driveLimitsDirty=true;
 }
}catch(_){}
function readDriveLimits(){
 const values={};for(const [key,id] of Object.entries(driveFields)){
  const text=$(id).value,value=Number(text);if(!text||!Number.isFinite(value)||value<0||value>100)throw Error('출력은 0~100%로 입력하세요');values[key]=value;
 }return values;
}
async function saveDriveLimits(){
 clearTimeout(driveLimitsTimer);driveLimitsTimer=null;
 let limits;try{limits=readDriveLimits();}catch(error){$('notice').textContent=error.message;return false;}
 driveLimitsDirty=true;try{localStorage.setItem('isv.drive-limits.v2',JSON.stringify(limits));}catch(_){}
 if(!driveLimitsReady){$('notice').textContent='출력 설정 PC에 저장됨 · 젯슨 적용 대기';return false;}
 if(driveLimitsBusy){driveLimitsQueued=true;return false;}
 driveLimitsBusy=true;
 try{
  const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'drive_limits',limits}),signal:AbortSignal.timeout(4000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'출력 설정 저장 실패');
  if(JSON.stringify(readDriveLimits())===JSON.stringify(limits)){driveLimitsDirty=false;$('notice').textContent='';}
  return true;
 }catch(error){driveLimitsReady=false;$('notice').textContent=error.message;return false;}
 finally{driveLimitsBusy=false;if(driveLimitsQueued){driveLimitsQueued=false;saveDriveLimits();}}
}
function renderDriveLimits(settings){
 if(!settings?.limits||settings.version!==2){driveLimitsReady=false;return;}
 driveLimitsReady=true;
 if(driveLimitsDirty){if(!driveLimitsBusy&&!driveLimitsTimer)driveLimitsTimer=setTimeout(saveDriveLimits,250);return;}
 for(const [key,id] of Object.entries(driveFields))if(document.activeElement!==$(id)&&Number.isFinite(settings.limits[key]))$(id).value=String(settings.limits[key]);
}
for(const id of Object.values(driveFields)){
 $(id).addEventListener('input',()=>{driveLimitsDirty=true;clearTimeout(driveLimitsTimer);driveLimitsTimer=setTimeout(saveDriveLimits,250);});
 $(id).addEventListener('change',saveDriveLimits);
}

let gpsArrivalTimer=null,gpsArrivalVersion=0,gpsArrivalBusy=false,gpsArrivalQueued=false;
function renderGpsArrival(settings){
 const input=$('gpsArrivalRadius'),radius=settings?.arrival_radius_m??data?.arrival_radius_m;
 if(document.activeElement!==input&&!gpsArrivalTimer&&!gpsArrivalBusy&&Number.isFinite(radius))input.value=String(radius);
}
async function saveGpsArrival(){
 clearTimeout(gpsArrivalTimer);gpsArrivalTimer=null;
 const input=$('gpsArrivalRadius'),radius=Number(input.value),version=gpsArrivalVersion;
 if(!input.value||!Number.isFinite(radius)||radius<=0)return;
 if(gpsArrivalBusy){gpsArrivalQueued=true;return;}
 gpsArrivalBusy=true;
 try{
  const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'gps_arrival',arrival_radius_m:radius}),signal:AbortSignal.timeout(4000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'GPS 도달 범위 저장 실패');
  if(version===gpsArrivalVersion)$('notice').textContent=`GPS 도달 범위 ${value.arrival_radius_m} m 저장됨`;
 }catch(error){if(version===gpsArrivalVersion)$('notice').textContent=error.message;}
 finally{gpsArrivalBusy=false;if(gpsArrivalQueued){gpsArrivalQueued=false;saveGpsArrival();}}
}
$('gpsArrivalRadius').addEventListener('input',()=>{gpsArrivalVersion++;clearTimeout(gpsArrivalTimer);gpsArrivalTimer=setTimeout(saveGpsArrival,250);});
$('gpsArrivalRadius').addEventListener('change',saveGpsArrival);

const pathFollowButton=document.createElement('button');pathFollowButton.id='startPathFollow';pathFollowButton.className='primary';pathFollowButton.textContent='주행 시작';pathFollowButton.disabled=true;$('trial').before(pathFollowButton);
pathFollowButton.onclick=async()=>{if(await saveDriveLimits())request('start_point_follow',{tracking:'path',percent:Number($('percent').value)||20,drive_limits:readDriveLimits(),position_source:virtualGps()?'virtual':'gps',buoy_course:true});};
const stopPathFollowButton=document.createElement('button');stopPathFollowButton.id='stopPathFollow';stopPathFollowButton.className='stop';stopPathFollowButton.textContent='주행 종료';stopPathFollowButton.disabled=true;pathFollowButton.after(stopPathFollowButton);
stopPathFollowButton.onclick=()=>request('stop');
const kindLabels={path:'경로추종',point:'위치추종',hold:'위치유지',dock:'도킹',dock_left:'도킹 left',dock_middle:'도킹 middle',dock_right:'도킹 right',orbit:'부표 선회'},kindColors={path:'#71b8ff',point:'#42e0b8',hold:'#f2c85e',dock:'#ed8ba0',dock_left:'#a4dba9',dock_middle:'#7cddd7',dock_right:'#f0b875',orbit:'#ff9b45'};
for(const [value,label] of Object.entries(kindLabels))if(![...$('waypointType').options].some(option=>option.value===value))$('waypointType').add(new Option(label,value));
$('waypointType').onchange=()=>{$('holdDurationLabel').hidden=$('waypointType').value!=='hold';};
function ensureDraft(){if(draft===null){draft=structuredClone(data?.route?.points??[]);draftTypes=[...routeTypes(data?.route)];draftHold=[...routeHold(data?.route)];draftOutputs=routeOutputs(data?.route);draftMission=data?.route?.mission||null;draftOrbits=routeOrbits(data?.route);}}
function routeOutputs(route){return route?.points?.map((_,i)=>route.waypoint_outputs?.[i]??null)||[];}
function routeTypes(route){return route?.points?.map((_,i)=>route.waypoint_types?.[i]||'path')||[];}
function routeHold(route){return route?.points?.map((_,i)=>route.waypoint_types?.[i]==='hold'?5:0)||[];}
function normalizeOrbitConfig(value,point){
 const original=value?.options||[{id:'candidate1',point,entry_side:'left'}];
 const selectedIndex=original.findIndex(option=>option.id===value?.selected);
 const options=Array.from({length:3},(_,i)=>{
  const option=original[i],side=option?.entry_side||(i===0?'left':'right');
  return {id:`candidate${i+1}`,point:option?.point?[...option.point]:null,entry_side:side,direction:side==='left'?'cw':'ccw'};
 });
 return {selected:selectedIndex>=0&&selectedIndex<3?options[selectedIndex].id:null,radius_m:value?.radius_m??3,options};
}
function routeOrbits(route){return route?.points?.map((point,i)=>route.waypoint_types?.[i]==='orbit'||route.waypoint_orbits?.[i]?normalizeOrbitConfig(route.waypoint_orbits?.[i],point):null)||[];}
function draftRoute(){
 const points=draft??data?.route?.points??[],types=draftTypes??routeTypes(data?.route);
 const orbits=structuredClone(draftOrbits??routeOrbits(data?.route));
 points.forEach((p,i)=>{
  if(types[i]==='orbit'&&!orbits[i])orbits[i]=normalizeOrbitConfig(null,p);
  const choice=orbits[i]?.options.find(option=>option.id===orbits[i].selected);
  if(choice&&p)choice.point=[...p];
 });
 return {points,waypoint_types:types,hold_seconds:draftHold??routeHold(data?.route),waypoint_outputs:draftOutputs??routeOutputs(data?.route),mission:draft!==null?draftMission:data?.route?.mission,waypoint_orbits:orbits};
}
let selectedOrbitCandidate=null;
const orbitSelection=document.createElement('div');orbitSelection.className='toolbar orbit-selection';orbitSelection.hidden=true;
orbitSelection.innerHTML='<strong id="orbitWaypointLabel">선회 후보</strong><select id="orbitTarget" aria-label="주행 시작 전 선회 후보 선택"></select><label>반경 <input id="orbitRadius" type="number" min="1.5" max="8" step="0.5" style="width:54px" value="3"> m</label><button id="editOrbitCandidates" type="button">후보 3개 설정</button><span id="orbitFeedback" role="status"></span>';
pathFollowButton.closest('.toolbar').before(orbitSelection);
const orbitEditor=document.createElement('details');orbitEditor.className='orbit-editor';orbitEditor.hidden=true;
orbitEditor.innerHTML='<summary>선회 후보 3개 · 좌표와 고정 선회방향</summary><div class="orbit-candidate-list"></div>';
for(let i=0;i<3;i++){
 const row=document.createElement('div');row.className='orbit-candidate-row';row.dataset.candidate=String(i);
 row.innerHTML=`<button data-action="select" type="button" aria-label="선회 후보 ${i+1} 선택">후보 ${i+1}</button><select data-mode aria-label="선회 후보 ${i+1} 진입·선회 방향"><option value="left">왼쪽 진입 · 시계방향</option><option value="right">오른쪽 진입 · 반시계방향</option></select><input data-lat type="number" step="any" min="-85" max="85" placeholder="위도" aria-label="선회 후보 ${i+1} 위도"><input data-lon type="number" step="any" min="-180" max="180" placeholder="경도" aria-label="선회 후보 ${i+1} 경도"><button data-action="apply" type="button">좌표 적용</button>`;
 orbitEditor.firstElementChild.nextElementSibling.append(row);
 let pastedPoint=null;
 row.querySelector('[data-mode]').onchange=()=>editOrbitCandidate(i,{entry_side:row.querySelector('[data-mode]').value});
 row.querySelector('[data-action="select"]').onclick=()=>{
  const waypoint=orbitWaypointIndex();selectOrbitCandidate({waypoint,candidate:i});
  const point=draftRoute().waypoint_orbits[waypoint]?.options[i]?.point;
  if(point){center=[...point];follow=false;soon();}
 };
 row.querySelector('[data-action="apply"]').onclick=async()=>{
  if(await applyOrbitCandidateCoordinates(i,row,pastedPoint))pastedPoint=null;
 };
 for(const input of row.querySelectorAll('input'))input.addEventListener('input',()=>{pastedPoint=null;});
 row.addEventListener('paste',event=>{
  if(!event.target.matches('[data-lat],[data-lon]'))return;
  let point;try{point=parseOrbitCandidateCoordinates(event.clipboardData.getData('text/plain'));}catch(_){return;}
  event.preventDefault();pastedPoint={waypoint:orbitWaypointIndex(),point};
  row.querySelector('[data-lat]').value=String(point[0]);row.querySelector('[data-lon]').value=String(point[1]);
  $('orbitFeedback').textContent=`후보 ${i+1} 복사 좌표 준비됨 · 좌표 적용을 누르세요.`;
 });
}
async function applyOrbitCandidateCoordinates(candidate,row,pastedPoint){
 const waypoint=orbitWaypointIndex(),lat=row.querySelector('[data-lat]'),lon=row.querySelector('[data-lon]');
 if(data?.execute||waypoint<0)return false;
 const previous=draftRoute().waypoint_orbits[waypoint]?.options[candidate]?.point;
 try{
  let point=pastedPoint?.waypoint===waypoint?pastedPoint.point:null;
  const manual=lat.value!==''&&lon.value!==''&&(!previous||[Number(lat.value),Number(lon.value)].some((v,i)=>Math.abs(v-previous[i])>6e-9));
  if(!point&&manual){if(!lat.reportValidity()||!lon.reportValidity())return false;point=[Number(lat.value),Number(lon.value)];}
  if(!point)point=parseOrbitCandidateCoordinates(await navigator.clipboard.readText());
  if(waypoint!==orbitWaypointIndex()||data?.execute)return false;
  editOrbitCandidate(candidate,{point},waypoint);return true;
 }catch(error){$('orbitFeedback').textContent=error.name==='NotAllowedError'?'좌표 칸에 Ctrl+V로 붙여넣은 뒤 좌표 적용을 누르세요.':error.message;return false;}
}
function parseOrbitCandidateCoordinates(text){const points=parseWaypointCoordinates(text);if(points.length!==1)throw Error('선회 후보에는 위도·경도 한 쌍만 적용하세요.');return points[0];}
function orbitWaypointIndex(){const route=draftRoute();if(selectedOrbitCandidate&&route.waypoint_types[selectedOrbitCandidate.waypoint]==='orbit')return selectedOrbitCandidate.waypoint;return route.waypoint_types[selectedWaypoint]==='orbit'?selectedWaypoint:route.waypoint_types.indexOf('orbit');}
function orbitCandidateLabel(option,index){return `후보 ${index+1} · ${option.entry_side==='left'?'왼쪽 진입 · 시계방향':'오른쪽 진입 · 반시계방향'}`;}
function selectOrbitCandidate(target){
 selectedOrbitCandidate=target;selectedWaypoint=-1;selectedCoordinate=null;placingWaypoint=-1;
 document.activeElement?.blur();renderPoints();soon();
}
function editOrbitCandidate(candidate,changes,waypoint=orbitWaypointIndex(),notify=true){
 if(data?.execute||waypoint<0||draftRoute().waypoint_types[waypoint]!=='orbit')return;
 if(changes.point&&(!changes.point.every(Number.isFinite)||Math.abs(changes.point[0])>=85||Math.abs(changes.point[1])>180)){$('orbitFeedback').textContent='후보 위도·경도 범위를 확인하세요.';return;}
 ensureDraft();draftOrbits[waypoint]=structuredClone(draftRoute().waypoint_orbits[waypoint]);
 const option=draftOrbits[waypoint].options[candidate];Object.assign(option,changes);option.direction=option.entry_side==='left'?'cw':'ccw';
 if(draftOrbits[waypoint].selected===option.id&&option.point)draft[waypoint]=[...option.point];
 selectedOrbitCandidate={waypoint,candidate};
 if(notify)document.activeElement?.blur();renderPoints();soon();if(notify)$('orbitFeedback').textContent=`후보 ${candidate+1} 편집됨 · 경로 저장을 누르세요.`;
}
$('editOrbitCandidates').onclick=()=>{orbitEditor.open=!orbitEditor.open;};
orbitEditor.addEventListener('toggle',()=>{if(!orbitEditor.open){if(drag?.orbit)cancelMapDrag();selectedOrbitCandidate=null;}renderPoints();soon();});
function renderOrbitSelection(){
 const route=draftRoute(),index=orbitWaypointIndex();orbitSelection.hidden=index<0;orbitEditor.hidden=index<0;
 if(index<0)return;const cfg=route.waypoint_orbits[index];if(!cfg)return;
 $('orbitWaypointLabel').textContent=`${index+1}번 선회`;
 const select=$('orbitTarget'),key=JSON.stringify(cfg.options);
 if(select.dataset.options!==key){select.replaceChildren(new Option('시작 전 후보 1개 선택',''));cfg.options.forEach((option,i)=>{const item=new Option(orbitCandidateLabel(option,i)+(option.point?'':' · 좌표 미지정'),option.id);item.disabled=!option.point;select.add(item);});select.dataset.options=key;}
 select.value=cfg.selected||'';select.disabled=!!data?.execute;$('orbitRadius').disabled=!!data?.execute;
 if(document.activeElement!==$('orbitRadius'))$('orbitRadius').value=String(cfg.radius_m);
 for(const [i,row] of [...orbitEditor.querySelectorAll('.orbit-candidate-row')].entries()){
  const option=cfg.options[i];row.classList.toggle('selected',cfg.selected===option.id);
  row.classList.toggle('editing',orbitEditor.open&&selectedOrbitCandidate?.waypoint===index&&selectedOrbitCandidate.candidate===i);
  for(const input of row.querySelectorAll('input,select,button'))input.disabled=!!data?.execute;
  if(!row.contains(document.activeElement)){row.querySelector('[data-mode]').value=option.entry_side;row.querySelector('[data-lat]').value=option.point?.[0]??'';row.querySelector('[data-lon]').value=option.point?.[1]??'';}
 }
}
$('orbitTarget').onchange=async()=>{
 const route=draftRoute(),index=orbitWaypointIndex();if(index<0||data?.execute)return;
 ensureDraft();draftOrbits[index]=structuredClone(route.waypoint_orbits[index]);draftOrbits[index].selected=$('orbitTarget').value||null;
 const choice=draftOrbits[index].options.find(p=>p.id===draftOrbits[index].selected);if(choice)draft[index]=[...choice.point];
 selectedOrbitCandidate=null;map.style.cursor='';renderPoints();soon();$('orbitFeedback').textContent='선회 후보 선택 저장 중';
 const result=await request('route',{...draftRoute(),archive:true,archive_name:activeWaypointRouteName||undefined});
 $('orbitFeedback').textContent=result?(result.saved?'선회 후보 선택 저장됨':'선회 후보 적용 대기'):'선회 후보 저장 실패 · 경로 저장을 다시 누르세요.';
};
$('orbitRadius').oninput=()=>{
 if(!$('orbitRadius').checkValidity()||data?.execute)return;const route=draftRoute(),index=orbitWaypointIndex();if(index<0)return;
 ensureDraft();draftOrbits[index]=structuredClone(route.waypoint_orbits[index]);draftOrbits[index].radius_m=Number($('orbitRadius').value);renderRouteControls();soon();
};

function selectedHold(){return $('waypointType').value==='hold'?5:0;}
const waypointEditor=document.createElement('form');waypointEditor.id='waypointEditor';waypointEditor.className='toolbar waypoint-editor';waypointEditor.hidden=true;
let waypointPastePending=null;
let waypointRestartSubmitting=false;
let waypointRestartRequestId=null;
waypointEditor.innerHTML='<strong id="selectedWaypointLabel"></strong><label>위도 <input id="waypointLat" type="number" step="any" min="-85" max="85" required aria-label="선택 지점 위도"></label><label>경도 <input id="waypointLon" type="number" step="any" min="-180" max="180" required aria-label="선택 지점 경도"></label><label>종류 <select id="selectedWaypointType" aria-label="선택 지점 종류"></select></label><button type="submit">좌표 적용</button><button id="startFromWaypoint" type="button" class="primary">이 지점부터 주행 시작</button><button id="closeWaypointEditor" type="button" aria-label="지점 선택 해제">닫기</button>';
$('points').before(waypointEditor);
for(const [value,label] of Object.entries(kindLabels))$('selectedWaypointType').add(new Option(label,value));
function renderWaypointEditor(){
 const route=draftRoute(),p=route.points[selectedWaypoint];waypointEditor.hidden=selectedWaypoint<0||selectedWaypoint>=route.points.length;
 if(waypointEditor.hidden){selectedWaypoint=-1;return;}
 $('selectedWaypointLabel').textContent=`${selectedWaypoint+1}번 지점`;
 if(!waypointEditor.contains(document.activeElement)){
  $('waypointLat').value=p?p[0].toFixed(8):'';$('waypointLon').value=p?p[1].toFixed(8):'';
  $('selectedWaypointType').value=route.waypoint_types[selectedWaypoint]||'path';
 }
}
function selectWaypoint(index){
 waypointPastePending=null;
 selectedOrbitCandidate=null;
 selectedWaypoint=index;selectedCoordinate=null;
 if(adding)insertionIndex=index>=0?index+1:null;
 placingWaypoint=index>=0&&!draftRoute().points[index]?index:-1;
 map.style.cursor=placingWaypoint>=0?'default':'';
 if(waypointEditor.contains(document.activeElement))document.activeElement.blur();
 renderPoints();soon();
}
function changeWaypointKind(index,kind){
 ensureDraft();draftTypes[index]=kind;draftHold[index]=kind==='hold'?5:0;
 lastPointsKey='';renderPoints();soon();
}
$('selectedWaypointType').onchange=()=>changeWaypointKind(selectedWaypoint,$('selectedWaypointType').value);
$('closeWaypointEditor').onclick=()=>selectWaypoint(-1);
$('startFromWaypoint').onclick=async()=>{
 const index=selectedWaypoint;
 if(waypointRestartSubmitting||data?.waypoint_restart||index<0||!waypointEditor.reportValidity())return;
 if([...$('points').querySelectorAll('input')].some(input=>!input.reportValidity()))return;
 for(const id of Object.values(driveFields))if(!$(id).reportValidity())return;
 const point=[Number($('waypointLat').value),Number($('waypointLon').value)];
 ensureDraft();draft[index]=point;
 const route=structuredClone(draftRoute());
 if(route.points.some(p=>!p)){$('notice').textContent='미지정 좌표를 입력하세요.';return;}
 waypointRestartSubmitting=true;renderRouteControls();
 try{
  const response=await request('restart_point_follow',{route,start_index:index,tracking:'path',percent:Number($('percent').value)||20,
   drive_limits:readDriveLimits(),position_source:virtualGps()?'virtual':'gps',buoy_course:true});
  waypointRestartRequestId=response?.request_id||null;
 }finally{waypointRestartSubmitting=false;renderRouteControls();}
};
waypointEditor.onsubmit=async e=>{
 e.preventDefault();if(selectedWaypoint<0||!waypointEditor.reportValidity())return;
 const index=selectedWaypoint,p=[Number($('waypointLat').value),Number($('waypointLon').value)],previous=draftRoute().points[index];
 const manuallyChanged=!previous||p.some((value,i)=>Math.abs(value-previous[i])>6e-9);
 try{
  let points=waypointPastePending?.index===index?waypointPastePending.points:null;
  if(!points&&manuallyChanged)points=[p];
  if(!points){
   try{points=parseWaypointCoordinates(await navigator.clipboard.readText());}
   catch(_){$('notice').textContent='위도 또는 경도 칸에 좌표를 Ctrl+V로 붙여넣은 뒤 좌표 적용을 누르세요.';$('waypointLat').focus();return;}
  }
  if(index!==selectedWaypoint)return;
  applyWaypointCoordinates(points,index);waypointPastePending=null;
 }catch(error){$('notice').textContent=error.message;}
};
for(const id of ['waypointLat','waypointLon'])$(id).addEventListener('input',()=>{waypointPastePending=null;});
waypointEditor.addEventListener('paste',event=>{
 if(!['waypointLat','waypointLon'].includes(event.target?.id))return;
 let points;try{points=parseWaypointCoordinates(event.clipboardData.getData('text/plain'));}catch(_){return;}
 event.preventDefault();waypointPastePending={index:selectedWaypoint,points};
 $('waypointLat').value=String(points[0][0]);$('waypointLon').value=String(points[0][1]);
 $('notice').textContent='복사 좌표 준비됨 · 좌표 적용을 누르세요.';
});

const dockingBadge=document.createElement('span');dockingBadge.id='dockingStatus';dockingBadge.className='muted';dockingBadge.setAttribute('role','status');stopPathFollowButton.after(dockingBadge);

document.querySelector('.map-legend').textContent='청록: GPS 기록 경로 · 노랑 점선: 저장 GPS 참고 경로 · 빨강: 라이다 · 파랑: 목표 경로 · 주황: MPC 목표 궤적 · 보라: 현재 명령 예측';
const layerSpec=[['boat','현재 위치·방향'],['trail','실제 이동 경로'],['reference','저장 GPS 참고 경로'],['waypoints','목적지'],['hits','라이다 히트'],['memory','누적 히트'],['astar','A* 원본'],['smooth','목표 경로'],['feasible','MPC 목표 궤적'],['prediction','현재 명령 예측']];
let layers={};try{layers=JSON.parse(localStorage.getItem('outdoor.layers.core.v2')||'{}')||{};}catch(_){}
const enabled=key=>layerSpec.some(([name])=>name===key)&&layers[key]!==false;
if(layers.memory===undefined)layers.memory=false;
for(const [key,label] of layerSpec){const el=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.checked=enabled(key);input.dataset.layer=key;input.onchange=()=>{layers[key]=input.checked;try{localStorage.setItem('outdoor.layers.core.v2',JSON.stringify(layers));}catch(_){}soon();};el.append(input,document.createTextNode(label));$('layers').append(el);}
const locationTools=document.createElement('div');locationTools.className='toolbar';locationTools.innerHTML='<button id="recordLocation" aria-pressed="false">현재위치 기록 꺼짐</button><button id="showGpsMarks" aria-pressed="true">기록점 켜짐</button><button id="gpsMarkSummary" aria-expanded="false">기록 좌표 · 0점</button><button id="showGpsMarks2" aria-pressed="true">기록점 2 켜짐</button><button id="gpsMarkSummary2" aria-expanded="false">기록좌표 2 · 12점</button><span id="gpsMarkStatus" role="status"></span>';
document.querySelector('.map-stage').before(locationTools);
const gpsMarkDetails=document.createElement('details');gpsMarkDetails.className='gps-mark-details';
gpsMarkDetails.innerHTML='<summary hidden>기록 좌표</summary><div id="gpsMarkList" class="gps-mark-list"></div>';
locationTools.after(gpsMarkDetails);
let gpsMarks=[],gpsMarkSelected=null,gpsMarkRecording=false,gpsMarksVisible=true,gpsMarkQueue=Promise.resolve();
const gpsMarks2=[].map(([lat,lon],index)=>({id:`recorded-2-${index+1}`,number:index+1,lat,lon}));
let gpsMarks2Visible=true,gpsMarkListGroup=1;
let gpsMarkDeleting=false;
const gpsKeyboardDocs=new WeakSet();
function gpsMarkText(point){return `(${point.lat.toFixed(7)}, ${point.lon.toFixed(7)}),`;}
async function copyGpsMarkText(text){
 try{await navigator.clipboard.writeText(text);$('gpsMarkStatus').textContent='좌표 복사됨';}
 catch(_){const input=document.createElement('textarea');input.value=text;document.body.append(input);input.select();const ok=document.execCommand('copy');input.remove();$('gpsMarkStatus').textContent=ok?'좌표 복사됨':'좌표 입력칸에서 직접 복사하세요.';}
}
function selectGpsMark(id,group=1){
 const point=(group===2?gpsMarks2:gpsMarks).find(p=>p.id===id);if(!point)return;
 gpsMarkListGroup=group;gpsMarkSelected=id;
 if(group===2)gpsMarks2Visible=true;else gpsMarksVisible=true;
 center=[point.lat,point.lon];follow=false;zoom=Math.max(zoom,19);
 gpsMarkDetails.open=true;renderGpsMarks();soon();
 const row=$('gpsMarkList').querySelector(`[data-mark-id="${id}"]`);row?.scrollIntoView({block:'nearest'});
}
function renderGpsMarks(){
 $('recordLocation').textContent='현재위치 기록 '+(gpsMarkRecording?'켜짐':'꺼짐');$('recordLocation').classList.toggle('active',gpsMarkRecording);$('recordLocation').setAttribute('aria-pressed',String(gpsMarkRecording));
 $('recordLocation').title='켜진 상태에서 스페이스바를 누르면 실제 GPS 위치를 한 점 기록합니다.';
 $('showGpsMarks').textContent='기록점 '+(gpsMarksVisible?'켜짐':'꺼짐');$('showGpsMarks').setAttribute('aria-pressed',String(gpsMarksVisible));
 $('gpsMarkSummary').textContent=`기록 좌표 · ${gpsMarks.length.toLocaleString()}점`;
 $('showGpsMarks2').textContent='기록점 2 '+(gpsMarks2Visible?'켜짐':'꺼짐');$('showGpsMarks2').setAttribute('aria-pressed',String(gpsMarks2Visible));
 for(const [id,group] of [['gpsMarkSummary',1],['gpsMarkSummary2',2]]){
  const open=gpsMarkDetails.open&&gpsMarkListGroup===group;
  $(id).setAttribute('aria-expanded',String(open));$(id).classList.toggle('active',open);
 }
 if(!gpsMarkDetails.open)return;
 const rows=document.createDocumentFragment();
 for(const point of gpsMarkListGroup===2?gpsMarks2:gpsMarks){
  const row=document.createElement('div');row.className='gps-mark-row'+(point.id===gpsMarkSelected?' selected':'');row.dataset.markId=point.id;
  const label=(gpsMarkListGroup===2?'2-':'P')+point.number;
  const number=document.createElement('button');number.textContent=label;number.title='지도에서 보기';number.onclick=()=>selectGpsMark(point.id,gpsMarkListGroup);
  const coordinates=document.createElement('input');coordinates.readOnly=true;coordinates.value=gpsMarkText(point);coordinates.setAttribute('aria-label',`${label} 위도 경도`);coordinates.onclick=()=>coordinates.select();
  const copy=document.createElement('button');copy.textContent='복사';copy.onclick=()=>copyGpsMarkText(gpsMarkText(point));
  row.append(number,coordinates,copy);
  if(gpsMarkListGroup===1){
   const remove=document.createElement('button');remove.textContent='삭제';remove.className='delete-gps-mark';remove.disabled=gpsMarkDeleting;remove.onclick=()=>deleteGpsMarkPoints([point.id]);row.append(remove);
  }
  rows.append(row);
 }
 $('gpsMarkList').replaceChildren(rows);
}
async function refreshGpsMarks(){
 try{const response=await fetch('/api/gps-paths?marks=1',{cache:'no-store',signal:AbortSignal.timeout(6000)}),value=await response.json();if(!response.ok)throw Error(value.error||'기록 좌표 수신 실패');gpsMarks=value.points||[];renderGpsMarks();soon();}
 catch(error){$('gpsMarkStatus').textContent=error.message;setTimeout(refreshGpsMarks,3000);}
}
async function recordGpsMark(){
 try{
  const response=await fetch('/api/gps-paths',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'mark',request_id:crypto.randomUUID()}),signal:AbortSignal.timeout(6000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'현재 위치 기록 실패');gpsMarks=value.points;const point=gpsMarks.at(-1);gpsMarkSelected=point.id;
  $('gpsMarkStatus').textContent=`P${point.number} 기록됨`;renderGpsMarks();soon();
 }catch(error){$('gpsMarkStatus').textContent=error.message;}
}
function gpsMarkKeydown(event){
 if(!gpsMarkRecording||(event.code!=='Space'&&event.key!==' ')||event.ctrlKey||event.altKey||event.metaKey)return;
 if(event.target?.closest?.('input,textarea,select,[contenteditable]:not([contenteditable="false"])'))return;
 event.preventDefault();event.stopPropagation();if(event.repeat)return;
 gpsMarkQueue=gpsMarkQueue.then(recordGpsMark);
}
function installGpsMarkKeyboard(doc){if(!doc||gpsKeyboardDocs.has(doc))return;gpsKeyboardDocs.add(doc);doc.addEventListener('keydown',gpsMarkKeydown,true);doc.addEventListener('keydown',undoWaypointMove,true);}
installGpsMarkKeyboard(document);
for(const id of ['sensorFrame','slamMapFrame']){const frame=$(id);frame.addEventListener('load',()=>installGpsMarkKeyboard(frame.contentDocument));installGpsMarkKeyboard(frame.contentDocument);}
$('recordLocation').onclick=()=>{gpsMarkRecording=!gpsMarkRecording;$('gpsMarkStatus').textContent=gpsMarkRecording?'스페이스바로 현재 위치 기록':'';renderGpsMarks();};
$('showGpsMarks').onclick=()=>{gpsMarksVisible=!gpsMarksVisible;renderGpsMarks();soon();};
$('showGpsMarks2').onclick=()=>{gpsMarks2Visible=!gpsMarks2Visible;renderGpsMarks();soon();};
async function deleteGpsMarkPoints(ids){
 if(gpsMarkDeleting||!ids.length)return;gpsMarkDeleting=true;renderGpsMarks();
 try{
  for(let start=0;start<ids.length;start+=16){
   const response=await fetch('/api/gps-paths',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'delete_marks',ids:ids.slice(start,start+16)}),signal:AbortSignal.timeout(6000)}),value=await response.json();
   if(!response.ok)throw Error(value.error||'기록점 삭제 실패');gpsMarks=value.points;
  }
  if(!gpsMarks.some(point=>point.id===gpsMarkSelected))gpsMarkSelected=null;
  $('gpsMarkStatus').textContent=`${ids.length}점 삭제됨`;
 }catch(error){$('gpsMarkStatus').textContent=error.message;}
 finally{gpsMarkDeleting=false;renderGpsMarks();soon();}
}
gpsMarkDetails.addEventListener('toggle',renderGpsMarks);
for(const [id,group] of [['gpsMarkSummary',1],['gpsMarkSummary2',2]]){
 $(id).onclick=()=>{
  const close=gpsMarkDetails.open&&gpsMarkListGroup===group;gpsMarkListGroup=group;gpsMarkDetails.open=!close;
  if(group===2&&!close){gpsMarks2Visible=true;fitGpsReference(gpsMarks2.map(point=>[point.lat,point.lon]));}
  renderGpsMarks();soon();
 };
}
const mapOverlayStatus=document.createElement('span');mapOverlayStatus.id='mapOverlayStatus';mapOverlayStatus.className='map-overlay-status';document.querySelector('#layers').append(mapOverlayStatus);
const mapAdvanced=document.createElement('section');mapAdvanced.className='map-advanced';mapAdvanced.setAttribute('aria-label','웨이포인트 설정');
for(const part of [$('addPoint').closest('.toolbar'),orbitEditor,waypointEditor,$('points')])mapAdvanced.append(part);
document.querySelector('.gps-map').append(mapAdvanced);
const waypointRouteTools=document.createElement('div');waypointRouteTools.className='toolbar waypoint-route-library';
waypointRouteTools.innerHTML='<select id="savedWaypointRoutes" aria-label="저장된 웨이포인트 경로"><option value="">저장 경로 선택</option></select><button id="loadWaypointRoute" disabled>경로 불러오기</button>';
mapAdvanced.firstElementChild.after(waypointRouteTools);
function parseWaypointCoordinates(text){
 const trimmed=String(text).trim();
 if(!trimmed||!/^[-+\d.eE\s()[\],;]+$/.test(trimmed))throw Error('복사한 위도·경도 좌표를 붙여넣으세요.');
 const numbers=trimmed.replace(/[()[\],;]/g,' ').trim().split(/\s+/).map(Number);
 if(numbers.length%2||!numbers.every(Number.isFinite))throw Error('좌표를 위도, 경도 한 쌍씩 입력하세요.');
 const points=[];for(let i=0;i<numbers.length;i+=2)points.push([numbers[i],numbers[i+1]]);
 if(points.some(point=>Math.abs(point[0])>=85||Math.abs(point[1])>180))throw Error('위도·경도 순서와 범위를 확인하세요.');
 return points;
}
function applyWaypointCoordinates(points,index){
 if(!points.length)return;
 if(points.some(point=>point.length!==2||!point.every(Number.isFinite)||Math.abs(point[0])>=85||Math.abs(point[1])>180))throw Error('위도·경도를 확인하세요.');
 if(index<0||index>=draftRoute().points.length)throw Error('좌표를 적용할 지점을 먼저 선택하세요.');
 if(draftRoute().points.length-1+points.length>100)throw Error('경로에는 최대 100점을 추가할 수 있습니다.');
 ensureDraft();const kind=$('selectedWaypointType').value;
 draft.splice(index,1,...points.map(point=>[...point]));draftTypes.splice(index,1,...points.map(()=>kind));draftHold.splice(index,1,...points.map(()=>kind==='hold'?5:0));
 const output=draftOutputs[index];draftOutputs.splice(index,1,...points.map((_,i)=>i===0?output:null));draftOrbits.splice(index,1,...points.map((_,i)=>i===0?draftOrbits[index]:null));placingWaypoint=-1;map.style.cursor='';
 $('waypointLat').value=String(points[0][0]);$('waypointLon').value=String(points[0][1]);
 selectedWaypoint=index;setMapClickMode('browse');follow=false;center=[...points[0]];renderPoints();soon();
 $('notice').textContent='좌표 적용됨 · 경로 저장을 누르세요.';
}
let waypointRouteBusy=false,activeWaypointRouteName='';
async function refreshWaypointRoutes(selectedName){
 try{
  const response=await fetch('/api/gps-paths?waypoints=1',{cache:'no-store',signal:AbortSignal.timeout(6000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'저장 경로 목록 수신 실패');
  activeWaypointRouteName=value.selected?.name||activeWaypointRouteName;
  const select=$('savedWaypointRoutes'),selected=selectedName||select.value||activeWaypointRouteName;
  select.replaceChildren(new Option('저장 경로 선택',''));
  for(const file of value.routes||[])select.add(new Option(`${file.label} · ${file.point_count}점 · ${file.saved_kst.replace('T',' ').slice(0,19)}`,file.name));
  if([...select.options].some(option=>option.value===selected))select.value=selected;
  renderRouteControls();
 }catch(error){$('notice').textContent=error.message;setTimeout(refreshWaypointRoutes,3000);}
}
$('savedWaypointRoutes').onchange=()=>renderRouteControls();
$('loadWaypointRoute').onclick=async()=>{
 const name=$('savedWaypointRoutes').value;if(!name||waypointRouteBusy)return;
 waypointRouteBusy=true;renderRouteControls();
 try{
  const value=await request('load_route',{name}),route=value?.loaded_route;if(!route)return;
  activeWaypointRouteName=value.loaded_profile.name;
  draft=structuredClone(route.points);draftTypes=[...route.waypoint_types];draftHold=[...route.hold_seconds];draftOutputs=routeOutputs(route);draftMission=route.mission||null;draftOrbits=routeOrbits(route);selectedWaypoint=-1;placingWaypoint=-1;
  setMapClickMode('browse');fitGpsReference(route.points.filter(Boolean));renderPoints();soon();
  $('notice').textContent=route.points.some(p=>!p)?'좌표 미지정 · 지점 번호를 선택한 뒤 지도 클릭 또는 좌표 입력':`저장 경로 불러옴 · ${route.points.length}점`;
 }finally{waypointRouteBusy=false;renderRouteControls();}
};
function virtualGps(){const v=sensorValues.virtual_gps;return v?.active&&v.age_s<2?v:null;}
function mapBoat(){
 const h=sensorValues.heading,v=virtualGps();
 if(Number.isFinite(h?.heading_deg))heldMapHeading=h.heading_deg;
 const heading=heldMapHeading;
 if(v?.point)return {lat:v.point[0],lon:v.point[1],heading_deg:heading,position_sigma_m:.5,virtual:true};
 const gps=sensorValues.gps;
 if(data?.state?.position_source==='gps'&&Number.isFinite(data.state.lat)&&Number.isFinite(data.state.lon))heldMapBoat={...data.state};
 else if(gps?.valid_fix&&Number.isFinite(gps.latitude_deg)&&Number.isFinite(gps.longitude_deg))heldMapBoat={lat:gps.latitude_deg,lon:gps.longitude_deg,position_sigma_m:gps.horizontal_sigma_m??0};
 if(!heldMapBoat)return null;
 return {...heldMapBoat,heading_deg:heading??heldMapBoat.heading_deg??null};
}
function dashboardMapSnapshot(){return {sensors:sensorValues,outdoor:data};}
function publishDashboardMapSnapshot(){
 const snapshot=dashboardMapSnapshot();
 for(const listener of mapSubscribers){try{listener(snapshot);}catch(error){console.error('SLAM map subscriber failed',error);}}
}
window.ISVDashboardMapBridge=Object.freeze({
 version:1,
 slamHost:$('slamMapHost'),
 getSnapshot:dashboardMapSnapshot,
 subscribe(listener){
  if(typeof listener!=='function')throw new TypeError('listener must be a function');
  mapSubscribers.add(listener);listener(dashboardMapSnapshot());
  return ()=>mapSubscribers.delete(listener);
 },
 setSlamStatus(message){$('slamMapStatus').textContent=String(message);},
 setSlamReady(ready=true){$('slamMapPlaceholder').hidden=Boolean(ready);}
});
function installSlamMapFrame(){
 try{
  const frame=$('slamMapFrame'),doc=frame.contentDocument;
  if(!doc?.getElementById('tankMap'))throw Error('수조 지도 요소를 찾지 못했습니다.');
  if(!doc.getElementById('isvSlamEmbedStyle')){
   const style=doc.createElement('style');style.id='isvSlamEmbedStyle';
   style.textContent='html,body{height:100%;overflow:hidden}body>header,body>#notice,body>main>*:not(.tank-panel){display:none!important}body>main{height:100%;max-width:none;margin:0;padding:0}.tank-panel{height:100%;min-height:0;margin:0!important;display:flex;flex-direction:column;border:0;border-radius:0}.tank-panel>.panel-head{min-height:38px;padding:4px 8px;flex:none}.tank-panel>.map-management{display:flex!important;flex:none;flex-wrap:wrap;padding:4px 6px;gap:4px;max-height:130px;overflow:auto}.tank-panel>.map-management input{max-width:140px}.tank-panel>.map-management select{max-width:160px}.tank-panel>.tank-toolbar{min-height:30px;padding:2px 8px;flex:none}.tank-panel>.tank-stage{height:0;min-height:130px;flex:1}.tank-panel>.panel-foot{padding:4px 8px;flex:none}.tank-panel .candidate-info{max-height:40px;overflow:auto}.tank-panel .candidate-controls{display:none!important}.tank-panel #candidateInfo,.tank-panel #candidateSelection,.tank-panel #globalMatchInfo{display:none!important}.tank-panel>.tank-mission{flex:none;padding:4px 8px;gap:3px 12px;font-size:11px}.tank-panel>.tank-mission>strong{font-size:13px;flex-basis:160px}.tank-panel>.tank-mission b{font-size:12px}.tank-panel>.panel-foot{gap:4px 12px;font-size:11px}.tank-panel>.panel-head h2{font-size:14px}.tank-panel>.panel-head p{font-size:10px;margin:2px 0 0}.tank-panel>.panel-head .tools{gap:4px}.tank-panel button,.tank-panel select,.tank-panel input{font-size:11px;padding:4px 6px}.tank-panel>.map-management{max-height:none;overflow:visible}.tank-panel>.map-management>*{flex:0 1 auto}.tank-panel>.map-management strong{font-size:11px}.tank-panel>.map-management #mapJobStatus{flex-basis:auto;margin-left:0}.tank-panel>.map-management input{width:125px}.tank-panel>.map-management select{width:auto;min-width:120px;max-width:160px}.tank-panel>.tank-stage{min-height:0}.tank-panel>.tank-toolbar{min-height:25px}.tank-panel #dockingRouteStatus:empty,.tank-panel #mapJobStatus:empty{display:none}';
   doc.head.append(style);
  }
  const pauseCloud=doc.getElementById('pauseCloud');if(pauseCloud)pauseCloud.checked=true;
  $('slamMapStatus').textContent='수조 SLAM 지도 연결됨';$('slamMapPlaceholder').hidden=true;
 }catch(error){$('slamMapStatus').textContent=error.message;$('slamMapPlaceholder').hidden=false;}
}
$('slamMapFrame').src=location.pathname==='/'?'/sensors?slam_embed=1':'/?slam_embed=1';
$('slamMapFrame').addEventListener('load',installSlamMapFrame);
function publishYawTarget(){
 const boat=mapBoat(),points=data?.route?.points||[],index=Math.max(0,Math.min(points.length-1,data?.goal||0)),goal=points[index];
 let target=null;
 if(boat&&goal&&performance.now()-lastReceived<3500){
  const north=(goal[0]-boat.lat)*Math.PI/180*6371000,east=(goal[1]-boat.lon)*Math.PI/180*6371000*Math.cos(boat.lat*Math.PI/180);
  if(Math.hypot(east,north)>.01)target={bearing_deg:(Math.atan2(east,north)*180/Math.PI+360)%360,distance_m:Math.hypot(east,north),index};
 }
 $('sensorFrame').contentWindow.postMessage({kind:'yaw-target',target},location.origin);
}
function renderLocation(){
 publishYawTarget();
 if(!center){const p=data?.route?.points?.[0]||sensorValues.map_reference?.point;if(p)center=[...p];}
 const s=mapBoat();if(s&&(follow||!center))center=[s.lat,s.lon];
  $('position').textContent=s?`${s.virtual?'가상 GPS · ':''}${s.lat.toFixed(6)}, ${s.lon.toFixed(6)} · ${Number.isFinite(s.heading_deg)?s.heading_deg.toFixed(1):'—'}° · ±${Number(s.position_sigma_m||0).toFixed(1)} m`:center?'이전 지도 위치 · 배 GPS 대기':'배 GPS 위치 수신 대기';
 soon();
}
function setMapClickMode(mode){mapClickMode=mode;adding=mode==='route';selectedCoordinate=null;selectedOrbitCandidate=null;renderLocation();renderPoints();soon();}
const project=(ll,z=zoom)=>{const n=256*2**z,s=Math.sin(ll[0]*Math.PI/180);return [(ll[1]+180)/360*n,(.5-Math.log((1+s)/(1-s))/(4*Math.PI))*n];};
const unproject=(p,z=zoom)=>{const n=256*2**z;return [Math.atan(Math.sinh(Math.PI*(1-2*p[1]/n)))*180/Math.PI,p[0]/n*360-180];};
const mapMetersPerPixel=(latitude,z)=>2*Math.PI*6371000*Math.cos(latitude*Math.PI/180)/(256*2**z);
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
function drawMap(){const [g,w,h]=canvas('map');if(!center){g.fillStyle='#bac9d5';g.font='14px system-ui';g.fillText('배 GPS 위치 또는 저장된 지도 위치를 기다리는 중입니다.',24,h/2);return;}if(performance.now()-lastCenterSaved>5000){lastCenterSaved=performance.now();try{localStorage.setItem('outdoor.map.center',JSON.stringify(center));}catch(_){}}const base=project(center),pixel=ll=>{const p=project(ll);return [p[0]-base[0]+w/2,p[1]-base[1]+h/2];};
 if($('tiles').checked){
  const native=Math.min(zoom,basemap==='satellite'?21:19),factor=2**(zoom-native),size=256*factor;
  const sx=Math.floor((base[0]-w/2)/size),sy=Math.floor((base[1]-h/2)/size);let visible=0;
  g.globalAlpha=basemap==='satellite'?.88:.72;
  for(let x=sx;x<=(base[0]+w/2)/size;x++)for(let y=sy;y<=(base[1]+h/2)/size;y++)if(drawTile(g,x,y,native,x*size-base[0]+w/2,y*size-base[1]+h/2,size))visible++;
  g.globalAlpha=1;map.dataset.tiles=visible;map.dataset.zoom=zoom;map.dataset.basemap=basemap;
  if(!visible){g.fillStyle='#bbb';g.font='12px system-ui';g.fillText('배경 지도 수신 대기 · 위치와 경로는 계속 표시',16,20);}
 }
 const metersPerPixel=mapMetersPerPixel(center[0],zoom),spacing=10/metersPerPixel;g.strokeStyle='#88888830';g.lineWidth=.5;g.beginPath();for(let x=w/2%spacing;x<w;x+=spacing){g.moveTo(x,0);g.lineTo(x,h);}for(let y=h/2%spacing;y<h;y+=spacing){g.moveTo(0,y);g.lineTo(w,y);}g.stroke();
 function line(points,color,dash=[],segmented=false,width=2){g.strokeStyle=color;g.lineWidth=width;g.setLineDash(dash);g.beginPath();let prev=null;for(const p of points||[]){if(!p){prev=null;continue;}const [x,y]=pixel(p);if(!prev||(segmented&&p[2]!==prev[2]))g.moveTo(x,y);else g.lineTo(x,y);prev=p;}g.stroke();g.setLineDash([]);}
 const fresh=performance.now()-lastReceived<2500,origin=data?.map_origin||data?.origin;
 if(fresh&&origin){
  const toLatlon=p=>[origin[0]+p[1]/6371000*180/Math.PI,origin[1]+p[0]/(6371000*Math.cos(origin[0]*Math.PI/180))*180/Math.PI];
  function hits(points,color,size,memory=false){g.fillStyle=color;for(const p of points||[]){const [x,y]=pixel(toLatlon(p));if(x<0||y<0||x>w||y>h)continue;g.globalAlpha=memory?Math.max(.05,.35*(1-p[2]/(data.map_layers.ttl_s||20))):1;g.fillRect(x-size/2,y-size/2,size,size);}g.globalAlpha=1;}
  if(enabled('memory'))hits(data?.map_layers?.accumulated,'#ffab85',2,true);
  if(enabled('hits'))hits(data?.map_layers?.hits,'#ff6767',3);
 }
 if(fresh){
  if(data?.execute&&data?.vision?.settings?.corridor&&data?.goal===2){
   line(data?.buoy_route?.left,'#e0bd67',[],false,3);
   line(data?.buoy_route?.right,'#e0bd67',[],false,3);
   line(data?.buoy_route?.center,'#7adce6',[5,4],false,2);
  }
  if(enabled('astar'))line(data?.display_guidance_raw?.length?data.display_guidance_raw:data?.guidance_raw,'#d6d6d6',[4,5]);
  if(enabled('smooth'))line(data?.point_follow_path?.length?data.point_follow_path:data?.display_guidance?.length?data.display_guidance:data?.guidance,'#64a8ff',[],false,2.5);
  if(enabled('feasible'))line(data?.prediction?.length?data.prediction:data?.feasible_path,'#ffad56',[],false,3.5);
  if(enabled('prediction'))line(data?.preview,'#e995f1',[6,4],false,2.5);
 }
 const followControl=data?.execute&&data.navigation_mode==='point_follow'&&data.point_follow?.control;
 $('mapOverlayStatus').textContent=fresh?(followControl?`${data.point_follow.maneuver} · ${followControl}`:data?.feasible_status||data?.preview_status||data?.display_planning?.status||'경로 계산 중'):'지도 데이터 대기';
 $('mapOverlayStatus').title='주황: 현재 자세·전진·횡방향 속도·회전속도·최근 추진 명령에서 모델로 계산한 MPC 후보 궤적 · 보라: 현재 명령 유지 시 3초 예측 · 점선: 기하학적 기준 경로';
 if(enabled('reference')){
  const points=gpsPaths.reference?.points||[];line(points,'#f0c86e',[7,4],true,3);
  if(points.length){const p=pixel(points[0]);g.fillStyle='#f0c86e';g.beginPath();g.arc(...p,4,0,Math.PI*2);g.fill();g.font='12px system-ui';g.fillText(points.length===1?'참고 GPS':'참고 경로 시작',p[0]+7,p[1]-7);}
 }
 if(enabled('trail'))line(visibleTrail(gpsPaths.active?.points??data?.trail??[]),C[0],[],true);
 const route=draftRoute(),points=enabled('waypoints')?route.points:[];line(points,'#b0b0b0',[3,7]);

 for(const group of [{points:gpsMarks,visible:gpsMarksVisible,color:'#ff90d5',prefix:'P'},{points:gpsMarks2,visible:gpsMarks2Visible,color:'#7de6ff',prefix:'2-'}]){
 if(!group.visible)continue;
 for(const point of group.points){
  const [x,y]=pixel([point.lat,point.lon]);if(x< -30||y< -30||x>w+30||y>h+30)continue;
  const selected=point.id===gpsMarkSelected;g.fillStyle=selected?'#fff2a6':group.color;g.strokeStyle='#15171c';g.lineWidth=2;
  g.beginPath();g.rect(x-5,y-5,10,10);g.fill();g.stroke();
  if(selected){g.strokeStyle='#fff2a6';g.beginPath();g.arc(x,y,13,0,Math.PI*2);g.stroke();}
  g.font='bold 12px system-ui';g.strokeStyle='#15171c';g.lineWidth=3;g.strokeText(group.prefix+point.number,x+8,y+15);g.fillText(group.prefix+point.number,x+8,y+15);
 }
 }
 points.forEach((p,i)=>{if(!p)return;const [x,y]=pixel(p),kind=route.waypoint_types[i]||'path';g.fillStyle=kindColors[kind];g.strokeStyle=i===data?.goal&&data?.execute?'#ffffff':'#111';g.lineWidth=i===data?.goal&&data?.execute?3:2;g.beginPath();g.arc(x,y,6,0,Math.PI*2);g.fill();g.stroke();if(i===selectedWaypoint){g.strokeStyle='#fff';g.lineWidth=2;g.beginPath();g.arc(x,y,11,0,Math.PI*2);g.stroke();}g.font='bold 12px system-ui';g.strokeStyle='#111';g.lineWidth=3;const label=`${i+1} ${kindLabels[kind]}`;g.strokeText(label,x+8,y-7);g.fillText(label,x+8,y-7);});
 if(enabled('waypoints'))for(const [i,cfg] of (route.waypoint_orbits||[]).entries())if(cfg&&route.waypoint_types[i]==='orbit'){
  for(const [candidate,option] of cfg.options.entries()){
   if(!option.point)continue;const [x,y]=pixel(option.point),selected=cfg.selected===option.id;
   const editing=orbitEditor.open&&selectedOrbitCandidate?.waypoint===i&&selectedOrbitCandidate.candidate===candidate;
   g.save();g.strokeStyle=selected?'#ff9b45':'#ba8c62';g.lineWidth=selected?2:1;g.setLineDash(selected?[]:[4,4]);
   g.beginPath();g.arc(x,y,cfg.radius_m/metersPerPixel,0,Math.PI*2);g.stroke();g.setLineDash([]);
   const color=g.strokeStyle;g.fillStyle=color;g.strokeStyle=editing?'#fff':'#111';g.lineWidth=editing?3:2;
   g.beginPath();g.arc(x,y,orbitEditor.open?8:4,0,Math.PI*2);g.fill();g.stroke();
   g.font='bold 11px system-ui';
   if(orbitEditor.open){g.fillStyle='#111';g.textAlign='center';g.textBaseline='middle';g.fillText(String(candidate+1),x,y);g.textAlign='start';g.textBaseline='alphabetic';}
   g.fillStyle=color;g.strokeStyle='#111';g.lineWidth=3;const label=`후보 ${candidate+1} · ${option.direction==='cw'?'시계':'반시계'}`;
   g.strokeText(label,x+11,y+17);g.fillText(label,x+11,y+17);g.restore();
  }
 }
 if(selectedCoordinate){g.save();g.globalAlpha=.6;g.fillStyle='#ffffff';g.strokeStyle='#111';g.lineWidth=1.5;g.beginPath();g.arc(...pixel(selectedCoordinate),8,0,Math.PI*2);g.fill();g.stroke();g.restore();}
 const s=mapBoat();if(s&&enabled('boat')){g.save();g.translate(...pixel([s.lat,s.lon]));g.rotate((s.heading_deg||0)*Math.PI/180);g.globalAlpha=s.virtual||sensorValues.gps?.age_s<3||performance.now()-lastReceived<3000?1:.35;g.fillStyle=s.virtual?'#ffe36b':'#48ffd0';g.strokeStyle='#101010';g.lineWidth=2;g.beginPath();if(s.heading_deg===null){g.arc(0,0,6,0,Math.PI*2);}else{g.moveTo(0,-14);g.lineTo(8,10);g.lineTo(0,6);g.lineTo(-8,10);g.closePath();}g.fill();g.stroke();g.restore();}
 const scale=Math.max(1,Math.round(90*metersPerPixel/5)*5);g.strokeStyle='#eeeeee';g.lineWidth=3;g.beginPath();g.moveTo(20,h-22);g.lineTo(20+scale/metersPerPixel,h-22);g.stroke();g.fillStyle='#eeeeee';g.font='12px system-ui';g.fillText(`${scale} m`,20,h-30);
}
let drag=null;
function undoWaypointMove(event){
 if(!(event.ctrlKey||event.metaKey)||event.key.toLowerCase()!=='z'||event.shiftKey||event.altKey||drag)return;
 if(event.target?.closest?.('input,textarea,select,[contenteditable]:not([contenteditable="false"])'))return;
 const move=waypointMoveHistory.at(-1);if(!move)return;
 if(JSON.stringify(draftRoute().points)!==move.pointsKey){waypointMoveHistory.length=0;return;}
 if(move.candidate!==undefined&&JSON.stringify(draftRoute().waypoint_orbits)!==move.orbitsKey){waypointMoveHistory.length=0;return;}
 event.preventDefault();event.stopPropagation();waypointMoveHistory.pop();ensureDraft();
 if(move.candidate!==undefined){editOrbitCandidate(move.candidate,{point:[...move.before]},move.index);follow=false;$('orbitFeedback').textContent='후보 이동 취소됨 · 경로 저장을 누르세요.';return;}
 draft[move.index]=[...move.before];selectedWaypoint=move.index;follow=false;
 if(waypointEditor.contains(document.activeElement)||$('points').contains(document.activeElement))document.activeElement.blur();
 renderPoints();soon();$('notice').textContent='지점 이동 취소됨 · 경로 저장을 누르세요.';
}
function mapCoordinate(e){if(!center)return null;const r=map.getBoundingClientRect(),b=project(center);return unproject([b[0]+e.clientX-r.left-r.width/2,b[1]+e.clientY-r.top-r.height/2]);}
function waypointAt(e){
 if(!center||!enabled('waypoints'))return -1;
 const r=map.getBoundingClientRect(),base=project(center),x=e.clientX-r.left,y=e.clientY-r.top;
 let index=-1,best=14;
 draftRoute().points.forEach((p,i)=>{if(!p)return;const q=project(p),d=Math.hypot(q[0]-base[0]+r.width/2-x,q[1]-base[1]+r.height/2-y);if(d<best){best=d;index=i;}});
 return index;
}
function orbitCandidateAt(e){
 if(!center||!enabled('waypoints')||!orbitEditor.open||orbitEditor.hidden||data?.execute)return null;
 const r=map.getBoundingClientRect(),base=project(center),x=e.clientX-r.left,y=e.clientY-r.top,route=draftRoute();
 let target=null,best=14;
 for(const [waypoint,cfg] of (route.waypoint_orbits||[]).entries())if(cfg&&route.waypoint_types[waypoint]==='orbit'){
  for(const [candidate,option] of cfg.options.entries()){
   if(!option.point)continue;const q=project(option.point),d=Math.hypot(q[0]-base[0]+r.width/2-x,q[1]-base[1]+r.height/2-y);
   const preferred=selectedOrbitCandidate?.waypoint===waypoint&&selectedOrbitCandidate.candidate===candidate;
   if(d<14&&(d<best||preferred&&Math.abs(d-best)<1)){best=d;target={waypoint,candidate};}
  }
 }
 return target;
}
map.addEventListener('pointerdown',e=>{
 if(!center||e.button!==0)return;e.preventDefault();map.setPointerCapture(e.pointerId);
 const orbit=placingWaypoint>=0?null:orbitCandidateAt(e),index=placingWaypoint>=0||orbit?-1:waypointAt(e);
 const point=orbit?[...draftRoute().waypoint_orbits[orbit.waypoint].options[orbit.candidate].point]:index>=0?[...draftRoute().points[index]]:null;
 drag={id:e.pointerId,x:e.clientX,y:e.clientY,base:project(center),moved:false,index,point,orbit};
 if(orbit){follow=false;selectOrbitCandidate(orbit);map.style.cursor='grabbing';}
 else if(index>=0){follow=false;selectWaypoint(index);}
});
map.addEventListener('pointermove',e=>{
 if(!drag){map.style.cursor=orbitCandidateAt(e)?'grab':'';return;}if(drag.id!==e.pointerId)return;
 const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(Math.hypot(dx,dy)>4)drag.moved=true;
 if(drag.moved){
  follow=false;
  if(drag.orbit){const p=project(drag.point);editOrbitCandidate(drag.orbit.candidate,{point:unproject([p[0]+dx,p[1]+dy])},drag.orbit.waypoint,false);}
  else if(drag.index>=0){ensureDraft();const p=project(drag.point);draft[drag.index]=unproject([p[0]+dx,p[1]+dy]);renderPoints();}
  else center=unproject([drag.base[0]-dx,drag.base[1]-dy]);
  soon();
 }
});
map.addEventListener('pointerup',e=>{
 if(!drag||drag.id!==e.pointerId)return;
 const finished=drag;drag=null;
 if(finished.orbit){
  map.style.cursor=orbitEditor.open?'grab':'';
  const target=finished.orbit,point=draftRoute().waypoint_orbits[target.waypoint]?.options[target.candidate]?.point;
  if(finished.moved&&point&&point.some((v,i)=>v!==finished.point[i])){
   waypointMoveHistory.push({index:target.waypoint,candidate:target.candidate,before:finished.point,pointsKey:JSON.stringify(draftRoute().points),orbitsKey:JSON.stringify(draftRoute().waypoint_orbits)});
   if(waypointMoveHistory.length>100)waypointMoveHistory.shift();
   $('orbitFeedback').textContent=`후보 ${target.candidate+1} 이동됨 · Ctrl+Z로 취소 · 경로 저장을 누르세요.`;
  }renderPoints();soon();return;
 }
 if(!finished.moved&&placingWaypoint>=0){const index=placingWaypoint;ensureDraft();draft[index]=mapCoordinate(e);placingWaypoint=-1;map.style.cursor='';document.activeElement?.blur();renderPoints();soon();$('notice').textContent=`${index+1}번 좌표 지정됨 · 경로 저장을 누르세요.`;return;}
 if(finished.index>=0){if(finished.moved){
  if(draft[finished.index].some((value,i)=>value!==finished.point[i])){
   waypointMoveHistory.push({index:finished.index,before:finished.point,pointsKey:JSON.stringify(draftRoute().points)});
   if(waypointMoveHistory.length>100)waypointMoveHistory.shift();
  }
  $('notice').textContent='지점 이동됨 · Ctrl+Z로 취소 · 경로 저장을 누르세요.';
 }renderPoints();soon();return;}
 if(!finished.moved&&!adding){selectWaypoint(-1);return;}
 if(!finished.moved&&adding){
  const p=mapCoordinate(e);if(!p)return;ensureDraft();
  const missing=insertionIndex===null?draft.indexOf(null):-1;
  if(missing>=0){draft[missing]=p;selectedWaypoint=missing;}
  else if(draft.length<100){
   const index=insertionIndex===null?draft.length:Math.min(insertionIndex,draft.length);
   draft.splice(index,0,p);draftTypes.splice(index,0,$('waypointType').value);draftHold.splice(index,0,selectedHold());draftOutputs.splice(index,0,null);draftOrbits.splice(index,0,null);
   selectedWaypoint=index;insertionIndex=index+1;
  }else{$('notice').textContent='경로에는 최대 100점을 추가할 수 있습니다.';return;}
  waypointMoveHistory.length=0;document.activeElement?.blur();
  renderPoints();soon();renderLocation();$('notice').textContent=`${selectedWaypoint+1}번 지점 추가됨 · 경로 저장을 누르세요.`;
 }
});
function cancelMapDrag(){if(drag?.orbit&&drag.moved)editOrbitCandidate(drag.orbit.candidate,{point:drag.point},drag.orbit.waypoint,false);else if(drag?.index>=0&&drag.moved){draft[drag.index]=drag.point;renderPoints();soon();}drag=null;map.style.cursor='';}
map.addEventListener('pointercancel',cancelMapDrag);
map.addEventListener('lostpointercapture',cancelMapDrag);
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
function sameRoute(a,b){return Array.isArray(a)&&Array.isArray(b)&&a.length===b.length&&a.every((p,i)=>p===null||b[i]===null?p===b[i]:p.length===2&&b[i].length===2&&p.every((v,j)=>Math.abs(v-b[i][j])<1e-9));}
function routeDirty(){return draft!==null&&(!sameRoute(draft,data?.route?.points??[])||JSON.stringify(draftTypes)!==JSON.stringify(routeTypes(data?.route))||JSON.stringify(draftHold)!==JSON.stringify(routeHold(data?.route))||JSON.stringify(draftOutputs)!==JSON.stringify(routeOutputs(data?.route))||draftMission!==(data?.route?.mission||null)||JSON.stringify(draftRoute().waypoint_orbits)!==JSON.stringify(routeOrbits(data?.route)));}
function renderRouteControls(){
 renderOrbitSelection();
 const selected=selectedWaypoint>=0&&selectedWaypoint<draftRoute().points.length;
 $('deleteWaypoint').disabled=!selected;
 $('addPoint').textContent=adding?'지점 추가 종료':selected?`${selectedWaypoint+1}번 뒤 지점 추가`:'다중 지점 추가';
 $('addPoint').title=selected?'선택 지점 바로 뒤에 지도 클릭 순서대로 삽입합니다.':'지도 클릭으로 지점을 추가합니다.';
 $('loadWaypointRoute').disabled=waypointRouteBusy||!$('savedWaypointRoutes').value||!!(data?.execute&&lastReceived>0&&performance.now()-lastReceived<3500);
 const priority=data?.priority_route,stage=priority?.state;
 dockingBadge.textContent=stage==='neutralizing'?'지도 포착 · 도킹 전환 중':stage==='docking'?`도킹 ${priority.docking_goal||'A'} 추종 · GPS 목표 ${Number(priority.gps_skipped_index)+1} 건너뛰기 예정`:stage==='error'?'도킹 대기 · 종료 상태 확인 필요':priority?.gps_skipped_index!==null&&priority?.gps_skipped_index!==undefined?`도킹 완료 · ${priority.gps_resume_index===null?'GPS 경로 완료':`GPS 목표 ${Number(priority.gps_resume_index)+1}로 복귀`}`:data?.execute?'GPS 주행 · 저장 지도 부분 일치 대기':'';
 dockingBadge.title=stage==='docking'?priority.reason||'':data?.error||'';
 const saved=data?.route?.points??[],dirty=routeDirty(),fresh=lastReceived>0&&performance.now()-lastReceived<3500;
 const restarting=waypointRestartSubmitting||!!data?.waypoint_restart||pending?.action==='restart_point_follow';
 const orbitMissing=(from=0)=>draftRoute().waypoint_types.some((kind,i)=>i>=from&&kind==='orbit'&&!draftRoute().waypoint_orbits[i]?.options.find(option=>option.id===draftRoute().waypoint_orbits[i].selected)?.point);
 const restartReason=orbitMissing(Math.max(0,selectedWaypoint))?'선회할 부표를 선택하세요':!fresh?'젯슨 상태 수신 대기':!data?.waypoint_restart_schema?'브링업에 선택 지점 시작 코드를 적용하세요':!selected?'시작 지점을 선택하세요':restarting?'기존 주행 종료 · 선택 지점 시작 대기':draftRoute().points.some(p=>!p)?'미지정 좌표를 입력하세요':'';
 $('startFromWaypoint').disabled=!!restartReason;$('startFromWaypoint').title=restartReason||'기존 주행 종료 후 선택 지점부터 시작합니다.';
 const preparation=data?.docking_start;
 const orbitPending=orbitMissing();
 const followReason=!fresh?'젯슨 상태 수신 대기':data?.execute?'주행 중':orbitPending?'선회할 부표를 선택하세요':draftRoute().points.some(p=>!p)?'미지정 좌표를 입력하고 경로를 저장하세요':dirty?'편집한 경로를 저장하세요':!saved.length?'목적지를 추가하고 경로 저장을 누르세요':!preparation?.ready?(preparation?.reason||'SLAM·도킹 경로 확인 중'):'';
 $('startPathFollow').disabled=!!followReason||restarting;$('startPathFollow').title=followReason||'파란 목표 경로를 따라 좌우 출력 제어 · 출력 상한 공유';
 if(data?.waypoint_restart)dockingBadge.textContent=`기존 주행 종료 · ${data.waypoint_restart.start_index+1}번 지점 시작 대기`;
 if(fresh&&!data?.execute&&saved.length&&!dirty&&followReason)dockingBadge.textContent=followReason;

 const pointRunning=!!(data?.execute&&data?.navigation_mode==='point_follow');

 const pathRunning=pointRunning&&data.point_follow_tracking==='path';
 const mission=data?.waypoint_mission;
 const orbit=data?.buoy_orbit;
 if(pointRunning&&orbit)dockingBadge.textContent=`${orbit.selected?.replace('candidate','후보 ')} · ${orbit.phase==='entry'?'선회 진입':orbit.source!=='lidar'?'부표 재탐색':orbit.direction==='cw'?'시계 선회':'반시계 선회'} · ${Math.min(360,Math.floor(orbit.progress_deg))}° / 360°`;
 else if(pointRunning&&mission?.holding)dockingBadge.textContent=`${data.goal+1}번 위치유지 중 · ${Math.ceil(mission.remaining_s??5)}초 후 다음 지점`;
 else if(pointRunning&&mission?.searching_for_map)dockingBadge.textContent=`${data.goal+1}번 도킹 지도 탐색 · ${Math.ceil(mission.search_remaining_s??120)}초 남음 · 미포착 시 다음 지점`;
 else if(stage==='gps'&&data?.execute&&priority?.gps_skipped_index==null)dockingBadge.textContent=`${data.goal+1}번 ${kindLabels[mission?.kind]||'경로추종'} 중`;

  $('stopPathFollow').disabled=!fresh||(!data?.execute&&!restarting)||pending?.action==='stop';
  $('stopPathFollow').title=restarting?'선택 지점 재시작을 취소합니다.':data?.execute?'주행을 종료합니다.':'주행 중 활성화됩니다.';
 $('startPathFollow').textContent=pathRunning?'주행 중':'주행 시작';$('startPathFollow').classList.toggle('running',pathRunning);
 $('savePoints').textContent=dirty?'경로 저장':saved.length?`경로 저장됨 · ${saved.length}점`:'경로 저장';
}
function renderPoints(){
 const root=$('points'),route=draftRoute(),key=JSON.stringify([route,selectedWaypoint]);
 if(key!==lastPointsKey&&!root.contains(document.activeElement)){
  root.replaceChildren();lastPointsKey=key;
  route.points.forEach((p,i)=>{
   const kind=route.waypoint_types[i]||'path',s=document.createElement('span');s.className='point waypoint-'+kind+(i===selectedWaypoint?' selected':'');s.style.borderColor=kindColors[kind];s.style.color=kindColors[kind];
   const text=document.createElement('button');text.type='button';text.className='waypoint-select';text.textContent=p?`${i+1} · ${p[0].toFixed(6)}, ${p[1].toFixed(6)}`:`${i+1} · 좌표 미지정`;text.setAttribute('aria-label',`${i+1}번 지점 편집`);text.onclick=()=>selectWaypoint(i);
   const select=document.createElement('select');select.setAttribute('aria-label',`${i+1}번 지점 종류`);
   for(const [value,label] of Object.entries(kindLabels)){const option=document.createElement('option');option.value=value;option.textContent=label;select.append(option);}
   select.value=kind;select.onchange=()=>{selectedWaypoint=i;changeWaypointKind(i,select.value);};s.append(text,select);
   const output=document.createElement('input');output.type='number';output.min='0';output.max='100';output.step='1';output.placeholder='기본';output.style.width='56px';output.value=route.waypoint_outputs[i]??'';output.setAttribute('aria-label',`${i+1}번 접근 출력 %`);output.title='이 지점까지 접근 출력 · 빈칸은 종류별 기본 출력';
   output.oninput=()=>{if(!output.checkValidity())return;ensureDraft();draftOutputs[i]=output.value===''?null:Number(output.value);renderRouteControls();};output.onchange=()=>{lastPointsKey='';renderPoints();};
   const outputLabel=document.createElement('label');outputLabel.append(document.createTextNode('출력 '),output,document.createTextNode(' %'));s.append(outputLabel);
   if(kind==='hold')s.append(document.createTextNode(' · 5초 후 다음 지점'));
   root.append(s);
  });
 }
 $('addPoint').classList.toggle('active',adding);renderWaypointEditor();renderRouteControls();
}
async function request(action,extra={}){
 try{
  const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(4000)}),value=await response.json();
  if(!response.ok)throw Error(value.error||'요청 실패');
  if(value.saved_profile)activeWaypointRouteName=value.saved_profile.name;
  if(value.loaded_profile)activeWaypointRouteName=value.loaded_profile.name;
  if(action==='route'||action==='load_route')refreshWaypointRoutes(value.saved_profile?.name||value.loaded_profile?.name);
  if(value.draft_saved){$('notice').textContent='편집용 경로 저장됨 · 좌표 지정 후 경로 저장';return value;}
  if(value.saved){data={...data,route:value.route};draft=null;draftTypes=null;draftHold=null;draftOutputs=null;draftMission=null;draftOrbits=null;renderPoints();soon();$('notice').textContent='경로 저장됨 · 브링업 후 사용';return value;}
  if(value.trail_cleared){setTrailAfter(value.trail_after_t);if(data)data={...data,trail:visibleTrail(data.trail||[])};$('notice').textContent='누적 경로 초기화됨';soon();return value;}
  if(value.simulation){if(value.active&&selectedCoordinate&&extra.point&&sameRoute([selectedCoordinate],[extra.point]))selectedCoordinate=null;renderLocation();$('notice').textContent=value.active?'가상 GPS 발행 중 · 지점 추종에 사용 가능':'가상 GPS 발행 종료';return;}
  pending={id:value.request_id,at:performance.now(),action};$('notice').textContent='적용 대기';
  if(action==='stop')waypointRestartRequestId=null;
  return value;
 }catch(e){$('notice').textContent=e.message;}
}
$('clearTrail').onclick=async()=>{if(gpsPaths.active?.recording===true)return gpsPathAction('start');setTrailAfter(Number(data?.stamp??data?.stamp_s??0));if(data)data={...data,trail:[]};soon();$('clearTrail').disabled=true;try{await request('clear_trail');}finally{$('clearTrail').disabled=false;}};$('follow').onclick=()=>{follow=true;if(data?.state)center=[data.state.lat,data.state.lon];soon();};$('tiles').onchange=soon;
const deleteWaypointButton=document.createElement('button');deleteWaypointButton.id='deleteWaypoint';deleteWaypointButton.type='button';deleteWaypointButton.textContent='선택 지점 삭제';deleteWaypointButton.disabled=true;$('undoPoint').before(deleteWaypointButton);
deleteWaypointButton.onclick=()=>{
 const index=selectedWaypoint;if(index<0||index>=draftRoute().points.length)return;
 ensureDraft();draft.splice(index,1);draftTypes.splice(index,1);draftHold.splice(index,1);draftOutputs.splice(index,1);draftOrbits.splice(index,1);
 waypointMoveHistory.length=0;selectedWaypoint=-1;placingWaypoint=-1;insertionIndex=null;map.style.cursor='';document.activeElement?.blur();setMapClickMode('browse');
 $('notice').textContent=`${index+1}번 지점 삭제됨 · 경로 저장을 누르세요.`;
};
$('addPoint').onclick=()=>{placingWaypoint=-1;map.style.cursor='';insertionIndex=selectedWaypoint>=0?selectedWaypoint+1:null;setMapClickMode(adding?'browse':'route');};
$('undoPoint').onclick=()=>{ensureDraft();draft.pop();draftTypes.pop();draftHold.pop();draftOutputs.pop();draftOrbits.pop();waypointMoveHistory.length=0;selectedWaypoint=-1;placingWaypoint=-1;insertionIndex=null;document.activeElement?.blur();renderPoints();soon();};
$('clearPoints').onclick=()=>{draft=[];draftTypes=[];draftHold=[];draftOutputs=[];draftOrbits=[];waypointMoveHistory.length=0;selectedWaypoint=-1;placingWaypoint=-1;insertionIndex=null;document.activeElement?.blur();renderPoints();soon();};
$('savePoints').onclick=()=>{if([...$('points').querySelectorAll('input')].some(input=>!input.reportValidity()))return;request('route',{...draftRoute(),archive:true,archive_name:activeWaypointRouteName||undefined});};
function render(){if(draft!==null&&!routeDirty()){draft=null;draftTypes=null;draftHold=null;draftOutputs=null;draftMission=null;draftOrbits=null;}$('trial').textContent=data.execute?`실행 중 · 다음 ${data.goal+1} · ${data.arrivals}회 도착`:'정지';
 renderRouteControls();if(pending&&data.ack?.id===pending.id){$('notice').textContent=data.ack.message||(data.ack.ok?'적용됨':'요청 실패');pending=null;}else if(pending&&performance.now()-pending.at>7000){$('notice').textContent='적용 응답을 확인하지 못했습니다. 현재 상태를 확인하세요.';pending=null;}else if(!pending&&data.error)$('notice').textContent=data.error==='저장 설정 확인: 현재 센서·선체 학습 설정과 다른 모델입니다'?'':data.error;
 if(waypointRestartRequestId&&data.ack?.id===waypointRestartRequestId&&!data.waypoint_restart){$('notice').textContent=data.ack.message;waypointRestartRequestId=null;}
 renderLocation();renderPoints();soon();}
async function poll(){try{const response=await fetch('/api/state?view=outdoor'+(captureView?'&capture=1':''),{cache:'no-store',signal:AbortSignal.timeout(4500)});if(response.ok){const body=await response.json(),next=body.values?.outdoor;sensorValues=body.values||{};renderGpsArrival(body.values?.gps_arrival);renderDriveLimits(body.values?.drive_limits);renderGpsPaths(body.gps_paths);if(next&&next.age_s<3){setTrailAfter(next.trail_after_t);data=next;lastReceived=performance.now();render();}else{const saved=body.values?.saved_route?.route;if(saved){data={...(data||{}),route:saved};renderPoints();}renderLocation();}publishDashboardMapSnapshot();}}catch(_){}if(performance.now()-lastReceived>3500){$('startPathFollow').disabled=true;$('startFromWaypoint').disabled=true;soon();}setTimeout(poll,document.hidden?1800:200);}
window.addEventListener('message',e=>{
 if(e.origin!==location.origin)return;
 const frame=['sensorFrame'].map(id=>$(id)).find(f=>e.source===f.contentWindow);if(!frame)return;
 if(!window.kabotCaptureLocked&&!window.matchMedia('(min-width: 1000px)').matches&&e.data?.kind==='sensor-height'&&Number.isFinite(e.data.height))frame.style.height=Math.max(160,Math.min(5000,e.data.height+4))+'px';
});
new ResizeObserver(()=>{soon();}).observe(document.querySelector('main'));

window.kabotCaptureView=(v)=>{
 if(!v)return {width:innerWidth,height:innerHeight,scale:Math.min(devicePixelRatio||1,2),center,zoom,follow,layers,basemap,tiles:$('tiles').checked,sensors:$('sensorFrame').contentWindow.kabotCaptureView?.(),selectedCoordinate};
 if(v.selectedCoordinate===null)selectedCoordinate=null;
 else if(Array.isArray(v.selectedCoordinate)&&v.selectedCoordinate.length===2&&v.selectedCoordinate.every(Number.isFinite))selectedCoordinate=[...v.selectedCoordinate];
 if(Array.isArray(v.center)&&v.center.length===2&&v.center.every(Number.isFinite))center=v.center;
 if(Number.isFinite(v.zoom))zoom=Math.max(10,Math.min(23,v.zoom));
 follow=v.follow!==false;
 for(const [key] of layerSpec)if(typeof v.layers?.[key]==='boolean')layers[key]=v.layers[key];
 for(const input of document.querySelectorAll('[data-layer]'))input.checked=enabled(input.dataset.layer);
 if(['satellite','street'].includes(v.basemap)){basemap=v.basemap;$('basemap').value=basemap;attribution();}
 if(typeof v.tiles==='boolean')$('tiles').checked=v.tiles;
 $('sensorFrame').contentWindow.kabotCaptureView?.(v.sensors||{});soon();
};
window.kabotCaptureDraw=()=>{drawMap();};
function installCaptureUi(){
 if(captureView||captureUi)return;
 try{
  const doc=$('sensorFrame').contentDocument,row=doc?.querySelector('.runtime-status');if(!row)return;
  const style=doc.createElement('style');style.textContent='.runtime-item.capture-control{font-size:10px;padding:3px 5px;cursor:pointer}.runtime-item.capture-feedback{max-width:170px;overflow:hidden;text-overflow:ellipsis}';doc.head.append(style);
  const make=(id,label)=>{const button=doc.createElement('button');button.id=id;button.type='button';button.className='runtime-item capture-control';button.textContent=label;row.append(button);return button;};
  const screenshot=make('captureScreenshot','스크린샷'),start=make('captureStart','영상 시작'),stop=make('captureStop','영상 종료'),feedback=doc.createElement('span');
  feedback.id='captureFeedback';feedback.className='runtime-item capture-feedback';feedback.setAttribute('role','status');feedback.setAttribute('aria-live','polite');row.append(feedback);
  screenshot.onclick=()=>captureAction('screenshot');start.onclick=()=>captureAction('start');stop.onclick=()=>captureAction('stop');
  captureUi={screenshot,start,stop,feedback};
 }catch(_){captureUi=null;}
}
document.getElementById('sensorFrame').addEventListener('load',()=>{captureUi=null;installCaptureUi();});
async function captureAction(action){
 try{const response=await fetch('/api/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,view:window.kabotCaptureView()})});const body=await response.json();if(!response.ok)throw Error(body.error||'촬영 요청 실패');if(captureUi)captureUi.feedback.textContent=action==='stop'?'영상 저장 중':'촬영 준비 중';}
 catch(e){if(captureUi)captureUi.feedback.textContent=e.message;}
}
async function captureStatus(){
 installCaptureUi();
 try{const r=await fetch('/api/capture',{signal:AbortSignal.timeout(3000)});if(r.ok){const s=await r.json(),busy=s.phase!=='idle';if(captureUi){captureUi.start.disabled=busy;captureUi.stop.disabled=!(s.phase==='recording'||(s.phase==='preparing'&&s.operation==='start'));captureUi.screenshot.disabled=busy&&!s.recording;}
 const label={idle:'',preparing:'촬영 준비 중',recording:`녹화 ${s.elapsed_s||0}s · 실제 ${s.capture_fps||0}fps`,saving:'전체·패널 영상 저장 중',unavailable:'촬영 기능 준비 필요'}[s.phase]||'';
 if(captureUi){captureUi.feedback.textContent=s.error||label||(s.last_path?'저장 완료':'');captureUi.feedback.title=s.last_path||'';}}}
 catch(_){}setTimeout(captureStatus,1000);
}
if(!captureView)captureStatus();

let connectionClickSequence=0,connectionAction=null,connectionSnapshot=null,connectionPosting=false;
let rocketPowerPosting=false,lastRocketPower=null;
function renderRocketPower(value){
 lastRocketPower=value;const button=$('rocketPowerOn'),status=$('rocketPowerStatus');if(!button||!status)return;
 const busy=rocketPowerPosting||!!value?.busy,on=!!value?.powered;
 button.disabled=busy;button.setAttribute('aria-busy',String(busy));button.setAttribute('aria-pressed',String(on));
 status.dataset.state=busy?'starting':on?'on':value?.state||'unknown';
 status.textContent=busy?'로켓 전원 연결 중':on?'로켓 전원 켜짐':value?.state==='error'?'로켓 연결 실패':'로켓 전원 미확인';
 status.title=value?.error||(busy?'필요하면 Windows 관리자 승인창에서 예를 누르세요.':on?`로켓 관리 응답 확인 · ${value.management_ip}`:'USB 랜·PoE 전원·케이블 연결을 확인하세요.');
}
if($('rocketPowerOn'))$('rocketPowerOn').onclick=async()=>{
 if(rocketPowerPosting||lastRocketPower?.busy)return;
 rocketPowerPosting=true;renderRocketPower(lastRocketPower);
 try{
  const response=await fetch('/api/rocket-power',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'start'}),signal:AbortSignal.timeout(4000)});
  const value=await response.json();if(!response.ok)throw Error(value.error||'로켓 실행 요청 실패');
  lastRocketPower=value;
 }catch(error){lastRocketPower={state:'error',error:error.message,powered:false,busy:false};}
 finally{rocketPowerPosting=false;renderRocketPower(lastRocketPower);}
};
const connectionClientId=crypto.randomUUID();
function renderConnectionAction(job){
 connectionAction=job;
 for(const [route,id] of [['hotspot','connectHotspot'],['rocket','connectRocket']]){
  const button=$(id);if(!button)continue;
  button.disabled=!!(job?.busy&&job.route===route);
  button.setAttribute('aria-busy',String(!!(job?.busy&&job.route===route)));
  button.setAttribute('aria-pressed',String(!!(connectionSnapshot?.connected&&connectionSnapshot.active_host===`jetson-${route}`)));
 }
 const status=$('connectionActionStatus');if(!status)return;
 status.hidden=!job;status.dataset.state=job?.state||'';
 const label=job?.route==='rocket'?'로켓':'핫스팟';
 status.textContent=!job?'':job.state==='waiting'?`${label} 연결 대기 · 자동 재시도`:job.busy?`${label} IP 확인 · 연결 중…`:job.state==='error'?job.error:job.vscode?.startsWith('error:')?'SSH 연결됨 · 원격창 열기 실패':`${label} 연결 완료`;
 status.title=job?.error||(job?.vscode?.startsWith('error:')?job.vscode:job?.ip?`${job.ip} · VS Code 원격창 열기 요청됨`:'');
}
async function connectFromDashboard(route){
 const sequence=++connectionClickSequence;
 connectionPosting=true;
 renderConnectionAction({route,busy:true,state:'queued'});
 try{
  const response=await fetch('/api/connection',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({route,client_id:connectionClientId,sequence}),signal:AbortSignal.timeout(4000)});
  const job=await response.json();if(!response.ok)throw Error(job.error||'연결 요청 실패');
  if(sequence===connectionClickSequence)renderConnectionAction(job);
 }catch(error){if(sequence===connectionClickSequence)renderConnectionAction({route,state:'error',busy:false,error:error.message});}
 finally{if(sequence===connectionClickSequence)connectionPosting=false;}
}
if($('connectHotspot'))$('connectHotspot').onclick=()=>connectFromDashboard('hotspot');
if($('connectRocket'))$('connectRocket').onclick=()=>connectFromDashboard('rocket');
async function connectionStatus(){
 const sequence=connectionClickSequence;
 const canRenderAction=!connectionPosting;
 try{
  const response=await fetch('/api/connection',{cache:'no-store',signal:AbortSignal.timeout(2500)});
  if(!response.ok)throw Error('HTTP '+response.status);
  const value=await response.json(),host=String(value.host||'').toLowerCase();
  connectionSnapshot=value;
  renderRocketPower(value.rocket_power);
  if(canRenderAction&&!connectionPosting&&sequence===connectionClickSequence)renderConnectionAction(value.connection_request||connectionAction);
  const targetIp=String(value.target_ip||'');
  const route=value.connection_path;
  const path=route==='hotspot'?'핫스팟 SSH':route==='rocket'?'Rocket SSH':'SSH 경로 미확인';
  const connected=typeof value.connected==='boolean'?value.connected:value.status==='연결됨'||Boolean(value.sensors_running&&!value.error);
  const state=connected?'연결됨':'연결 대기';
  const target=value.target_ip?' · '+value.target_ip:'';
  $('connectionPath').textContent=`${path} · ${state}`;
  $('connectionPath').title=value.connection_evidence||'실제 SSH 접속 인터페이스 확인 중';
  for(const [kind,id] of [['hotspot','hotspotEndpoint'],['rocket','rocketEndpoint']]){const endpoint=value.endpoints?.[kind],ips=endpoint?.addresses||[];$(id).textContent=ips.length?ips.map(ip=>`ssh ${value.remote_user||'isv'}@${ip}`).join(' / '):'현재 주소 확인 안 됨';$(id).title=endpoint?`${endpoint.interface} · MAC ${endpoint.mac} · ${endpoint.link_up?'링크 연결':'링크 끊김'}`:'젯슨 네트워크 정보 수신 대기';}

  $('connectionPath').dataset.link=connected?'up':'down';
 }catch(_){$('connectionPath').textContent='젯슨 연결 정보 대기';$('connectionPath').dataset.link='down';}
 setTimeout(connectionStatus,1500);
}
connectionStatus();
refreshGpsPaths();
refreshWaypointRoutes();
refreshGpsMarks();renderGpsMarks();
poll();
})();
