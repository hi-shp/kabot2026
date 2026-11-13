/* Satellite map: bundled into app.js so the existing read-only server needs no change. */
const SatelliteMap = (() => {
  const R = 6378137, LIMIT = 85.05112878;
  const clamp = (n, lo, hi) => Math.max(lo, Math.min(hi, n));
  const wrap = n => ((n % 1) + 1) % 1;
  const valid = p => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite)
    && Math.abs(p[0]) <= LIMIT && Math.abs(p[1]) <= 180;
  function project([lat, lon]) {
    const s = Math.sin(clamp(lat, -LIMIT, LIMIT) * Math.PI / 180);
    return [(lon + 180) / 360, .5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)];
  }
  function unproject([x, y]) {
    return [Math.atan(Math.sinh(Math.PI * (1 - 2 * clamp(y, 0, 1)))) * 180 / Math.PI,
      wrap(x) * 360 - 180];
  }
  const delta = x => x - Math.round(x);
  function fit(points, w, h) {
    const origin = project(points[0]);
    const xy = points.map(p => {const q=project(p);return [origin[0]+delta(q[0]-origin[0]),q[1]];});
    const xs=xy.map(p=>p[0]), ys=xy.map(p=>p[1]);
    const lo=[Math.min(...xs),Math.min(...ys)], hi=[Math.max(...xs),Math.max(...ys)];
    const margin=10/(2*Math.PI*R*Math.max(.01,Math.cos(points[0][0]*Math.PI/180)));
    const zoom=clamp(Math.floor(Math.log2(Math.min(Math.max(80,w-100)/(hi[0]-lo[0]+2*margin),
      Math.max(70,h-100)/(hi[1]-lo[1]+2*margin))/256)),2,20);
    return {center:[wrap((lo[0]+hi[0])/2),(lo[1]+hi[1])/2],zoom};
  }
  class View {
    constructor(canvas) {
      this.el=canvas;this.mapOnly=canvas.dataset?.gpsMapOnly==='true';this.center=null;this.zoom=19;this.satellite=true;this.tiles=new Map();
      this.trail=[];this.routeKey=null;this.data={};this.queued=false;this.selection=null;
      this.width=1;this.height=1;this.moved=false;this.pickedGPS=null;
      const el=id=>document.getElementById(id);
      this.status=el('mapStatus');this.info=el('mapInfo');this.scaleLabel=el('mapscale');
      el('mapFit').onclick=()=>{this.fitAll();this.schedule();};
      el('mapPlus').onclick=()=>this.setZoom(this.zoom+1);
      el('mapMinus').onclick=()=>this.setZoom(this.zoom-1);
      el('mapLayer').onclick=()=>{this.satellite=!this.satellite;this.schedule();};
      el('mapBoat').onclick=()=>{if(this.position){this.center=project(this.position);this.schedule();}};
      canvas.addEventListener('wheel',e=>{e.preventDefault();const b=canvas.getBoundingClientRect();
        if(this.pointDrag)return;
        this.setZoom(this.zoom+(e.deltaY<0?1:-1),[e.clientX-b.left,e.clientY-b.top]);},{passive:false});
      canvas.addEventListener('pointerdown',e=>{if(e.button!==0||!this.center)return;
        if(window.MissionMapOverlay?.dragStart(this,e))return;
        this.drag={id:e.pointerId,x:e.clientX,y:e.clientY,center:[...this.center]};this.moved=false;
        canvas.setPointerCapture(e.pointerId);canvas.classList.add('dragging');});
      canvas.addEventListener('pointermove',e=>{if(window.MissionMapOverlay?.dragMove(this,e))return;
        if(!this.drag||this.drag.id!==e.pointerId)return;
        const dx=e.clientX-this.drag.x,dy=e.clientY-this.drag.y,s=256*2**this.zoom;
        if(Math.hypot(dx,dy)>3)this.moved=true;
        this.center=[wrap(this.drag.center[0]-dx/s),clamp(this.drag.center[1]-dy/s,0,1)];this.schedule();});
      canvas.addEventListener('pointerup',e=>{if(window.MissionMapOverlay?.dragEnd(this,e))return;
        if(!this.drag||this.drag.id!==e.pointerId)return;
        if(!this.moved){const b=canvas.getBoundingClientRect(),x=e.clientX-b.left,y=e.clientY-b.top;
          if(!window.MissionMapOverlay?.mapClick(this,x,y))this.pick(x,y);}
        this.drag=null;canvas.classList.remove('dragging');});
      canvas.addEventListener('pointercancel',e=>{window.MissionMapOverlay?.dragEnd(this,e,true);this.drag=null;canvas.classList.remove('dragging');});
      canvas.addEventListener('keydown',e=>{if(!this.center)return;const s=256*2**this.zoom;
        if(e.key==='+'||e.key==='=')this.setZoom(this.zoom+1);
        else if(e.key==='-')this.setZoom(this.zoom-1);
        else if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key)){
          e.preventDefault();this.center=[wrap(this.center[0]+(e.key==='ArrowRight'?80:e.key==='ArrowLeft'?-80:0)/s),
            clamp(this.center[1]+(e.key==='ArrowDown'?80:e.key==='ArrowUp'?-80:0)/s,0,1)];this.schedule();}});
    }
    update(data) {
      window.MissionMapOverlay?.bindMap(this);
      this.data=data;this.waypoints=(data.waypoints||[]).map((p,i)=>({p,i})).filter(row=>valid(row.p));
      const p=data.position?[data.position.latitude,data.position.longitude]:null;
      this.position=valid(p)?p:null;
      const key=JSON.stringify(this.waypoints);
      if(key!==this.routeKey){this.routeKey=key;this.trail=[];this.selection=null;this.fitAll();}
      if(this.position){
        if(!this.center)this.fitAll();
        const last=this.trail[this.trail.length-1];
        if(!last||last[0]!==p[0]||last[1]!==p[1]){this.trail.push(p);if(this.trail.length>1500)this.trail.shift();}
      }
      this.schedule();
    }
    fitAll() {
      const b=this.el.getBoundingClientRect(),points=this.mapOnly?[...(window.MissionMapOverlay?.pointsForMap?.()||[])]:this.waypoints.map(row=>row.p);
      if(this.position)points.push(this.position);
      if(points.length)Object.assign(this,fit(points,b.width,b.height));
    }
    setZoom(z,anchor=[this.width/2,this.height/2]) {
      if(!this.center)return;
      z=clamp(z,2,23);const old=256*2**this.zoom,next=256*2**z;
      const dx=anchor[0]-this.width/2,dy=anchor[1]-this.height/2;
      this.center=[wrap(this.center[0]+dx/old-dx/next),clamp(this.center[1]+dy/old-dy/next,0,1)];
      this.zoom=z;this.schedule();
    }
    screen(p) {
      const q=project(p),s=256*2**this.zoom;
      return [this.width/2+delta(q[0]-this.center[0])*s,this.height/2+(q[1]-this.center[1])*s];
    }
    pick(x,y) {
      if(this.mapOnly){
        const world=256*2**this.zoom;
        this.pickedGPS=unproject([this.center[0]+(x-this.width/2)/world,
                                 this.center[1]+(y-this.height/2)/world]);
        const output=document.getElementById('mapPickedGPS');
        if(output)output.value=this.pickedGPS[0].toFixed(8)+', '+this.pickedGPS[1].toFixed(8);
        window.dispatchEvent(new Event('gps-picked'));
        this.schedule();return;
      }
      const candidates=this.mapOnly?[]:this.waypoints.map(row=>({...row,name:'WP'+(row.i+1)}));
      if(this.position)candidates.push({p:this.position,name:'현재 선박'});
      this.selection=candidates.map(row=>({...row,d:Math.hypot(...this.screen(row.p).map((v,i)=>v-[x,y][i]))}))
        .filter(row=>row.d<22).sort((a,b)=>a.d-b.d)[0]||null;this.schedule();
    }
    schedule() {
      if(this.queued)return;this.queued=true;
      requestAnimationFrame(()=>{this.queued=false;this.draw();});
    }
    tile(z,x,y) {
      const n=2**z;if(y<0||y>=n)return null;x=((x%n)+n)%n;
      const key=z+'/'+y+'/'+x;let tile=this.tiles.get(key);
      if(tile){this.tiles.delete(key);this.tiles.set(key,tile);return tile;}
      tile={image:new Image(),ready:false,error:false};this.tiles.set(key,tile);
      // The Windows browser fetches only visible imagery; no boat/mission telemetry is uploaded.
      tile.image.referrerPolicy='no-referrer';
      tile.image.onload=()=>{tile.ready=true;this.schedule();};
      tile.image.onerror=()=>{tile.error=true;this.schedule();};
      tile.image.src='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/'+key;
      while(this.tiles.size>160)this.tiles.delete(this.tiles.keys().next().value);
      return tile;
    }
    draw() {
      const b=this.el.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
      this.width=Math.max(1,b.width);this.height=Math.max(1,b.height);
      this.el.width=Math.round(this.width*dpr);this.el.height=Math.round(this.height*dpr);
      const ctx=this.el.getContext('2d'),w=this.width,h=this.height;ctx.scale(dpr,dpr);
      ctx.fillStyle='#142128';ctx.fillRect(0,0,w,h);
      const el=id=>document.getElementById(id);
      el('mapLayer').textContent=this.satellite?'위성':'격자';
      el('mapLayer').setAttribute('aria-pressed',String(this.satellite));
      el('mapCredits').hidden=!this.satellite;
      if(!this.center){this.status.textContent='현재 위치 / 목표 GPS 수신 대기';return;}
      const world=256*2**this.zoom;
      for(let x=-(this.center[0]*world-w/2)%64;x<w;x+=64){ctx.strokeStyle='#2b3b43';ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();}
      for(let y=-(this.center[1]*world-h/2)%64;y<h;y+=64){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
      let ready=0,failed=0,pending=0;
      if(this.satellite){
        // Overzoom z18 imagery: z19 returns an unavailable placeholder at this site.
        const z=Math.min(this.zoom,18),factor=2**(this.zoom-z),size=256*factor;
        const left=this.center[0]*256*2**z-w/(2*factor),top=this.center[1]*256*2**z-h/(2*factor);
        for(let y=Math.floor(top/256);y<=Math.floor((top+h/factor)/256);y++)
          for(let x=Math.floor(left/256);x<=Math.floor((left+w/factor)/256);x++){
            const tile=this.tile(z,x,y);if(!tile)continue;
            if(tile.ready){ctx.drawImage(tile.image,(x*256-left)*factor,(y*256-top)*factor,size+.5,size+.5);ready++;}
            else if(tile.error)failed++;else pending++;
          }
      }
      const path=(points,color,width,dash=[])=>{if(!points.length)return;ctx.beginPath();ctx.strokeStyle=color;
        ctx.lineWidth=width;ctx.setLineDash(dash);points.forEach((p,i)=>{const [x,y]=this.screen(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
        ctx.stroke();ctx.setLineDash([]);};
      const shownWaypoints=this.mapOnly?[]:this.waypoints;
      const wp=shownWaypoints.map(row=>row.p);
      path(wp,'#10202d',5,[8,6]);path(wp,'#f2f7ff',2,[8,6]);if(!this.mapOnly){path(this.trail,'#142128',5);path(this.trail,'#63dec8',2.5);}
      // LiDAR hits are independent of gap/path completion and remain live while planning.
      const lidar=this.data.lidar_display,anchor=lidar?.map_pose;
      if(!this.mapOnly&&lidar?.status==='LIVE'&&this.data.report_age<=1.5&&Number.isFinite(anchor?.yaw)){
        const c=Math.cos(anchor.yaw),s=Math.sin(anchor.yaw),rad=Math.PI/180;
        ctx.fillStyle='#ff6767';
        for(const [x,y] of lidar.points||[]){const east=c*x-s*y,north=s*x+c*y;
          const ll=[anchor.latitude+north/R/rad,anchor.longitude+east/(R*Math.cos(anchor.latitude*rad))/rad];
          const p=this.screen(ll);if(p[0]>=0&&p[0]<=w&&p[1]>=0&&p[1]<=h)ctx.fillRect(p[0]-1.5,p[1]-1.5,3,3);
        }
      }
      window.MissionMapOverlay?.draw(this,ctx,w,h);
      if(this.pickedGPS){
        const [x,y]=this.screen(this.pickedGPS);
        ctx.save();ctx.strokeStyle='#63dec8';ctx.lineWidth=2;ctx.setLineDash([]);
        ctx.beginPath();ctx.arc(x,y,6,0,2*Math.PI);ctx.moveTo(x-11,y);ctx.lineTo(x+11,y);
        ctx.moveTo(x,y-11);ctx.lineTo(x,y+11);ctx.stroke();ctx.restore();
      }
      const occupied=shownWaypoints.map(({p})=>{const [x,y]=this.screen(p);return [x-10,y-10,20,20];});
      const label=(name,x,y)=>{ctx.font='bold 12px "Malgun Gothic",sans-serif';
        const tw=ctx.measureText(name).width+14,th=23;
        const options=[[x+13,y-28],[x+13,y+9],[x-tw-13,y+9],[x-tw-13,y-28],[x-tw/2,y-51]];
        let box=options.map(([a,b])=>[clamp(a,4,Math.max(4,w-tw-4)),clamp(b,40,Math.max(40,h-th-30)),tw,th])
          .find(a=>!occupied.some(b=>a[0]<b[0]+b[2]+3&&a[0]+a[2]+3>b[0]&&a[1]<b[1]+b[3]+3&&a[1]+a[3]+3>b[1]));
        box=box||options[0].concat([tw,th]);occupied.push(box);
        ctx.strokeStyle='#e3f4ff';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(box[0]+tw/2,box[1]+th/2);ctx.stroke();
        ctx.fillStyle='rgba(12,25,34,.9)';ctx.fillRect(...box);ctx.fillStyle='#fff';ctx.fillText(name,box[0]+7,box[1]+16);};
      shownWaypoints.forEach(({p,i})=>{const [x,y]=this.screen(p);if(x<-40||x>w+40||y<-40||y>h+40)return;
        ctx.beginPath();ctx.arc(x,y,7,0,2*Math.PI);ctx.fillStyle=i===this.data.wp_index?'#63dec8':'#4d94e2';ctx.fill();
        ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();label('WP'+(i+1),x,y);});
      if(this.position){const [x,y]=this.screen(this.position),yaw=this.data.yaw;
        ctx.save();ctx.translate(x,y);ctx.fillStyle='#ffb270';ctx.strokeStyle='#13212b';ctx.lineWidth=2.5;ctx.beginPath();
        if(Number.isFinite(yaw)){ctx.rotate(-yaw);ctx.moveTo(15,0);ctx.lineTo(-10,-8);ctx.lineTo(-5,0);ctx.lineTo(-10,8);ctx.closePath();}
        else ctx.arc(0,0,7,0,2*Math.PI);ctx.fill();ctx.stroke();ctx.restore();label('선박',x,y);}
      const lat=unproject(this.center)[0],mpp=2*Math.PI*R*Math.cos(lat*Math.PI/180)/world;
      this.scaleLabel.textContent=(50*mpp<10?(50*mpp).toFixed(1):Math.round(50*mpp))+' m / 50 px · Z'+this.zoom;
      const gps=(!this.position?' · GPS 위치 대기':this.data.gps_age>2||this.data.report_age>1.5?' · GPS/상태 지연':'');
      this.status.textContent=(this.satellite?(failed?(ready?'영상 일부 미수신':'영상 미수신 · 인터넷 확인'):pending?'위성영상 로딩…':'위성영상'):'좌표 격자')+gps;
      if(this.selection)this.info.textContent=this.selection.name+'  '+this.selection.p[0].toFixed(7)+', '+this.selection.p[1].toFixed(7);
      else this.info.textContent=this.mapOnly?'임무점 드래그: GPS 이동 · 빈 곳 드래그: 지도 이동 · 저장 버튼으로 반영':this.waypoints.length?'WP 클릭: 좌표 확인 · 드래그: 이동 · 휠: 확대':'설정된 WP 없음';
      el('mapBoat').disabled=!this.position;el('mapPlus').disabled=this.zoom>=23;el('mapMinus').disabled=this.zoom<=2;
    }
  }
  return {View,project,unproject,valid,fit,delta};
})();

const $=id=>document.getElementById(id);
let latest=null,map=null,busy=false,cameraBusy=false;
const shortNames=['시작점','경로 3점','호핑','도킹 접근','도킹','탐색','회피 접근','회피 주행'];
shortNames.forEach((name,i)=>{const node=document.createElement('span');node.id='stage'+i;node.textContent=i+' · '+name;$('stages').append(node)});
const number=(v,unit='',places=1)=>Number.isFinite(v)?v.toFixed(places)+unit:'—';
const pct=v=>Number.isFinite(v)?((v-1500)/4).toFixed(1)+'%':'—';
const angle=v=>Number.isFinite(v)?(v>0?'좌 ':v<0?'우 ':'')+Math.abs(v).toFixed(1)+'°':'—';
function phase(m){
 if(!m.state)return '주행 프로그램 상태 대기';
 if(m.state.endsWith('_REVERSE'))return '도착 후 후진 명령';
 if(m.state.endsWith('_WAIT'))return '도착 후 중립 대기';
 if(m.state.startsWith('WAIT_'))return '입력 대기 · '+m.state;
 if(m.state==='OUTPUT_STOPPED')return '출력 정지';
 if(m.state.startsWith('STATUS'))return '다음 임무로 전환';
 return '목표 추종 중';
}
function update(data){
 latest=data;const m=data.mission||{},live=Number.isFinite(data.mission_age)&&data.mission_age<=1.5;
 $('connection').textContent=live?'● 주행 상태 수신 중':'● 주행 상태 대기 / 지연';
 $('connection').style.color=live?'#69e0ba':'#ffbb88';
 shortNames.forEach((_,i)=>$('stage'+i).classList.toggle('active',m.status===i));
 $('missionTitle').textContent=Number.isInteger(m.status)?'status '+m.status+' · '+data.missions[m.status]:'status 대기';
 $('missionState').textContent=phase(m)+(m.status===1?' · 경로 '+m.route_point+'/3':m.status===7?(m.avoidance_final_goal?' · 최종 목표':' · 중간 지점 '+m.avoidance_point):'');
 const actual=data.actuator||{},mcuFresh=Number.isFinite(data.actuator_age)&&data.actuator_age<=.5&&actual.receiving===true;
 const running=live&&m.active&&m.armed&&mcuFresh&&actual.armed;
 $('output').textContent=running?'추진 활성':live?'출력 대기 / 중립':'연결 대기';$('output').classList.toggle('live',running);
 let mode=m.status===4?(m.docking_mode==='CAMERA'?'카메라':m.docking_mode==='GPS'?'GPS':'대기'):[2,3,7].includes(m.status)?'V3 회피':'GPS';
 if(m.state?.endsWith('_REVERSE'))mode='후진';else if(m.state?.endsWith('_WAIT'))mode='중립 대기';
 $('controlMode').textContent=mode;
 $('target').textContent=m.status===4?m.docking_target:m.status===5?m.search_target:Number.isFinite(m.goal_latitude)?number(m.goal_latitude,'',7)+', '+number(m.goal_longitude,'',7):'미지정';
 $('angle').textContent=angle(m.final_target_deg);$('distance').textContent=number(m.distance_m,' m',2);
 $('left').textContent=pct(m.intended_pwm?.[0])+' / '+(mcuFresh?pct(actual.left):'미수신');
 $('right').textContent=pct(m.intended_pwm?.[1])+' / '+(mcuFresh?pct(actual.right):'미수신');
 $('fault').textContent=m.output_error?m.output_error:!live?'주행 상태가 최신이 아닙니다. 표시값은 마지막 수신 값입니다.':'';
 const sensorRoot=$('sensors');sensorRoot.replaceChildren();
 for(const [name,age,limit] of [['GPS',data.gps_age,2],['IMU',data.imu_age,.5],['MCU',data.actuator_age,.5],['라이다',data.scan_age,.5],['카메라',data.camera_age,2]]){
  const el=document.createElement('span');el.className=Number.isFinite(age)&&age<=limit?'live':'';el.textContent=name+' '+(Number.isFinite(age)?age.toFixed(1)+'초':'대기');sensorRoot.append(el);
 }
 $('position').textContent=data.position?'현재 선박 GPS · '+number(data.position.latitude,'',8)+', '+number(data.position.longitude,'',8):'현재 선박 GPS 대기';
 const cameraFresh=Number.isFinite(data.camera_age)&&data.camera_age<2;
 $('cameraStatus').textContent=cameraFresh?'검출 영상 수신 중':'영상 대기 / 지연';$('cameraWait').hidden=cameraFresh;
 $('cameraWait').textContent=data.camera_age===null?'카메라 영상 대기':'카메라 영상 지연 · 마지막 화면';
 $('objectInfo').textContent=m.status===4?'도킹 대상: '+m.docking_target+' · '+(m.docking_mode==='CAMERA'?'목표 검출 · 카메라 제어':'카메라 추종 비활성 · '+mode):m.status===5?'탐색 대상: '+m.search_target+' · 지정 GPS 추종':'객체 검출 화면 · 카메라 도킹은 status 4에서 사용';
 $('scanStatus').textContent=(Number.isFinite(data.scan_age)&&data.scan_age<=.5?'수신 중':'대기 / 지연')+' · '+(data.scan_source||'—');
 const mapPoints=(data.configured_points||[]).map(point=>point.gps);
 map.update({...data,report_age:data.mission_age,waypoints:mapPoints,wp_index:0});
 drawRadar();
}
// Object colors affect map drawing only; mission selection/GPS/control are unchanged.
function missionObjectColor(point){
 if(![4,5].includes(point.status))return null;
 const name=String(point.label||point.name||'').toLowerCase();
 if(/green|초록/.test(name))return '#43dc83';
 if(/red|빨강/.test(name))return '#ff5353';
 if(/blue|파랑|파란/.test(name))return '#499dff';
 if(/black|검정|검은/.test(name))return '#151515';
 return null;
}
let gpsDraft=null;
function editableMapPoints(){
 const points=gpsConfig?.slots?.map(slot=>({id:slot.id,status:Number(slot.id.split('/')[0]),label:slot.label,gps:slot.gps}))||latest?.configured_points||[];
 return points.map(p=>gpsDraft?.id===p.id?{...p,gps:gpsDraft.gps,label:p.label+' (미저장)'}:p)
  .filter(p=>SatelliteMap.valid(p.gps));
}
function orbitPreviewPoints(center,count,radius){
 const lat=center[0]*Math.PI/180,lon=center[1]*Math.PI/180,d=radius/6371008.8;
 const origin=[Math.cos(lat)*Math.cos(lon),Math.cos(lat)*Math.sin(lon),Math.sin(lat)];
 const north=[-Math.sin(lat)*Math.cos(lon),-Math.sin(lat)*Math.sin(lon),Math.cos(lat)];
 const east=[-Math.sin(lon),Math.cos(lon),0];
 return Array.from({length:count},(_,i)=>{
  const bearing=2*Math.PI*i/count,n=Math.sin(d)*Math.cos(bearing),e=Math.sin(d)*Math.sin(bearing);
  const [x,y,z]=origin.map((v,k)=>v*Math.cos(d)+north[k]*n+east[k]*e);
  return [Math.atan2(z,Math.hypot(x,y))*180/Math.PI,Math.atan2(y,x)*180/Math.PI];
 });
}
function displayBuoyOrbits(){
 return (latest?.buoy_orbits||[]).map(o=>{
  if(gpsDraft?.id!=='5/gps_by_buoy/'+o.name)return o;
  return {...o,center:gpsDraft.gps,points:orbitPreviewPoints(gpsDraft.gps,o.points.length,o.radius_m)};
 });
}
function resetGPSDraft(){
 gpsDraft=null;if(map){map.pickedGPS=null;map.schedule()}
 $('mapPickedGPS').value='';updateGPSChoice();
}
function stageGPSDraft(){
 const slot=gpsChoice();
 if(slot&&map.pickedGPS){gpsDraft={id:slot.id,gps:map.pickedGPS.slice()};
  $('gpsSaveStatus').textContent=slot.label+' 이동 미리보기 · 저장 버튼을 눌러야 설정에 반영됩니다.';}
 updateGPSChoice();map.schedule();
}
function setPickedGPS(view,gps){
 view.pickedGPS=gps.slice();$('mapPickedGPS').value=gps.map(v=>v.toFixed(8)).join(', ');
 window.dispatchEvent(new Event('gps-picked'));view.schedule();
}
window.MissionMapOverlay={
 pointsForMap:()=>[...editableMapPoints().map(point=>point.gps),...(latest?.waypoints||[]),
  ...displayBuoyOrbits().flatMap(o=>o.points)],
 bindMap(){},
 dragStart(view,e){
  if(gpsSaving||!gpsConfig)return false;
  const b=view.el.getBoundingClientRect(),x=e.clientX-b.left,y=e.clientY-b.top;
  const selected=$('gpsMission').value;
  const hits=editableMapPoints().map(p=>({...p,xy:view.screen(p.gps)}))
   .map(p=>({...p,d:Math.hypot(p.xy[0]-x,p.xy[1]-y)})).filter(p=>p.d<=12)
   .sort((a,b)=>Math.abs(a.d-b.d)<1?Number(b.id===selected)-Number(a.id===selected):a.d-b.d);
  const hit=hits[0];if(!hit)return false;
  if(hit.id!==selected){$('gpsMission').value=hit.id;resetGPSDraft()}
  view.pointDrag={id:e.pointerId,slot:hit.id,x:e.clientX,y:e.clientY,
   offset:[x-hit.xy[0],y-hit.xy[1]],moved:false,previous:gpsDraft};
  view.el.setPointerCapture(e.pointerId);view.el.classList.add('dragging');updateGPSChoice();
  return true;
 },
 dragMove(view,e){
  const drag=view.pointDrag;if(!drag||drag.id!==e.pointerId)return false;
  if(!drag.moved&&Math.hypot(e.clientX-drag.x,e.clientY-drag.y)<=3)return true;
  drag.moved=true;
  const b=view.el.getBoundingClientRect(),world=256*2**view.zoom;
  const x=e.clientX-b.left-drag.offset[0],y=e.clientY-b.top-drag.offset[1];
  setPickedGPS(view,SatelliteMap.unproject([view.center[0]+(x-view.width/2)/world,
   view.center[1]+(y-view.height/2)/world]));
  return true;
 },
 dragEnd(view,e,cancel=false){
  const drag=view.pointDrag;if(!drag||drag.id!==e.pointerId)return false;
  if(cancel){gpsDraft=drag.previous;view.pickedGPS=gpsDraft?.gps?.slice()||null;
   $('mapPickedGPS').value=view.pickedGPS?view.pickedGPS.map(v=>v.toFixed(8)).join(', '):'';updateGPSChoice();}
  else this.dragMove(view,e);
  view.pointDrag=null;view.el.classList.remove('dragging');
  if(view.el.hasPointerCapture(e.pointerId))view.el.releasePointerCapture(e.pointerId);
  view.schedule();return true;
 },
 mapClick(view,x,y){
  const near=displayBuoyOrbits().flatMap(o=>o.points).map(gps=>({gps,xy:view.screen(gps)}))
   .map(p=>({...p,d:Math.hypot(p.xy[0]-x,p.xy[1]-y)})).filter(p=>p.d<=9).sort((a,b)=>a.d-b.d)[0];
  if(!near)return false;
  view.pickedGPS=near.gps.slice();
  $('mapPickedGPS').value=near.gps.map(v=>v.toFixed(8)).join(', ');
  window.dispatchEvent(new Event('gps-picked'));view.schedule();return true;
 },
 draw(view,ctx,w,h){
  const colors=['#66c6ff','#82a6ff','#f7c66d','#ffad80','#e39cff','#ffe28a','#7fd9df','#ff8c9b'];
  const live=Number.isFinite(latest?.mission_age)&&latest.mission_age<=1.5;
  const target=live?latest.waypoints?.[0]:null;
  const points=editableMapPoints();
  if(target&&!points.some(p=>p.status===latest.mission.status&&p.gps.every((v,i)=>v===target[i]))){
   points.push({status:latest.mission.status,label:'현재 목표',gps:target});
  }
  // Fixed preview point numbers are not the live vessel's visit order.
  for(const orbit of displayBuoyOrbits()){
   const ring=orbit.points||[];if(!ring.length)continue;
   ctx.save();const ringColor=missionObjectColor({status:5,name:orbit.name})||colors[5];ctx.strokeStyle=ringColor;
   ctx.lineWidth=1.5;ctx.setLineDash([5,4]);ctx.beginPath();
   ring.forEach((gps,i)=>{const [x,y]=view.screen(gps);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});
   ctx.closePath();
   ctx.strokeStyle='#eef5fa';ctx.lineWidth=3.5;ctx.stroke();
   ctx.strokeStyle=ringColor;ctx.lineWidth=1.5;ctx.stroke();ctx.restore();
   ring.forEach((gps,i)=>points.push({status:5,gps,orbit:true,
    label:orbit.name+' · 선회점 '+(i+1)}));
  }
  const marks=points.map(p=>({...p,color:missionObjectColor(p),xy:view.screen(p.gps),active:target&&p.status===latest.mission.status&&p.gps.every((v,i)=>v===target[i])}))
   .filter(({xy:[x,y]})=>x>=0&&y>=0&&x<=w&&y<=h);
  ctx.save();
  for(const {xy:[x,y],status,active,color} of marks){
   ctx.beginPath();ctx.arc(x,y,active?8:5,0,Math.PI*2);ctx.fillStyle=color||(active?'#69e0ba':colors[status]);ctx.fill();
   ctx.lineWidth=active?3:1.5;ctx.strokeStyle=active||color==='#151515'?'#fff':'#10202d';ctx.stroke();
  }
  ctx.font='bold 11px sans-serif';
  const occupied=marks.map(({xy:[x,y]})=>[x-8,y-8,16,16]);
  occupied.push([0,0,330,96],[0,h-48,w,48]);
  marks.sort((a,b)=>Number(Boolean(b.active))-Number(Boolean(a.active)));
  for(const {xy:[x,y],label,status,active,color} of marks){
   const name=active?'▶ '+label:label;
   const width=ctx.measureText(name).width+12,height=18;
   let box;
   for(const dy of [-26,12,-47,33,-68,54,-89,75]){
    for(const dx of [10,-width-10,-width/2]){
     const candidate=[Math.max(3,Math.min(w-width-3,x+dx)),y+dy,width,height];
     const [a,b]=candidate;
     if(b>=3&&b+height<=h-3&&!occupied.some(([ox,oy,ow,oh])=>a<ox+ow+2&&a+width+2>ox&&b<oy+oh+2&&b+height+2>oy)){box=candidate;break}
    }
    if(box)break;
   }
   if(!box)continue;
   occupied.push(box);
   const [a,b]=box;
   ctx.strokeStyle=color==='#151515'?'#d2d7dd':color||(active?'#69e0ba':colors[status]);ctx.lineWidth=1;ctx.beginPath();
   ctx.moveTo(x,y);ctx.lineTo(a+width/2,b+height/2);ctx.stroke();
   ctx.fillStyle=color==='#151515'?'#151515':'#10202deb';ctx.fillRect(...box);
   if(color==='#151515'){ctx.strokeStyle='#d2d7dd';ctx.strokeRect(...box)}
   ctx.fillStyle=color==='#151515'?'#fff':color||(active?'#69e0ba':colors[status]);
   ctx.fillText(name,a+6,b+13);
  }
  ctx.restore();
 }
};
function drawRadar(){
 const el=$('radar'),b=el.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(1,b.width),h=Math.max(1,b.height);
 el.width=Math.round(w*dpr);el.height=Math.round(h*dpr);const ctx=el.getContext('2d');ctx.scale(dpr,dpr);
 ctx.fillStyle='#101c25';ctx.fillRect(0,0,w,h);const cx=w/2,cy=h/2,scale=Math.min(w,h)*.43/5;
 ctx.strokeStyle='#2a4254';ctx.fillStyle='#91a8b9';ctx.font='10px sans-serif';
 for(let r=1;r<=5;r++){ctx.beginPath();ctx.arc(cx,cy,r*scale,0,Math.PI*2);ctx.stroke();ctx.fillText(r+'m',cx+5,cy-r*scale-3)}
 ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,h);ctx.moveTo(0,cy);ctx.lineTo(w,cy);ctx.stroke();
 ctx.fillText('선수',cx+8,18);ctx.fillText('좌현',10,cy-8);ctx.fillText('우현',w-35,cy-8);
 const scanLive=latest&&Number.isFinite(latest.scan_age)&&latest.scan_age<=.5;
 ctx.fillStyle=scanLive?'#b3cadb':'#4f6270';for(const [x,y] of latest?.scan||[]){if(Math.hypot(x,y)>5)continue;ctx.fillRect(cx-y*scale-1.5,cy-x*scale-1.5,3,3)}
 const live=latest&&Number.isFinite(latest.mission_age)&&latest.mission_age<=1.5;
 for(const [a,color,length]of [[latest?.mission?.target_deg,'#ffb270',3.6],[latest?.mission?.final_target_deg,'#69e0ba',3.]]){
  if(!live||!Number.isFinite(a))continue;const rad=a*Math.PI/180;ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx-Math.sin(rad)*length*scale,cy-Math.cos(rad)*length*scale);ctx.stroke();
 }
 ctx.fillStyle='#edf5fa';ctx.beginPath();ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+6);ctx.lineTo(cx+6,cy+6);ctx.closePath();ctx.fill();
}
async function poll(){if(busy)return;busy=true;try{const response=await fetch('/api/state',{cache:'no-store'});if(!response.ok)throw Error('state');update(await response.json())}catch(e){$('connection').textContent='● GUI 서버 연결 지연';$('fault').textContent='서버 연결을 확인하세요. 표시값은 마지막 수신 값입니다.'}finally{busy=false}}
function refreshCamera(){if(cameraBusy||!latest||latest.camera_age===null)return;cameraBusy=true;$('camera').src='/camera.jpg?t='+Date.now()}
$('camera').onload=$('camera').onerror=()=>{cameraBusy=false};
map=new SatelliteMap.View($('map'));
window.addEventListener('resize',()=>{map.schedule();drawRadar()});
poll();setInterval(poll,400);setInterval(refreshCamera,700);

// GPS configuration only; no ROS commands or mission starts are sent.
let gpsConfig=null,gpsSaving=false;
function gpsChoice(){return gpsConfig?.slots.find(s=>s.id===$('gpsMission').value)}
function updateGPSChoice(){
 const slot=gpsChoice();
 $('gpsCurrent').textContent=slot?'현재 저장 GPS: '+(slot.gps?.every(Number.isFinite)?slot.gps.map(v=>v.toFixed(8)).join(', '):'미지정'):'';
 $('gpsSave').disabled=gpsSaving||!slot||!map?.pickedGPS;
 $('gpsCancel').disabled=gpsSaving||!gpsDraft;
}
async function loadGPSConfig(){
 const old=$('gpsMission').value;
 const response=await fetch('/api/gps-config',{cache:'no-store'});
 if(!response.ok)throw Error('GPS 설정을 불러오지 못했습니다. GUI 서버 업데이트가 필요합니다.');
 gpsConfig=await response.json();$('gpsMission').replaceChildren();
 for(const slot of gpsConfig.slots){const o=document.createElement('option');o.value=slot.id;o.textContent=slot.label;$('gpsMission').append(o)}
 if(gpsConfig.slots.some(s=>s.id===old))$('gpsMission').value=old;
 resetGPSDraft();
}
$('gpsMission').onchange=resetGPSDraft;
window.addEventListener('gps-picked',stageGPSDraft);
$('gpsCancel').onclick=()=>{resetGPSDraft();$('gpsSaveStatus').textContent='미저장 이동 취소 · 저장된 GPS는 유지됩니다.'};
$('gpsReload').onclick=()=>loadGPSConfig().catch(e=>{$('gpsSaveStatus').textContent=e.message});
$('gpsSave').onclick=async()=>{
 const slot=gpsChoice(),gps=map.pickedGPS?.slice(),revision=gpsConfig?.revision;
 if(!slot||!gps||gpsSaving)return;
 gpsSaving=true;$('gpsMission').disabled=$('gpsReload').disabled=true;updateGPSChoice();
 try{
  const response=await fetch('/api/gps-config',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({id:slot.id,gps,revision})});
  const result=await response.json();if(!response.ok)throw Error(result.error||'GPS 저장 실패');
  await loadGPSConfig();await poll();
  $('gpsSaveStatus').textContent=slot.label+' 저장 완료: '+gps.map(v=>v.toFixed(8)).join(', ')+' · 다음 주행 실행부터 적용됩니다.';
 }catch(e){$('gpsSaveStatus').textContent=e.message}
 finally{gpsSaving=false;$('gpsMission').disabled=$('gpsReload').disabled=false;updateGPSChoice()}
};
loadGPSConfig().catch(e=>{$('gpsSaveStatus').textContent=e.message});
