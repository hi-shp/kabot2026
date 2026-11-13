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
      this.el=canvas;this.center=null;this.zoom=19;this.satellite=true;this.tiles=new Map();
      this.trail=[];this.routeKey=null;this.data={};this.queued=false;this.selection=null;
      this.width=1;this.height=1;this.moved=false;
      const el=id=>document.getElementById(id);
      this.status=el('mapStatus');this.info=el('mapInfo');this.scaleLabel=el('mapscale');
      el('mapFit').onclick=()=>{this.fitAll();this.schedule();};
      el('mapPlus').onclick=()=>this.setZoom(this.zoom+1);
      el('mapMinus').onclick=()=>this.setZoom(this.zoom-1);
      el('mapLayer').onclick=()=>{this.satellite=!this.satellite;this.schedule();};
      el('mapBoat').onclick=()=>{if(this.position){this.center=project(this.position);this.schedule();}};
      canvas.addEventListener('wheel',e=>{e.preventDefault();const b=canvas.getBoundingClientRect();
        this.setZoom(this.zoom+(e.deltaY<0?1:-1),[e.clientX-b.left,e.clientY-b.top]);},{passive:false});
      canvas.addEventListener('pointerdown',e=>{if(e.button!==0||!this.center)return;
        this.drag={id:e.pointerId,x:e.clientX,y:e.clientY,center:[...this.center]};this.moved=false;
        canvas.setPointerCapture(e.pointerId);canvas.classList.add('dragging');});
      canvas.addEventListener('pointermove',e=>{if(!this.drag||this.drag.id!==e.pointerId)return;
        const dx=e.clientX-this.drag.x,dy=e.clientY-this.drag.y,s=256*2**this.zoom;
        if(Math.hypot(dx,dy)>3)this.moved=true;
        this.center=[wrap(this.drag.center[0]-dx/s),clamp(this.drag.center[1]-dy/s,0,1)];this.schedule();});
      canvas.addEventListener('pointerup',e=>{if(!this.drag||this.drag.id!==e.pointerId)return;
        if(!this.moved){const b=canvas.getBoundingClientRect();this.pick(e.clientX-b.left,e.clientY-b.top);}
        this.drag=null;canvas.classList.remove('dragging');});
      canvas.addEventListener('pointercancel',()=>{this.drag=null;canvas.classList.remove('dragging');});
      canvas.addEventListener('keydown',e=>{if(!this.center)return;const s=256*2**this.zoom;
        if(e.key==='+'||e.key==='=')this.setZoom(this.zoom+1);
        else if(e.key==='-')this.setZoom(this.zoom-1);
        else if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key)){
          e.preventDefault();this.center=[wrap(this.center[0]+(e.key==='ArrowRight'?80:e.key==='ArrowLeft'?-80:0)/s),
            clamp(this.center[1]+(e.key==='ArrowDown'?80:e.key==='ArrowUp'?-80:0)/s,0,1)];this.schedule();}});
    }
    update(data) {
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
      const b=this.el.getBoundingClientRect(),points=this.waypoints.map(row=>row.p);
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
      const candidates=this.waypoints.map(row=>({...row,name:'WP'+(row.i+1)}));
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
      if(!this.center){this.status.textContent='WP 좌표 / GPS 수신 대기';return;}
      const world=256*2**this.zoom;
      for(let x=-(this.center[0]*world-w/2)%64;x<w;x+=64){ctx.strokeStyle='#2b3b43';ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();}
      for(let y=-(this.center[1]*world-h/2)%64;y<h;y+=64){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
      let ready=0,failed=0,pending=0;
      if(this.satellite){
        // Overzoom z19 imagery at closer scales: avoids blank high-detail tiles in uncovered areas.
        const z=Math.min(this.zoom,19),factor=2**(this.zoom-z),size=256*factor;
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
      const wp=this.waypoints.map(row=>row.p);
      path(wp,'#10202d',5,[8,6]);path(wp,'#f2f7ff',2,[8,6]);path(this.trail,'#142128',5);path(this.trail,'#63dec8',2.5);
      const occupied=this.waypoints.map(({p})=>{const [x,y]=this.screen(p);return [x-10,y-10,20,20];});
      const label=(name,x,y)=>{ctx.font='bold 12px "Malgun Gothic",sans-serif';
        const tw=ctx.measureText(name).width+14,th=23;
        const options=[[x+13,y-28],[x+13,y+9],[x-tw-13,y+9],[x-tw-13,y-28],[x-tw/2,y-51]];
        let box=options.map(([a,b])=>[clamp(a,4,Math.max(4,w-tw-4)),clamp(b,40,Math.max(40,h-th-30)),tw,th])
          .find(a=>!occupied.some(b=>a[0]<b[0]+b[2]+3&&a[0]+a[2]+3>b[0]&&a[1]<b[1]+b[3]+3&&a[1]+a[3]+3>b[1]));
        box=box||options[0].concat([tw,th]);occupied.push(box);
        ctx.strokeStyle='#e3f4ff';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(box[0]+tw/2,box[1]+th/2);ctx.stroke();
        ctx.fillStyle='rgba(12,25,34,.9)';ctx.fillRect(...box);ctx.fillStyle='#fff';ctx.fillText(name,box[0]+7,box[1]+16);};
      this.waypoints.forEach(({p,i})=>{const [x,y]=this.screen(p);if(x<-40||x>w+40||y<-40||y>h+40)return;
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
      else this.info.textContent=this.waypoints.length?'WP 클릭: 좌표 확인 · 드래그: 이동 · 휠: 확대':'설정된 WP 없음';
      el('mapBoat').disabled=!this.position;el('mapPlus').disabled=this.zoom>=23;el('mapMinus').disabled=this.zoom<=2;
    }
  }
  return {View,project,unproject,valid,fit,delta};
})();
/* END SATELLITE MAP */

'use strict';
const viewerOnly=false; // Mission commands still pass the Jetson controller's validation.
const $=id=>document.getElementById(id);
const labels={TO_WP1:'WP1 이동',WP1_TO_WP2:'WP1 → WP2 주행',AVOID_OBSTACLE:'장애물 회피',WAYPOINT_ARRIVAL:'웨이포인트 도착 확인',WP2_HOLD:'WP2 · 10초 정지',WP2_TO_WP3_ALIGN:'WP3 방향 정렬',SEARCH_SHAPE:'도형 탐색',VERIFY_TARGET:'도형 확인 중',ALIGN_SHAPE:'도형 방향 정렬',APPROACH_SHAPE:'도형 접근',ENTER_REGION:'구역 진입',REGION_ARRIVAL_HOLD:'도킹 정지 확인',MISSION_COMPLETE:'임무 완료',WAIT_SCAN:'라이다 대기',WAIT_GPS:'GPS 대기',WAIT_HEADING:'IMU 방향 대기',WAIT_TARGET:'도형 대기',TARGET_LOST:'목표 유실 · 정지',NO_SAFE_GAP:'통과 구간 없음 · 정지',OBSTACLE_STOP:'근접 장애물 · 정지',GPS_UNCERTAIN:'GPS 정확도 확인',WAIT_SYNC:'센서 시각 정렬 대기',WP3_POSITION_MISMATCH:'도형 위치와 WP3 불일치'};
let latest=null,trail=[],lastPosition=null,settingsLoaded=false,commandBusy=false,lastCommand=null;
for(const color of ['blue','green','red'])for(const shape of ['circle','cross','triangle']){let option=document.createElement('option');option.value=option.textContent=color+' '+shape;$('target').append(option)}
function text(id,value){$(id).textContent=value??'—'}
function fixed(value,digits=1,suffix=''){return Number.isFinite(value)?value.toFixed(digits)+suffix:'—'}
async function command(action,extra={}){
 if(commandBusy)return;
 if(!latest?.settings_available){text('notice','Windows GUI 연결 서비스를 기다리는 중입니다. 업데이트 후 임무/보기 프로그램을 다시 실행하세요.');return;}
 commandBusy=true;text('notice','명령 처리 중…');
 try{const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra})});
 const result=await response.json();if(!response.ok)throw Error(result.error||'명령 전송 실패');
 text('notice',(result.success?'✓ ':'! ')+result.message);
 if(result.success&&['set_route','capture_wp','set_target'].includes(action))settingsLoaded=false;
 }catch(error){text('notice',error.message)}finally{commandBusy=false;}
}
$('start').onclick=()=>command('start');$('stop').onclick=()=>command('stop');$('reset').onclick=()=>command('reset');
$('applyRoute').onclick=()=>{const values=[0,1,2].map(i=>[Number($('lat'+i).value),Number($('lon'+i).value)]);if(values.some(([lat,lon])=>!Number.isFinite(lat)||!Number.isFinite(lon)||Math.abs(lat)>=85||Math.abs(lon)>180)){text('notice','유효한 위도·경도를 입력하세요.');return}command('set_route',{waypoints:values})};
document.querySelectorAll('[data-capture]').forEach(button=>button.onclick=()=>command('capture_wp',{index:Number(button.dataset.capture)}));
$('applyTarget').onclick=()=>command('set_target',{target_label:$('target').value});
$('export').onclick=()=>{const data={wp_latitudes:[0,1,2].map(i=>Number($('lat'+i).value)),wp_longitudes:[0,1,2].map(i=>Number($('lon'+i).value)),target_label:$('target').value,wp2_stop_seconds:10};const blob=new Blob([JSON.stringify(data,null,2)],{type:'application/json'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='course_settings.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
function canvas(id){const el=$(id),bounds=el.getBoundingClientRect(),dpr=devicePixelRatio||1;const w=Math.max(1,bounds.width),h=Math.max(1,bounds.height);el.width=Math.round(w*dpr);el.height=Math.round(h*dpr);const ctx=el.getContext('2d');ctx.scale(dpr,dpr);return {ctx,w,h}}
function line(ctx,points,color,width=1,dash=[]){if(!points.length)return;ctx.beginPath();ctx.setLineDash(dash);ctx.strokeStyle=color;ctx.lineWidth=width;points.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.stroke();ctx.setLineDash([])}
let gpsMap=null;
function drawMap(data){if(!gpsMap)gpsMap=new SatelliteMap.View($('map'));gpsMap.update(data)}
function drawRadar(data){const {ctx,w,h}=canvas('radar'),cx=w/2,cy=h-30,scale=Math.min(w*.43,h-40)/3;for(const r of [1,2,3]){ctx.beginPath();ctx.arc(cx,cy,r*scale,Math.PI,2*Math.PI);ctx.strokeStyle='#31444c';ctx.stroke();ctx.fillStyle='#78919a';ctx.font='10px Consolas';ctx.fillText(r+'m',cx+5,cy-r*scale+11)}line(ctx,[[cx,cy],[cx,10]],'#34464e',1,[4,5]);for(const [x,y]of data.slam_display?.body_points||data.avoidance?.obstacles||[]){if(x<0||Math.hypot(x,y)>4)continue;ctx.beginPath();ctx.arc(cx-y*scale,cy-x*scale,2.5,0,Math.PI*2);ctx.fillStyle='#ffb270';ctx.fill()}const angle=data.avoidance?.selected_angle_deg;if(Number.isFinite(angle)){let r=angle*Math.PI/180;line(ctx,[[cx,cy],[cx-Math.sin(r)*2.8*scale,cy-Math.cos(r)*2.8*scale]],'#63dec8',3)}ctx.fillStyle='#dfeff4';ctx.fillRect(cx-7,cy-10,14,24);ctx.font='10px Segoe UI';ctx.fillStyle='#8a9fa7';ctx.fillText('LEFT',12,cy+20);ctx.fillText('FORWARD',cx-24,16);ctx.fillText('RIGHT',w-45,cy+20)}
function drawPWM(data){const {ctx,w,h}=canvas('pwm'),hist=data.history||[],left=40,right=w-12,top=12,bottom=h-22;const y=value=>bottom-(value-1100)/800*(bottom-top);ctx.font='9px Consolas';for(const value of [1100,1500,1900]){line(ctx,[[left,y(value)],[right,y(value)]],value===1500?'#60747c':'#34424a',1,value===1500?[4,4]:[]);ctx.fillStyle='#8da4ad';ctx.fillText(value,6,y(value)+3)}if(hist.length<2)return;for(const [key,index,color,dash]of [['pwm',0,'#63dec8',[]],['pwm',1,'#ffb270',[]],['feedback',0,'#63dec8',[3,3]],['feedback',1,'#ffb270',[3,3]]]){let points=[];hist.forEach((row,i)=>{const value=row[key]?.[index];if(Number.isFinite(value))points.push([left+i/(hist.length-1)*(right-left),y(value)])});line(ctx,points,color,key==='pwm'?1.7:1,dash)}}
function render(data){latest=data;const stale=data.report_age===null||data.report_age>1.5;const demo=Boolean(data.demo);text('connection',stale?'● 연결 지연':'● '+(demo?'시뮬레이션':'ROS 연결'));text('mode',demo?'DEMO · 가상 센서':data.output?.enabled?'OUTPUT ENABLED':'PREVIEW');text('state',labels[data.state]||data.state);text('detail',data.stage==='TO_WP1'?'WP1부터 순서대로 진행':'단계: '+(data.stage||'—'));text('countdown',data.stage==='WP2_HOLD'?fixed(data.hold_remaining,1,'초 남음'):'10초 중립 유지');const stage=data.stage==='COMPLETE'?3:data.stage==='WP2_HOLD'?1:['DOCK_TO_WP3','WP2_TO_WP3_ALIGN'].includes(data.stage)?2:0;for(let i=0;i<4;i++)$('stage'+i).classList.toggle('active',i===stage);text('wp','WP'+((data.wp_index??0)+1));text('distance',fixed(data.distance_to_wp,1,' m'));text('bearing',fixed(data.heading_error_deg,1,'°'));text('sigma',fixed(data.position?.sigma,2,' m'));text('position',data.position?fixed(data.position.latitude,7)+' / '+fixed(data.position.longitude,7):'위치 대기');text('gap',data.avoidance?.gaps?.length?data.avoidance.gaps.length+'개 통과 후보':'관측 대기');text('targetLabel',data.target_label);text('frames',(data.stable_frames||0)+' / 4');text('targetDistance',fixed(data.target?.distance,2,' m'));text('left',data.intended_pwm?.[0]);text('right',data.intended_pwm?.[1]);text('leftFeedback','피드백 '+(data.feedback_pwm?.[0]??'—'));text('rightFeedback','피드백 '+(data.feedback_pwm?.[1]??'—'));text('outputState',data.output?.active?'임무 실행 중':data.output?.enabled?'시작 대기 / 정지':'출력 비활성');$('start').disabled=stale||!data.output?.enabled||data.output?.active||data.stage==='COMPLETE';const editing=data.output?.active||stale;['applyRoute','applyTarget','reset'].forEach(id=>$(id).disabled=editing);document.querySelectorAll('[data-capture]').forEach(b=>b.disabled=editing);if(!settingsLoaded&&data.waypoints){data.waypoints.forEach(([lat,lon],i)=>{$('lat'+i).value=lat;$('lon'+i).value=lon});$('target').value=data.target_label;settingsLoaded=true}if(data.last_command&&JSON.stringify(data.last_command)!==lastCommand){lastCommand=JSON.stringify(data.last_command);text('notice',(data.last_command.success?'✓ ':'! ')+data.last_command.message);if(data.last_command.success&&['set_route','capture_wp','set_target'].includes(data.last_command.action))settingsLoaded=false}const sensors=$('sensors');sensors.replaceChildren();for(const [label,value,max]of [['GPS',data.gps_age,2],['IMU',data.heading_age,.5],['LIDAR',data.scan_age,.5],['MCU',data.telemetry_age,.5]]){const span=document.createElement('span');span.textContent=label;const b=document.createElement('b');b.textContent=fixed(value,2,'s');b.className=Number.isFinite(value)&&value<=max?'':'bad';span.append(b);sensors.append(span)}const events=$('events');events.replaceChildren();for(const event of (data.events||[]).slice(0,8)){const span=document.createElement('span');span.textContent=event.time+' '+(labels[event.state]||event.state);events.append(span)}$('cameraStale').hidden=data.camera_age===null||data.camera_age<2;drawMap(data);drawRadar(data);drawPWM(data);const d=v=>Number.isFinite(v)?(v>0?'좌 ':v<0?'우 ':'')+Math.abs(v).toFixed(1)+'°':'—';text('viewTarget',d(data.decision?.target_relative_deg));text('viewCommand',d(data.decision?.turn_command_deg));text('viewActual',d(data.decision?.actual_heading_change_deg));const connected=Boolean(data.settings_available),locked=Boolean(data.output?.settings_locked??data.output?.active);
 const canApply=connected&&!stale&&!locked&&!commandBusy;
 ['applyRoute','applyTarget','reset'].forEach(id=>$(id).disabled=!canApply);
 // Draft values can be edited while the application is running, including during navigation.
 document.querySelectorAll('.settings input,.settings select').forEach(e=>e.disabled=!connected);
 document.querySelectorAll('[data-capture]').forEach(e=>e.disabled=!canApply||!data.position||data.gps_age>2);
 $('start').disabled=!connected||stale||locked||commandBusy||!data.output?.enabled||data.stage==='COMPLETE';
 $('stop').disabled=!connected||commandBusy;
 $('export').disabled=!settingsLoaded;
 text('editState',!connected?'업데이트된 임무 연결 대기':locked?'운항/중립 확인 중 · 정지 / 설정 버튼을 누르세요':'설정 적용 가능 · 프로그램을 종료할 필요 없습니다');
 if(!lastCommand&&!commandBusy)text('notice',connected?'운항 중에도 값 입력 가능 · 정지 / 설정 → 중립 확인 → 설정 적용 → 시작 / 재개':'상태 조회 중 · 설정/제어 서비스를 기다립니다. 임무와 Windows 보기를 업데이트 후 다시 실행하세요.');
 }
async function refresh(){try{const response=await fetch('/api/state');if(!response.ok)throw Error('HTTP');render(await response.json())}catch(_){text('connection','● 연결 끊김');$('start').disabled=true}finally{setTimeout(refresh,250)}}
$('camera').onload=()=>{$('cameraEmpty').hidden=true};$('camera').onerror=()=>{$('cameraEmpty').hidden=false};setInterval(()=>{if(latest?.camera_age!==null&&latest?.camera_age<2)$('camera').src='/camera.jpg?t='+Date.now()},600);setInterval(()=>text('clock',new Date().toLocaleTimeString('ko-KR',{hour12:false})),1000);window.addEventListener('resize',()=>{if(latest){drawMap(latest);drawRadar(latest);drawPWM(latest)}});refresh();

document.querySelectorAll('button:not([data-map-control]),input,select').forEach(e=>e.disabled=true);
