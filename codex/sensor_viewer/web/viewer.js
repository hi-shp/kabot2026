'use strict';
const viewQuery=new URLSearchParams(location.search),embeddedSensors=viewQuery.get('sensors_only')==='1',sensorSection=embeddedSensors?viewQuery.get('section'):null;
const graphOnly=sensorSection==='graph',sensorOnly=sensorSection==='sensors';
const slamOnly=viewQuery.get('slam_embed')==='1';
let slamDisplay=true,slamControlPending=false;
try{slamDisplay=localStorage.getItem('isv.slam.display')!=='off';}catch(_){}
if(embeddedSensors){
 document.body.classList.add('sensors-only');if(sensorSection)document.body.classList.add('section-'+sensorSection);
 new ResizeObserver(()=>parent.postMessage({kind:'sensor-height',height:document.body.scrollHeight},location.origin)).observe(document.body);
}
if(sensorOnly){const controls=document.querySelector('.hull-controls'),details=document.createElement('details');details.className='hull-options';details.innerHTML='<summary>배 형상 설정</summary>';controls.before(details);details.append(controls);}
const $=id=>document.getElementById(id), F=(v,n=1)=>Number.isFinite(v)?v.toFixed(n):'—', clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const radians=x=>x*Math.PI/180, wrap=x=>((x%360)+360)%360;
let state=null,lastStateAt=0,connection=null,tableMode='topics',previewTopic='',selectedMapAction=null,mapMeta=null,mapPixels=null,mapCells=null,mapSeq=-1,pointBytes=0;
let route={a:null,b:null},latestGps=null,history=[],lastImuStamp=0;
let routeRevision="",routeDirty=false,routeAppliedAt=0,loadedDockingPath='',appliedDockingSelection=null,dockingApplyBusy=false;
let yawTarget=null,yawTargetAt=0;
const yawTargetPanel=document.createElement('div');yawTargetPanel.className='yaw-target';
yawTargetPanel.innerHTML='<div><span>목표 상대각 · 정면 90°</span><strong id="targetRelative">—</strong></div><div><span>목표 방위 · 북쪽 0°</span><strong id="targetBearing">—</strong></div><p id="targetDifference">목적지 대기 · 왼쪽 0° / 오른쪽 180°</p>';
document.querySelector('.imu-propulsion').before(yawTargetPanel);
function updateYawTarget(){
 const heading=northHeading(),target=performance.now()-yawTargetAt<3000?yawTarget:null;
 const bearing=target?.bearing_deg;
 const error=heading!==null&&Number.isFinite(bearing)?wrap(bearing-heading+180)-180:null;
 $('targetBearing').textContent=Number.isFinite(bearing)?F(bearing,1)+'°':'—';
 $('targetRelative').textContent=error===null?'—':F(wrap(90+error),1)+'°';
 $('targetDifference').textContent=error===null?'목적지 대기 · 왼쪽 0° / 오른쪽 180°':`${Math.abs(error)<.05?'정면':(error<0?'왼쪽 ':'오른쪽 ')+F(Math.abs(error),1)+'°'} · 목표 ${target.index+1} · ${F(target.distance_m,1)} m`;
 return error===null?null:target;
}
window.addEventListener('message',e=>{if(e.origin!==location.origin||e.source!==parent||e.data?.kind!=='yaw-target')return;yawTarget=e.data.target;yawTargetAt=performance.now();updateYawTarget();drawAttitude();});
const colors=['#a6a6a6','#d5a779','#f1dc6c'];
function fitCanvas(canvas){const rect=canvas.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2),w=Math.max(1,Math.round(rect.width*dpr)),h=Math.max(1,Math.round(rect.height*dpr));if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}return {w,h,dpr};}
function ageText(id,value){const age=value?.age_s;$(id).textContent=age==null?'대기':age<2?`${F(age,1)} s 전`:`${F(age,1)} s 지연`;$(id).className='age '+(age==null?'':age<2?'fresh':'stale');}
function vectorText(v){return v?['x','y','z'].map(k=>`${k.toUpperCase()} ${F(v[k],2)}`).join('  '):'—';}
function angleValue(data,key){const item=data?.[$('signedAngles').checked?'deg_signed':'deg_360']?.[key];return Number.isFinite(item)?`${F($('signedAngles').checked?item:wrap(Math.round(item*10)/10),1)}°`:'—';}
function cross(a,b){return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function normal(a){const n=Math.hypot(...a);return a.map(x=>x/n);}
function multiply(a,b){const o=new Float32Array(16);for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)o[c*4+r]+=a[k*4+r]*b[c*4+k];return o;}
function lookAt(eye,up){const z=normal(eye),x=normal(cross(up,z)),y=cross(z,x),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1]);}
function orthographic(l,r,b,t,n,f){return new Float32Array([2/(r-l),0,0,0,0,2/(t-b),0,0,0,0,-2/(f-n),0,-(r+l)/(r-l),-(t+b)/(t-b),-(f+n)/(f-n),1]);}
function perspective(aspect){const s=1/Math.tan(radians(55)/2),n=.1,f=1000;return new Float32Array([s/aspect,0,0,0,0,s,0,0,0,0,-(f+n)/(f-n),-1,0,0,-2*f*n/(f-n),0]);}

class CloudView{
 constructor(canvas,overlay){this.canvas=canvas;this.overlay=overlay;this.points=new Float32Array(0);this.mode='top';this.azimuth=-2.2;this.elevation=.7;this.range=20;this.dirty=false;this.drag=null;this.matrix=null;this.gl=canvas.getContext('webgl',{alpha:true,antialias:false,preserveDrawingBuffer:viewQuery.get('capture')==='1'});
  if(this.gl){try{this.initializeGL();}catch(error){console.warn('WebGL fallback:',error.message);const replacement=document.createElement('canvas');replacement.id=canvas.id;canvas.replaceWith(replacement);canvas=replacement;this.canvas=canvas;this.gl=null;this.context=canvas.getContext('2d');}}else this.context=canvas.getContext('2d');
  canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);this.drag={x:e.clientX,y:e.clientY};});
  canvas.addEventListener('pointerup',()=>this.drag=null);canvas.addEventListener('pointercancel',()=>this.drag=null);
  canvas.addEventListener('pointermove',e=>{if(this.drag){if(this.mode==='orbit'){this.azimuth+=(e.clientX-this.drag.x)*.008;this.elevation=clamp(this.elevation+(e.clientY-this.drag.y)*.006,.08,1.5);}this.drag={x:e.clientX,y:e.clientY};this.drawSoon();}else this.hover(e);});
  canvas.addEventListener('pointerleave',()=>{$('pointReadout').hidden=true;});
  canvas.addEventListener('wheel',e=>{e.preventDefault();this.range=clamp(this.range*Math.exp(e.deltaY*.001),2,Math.max(200,state?.limits?.max_range_m||0));this.drawSoon();},{passive:false});
  new ResizeObserver(()=>this.drawSoon()).observe(canvas);
 }
 initializeGL(){const gl=this.gl,vs=`attribute vec3 aPosition;attribute float aRing;uniform mat4 uMatrix;uniform float uSize;uniform float uRange;uniform float uMode;uniform float uGrid;varying vec4 vColor;vec3 hue(float h){return clamp(abs(mod(h*6.0+vec3(0.,4.,2.),6.0)-3.0)-1.0,0.0,1.0);}void main(){gl_Position=uMatrix*vec4(aPosition,1.);gl_PointSize=uSize;float h=uMode<.5?fract(max(aRing,0.)/16.):uMode<1.5?clamp(length(aPosition)/uRange,0.,1.)*.65:fract(aPosition.z*.15+.5);vColor=uGrid>.5?vec4(.25,.40,.49,.35):vec4(mix(vec3(.35),hue(h),.78),.92);}`;
  const fs=`precision highp float;uniform float uGrid;varying vec4 vColor;void main(){if(uGrid<.5&&length(gl_PointCoord-vec2(.5))>.5)discard;gl_FragColor=vColor;}`;
  const compile=(type,text)=>{const s=gl.createShader(type);gl.shaderSource(s,text);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
  const program=gl.createProgram();gl.attachShader(program,compile(gl.VERTEX_SHADER,vs));gl.attachShader(program,compile(gl.FRAGMENT_SHADER,fs));gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));this.program=program;
  this.aPosition=gl.getAttribLocation(program,'aPosition');this.aRing=gl.getAttribLocation(program,'aRing');this.uniforms=Object.fromEntries(['Matrix','Size','Range','Mode','Grid'].map(k=>[k,gl.getUniformLocation(program,'u'+k)]));this.pointBuffer=gl.createBuffer();this.gridBuffer=gl.createBuffer();gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);
 }
 update(buffer){const view=new DataView(buffer);if(buffer.byteLength<24||view.getUint32(0,true)!==0x3143504b||view.getUint32(20,true)!==16)return;const count=view.getUint32(8,true);if(count>30000||buffer.byteLength!==24+count*16)return;this.sequence=view.getUint32(4,true);this.stamp=view.getFloat64(12,true);this.points=new Float32Array(buffer,24,count*4);if(this.gl){this.gl.bindBuffer(this.gl.ARRAY_BUFFER,this.pointBuffer);this.gl.bufferData(this.gl.ARRAY_BUFFER,this.points,this.gl.DYNAMIC_DRAW);}this.drawSoon();}
 drawSoon(){if(this.dirty)return;this.dirty=true;requestAnimationFrame(()=>{this.dirty=false;this.draw();});}
 projection(w,h){const aspect=w/h;if(this.mode==='top')return multiply(orthographic(-this.range*aspect,this.range*aspect,-this.range,this.range,.01,1000),lookAt([0,0,this.range*3],[1,0,0]));const r=this.range*2.3,e=[r*Math.cos(this.azimuth)*Math.cos(this.elevation),r*Math.sin(this.azimuth)*Math.cos(this.elevation),r*Math.sin(this.elevation)];return multiply(perspective(aspect),lookAt(e,[0,0,1]));}
 project(x,y,z,w,h){const a=this.matrix,px=a[0]*x+a[4]*y+a[8]*z+a[12],py=a[1]*x+a[5]*y+a[9]*z+a[13],pw=a[3]*x+a[7]*y+a[11]*z+a[15];return pw>0?[(px/pw*.5+.5)*w,(-py/pw*.5+.5)*h]:null;}
 draw(){this.canvas.dataset.viewRangeM=String(this.range);const {w,h,dpr}=fitCanvas(this.canvas);fitCanvas(this.overlay);this.matrix=this.projection(w,h);const grid=[],step=gridStep(this.range),extentX=this.mode==='top'?this.range+step:this.range*20*Math.max(1,w/h),extentY=this.mode==='top'?this.range*w/h+step:extentX;for(let k=-Math.ceil(extentY/step)*step;k<=extentY;k+=step)grid.push(-extentX,k,0,-1,extentX,k,0,-1);for(let k=-Math.ceil(extentX/step)*step;k<=extentX;k+=step)grid.push(k,-extentY,0,-1,k,extentY,0,-1);
  if(this.gl){const gl=this.gl;gl.viewport(0,0,w,h);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);gl.uniformMatrix4fv(this.uniforms.Matrix,false,this.matrix);gl.uniform1f(this.uniforms.Size,Number($('pointSize').value)*dpr);gl.uniform1f(this.uniforms.Range,this.range);gl.uniform1f(this.uniforms.Mode,({ring:0,distance:1,height:2})[$('color').value]);
   const bind=buffer=>{gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(this.aPosition);gl.vertexAttribPointer(this.aPosition,3,gl.FLOAT,false,16,0);gl.enableVertexAttribArray(this.aRing);gl.vertexAttribPointer(this.aRing,1,gl.FLOAT,false,16,12);};
   bind(this.gridBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(grid),gl.DYNAMIC_DRAW);gl.uniform1f(this.uniforms.Grid,1);gl.drawArrays(gl.LINES,0,grid.length/4);bind(this.pointBuffer);gl.uniform1f(this.uniforms.Grid,0);gl.drawArrays(gl.POINTS,0,this.points.length/4);
  }else if(this.context){const c=this.context;c.clearRect(0,0,w,h);c.strokeStyle='#444444';c.beginPath();for(let i=0;i<grid.length;i+=8){const a=this.project(...grid.slice(i,i+3),w,h),b=this.project(...grid.slice(i+4,i+7),w,h);if(a&&b){c.moveTo(...a);c.lineTo(...b);}}c.stroke();c.fillStyle='#b5c9b0';for(let i=0;i<this.points.length;i+=4){const p=this.project(this.points[i],this.points[i+1],this.points[i+2],w,h);if(p)c.fillRect(p[0],p[1],2*dpr,2*dpr);}}
  const c=this.overlay.getContext('2d');c.clearRect(0,0,w,h);c.font=`${11*dpr}px Segoe UI`;c.lineWidth=dpr;const center=this.project(0,0,0,w,h);if(center){for(const [x,y,z,label,color] of [[Math.min(4,this.range*.4),0,0,'선수 +X','#f5bb87'],[0,Math.min(4,this.range*.4),0,'좌현 +Y','#b9aaca']]){const p=this.project(x,y,z,w,h);if(p){c.strokeStyle=color;c.fillStyle=color;c.beginPath();c.moveTo(...center);c.lineTo(...p);c.stroke();c.fillText(label,p[0]+4*dpr,p[1]-4*dpr);}}}c.fillStyle='#bbbbbb';c.fillText(`격자 ${step} m · 세로 ±${F(this.range,1)} m`,12*dpr,20*dpr);
 }
 hover(event){if(!this.matrix||!this.points.length)return;const rect=this.canvas.getBoundingClientRect(),w=this.canvas.width,h=this.canvas.height,x=(event.clientX-rect.left)*w/rect.width,y=(event.clientY-rect.top)*h/rect.height;let best=-1,dist=(13*w/rect.width)**2;for(let i=0;i<this.points.length;i+=4){const p=this.project(this.points[i],this.points[i+1],this.points[i+2],w,h);if(!p)continue;const d=(p[0]-x)**2+(p[1]-y)**2;if(d<dist){dist=d;best=i;}}$('pointReadout').hidden=best<0;if(best>=0){const [a,b,c,r]=this.points.slice(best,best+4);$('pointReadout').textContent=`X ${F(a,2)} m · Y ${F(b,2)} m · Z ${F(c,2)} m\n거리 ${F(Math.hypot(a,b,c),2)} m · 방위각 ${F(wrap(Math.atan2(b,a)*180/Math.PI),1)}°\nRing ${r>=0?r:'—'}`;}}
}
const cloud=new CloudView($('cloud'),$('cloudOverlay'));
let scanRange=20;
function zoomCloud(factor){cloud.range=clamp(cloud.range/factor,2,Math.max(200,state?.limits?.max_range_m||0));cloud.drawSoon();}
function zoomScan(factor){scanRange=clamp(scanRange/factor,2,Math.max(200,state?.values?.scan?.range_max_m||0));drawScan();}
$('cloudZoomIn').onclick=()=>zoomCloud(1.4);$('cloudZoomOut').onclick=()=>zoomCloud(1/1.4);
$('scanZoomIn').onclick=()=>zoomScan(1.4);$('scanZoomOut').onclick=()=>zoomScan(1/1.4);
$('scan').addEventListener('wheel',event=>{event.preventDefault();zoomScan(Math.exp(-clamp(event.deltaY,-200,200)*.001));},{passive:false});

$('topView').onclick=()=>{cloud.mode='top';$('topView').classList.add('active');$('orbitView').classList.remove('active');cloud.drawSoon();};
$('orbitView').onclick=()=>{cloud.mode='orbit';$('orbitView').classList.add('active');$('topView').classList.remove('active');cloud.drawSoon();};
$('resetView').onclick=()=>{cloud.range=selectedRange('cloud');cloud.azimuth=-2.2;cloud.elevation=.7;cloud.drawSoon();};
$('range').onchange=()=>{cloud.range=selectedRange('cloud');cloud.drawSoon();drawScan();};for(const id of ['color','pointSize'])$(id).oninput=()=>cloud.drawSoon();

function gridStep(range){const raw=range/5,power=10**Math.floor(Math.log10(raw)),n=raw/power;return (n<=1?1:n<=2?2:n<=5?5:10)*power;}
function selectedRange(kind){if(kind==='scan')return scanRange;const value=$('range').value;return value==='max'?Math.max(1,kind==='scan'?(state?.values?.scan?.range_max_m||100):(state?.limits?.max_range_m||100)):Number(value);}
function drawScan(){
 const canvas=$('scan'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),data=state?.values?.scan,R=selectedRange('scan'),s=Math.min(w,h)/2/R,cx=w/2,cy=h/2,step=gridStep(R),edge=Math.hypot(w/2,h/2)/s;
 canvas.dataset.scanSource=data?.topic||'';canvas.dataset.viewRangeM=String(R);c.clearRect(0,0,w,h);c.strokeStyle='#444444';c.fillStyle='#aaaaaa';c.font=`${10*dpr}px Segoe UI`;
 c.beginPath();for(let x=cx% (step*s);x<=w;x+=step*s){c.moveTo(x,0);c.lineTo(x,h);}for(let y=cy% (step*s);y<=h;y+=step*s){c.moveTo(0,y);c.lineTo(w,y);}c.stroke();
 c.strokeStyle='#44444488';for(let r=step;r<=edge;r+=step){c.beginPath();c.arc(cx,cy,r*s,0,2*Math.PI);c.stroke();if(cy-r*s>12*dpr)c.fillText(`${F(r,r<5?1:0)} m`,cx+4*dpr,cy-r*s-3*dpr);}
 c.strokeStyle='#777777';c.beginPath();c.moveTo(cx,0);c.lineTo(cx,h);c.moveTo(0,cy);c.lineTo(w,cy);c.stroke();
 c.fillText('선수 · 0°',cx+4*dpr,14*dpr);c.fillText('좌현 90°',6*dpr,cy-5*dpr);c.fillText('우현 270°',w-60*dpr,cy-5*dpr);c.fillText('선미 · 케이블',cx-28*dpr,h-8*dpr);
 c.fillStyle='#b5c9b0';for(const [angle,distance] of data?.polar_deg_m||[]){if(!Number.isFinite(distance)||distance<0||distance>(data.range_max_m||Infinity))continue;const a=radians(angle);c.fillRect(cx-Math.sin(a)*distance*s,cy-Math.cos(a)*distance*s,2*dpr,2*dpr);}
 c.fillStyle='#eeeeee';c.fillRect(cx-2*dpr,cy-2*dpr,4*dpr,4*dpr);
}
function northHeading(){const v=state?.values?.heading,h=v?.heading_deg;return v?.age_s<2&&Number.isFinite(h)?wrap(h):null;}
function drawAttitude(){
 const canvas=$('attitude'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),heading=northHeading(),r=Math.max(10*dpr,Math.min(w/2-38*dpr,h/2-25*dpr)),x=w/2,y=h/2;
 c.clearRect(0,0,w,h);canvas.dataset.headingDeg=heading===null?'':String(heading);canvas.setAttribute('aria-label',heading===null?'북쪽 기준 방위각 수신 대기':`북쪽 기준 ${F(heading,1)}도`);
 c.fillStyle=getComputedStyle(document.documentElement).getPropertyValue('--surface-panel').trim()||'#242424';c.beginPath();c.arc(x,y,r,0,2*Math.PI);c.fill();c.strokeStyle='#777777';c.lineWidth=1.5*dpr;c.stroke();
 for(let deg=0;deg<360;deg+=10){const a=radians(deg),major=deg%30===0,inner=r-(major?11:5)*dpr;c.strokeStyle=deg===0?'#d5a779':major?'#999999':'#555555';c.lineWidth=dpr;c.beginPath();c.moveTo(x+Math.sin(a)*inner,y-Math.cos(a)*inner);c.lineTo(x+Math.sin(a)*r,y-Math.cos(a)*r);c.stroke();if(deg%90!==0&&major&&r>=65*dpr){c.fillStyle='#999999';c.font=`${9*dpr}px Segoe UI`;c.textAlign='center';c.textBaseline='middle';c.fillText(String(deg),x+Math.sin(a)*(r-22*dpr),y-Math.cos(a)*(r-22*dpr));}}
 c.textAlign='center';c.textBaseline='middle';c.font=`bold ${12*dpr}px Malgun Gothic`;for(const [deg,text] of [[0,'북 N'],[90,'동 E'],[180,'남 S'],[270,'서 W']]){const a=radians(deg);c.fillStyle=deg===0?'#d5a779':'#bbbbbb';c.fillText(text,x+Math.sin(a)*(r+15*dpr),y-Math.cos(a)*(r+15*dpr));}
 if(heading!==null){c.save();c.translate(x,y);c.rotate(radians(heading));c.fillStyle='#f1dc6c';c.strokeStyle='#fff0ad';c.lineWidth=dpr;c.beginPath();c.moveTo(0,-r*.67);c.lineTo(r*.17,-r*.23);c.lineTo(0,-r*.32);c.lineTo(-r*.17,-r*.23);c.closePath();c.fill();c.stroke();c.strokeStyle='#f1dc6c99';c.beginPath();c.moveTo(0,-r*.32);c.lineTo(0,r*.23);c.stroke();c.restore();}
 const target=updateYawTarget();if(target){c.save();c.translate(x,y);c.rotate(radians(target.bearing_deg));c.strokeStyle='#48e5c2';c.fillStyle='#48e5c2';c.lineWidth=2*dpr;c.setLineDash([4*dpr,3*dpr]);c.beginPath();c.moveTo(0,0);c.lineTo(0,-r*.84);c.stroke();c.setLineDash([]);c.beginPath();c.moveTo(0,-r*.89);c.lineTo(-5*dpr,-r*.74);c.lineTo(5*dpr,-r*.74);c.closePath();c.fill();c.restore();}
 c.textAlign='left';c.textBaseline='alphabetic';
}
function drawTrend(){const canvas=$('imuTrend'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d');c.clearRect(0,0,w,h);if(history.length<2)return;const start=history.at(-1).t-30;for(let k=0;k<3;k++){c.strokeStyle=colors[k];c.lineWidth=dpr;c.beginPath();let active=false,last=null;for(const row of history){if(row.t<start)continue;const v=row.v[k];if(!Number.isFinite(v)){active=false;last=null;continue;}const x=(row.t-start)/30*w,y=h*(.5-v/360);if(!active||last!==null&&Math.abs(v-last)>180)c.moveTo(x,y);else c.lineTo(x,y);last=v;active=true;}c.stroke();}}
function drawGps(){const canvas=$('gpsTrack'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),track=latestGps?.track_enu_m||[];c.clearRect(0,0,w,h);if(!track.length){c.fillStyle='#aaaaaa';c.font=`${12*dpr}px Segoe UI`;c.fillText('유효한 GPS 위치를 기다리고 있습니다.',14*dpr,h/2);return;}const xs=track.map(p=>p[0]),ys=track.map(p=>p[1]),minX=Math.min(0,...xs),maxX=Math.max(0,...xs),minY=Math.min(0,...ys),maxY=Math.max(0,...ys),span=Math.max(10,maxX-minX,maxY-minY),s=Math.min(w*.8,h*.7)/span,cx=w/2-(minX+maxX)*s/2,cy=h/2+(minY+maxY)*s/2;c.strokeStyle='#444444';c.beginPath();c.moveTo(0,cy);c.lineTo(w,cy);c.moveTo(cx,0);c.lineTo(cx,h);c.stroke();c.strokeStyle='#b5c9b0';c.lineWidth=2*dpr;c.beginPath();track.forEach((p,i)=>i?c.lineTo(cx+p[0]*s,cy-p[1]*s):c.moveTo(cx+p[0]*s,cy-p[1]*s));c.stroke();const p=track.at(-1);c.fillStyle='#eeeeee';c.beginPath();c.arc(cx+p[0]*s,cy-p[1]*s,3*dpr,0,Math.PI*2);c.fill();c.fillStyle='#aaaaaa';c.font=`${10*dpr}px Segoe UI`;c.fillText(`동쪽 → · 북쪽 ↑ · 표시 폭 약 ${F(w/s,0)} m`,12*dpr,h-10*dpr);}

function updateSensors(){if(!state)return;if(slamOnly){renderSlamControls();if(slamDisplay)drawTank();else renderTankMission();return;}renderHull();const values=state.values||{},raw=values.vectornav,imu=raw?.orientation&&raw.age_s<2&&values.heading?.imu_heading_mode==='absolute_magnetic_raw'?raw:values.imu?.orientation&&values.imu.age_s<2?values.imu:raw?.orientation&&raw.age_s<2?raw:values.imu||raw,cloudData=values.cloud,scan=values.scan,gps=values.gps;
 $('nodeCount').textContent=state.nodes.length;$('topicCount').textContent=`${state.topics.filter(t=>t.age_s!==null&&t.age_s<2).length} / ${state.topics.length}`;$('pointCount').textContent=cloudData?`${cloudData.display_points.toLocaleString()} / ${cloudData.input_points.toLocaleString()}`:'—';const topic=state.topics.find(t=>t.name===cloudData?.topic);$('ringCount').textContent=cloudData?`${cloudData.rings.length||'—'} 층 · ${F(topic?.hz,1)} Hz`:'—';
 ageText('cloudAge',cloudData);ageText('imuAge',imu);ageText('scanAge',scan);ageText('gpsAge',gps);$('cloudEmpty').hidden=Boolean(cloudData&&cloud.points.length);$('cloudFrame').textContent=`${cloudData?.topic||'/velodyne_points'} · ${cloudData?.frame_id||'센서 좌표'} · +X 선수 / +Y 좌현 · m`;$('decodeInfo').textContent=cloudData?`변환 ${F(cloudData.decode_ms,1)} ms · ${state.limits.cloud_hz} FPS 상한`:'';
 const heading=northHeading(),freshImu=imu?.age_s<2;
 $('imuFrame').textContent=`${imu?.topic||'/vectornav/imu'} · ${values.heading?.reference==='magnetic_north'?'자북 기준':'북쪽 기준'}`;
 $('imuFrame').title=`자세: ${imu?.topic||'수신 대기'} · 방향: ${values.heading?.topic||'/navigation/heading'} · ${imu?.frame_id||''}`;
 if(freshImu&&!imu.orientation&&heading===null){$('imuAge').textContent='자세값 오류';$('imuAge').className='age stale';}
 for(const k of ['roll','pitch'])$(k).textContent=angleValue(freshImu?imu.orientation:null,k);$('yaw').textContent=heading===null?'—':F(wrap(Math.round(heading*10)/10),1)+'°';
 if(imu?.orientation&&imu.stamp_s!==lastImuStamp){lastImuStamp=imu.stamp_s;history.push({t:imu.stamp_s,v:[imu.orientation.deg_signed.roll,imu.orientation.deg_signed.pitch,heading===null?NaN:wrap(heading+180)-180]});if(history.length>400)history.shift();}
 const navigationScan=scan?.topic==='/outdoor/navigation_scan';
 $('scanAge').title=navigationScan?'주행에 사용하는 최종 장애물 스캔 · 낮은 장애물 3D 보정 포함':'원본 스캔 · 주행 장애물 스캔 수신 대기';
 $('scan').setAttribute('aria-label',navigationScan?'주행 장애물 2D 스캔':'원본 2D 스캔');
 $('scanFrame').textContent=`${scan?.frame_id||'센서 좌표'} · 선수 0° / 좌현 90° · 거리 m`;$('nearest').textContent=scan?.nearest?`최근접 ${F(scan.nearest.distance_m,2)} m / ${F(scan.nearest.azimuth_deg,1)}°`:'최근접 —';
 latestGps=gps;$('fixStatus').textContent=gps?.valid_fix?'위치 고정됨':gps?'위치 고정 대기 · fix '+gps.fix_status:'GPS 메시지 대기';const correction=values.gps_corrections;
 if(correction?.age_s<4){
  const quality=correction.gga_age_s<10?correction.fix_quality:0;
  const fix={1:'단독 측위',2:'차분 측위',4:'RTK Fixed',5:'RTK Float'}[quality]||'위치 대기';
  const receiving=correction.rtcm_age_s!==null&&correction.rtcm_age_s<5;
  $('fixStatus').textContent=fix+' · '+(receiving?'NTRIP 수신 중':'보정 수신 대기');
  $('fixStatus').title=`보정 ${correction.frames||0} 프레임 · 최근 ${correction.rtcm_age_s??'—'}초 · ${correction.state}`;
 }
 $('openMap').disabled=!gps?.valid_fix;$('latlon').textContent=gps?.valid_fix?`${F(gps.latitude_deg,7)} / ${F(gps.longitude_deg,7)}`:'—';$('altitude').textContent=gps?.valid_fix?`${F(gps.altitude_m,2)} m / ${F(values.gps_velocity?.speed_m_s,2)} m/s`:'—';$('enu').textContent=gps?.valid_fix?`${F(gps.east_m,2)} / ${F(gps.north_m,2)} m`:'—';
 const a=values.actuators,actuatorLive=!!a&&a.receiving!==false;$('actuatorMode').textContent=actuatorLive?(a.mode_name||a.mode||'대기'):'대기';const command=(side)=>{const pwm=a?.['logical_'+side+'_us']??a?.[side],percent=a?.[side+'_percent']??(Number.isFinite(pwm)?(pwm-1500)/4:null);return actuatorLive&&Number.isFinite(percent)?`${F(percent,1)}% · ${pwm} µs`:'—';};$('actuatorLeft').textContent=command('left');$('actuatorRight').textContent=command('right');$('actuatorDetail').textContent=actuatorLive?'상태 수신 중':'수신 대기';
 drawAttitude();drawTrend();drawScan();if(!sensorOnly)drawGps();if(!embeddedSensors)drawTank();if(!sensorOnly){renderGraph();renderPreview();}
 const errors=Object.entries(state.errors||{});$('notice').hidden=!errors.length;if(errors.length)$('notice').textContent=errors.map(([name,error])=>`${name}: ${error}`).join(' · ');
}
function graphRows(html){if($('graphRows').innerHTML!==html)$('graphRows').innerHTML=html;}
function renderGraph(){const query=$('search').value.toLowerCase();if(!state)return;if(tableMode==='nodes'){$('tableHead').innerHTML='<tr><th>노드</th><th>발행 토픽</th></tr>';graphRows(state.nodes.filter(n=>n.toLowerCase().includes(query)).map(n=>`<tr><td class="topic">${esc(n)}</td><td>${state.topics.filter(t=>t.publisher_nodes.includes(n)).length}</td></tr>`).join(''));return;}
 $('tableHead').innerHTML='<tr><th>토픽</th><th>형식</th><th>Pub / Sub</th><th>Hz</th><th>최근</th><th></th></tr>';graphRows(state.topics.filter(t=>(t.name+' '+t.types.join(' ')).toLowerCase().includes(query)).map(t=>`<tr class="${t.name===previewTopic?'selected':''}"><td class="topic"><span class="dot ${t.age_s===null?'':t.age_s<2?'fresh':'stale'}"></span>${esc(t.name)}</td><td class="type">${esc(t.types.join(', '))}</td><td>${t.publishers} / ${t.subscribers}</td><td>${F(t.hz,1)}</td><td>${t.age_s===null?'대기':F(t.age_s,1)+' s'}</td><td><button data-topic="${esc(t.name)}">데이터</button></td></tr>`).join(''));$('graphUpdated').textContent=`${state.topics.length}개 토픽`;}
function renderPreview(){const item=state?.preview;$('previewTitle').textContent=previewTopic||state?.selected_topic||'토픽을 선택하세요';if(item&&item.topic===(previewTopic||state.selected_topic)){$('previewUnits').textContent=$('rawPreview').checked?'원본 메시지 · 형식별 원래 단위':item.units;$('preview').textContent=JSON.stringify($('rawPreview').checked?item.raw:item.display,null,2);}else{$('preview').textContent='선택한 토픽의 메시지를 기다리고 있습니다.';}}
$('topicsTab').onclick=()=>{tableMode='topics';$('topicsTab').classList.add('active');$('nodesTab').classList.remove('active');renderGraph();};$('nodesTab').onclick=()=>{tableMode='nodes';$('nodesTab').classList.add('active');$('topicsTab').classList.remove('active');renderGraph();};$('search').oninput=renderGraph;$('rawPreview').onchange=renderPreview;$('signedAngles').onchange=updateSensors;
$('graphRows').onclick=async e=>{const button=e.target.closest('button[data-topic]');if(!button)return;previewTopic=button.dataset.topic;renderGraph();renderPreview();await post('/api/select',{topic:previewTopic});};
$('openMap').onclick=()=>{if(latestGps?.valid_fix)window.open(`https://www.google.com/maps?q=${latestGps.latitude_deg.toFixed(8)},${latestGps.longitude_deg.toFixed(8)}`,'_blank','noopener,noreferrer');};
async function post(path,data){try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}),value=await r.json();if(!r.ok)throw Error(value.error||'요청을 처리하지 못했습니다.');return value;}catch(e){$('tankStatus').textContent=e.message;return null;}}
async function mapRequest(action,extra={}){try{const r=await fetch('/api/maps'+(extra.job_id?'?job='+encodeURIComponent(extra.job_id):''),{method:action==='list'||action==='status'?'GET':'POST',headers:{'Content-Type':'application/json'},...(action==='list'||action==='status'?{}:{body:JSON.stringify({action,...extra})}),cache:'no-store'}),value=await r.json();if(!r.ok)throw Error(value.error||'지도 요청을 처리하지 못했습니다.');return value;}catch(e){$('mapJobStatus').textContent=e.message;return null;}}
async function refreshSavedMaps(){const value=await mapRequest('list');if(!value)return;const select=$('savedMaps'),current=select.value;select.replaceChildren(new Option('저장된 지도 선택',''));for(const map of value.maps||[])select.add(new Option(map.name,map.path));if([...select.options].some(option=>option.value===current))select.value=current;$('loadMap').disabled=!select.value;}
async function watchMapJob(jobId){for(let i=0;i<120;i++){await new Promise(resolve=>setTimeout(resolve,1000));const value=await mapRequest('status',{job_id:jobId});if(!value)return null;$('mapJobStatus').textContent=value.message||value.state;if(value.state==='done'){if(value.action==='load'||value.action==='load_route'){resetTankView();drawTank();}if(value.applied)appliedDockingSelection=value.applied;if(value.route){applyDockingRoute(value.route);drawTank();$('dockingRouteStatus').textContent=value.message;}await refreshSavedMaps();await refreshDockingRoutes();if(value.action==='load_route'||value.action==='erase'){$('mapJobStatus').textContent='';$('dockingRouteStatus').textContent='';}return value;}if(value.state==='error'){if(value.action==='load_route')$('dockingRouteStatus').textContent=value.message;return value;}}return null;}
for(const id of ['setA','setB'])$(id).onclick=()=>{eraserMode=false;renderMapEraser();tankCropMode=false;tankCropDrag=null;renderMapCrop();selectedMapAction=id==='setA'?'a':'b';$(id).classList.add('active');$('tankStatus').textContent=`${selectedMapAction.toUpperCase()} 선택`;};
$('savedMaps').onchange=()=>{$('loadMap').disabled=!$('savedMaps').value;};
$('createMap').onclick=async()=>{const button=$('createMap');button.disabled=true;try{const result=await mapRequest('create');if(result?.job_id){tankResetPending=true;resetTankDisplay();if(state?.values)for(const key of ['map','tank_pose','tank_route','tank_session','tank_status','tank_localization_candidates'])delete state.values[key];drawTank();$('mapJobStatus').textContent='기존 지도 초기화 · 원점에서 새 라이다 지도 시작 중';await watchMapJob(result.job_id);}}finally{tankResetPending=false;button.disabled=false;}};
$('saveMap').onclick=async()=>{const name=$('mapName').value.trim();if(!name){$('mapJobStatus').textContent='지도 이름을 입력하세요.';return;}const result=await mapRequest('save',{name,...(tankCrop?{crop:tankCrop}:{})});if(result?.job_id){$('mapJobStatus').textContent=result.message||'현재 지도를 저장 중';watchMapJob(result.job_id);}};
$('loadMap').onclick=async()=>{const path=$('savedMaps').value;if(!path)return;const result=await mapRequest('load',{path});if(result?.job_id){$('mapJobStatus').textContent=result.message||'저장 지도를 불러오는 중';watchMapJob(result.job_id);}};
$('loadDockingRoute').textContent='불러오기 및 적용';
const dockingAppliedStatus=document.createElement('span');
dockingAppliedStatus.id='dockingAppliedStatus';dockingAppliedStatus.className='muted';
dockingAppliedStatus.setAttribute('role','status');
dockingAppliedStatus.style.cssText='font-size:11px;white-space:nowrap';
$('loadDockingRoute').insertAdjacentElement('afterend',dockingAppliedStatus);
async function dockingRequest(action,extra={}){
 try{const response=await fetch('/api/docking-routes',{method:action==='list'?'GET':'POST',headers:{'Content-Type':'application/json'},...(action==='list'?{}:{body:JSON.stringify({action,...extra})}),cache:'no-store'}),value=await response.json();if(!response.ok)throw Error(value.error||'도킹 경로 요청 실패');return value;}
 catch(error){$('dockingRouteStatus').textContent=error.message;return null;}
}
async function refreshDockingRoutes(){
 const value=await dockingRequest('list');if(!value)return;
 appliedDockingSelection=value.applied||null;
 const select=$('savedDockingRoutes'),current=select.value||state?.values?.tank_route?.profile_file||'';select.replaceChildren(new Option('저장된 도킹 경로 선택',''));
 for(const item of value.routes||[])select.add(new Option(`${item.map_name} / ${item.version}`,item.path));
 if([...select.options].some(option=>option.value===current))select.value=current;
 $('loadDockingRoute').disabled=!select.value;
 routeLabel();
}
function dockingRouteKey(saved){return saved.revision||JSON.stringify([saved.session_id,saved.a,saved.b]);}
function applyDockingRoute(saved){
 route={a:saved.a,b:saved.b};routeRevision=dockingRouteKey(saved);routeAppliedAt=saved.updated_unix||0;routeDirty=false;
 loadedDockingPath=saved.profile_file||'';
 if(saved.profile_version)$('dockingVersion').value=saved.profile_version;
 if(saved.profile_file)$('savedDockingRoutes').value=saved.profile_file;
 selectedMapAction=null;for(const id of ['setA','setB'])$(id).classList.remove('active');routeLabel();
}
function syncTankRoute(session){
 const observer=state?.values?.outdoor?.docking_observation;
 const saved=observer?.active&&observer.map_id===session?observer.route:state?.values?.tank_route;if(!saved||saved.session_id!==session)return;
 const revision=dockingRouteKey(saved);if(revision===routeRevision||(saved.updated_unix||0)<routeAppliedAt)return;
 routeRevision=revision;if(!routeDirty)applyDockingRoute(saved);
}
$('savedDockingRoutes').onchange=()=>{$('loadDockingRoute').disabled=!$('savedDockingRoutes').value;routeLabel();};
$('loadDockingRoute').onclick=async()=>{
 const path=$('savedDockingRoutes').value;if(!path||dockingApplyBusy)return;
 dockingApplyBusy=true;routeLabel();
 try{const result=await dockingRequest('load',{path});if(result?.job_id){$('dockingRouteStatus').textContent='지도와 A/B 경로 불러오기 및 적용 중';await watchMapJob(result.job_id);}}
 finally{dockingApplyBusy=false;routeLabel();}
};
if(!embeddedSensors){refreshSavedMaps();refreshDockingRoutes();setInterval(refreshDockingRoutes,20000);}
const slamProcessButton=document.createElement('button');slamProcessButton.id='toggleSlamProcess';
const slamDisplayButton=document.createElement('button');slamDisplayButton.id='toggleSlamDisplay';
document.querySelector('.tank-panel .panel-head').append(slamProcessButton,slamDisplayButton);
function slamRunning(){return state?.values?.runtime?.services?.mapping==='running';}
function mapSaveAvailable(){return !!state?.values?.map&&(state.values.map.source==='checkpoint'||state?.values?.tank_session?.mode==='mapping');}
const checkpointStatus=document.createElement('span');checkpointStatus.className='muted';checkpointStatus.setAttribute('role','status');document.querySelector('.map-management').append(checkpointStatus);

function renderSlamControls(){
 renderTankMission();
 renderMapEraser();
 const service=state?.values?.runtime?.services?.mapping;
 slamProcessButton.textContent=slamControlPending?'SLAM 전환 중':service==='failed'?'SLAM 중단':service==='restarting'?'SLAM 복구 중':slamRunning()?'SLAM 켜짐':'SLAM 꺼짐';
 $('saveMap').disabled=!mapSaveAvailable();
 const checkpoint=state?.values?.map?.source==='checkpoint'?state.values.map:null;
 checkpointStatus.textContent=checkpoint?`자동 저장 지도 복원 · ${new Date(checkpoint.checkpoint_saved_unix*1000).toLocaleTimeString('ko-KR',{timeZone:'Asia/Seoul'})} 기준`:'';

 slamProcessButton.disabled=slamControlPending||!state||service==='restarting';
 slamProcessButton.setAttribute('aria-pressed',String(slamRunning()));
 slamDisplayButton.textContent=slamDisplay?'지도 표시 켜짐':'지도 표시 꺼짐';
 slamDisplayButton.setAttribute('aria-pressed',String(slamDisplay));
 slamProcessButton.title='SLAM 지도 처리만 켜거나 끕니다. 센서와 선체 데이터 기록은 계속됩니다.';
 slamDisplayButton.title='지도 수신과 화면 그리기만 중단합니다. 선체의 주행과 SLAM 처리는 유지됩니다.';
}
slamProcessButton.onclick=async()=>{
 if(slamControlPending)return;
 slamControlPending=true;renderSlamControls();
 try{const result=await mapRequest(slamRunning()?'disable':'enable');if(result?.job_id)await watchMapJob(result.job_id);}
 finally{slamControlPending=false;renderSlamControls();}
};
slamDisplayButton.onclick=()=>{
 slamDisplay=!slamDisplay;
 try{localStorage.setItem('isv.slam.display',slamDisplay?'on':'off');}catch(_){}
 renderSlamControls();if(slamDisplay)drawTank();
};

let tankView={x:null,y:null,zoom:1,auto:true},tankPointer=null,tankDragged=false;
function resetTankView(){tankView={x:null,y:null,zoom:1,auto:true};}
let priorMapId=null,expectedPrior=null,priorDraft=null,priorPick=null,priorBusy=false;
let expectedVisibilityMap=null,expectedPoseHidden=false;
let localizationRegion=null,regionMode=false,regionDrag=null,regionDraft=null,regionBusy=false,regionError='',regionHandleHover=false;
function updateExpectedPoseVisibility(){
 const values=state?.values||{},session=values.tank_session,report=values.tank_localization_candidates,o=values.outdoor;
 if(expectedVisibilityMap!==session?.session_id){expectedVisibilityMap=session?.session_id;expectedPoseHidden=false;}
 if(!o||!Number.isFinite(o.age_s)||o.age_s>=3)return;
 const phase=o.docking_phase,priority=o.priority_route||{};
 if(!o.execute||phase==='complete'||priority.complete){expectedPoseHidden=false;return;}
 const matched=report?.map_id===session?.session_id&&report?.age_s<2&&report.docking_match_ready&&
  report.docking_search_id===o.docking_search_id&&report.docking_goal_index===o.goal;
 if(phase==='docking'||['neutralizing','docking'].includes(priority.state)||(phase==='searching'&&matched))expectedPoseHidden=true;
}
const priorControls=document.createElement('div');priorControls.className='map-management expected-pose-management';
priorControls.innerHTML='<strong>공통 예상 자세</strong><button id="pickExpectedPose" disabled>위치·방향 지정</button><label>지도 각도 <input id="expectedYaw" type="number" value="0" step="1" aria-label="예상 선수 지도 각도">°</label><label>거리 가중 <input id="expectedRadius" type="number" value="3" min="0.1" step="0.5" aria-label="위치 가중 기준 거리">m</label><label>각도 가중 <input id="expectedYawWidth" type="number" value="20" min="1" max="180" step="1" aria-label="방향 가중 기준 각도">°</label><button id="saveExpectedPose" disabled>예상 자세 저장</button><button id="clearExpectedPose" disabled>해제</button><span id="expectedPoseStatus" class="muted" role="status">left / middle / right 공통</span>';
document.querySelector('.docking-management').after(priorControls);
$('expectedYaw').title='지도 오른쪽 0°, 위쪽 90° · 선수 방향';
$('expectedRadius').title='가까울수록 강하게 우선합니다. 이 거리 밖의 후보도 탐색합니다.';
$('expectedYawWidth').title='예상 방향과 비슷할수록 강하게 우선합니다. 다른 방향도 탐색합니다.';
function expectedInputs(prior){for(const [id,key] of [['expectedYaw','yaw_deg'],['expectedRadius','position_radius_m'],['expectedYawWidth','yaw_tolerance_deg']])$(id).value=String(prior?.[key]??(key==='position_radius_m'?3:key==='yaw_tolerance_deg'?20:0));}
function expectedValues(){const value={yaw_deg:Number($('expectedYaw').value),position_radius_m:Number($('expectedRadius').value),yaw_tolerance_deg:Number($('expectedYawWidth').value)};if(!Object.values(value).every(Number.isFinite)||value.position_radius_m<=0||value.yaw_tolerance_deg<=0||value.yaw_tolerance_deg>180)throw Error('거리 가중은 양수, 각도 가중은 1~180°로 입력하세요.');return value;}
function renderExpectedPose(){
 const point=priorDraft||expectedPrior;
 $('pickExpectedPose').disabled=!priorMapId||priorBusy;$('pickExpectedPose').classList.toggle('active',!!priorPick);
 $('pickExpectedPose').textContent=priorPick==='position'?'예상 위치 선택':priorPick==='heading'?'선수 방향 선택':'위치·방향 지정';
 $('saveExpectedPose').disabled=!priorDraft||priorBusy||!!priorPick;$('clearExpectedPose').disabled=(!point&&!priorPick)||priorBusy;
 $('tankMap').classList.toggle('expected-pose-picking',!!priorPick);
 $('expectedPoseStatus').textContent=priorPick==='position'?'지도에서 위치 선택':priorPick==='heading'?'선수 쪽을 한 번 더 선택':point?`공통 · ${F(point.x,1)}, ${F(point.y,1)}m · ${F(point.yaw_deg,1)}°${priorDraft?' · 저장 전':' · 저장됨'}`:'left / middle / right 공통';
}
async function loadExpectedPose(mapId){
 try{const response=await fetch('/api/localization-prior?map_id='+encodeURIComponent(mapId),{cache:'no-store'}),body=await response.json();if(!response.ok)throw Error(body.error||'예상 자세 수신 실패');if(priorMapId!==mapId)return;if(!priorDraft){expectedPrior=body.prior;expectedInputs(expectedPrior);}const cached=queuedRegion(mapId);localizationRegion=cached?.pending?cached.region:body.region||null;if(!cached?.pending)writeRegionCache(mapId,{region:localizationRegion,pending:false});renderExpectedPose();renderLocalizationRegion();drawTank();flushLocalizationRegion();}
 catch(error){if(priorMapId===mapId)$('expectedPoseStatus').textContent=error.message;}
}
function syncExpectedPose(){const mapId=state?.values?.tank_session?.session_id||null;if(!mapId||mapId===priorMapId)return;priorMapId=mapId;expectedPrior=null;priorDraft=null;priorPick=null;localizationRegion=queuedRegion(mapId)?.region||null;regionDraft=null;regionMode=false;regionDrag=null;regionError='';expectedInputs(null);renderExpectedPose();renderLocalizationRegion();loadExpectedPose(mapId);}
function cancelExpectedPick(){priorPick=null;renderExpectedPose();}
$('pickExpectedPose').onclick=()=>{cancelRegionPick();if(priorPick){cancelExpectedPick();return;}eraserMode=false;renderMapEraser();tankCropMode=false;tankCropDrag=null;renderMapCrop();selectedMapAction=null;for(const id of ['setA','setB'])$(id).classList.remove('active');priorPick='position';tankDragged=false;renderExpectedPose();};
for(const id of ['setA','setB','cropMap'])$(id).addEventListener('click',cancelExpectedPick);
for(const id of ['expectedYaw','expectedRadius','expectedYawWidth'])$(id).addEventListener('input',()=>{if(!priorDraft&&!expectedPrior)return;try{priorDraft={...(priorDraft||expectedPrior),...expectedValues()};renderExpectedPose();drawTank();}catch(error){$('expectedPoseStatus').textContent=error.message;}});
async function saveExpectedPose(action){
 if(!priorMapId||priorBusy)return;const mapId=priorMapId;let prior;
 try{prior=action==='save'?{...priorDraft,...expectedValues()}:null;}catch(error){$('expectedPoseStatus').textContent=error.message;return;}
 priorBusy=true;let failure=null;renderExpectedPose();
 try{const response=await fetch('/api/localization-prior',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,map_id:mapId,prior})}),body=await response.json();if(!response.ok)throw Error(body.error||'예상 자세 저장 실패');if(priorMapId!==mapId)return;expectedPrior=body.prior;priorDraft=null;priorPick=null;expectedInputs(expectedPrior);}
 catch(error){failure=error.message;}
 finally{priorBusy=false;renderExpectedPose();if(failure)$('expectedPoseStatus').textContent=failure;drawTank();}
}
$('saveExpectedPose').onclick=()=>saveExpectedPose('save');$('clearExpectedPose').onclick=()=>saveExpectedPose('clear');
function expectedPoseClick(event){
 if(!priorPick||!tankTransform)return false;const [x,y]=tankWorld(event);
 if(priorPick==='position'){try{priorDraft={x,y,...expectedValues()};priorPick='heading';}catch(error){$('expectedPoseStatus').textContent=error.message;return true;}}
 else{const dx=x-priorDraft.x,dy=y-priorDraft.y;if(Math.hypot(dx,dy)<.01)return true;priorDraft.yaw_deg=wrap(Math.atan2(dy,dx)*180/Math.PI);expectedInputs(priorDraft);priorPick=null;}
 renderExpectedPose();drawTank();return true;
}
function drawExpectedPose(c,screen,s,dpr){
 const point=priorDraft||expectedPrior;if(!point||expectedPoseHidden)return;const [x,y]=screen(point.x,point.y);
 c.save();c.strokeStyle='#e87914';c.lineWidth=.75*dpr;c.globalAlpha=.45;c.setLineDash([4*dpr,5*dpr]);c.beginPath();c.arc(x,y,point.position_radius_m*s,0,2*Math.PI);c.stroke();c.setLineDash([]);c.globalAlpha=1;
 c.translate(x,y);c.rotate(-radians(point.yaw_deg));c.fillStyle='#e87914';c.beginPath();c.moveTo(12*dpr,0);c.lineTo(-7*dpr,-5*dpr);c.lineTo(-4*dpr,0);c.lineTo(-7*dpr,5*dpr);c.closePath();c.fill();c.restore();
 c.save();c.font=`${10*dpr}px Segoe UI`;c.fillStyle='#e87914';c.fillText('예상',x+9*dpr,y-9*dpr);c.restore();
}
let tankCrop=null,tankCropMode=false,tankCropDrag=null;
let eraserMode=false,eraserBusy=false,eraserPointer=null,eraserStrokes=[],eraserMapId=null,eraserResult='';
const eraserTools=document.createElement('span');eraserTools.className='map-eraser-tools';
eraserTools.innerHTML='<button id="toggleMapEraser" aria-pressed="false">지우개 꺼짐</button><label>크기 <input id="mapEraserSize" type="number" value="0.6" min="0.05" step="0.1" aria-label="지우개 지름 미터">m</label><button id="saveMapErasure" disabled>지운 지도 저장</button><button id="cancelMapErasure" disabled>취소</button>';
document.querySelector('.tank-toolbar .tools').append(eraserTools);

const regionControls=document.createElement('div');regionControls.className='map-management';
regionControls.innerHTML='<strong>공통 정합 구역</strong><button id="pickLocalizationRegion">구역 지정</button><button id="toggleLocalizationRegion" aria-pressed="false">구역 제한 꺼짐</button><button id="clearLocalizationRegion">해제</button><span id="localizationRegionStatus" class="muted" role="status"></span>';
priorControls.after(regionControls);
function regionVertices(value){if(value?.vertices)return value.vertices.map(p=>[...p]);if(!value?.bounds)return null;const [x0,y0,x1,y1]=value.bounds;return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]];}
function rectangleVertices(a,b){const x0=Math.min(a[0],b[0]),y0=Math.min(a[1],b[1]),x1=Math.max(a[0],b[0]),y1=Math.max(a[1],b[1]);return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]];}
function regionValue(vertices,enabled){
 const turns=vertices.map((a,i)=>{const b=vertices[(i+1)%4],c=vertices[(i+2)%4];return (b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);});
 if(!turns.every(v=>v>1e-8)&&!turns.every(v=>v< -1e-8))throw Error('꼭짓점이 서로 교차하지 않도록 이동하세요.');
 return {vertices:vertices.map(p=>[...p]),bounds:[Math.min(...vertices.map(p=>p[0])),Math.min(...vertices.map(p=>p[1])),Math.max(...vertices.map(p=>p[0])),Math.max(...vertices.map(p=>p[1]))],enabled};
}
function readRegionCache(mapId){try{return JSON.parse(localStorage.getItem('isv.slam.region.v2.'+mapId)||'null');}catch(_){return null;}}
function writeRegionCache(mapId,value){try{localStorage.setItem('isv.slam.region.v2.'+mapId,JSON.stringify(value));}catch(_){regionError='브라우저 임시 저장 실패 · 젯슨 저장 확인 필요';}return value;}
let pendingRegionMemory=new Map();
function queuedRegion(mapId){return pendingRegionMemory.get(mapId)||readRegionCache(mapId);}
function regionHandleAt(event){
 if(!tankTransform||regionMode||regionBusy||priorPick||eraserMode||tankCropMode)return -1;
 const vertices=regionVertices(localizationRegion);if(!vertices)return -1;
 const rect=$('tankMap').getBoundingClientRect(),{s,cx,cy,dpr}=tankTransform;
 return vertices.findIndex(([x,y])=>Math.hypot((cx+x*s)/dpr-(event.clientX-rect.left),(cy-y*s)/dpr-(event.clientY-rect.top))<=12);
}
function renderLocalizationRegion(){
 const active=!!localizationRegion?.enabled,ready=!!priorMapId;
 $('pickLocalizationRegion').disabled=!ready;$('pickLocalizationRegion').classList.toggle('active',regionMode);
 $('pickLocalizationRegion').textContent=regionMode?'구역 선택 중':'구역 지정';
 $('toggleLocalizationRegion').disabled=!ready||!localizationRegion;
 $('toggleLocalizationRegion').textContent=regionBusy?'구역 저장 중':active?'구역 제한 켜짐':'구역 제한 꺼짐';
 $('toggleLocalizationRegion').setAttribute('aria-pressed',String(active));
 $('toggleLocalizationRegion').style.cssText=active?'background:#164f3a;border-color:#65d39c;color:#e1ffed':'background:#4c2929;border-color:#bb7474;color:#ffe2de';
 $('clearLocalizationRegion').disabled=!ready||!localizationRegion;
 const vertices=regionDraft||regionVertices(localizationRegion),pending=!!queuedRegion(priorMapId)?.pending;
 $('localizationRegionStatus').textContent=regionError|| (regionMode?'드래그로 생성 · 꼭짓점 4개 이동 가능':vertices?`left / middle / right 공통 · ${regionBusy?'저장 중':pending?'임시 저장 · 자동 재연결 저장':'자동 저장됨'}`:'미지정 · 전체 탐색');
 $('tankMap').classList.toggle('localization-region-picking',regionMode);
 updateRegionCursor();
}
function updateRegionCursor(){
 $('tankMap').style.cursor=regionHandleHover||regionDrag?.corner!==undefined?'default':regionMode?'crosshair':'';
}
$('tankMap').addEventListener('pointermove',event=>{regionHandleHover=regionDrag?.corner!==undefined||regionHandleAt(event)>=0;updateRegionCursor();});
$('tankMap').addEventListener('pointerdown',event=>{if(regionHandleAt(event)>=0){regionHandleHover=true;updateRegionCursor();}});
$('tankMap').addEventListener('pointerleave',()=>{regionHandleHover=false;updateRegionCursor();});
function cancelRegionPick(){regionMode=false;regionDrag=null;regionDraft=null;renderLocalizationRegion();drawTank();}
function saveLocalizationRegion(region){
 if(!priorMapId)return;
 try{if(region)region=regionValue(regionVertices(region),region.enabled);}catch(error){regionError=error.message;regionDraft=null;renderLocalizationRegion();drawTank();return;}
 const entry={region,pending:true,id:Date.now()+':'+Math.random()};
 pendingRegionMemory.set(priorMapId,entry);writeRegionCache(priorMapId,entry);
 localizationRegion=region;regionDraft=null;regionError='';renderLocalizationRegion();drawTank();flushLocalizationRegion();
}
async function flushLocalizationRegion(){
 if(!priorMapId||regionBusy)return;const mapId=priorMapId,entry=queuedRegion(mapId);
 if(!entry?.pending)return;
 regionBusy=true;regionError='';renderLocalizationRegion();
 try{
  const response=await fetch('/api/localization-prior',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:entry.region?'save_region':'clear_region',map_id:mapId,region:entry.region}),signal:AbortSignal.timeout(8000)});
  const body=await response.json();if(!response.ok)throw Error(body.error||'정합 구역 저장 실패');
  if(queuedRegion(mapId)?.id===entry.id){pendingRegionMemory.delete(mapId);writeRegionCache(mapId,{region:body.region||null,pending:false});if(priorMapId===mapId)localizationRegion=body.region||null;}
 }catch(error){if(priorMapId===mapId)regionError=error.message;}
 finally{regionBusy=false;renderLocalizationRegion();drawTank();}
}
setInterval(flushLocalizationRegion,2000);
$('pickLocalizationRegion').onclick=()=>{
 if(regionMode){cancelRegionPick();return;}
 cancelExpectedPick();eraserMode=false;renderMapEraser();tankCropMode=false;tankCropDrag=null;renderMapCrop();
 selectedMapAction=null;for(const id of ['setA','setB'])$(id).classList.remove('active');
 regionMode=true;regionDraft=null;regionError='';tankDragged=false;renderLocalizationRegion();drawTank();
};
$('toggleLocalizationRegion').onclick=()=>saveLocalizationRegion({...localizationRegion,enabled:!localizationRegion.enabled});
$('clearLocalizationRegion').onclick=()=>{cancelRegionPick();saveLocalizationRegion(null);};
for(const id of ['setA','setB','cropMap','toggleMapEraser'])$(id)?.addEventListener('click',cancelRegionPick);
function drawLocalizationRegion(c,screen,dpr){
 const vertices=regionDraft||regionVertices(localizationRegion);if(!vertices)return;
 const points=vertices.map(p=>screen(...p)),active=!!regionDraft||localizationRegion.enabled;
 c.save();c.strokeStyle=active?'#47da91':'#a8afb3';c.fillStyle=active?'#27c57418':'#a8afb308';c.lineWidth=2*dpr;
 c.setLineDash(active?[]:[6*dpr,4*dpr]);c.beginPath();points.forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.closePath();c.fill();c.stroke();c.setLineDash([]);
 points.forEach(([x,y],i)=>{c.fillStyle='#163a2c';c.fillRect(x-5*dpr,y-5*dpr,10*dpr,10*dpr);c.strokeRect(x-5*dpr,y-5*dpr,10*dpr,10*dpr);c.fillStyle='#effff3';c.font=`${10*dpr}px Segoe UI`;c.fillText(String(i+1),x+7*dpr,y-7*dpr);});
 const a=points[0];c.font=`${11*dpr}px Segoe UI`;c.fillStyle=c.strokeStyle;c.fillText(active?'공통 정합 구역':'정합 구역 · 제한 꺼짐',a[0]+5*dpr,a[1]+20*dpr);c.restore();
}

function renderMapEraser(){
 const session=state?.values?.tank_session,id=session?.session_id;
 if(!eraserBusy&&eraserMapId&&id&&eraserMapId!==id){eraserStrokes=[];eraserPointer=null;eraserMode=false;eraserMapId=null;}
 const available=!!(session?.fixed_map&&session.map_file&&mapMeta);
 $('toggleMapEraser').disabled=!available||eraserBusy;
 $('toggleMapEraser').textContent=eraserMode?'지우개 켜짐':'지우개 꺼짐';
 $('toggleMapEraser').setAttribute('aria-pressed',String(eraserMode));
 $('tankMap').classList.toggle('map-erasing',eraserMode);
 $('saveMapErasure').disabled=eraserBusy||!eraserStrokes.length;
 $('saveMapErasure').textContent=eraserBusy?'지도 저장 중':eraserResult||'지운 지도 저장';
 $('saveMapErasure').title=eraserResult;
 $('cancelMapErasure').disabled=eraserBusy||(!eraserMode&&!eraserStrokes.length);
 $('mapEraserSize').disabled=eraserBusy;
}
$('toggleMapEraser').onclick=()=>{eraserMode=!eraserMode;eraserPointer=null;cancelExpectedPick();tankCropMode=false;tankCropDrag=null;renderMapCrop();selectedMapAction=null;for(const id of ['setA','setB'])$(id).classList.remove('active');renderMapEraser();};
$('cancelMapErasure').onclick=()=>{eraserStrokes=[];eraserPointer=null;eraserMode=false;eraserMapId=null;renderMapEraser();drawTank();};
function erasePointer(event,start=false){
 if(!tankTransform||eraserBusy)return;const point=tankWorld(event),size=Number($('mapEraserSize').value);
 if(!Number.isFinite(size)||size<=0)return;
 if(start){eraserResult='';eraserMapId=state?.values?.tank_session?.session_id;eraserPointer={id:event.pointerId,point,radius:size/2};}
 if(!eraserPointer||eraserPointer.id!==event.pointerId)return;
 const from=eraserPointer.point;if(!start&&Math.hypot(point[0]-from[0],point[1]-from[1])<eraserPointer.radius*.15)return;
 eraserStrokes.push([...from,...point,eraserPointer.radius].map(v=>Number(v.toFixed(4))));eraserPointer.point=point;tankDragged=true;renderMapEraser();drawTank();
}
function drawMapErasure(c,screen,s){
 if(!eraserStrokes.length)return;c.save();c.strokeStyle='#bbcbcf';c.fillStyle='#bbcbcf';c.lineCap='round';
 for(const [x0,y0,x1,y1,r] of eraserStrokes){c.lineWidth=2*r*s;c.beginPath();c.moveTo(...screen(x0,y0));c.lineTo(...screen(x1,y1));c.stroke();c.beginPath();c.arc(...screen(x1,y1),r*s,0,2*Math.PI);c.fill();}c.restore();
}
$('saveMapErasure').onclick=async()=>{
 if(eraserBusy||!eraserStrokes.length)return;const session=state?.values?.tank_session;
 if(!session?.map_file||session.session_id!==eraserMapId){eraserResult='지도 변경됨 · 다시 불러오세요';renderMapEraser();return;}
 eraserBusy=true;eraserMode=false;eraserPointer=null;renderMapEraser();
 try{const result=await mapRequest('erase',{path:session.map_file,map_id:eraserMapId,strokes:eraserStrokes});
  if(!result?.job_id)throw Error($('mapJobStatus').textContent||'저장 요청 실패 · 다시 시도');
  const done=await watchMapJob(result.job_id);
  if(done?.state!=='done')throw Error(done?.message||$('mapJobStatus').textContent||'저장 확인 실패 · 다시 시도');
  eraserStrokes=[];eraserMapId=null;
  eraserResult=done.erased_cells?'저장됨 · '+done.erased_cells+'칸':'저장됨 · 지울 장애물 없음';
  // Fetch the saved pixels even when the replacement has the same dimensions.
  mapSeq=-1;mapStream=null;
 }catch(error){eraserResult='저장 실패 · '+error.message;$('mapJobStatus').textContent=eraserResult;
 }finally{eraserBusy=false;renderMapEraser();drawTank();}
};
function cancelTankDrag(){
 if(tankPointer&&$('tankMap').hasPointerCapture(tankPointer.id))$('tankMap').releasePointerCapture(tankPointer.id);
 tankPointer=null;tankDragged=false;$('tankMap').classList.remove('dragging');
}
window.addEventListener('blur',cancelTankDrag);
let trailWasProvisional=false;
let tankTransform=null,tankTrail=[],lastTankStamp=null,tankSession=null,mapStream=null,serverInstance=null;
let tankResetId=null,tankResetPending=false;
function resetTankDisplay(){
 tankCrop=null;tankCropDrag=null;tankCropMode=false;renderMapCrop();
 mapSeq=-1;mapStream=null;mapMeta=null;mapPixels=null;mapCells=null;
 tankTransform=null;tankTrail=[];lastTankStamp=null;tankSession=null;trailWasProvisional=false;
 resetTankView();route={a:null,b:null};routeRevision='';routeDirty=false;routeAppliedAt=0;selectedMapAction=null;
 selectedCandidateId=null;candidateMapId=null;
 for(const id of ['setA','setB'])$(id).classList.remove('active');
 routeLabel();$('mapInfo').textContent='새 지도 · 원점 (0, 0) · 라이다 수신 대기';
}
function acceptState(next){
 const instance=next.instance_id||null;
 if(instance&&instance!==serverInstance){mapSeq=-1;mapStream=null;mapMeta=null;mapPixels=null;mapCells=null;cloud.sequence=-1;lastTankStamp=null;}
 const reset=next.values?.tank_map_reset?.id||null;
 if(reset&&reset!==tankResetId){tankResetId=reset;tankResetPending=false;resetTankDisplay();}
 if(tankResetPending&&next.values)for(const key of ['map','tank_pose','tank_route','tank_session','tank_status','tank_localization_candidates'])delete next.values[key];
 serverInstance=instance;state=withFastSensors(next);syncExpectedPose();updateExpectedPoseVisibility();
}

let selectedCandidateId=null,candidateMapId=null;
function liveTankCandidates(){
 const report=state?.values?.tank_localization_candidates,session=state?.values?.tank_session;
 if(!report||report.manual_locked||report.map_id!==session?.session_id||report.age_s>=1)return [];
 const now=state.server_time_s||report.stamp_s;
 return (report.live_candidates||[]).filter(p=>p.candidate_id&&[p.x,p.y,p.yaw_deg,p.score,p.stamp_s].every(Number.isFinite)&&now-p.stamp_s>=-.1&&now-p.stamp_s<1).slice(0,5);
}
function candidateControls(){
 const locked=!!state?.values?.tank_localization_candidates?.manual_locked;document.querySelector('.candidate-controls').hidden=locked;$('candidateInfo').hidden=locked;
 const mapId=state?.values?.tank_session?.session_id,candidates=liveTankCandidates();
 if(mapId!==candidateMapId){selectedCandidateId=null;candidateMapId=mapId;}
 if(selectedCandidateId&&!candidates.some(p=>p.candidate_id===selectedCandidateId))selectedCandidateId=null;
 const box=$('candidateButtons'),key=candidates.map(p=>p.candidate_id).join(',');
 if(box.dataset.key!==key){box.dataset.key=key;box.replaceChildren(...candidates.map((p,i)=>{const b=document.createElement('button');b.dataset.candidateId=p.candidate_id;b.textContent='후보 '+(i+1);return b;}));}
 for(const b of box.children){const active=b.dataset.candidateId===selectedCandidateId;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));}
 const selected=candidates.find(p=>p.candidate_id===selectedCandidateId),trial=state?.values?.tank_status;
 const mcu=state?.values?.actuators,rc=mcu?.receiving&&mcu.mode===1&&mcu.age_s<.5;
 $('applyCandidate').disabled=!selected||!!(trial?.execute&&trial.age_s<3&&!rc);
 $('candidateSelection').textContent=selected?`${F(selected.x,2)}, ${F(selected.y,2)} m · ${F(wrap(selected.yaw_deg),1)}° · 점수 ${F(selected.score,2)}`:'후보를 선택하면 해당 화살표가 강조됩니다.';
}
function tankArrow(c,point,screen,dpr,color,size=1,alpha=1){
 c.save();c.globalAlpha=alpha;c.translate(...screen(point.x,point.y));c.rotate(-radians(point.yaw_deg));c.scale(size,size);
 c.fillStyle=color;c.strokeStyle='#222222';c.lineWidth=1.5*dpr;c.beginPath();c.moveTo(13*dpr,0);c.lineTo(-8*dpr,-7*dpr);c.lineTo(-5*dpr,0);c.lineTo(-8*dpr,7*dpr);c.closePath();c.fill();c.stroke();c.restore();
}
function tankDisplayPose(){
 const values=state?.values||{},session=values.tank_session,actual=values.tank_pose,report=values.tank_localization_candidates;
 const gpsDocking=values.outdoor?.gps_docking;
 const observing=values.outdoor?.docking_observation?.active;
 if(!observing&&gpsDocking?.active&&gpsDocking.map_id===session?.session_id){
  const p=gpsDocking.display_pose;
  if(values.outdoor.age_s>=2||!p||![p.x,p.y,p.yaw_deg,p.stamp_s].every(Number.isFinite))return null;
  return {position_m:{x:p.x,y:p.y},orientation:{deg_signed:{yaw:p.yaw_deg}},
   age_s:Math.max(0,(state.server_time_s||p.stamp_s)-p.stamp_s),stamp_s:p.stamp_s,
   held:!!p.held,position_source:'gps',heading_source:'imu',gps_docking:true};
 }
 if(!observing&&(!session?.fixed_map||session.localization_ready===true)&&actual?.age_s<1)return actual;
 const p=report?.display_pose;
 if(!p||report.map_id!==session?.session_id||report.age_s>=1||!Number.isFinite(p.stamp_s))return null;
 const age=(state.server_time_s||report.stamp_s)-p.stamp_s;
 if(age<-.1||(!p.held&&age>=1)||![p.x,p.y,p.yaw_deg].every(Number.isFinite))return null;
 return {position_m:{x:p.x,y:p.y},orientation:{deg_signed:{yaw:p.yaw_deg}},age_s:age,stamp_s:p.stamp_s,provisional:true,held:!!p.held,manual_locked:!!p.manual_locked,display_fallback:!!p.display_fallback};
}
function tankMissionSummary(data){
 const empty={fresh:false,label:'도킹 웨이포인트 대기',tone:'waiting',target:null,lastReached:null,total:null,distance:null,pointStates:{}};
 if(!data||!Number.isFinite(data.age_s)||data.age_s>=2)return empty;
 const target=data.goal==null?null:String(data.goal).toUpperCase(),total=Math.max(0,Math.floor(Number(data.legs)||0));
 const count=data.mission_waypoints_m?.length||2,arrived=data.state==='ARRIVED',following=data.execute===true&&data.state==='RUNNING';
 let lastReached=arrived?target:null;
 if(!lastReached&&total>0&&count===2&&['A','B'].includes(target))lastReached=target==='A'?'B':'A';
 const pointStates={};
 if(count===2&&total>=2){pointStates.A='도달';pointStates.B='도달';}
 if(lastReached)pointStates[lastReached]='도달';
 if(target)pointStates[target]=arrived?'도달':following?'추종 중':'다음 목표';
 let label='주행 대기',tone='waiting';
 if(arrived){label=`${target||'목표'} 도달 · 대기`;tone='arrived';}
 else if(data.state==='DWELL'){
  label=lastReached?`${lastReached} 도달 · ${target||'다음 목표'} 출발 준비`:`${target||'목표'} 출발 준비`;
  tone=lastReached?'arrived':'waiting';
 }else if(data.state==='RC'){label='수동 조종';tone='manual';}
 else if(following){label=`${target||'목표'} 지점 추종 중`;tone='following';}
 else if(data.state==='BLOCKED'){label=`${target||'목표'} 추종 대기 · 회피/재탐색`;}
 else if(data.state==='PREVIEW'||data.execute===false){label=`미리보기 · 목표 ${target||'—'}`;tone='preview';}
 const p=data.map_position_m,g=data.goal_point_m;
 const distance=Array.isArray(p)&&Array.isArray(g)&&p.length>=2&&g.length>=2&&[p[0],p[1],g[0],g[1]].every(Number.isFinite)?Math.hypot(p[0]-g[0],p[1]-g[1]):null;
 return {fresh:true,label,tone,target,lastReached,total,distance,pointStates,reason:data.reason||''};
}
const tankMissionPanel=document.createElement('div');tankMissionPanel.className='tank-mission';tankMissionPanel.dataset.state='stale';
tankMissionPanel.innerHTML='<strong id="tankMissionState" role="status" aria-live="polite">도킹 웨이포인트 대기</strong><span>현재 목표 <b id="tankMissionTarget">—</b></span><span>남은 거리 <b id="tankMissionDistance">—</b></span><span>최근 도달 <b id="tankMissionArrival">—</b></span><span>도달 횟수 <b id="tankMissionCount">—</b></span>';
document.querySelector('.tank-panel .panel-head').after(tankMissionPanel);
const abortDockingButton=document.createElement('button');
abortDockingButton.id='abortDocking';abortDockingButton.textContent='도킹 중단';
abortDockingButton.title='현재 도킹을 종료하고 다음 GPS 웨이포인트로 진행';
abortDockingButton.style.cssText='color:#ffdad5;border-color:#e56b61;background:#642c29';
tankMissionPanel.append(abortDockingButton);
const abortOrbitButton=document.createElement('button');
abortOrbitButton.id='abortOrbit';abortOrbitButton.textContent='선회 중단';
abortOrbitButton.title='현재 선회를 종료하고 다음 GPS 웨이포인트로 진행';
abortOrbitButton.style.cssText=abortDockingButton.style.cssText;
abortOrbitButton.disabled=true;abortDockingButton.after(abortOrbitButton);
let orbitAbortPending=null,orbitAbortBusy=false,orbitAbortError='';
abortOrbitButton.onclick=async()=>{
 const o=state?.values?.outdoor;
 if(orbitAbortBusy||orbitAbortPending||!o?.execute||!o?.buoy_orbit||o.age_s>=2)return;
 orbitAbortBusy=true;orbitAbortError='';renderTankMission();
 try{
  const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({action:'abort_orbit',goal_index:o.goal,docking_search_id:o.docking_search_id}),signal:AbortSignal.timeout(4000)});
  const value=await response.json();if(!response.ok)throw Error(value.error||'선회 중단 요청 실패');
  orbitAbortPending={id:value.request_id,at:performance.now()};
 }catch(error){orbitAbortError=error.message;}
 finally{orbitAbortBusy=false;renderTankMission();}
};
let dockingAbortPending=null,dockingAbortBusy=false,dockingAbortError='';
abortDockingButton.onclick=async()=>{
 if(dockingAbortBusy||dockingAbortPending)return;
 const o=state?.values?.outdoor;if(!o)return;
 dockingAbortBusy=true;dockingAbortError='';renderTankMission();
 try{
  const response=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({action:'abort_docking',goal_index:o.goal,docking_search_id:o.docking_search_id}),signal:AbortSignal.timeout(4000)});
  const value=await response.json();if(!response.ok)throw Error(value.error||'도킹 중단 요청 실패');
  dockingAbortPending={id:value.request_id,at:performance.now()};
 }catch(error){dockingAbortError=error.message;}
 finally{dockingAbortBusy=false;renderTankMission();}
};
function dockingMissionSummary(tank,outdoor){
 const summary=tankMissionSummary(tank),fresh=outdoor&&Number.isFinite(outdoor.age_s)&&outdoor.age_s<2;
 const phase=fresh?outdoor.docking_phase:'waiting';
 const labels={waiting:'도킹 웨이포인트 대기',searching:'탐색 중',docking:'도킹 중',complete:'도킹 완료',aborting:'도킹 중단 중',aborted:'도킹 중단 · 다음 지점 진행'};
 summary.label=phase==='docking'&&outdoor?.gps_docking?.active?'GPS 도킹 중':labels[phase]||labels.waiting;
 summary.fresh=!!fresh;
 summary.tone=phase==='complete'?'arrived':phase==='docking'||phase==='searching'?'following':'waiting';
 if(phase==='waiting'||phase==='searching')Object.assign(summary,{target:null,lastReached:null,total:null,distance:null,pointStates:{}});
 if(phase==='complete')Object.assign(summary,{target:'B',lastReached:'B',total:Math.max(2,summary.total||0),distance:0,pointStates:{A:'도달',B:'도달'}});
 const observer=outdoor?.docking_observation;
 if(fresh&&observer?.active&&!['aborting','aborted','complete'].includes(phase)){
  const returning=observer.phase==='B';summary.label=`${observer.version} · GPS ${returning?'B 후진':'A 접근'} · SLAM 관측`;
  Object.assign(summary,{target:returning?'B':'A',total:returning?1:0,lastReached:returning?'A':null,
   distance:Number.isFinite(outdoor.point_follow?.distance_m)?outdoor.point_follow.distance_m:null,
   pointStates:returning?{A:'도달',B:'추종 중'}:{A:'추종 중'},tone:'following'});
 }
 return summary;
}
function renderTankMission(){
 const current=performance.now()-lastStateAt<3000?state?.values:null;
 const summary=dockingMissionSummary(current?.tank_status,current?.outdoor);
 const o=current?.outdoor,stage=o?.priority_route?.state;
 if(dockingAbortPending&&o?.ack?.id===dockingAbortPending.id){
  if(!o.ack.ok)dockingAbortError=o.ack.message||'도킹 중단 요청 실패';
  dockingAbortPending=null;
 }
 if(dockingAbortPending&&performance.now()-dockingAbortPending.at>5000){dockingAbortPending=null;dockingAbortError='중단 요청 응답 대기';}
 if(orbitAbortPending&&o?.ack?.id===orbitAbortPending.id){
  if(!o.ack.ok)orbitAbortError=o.ack.message||'선회 중단 요청 실패';
  orbitAbortPending=null;
 }
 if(orbitAbortPending&&performance.now()-orbitAbortPending.at>5000){orbitAbortPending=null;orbitAbortError='선회 중단 응답 확인 필요';}
 const orbitBusy=orbitAbortBusy||!!orbitAbortPending;
 abortOrbitButton.disabled=orbitBusy||!o||o.age_s>=2||!o.execute||!o.buoy_orbit||!o.orbit_abort_schema;
 abortOrbitButton.textContent=orbitBusy?'선회 중단 중':orbitAbortError?'선회 중단 재시도':'선회 중단';
 abortOrbitButton.title=orbitAbortError||(!o?.orbit_abort_schema?'선회 중단 코드를 적용한 브링업에서 사용':'현재 선회를 종료하고 다음 GPS 웨이포인트로 진행');
 const busy=dockingAbortBusy||!!dockingAbortPending||stage==='aborting';
 abortDockingButton.disabled=busy||!o||o.age_s>=2||!o.execute||!(o.docking_phase==='searching'||['neutralizing','docking','completing','error'].includes(stage));
 abortDockingButton.textContent=busy?'도킹 중단 중':dockingAbortError?'도킹 중단 재시도':'도킹 중단';
 abortDockingButton.title=dockingAbortError||'현재 도킹을 종료하고 다음 GPS 웨이포인트로 진행';
 if(dockingAbortError){summary.label=dockingAbortError;summary.tone='waiting';}
 tankMissionPanel.dataset.state=summary.tone;tankMissionPanel.title=summary.reason||'';
 for(const [id,text] of [['tankMissionState',summary.label],['tankMissionTarget',summary.target||'—'],['tankMissionDistance',summary.distance===null?'—':F(summary.distance,2)+' m'],['tankMissionArrival',summary.lastReached||'—'],['tankMissionCount',summary.total===null?'—':summary.total+'회']])if($(id).textContent!==text)$(id).textContent=text;
 return summary;
}
function drawTank(){if(slamOnly&&!slamDisplay)return;const mission=renderTankMission(),canvas=$('tankMap'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d');c.clearRect(0,0,w,h);const meta=state?.values?.map,pose=tankDisplayPose();ageText('mapAge',meta);$('mapEmpty').hidden=Boolean(mapPixels&&mapMeta);$('saveMap').disabled=!mapSaveAvailable();if(!mapPixels||!mapMeta)return;const session=state?.values?.tank_session?.session_id;if(session&&session!==tankSession){tankSession=session;resetTankView();tankTrail=[];route={a:null,b:null};routeRevision='';routeDirty=false;routeAppliedAt=0;routeLabel();}syncTankRoute(session);const m=mapMeta,r=m.resolution_m,theta=radians(m.origin_yaw_deg||0),co=Math.cos(theta),si=Math.sin(theta),corners=[[0,0],[m.width*r,0],[0,m.height*r],[m.width*r,m.height*r]].map(([x,y])=>[m.origin_x_m+co*x-si*y,m.origin_y_m+si*x+co*y]),xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),base=Math.min(w/(maxX-minX||1),h/(maxY-minY||1));if(tankView.auto)tankView=fitTankBounds(corners);const s=base*tankView.zoom,viewX=tankView.x??(minX+maxX)/2,viewY=tankView.y??(minY+maxY)/2,cx=w/2-viewX*s,cy=h/2+viewY*s;const screen=(x,y)=>[cx+x*s,cy-y*s];tankTransform={s,cx,cy,dpr,w,h,viewX,viewY};$("tankZoomLabel").textContent=Math.round(tankView.zoom*100)+"%";$("tankZoomOut").disabled=tankView.zoom<=.01;
 c.save();c.translate(...screen(m.origin_x_m,m.origin_y_m));c.rotate(-theta);c.scale(s,-s);c.imageSmoothingEnabled=false;c.drawImage(mapPixels,0,0,m.width*r,m.height*r);c.restore();
 drawMapErasure(c,screen,s);
 drawLocalizationRegion(c,screen,dpr);drawExpectedPose(c,screen,s,dpr);
 if(tankCrop){const [x0,y0,x1,y1]=tankCrop,a=screen(x0,y1),b=screen(x1,y0);c.save();c.fillStyle='#f4bb4228';c.strokeStyle='#ffc85c';c.lineWidth=2*dpr;c.setLineDash([7*dpr,4*dpr]);c.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);c.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);c.restore();}

 const reportTrail=state?.values?.tank_localization_candidates;
 if(pose&&reportTrail?.map_id===session&&reportTrail?.age_s<1&&reportTrail.display_pose&&reportTrail.display_trail_m?.length){
  // Replace the entire odometry-derived trace after every candidate correction.
  tankTrail=(reportTrail.display_trail_m||[]).filter(p=>p.length===2&&p.every(Number.isFinite)).map(p=>[...p]);
  if(!pose.provisional){const from=reportTrail.display_pose,to=pose.position_m,a=radians(pose.orientation.deg_signed.yaw-from.yaw_deg),co=Math.cos(a),si=Math.sin(a);tankTrail=tankTrail.map(([x,y])=>[to.x+co*(x-from.x)-si*(y-from.y),to.y+si*(x-from.x)+co*(y-from.y)]);}
  const point=pose.position_m;tankTrail.push([point.x,point.y]);lastTankStamp=pose.stamp_s;trailWasProvisional=true;
 }else if(pose?.age_s<1&&pose.stamp_s!==lastTankStamp){
  if(trailWasProvisional){tankTrail=[];trailWasProvisional=false;}
  lastTankStamp=pose.stamp_s;const p=pose.position_m,previous=tankTrail.at(-1);if(!previous||Math.hypot(p.x-previous[0],p.y-previous[1])>.02)tankTrail.push([p.x,p.y]);if(tankTrail.length>600)tankTrail.shift();
 }else if(!pose&&trailWasProvisional){tankTrail=[];}

 c.strokeStyle='#7696a6';c.lineWidth=1.5*dpr;c.beginPath();tankTrail.forEach((p,i)=>i?c.lineTo(...screen(...p)):c.moveTo(...screen(...p)));c.stroke();
 if(route.a&&route.b){c.strokeStyle='#e6b36a';c.lineWidth=2*dpr;c.setLineDash([5*dpr,5*dpr]);c.beginPath();c.moveTo(...screen(route.a.x,route.a.y));c.lineTo(...screen(route.b.x,route.b.y));c.stroke();c.setLineDash([]);}
 const predicted=state?.values?.tank_prediction,planned=state?.values?.tank_status;
 for(const [points,color,dashed] of [[planned?.age_s<2?planned.path_m:null,'#b5c9b0',false],[predicted?.age_s<1?predicted.points_m:null,'#de90ba',true]]){if(!points?.length)continue;c.strokeStyle=color;c.lineWidth=2*dpr;c.setLineDash(dashed?[4*dpr,4*dpr]:[]);c.beginPath();points.forEach((p,i)=>i?c.lineTo(...screen(...p)):c.moveTo(...screen(...p)));c.stroke();c.setLineDash([]);}
 for(const [key,point] of Object.entries(route)){
  if(!point)continue;const p=screen(point.x,point.y),name=key.toUpperCase(),tag=mission.pointStates[name];
  c.fillStyle=key==='a'?'#1f918b':'#bd843c';c.beginPath();c.arc(...p,8*dpr,0,2*Math.PI);c.fill();
  if(tag){c.strokeStyle=tag==='추종 중'?'#7fe6dc':tag==='도달'?'#a8d79b':'#e6b36a';c.lineWidth=2*dpr;c.beginPath();c.arc(...p,12*dpr,0,2*Math.PI);c.stroke();}
  c.font=`bold ${10*dpr}px Segoe UI`;c.fillStyle='#fff';c.textAlign='center';c.fillText(name,p[0],p[1]+3.5*dpr);
  if(tag){c.font=`bold ${12*dpr}px Segoe UI`;c.strokeStyle='#13212b';c.lineWidth=4*dpr;c.strokeText(`${name} ${tag}`,p[0],p[1]-18*dpr);c.fillText(`${name} ${tag}`,p[0],p[1]-18*dpr);}
  c.textAlign='left';
 }
 const localization=state?.values?.tank_localization_candidates;
 const sameMap=Boolean(localization?.map_id)&&localization.map_id===state?.values?.tank_session?.session_id;
 const fresh=sameMap&&localization.age_s<5, ready=state?.values?.tank_session?.localization_ready===true;
 const candidateAge=Number.isFinite(localization?.candidate_stamp_s)?Math.max(0,(state.server_time_s||localization.stamp_s)-localization.candidate_stamp_s):0;
 const liveCandidates=liveTankCandidates();candidateControls();
 const candidates=fresh&&!localization.manual_locked?(liveCandidates.length?liveCandidates:localization.candidates||[]).filter(p=>[p.x,p.y,p.yaw_deg,p.score].every(Number.isFinite)).slice(0,5):[];
 const historical=localization?.state==='SEARCHING'||candidateAge>15;
 $('globalMatchInfo').textContent=fresh?'자동 위치: '+localization.reason+(localization.observation_count?' · 관측 '+localization.observation_count+'곳 / 이동 '+F(localization.observation_baseline_m,2)+' m':'')+(localization.path_rejected?' · 경로 불일치 '+localization.path_rejected+'개 제외':'')+(Number.isFinite(localization.margin)?' · 후보 점수차 '+F(localization.margin,2):''):'자동 위치 탐색 상태 수신 대기';
 $('candidateInfo').textContent=candidates.length?(localization.display_fallback?'임시 후보 유지 · ':historical?'이전 관측 후보 · ':'미확정 후보 · ')+candidates.map((p,i)=>`#${i+1} (${F(p.x,2)}, ${F(p.y,2)}) m / ${F(p.yaw_deg,1)}° / 점수 ${F(p.score,2)}`).join('   |   ')+(historical?` · ${F(candidateAge,0)}초 전 결과`:''):(ready?'현재 위치 정합 확인됨':fresh&&localization.state==='SEARCHING'?'위치·방향 후보 계산 중입니다. 방향 입력 없이 탐색합니다.':fresh?localization.reason:'자동 탐색 결과를 기다리고 있습니다.');
 for(const [i,point] of candidates.entries()){
  tankArrow(c,point,screen,dpr,point.score>=(localization.accept_score||.8)?'#ffe078':'#c28f55',.8,liveCandidates.length?.65:.3);
 }
 const q=pose?.position_m,angle=pose?.orientation?.deg_signed?.yaw;if(q&&Number.isFinite(angle)){const p=screen(q.x,q.y);c.save();c.translate(...p);c.rotate(-radians(angle));c.fillStyle=pose.age_s<1&&state?.values?.tank_session?.localization_ready!==false?'#3b9cbd':'#bf9a56';c.beginPath();c.moveTo(13*dpr,0);c.lineTo(-8*dpr,-7*dpr);c.lineTo(-5*dpr,0);c.lineTo(-8*dpr,7*dpr);c.closePath();c.fill();c.restore();}
 const selected=liveCandidates.find(p=>p.candidate_id===selectedCandidateId);if(selected)tankArrow(c,selected,screen,dpr,'#ffffff',1.3,1);
 c.fillStyle='#92aabc';c.font=`${11*dpr}px Segoe UI`;c.fillText(`X → · Y ↑ · ${F(r,3)} m / 셀`,12*dpr,h-10*dpr);$('mapInfo').textContent=`${m.frame_id} · ${m.width} × ${m.height} 셀 · ${pose?.held?'마지막 확인 위치':pose?.manual_locked?'지정 위치 추적':pose?.display_fallback?'임시 위치 (재검토 중)':pose?.provisional?'추정 위치 (검증 중)':'현재 위치'} ${q?F(q.x,2)+', '+F(q.y,2)+' m':'대기'}`;const status=state?.values?.tank_status,sessionInfo=state?.values?.tank_session;if(sessionInfo?.fixed_map&&sessionInfo.localization_ready===false)$('tankStatus').textContent='고정 지도 · '+(pose?.held?(localization.reason||'위치 신호 대기 · 마지막 위치 표시'):localization?.manual_locked?(localization.reason||'주변 지도 정합 중'):pose?.display_fallback?'임시 위치 추적 · 후보 비교 2초 / 전체 탐색 12초':pose?.provisional?'추정 위치 실시간 추적 · 후보 비교 2초 / 전체 탐색 60초':fresh?'자동 위치 확정 대기':'위치 추정 상태 수신 대기');else if(status?.age_s<2)$('tankStatus').textContent=status.state+(status.reason?' · '+status.reason:'');else if(sessionInfo?.fixed_map)$('tankStatus').textContent='고정 지도 · 위치 정합 확인됨';}
let trialClock={session:null,last:null,at:0,scales:{}};
function trialWindow(history,end){
 const rows=history.filter(r=>Array.isArray(r)&&Number.isFinite(r[0])&&r[0]>=end-30&&r[0]<=end).sort((a,b)=>a[0]-b[0]);
 const gaps=rows.slice(1).map((r,i)=>r[0]-rows[i][0]).filter(x=>x>0).sort((a,b)=>a-b);
 const interval=gaps.length?gaps[Math.floor(gaps.length/2)]:.3;
 return {rows,interval,gapLimit:Math.max(1,Math.min(3,interval*2.5))};
}
function drawTrial(){
 const data=state?.values?.tank_status,history=(data?.history||[]).filter(r=>Array.isArray(r)&&Number.isFinite(r[0]));
 const newest=history.length?Math.max(...history.map(r=>r[0])):0,now=performance.now(),session=data?.log_directory||null;
 if(trialClock.session!==session)trialClock={session,last:null,at:now,scales:{}};
 if(trialClock.last!==newest){trialClock.last=newest;trialClock.at=now;}
 const end=Number.isFinite(data?.elapsed_s)?data.elapsed_s:newest+(now-trialClock.at)/1000;
 const {rows,interval,gapLimit}=trialWindow(history,end);
 $('trialInfo').textContent=data?`${data.state} · ${data.phase||''} · ${data.legs||0}회 도착 · ${data.age_s>2?'수신 지연':'최근 30초'} · 표시 간격 약 ${F(interval,1)}초`:'젯슨 기록 대기';
 $('trialInfo').title=data?.log_directory||'';
 $('trialReason').textContent=data?.reason||'';$('trialReason').hidden=!data?.reason;
 for(const [id,cols,factor,unit,minimum,step,labels] of [
  ['trialInputs',[4,5],1,'%',Math.max(20,data?.output_limit_percent||20),5,['좌현','우현']],
  ['trialSpeed',[1,2],1,'m/s',.1,.1,['전진','횡방향']],
  ['trialYaw',[3,6],180/Math.PI,'°/s',5,5,['라이다','IMU Z']]]){
  const canvas=$(id),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d');c.clearRect(0,0,w,h);
  const values=rows.flatMap(r=>cols.filter(k=>Number.isFinite(r[k])).map(k=>r[k]*factor));
  const magnitude=Math.max(minimum,...values.map(Math.abs));
  const bound=Math.max(trialClock.scales[id]||minimum,Math.ceil(magnitude/step)*step);trialClock.scales[id]=bound;
  const left=53*dpr,right=w-12*dpr,top=30*dpr,bottom=h-29*dpr,span=bound*2.16;
  const X=t=>left+(right-left)*(t-(end-30))/30,Y=v=>(top+bottom)/2-v/span*(bottom-top);
  c.font=`${11*dpr}px Segoe UI`;c.lineWidth=dpr;c.setLineDash([]);
  for(const value of [-bound,-bound/2,0,bound/2,bound]){
   const y=Y(value);c.strokeStyle=value===0?'#607b8e':'#263b4a';c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();
   c.fillStyle='#a6bdcb';c.textAlign='right';c.fillText(F(value,unit==='m/s'?2:1),left-7*dpr,y+4*dpr);
  }
  for(const seconds of [-30,-20,-10,0]){
   const x=X(end+seconds);c.strokeStyle='#263b4a';c.beginPath();c.moveTo(x,top);c.lineTo(x,bottom);c.stroke();
   c.textAlign=seconds===-30?'left':seconds===0?'right':'center';c.fillStyle='#a6bdcb';c.fillText(seconds===0?'현재':`${seconds}초`,x,h-8*dpr);
  }
  c.textAlign='left';c.fillStyle='#a6bdcb';c.fillText(unit,4*dpr,15*dpr);
  cols.forEach((k,j)=>{
   const color=j?'#ffad66':'#63ded0',legendX=left+j*95*dpr;c.strokeStyle=color;c.lineWidth=1.7*dpr;c.setLineDash([]);
   c.beginPath();c.moveTo(legendX,12*dpr);c.lineTo(legendX+18*dpr,12*dpr);c.stroke();c.fillStyle=color;c.fillText(labels[j],legendX+23*dpr,16*dpr);
   c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();c.beginPath();let previous=null;
   for(const row of rows){
    if(!Number.isFinite(row[k])){previous=null;continue;}
    const x=X(row[0]),y=Y(row[k]*factor),connected=previous&&row[0]>previous[0]&&row[0]-previous[0]<=gapLimit;
    if(connected){if(id==='trialInputs')c.lineTo(x,Y(previous[k]*factor));c.lineTo(x,y);}else c.moveTo(x,y);
    previous=row;
   }
   c.stroke();c.setLineDash([]);c.fillStyle=color;
   for(const row of rows)if(Number.isFinite(row[k])){c.beginPath();c.arc(X(row[0]),Y(row[k]*factor),1.5*dpr,0,2*Math.PI);c.fill();}
   c.restore();c.setLineDash([]);
  });
  c.fillStyle='#a6bdcb';c.textAlign='center';
  if(!values.length)c.fillText('최근 표본 대기',(left+right)/2,(top+bottom)/2-9*dpr);
  else if(rows.some((r,i)=>i&&r[0]-rows[i-1][0]>gapLimit))c.fillText('긴 수신 공백은 선을 연결하지 않습니다',(left+right)/2,bottom-3*dpr);
  c.textAlign='left';
 }
}
new ResizeObserver(()=>{if(!slamOnly&&!embeddedSensors)drawTrial();}).observe(document.querySelector('.trial-charts'));
function routeLabel(){
 const point=p=>p?`${F(p.x,2)}, ${F(p.y,2)} m`:'미지정';
 $('routeInfo').textContent=`${$('dockingVersion').value} · A ${point(route.a)} / B ${point(route.b)}${routeDirty?' · 저장 전':''}`;
 $('saveRoute').disabled=!(route.a&&route.b);
 const button=$('loadDockingRoute'),selected=!!loadedDockingPath&&$('savedDockingRoutes').value===loadedDockingPath;
 const applied=selected&&appliedDockingSelection?.route?.revision===routeRevision&&!routeDirty;
 button.disabled=dockingApplyBusy||!$('savedDockingRoutes').value;
 button.textContent=dockingApplyBusy?'불러오기 및 적용 중':'불러오기 및 적용';
 button.classList.toggle('active',applied);button.setAttribute('aria-pressed',String(applied));
 button.title='선택한 지도와 A/B 경로를 불러와 다음 주행 시작에 적용합니다.';
 $('savedDockingRoutes').disabled=dockingApplyBusy;
 dockingAppliedStatus.textContent=dockingApplyBusy?'적용 중':applied?`적용됨 · ${appliedDockingSelection.version}`:routeDirty?'수정됨 · 재적용 필요':'미적용';
 dockingAppliedStatus.style.color=dockingApplyBusy||routeDirty?'var(--amber)':applied?'var(--cyan)':'var(--muted)';
}
$('tankMap').onclick=e=>{if(regionMode||regionDrag||regionBusy)return;if(eraserMode||eraserBusy)return;if(expectedPoseClick(e))return;if(tankCropMode||tankCropDrag)return;if(tankDragged){tankDragged=false;return;}if(!tankTransform||!selectedMapAction)return;const rect=$('tankMap').getBoundingClientRect(),{s,cx,cy,dpr}=tankTransform;route[selectedMapAction]={x:((e.clientX-rect.left)*dpr-cx)/s,y:(cy-(e.clientY-rect.top)*dpr)/s};routeDirty=true;const key=selectedMapAction;selectedMapAction=null;$(`set${key.toUpperCase()}`).classList.remove('active');routeLabel();drawTank();};
function fitTankBounds(corners){
 const xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]);
 return {x:(Math.min(...xs)+Math.max(...xs))/2,y:(Math.min(...ys)+Math.max(...ys))/2,zoom:1,auto:true};
}
function zoomTank(factor,clientX,clientY){
 if(!tankTransform)return;const rect=$('tankMap').getBoundingClientRect(),t=tankTransform;
 const px=clientX===undefined?t.w/2:(clientX-rect.left)*t.dpr,py=clientY===undefined?t.h/2:(clientY-rect.top)*t.dpr;
 const worldX=(px-t.cx)/t.s,worldY=(t.cy-py)/t.s,next=clamp(tankView.zoom*factor,.01,32),scale=t.s*next/tankView.zoom;
 tankView.auto=false;tankView.x=worldX-(px-t.w/2)/scale;tankView.y=worldY+(py-t.h/2)/scale;tankView.zoom=next;drawTank();
}
function tankWorld(event){const r=$('tankMap').getBoundingClientRect(),t=tankTransform;return [((event.clientX-r.left)*t.dpr-t.cx)/t.s,(t.cy-(event.clientY-r.top)*t.dpr)/t.s];}
function renderMapCrop(){
 $('cropMap').classList.toggle('active',tankCropMode);$('clearMapCrop').disabled=!tankCrop;
 $('saveMap').textContent=tankCrop?'구역만 저장':state?.values?.map?.source==='checkpoint'?'자동 저장 지도 저장':'현재 지도 저장';
 $('mapCropInfo').textContent=tankCrop?`선택 구역 ${(tankCrop[2]-tankCrop[0]).toFixed(1)} × ${(tankCrop[3]-tankCrop[1]).toFixed(1)} m`:tankCropMode?'지도에서 드래그 · 벽면과 A/B·접근 공간을 포함하세요.':'전체 지도 저장';
}
$('cropMap').onclick=()=>{eraserMode=false;renderMapEraser();tankCropMode=!tankCropMode;selectedMapAction=null;for(const id of ['setA','setB'])$(id).classList.remove('active');renderMapCrop();};
$('clearMapCrop').onclick=()=>{tankCrop=null;tankCropDrag=null;tankCropMode=false;renderMapCrop();drawTank();};
function onKnownTankMap(clientX,clientY){
 if(!tankTransform||!mapMeta||!mapCells)return false;
 const rect=$('tankMap').getBoundingClientRect(),t=tankTransform,m=mapMeta;
 const x=((clientX-rect.left)*t.dpr-t.cx)/t.s-m.origin_x_m,y=(t.cy-(clientY-rect.top)*t.dpr)/t.s-m.origin_y_m;
 const a=radians(m.origin_yaw_deg||0),col=Math.floor((Math.cos(a)*x+Math.sin(a)*y)/m.resolution_m),row=Math.floor((-Math.sin(a)*x+Math.cos(a)*y)/m.resolution_m);
 return col>=0&&row>=0&&col<m.width&&row<m.height&&mapCells[row*m.width+col]>=0;
}
$('tankMap').addEventListener('wheel',e=>{if(!tankTransform||!mapMeta)return;e.preventDefault();zoomTank(Math.exp(-clamp(e.deltaY,-200,200)*.003),e.clientX,e.clientY);},{passive:false});
$('tankMap').addEventListener('pointerdown',e=>{if(e.button!==0)return;tankDragged=false;const corner=regionHandleAt(e);if(corner>=0){regionDrag={id:e.pointerId,corner,vertices:regionVertices(localizationRegion),enabled:localizationRegion.enabled};regionDraft=regionVertices(localizationRegion);$('tankMap').setPointerCapture(e.pointerId);return;}if(regionMode&&tankTransform&&!regionBusy){regionDrag={id:e.pointerId,start:tankWorld(e)};regionDraft=null;$('tankMap').setPointerCapture(e.pointerId);return;}if(regionBusy)return;if(eraserMode&&tankTransform&&!eraserBusy){erasePointer(e,true);$('tankMap').setPointerCapture(e.pointerId);return;}if(eraserBusy)return;if(priorPick)return;if(tankCropMode&&tankTransform){tankCropDrag={id:e.pointerId,start:tankWorld(e)};tankCrop=null;$('tankMap').setPointerCapture(e.pointerId);return;}if(!tankTransform||!mapMeta)return;tankPointer={id:e.pointerId,x:e.clientX,y:e.clientY,viewX:tankTransform.viewX,viewY:tankTransform.viewY,scale:tankTransform.s/tankTransform.dpr};$('tankMap').setPointerCapture(e.pointerId);});
$('tankMap').addEventListener('pointermove',e=>{if(regionDrag?.id===e.pointerId){const b=tankWorld(e);if(regionDrag.corner!==undefined){regionDraft=regionDrag.vertices.map(p=>[...p]);regionDraft[regionDrag.corner]=b;}else regionDraft=rectangleVertices(regionDrag.start,b);tankDragged=true;renderLocalizationRegion();drawTank();return;}if(eraserPointer?.id===e.pointerId){erasePointer(e);return;}if(tankCropDrag?.id===e.pointerId){const a=tankCropDrag.start,b=tankWorld(e);tankCrop=[Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.max(a[0],b[0]),Math.max(a[1],b[1])];tankDragged=true;renderMapCrop();drawTank();return;}if(!tankPointer||tankPointer.id!==e.pointerId)return;const dx=e.clientX-tankPointer.x,dy=e.clientY-tankPointer.y;if(Math.hypot(dx,dy)>4)tankDragged=true;if(!tankDragged)return;tankView.auto=false;tankView.x=tankPointer.viewX-dx/tankPointer.scale;tankView.y=tankPointer.viewY+dy/tankPointer.scale;$('tankMap').classList.add('dragging');drawTank();});
function endTankDrag(e){if(regionDrag?.id===e.pointerId){const editing=regionDrag,b=tankWorld(e),a=editing.start;regionDrag=null;regionMode=false;tankDragged=true;if(editing.corner!==undefined){if(e.type==='pointerup'){const vertices=editing.vertices.map(p=>[...p]);vertices[editing.corner]=b;saveLocalizationRegion({vertices,enabled:editing.enabled});}else{regionDraft=null;renderLocalizationRegion();drawTank();}return;}
 if(e.type==='pointerup'&&Math.hypot(b[0]-a[0],b[1]-a[1])*tankTransform.s>4*tankTransform.dpr&&b[0]!==a[0]&&b[1]!==a[1]){regionDraft=rectangleVertices(a,b);saveLocalizationRegion({vertices:regionDraft,enabled:true});}
 else{regionDraft=null;renderLocalizationRegion();drawTank();}return;}if(eraserPointer?.id===e.pointerId){if(e.type==='pointerup')erasePointer(e);eraserPointer=null;}if(tankCropDrag?.id===e.pointerId){tankCropDrag=null;tankCropMode=false;if(tankCrop&&(tankCrop[2]<=tankCrop[0]||tankCrop[3]<=tankCrop[1]))tankCrop=null;renderMapCrop();}if(tankPointer?.id===e.pointerId){tankPointer=null;$('tankMap').classList.remove('dragging');}}
$('tankMap').addEventListener('pointerup',endTankDrag);$('tankMap').addEventListener('pointercancel',endTankDrag);
$('tankZoomIn').onclick=()=>{zoomTank(1.4);};$('tankZoomOut').onclick=()=>{zoomTank(1/1.4);};
$('tankFit').onclick=()=>{resetTankView();drawTank();};
document.addEventListener('keydown',e=>{if(e.key==='Escape'){cancelRegionPick();eraserMode=false;eraserPointer=null;renderMapEraser();cancelExpectedPick();tankCropDrag=null;tankCropMode=false;renderMapCrop();cancelTankDrag();selectedMapAction=null;for(const key of ['setA','setB'])$(key).classList.remove('active');drawTank();}});
new ResizeObserver(()=>drawTank()).observe($('tankMap'));
$('candidateButtons').onclick=e=>{const b=e.target.closest('button[data-candidate-id]');if(!b)return;selectedCandidateId=b.dataset.candidateId;drawTank();};
$('applyCandidate').onclick=async()=>{const selected=liveTankCandidates().find(p=>p.candidate_id===selectedCandidateId);if(!selected)return;const result=await post('/api/initial_pose',{map_id:candidateMapId,candidate_id:selected.candidate_id});if(result)$('tankStatus').textContent=result.message;};
$('dockingVersion').onchange=()=>{routeDirty=true;routeLabel();};
$('saveRoute').onclick=async()=>{
 const button=$('saveRoute');button.disabled=true;
 try{const result=await dockingRequest('save',{version:$('dockingVersion').value,a:route.a,b:route.b});if(result){applyDockingRoute(result.route);await refreshDockingRoutes();$('savedDockingRoutes').value=result.route.profile_file;$('loadDockingRoute').disabled=false;$('dockingRouteStatus').textContent=result.message;}}
 finally{routeLabel();}
};
new ResizeObserver(()=>{if(graphOnly)return;if(slamOnly){if(slamDisplay)drawTank();return;}drawAttitude();drawTrend();drawScan();if(!sensorOnly)drawGps();if(!embeddedSensors)drawTank();}).observe(document.querySelector('.main-grid'));

let sensorRestartPending=false,sensorRestartError='';
let watertrialControlPending=false,watertrialControlError='',watertrialControlAction='';
document.querySelectorAll('[data-watertrial-action]').forEach(button=>button.addEventListener('click',async()=>{
 if(watertrialControlPending||state?.watertrial_control?.busy)return;
 const action=button.dataset.watertrialAction,requestAction=action==='stop_other'?'stop_all':action;
 watertrialControlPending=true;watertrialControlError='';watertrialControlAction=action;renderWatertrialControl();
 try{
  const response=await fetch('/api/sensors',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:requestAction}),signal:AbortSignal.timeout(8000)});
  const result=await response.json();if(!response.ok)throw Error(result.error||'WaterTrial 작업 요청 실패');
  if(state)state.watertrial_control={busy:true,action,error:null};
 }catch(error){watertrialControlError=error.message;}
 finally{watertrialControlPending=false;renderWatertrialControl();}
}));
function renderWatertrialControl(){
 const control=state?.watertrial_control||{},action=control.action||watertrialControlAction,legacyStop=action==='stop_other'&&state?.node_stop;
 const busy=watertrialControlPending||control.busy||!!legacyStop?.busy,status=$('watertrialControlStatus');
 document.querySelectorAll('[data-watertrial-action]').forEach(button=>button.disabled=busy||sensorRestartPending||!!state?.sensor_restart?.busy||!!state?.node_stop?.busy||!!state?.esc_initialize?.busy);
 if(busy){const labels={stop_other:'WaterTrial 외 노드 종료 중',stop_watertrial:'WaterTrial 전체 종료 중'};status.textContent=labels[action]||'요청 처리 중';status.style.color='var(--amber)';}
 else if(watertrialControlError||control.error||legacyStop?.error){status.textContent=watertrialControlError||control.error||legacyStop?.error;status.style.color='var(--red)';}
 else if(action==='stop_other'&&state?.node_stop?.completed){status.textContent='WaterTrial 외 노드 종료 완료';status.style.color='var(--cyan)';}
 else if(action){const labels={stop_other:'WaterTrial 외 노드 종료 완료',stop_watertrial:'WaterTrial 종료 완료'};status.textContent=labels[action]||'준비됨';status.style.color='var(--cyan)';}
 else{status.textContent='준비됨';status.style.color='';}
}
let escInitializePending=false,escInitializeError='';
$('initializeEsc').onclick=async()=>{
 if(escInitializePending||state?.esc_initialize?.busy)return;
 escInitializePending=true;escInitializeError='';renderRuntime();
 try{
  const response=await fetch('/api/sensors',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'initialize_esc'}),signal:AbortSignal.timeout(8000)});
  const result=await response.json();if(!response.ok)throw Error(result.error||'ESC 초기화 요청 실패');
  if(state)state.esc_initialize={busy:true,error:null,acknowledged:false};
 }catch(error){escInitializeError=error.message;}
 finally{escInitializePending=false;renderRuntime();}
};
$('runtime-sensors').onclick=async()=>{
 if(sensorRestartPending||state?.sensor_restart?.busy||state?.watertrial_control?.busy)return;
 sensorRestartPending=true;sensorRestartError='';renderRuntime();
 try{
  const response=await fetch('/api/sensors',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'restart'}),signal:AbortSignal.timeout(8000)});
  const result=await response.json();if(!response.ok)throw Error(result.error||'센서 재시작 요청 실패');
  if(state)state.sensor_restart={busy:true,error:null};
 }catch(error){sensorRestartError=error.message;}
 finally{sensorRestartPending=false;renderRuntime();}
};
function renderRuntime(){
 renderWatertrialControl();
 const esc=$('initializeEsc'),initializing=escInitializePending||!!state?.esc_initialize?.busy;
 const escError=state?.esc_initialize?.acknowledged&&!state?.esc_initialize?.busy?state?.esc_initialize?.error:(escInitializeError||state?.esc_initialize?.error);
 esc.disabled=initializing||sensorRestartPending||!!state?.sensor_restart?.busy;
 esc.setAttribute('aria-busy',String(initializing));
 esc.className='runtime-item '+(initializing?'restarting':escError?'unknown':'');
 esc.textContent=initializing?'ESC 초기화 중':escError?'ESC 요청 실패':'ESC 초기화';
 esc.title=escError||(state?.esc_initialize?.acknowledged?'중립 요청 전달 완료 · ESC 실제 초기화는 소리로 확인하세요.':'양쪽 중립으로 ESC 초기화를 요청합니다. 실행 중인 mode_test는 7초 뒤 출력을 재개합니다.');
 const runtime=state?.values?.runtime,fresh=performance.now()-lastStateAt<3000&&runtime?.age_s<7;
 const labels={sensors:'센서',mapping:'매핑'},texts={running:'실행 중',stopped:'정지',restarting:'재시작 중',unknown:'확인 중'};
 for(const [key,label] of Object.entries(labels)){
  const status=fresh&&texts[runtime.services?.[key]]?runtime.services[key]:'unknown',el=$('runtime-'+key);
  let text=texts[status];
  el.className='runtime-item '+status;el.textContent=label+' · '+text;
  el.title='젯슨 프로세스 기준 · 약 3초마다 갱신';
  if(key==='sensors'){
   const busy=sensorRestartPending||!!state?.sensor_restart?.busy,error=sensorRestartError||state?.sensor_restart?.error;
   el.disabled=busy||initializing;el.setAttribute('aria-busy',String(busy));
   el.title=error||'클릭하면 센서 브링업을 재시작합니다.';
   if(busy){el.className='runtime-item restarting';el.textContent='센서 · 재시작 중';}
   else if(error){el.className='runtime-item unknown';el.textContent='센서 · 재시작 실패';}
  }
 }
 $('runtime-sensors').disabled=sensorRestartPending||!!state?.sensor_restart?.busy||!!state?.watertrial_control?.busy||initializing;
}
setInterval(()=>{if(slamOnly){renderSlamControls();renderTankMission();return;}renderRuntime();if(!embeddedSensors&&performance.now()-lastStateAt>=3000)drawTank();},1000);
async function stateLoop(){
 try{
  const query=new URLSearchParams({view:slamOnly?'slam':embeddedSensors?'sensors':'full'});
  if(viewQuery.get('capture')==='1')query.set('capture','1');
  const response=await fetch('/api/state?'+query,{cache:'no-store',signal:AbortSignal.timeout(6000)});
  if(response.ok){acceptState(await response.json());lastStateAt=performance.now();if(!previewTopic)previewTopic=state.selected_topic;updateSensors();if(!slamOnly){if(!embeddedSensors)drawTrial();renderRuntime();}}
 }catch(_){}
 setTimeout(stateLoop,document.hidden?1500:slamOnly&&!slamDisplay?1000:250);
}
async function cloudLoop(){try{if(!slamOnly&&!$('pauseCloud').checked&&!document.hidden){const response=await fetch('/api/cloud?after='+(cloud.sequence??-1),{cache:'no-store',signal:AbortSignal.timeout(6000)});if(response.status===200){const buffer=await response.arrayBuffer();pointBytes+=buffer.byteLength;cloud.update(buffer);}}}catch(_){}setTimeout(cloudLoop,document.hidden?1500:330);}
let fastSensorFrame=null,fastPaintPending=false;
function withFastSensors(next){
 const packet=fastSensorFrame?.packet,elapsed=fastSensorFrame?(performance.now()-fastSensorFrame.at)/1000:Infinity;
 if(!packet||packet.error||!Number.isFinite(packet.age_s)||packet.age_s+elapsed>=1)return next;
 const values={...next.values};
 for(const key of ['heading','gps','gps_velocity','virtual_gps','scan']){
  const value=packet.values?.[key],base=values[key];if(!value)continue;
  // Preserve the owner's selected heading and obstacle-scan sources.
  if(['heading','scan'].includes(key)&&(!base?.topic||base.topic!==value.topic))continue;
  values[key]={...base,...value,age_s:(value.age_s||0)+Math.max(0,packet.age_s)+elapsed};
  if(key==='gps_velocity'&&Number.isFinite(value.east_m_s)&&Number.isFinite(value.north_m_s))values[key].speed_m_s=Math.hypot(value.east_m_s,value.north_m_s);
 }
 return {...next,values};
}
function startFastSensors(){
 const stream=new EventSource('http://127.0.0.1:18766/events');
 stream.onmessage=event=>{
  try{
   const packet=JSON.parse(event.data);
   if(packet.version!=='kabot-pc-fast-feed-v1'||packet.error||!Number.isFinite(packet.age_s)||packet.age_s<0||packet.age_s>=1){fastSensorFrame=null;return;}
   fastSensorFrame={packet,at:performance.now()};
   if(!state||fastPaintPending||document.hidden)return;
   fastPaintPending=true;requestAnimationFrame(()=>{fastPaintPending=false;state=withFastSensors(state);updateSensors();
    if(embeddedSensors)parent.postMessage({kind:'fast-sensors',values:{heading:state.values.heading,gps:state.values.gps,gps_velocity:state.values.gps_velocity}},location.origin);
   });
  }catch(_){}
 };
 stream.onerror=()=>{fastSensorFrame=null;};
}
function trafficRates(current,previous){
 if(!previous||current.at<=previous.at)return null;
 const elapsed=current.at-previous.at,rate=(a,b)=>Number.isFinite(a)&&Number.isFinite(b)&&a>=b?(a-b)/elapsed:null;
 const proxy=current.connection.uptime_s>=previous.connection.uptime_s?rate(current.connection.received_bytes,previous.connection.received_bytes):null;
 const sensor=current.feed?.instance_id&&current.feed.instance_id===previous.feed?.instance_id?rate(current.feed.received_bytes,previous.feed.received_bytes):null;
 return {proxy,sensor,total:(proxy??0)+(sensor??0),partial:proxy===null||sensor===null};
}
let networkSample=null;
async function connectionLoop(){
 try{
  const [response,feed]=await Promise.all([fetch('/api/connection',{cache:'no-store',signal:AbortSignal.timeout(3000)}),fetch('http://127.0.0.1:18766/metrics',{cache:'no-store',signal:AbortSignal.timeout(1500)}).then(r=>r.ok?r.json():null).catch(()=>null)]);
  if(!response.ok)throw Error('Connection status unavailable');
  connection=await response.json();
  const sample={at:performance.now(),connection,feed},rates=trafficRates(sample,networkSample);networkSample=sample;
  const age=feed&&!feed.error&&feed.connected_host===connection.active_host&&Number.isFinite(feed.age_s)?feed.age_s:null;
  $('network').textContent=rates?`수신 ${F(rates.total,1)} KB/s${rates.partial?' · 일부':''}${age===null?' · 센서 대기':` · ${F(age*1000,0)} ms`}`:'수신 측정 중';
  $('network').title=`현재 대시보드·센서 데이터 수신량이며 최대 회선 속도가 아닙니다. 대시보드 ${F(rates?.proxy,1)} KB/s · 센서 ${F(rates?.sensor,1)} KB/s`;
 }catch(_){$('network').textContent='수신 확인 대기';}
 const route=connection?.connection_path,pathLabels={hotspot:'핫스팟 SSH',ssh:'SSH 연결',starlink:'스타링크 SSH',direct:'직결'},path=pathLabels[route]||'SSH 경로 미확인';
 const fresh=performance.now()-lastStateAt<3000&&state,runtimeSensors=state?.values?.runtime?.services?.sensors,sensorsActive=runtimeSensors?runtimeSensors==='running':connection?.sensors_running!==false;
 const live=fresh&&sensorsActive,status=fresh?(live?'연결됨':'브링업 대기'):connection?.status||'연결 대기';
 $('connection').className='badge '+(live?'live':'waiting');$('connection').textContent=`${path} · ${status}`;$('connection').title=[connection?.host,connection?.target_ip].filter(Boolean).join(' · ');
 if(!fresh&&lastStateAt){for(const id of ['cloudAge','imuAge','scanAge','gpsAge']){$(id).textContent='연결 지연';$(id).className='age stale';}}
 setTimeout(connectionLoop,1000);
}
async function mapLoop(){try{const meta=state?.values?.map,instance=serverInstance,reset=tankResetId,newStream=meta?.stream_id!==mapStream;if(!document.hidden&&(!slamOnly||slamDisplay)&&!tankResetPending&&meta&&(newStream||meta.sequence!==mapSeq)){const response=await fetch('/api/map?after='+(newStream?-1:mapSeq),{cache:'no-store',signal:AbortSignal.timeout(6000)});if(response.status===200){const bytes=await response.arrayBuffer(),v=new DataView(bytes);if(bytes.byteLength>=16&&v.getUint32(0,true)===0x50414d4b){const seq=v.getUint32(4,true),w=v.getUint32(8,true),h=v.getUint32(12,true);if(!tankResetPending&&reset===tankResetId&&instance===serverInstance&&seq===meta.sequence&&w*h<=8_000_000&&bytes.byteLength===16+w*h){const cells=new Int8Array(bytes,16),image=new ImageData(w,h);for(let i=0;i<cells.length;i++){const value=cells[i],j=i*4;image.data[j]=value<0?29:value>50?17:187;image.data[j+1]=value<0?48:value>50?31:203;image.data[j+2]=value<0?61:value>50?43:207;image.data[j+3]=255;}mapPixels=document.createElement('canvas');mapPixels.width=w;mapPixels.height=h;mapPixels.getContext('2d').putImageData(image,0,0);mapCells=cells;if(!mapMeta||mapMeta.stream_id!==meta.stream_id)resetTankView();mapMeta=meta;mapSeq=seq;mapStream=meta.stream_id;drawTank();}}}}}catch(_){}setTimeout(mapLoop,document.hidden||!slamDisplay?1800:800);}
if(graphOnly){window.addEventListener('message',e=>{if(e.source===parent&&e.origin===location.origin&&e.data?.kind==='sensor-graph'){state=e.data.state;if(!previewTopic)previewTopic=state.selected_topic;renderGraph();renderPreview();}});}else{stateLoop();if(!slamOnly){startFastSensors();cloudLoop();connectionLoop();}if(!embeddedSensors)mapLoop();}

window.kabotCaptureView=v=>{
 if(!v)return {mode:cloud.mode,range:cloud.range,scanRange,azimuth:cloud.azimuth,elevation:cloud.elevation,rangeValue:$('range').value,color:$('color').value,pointSize:$('pointSize').value,signedAngles:$('signedAngles').checked,tableMode,search:$('search').value};
 if(Number.isFinite(v.scanRange))scanRange=clamp(v.scanRange,2,Math.max(200,state?.values?.scan?.range_max_m||0));
 if(['top','orbit'].includes(v.mode))cloud.mode=v.mode;
 for(const key of ['range','azimuth','elevation'])if(Number.isFinite(v[key]))cloud[key]=v[key];
 for(const [id,key] of [['range','rangeValue'],['color','color'],['pointSize','pointSize']])if(v[key]!=null)$(id).value=String(v[key]);
 if(['nodes','topics'].includes(v.tableMode)){tableMode=v.tableMode;$('nodesTab').classList.toggle('active',tableMode==='nodes');$('topicsTab').classList.toggle('active',tableMode==='topics');}if(typeof v.search==='string')$('search').value=v.search;if(Array.isArray(state?.topics)&&Array.isArray(state?.nodes))renderGraph();$('signedAngles').checked=!!v.signedAngles;cloud.drawSoon();drawScan();
};
window.kabotCaptureDraw=()=>{cloud.draw();drawScan();};


let hullPending=null,hullBusy=false;
async function hullRequest(action){
 hullBusy=true;renderHull();
 try{const r=await fetch('/api/outdoor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,name:$('hullName').value.trim()}),signal:AbortSignal.timeout(4000)});const v=await r.json();if(!r.ok)throw Error(v.error||'형상 요청 실패');hullPending={id:v.request_id,at:performance.now()};}
 catch(e){$('hullStatus').textContent=e.message;}finally{hullBusy=false;}
}
function renderHull(){
 const s=state?.values?.hull_mask,c=s?.capture||{},fresh=s?.age_s<3;
 if(hullPending&&s?.ack?.id===hullPending.id){if(!s.ack.ok)$('hullStatus').textContent=s.ack.error;hullPending=null;}
 if(hullPending&&performance.now()-hullPending.at>5000){hullPending=null;$('hullStatus').textContent='형상 요청 응답 대기';}
 const busy=hullBusy||!!hullPending;
 $('captureHull').disabled=!fresh||busy;$('captureHull').textContent=s?.capturing?'기록 취소':'배 형상 기록';
 $('saveHull').disabled=!fresh||busy||!s?.capturing||!c.ready;$('disableHull').disabled=!fresh||busy||!s?.active;
 if(busy){$('hullStatus').textContent='적용 대기';return;}
 $('hullStatus').textContent=!fresh?'형상 필터 연결 대기':s.error||c.error||(s.capturing?(c.ready?`형상 수렴 · ${c.points}점 · 저장 가능`:`기록 ${Math.round(c.elapsed_s||0)}초 · 회전 ${Math.round(c.yaw_span_deg||0)}° · 고정 ${c.points||0}점 · 주변과 떨어져 천천히 회전`):s.active?`${s.name} 적용 · 이번 프레임 ${s.removed}점 제외`:'저장 형상 없음');
}
$('captureHull').onclick=()=>hullRequest(state?.values?.hull_mask?.capturing?'hull_stop':'hull_start');
$('saveHull').onclick=()=>hullRequest('hull_save');$('disableHull').onclick=()=>hullRequest('hull_disable');
$('viewHull').onclick=async()=>{await post('/api/select',{topic:'/lidar/self_shape'});cloud.mode='orbit';cloud.range=2;cloud.drawSoon();};
$('normalCloud').onclick=async()=>{await post('/api/select',{topic:'/velodyne_points'});cloud.range=Number($('range').value)||20;cloud.drawSoon();};

// Detection display is independent of the two navigation-consumer switches.
let cameraState=null,cameraPending=false,cameraBlob=null,cameraLastFrame=null,cameraRequestError='',cameraShownBackend=null;
let cameraFpsTimer=null,cameraFpsDirty=false,cameraFpsVersion=0,cameraFpsSaving=false;
function localCameraState(){
 let saved={};try{saved=JSON.parse(localStorage.getItem('kabot-vision-local')||'{}')||{};}catch(_){}
 const settings={camera:false,camera_backend:saved.camera_backend||'logitech',fps:Number(saved.fps)||5,corridor:false,objects:false,updated_unix:Number(saved.updated_unix)||0};
 for(const key of ['camera','corridor','objects'])if(typeof saved[key]==='boolean')settings[key]=saved[key];
 return {settings,local_only:true,camera_selection_schema:1,camera_fps_schema:1,camera_running:false,corridor_active:false,runtime_fresh:false,error:'PC 설정 저장됨 · 젯슨 카메라 기능 적용 대기'};
}
const cameraButtons=[['corridorToggle','corridor','라이다보조'],['objectsToggle','objects','객체인식']];
const cameraBackendButtons=[['logitechCameraToggle','logitech','로지텍 카메라'],['depthCameraToggle','depth','뎁스 카메라']];
function renderCamera(){
 const config=cameraState?.settings||{};
 for(const [id,backend,label] of cameraBackendButtons){const button=$(id),on=!!config.camera&&config.camera_backend===backend;button.textContent=label+' '+(on?'켜짐':'꺼짐');button.setAttribute('aria-pressed',String(on));button.disabled=cameraPending||cameraState?.camera_selection_schema!==1;button.title=cameraRequestError||'';}
 for(const [id,key,label] of cameraButtons){const button=$(id);button.textContent=label+' '+(config[key]?'켜짐':'꺼짐');button.setAttribute('aria-pressed',String(!!config[key]));button.disabled=cameraPending||!cameraState;}
 if(!cameraFpsDirty&&document.activeElement!==$('cameraMaxFps'))$('cameraMaxFps').value=String(config.fps||5);
 $('cameraMaxFps').disabled=cameraState?.camera_fps_schema!==1;
 $('cameraAge').textContent=config.camera?(cameraState.camera_running?'수신 중':'영상 대기'):'꺼짐';
 if(!config.camera||cameraShownBackend!==config.camera_backend){cameraLastFrame=null;if(cameraBlob){URL.revokeObjectURL(cameraBlob);cameraBlob=null;}$('cameraImage').removeAttribute('src');cameraShownBackend=config.camera_backend;}
 $('cameraEmpty').hidden=!!(config.camera&&cameraState.camera_running&&cameraBlob);
 $('cameraImage').hidden=!$('cameraEmpty').hidden;
 $('cameraEmpty').textContent='';
 const parts=[];
 if(cameraState?.preview)parts.push('로컬 미리보기 · 선체 제어 없음');
 if(cameraState?.error)parts.push(cameraState.error);
 if(cameraState?.yolo_error)parts.push(cameraState.yolo_error);
 if(cameraRequestError)parts.push(cameraRequestError);
 if(cameraState?.camera_running){const hsv=cameraState.hsv||[],objects=cameraState.objects||[];parts.push('초록 '+hsv.filter(x=>x.colour==='green').length+' · 빨강 '+hsv.filter(x=>x.colour==='red').length+' · YOLO '+objects.length);}
 if(config.corridor)parts.push(cameraState.corridor_active?'2→3 라이다 통로 보조 중':'라이다보조: GPS 추종 · 2→3 구간/좌우 경계 대기');
 if(config.objects)parts.push(cameraState.object_mapping_ready?'도킹 객체 제어 사용':'도킹 객체 클래스·경로 연결 대기');
 if(config.camera&&!cameraState?.yolo_input)parts.push('YOLO 입력 대기');
 $('cameraStatus').textContent=parts.join(' · ')||'표시와 주행 제어는 독립 설정';
}
for(const [id,backend] of cameraBackendButtons)$(id).onclick=async()=>{
 if(cameraPending||cameraState?.camera_selection_schema!==1)return;cameraPending=true;cameraRequestError='';renderCamera();
 const enabled=!(cameraState.settings.camera&&cameraState.settings.camera_backend===backend);
 try{
  if(cameraState.local_only){const config={...cameraState.settings,camera:enabled,camera_backend:backend,updated_unix:Date.now()/1000};localStorage.setItem('kabot-vision-local',JSON.stringify(config));cameraState=localCameraState();}
  else{const response=await fetch('/api/vision',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:'camera',backend,enabled}),signal:AbortSignal.timeout(4000)}),value=await response.json();if(!response.ok)throw Error(value.error||'카메라 선택 실패');cameraState=value;}
 }catch(error){cameraRequestError=error.message;}
 finally{cameraPending=false;renderCamera();}
};
async function saveCameraFps(){
 clearTimeout(cameraFpsTimer);cameraFpsTimer=null;const input=$('cameraMaxFps'),version=cameraFpsVersion;
 if(cameraFpsSaving)return;
 if(!input.value||!input.reportValidity()||cameraState?.camera_fps_schema!==1)return;
 cameraFpsSaving=true;
 try{
  const fps=Number(input.value);
  if(cameraState.local_only){localStorage.setItem('kabot-vision-local',JSON.stringify({...cameraState.settings,fps,updated_unix:Date.now()/1000}));cameraState=localCameraState();}
  else{const response=await fetch('/api/vision',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:'fps',value:fps}),signal:AbortSignal.timeout(4000)}),value=await response.json();if(!response.ok)throw Error(value.error||'최대 FPS 저장 실패');if(version===cameraFpsVersion)cameraState=value;}
  if(version===cameraFpsVersion){cameraFpsDirty=false;input.title='';cameraRequestError='';}
 }catch(error){input.title=error.message;cameraRequestError=error.message;}
 finally{cameraFpsSaving=false;if(version!==cameraFpsVersion)cameraFpsTimer=setTimeout(saveCameraFps,0);}
 renderCamera();
}
$('cameraMaxFps').addEventListener('input',()=>{cameraFpsDirty=true;cameraFpsVersion++;clearTimeout(cameraFpsTimer);cameraFpsTimer=setTimeout(saveCameraFps,350);});
$('cameraMaxFps').addEventListener('change',saveCameraFps);
for(const [id,key] of cameraButtons)$(id).onclick=async()=>{
 if(cameraPending||!cameraState)return;cameraPending=true;cameraRequestError='';renderCamera();
 try{
  if(cameraState.local_only){const config={...cameraState.settings,[key]:!cameraState.settings[key],updated_unix:Date.now()/1000};localStorage.setItem('kabot-vision-local',JSON.stringify(config));cameraState=localCameraState();}
  else{const response=await fetch('/api/vision',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key,enabled:!cameraState.settings[key]}),signal:AbortSignal.timeout(4000)}),value=await response.json();if(!response.ok)throw Error(value.error||'카메라 설정 실패');cameraState=value;}
 }
 catch(error){cameraRequestError=error.message;}
 finally{cameraPending=false;renderCamera();}
};
async function cameraStatusLoop(){
 try{const response=await fetch('/api/vision',{cache:'no-store',signal:AbortSignal.timeout(3500)});if(response.ok){const incoming=await response.json();if(cameraState?.local_only||(incoming.settings?.updated_unix||0)>=(cameraState?.settings?.updated_unix||0))cameraState=incoming;renderCamera();}else if(response.status===404){cameraState=localCameraState();renderCamera();}else{$('cameraStatus').textContent='카메라 기능 적용/연결 대기';}}
 catch(error){$('cameraStatus').textContent='카메라 연결 대기';}
 setTimeout(cameraStatusLoop,document.hidden?2000:650);
}
async function cameraImageLoop(){
 try{if(!document.hidden&&cameraState?.settings?.camera&&cameraState.camera_running){
  const backend=cameraState.settings.camera_backend,response=await fetch('/api/camera.jpg?frame='+performance.now(),{cache:'no-store',signal:AbortSignal.timeout(3500)});
  if(response.status===200){const blob=await response.blob();if(cameraState.settings.camera&&backend===cameraState.settings.camera_backend){const previous=cameraBlob;cameraBlob=URL.createObjectURL(blob);$('cameraImage').src=cameraBlob;cameraLastFrame=cameraState.frame_stamp_s;if(previous)URL.revokeObjectURL(previous);renderCamera();}}
 }}catch(_){}setTimeout(cameraImageLoop,document.hidden?2000:1000/Math.max(1,Math.min(30,Number(cameraState?.settings?.fps)||5)));
}
if(!slamOnly&&!graphOnly){renderCamera();cameraStatusLoop();cameraImageLoop();}
