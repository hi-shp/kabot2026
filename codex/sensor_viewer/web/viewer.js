'use strict';
const viewQuery=new URLSearchParams(location.search),embeddedSensors=viewQuery.get('sensors_only')==='1',sensorSection=embeddedSensors?viewQuery.get('section'):null;
const graphOnly=sensorSection==='graph',sensorOnly=sensorSection==='sensors';
if(embeddedSensors){
 document.body.classList.add('sensors-only');if(sensorSection)document.body.classList.add('section-'+sensorSection);
 new ResizeObserver(()=>parent.postMessage({kind:'sensor-height',height:document.body.scrollHeight},location.origin)).observe(document.body);
}
const $=id=>document.getElementById(id), F=(v,n=1)=>Number.isFinite(v)?v.toFixed(n):'—', clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const radians=x=>x*Math.PI/180, wrap=x=>((x%360)+360)%360;
let state=null,lastStateAt=0,connection=null,tableMode='topics',previewTopic='',selectedMapAction=null,mapMeta=null,mapPixels=null,mapCells=null,mapSeq=-1,pointBytes=0;
let route={a:null,b:null},latestGps=null,history=[],lastImuStamp=0;
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
 constructor(canvas,overlay){this.canvas=canvas;this.overlay=overlay;this.points=new Float32Array(0);this.mode='top';this.azimuth=-2.2;this.elevation=.7;this.range=20;this.dirty=false;this.drag=null;this.matrix=null;this.gl=canvas.getContext('webgl',{alpha:true,antialias:false,preserveDrawingBuffer:false});
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
 draw(){const {w,h,dpr}=fitCanvas(this.canvas);fitCanvas(this.overlay);this.matrix=this.projection(w,h);const grid=[],step=gridStep(this.range),extentX=this.mode==='top'?this.range+step:this.range*20*Math.max(1,w/h),extentY=this.mode==='top'?this.range*w/h+step:extentX;for(let k=-Math.ceil(extentY/step)*step;k<=extentY;k+=step)grid.push(-extentX,k,0,-1,extentX,k,0,-1);for(let k=-Math.ceil(extentX/step)*step;k<=extentX;k+=step)grid.push(k,-extentY,0,-1,k,extentY,0,-1);
  if(this.gl){const gl=this.gl;gl.viewport(0,0,w,h);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);gl.uniformMatrix4fv(this.uniforms.Matrix,false,this.matrix);gl.uniform1f(this.uniforms.Size,Number($('pointSize').value)*dpr);gl.uniform1f(this.uniforms.Range,this.range);gl.uniform1f(this.uniforms.Mode,({ring:0,distance:1,height:2})[$('color').value]);
   const bind=buffer=>{gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(this.aPosition);gl.vertexAttribPointer(this.aPosition,3,gl.FLOAT,false,16,0);gl.enableVertexAttribArray(this.aRing);gl.vertexAttribPointer(this.aRing,1,gl.FLOAT,false,16,12);};
   bind(this.gridBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(grid),gl.DYNAMIC_DRAW);gl.uniform1f(this.uniforms.Grid,1);gl.drawArrays(gl.LINES,0,grid.length/4);bind(this.pointBuffer);gl.uniform1f(this.uniforms.Grid,0);gl.drawArrays(gl.POINTS,0,this.points.length/4);
  }else if(this.context){const c=this.context;c.clearRect(0,0,w,h);c.strokeStyle='#444444';c.beginPath();for(let i=0;i<grid.length;i+=8){const a=this.project(...grid.slice(i,i+3),w,h),b=this.project(...grid.slice(i+4,i+7),w,h);if(a&&b){c.moveTo(...a);c.lineTo(...b);}}c.stroke();c.fillStyle='#b5c9b0';for(let i=0;i<this.points.length;i+=4){const p=this.project(this.points[i],this.points[i+1],this.points[i+2],w,h);if(p)c.fillRect(p[0],p[1],2*dpr,2*dpr);}}
  const c=this.overlay.getContext('2d');c.clearRect(0,0,w,h);c.font=`${11*dpr}px Segoe UI`;c.lineWidth=dpr;const center=this.project(0,0,0,w,h);if(center){for(const [x,y,z,label,color] of [[Math.min(4,this.range*.4),0,0,'선수 +X','#f5bb87'],[0,Math.min(4,this.range*.4),0,'좌현 +Y','#b9aaca']]){const p=this.project(x,y,z,w,h);if(p){c.strokeStyle=color;c.fillStyle=color;c.beginPath();c.moveTo(...center);c.lineTo(...p);c.stroke();c.fillText(label,p[0]+4*dpr,p[1]-4*dpr);}}}c.fillStyle='#bbbbbb';c.fillText(`격자 ${step} m · 세로 ±${F(this.range,1)} m`,12*dpr,20*dpr);
 }
 hover(event){if(!this.matrix||!this.points.length)return;const rect=this.canvas.getBoundingClientRect(),w=this.canvas.width,h=this.canvas.height,x=(event.clientX-rect.left)*w/rect.width,y=(event.clientY-rect.top)*h/rect.height;let best=-1,dist=(13*w/rect.width)**2;for(let i=0;i<this.points.length;i+=4){const p=this.project(this.points[i],this.points[i+1],this.points[i+2],w,h);if(!p)continue;const d=(p[0]-x)**2+(p[1]-y)**2;if(d<dist){dist=d;best=i;}}$('pointReadout').hidden=best<0;if(best>=0){const [a,b,c,r]=this.points.slice(best,best+4);$('pointReadout').textContent=`X ${F(a,2)} m · Y ${F(b,2)} m · Z ${F(c,2)} m\n거리 ${F(Math.hypot(a,b,c),2)} m · 방위각 ${F(wrap(Math.atan2(b,a)*180/Math.PI),1)}°\nRing ${r>=0?r:'—'}`;}}
}
const cloud=new CloudView($('cloud'),$('cloudOverlay'));
$('topView').onclick=()=>{cloud.mode='top';$('topView').classList.add('active');$('orbitView').classList.remove('active');cloud.drawSoon();};
$('orbitView').onclick=()=>{cloud.mode='orbit';$('orbitView').classList.add('active');$('topView').classList.remove('active');cloud.drawSoon();};
$('resetView').onclick=()=>{cloud.range=selectedRange('cloud');cloud.azimuth=-2.2;cloud.elevation=.7;cloud.drawSoon();};
$('range').onchange=()=>{cloud.range=selectedRange('cloud');cloud.drawSoon();drawScan();};for(const id of ['color','pointSize'])$(id).oninput=()=>cloud.drawSoon();

function gridStep(range){const raw=range/5,power=10**Math.floor(Math.log10(raw)),n=raw/power;return (n<=1?1:n<=2?2:n<=5?5:10)*power;}
function selectedRange(kind){const value=$('range').value;return value==='max'?Math.max(1,kind==='scan'?(state?.values?.scan?.range_max_m||100):(state?.limits?.max_range_m||100)):Number(value);}
function drawScan(){
 const canvas=$('scan'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),data=state?.values?.scan,R=selectedRange('scan'),s=Math.min(w,h)/2/R,cx=w/2,cy=h/2,step=gridStep(R),edge=Math.hypot(w/2,h/2)/s;
 c.clearRect(0,0,w,h);c.strokeStyle='#444444';c.fillStyle='#aaaaaa';c.font=`${10*dpr}px Segoe UI`;
 c.beginPath();for(let x=cx% (step*s);x<=w;x+=step*s){c.moveTo(x,0);c.lineTo(x,h);}for(let y=cy% (step*s);y<=h;y+=step*s){c.moveTo(0,y);c.lineTo(w,y);}c.stroke();
 c.strokeStyle='#44444488';for(let r=step;r<=edge;r+=step){c.beginPath();c.arc(cx,cy,r*s,0,2*Math.PI);c.stroke();if(cy-r*s>12*dpr)c.fillText(`${F(r,r<5?1:0)} m`,cx+4*dpr,cy-r*s-3*dpr);}
 c.strokeStyle='#777777';c.beginPath();c.moveTo(cx,0);c.lineTo(cx,h);c.moveTo(0,cy);c.lineTo(w,cy);c.stroke();
 c.fillText('선수 · 0°',cx+4*dpr,14*dpr);c.fillText('좌현 90°',6*dpr,cy-5*dpr);c.fillText('우현 270°',w-60*dpr,cy-5*dpr);c.fillText('선미 · 케이블',cx-28*dpr,h-8*dpr);
 c.fillStyle='#b5c9b0';for(const [angle,distance] of data?.polar_deg_m||[]){if(!Number.isFinite(distance)||distance<0||distance>(data.range_max_m||Infinity))continue;const a=radians(angle);c.fillRect(cx-Math.sin(a)*distance*s,cy-Math.cos(a)*distance*s,2*dpr,2*dpr);}
 c.fillStyle='#eeeeee';c.fillRect(cx-2*dpr,cy-2*dpr,4*dpr,4*dpr);
}
function northHeading(){const v=state?.values?.vectornav,h=v?.deg_360?.yaw;return v?.age_s<2&&Number.isFinite(h)?wrap(h):null;}
function compassDirection(h){return ['북','북동','동','남동','남','남서','서','북서'][Math.round(wrap(h)/45)%8];}
function drawAttitude(){
 const canvas=$('attitude'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),heading=northHeading(),r=Math.max(10*dpr,Math.min(w/2-38*dpr,h/2-25*dpr)),x=w/2,y=h/2;
 c.clearRect(0,0,w,h);canvas.dataset.headingDeg=heading===null?'':String(heading);canvas.setAttribute('aria-label',heading===null?'북쪽 기준 방위각 수신 대기':`북쪽 기준 ${F(heading,1)}도 ${compassDirection(heading)}`);
 c.fillStyle='#0e1923';c.beginPath();c.arc(x,y,r,0,2*Math.PI);c.fill();c.strokeStyle='#777777';c.lineWidth=1.5*dpr;c.stroke();
 for(let deg=0;deg<360;deg+=10){const a=radians(deg),major=deg%30===0,inner=r-(major?11:5)*dpr;c.strokeStyle=deg===0?'#d5a779':major?'#999999':'#555555';c.lineWidth=dpr;c.beginPath();c.moveTo(x+Math.sin(a)*inner,y-Math.cos(a)*inner);c.lineTo(x+Math.sin(a)*r,y-Math.cos(a)*r);c.stroke();if(deg%90!==0&&major&&r>=65*dpr){c.fillStyle='#999999';c.font=`${9*dpr}px Segoe UI`;c.textAlign='center';c.textBaseline='middle';c.fillText(String(deg),x+Math.sin(a)*(r-22*dpr),y-Math.cos(a)*(r-22*dpr));}}
 c.textAlign='center';c.textBaseline='middle';c.font=`bold ${12*dpr}px Malgun Gothic`;for(const [deg,text] of [[0,'북 N'],[90,'동 E'],[180,'남 S'],[270,'서 W']]){const a=radians(deg);c.fillStyle=deg===0?'#d5a779':'#bbbbbb';c.fillText(text,x+Math.sin(a)*(r+15*dpr),y-Math.cos(a)*(r+15*dpr));}
 if(heading!==null){c.save();c.translate(x,y);c.rotate(radians(heading));c.fillStyle='#f1dc6c';c.strokeStyle='#fff0ad';c.lineWidth=dpr;c.beginPath();c.moveTo(0,-r*.67);c.lineTo(r*.17,-r*.23);c.lineTo(0,-r*.32);c.lineTo(-r*.17,-r*.23);c.closePath();c.fill();c.stroke();c.strokeStyle='#f1dc6c99';c.beginPath();c.moveTo(0,-r*.32);c.lineTo(0,r*.23);c.stroke();c.restore();}
 c.textAlign='left';c.textBaseline='alphabetic';
}
function drawTrend(){const canvas=$('imuTrend'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d');c.clearRect(0,0,w,h);if(history.length<2)return;const start=history.at(-1).t-30;for(let k=0;k<3;k++){c.strokeStyle=colors[k];c.lineWidth=dpr;c.beginPath();let active=false,last=null;for(const row of history){if(row.t<start)continue;const v=row.v[k];if(!Number.isFinite(v)){active=false;last=null;continue;}const x=(row.t-start)/30*w,y=h*(.5-v/360);if(!active||last!==null&&Math.abs(v-last)>180)c.moveTo(x,y);else c.lineTo(x,y);last=v;active=true;}c.stroke();}}
function drawGps(){const canvas=$('gpsTrack'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d'),track=latestGps?.track_enu_m||[];c.clearRect(0,0,w,h);if(!track.length){c.fillStyle='#aaaaaa';c.font=`${12*dpr}px Segoe UI`;c.fillText('유효한 GPS 위치를 기다리고 있습니다.',14*dpr,h/2);return;}const xs=track.map(p=>p[0]),ys=track.map(p=>p[1]),minX=Math.min(0,...xs),maxX=Math.max(0,...xs),minY=Math.min(0,...ys),maxY=Math.max(0,...ys),span=Math.max(10,maxX-minX,maxY-minY),s=Math.min(w*.8,h*.7)/span,cx=w/2-(minX+maxX)*s/2,cy=h/2+(minY+maxY)*s/2;c.strokeStyle='#444444';c.beginPath();c.moveTo(0,cy);c.lineTo(w,cy);c.moveTo(cx,0);c.lineTo(cx,h);c.stroke();c.strokeStyle='#b5c9b0';c.lineWidth=2*dpr;c.beginPath();track.forEach((p,i)=>i?c.lineTo(cx+p[0]*s,cy-p[1]*s):c.moveTo(cx+p[0]*s,cy-p[1]*s));c.stroke();const p=track.at(-1);c.fillStyle='#eeeeee';c.beginPath();c.arc(cx+p[0]*s,cy-p[1]*s,3*dpr,0,Math.PI*2);c.fill();c.fillStyle='#aaaaaa';c.font=`${10*dpr}px Segoe UI`;c.fillText(`동쪽 → · 북쪽 ↑ · 표시 폭 약 ${F(w/s,0)} m`,12*dpr,h-10*dpr);}

function updateSensors(){if(!state)return;const values=state.values||{},imu=values.imu,cloudData=values.cloud,scan=values.scan,gps=values.gps;
 $('nodeCount').textContent=state.nodes.length;$('topicCount').textContent=`${state.topics.filter(t=>t.age_s!==null&&t.age_s<2).length} / ${state.topics.length}`;$('pointCount').textContent=cloudData?`${cloudData.display_points.toLocaleString()} / ${cloudData.input_points.toLocaleString()}`:'—';const topic=state.topics.find(t=>t.name===cloudData?.topic);$('ringCount').textContent=cloudData?`${cloudData.rings.length||'—'} 층 · ${F(topic?.hz,1)} Hz`:'—';
 ageText('cloudAge',cloudData);ageText('imuAge',imu);ageText('scanAge',scan);ageText('gpsAge',gps);$('cloudEmpty').hidden=Boolean(cloudData&&cloud.points.length);$('cloudFrame').textContent=`${cloudData?.topic||'/velodyne_points'} · ${cloudData?.frame_id||'센서 좌표'} · +X 선수 / +Y 좌현 · m`;$('decodeInfo').textContent=cloudData?`변환 ${F(cloudData.decode_ms,1)} ms · ${state.limits.cloud_hz} FPS 상한`:'';
 $('imuFrame').textContent=`${imu?.topic||'/vectornav/imu'} · ${imu?.frame_id||'센서 프레임'}`;
 for(const k of ['roll','pitch'])$(k).textContent=angleValue(imu?.orientation,k);const heading=northHeading();$('yaw').textContent=heading===null?'—':F(wrap(Math.round(heading*10)/10),1)+'°';$('heading').textContent=heading===null?'대기':compassDirection(heading);$('gyro').textContent=vectorText(imu?.angular_velocity_deg_s);$('accel').textContent=vectorText(imu?.acceleration_m_s2);
 if(imu?.orientation&&imu.stamp_s!==lastImuStamp){lastImuStamp=imu.stamp_s;history.push({t:imu.stamp_s,v:[imu.orientation.deg_signed.roll,imu.orientation.deg_signed.pitch,heading===null?NaN:wrap(heading+180)-180]});if(history.length>400)history.shift();}
 $('scanFrame').textContent=`${scan?.frame_id||'센서 좌표'} · 선수 0° / 좌현 90° · 거리 m`;$('nearest').textContent=scan?.nearest?`최근접 ${F(scan.nearest.distance_m,2)} m / ${F(scan.nearest.azimuth_deg,1)}°`:'최근접 —';
 latestGps=gps;$('fixStatus').textContent=gps?.valid_fix?'위치 고정됨':gps?'위치 고정 대기 · fix '+gps.fix_status:'GPS 메시지 대기';$('openMap').disabled=!gps?.valid_fix;$('latlon').textContent=gps?.valid_fix?`${F(gps.latitude_deg,7)} / ${F(gps.longitude_deg,7)}`:'—';$('altitude').textContent=gps?.valid_fix?`${F(gps.altitude_m,2)} m / ${F(values.gps_velocity?.speed_m_s,2)} m/s`:'—';$('enu').textContent=gps?.valid_fix?`${F(gps.east_m,2)} / ${F(gps.north_m,2)} m`:'—';
 const a=values.actuators;$('actuatorMode').textContent=a?.mode_name||a?.mode||'대기';const command=(side)=>{const pwm=a?.['logical_'+side+'_us']??a?.[side],percent=a?.[side+'_percent']??(Number.isFinite(pwm)?(pwm-1500)/4:null);return Number.isFinite(percent)?`${F(percent,1)}% · ${pwm} µs`:'토픽 데이터에서 확인';};$('actuatorLeft').textContent=command('left');$('actuatorRight').textContent=command('right');$('actuatorDetail').textContent=a?'상태 수신 중':'수신 대기';
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

let tankView={x:null,y:null,zoom:1},tankPointer=null,tankDragged=false;
function cancelTankDrag(){
 if(tankPointer&&$('tankMap').hasPointerCapture(tankPointer.id))$('tankMap').releasePointerCapture(tankPointer.id);
 tankPointer=null;tankDragged=false;$('tankMap').classList.remove('dragging');
}
window.addEventListener('blur',cancelTankDrag);
let trailWasProvisional=false;
let tankTransform=null,tankTrail=[],lastTankStamp=null,tankSession=null,mapStream=null,serverInstance=null;
function acceptState(next){
 const instance=next.instance_id||null;
 if(instance&&instance!==serverInstance){mapSeq=-1;mapStream=null;mapMeta=null;mapPixels=null;mapCells=null;cloud.sequence=-1;lastTankStamp=null;}
 serverInstance=instance;state=next;
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
 if((!session?.fixed_map||session.localization_ready===true)&&actual?.age_s<1)return actual;
 const p=report?.display_pose;
 if(!p||report.map_id!==session?.session_id||report.age_s>=1||!Number.isFinite(p.stamp_s))return null;
 const age=(state.server_time_s||report.stamp_s)-p.stamp_s;
 if(age<-.1||(!p.held&&age>=1)||![p.x,p.y,p.yaw_deg].every(Number.isFinite))return null;
 return {position_m:{x:p.x,y:p.y},orientation:{deg_signed:{yaw:p.yaw_deg}},age_s:age,stamp_s:p.stamp_s,provisional:true,held:!!p.held,manual_locked:!!p.manual_locked,display_fallback:!!p.display_fallback};
}
function drawTank(){const canvas=$('tankMap'),{w,h,dpr}=fitCanvas(canvas),c=canvas.getContext('2d');c.clearRect(0,0,w,h);const meta=state?.values?.map,pose=tankDisplayPose();ageText('mapAge',meta);$('mapEmpty').hidden=Boolean(mapPixels&&mapMeta);if(!mapPixels||!mapMeta)return;const m=mapMeta,r=m.resolution_m,theta=radians(m.origin_yaw_deg||0),co=Math.cos(theta),si=Math.sin(theta),corners=[[0,0],[m.width*r,0],[0,m.height*r],[m.width*r,m.height*r]].map(([x,y])=>[m.origin_x_m+co*x-si*y,m.origin_y_m+si*x+co*y]),xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),base=Math.min((w-24*dpr)/(maxX-minX||1),(h-24*dpr)/(maxY-minY||1)),s=base*tankView.zoom,viewX=tankView.x??(minX+maxX)/2,viewY=tankView.y??(minY+maxY)/2,cx=w/2-viewX*s,cy=h/2+viewY*s;const screen=(x,y)=>[cx+x*s,cy-y*s];tankTransform={s,cx,cy,dpr,w,h,viewX,viewY};$("tankZoomLabel").textContent=Math.round(tankView.zoom*100)+"%";$("tankZoomOut").disabled=tankView.zoom<=1;
 c.save();c.translate(...screen(m.origin_x_m,m.origin_y_m));c.rotate(-theta);c.scale(s,-s);c.imageSmoothingEnabled=false;c.drawImage(mapPixels,0,0,m.width*r,m.height*r);c.restore();
 const session=state?.values?.tank_session?.session_id;if(session&&session!==tankSession){tankSession=session;tankView={x:null,y:null,zoom:1};tankTrail=[];route={a:null,b:null};routeLabel();}const saved=state?.values?.tank_route;if(saved?.session_id===session&&!route.a&&!route.b){route={a:saved.a,b:saved.b};routeLabel();}
 const reportTrail=state?.values?.tank_localization_candidates;
 if(pose&&reportTrail?.map_id===session&&reportTrail?.age_s<1&&reportTrail.display_pose&&reportTrail.display_trail_m?.length){
  // Replace the entire odometry-derived trace after every candidate correction.
  tankTrail=(reportTrail.display_trail_m||[]).filter(p=>p.length===2&&p.every(Number.isFinite)).map(p=>[...p]);
  if(!pose.provisional){const from=reportTrail.display_pose,to=pose.position_m,a=radians(pose.orientation.deg_signed.yaw-from.yaw_deg),co=Math.cos(a),si=Math.sin(a);tankTrail=tankTrail.map(([x,y])=>[to.x+co*(x-from.x)-si*(y-from.y),to.y+si*(x-from.x)+co*(y-from.y)]);}
  const point=pose.position_m;tankTrail.push([point.x,point.y]);lastTankStamp=pose.stamp_s;trailWasProvisional=true;
 }else if(pose?.age_s<1&&pose.stamp_s!==lastTankStamp){
  if(trailWasProvisional){tankTrail=[];trailWasProvisional=false;}
  lastTankStamp=pose.stamp_s;const p=pose.position_m,previous=tankTrail.at(-1);if(!previous||Math.hypot(p.x-previous[0],p.y-previous[1])>.02)tankTrail.push([p.x,p.y]);if(tankTrail.length>2000)tankTrail.shift();
 }else if(!pose&&trailWasProvisional){tankTrail=[];}

 c.strokeStyle='#7696a6';c.lineWidth=1.5*dpr;c.beginPath();tankTrail.forEach((p,i)=>i?c.lineTo(...screen(...p)):c.moveTo(...screen(...p)));c.stroke();
 if(route.a&&route.b){c.strokeStyle='#e6b36a';c.lineWidth=2*dpr;c.setLineDash([5*dpr,5*dpr]);c.beginPath();c.moveTo(...screen(route.a.x,route.a.y));c.lineTo(...screen(route.b.x,route.b.y));c.stroke();c.setLineDash([]);}
 const predicted=state?.values?.tank_prediction,planned=state?.values?.tank_status;
 for(const [points,color,dashed] of [[planned?.age_s<2?planned.path_m:null,'#b5c9b0',false],[predicted?.age_s<1?predicted.points_m:null,'#de90ba',true]]){if(!points?.length)continue;c.strokeStyle=color;c.lineWidth=2*dpr;c.setLineDash(dashed?[4*dpr,4*dpr]:[]);c.beginPath();points.forEach((p,i)=>i?c.lineTo(...screen(...p)):c.moveTo(...screen(...p)));c.stroke();c.setLineDash([]);}
 for(const [key,point] of Object.entries(route)){if(!point)continue;const p=screen(point.x,point.y);c.fillStyle=key==='a'?'#1f918b':'#bd843c';c.beginPath();c.arc(...p,8*dpr,0,2*Math.PI);c.fill();c.font=`bold ${10*dpr}px Segoe UI`;c.fillStyle='#fff';c.textAlign='center';c.fillText(key.toUpperCase(),p[0],p[1]+3.5*dpr);c.textAlign='left';}
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
new ResizeObserver(()=>drawTrial()).observe(document.querySelector('.trial-charts'));
function routeLabel(){const point=p=>p?`${F(p.x,2)}, ${F(p.y,2)} m`:'미지정';$('routeInfo').textContent=`A ${point(route.a)} / B ${point(route.b)}`;$('saveRoute').disabled=!(route.a&&route.b);}
for(const [id,mode] of [['setA','a'],['setB','b']])$(id).onclick=()=>{selectedMapAction=mode;$('tankStatus').textContent=`${mode.toUpperCase()} 지점을 클릭하세요.`;};
$('tankMap').onclick=e=>{if(tankDragged){tankDragged=false;return;}if(!tankTransform||!selectedMapAction||!onKnownTankMap(e.clientX,e.clientY))return;const rect=$('tankMap').getBoundingClientRect(),{s,cx,cy,dpr}=tankTransform;route[selectedMapAction]={x:((e.clientX-rect.left)*dpr-cx)/s,y:(cy-(e.clientY-rect.top)*dpr)/s};routeLabel();drawTank();if(selectedMapAction==='a')$('setB').click();};
function zoomTank(factor,clientX,clientY){
 if(!tankTransform)return;const rect=$('tankMap').getBoundingClientRect(),t=tankTransform;
 const px=clientX===undefined?t.w/2:(clientX-rect.left)*t.dpr,py=clientY===undefined?t.h/2:(clientY-rect.top)*t.dpr;
 const worldX=(px-t.cx)/t.s,worldY=(t.cy-py)/t.s,next=clamp(tankView.zoom*factor,1,32),scale=t.s*next/tankView.zoom;
 if(next===1){tankView={x:null,y:null,zoom:1};drawTank();return;}tankView.x=worldX-(px-t.w/2)/scale;tankView.y=worldY+(py-t.h/2)/scale;tankView.zoom=next;drawTank();
}
function onKnownTankMap(clientX,clientY){
 if(!tankTransform||!mapMeta||!mapCells)return false;
 const rect=$('tankMap').getBoundingClientRect(),t=tankTransform,m=mapMeta;
 const x=((clientX-rect.left)*t.dpr-t.cx)/t.s-m.origin_x_m,y=(t.cy-(clientY-rect.top)*t.dpr)/t.s-m.origin_y_m;
 const a=radians(m.origin_yaw_deg||0),col=Math.floor((Math.cos(a)*x+Math.sin(a)*y)/m.resolution_m),row=Math.floor((-Math.sin(a)*x+Math.cos(a)*y)/m.resolution_m);
 return col>=0&&row>=0&&col<m.width&&row<m.height&&mapCells[row*m.width+col]>=0;
}
$('tankMap').addEventListener('wheel',e=>{if(!onKnownTankMap(e.clientX,e.clientY))return;e.preventDefault();zoomTank(Math.exp(-clamp(e.deltaY,-200,200)*.003),e.clientX,e.clientY);},{passive:false});
$('tankMap').addEventListener('pointerdown',e=>{if(e.button!==0)return;tankDragged=false;if(!onKnownTankMap(e.clientX,e.clientY))return;tankPointer={id:e.pointerId,x:e.clientX,y:e.clientY,viewX:tankTransform.viewX,viewY:tankTransform.viewY,scale:tankTransform.s/tankTransform.dpr};$('tankMap').setPointerCapture(e.pointerId);});
$('tankMap').addEventListener('pointermove',e=>{if(!tankPointer||tankPointer.id!==e.pointerId)return;const dx=e.clientX-tankPointer.x,dy=e.clientY-tankPointer.y;if(Math.hypot(dx,dy)>4)tankDragged=true;if(!tankDragged)return;tankView.x=tankPointer.viewX-dx/tankPointer.scale;tankView.y=tankPointer.viewY+dy/tankPointer.scale;$('tankMap').classList.add('dragging');drawTank();});
function endTankDrag(e){if(tankPointer?.id===e.pointerId){tankPointer=null;$('tankMap').classList.remove('dragging');}}
$('tankMap').addEventListener('pointerup',endTankDrag);$('tankMap').addEventListener('pointercancel',endTankDrag);
$('tankZoomIn').onclick=()=>{zoomTank(1.4);};$('tankZoomOut').onclick=()=>{zoomTank(1/1.4);};
$('tankFit').onclick=()=>{tankView={x:null,y:null,zoom:1};drawTank();};
$('tankExpand').onclick=()=>{const panel=document.querySelector('.tank-panel');panel.classList.toggle('expanded');$('tankExpand').textContent=panel.classList.contains('expanded')?'화면 복귀':'크게 보기';drawTank();};
document.addEventListener('keydown',e=>{if(e.key==='Escape'){cancelTankDrag();selectedMapAction=null;for(const key of ['setA','setB'])$(key).classList.remove('active');document.querySelector('.tank-panel').classList.remove('expanded');$('tankExpand').textContent='크게 보기';drawTank();}});
new ResizeObserver(()=>drawTank()).observe($('tankMap'));
$('candidateButtons').onclick=e=>{const b=e.target.closest('button[data-candidate-id]');if(!b)return;selectedCandidateId=b.dataset.candidateId;drawTank();};
$('applyCandidate').onclick=async()=>{const selected=liveTankCandidates().find(p=>p.candidate_id===selectedCandidateId);if(!selected)return;const result=await post('/api/initial_pose',{map_id:candidateMapId,candidate_id:selected.candidate_id});if(result)$('tankStatus').textContent=result.message;};
$('autoLocate').onclick=async()=>{const result=await post('/api/localize',{});if(result)$('globalMatchInfo').textContent=result.message;};
$('saveRoute').onclick=async()=>{const result=await post('/api/route',{a:route.a,b:route.b});if(result)$('tankStatus').textContent=result.message;};
new ResizeObserver(()=>{if(graphOnly)return;drawAttitude();drawTrend();drawScan();if(!sensorOnly)drawGps();if(!embeddedSensors)drawTank();}).observe(document.querySelector('.main-grid'));

function renderRuntime(){
 const runtime=state?.values?.runtime,fresh=performance.now()-lastStateAt<3000&&runtime?.age_s<7;
 const labels={sensors:'센서',mapping:'매핑',trial:'왕복'},texts={running:'실행 중',stopped:'정지',restarting:'재시작 중',unknown:'확인 중'};
 for(const [key,label] of Object.entries(labels)){
  const status=fresh&&texts[runtime.services?.[key]]?runtime.services[key]:'unknown',el=$('runtime-'+key);
  let text=texts[status];
  const trial=state?.values?.tank_status;
  if(key==='trial'&&status==='running'&&trial?.age_s<3&&trial.state==='WAITING')text='실행 중 · 대기';
  el.className='runtime-item '+status;el.textContent=label+' · '+text;
  el.title=key==='trial'&&status==='running'&&trial?.age_s<3?trial.reason||'왕복 노드 실행 중':'젯슨 프로세스 기준 · 약 3초마다 갱신';
 }
}
setInterval(renderRuntime,1000);
async function stateLoop(){try{const response=await fetch(embeddedSensors?'/api/state?sensors=1':'/api/state',{cache:'no-store',signal:AbortSignal.timeout(6000)});if(response.ok){acceptState(await response.json());lastStateAt=performance.now();if(!previewTopic)previewTopic=state.selected_topic;updateSensors();if(!embeddedSensors)drawTrial();renderRuntime();if(sensorOnly)parent.postMessage({kind:'sensor-graph',state:{nodes:state.nodes,topics:state.topics,preview:state.preview,selected_topic:state.selected_topic,graph_updated_s:state.graph_updated_s}},location.origin);}}catch(_){}setTimeout(stateLoop,document.hidden?1200:250);}
async function cloudLoop(){try{if(!$('pauseCloud').checked&&!document.hidden){const response=await fetch('/api/cloud?after='+(cloud.sequence??-1),{cache:'no-store',signal:AbortSignal.timeout(6000)});if(response.status===200){const buffer=await response.arrayBuffer();pointBytes+=buffer.byteLength;cloud.update(buffer);}}}catch(_){}setTimeout(cloudLoop,document.hidden?1500:200);}
let lastNetworkBytes=null,lastNetworkAt=performance.now();async function connectionLoop(){try{const response=await fetch('/api/connection',{cache:'no-store'});connection=await response.json();const now=performance.now(),kb=lastNetworkBytes===null?0:Math.max(0,connection.received_bytes-lastNetworkBytes)/Math.max(1,now-lastNetworkAt);lastNetworkBytes=connection.received_bytes;lastNetworkAt=now;$('network').textContent=`${F(kb,1)} KB/s`;}catch(_){}const fresh=performance.now()-lastStateAt<3000&&state;const live=fresh&&connection?.sensors_running!==false;$('connection').className='badge '+(live?'live':'waiting');$('connection').textContent=fresh?(live?'연결됨':'브링업 대기'):connection?.status||'연결 대기';if(!fresh&&lastStateAt){for(const id of ['cloudAge','imuAge','scanAge','gpsAge']){$(id).textContent='연결 지연';$(id).className='age stale';}}setTimeout(connectionLoop,1000);}
async function mapLoop(){try{const meta=state?.values?.map,instance=serverInstance,newStream=meta?.stream_id!==mapStream;if(meta&&(newStream||meta.sequence!==mapSeq)){const response=await fetch('/api/map?after='+(newStream?-1:mapSeq),{cache:'no-store',signal:AbortSignal.timeout(6000)});if(response.status===200){const bytes=await response.arrayBuffer(),v=new DataView(bytes);if(bytes.byteLength>=16&&v.getUint32(0,true)===0x50414d4b){const seq=v.getUint32(4,true),w=v.getUint32(8,true),h=v.getUint32(12,true);if(instance===serverInstance&&seq===meta.sequence&&w*h<=8_000_000&&bytes.byteLength===16+w*h){const cells=new Int8Array(bytes,16),image=new ImageData(w,h);for(let i=0;i<cells.length;i++){const value=cells[i],color=value<0?[29,48,61]:value>50?[17,31,43]:[187,203,207];image.data.set([...color,255],i*4);}mapPixels=document.createElement('canvas');mapPixels.width=w;mapPixels.height=h;mapPixels.getContext('2d').putImageData(image,0,0);mapCells=cells;mapMeta=meta;mapSeq=seq;mapStream=meta.stream_id;drawTank();}}}}}catch(_){}setTimeout(mapLoop,1000);}
if(graphOnly){window.addEventListener('message',e=>{if(e.source===parent&&e.origin===location.origin&&e.data?.kind==='sensor-graph'){state=e.data.state;if(!previewTopic)previewTopic=state.selected_topic;renderGraph();renderPreview();}});}else{stateLoop();cloudLoop();connectionLoop();if(!embeddedSensors)mapLoop();}
