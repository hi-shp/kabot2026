/* Real PointCloud2 XYZ preview. Display only; never issues mission commands. */
const Lidar3D = (() => {
  const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
  function project(p,view,w,h) {
    const cy=Math.cos(view.yaw),sy=Math.sin(view.yaw),cp=Math.cos(view.pitch),sp=Math.sin(view.pitch);
    const depth=view.distance-(p[0]*cy*cp+p[1]*sy*cp+p[2]*sp);
    if(depth<.2)return null;
    const f=Math.min(w,h)*1.3/depth;
    return [w/2+(-p[0]*sy+p[1]*cy)*f,
      h*.56-(-p[0]*cy*sp-p[1]*sy*sp+p[2]*cp)*f,depth];
  }
  function validPoints(points) {
    return Array.isArray(points)?points.filter(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)):[];
  }
  function bodyToCloud(p,t) {
    const a=t.yaw_deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a),x=p[0]-t.x_m,y=p[1]-t.y_m;
    return [c*x+s*y,-s*x+c*y,p[2]];
  }
  function navigationModel(report,alignment,frame,elapsed=0,model=RadarDisplay.model,alignmentElapsed=0) {
    const matched=alignment?.valid&&alignment.frame_id===frame&&
      ['yaw_deg','x_m','y_m','age_s'].every(k=>Number.isFinite(alignment[k]))&&alignment.age_s+alignmentElapsed<6;
    if(!matched)return {aligned:false,reason:'방향 오버레이: 선체·라이다 좌표 정렬 대기'};
    const d={...report};
    for(const k of ['report_age','scan_age','heading_age'])if(Number.isFinite(d[k]))d[k]+=elapsed;
    const m=model(d);
    return {...m,aligned:true,transform:alignment,gaps:m.fresh&&m.headingFresh?m.gaps:[],
      selected:m.fresh&&m.headingFresh?m.selected:null};
  }
  function drawNavigation(ctx,pr,m) {
    if(!m.aligned)return;
    const body=p=>pr(bodyToCloud(p,m.transform)),at=(r,a,z=.08)=>[r*Math.cos(a*Math.PI/180),r*Math.sin(a*Math.PI/180),z];
    const stroke=(points,color,width=1,dash=[],fill=null)=>{
      const q=points.map(body);if(q.some(p=>!p))return;
      ctx.beginPath();q.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));
      if(fill){ctx.closePath();ctx.fillStyle=fill;ctx.fill();}
      ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
    };
    const arrow=(from,to,color,width=2,dash=[])=>{
      const p=body(from),q=body(to);if(!p||!q)return;
      const dx=q[0]-p[0],dy=q[1]-p[1],len=Math.hypot(dx,dy);if(len<2)return;
      stroke([from,to],color,width,dash);const ux=dx/len,uy=dy/len;
      ctx.fillStyle=color;ctx.beginPath();ctx.moveTo(q[0],q[1]);
      ctx.lineTo(q[0]-ux*8-uy*4,q[1]-uy*8+ux*4);ctx.lineTo(q[0]-ux*8+uy*4,q[1]-uy*8-ux*4);ctx.closePath();ctx.fill();
    };
    for(const g of m.gaps) {
      const count=Math.max(3,Math.ceil(g.width/4));
      const angles=Array.from({length:count+1},(_,i)=>g.right+(g.left-g.right)*i/count);
      stroke([...angles.map(a=>at(3.2,a)),...angles.slice().reverse().map(a=>at(2.7,a))],
        g.selected?'#63dec8':'#82b3ff',g.selected?2:1,[],g.selected?'#63dec84d':'#82b3ff18');
      const q=body(at(3,g.centre));if(q){ctx.font='bold 10px Consolas';ctx.fillStyle=g.selected?'#a5fff0':'#b5d0ed';
        ctx.strokeStyle='#10212b';ctx.lineWidth=3;ctx.strokeText('G'+g.index,q[0],q[1]);ctx.fillText('G'+g.index,q[0],q[1]);}
    }
    const origin=[0,0,.12];
    if(Number.isFinite(m.target))arrow(origin,at(4,m.target,.12),'#82b3ff',2,[5,4]);
    if(Number.isFinite(m.command))arrow(origin,at(2.7,m.command,.16),'#63dec8',3);
    if(Number.isFinite(m.actual)&&Math.abs(m.actual)>.5){
      const steps=Math.max(4,Math.ceil(Math.abs(m.actual)/5));
      stroke(Array.from({length:steps+1},(_,i)=>at(1.1,-m.actual*(1-i/steps),.2)),'#c59bff',2);
      arrow(at(1.1,-Math.sign(m.actual)*Math.min(12,Math.abs(m.actual)),.2),at(1.1,0,.2),'#c59bff',2);
    }
    stroke([[.45,0,.12],[-.3,.23,.12],[-.3,-.23,.12],[.45,0,.12]],'#e6f0f8',1.5,[],'#507c9d');
    arrow(origin,[1.3,0,.12],'#f0f6fa',2);
  }
  class View {
    constructor() {
      this.el=document.getElementById('radar3d');
      this.status=document.getElementById('pointcloudStatus');
      this.active=false;this.points=[];this.data=null;this.received=0;this.queued=false;
      this.report=typeof latest==='undefined'?{}:latest||{};this.reportReceived=performance.now();
      window.addEventListener('mission-view-update',e=>{this.report=e.detail;this.reportReceived=performance.now();this.schedule();});
      this.defaults={yaw:-2.15,pitch:.72,distance:15};this.view={...this.defaults};
      this.buttons=[document.getElementById('lidar2d'),document.getElementById('lidar3d')];
      this.buttons[0].onclick=()=>this.setMode(false);this.buttons[1].onclick=()=>this.setMode(true);
      document.getElementById('cloudReset').onclick=()=>{this.view={...this.defaults};this.schedule();};
      this.el.addEventListener('pointerdown',e=>{if(e.button!==0)return;
        this.drag={id:e.pointerId,x:e.clientX,y:e.clientY,yaw:this.view.yaw,pitch:this.view.pitch};
        this.el.setPointerCapture(e.pointerId);this.el.classList.add('dragging');});
      this.el.addEventListener('pointermove',e=>{if(!this.drag||this.drag.id!==e.pointerId)return;
        this.view.yaw=this.drag.yaw-(e.clientX-this.drag.x)*.008;
        this.view.pitch=clamp(this.drag.pitch+(e.clientY-this.drag.y)*.008,.08,1.5);this.schedule();});
      const release=()=>{this.drag=null;this.el.classList.remove('dragging');};
      this.el.addEventListener('pointerup',release);this.el.addEventListener('pointercancel',release);
      this.el.addEventListener('wheel',e=>{e.preventDefault();
        this.view.distance=clamp(this.view.distance*Math.exp(clamp(e.deltaY,-150,150)*.0018),4,55);this.schedule();},{passive:false});
      this.el.addEventListener('keydown',e=>{
        if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','0'].includes(e.key))return;e.preventDefault();
        if(e.key==='0')this.view={...this.defaults};
        else if(e.key==='ArrowLeft'||e.key==='ArrowRight')this.view.yaw+=e.key==='ArrowLeft'?-.12:.12;
        else if(e.key==='ArrowUp'||e.key==='ArrowDown')this.view.pitch=clamp(this.view.pitch+(e.key==='ArrowUp'?.08:-.08),.08,1.5);
        else this.view.distance=clamp(this.view.distance*(e.key==='-'?1.1:.9),4,55);
        this.schedule();});
      window.addEventListener('resize',()=>this.schedule());
      document.addEventListener('visibilitychange',()=>{if(this.active&&!document.hidden)this.schedule();});
      this.staleTimer=setInterval(()=>{if(this.active)this.schedule();},500);
    }
    setMode(active) {
      if(active===this.active)return;
      this.active=active;
      this.buttons.forEach((b,i)=>b.setAttribute('aria-pressed',String(Boolean(i)===active)));
      document.getElementById('radar').hidden=active;
      this.el.hidden=!active;
      document.getElementById('cloudTools').hidden=!active;
      document.getElementById('radar2dLegend').hidden=active;
      document.getElementById('cloudLegend').hidden=!active;
      document.getElementById('radarRange').hidden=active;
      if(active){this.points=[];this.data=null;this.status.textContent='3D 점군 수신 대기';this.poll();this.schedule();}
      else {clearTimeout(this.timer);this.request?.abort();window.dispatchEvent(new Event('resize'));}
    }
    async poll() {
      if(!this.active)return;
      const request=new AbortController();this.request=request;
      const timeout=setTimeout(()=>request.abort(),3000);
      try {
        const response=await fetch('/api/pointcloud',{signal:request.signal,cache:'no-store'});
        if(!response.ok)throw Error('HTTP '+response.status);
        const data=await response.json();
        if(this.active&&this.request===request){this.data=data;this.points=validPoints(data.points);this.received=performance.now();this.schedule();}
      } catch(error) {
        if(this.active&&this.request===request){this.status.textContent='3D 점군 연결 대기 · '+error.message;this.status.classList.add('bad');}
      } finally {
        clearTimeout(timeout);
        if(this.active&&this.request===request)this.timer=setTimeout(()=>this.poll(),500);
      }
    }
    schedule() {
      if(!this.active||this.queued)return;this.queued=true;
      requestAnimationFrame(()=>{this.queued=false;if(this.active)this.draw();});
    }
    draw() {
      const rect=this.el.getBoundingClientRect(),w=Math.max(1,rect.width),h=Math.max(1,rect.height),dpr=Math.min(devicePixelRatio||1,2);
      this.el.width=Math.round(w*dpr);this.el.height=Math.round(h*dpr);
      const ctx=this.el.getContext('2d');ctx.scale(dpr,dpr);ctx.fillStyle='#101c24';ctx.fillRect(0,0,w,h);
      const pr=p=>project(p,this.view,w,h);
      const segment=(a,b,color,width=1)=>{const p=pr(a),q=pr(b);if(!p||!q)return;
        ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();};
      for(let i=-5;i<=5;i++) {segment([i,-5,0],[i,5,0],i===0?'#41606d':'#253e4b');segment([-5,i,0],[5,i,0],i===0?'#41606d':'#253e4b');}
      const age=this.data?.age_s;
      const elapsed=(performance.now()-this.received)/1000;
      const stale=!Number.isFinite(age)||age+elapsed>1.5||Boolean(this.data?.error);
      ctx.globalAlpha=stale?.25:1;
      const points=this.points.map(p=>({p,q:pr(p)})).filter(o=>o.q&&o.q[0]>=-4&&o.q[0]<=w+4&&o.q[1]>=-4&&o.q[1]<=h+4).sort((a,b)=>b.q[2]-a.q[2]);
      for(const {p,q} of points){ctx.fillStyle=`hsl(${215-clamp((p[2]+1.5)/3,0,1)*185} 90% 66%)`;
        const size=clamp(24/q[2],1.4,4);ctx.fillRect(q[0]-size/2,q[1]-size/2,size,size);}
      ctx.globalAlpha=1;ctx.font='10px Consolas, "Malgun Gothic", sans-serif';
      for(const [p,color,label] of [[[2,0,0],'#ff807d','X'],[[0,2,0],'#63dec8','Y'],[[0,0,2],'#82b3ff','Z ↑']]) {
        segment([0,0,0],p,color,2);const q=pr(p);if(q){ctx.fillStyle=color;ctx.fillText(label,q[0]+5,q[1]-4);}}
      const origin=pr([0,0,0]);if(origin){ctx.fillStyle='#f0f6fa';ctx.beginPath();ctx.arc(origin[0],origin[1],3,0,Math.PI*2);ctx.fill();}
      const frame=this.data?.frame_id||'—';
      const reportElapsed=(performance.now()-this.reportReceived)/1000;
      const nav=navigationModel(this.report,this.data?.body_from_cloud,frame,reportElapsed,undefined,elapsed);
      drawNavigation(ctx,pr,nav);
      const description=document.getElementById('cloudNavigation');
      description.textContent=(this.report?.demo?'DEMO · ':'')+(!nav.aligned?nav.reason:
        Number.isFinite(nav.command)?'2D와 동일한 계산 결과 · '+(nav.selected?'선택 G'+nav.selected.index:'이동 명령'):
        '2D와 동일한 계산 결과 · '+(nav.reason||'항법 입력 대기'));
      this.status.textContent=this.data?.error?'점군 형식 확인 필요: '+this.data.error:
        !this.data||age===null?'3D 점군 수신 대기 · GPS와 독립':
        (stale?'수신 지연 · 마지막 점군':'실시간 XYZ')+' · '+this.points.length.toLocaleString()+'점 · '+(age+elapsed).toFixed(1)+'초';
      this.status.classList.toggle('bad',stale&&Boolean(this.data));
      document.getElementById('cloudFrame').textContent='LiDAR 좌표: '+frame+' · 1 m 격자 · 반경 12 m';
      ctx.fillStyle='#819ba9';ctx.fillText('원점 = LiDAR · 높이 Z 색상',10,h-12);
    }
  }
  return {View,project,validPoints,bodyToCloud,navigationModel,drawNavigation};
})();
if(typeof document!=='undefined'&&document.getElementById('radar3d'))new Lidar3D.View();
