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
  function cloudToBody(p,t) {
    const a=t.yaw_deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a);
    return [t.x_m+c*p[0]-s*p[1],t.y_m+s*p[0]+c*p[1],p[2]];
  }
  function topdownProject(p,v,w,h) {
    const scale=Math.min(w,h)/(2*v.range);
    return [w/2-(p[1]-v.y)*scale,h/2-(p[0]-v.x)*scale];
  }
  function topdownUnproject(p,v,w,h) {
    const scale=Math.min(w,h)/(2*v.range);
    return [v.x+(h/2-p[1])/scale,v.y+(w/2-p[0])/scale,0];
  }
  function gridStep(range) {return range<=3?.5:range<=7?1:range<=12?2:5;}
  class TopDownView {
    constructor(owner) {
      this.owner=owner;this.el=document.getElementById('radar');
      this.view={x:.8,y:0,range:5};this.showAvoidance=false;
      this.el.addEventListener('pointerdown',e=>{
        if(e.button!==0)return;
        const rect=this.el.getBoundingClientRect(),scale=Math.min(rect.width,rect.height)/(2*this.view.range);
        this.drag={id:e.pointerId,x:e.clientX,y:e.clientY,view:{...this.view},scale};
        this.el.setPointerCapture(e.pointerId);this.el.classList.add('dragging');});
      this.el.addEventListener('pointermove',e=>{
        if(!this.drag||this.drag.id!==e.pointerId)return;
        this.view.x=this.drag.view.x+(e.clientY-this.drag.y)/this.drag.scale;
        this.view.y=this.drag.view.y+(e.clientX-this.drag.x)/this.drag.scale;
        this.limit();owner.schedule();});
      const release=()=>{this.drag=null;this.el.classList.remove('dragging');};
      this.el.addEventListener('pointerup',release);this.el.addEventListener('pointercancel',release);
      this.el.addEventListener('wheel',e=>{
        e.preventDefault();const b=this.el.getBoundingClientRect();
        this.zoom(Math.exp(clamp(e.deltaY,-150,150)*.002),[e.clientX-b.left,e.clientY-b.top]);},{passive:false});
      this.el.addEventListener('keydown',e=>{
        if(!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','+','=','-','0'].includes(e.key))return;
        e.preventDefault();
        if(e.key==='0')this.reset();
        else if(['+','=','-'].includes(e.key))this.zoom(e.key==='-'?1.2:1/1.2);
        else {
          const step=this.view.range*.16;
          if(e.key==='ArrowUp')this.view.x+=step;if(e.key==='ArrowDown')this.view.x-=step;
          if(e.key==='ArrowLeft')this.view.y+=step;if(e.key==='ArrowRight')this.view.y-=step;
          this.limit();owner.schedule();
        }});
      document.getElementById('topdownMinus').onclick=()=>this.zoom(1.3);
      document.getElementById('topdownPlus').onclick=()=>this.zoom(1/1.3);
      document.getElementById('topdownHome').onclick=()=>this.reset();
      document.getElementById('topdownFit').onclick=()=>this.fit();
      const overlay=document.getElementById('topdownAvoidance');
      overlay.onclick=()=>{this.showAvoidance=!this.showAvoidance;
        overlay.setAttribute('aria-pressed',String(this.showAvoidance));
        overlay.textContent='회피 입력 '+(this.showAvoidance?'ON':'OFF');
        document.getElementById('topdownAvoidanceLegend').hidden=!this.showAvoidance;owner.schedule();};
      const range=document.getElementById('radarRange');
      range.onclick=()=>{this.view.range=this.view.range<4?5:this.view.range<8?10:3;owner.schedule();};
    }
    limit(){this.view.x=clamp(this.view.x,-30,30);this.view.y=clamp(this.view.y,-30,30);}
    reset(){this.view={x:.8,y:0,range:5};this.owner.schedule();}
    zoom(factor,anchor) {
      const r=this.el.getBoundingClientRect(),p=anchor||[r.width/2,r.height/2];
      const before=topdownUnproject(p,this.view,r.width,r.height);
      this.view.range=clamp(this.view.range*factor,.8,20);
      const after=topdownUnproject(p,this.view,r.width,r.height);
      this.view.x+=before[0]-after[0];this.view.y+=before[1]-after[1];this.limit();this.owner.schedule();
    }
    fit() {
      const o=this.owner,nav=o.navigation();
      if(!o.points.length)return;
      const points=o.points.map(p=>nav.aligned?cloudToBody(p,nav.transform):p);
      const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);xs.push(0);ys.push(0);
      const loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);
      const r=this.el.getBoundingClientRect(),m=Math.min(r.width,r.height);
      this.view={x:(loX+hiX)/2,y:(loY+hiY)/2,
        range:clamp(Math.max((hiX-loX)*m/r.height,(hiY-loY)*m/r.width)*.58,1,20)};
      o.schedule();
    }
    draw() {
      const o=this.owner,rect=this.el.getBoundingClientRect(),w=Math.max(1,rect.width),h=Math.max(1,rect.height);
      const dpr=Math.min(devicePixelRatio||1,2);this.el.width=Math.round(w*dpr);this.el.height=Math.round(h*dpr);
      const ctx=this.el.getContext('2d');ctx.scale(dpr,dpr);
      ctx.fillStyle='#0b1720';ctx.fillRect(0,0,w,h);
      const v=this.view,scale=Math.min(w,h)/(2*v.range),step=gridStep(v.range),pr=p=>topdownProject(p,v,w,h);
      const a=topdownUnproject([0,0],v,w,h),b=topdownUnproject([w,h],v,w,h);
      ctx.font='9px Consolas, "Malgun Gothic", sans-serif';
      const segment=(p,q,color,width=1)=>{ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();};
      // Isotropic metre grid: no perspective, interpolation or invented wall surfaces.
      for(let x=Math.ceil(b[0]/step)*step;x<=a[0]+step*.01;x+=step){
        const y=pr([x,0,0])[1],major=Math.abs(x/5-Math.round(x/5))<1e-6;
        segment([0,y],[w,y],major?'#244353':'#152b38',major?1.2:.65);
        if(y>42&&y<h-32){ctx.fillStyle='#647e90';ctx.fillText((x>0?'+':'')+x.toFixed(step<1?1:0)+' m',7,y-4);}}
      for(let y=Math.ceil(b[1]/step)*step;y<=a[1]+step*.01;y+=step){
        const x=pr([0,y,0])[0],major=Math.abs(y/5-Math.round(y/5))<1e-6;
        segment([x,0],[x,h],major?'#244353':'#152b38',major?1.2:.65);
        if(x>40&&x<w-35){ctx.fillStyle='#647e90';ctx.fillText((y>0?'+':'')+y.toFixed(step<1?1:0),x+4,h-7);}}
      const nav=o.navigation(),elapsed=(performance.now()-o.received)/1000;
      const age=Number.isFinite(o.data?.age_s)?o.data.age_s+elapsed:null;
      const stale=age===null||age>1.5||Boolean(o.data?.error);
      const points=o.points.map(p=>({p,body:nav.aligned?cloudToBody(p,nav.transform):p}));
      ctx.globalAlpha=stale?.25:.85;
      const visible=points.filter(({body})=>{const q=pr(body);return q[0]>=0&&q[0]<=w&&q[1]>=0&&q[1]<=h;});
      // Thin height-coloured returns retain the actual cloud's density and shape.
      for(const {p,body} of visible){const q=pr(body);
        ctx.fillStyle=`hsl(${215-clamp((p[2]+1.5)/3,0,1)*185} 85% 65%)`;
        const size=clamp(scale*.03,1.1,2.3);ctx.fillRect(q[0]-size/2,q[1]-size/2,size,size);}
      ctx.globalAlpha=1;
      const m=nav;
      if(this.showAvoidance&&nav.aligned&&m.fresh){
        ctx.fillStyle='#ffb270';ctx.globalAlpha=.95;
        for(const p of m.points){const q=pr(p);if(q[0]<0||q[0]>w||q[1]<0||q[1]>h)continue;
          ctx.fillRect(q[0]-1.7,q[1]-1.7,3.4,3.4);}
        ctx.globalAlpha=1;
      }
      const origin=pr([0,0,0]);
      ctx.setLineDash([3,6]);segment([origin[0],0],[origin[0],h],'#3e657952');
      segment([0,origin[1]],[w,origin[1]],'#3e657952');ctx.setLineDash([]);
      if(nav.aligned)drawNavigation(ctx,p=>pr(cloudToBody(p,nav.transform)),nav);
      else {ctx.strokeStyle='#a4bfd0';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(...origin,4,0,2*Math.PI);ctx.stroke();}
      // A fixed screen scale is computed from the same metre projection as the cloud.
      const bar=step*scale,bx=14,by=h-29;
      ctx.fillStyle='#0b1720df';ctx.fillRect(8,h-46,Math.max(95,bar+24),31);
      segment([bx,by],[bx+bar,by],'#b0d1df',2);segment([bx,by-3],[bx,by+3],'#b0d1df');segment([bx+bar,by-3],[bx+bar,by+3],'#b0d1df');
      ctx.fillStyle='#b0d1df';ctx.fillText(step.toFixed(step<1?1:0)+' m',bx,by-7);
      ctx.fillStyle='#7e9eaf';ctx.textAlign='right';ctx.fillText(nav.aligned?'선수 +X ↑ · 좌현 +Y ←':'LiDAR +X ↑ · +Y ←',w-12,h-22);ctx.textAlign='left';
      const status=document.getElementById('topdownStatus');
      status.textContent=o.data?.error?'점군 형식 확인 필요':age===null?'점군 수신 대기':
        (stale?'지연 · 마지막 점군':'LIVE')+' · '+o.points.length.toLocaleString()+'점 · '+age.toFixed(1)+'초';
      status.classList.toggle('bad',stale);status.classList.toggle('live',!stale);
      document.getElementById('topdownFrame').textContent=(nav.aligned?'선체 기준':'LiDAR 기준')+' · 격자 '+step+' m · 화면 '+(w/scale).toFixed(1)+' × '+(h/scale).toFixed(1)+' m';
      document.getElementById('radarRange').textContent='±'+v.range.toFixed(v.range<3?1:0)+' m';
      document.getElementById('topdownNote').textContent='실제 XYZ의 XY 투영 · '+visible.length.toLocaleString()+'점 표시 · 높이 색상'+(this.showAvoidance?' · 주황 회피 입력 겹쳐 보기':' · 회피 입력 겹침 OFF')+
        (!nav.aligned?' · 선체 좌표 정렬 대기':nav.gapFresh?' · 관측 '+nav.gaps.map(g=>'G'+g.index).join(' / '):' · 후보 관측 대기');
    }
  }
  function navigationModel(report,alignment,frame,elapsed=0,model=RadarDisplay.model,alignmentElapsed=0) {
    const matched=alignment?.valid&&alignment.frame_id===frame&&
      ['yaw_deg','x_m','y_m','age_s'].every(k=>Number.isFinite(alignment[k]))&&alignment.age_s+alignmentElapsed<6;
    if(!matched)return {aligned:false,reason:'방향 오버레이: 선체·라이다 좌표 정렬 대기'};
    const d={...report};
    for(const k of ['report_age','scan_age','heading_age'])if(Number.isFinite(d[k]))d[k]+=elapsed;
    if(d.gap_preview?.independent)d.gap_preview={...d.gap_preview,
      age_s:Number.isFinite(d.gap_preview.age_s)?d.gap_preview.age_s+elapsed:null,
      receive_age:Number.isFinite(d.gap_preview.receive_age)?d.gap_preview.receive_age+elapsed:null};
    const m=model(d);
    return {...m,aligned:true,transform:alignment,gaps:m.gapFresh?m.gaps:[],
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
      // A dotted guide and a separate badge keep narrow angular bands visible.
      // The band angles themselves are unchanged; this is never a motion arrow.
      stroke([at(.65,g.centre,.1),at(3.2,g.centre,.1)],g.selected?'#63dec8':'#82b3ff',1.4,[3,5]);
      const labelRadius=3.55+(g.index%2===0?.4:0);
      const q=body(at(labelRadius,g.centre,.12));if(q){
        const tail=body(at(3.2,g.centre,.12));if(tail){ctx.beginPath();ctx.strokeStyle=g.selected?'#63dec8':'#82b3ff';ctx.lineWidth=1;ctx.moveTo(tail[0],tail[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();}
        ctx.fillStyle='#10212bf2';ctx.fillRect(q[0]-12,q[1]-11,26,19);
        ctx.font='bold 12px Consolas';ctx.fillStyle=g.selected?'#a5fff0':'#d1e7ff';
        ctx.fillText('G'+g.index,q[0]-8,q[1]+3);}
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
      document.addEventListener('visibilitychange',()=>{if(!document.hidden)this.schedule();});
      this.topdown=new TopDownView(this);
      this.staleTimer=setInterval(()=>this.schedule(),500);
      this.poll();this.schedule();
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
      document.getElementById('topdownTools').hidden=active;
      document.getElementById('topdownLegend').hidden=active;
      this.schedule();
    }
    async poll() {
      if(document.hidden){this.timer=setTimeout(()=>this.poll(),500);return;}
      const request=new AbortController();this.request=request;
      const timeout=setTimeout(()=>request.abort(),3000);
      try {
        const response=await fetch('/api/pointcloud',{signal:request.signal,cache:'no-store'});
        if(!response.ok)throw Error('HTTP '+response.status);
        const data=await response.json();
        if(this.request===request){this.data=data;this.points=validPoints(data.points);this.received=performance.now();this.schedule();}
      } catch(error) {
        if(this.request===request){this.data={...(this.data||{}),error:'점군 연결 대기 · '+error.message};this.status.textContent=this.data.error;this.status.classList.add('bad');this.schedule();}
      } finally {
        clearTimeout(timeout);
        if(this.request===request)this.timer=setTimeout(()=>this.poll(),500);
      }
    }
    schedule() {
      if(this.queued||document.hidden)return;this.queued=true;
      requestAnimationFrame(()=>{this.queued=false;if(this.active)this.draw();else this.topdown.draw();});
    }
    navigation() {
      return navigationModel(this.report,this.data?.body_from_cloud,this.data?.frame_id,
        (performance.now()-this.reportReceived)/1000,undefined,(performance.now()-this.received)/1000);
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
        (nav.observationOnly?'관측 후보 표시 · ':'2D와 동일한 계산 결과 · ')+(nav.reason||'항법 입력 대기'));
      this.status.textContent=this.data?.error?'점군 형식 확인 필요: '+this.data.error:
        !this.data||age===null?'3D 점군 수신 대기 · GPS와 독립':
        (stale?'수신 지연 · 마지막 점군':'실시간 XYZ')+' · '+this.points.length.toLocaleString()+'점 · '+(age+elapsed).toFixed(1)+'초';
      this.status.classList.toggle('bad',stale&&Boolean(this.data));
      document.getElementById('cloudFrame').textContent='LiDAR 좌표: '+frame+' · 1 m 격자 · 반경 12 m';
      ctx.fillStyle='#819ba9';ctx.fillText('원점 = LiDAR · 높이 Z 색상',10,h-12);
    }
  }
  return {View,TopDownView,project,validPoints,bodyToCloud,cloudToBody,topdownProject,topdownUnproject,gridStep,navigationModel,drawNavigation};
})();
if(typeof document!=='undefined'&&document.getElementById('radar3d'))new Lidar3D.View();
