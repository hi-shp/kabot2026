/* Mapping and hopping editor on the existing SatelliteMap canvas. */
window.DockingMapUI=(()=>{
  const $=id=>document.getElementById(id),finite=Number.isFinite;
  let data={},view=null,inputMode=null,corners=[],draftHop=null,httpBusy=false,pending=null,loadedHopKey='',targetPickMap=null;
  let commandEpoch=0;
  const note=text=>{$('dockMessage').textContent=text;};
  const numeric=id=>{const text=$(id).value.trim();return text===''?null:Number(text);};
  const validLL=p=>Array.isArray(p)&&p.length===2&&p.every(finite)&&Math.abs(p[0])<=85.05112878&&Math.abs(p[1])<=180;
  const draftKey='kaboat.docking.hopDraft.v1';
  function rememberDraft(){try{if(draftHop)window.localStorage?.setItem(draftKey,JSON.stringify(draftHop));else window.localStorage?.removeItem(draftKey);}catch(_){}}
  try{const saved=JSON.parse(window.localStorage?.getItem(draftKey)||'null');if(validLL(saved))draftHop=saved;}catch(_){}
  function clickToLatLon(map,x,y){
    const world=256*2**map.zoom;
    return SatelliteMap.unproject([map.center[0]+(x-map.width/2)/world,map.center[1]+(y-map.height/2)/world]);
  }
  function bindMap(map){view=map;if(!map.center&&validLL(data.origin_lla?.slice(0,2)))map.center=SatelliteMap.project(data.origin_lla);}
  function mapClick(map,x,y){
    if(!inputMode)return false;
    if(inputMode==='gps_target'){
      if(httpBusy||pending)return true;
      if(!data.connected||!data.controls_enabled||!data.status?.gps_map_loaded||data.status.gps_map_name!==targetPickMap){inputMode=null;note('맵이 바뀌었습니다. 목표 지점 지정을 다시 누르세요.');render();return true;}
      const ll=clickToLatLon(map,x,y);inputMode=null;
      command('set_gps_target',{map_name:targetPickMap,latitude:ll[0],longitude:ll[1]});
      return true;
    }
    const ll=clickToLatLon(map,x,y);
    if(inputMode==='roi'){
      if(corners.length===2)corners=[];
      corners.push(ll);note(corners.length===1?'구조물 영역의 반대쪽 모서리를 클릭하세요.':'영역을 확인하고 높이 범위와 함께 영역 적용을 누르세요.');
    }else{draftHop=ll;rememberDraft();inputMode=null;note(data.status?.reference_id?'임시 후보를 선택했습니다. 기준 맵과 높이를 확인하고 확정하세요.':'임시 후보를 선택했습니다. 기준 맵 생성·저장 전에는 운항 지점으로 확정되지 않습니다.');}
    map.schedule();render();return true;
  }
  function referenceFields(){const s=data.status||{};return {reference_id:s.reference_id,reference_version:s.reference_version};}
  async function command(action,extra={}){
    const urgent=['auto_stop','auto_emergency','map_dry_run_stop'].includes(action);
    if(!urgent&&(httpBusy||pending))return;
    const epoch=++commandEpoch;
    if(urgent)pending=null;
    httpBusy=true;render();
    try{
      const response=await fetch('/api/docking/command',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({action,...extra})});
      const result=await response.json();
      if(epoch!==commandEpoch)return;
      if(!response.ok||!result.success)throw Error(result.error||result.message||'매핑 요청 실패');
      pending={id:result.request_id,action,at:Date.now()};note('명령을 접수했습니다. 매핑 노드의 처리 결과를 기다립니다.');
    }catch(error){note(error.message);}finally{httpBusy=false;render();}
  }
  $('mapOutputPrepare').onclick=()=>command('map_output_prepare');
  $('mapDriveStart').onclick=()=>command('map_drive_start');
  $('mapDryRunStart').onclick=()=>command('map_dry_run_start');
  $('mapDryRunStop').onclick=()=>command('map_dry_run_stop');
  $('dockAutoStart').onclick=()=>command('auto_start');
  $('dockAutoStop').onclick=()=>command('auto_stop');
  $('dockAutoEmergency').onclick=()=>command('auto_emergency');
  function arm(mode){inputMode=mode;if(mode==='roi')corners=[];
    note(mode==='roi'?'지도에서 구조물 영역의 두 모서리를 클릭하세요.':'지도에서 호핑 지점을 클릭하세요. 드래그는 지도 이동입니다.');view?.schedule();}
  $('gpsTargetPick').onclick=()=>{
    if(!data.status?.gps_map_loaded||httpBusy||pending)return;
    inputMode=inputMode==='gps_target'?null:'gps_target';targetPickMap=data.status.gps_map_name;
    note(inputMode?'왼쪽 위성지도에서 목표 지점을 클릭하세요. 클릭한 위치를 이 맵에 저장합니다.':'목표 지점 지정을 취소했습니다.');render();
  };
  $('dockStart').onclick=()=>command(['PAUSED','PAUSED_RC_LOST'].includes(data.status?.initial_mapping_state)?'resume':'start');
  for(const [id,action]of [['dockStop','stop'],['dockSave','save_gps_map'],['dockLoad','load_gps_map']])$(id).onclick=()=>command(action);
  $('dockReset').onclick=()=>{if(confirm('현재 메모리 맵과 호핑 편집값을 초기화할까요? 저장 파일은 유지됩니다.'))command('reset');};
  $('dockSelectRoi').onclick=()=>arm('roi');
  $('dockApplyRoi').onclick=()=>{
    const zmin=numeric('dockZMin'),zmax=numeric('dockZMax'),yaw=numeric('dockRefYaw');
    if(corners.length!==2||![zmin,zmax,yaw].every(finite)||zmin>=zmax){note('영역 두 모서리와 유효한 ENU 높이 범위를 입력하세요.');return;}
    command('select_roi',{corners,z_min:zmin,z_max:zmax,yaw_enu_deg:yaw});
  };
  $('dockHopAdd').onclick=()=>arm('hop');$('dockHopEdit').onclick=()=>arm('hop');
  $('dockCancel').onclick=()=>{inputMode=null;corners=[];draftHop=null;rememberDraft();view?.schedule();note('지도 선택을 취소했습니다.');render();};
  $('dockHopApply').onclick=()=>{
    if(!data.status?.reference_id){note('임시 후보만 선택된 상태입니다. 유효한 기준 맵을 저장하거나 불러온 뒤 확정하세요.');return;}
    if(!data.controls_enabled||!data.status?.rc_ready){note('확정에는 매핑 연결과 RC 수동 상태가 필요합니다.');return;}
    const ll=draftHop||data.hopping?.latlon,height=numeric('dockHopZ'),radius=numeric('dockHopRadius'),hold=numeric('dockHopHold');
    const heading=numeric('dockHopHeading');
    if(!validLL(ll)||![height,radius,hold].every(finite)||(heading!==null&&!finite(heading))){note('지도 위치와 ENU 높이·반경·유지 시간을 입력하세요.');return;}
    command('set_hopping',{...referenceFields(),latitude:ll[0],longitude:ll[1],height_enu_m:height,
      name:$('dockHopName').value.trim(),radius_m:radius,hold_seconds:hold,heading_dock_deg:heading});
  };
  $('dockHopDelete').onclick=()=>{draftHop=null;inputMode=null;command('delete_hopping',referenceFields());};
  $('dockHopSave').onclick=()=>command('save_hopping',referenceFields());
  $('dockFit').onclick=()=>{
    if(!view)return;const points=[...(data.map_points||[])];
    if(validLL(data.hopping?.latlon))points.push(data.hopping.latlon);
    if(validLL(data.origin_lla?.slice(0,2)))points.push(data.origin_lla.slice(0,2));
    if(data.status?.gps_target)points.push([data.status.gps_target.latitude,data.status.gps_target.longitude]);
    if(!points.length){note('표시할 도킹 맵을 기다립니다.');return;}
    Object.assign(view,SatelliteMap.fit(points,view.width,view.height));view.schedule();
  };
  for(const id of ['dockShowMap','dockShowScan','dockShowPath','dockShowCompare','dockOpacity'])$(id).oninput=()=>view?.schedule();
  function render(){
    const s=data.status||{},busy=httpBusy||Boolean(pending),enabled=data.controls_enabled&&!busy,rc=enabled&&s.rc_ready;
    const nav=s.map_navigation||{},navPlan=nav.plan||{},navGoal=navPlan.goal;
    const phases={GPS_APPROACH:'GPS 접근',LIDAR_DOCKING:'라이다 도킹',STOPPED:'정지',IDLE:'대기'};
    $('mapDryRunStart').disabled=!enabled||nav.active||s.initial_mapping_state==='ACTIVE'||s.autonomy?.active||!s.gps_map_loaded||!s.gps_target;
    $('mapDryRunStop').disabled=!data.controls_enabled;
    const drive=nav.drive||{};
    $('mapOutputPrepare').disabled=!enabled||drive.prepared||s.autonomy?.active||s.initial_mapping_state==='ACTIVE'||!s.gps_map_loaded||!s.gps_target;
    $('mapDriveStart').disabled=!enabled||!nav.active||!nav.drive_ready||drive.requested;
    $('mapOutputPrepare').textContent=drive.prepared?'운항 출력 준비됨':'맵 도킹 운항 준비';
    const metrics=nav.registration||{};
    $('mapDryRunStatus').textContent=`${phases[nav.phase]||nav.phase||'대기'} · ${nav.state||'IDLE'} · ${drive.requested?'실제 주행 요청됨':drive.prepared?'운항 준비 · 출발 대기':'실제 추진 출력 차단'} · `+
      (navGoal?`목표 ${navGoal.distance_m.toFixed(2)} m / ${navGoal.bearing_deg.toFixed(1)}° · `:'상대 목표 대기 · ')+
      `계산 PWM ${(nav.calculated_pwm||[1500,1500]).join(' / ')} · ${nav.phase==='LIDAR_DOCKING'?'GPS 제외':'실시간 GPS 접근'}`+
      (metrics.rmse!=null?` · 정합 RMSE ${metrics.rmse.toFixed(3)} m`:'')+
      ` · 연속 정합 ${nav.confirmation_count||0}/${nav.confirmation_required||3}`+
      (navPlan.pose_source==='LIDAR_PREDICTING'?` · 정합 공백 추정 ${Number(nav.prediction_uncertainty_m||0).toFixed(2)} m`:'')+
      (nav.registration_busy?' · 정합 계산 중':'')+
      (nav.reason?` · ${nav.reason}`:'')+
      (drive.prepared&&!drive.requested?` · 시작 대기: ${nav.drive_blocker||'주행 시작 가능'}`:'')+
      (drive.owned?` · arm ${drive.armed?'확인':'대기'}`:'')+
      (drive.owned&&!drive.requested&&drive.stop_confirmed===false?' · 정지 응답 미확인':'');
    const frozen=Boolean(s.reference_id),active=s.initial_mapping_state==='ACTIVE';
    const labels={WAIT_EXTRINSICS:'센서 장착 위치·방향 보정 미확인',WAIT_DECLINATION:'자기편각 미설정',WAIT_HEADING_REFERENCE:'방위 기준 미설정',WAIT_VALID_GPS:'유효한 GPS 위치 대기',WAIT_SYNC:'GPS·IMU·라이다 시간 일치 대기',WAIT_RC:'RC 수동 신호 대기'};
    const blockers=[...(s.mapping_blockers||[])];if(s.extrinsics_verified===false&&!blockers.includes('WAIT_EXTRINSICS'))blockers.push('WAIT_EXTRINSICS');if(labels[s.processing_state]&&!blockers.includes(s.processing_state))blockers.push(s.processing_state);
    $('dockError').textContent=[...blockers.map(k=>labels[k]||k),...(s.error?['처리 오류: '+s.error]:[])].join(' · ')||(!data.map_points?.length?'누적 맵이 아직 없습니다. 현재 스캔 XYZ에서 형상을 확인하세요.':'');
    if(s.relative_reference)$('dockError').textContent='저장 맵 기준 좌표 · GPS로 접근 후 라이다 정합 연속 확인 시 전환'+($('dockError').textContent?' · '+$('dockError').textContent:'');
    const auto=s.autonomy||{},goal=auto.relative_goal;
    $('dockAutoStart').disabled=!enabled||!auto.auto_ready||auto.active||!frozen||!data.hopping||s.hopping_dirty||active;
    $('dockAutoStop').disabled=!data.controls_enabled;
    $('dockAutoEmergency').disabled=!data.controls_enabled;
    $('dockAutoStatus').textContent=`${auto.state||'IDLE'} · ${auto.enabled?'실제 출력 설정':'Dry-run / 출력 꺼짐'}`;
    $('dockAutoDetail').textContent=(goal?`목표 ${goal.distance_m.toFixed(2)} m · 선수 기준 ${goal.bearing_deg.toFixed(1)}° · `:'상대 목표 대기 · ')+
      `유지 ${(auto.hold_elapsed_s||0).toFixed(1)} s · PWM ${(auto.pwm||[1500,1500]).join(' / ')} · `+
      `전방 ${auto.forward_clear?'관측됨':'미확인/차단'} · 후방 ${auto.rear_clear?'관측됨':'미확인/차단'} · `+
      (auto.world_anchor_age_s>2?'지도 GPS 위치 갱신 지연 · ':'')+
      (auto.reason||auto.registration_reason||'');
    $('dockStatus').textContent=(s.simulation?'가상 검증 · ':'')+(data.connected?'연결됨':'연결 대기 / 지연')+' · '+(s.phase||'INITIAL_MAPPING')+' · '+(s.state||'WAIT_MAPPING_NODE');
    $('dockStart').disabled=!rc||frozen||!['IDLE','PAUSED','PAUSED_RC_LOST'].includes(s.initial_mapping_state);
    $('dockStart').textContent=['PAUSED','PAUSED_RC_LOST'].includes(s.initial_mapping_state)?'맵 생성 재개':'맵 생성';
    $('dockStop').disabled=!enabled||frozen||s.initial_mapping_state==='IDLE';
    $('dockSave').disabled=!enabled||active||!data.map_points_xyz?.length;
    $('dockLoad').disabled=!enabled||active||!s.gps_map_available;$('dockReset').disabled=!enabled||active;
    $('dockSelectRoi').disabled=!rc||active||frozen||!data.origin_lla;
    $('dockApplyRoi').disabled=!rc||active||frozen||corners.length!==2;
    $('dockHopAdd').disabled=busy;$('dockHopAdd').textContent='지도에서 임시 후보 선택';
    $('dockHopApply').disabled=!rc||!frozen||!validLL(draftHop||data.hopping?.latlon);$('dockHopApply').textContent='기준 맵에 확정 적용';
    $('dockHopEdit').disabled=busy||!(draftHop||data.hopping);
    $('dockHopDelete').disabled=!rc||!data.hopping;
    $('dockHopSave').disabled=!rc||!frozen||!s.hopping_dirty;
    for(const id of ['dockStart','dockSave','dockLoad','dockReset','dockSelectRoi','dockApplyRoi','dockHopApply','dockHopDelete','dockHopSave','dockAutoStart'])if(nav.active)$(id).disabled=true;
    if(inputMode==='gps_target'&&(!s.gps_map_loaded||s.gps_map_name!==targetPickMap))inputMode=null;
    $('gpsTargetPick').disabled=!enabled||active||nav.active||!s.gps_map_loaded;
    $('gpsTargetPick').textContent=inputMode==='gps_target'?'목표 지정 취소':'목표 지점 지정';
    const target=s.gps_target;
    $('gpsTargetCoords').textContent=inputMode==='gps_target'?'왼쪽 위성지도에서 목표 지점을 클릭하세요.':target?`목표: 위도 ${target.latitude.toFixed(7)} · 경도 ${target.longitude.toFixed(7)} · 맵에 저장됨`:'목표 미지정 · 맵을 불러온 뒤 목표 지점을 지정하세요.';
    $('dockFit').disabled=!s.gps_map_loaded||!data.map_points?.length;
    const hop=data.hopping,key=JSON.stringify([s.reference_id,hop?.point_dock,hop?.name,hop?.radius_m,hop?.hold_seconds,hop?.heading_dock_deg]);
    if(key!==loadedHopKey){
      const changedReference=s.reference_id!==render.referenceId;
      if(changedReference){corners=[];if(inputMode==='roi')inputMode=null;render.referenceId=s.reference_id;}
      if(hop&&!draftHop){
        $('dockHopName').value=hop.name;$('dockHopZ').value=hop.height_enu_m.toFixed(4);
        $('dockHopRadius').value=hop.radius_m;$('dockHopHold').value=hop.hold_seconds;$('dockHopHeading').value=hop.heading_dock_deg??'';
        draftHop=null;
      }else if(changedReference&&!draftHop){$('dockHopZ').value=s.hopping_config?.plane_height_enu_m??'';}
      loadedHopKey=key;
    }
    const marker=draftHop||hop?.latlon;
    $('dockHopCoords').textContent=validLL(marker)?`${draftHop?'임시 후보 · ':''}위도 ${marker[0].toFixed(7)} · 경도 ${marker[1].toFixed(7)}${draftHop?' · 아직 운항 지점으로 확정되지 않음':''}`:'지도를 클릭해 호핑 위치를 지정하세요.';
    $('dockSummary').textContent=`2D: ${s.gps_map_loaded?'불러온 맵 + 현재 위치':'현재 위치만'} · 맵 ${data.map_points_xyz?.length||0}점 · ${s.gps_map_available?'저장 맵 있음':'저장 맵 없음'}`;
  }
  async function poll(){
    try{
      const response=await fetch('/api/docking');if(!response.ok)throw Error('도킹 지도 응답 오류');
      data=await response.json();const result=data.status?.last_gui_result;
      if(pending&&result?.request_id===pending.id){
        if(result.success&&['set_hopping','delete_hopping'].includes(pending.action)){draftHop=null;rememberDraft();inputMode=null;}
        if(result.success&&pending.action==='load_gps_map'){inputMode=null;if(view)view.fitAll?.();}
        note((result.success?'완료: ':'실패: ')+result.message);pending=null;
      }
      if(pending&&Date.now()-pending.at>10000){note('처리 결과가 지연됩니다. 현재 상태를 확인하세요. 자동 재전송하지 않습니다.');pending=null;}
      window.dispatchEvent?.(new CustomEvent('docking-map-update',{detail:data}));
      if(view){bindMap(view);view.schedule();}render();
    }catch(error){data={...data,connected:false,controls_enabled:false};render();}
    setTimeout(poll,500);
  }
  function draw(map,ctx,w,h){
    if(map.mapOnly){
      if(!data.status?.gps_map_loaded)return;
      ctx.save();ctx.fillStyle='#4c9fff';
      for(const ll of data.map_points||[]){if(!validLL(ll))continue;const [x,y]=map.screen(ll);if(x>=0&&x<=w&&y>=0&&y<=h)ctx.fillRect(x-2,y-2,4,4);}
      const target=data.status.gps_target;
      if(target){const [x,y]=map.screen([target.latitude,target.longitude]);ctx.beginPath();ctx.arc(x,y,7,0,2*Math.PI);ctx.fillStyle='#ffda4b';ctx.fill();ctx.strokeStyle='#17242c';ctx.lineWidth=2;ctx.stroke();ctx.font='bold 12px sans-serif';ctx.fillText('목표',x+11,y-9);}
      ctx.restore();return;
    }
    ctx.save();const dot=(ll,color,size=2)=>{if(!validLL(ll))return;const [x,y]=map.screen(ll);
      if(x<0||x>w||y<0||y>h)return;ctx.fillStyle=color;ctx.fillRect(x-size/2,y-size/2,size,size);};
    const path=(points,color)=>{if(!points?.length)return;ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;
      points.forEach((ll,i)=>{const p=map.screen(ll);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.stroke();};
    if($('dockShowMap').checked){ctx.globalAlpha=Number($('dockOpacity').value);for(const p of data.map_points||[])dot(p,data.status?.map_quality==='CONFIRMED'?'#4c9fff':'#9ba6ad',3);ctx.globalAlpha=1;}
    if($('dockShowScan').checked)for(const p of data.current_scan||[])dot(p,'#55e39b',2.5);
    if($('dockShowPath').checked)path(data.rc_path,'#c68bff');
    const label=(ll,name,color)=>{if(!validLL(ll))return;const [x,y]=map.screen(ll);dot(ll,color,8);
      ctx.font='12px "Malgun Gothic",sans-serif';ctx.lineWidth=3;ctx.strokeStyle='#142128';ctx.strokeText(name,x+8,y-7);ctx.fillStyle=color;ctx.fillText(name,x+8,y-7);};
    if($('dockShowCompare').checked){
      for(const [key,name,color]of [['gps_position','GPS','#ffb270'],['corrected_position','정합','#ff6666']]){
        const pose=data[key];if(!pose)continue;label(pose.latlon,pose.age_s>2?name+'(최근)':name,color);
        const [x,y]=map.screen(pose.latlon),a=-pose.yaw_enu_deg*Math.PI/180;
        ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+18*Math.cos(a),y+18*Math.sin(a));ctx.stroke();
      }
    }
    if(data.origin_lla)label(data.origin_lla.slice(0,2),'원점','#e4edf3');
    const hop=data.hopping;
    if(validLL(hop?.latlon)){
      const [x,y]=map.screen(hop.latlon),mpp=2*Math.PI*6378137*Math.cos(hop.latlon[0]*Math.PI/180)/(256*2**map.zoom);
      ctx.beginPath();ctx.arc(x,y,hop.radius_m/mpp,0,2*Math.PI);ctx.fillStyle='rgba(255,218,75,.15)';ctx.fill();ctx.strokeStyle='#ffda4b';ctx.stroke();
      ctx.beginPath();ctx.arc(x,y,6,0,2*Math.PI);ctx.fillStyle='#ffda4b';ctx.fill();label(hop.latlon,hop.name,'#ffda4b');
    }
    if(corners.length){for(const p of corners)dot(p,'#a4ddff',7);
      if(corners.length===2){const a=map.screen(corners[0]),b=map.screen(corners[1]);ctx.setLineDash([5,4]);ctx.strokeStyle='#a4ddff';ctx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);ctx.setLineDash([]);}}
    if(draftHop)label(draftHop,'임시 후보 (미확정)','#ffe98e');
    ctx.restore();
  }
  render();poll();return {bindMap,mapClick,draw,clickToLatLon,navigationActive:()=>Boolean(data.status?.map_navigation?.active||data.status?.map_navigation?.drive?.owned),navigationStatus:()=>data.status?.map_navigation,stop:()=>command('map_dry_run_stop'),pointsForMap:()=>data.status?.gps_map_loaded?(data.map_points||[]):[]};
})();

/* Display-only XYZ: a 2D LaserScan is never used to invent height. */
window.DockingLivePreview=(()=>{
  const valid=p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite);
  function choosePreview(raw){return raw||{points:[],stale:true};}
  function project(point,range,width,height,azimuth=0,elevation=.6){
    const [x,y,z]=point,ca=Math.cos(azimuth),sa=Math.sin(azimuth),ce=Math.cos(elevation),se=Math.sin(elevation);
    const scale=(Math.min(width,height)-56)/(2*range);
    const forward=ca*x+sa*y,left=-sa*x+ca*y;
    return [width/2-left*scale,height/2-(forward*se+z*ce)*scale,-forward*ce+z*se];
  }
  const canvas=document.getElementById('dockLiveCanvas');
  if(!canvas||typeof canvas.getContext!=='function')return {project,choosePreview};
  const ctx=canvas.getContext('2d'),$=id=>document.getElementById(id),status=$('dockLiveStatus'),detail=$('dockLiveDetail'),mode=$('dockLiveRange');
  let raw={points:[],stale:true},map={},yaw=0,pitch=.6,zoom=1,drag=null,loadedMapKey=null;
  function draw(){
    const width=Math.max(280,Math.round(canvas.getBoundingClientRect().width||600)),height=420;
    canvas.width=width;canvas.height=height;
    const mapped=mode.value==='map',source=mapped?map:raw;
    const points=(mapped?map.map_points_xyz:raw.points)||[],validPoints=points.filter(valid);
    const center=[0,0,0];let range=mapped ? .65 : Math.max(.65,Number(raw.max_range_m||2)*1.25);
    if(mapped&&validPoints.length){
      const low=[Infinity,Infinity,Infinity],high=[-Infinity,-Infinity,-Infinity];
      validPoints.forEach(p=>p.forEach((v,i)=>{low[i]=Math.min(low[i],v);high[i]=Math.max(high[i],v);}));
      center.forEach((_,i)=>center[i]=(low[i]+high[i])/2);
      range=Math.max(.65,...high.map((v,i)=>(v-low[i])*.65));
    }else if(validPoints.length){range=Math.max(range,...validPoints.map(p=>Math.abs(p[2])*1.1));}
    range/=zoom;
    const screen=p=>project(p.map((v,i)=>v-center[i]),range,width,height,yaw,pitch);
    ctx.fillStyle='#0b151e';ctx.fillRect(0,0,width,height);ctx.font='12px sans-serif';
    const line=(a,b,color)=>{const p=screen(a),q=screen(b);ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();};
    const step=10**Math.floor(Math.log10(Math.max(.1,range/3))),extent=step*Math.ceil(range/step);
    for(let n=-extent;n<=extent;n+=step){line([center[0]+n,center[1]-extent,0],[center[0]+n,center[1]+extent,0],'#233847');line([center[0]-extent,center[1]+n,0],[center[0]+extent,center[1]+n,0],'#233847');}
    const axis=Math.max(.25,range*.45);
    for(const [v,name,color]of [[[axis,0,0],'X 선수 / 0°','#ff7979'],[[0,axis,0],'Y 좌현','#74e39c'],[[0,0,axis],'Z 위','#77b5ff']]){line([0,0,0],v,color);const p=screen(v);ctx.fillStyle=color;ctx.fillText(name,p[0]+5,p[1]-5);}
    const stale=mapped?!map.connected:raw.stale;
    const ordered=validPoints.map(p=>({p,s:screen(p)})).sort((a,b)=>a.s[2]-b.s[2]);
    for(const {p,s}of ordered){ctx.fillStyle=stale?'#697680':`hsl(${210-160*(Math.atan(p[2])*2/Math.PI+1)/2} 80% 65%)`;ctx.fillRect(s[0]-2,s[1]-2,4,4);}
    if(mapped&&map.rc_path_xyz?.length){for(let i=1;i<map.rc_path_xyz.length;i++)line(map.rc_path_xyz[i-1],map.rc_path_xyz[i],'#c68bff');}
    function marker(p,name,color){const q=screen(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(q[0],q[1],4,0,2*Math.PI);ctx.fill();ctx.fillText(name,q[0]+7,q[1]+14);}
    if(!mapped){marker([0,0,0],'LiDAR','#ffffff');marker([-.4,0,0],'GPS −0.40m','#ffbd78');line([0,0,0],[-.4,0,0],'#ffbd78');}
    else if(map.corrected_position?.relative_xyz){marker(map.corrected_position.relative_xyz,'현재 선체','#ffffff');}
    ctx.fillStyle='#9cb1c0';ctx.fillText(`격자 ${step.toPrecision(1)}m · 높이 제한 없음`,8,18);
    if(!validPoints.length){ctx.fillStyle='#c3d0da';ctx.fillText(mapped?'누적 맵 없음 · 맵 생성을 시작하면 표시됩니다.':`전방 180° · 수평 ${Number(raw.max_range_m||2)}m 내 실제 XYZ 점 수신 대기`,12,height/2);}
    status.textContent=mapped?`누적 3D 맵 · ${validPoints.length}점${stale?' · 연결 지연':''}`:raw.error?'XYZ 수신 오류':raw.stale?'실시간 XYZ 대기 / 지연':`실시간 3D · ${validPoints.length}점`;
    const age=Number.isFinite(raw.age_s)?raw.age_s.toFixed(1)+'초':'미수신';
    detail.textContent=mapped?`시작 선수 기준 상대좌표 · ${validPoints.length}점 · 이동에 따라 누적 범위 확장 · 현재 yaw − 시작 yaw · GPS 지도 방향 미보정`:`${raw.source_topic||'PointCloud2'} · 전방 180°(±90°) · 수평 0.10–${Number(raw.max_range_m||2).toFixed(2)}m · 높이 무제한 · ${validPoints.length}/${raw.input_count||0}점 · 수신 경과 ${age}`+(raw.error?' · '+raw.error:'');
  }
  canvas.style.touchAction='none';
  canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,id:e.pointerId};canvas.setPointerCapture?.(e.pointerId);});
  canvas.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-drag.x)*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+(e.clientY-drag.y)*.008));drag.x=e.clientX;drag.y=e.clientY;draw();});
  const end=()=>{drag=null;};canvas.addEventListener('pointerup',end);canvas.addEventListener('pointercancel',end);
  canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.1,Math.min(30,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
  $('dock3dReset').onclick=()=>{yaw=0;pitch=.6;zoom=1;draw();};mode.onchange=()=>{zoom=1;draw();};
  window.addEventListener('docking-map-update',e=>{map=e.detail||{};if(map.status?.gps_map_loaded&&loadedMapKey!==map.status.gps_map_name){mode.value='map';zoom=1;}loadedMapKey=map.status?.gps_map_loaded?map.status.gps_map_name:null;draw();});window.addEventListener('resize',draw);
  async function poll(){try{const r=await fetch('/api/mapping_pointcloud',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);raw=await r.json();}catch(e){raw={...raw,stale:true,error:e.message};}draw();setTimeout(poll,document.hidden?2000:700);}
  draw();poll();return {project,choosePreview};
})();
