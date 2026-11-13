(()=>{
const el=id=>document.getElementById(id);let state={},busy=false,note='';
function render(){const locked=busy||state.active||state.armed||state.docking_active;
el('runtimeStatus').textContent=`${state.mission_running?(state.variant==='v3'?'V3':'3-WP 도킹'):'임무 꺼짐'} · ${state.output_enabled?'운항 모드':'미리보기 / 출력 꺼짐'} · 맵 ${state.mapping_connected?'연결됨':state.mapper_running?'시작 중':'꺼짐'}`;
for(const id of ['runtimePreview','runtimeAutonomy','runtimeMissionStop','runtimeMapStop','runtimeDockPrepare','runtimeDockPreview','runtimeDockAutonomy'])el(id).disabled=Boolean(locked);
el('runtimeMapStart').disabled=busy||state.mapper_running;el('runtimeVariant').disabled=Boolean(locked);
el('runtimeMessage').textContent=note||'프로그램 준비 → WP·목표 적용 → 시작 / 재개. 맵 생성·편집은 RC 수동 모드에서 진행하세요.';
el('runtimeWarnings').textContent=(state.warnings||[]).join(' · ')+(state.docking_output_enabled?'':' · 맵 기반 도킹은 현재 Dry-run입니다.');
window.missionVariant=state.variant;
const third=el('lat2')?.closest('.waypoint-row');if(third)third.hidden=state.variant==='v3';
const isV3=state.variant==='v3';const stages=document.querySelectorAll('nav.mission>div');if(stages.length===4){stages[0].querySelector('span').innerHTML=isV3?'WP1<small>장애물 회피</small>':'WP1 → WP2<small>GPS · 장애물 회피</small>';stages[1].querySelector('span').innerHTML=isV3?'WP2<small id="countdown">GPS 호핑 · 위치 유지</small>':'WP2 정지<small id="countdown">10초 중립 유지</small>';stages[2].hidden=isV3;const arrows=document.querySelectorAll('nav.mission>i');if(arrows[1])arrows[1].hidden=isV3;}
if(el('applyRoute'))el('applyRoute').textContent=state.variant==='v3'?'WP1 · WP2 적용':'3개 좌표 적용';}
async function command(action){if(busy)return;busy=true;note='프로그램 연결 처리 중…';render();try{
const r=await fetch('/api/runtime/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});const d=await r.json();if(!r.ok||!d.success)throw Error(d.error||d.message||'실행 실패');state=d;note=d.message;
}catch(e){note=e.message;}finally{busy=false;render();}}
el('runtimePreview').onclick=()=>command(el('runtimeVariant').value+'_preview');el('runtimeAutonomy').onclick=()=>command(el('runtimeVariant').value+'_autonomy');el('runtimeMissionStop').onclick=()=>command('mission_stop');el('runtimeMapStart').onclick=()=>command('mapper_start');el('runtimeMapStop').onclick=()=>command('mapper_stop');el('runtimeDockPrepare').onclick=()=>command('docking_prepare');el('runtimeDockPreview').onclick=()=>command('docking_preview');el('runtimeDockAutonomy').onclick=()=>command('docking_autonomy');
async function poll(){try{const r=await fetch('/api/runtime');if(!r.ok)throw Error();state=await r.json();render();}catch(e){el('runtimeStatus').textContent='서버 연결 대기';}setTimeout(poll,1000);}poll();
})();
