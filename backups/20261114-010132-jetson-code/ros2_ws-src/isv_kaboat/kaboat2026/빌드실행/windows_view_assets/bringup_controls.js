/* Equipment launch controls; independent of mission and docking start buttons. */
(()=>{
  const start=document.getElementById('bringupStart'),stop=document.getElementById('bringupStop');
  const status=document.getElementById('bringupStatus'),message=document.getElementById('bringupMessage');
  let busy=false,last={},note='';
  function render(){
    const labels={RUNNING:'실행 중',STOPPED:'꺼짐',STOPPING:'종료 중',CONFLICT:'중복 실행',DISABLED:'조작 불가'};
    status.textContent=(labels[last.state]||'연결 대기')+(last.pid?` · PID ${last.pid}`:'');
    start.disabled=busy||!last.can_start;stop.disabled=busy||!last.can_stop;
    message.textContent=note||last.message||'센서·YOLO·퓨전 브링업 상태를 기다립니다.';
  }
  async function command(action){
    if(busy)return;busy=true;note='처리 중';render();
    try{
      const response=await fetch('/api/bringup/command',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({action})});
      const result=await response.json();
      if(!response.ok||!result.success)throw Error(result.error||result.message||'브링업 요청 실패');
      last={...last,...result};note=result.message||'요청 완료';
    }catch(error){note=error.message;}finally{busy=false;render();}
  }
  start.onclick=()=>command('start');stop.onclick=()=>command('stop');
  async function poll(){
    try{
      const response=await fetch('/api/bringup');if(!response.ok)throw Error('상태 수신 실패');
      const previous=last.state;last=await response.json();
      if(previous!==last.state&&!busy)note='';render();
    }catch(error){last={state:'DISCONNECTED',can_start:false,can_stop:false};note='브링업 GUI 연결 대기';render();}
    setTimeout(poll,1000);
  }
  render();poll();
})();
