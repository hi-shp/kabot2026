const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=process.argv[2],html=fs.readFileSync(path.join(root,'index.html'),'utf8'),ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
const storageValues=new Map(),storage={getItem:k=>storageValues.get(k)||null,setItem:(k,v)=>storageValues.set(k,v),removeItem:k=>storageValues.delete(k)};
const flush=()=>new Promise(r=>setImmediate(r));
function setup(){
 let state={connected:true,controls_enabled:true,map_points:[],current_scan:[],rc_path:[],origin_lla:null,hopping:null,status:{state:'MAPPING_FINISHED',initial_mapping_state:'FINISHED',rc_ready:true,extrinsics_verified:false,mapping_blockers:['WAIT_EXTRINSICS','WAIT_DECLINATION'],saved_reference_available:false}};
 const nodes=new Map(ids.map(id=>[id,{value:'',disabled:false,checked:true,textContent:''}]));for(const [id,value]of Object.entries({dockOpacity:'.7',dockHopName:'candidate',dockHopZ:'0',dockHopRadius:'1',dockHopHold:'5',dockHopHeading:''}))nodes.get(id).value=value;
 const requests=[],timers=[];const context={window:{localStorage:storage},document:{getElementById:id=>nodes.get(id)||null},console,Math,Date,JSON,Number,setTimeout:fn=>timers.push(fn),confirm:()=>true,fetch:async(url,opt)=>{if(opt)requests.push(JSON.parse(opt.body));return{ok:true,json:async()=>opt?{success:true,request_id:'mock'}:state}}};vm.createContext(context);
 vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8').split('/* END SATELLITE MAP */')[0],context);vm.runInContext(fs.readFileSync(path.join(root,'docking_map.js'),'utf8'),context);
 const Satellite=vm.runInContext('SatelliteMap',context),map={center:Satellite.project([35.12,129.1]),zoom:19,width:800,height:500,schedule(){}};context.window.DockingMapUI.bindMap(map);
 return{nodes,requests,timers,context,map,setState:value=>{state=value},getState:()=>state};
}
(async()=>{
 const t=setup();await flush();assert.strictEqual(t.nodes.get('dockHopAdd').disabled,false);assert.strictEqual(t.nodes.get('dockHopApply').disabled,true);assert.strictEqual(t.nodes.get('dockLoad').disabled,true);
 t.nodes.get('dockHopAdd').onclick();assert(t.context.window.DockingMapUI.mapClick(t.map,450,230));assert(t.nodes.get('dockHopCoords').textContent.includes('임시 후보'));assert(storageValues.size===1);t.nodes.get('dockHopApply').onclick();assert.strictEqual(t.requests.length,0,'no command before reference');
 assert(t.nodes.get('dockError').textContent.includes('센서 장착'));assert(t.nodes.get('dockError').textContent.includes('자기편각'));
 const restored=setup();await flush();assert(restored.nodes.get('dockHopCoords').textContent.includes('임시 후보'),'draft survives reload');
 t.setState({...t.getState(),status:{...t.getState().status,reference_id:'saved-ref',reference_version:1,extrinsics_verified:true,mapping_blockers:[],saved_reference_available:true}});await t.timers.shift()();await flush();assert(t.nodes.get('dockHopCoords').textContent.includes('임시 후보'),'reference arrival preserves draft');assert.strictEqual(t.nodes.get('dockHopApply').disabled,false);t.nodes.get('dockHopApply').onclick();await flush();assert.strictEqual(t.requests[0].action,'set_hopping');assert.strictEqual(t.requests[0].reference_id,'saved-ref');
 const project=t.context.window.DockingLivePreview.project;assert(project([0,0,1],12,600,320)[1]!==project([0,0,0],12,600,320)[1]);
 const choose=t.context.window.DockingLivePreview.choosePreview;const staleRaw={points:[],stale:true};assert.strictEqual(choose(staleRaw,{points:[[1,2,0]],stale:false,is2d:true}),staleRaw);
 console.log('MAP_DRAFTS_OK: no-reference picking; no premature ROS write; reload retention; reference transition; persistent blockers; sensor-only preview axes');
})().catch(e=>{console.error(e);process.exitCode=1});
