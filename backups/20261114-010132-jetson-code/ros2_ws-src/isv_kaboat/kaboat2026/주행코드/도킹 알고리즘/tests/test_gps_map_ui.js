const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=process.argv[2],html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.strictEqual(new Set(ids).size,ids.length,'unique DOM ids');
assert(html.indexOf('class="panel map-panel"')<html.indexOf('class="dock-live-panel"'),'2D left of 3D');
assert(html.includes('data-gps-map-only="true"'));
const nodes=new Map(ids.map(id=>[id,{value:'',disabled:false,checked:true,textContent:'',setAttribute(){}}]));
for(const [id,v]of Object.entries({dockOpacity:'.7',dockHopName:'hop1',dockHopZ:'0',dockHopRadius:'1',dockHopHold:'10'}))nodes.get(id).value=v;
let state={connected:true,controls_enabled:true,map_points:[[35.12,129.1],[35.12001,129.10001]],map_points_xyz:[[0,0,0],[1,1,1]],current_scan:[],rc_path:[],status:{initial_mapping_state:'FINISHED',gps_map_available:false,gps_map_loaded:false,rc_ready:true}};
const timers=[],requests=[],calls=[],ctx=new Proxy({}, {get:(t,k)=>k==='measureText'?()=>({width:30}):t[k]??((...a)=>calls.push([k,...a])),set:(t,k,v)=>(t[k]=v,true)});
const canvas={dataset:{gpsMapOnly:'true'},getContext:()=>ctx,getBoundingClientRect:()=>({width:800,height:420}),addEventListener(){},classList:{add(){},remove(){}}};
const context={window:{devicePixelRatio:1,localStorage:{getItem:()=>null},dispatchEvent(){}},document:{getElementById:id=>nodes.get(id)},CustomEvent:class{},console,Math,Date,Number,JSON,Image:class{},requestAnimationFrame(){},setTimeout:f=>timers.push(f),confirm:()=>true,fetch:async(url,opt)=>{if(opt)requests.push(JSON.parse(opt.body));return{ok:true,json:async()=>opt?{success:true,request_id:'test'}:state}}};
vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8').split('/* END SATELLITE MAP */')[0],context);vm.runInContext(fs.readFileSync(path.join(root,'docking_map.js'),'utf8'),context);
const flush=()=>new Promise(r=>setImmediate(r));
(async()=>{
 await flush();const MapView=vm.runInContext('SatelliteMap.View',context),view=new MapView(canvas);view.satellite=false;
 view.update({waypoints:[[36,130]],position:{latitude:35.12,longitude:129.1},yaw:.7,report_age:0,gps_age:0,lidar_display:{status:'LIVE',map_pose:{yaw:0,latitude:35.12,longitude:129.1},points:[[1,1]]}});
 calls.length=0;context.window.DockingMapUI.draw(view,ctx,800,420);assert.strictEqual(calls.length,0,'no overlay before load, even when live map exists');
 calls.length=0;view.draw();assert(!calls.some(c=>c[0]==='fillText'&&String(c[1]).startsWith('WP')),'no WP labels');assert(calls.some(c=>c[0]==='rotate'&&c[1]===-.7),'boat arrow follows IMU yaw');
 assert.strictEqual(nodes.get('dockLoad').disabled,true);assert.strictEqual(nodes.get('dockSave').disabled,false);
 nodes.get('dockSave').onclick();await flush();assert.strictEqual(requests[0].action,'save_gps_map');
 // Finish the save request, then publish a loaded snapshot through normal polling.
 state={...state,status:{...state.status,gps_map_available:true,gps_map_loaded:true,gps_map_name:'stored-map',last_gui_result:{request_id:'test',success:true,message:'saved'}}};
 await timers.shift()();await flush();calls.length=0;context.window.DockingMapUI.draw(view,ctx,800,420);
 assert.strictEqual(calls.filter(c=>c[0]==='fillRect').length,2,'loaded WGS84 map is drawn');
 assert.strictEqual(nodes.get('dockLoad').disabled,false);nodes.get('dockLoad').onclick();await flush();assert.strictEqual(requests[1].action,'load_gps_map');
 console.log('GPS_MAP_UI_OK: clean initial view; bow arrow; save/load payloads; loaded-only overlay; side-by-side DOM');
})().catch(e=>{console.error(e);process.exitCode=1});
