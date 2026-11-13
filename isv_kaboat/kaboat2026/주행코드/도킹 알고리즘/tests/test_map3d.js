const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const root=process.argv[2],source=fs.readFileSync(path.join(root,'docking_map.js'),'utf8').split('/* Display-only XYZ:')[1];
const events={},labels=new Map(),calls=[],timers=[];
const ctx=new Proxy({}, {get:(t,k)=>t[k]??(()=>{}),set:(t,k,v)=>(t[k]=v,true)});
const canvas={style:{},width:600,height:360,getContext:()=>ctx,getBoundingClientRect:()=>({width:600}),addEventListener:(n,f)=>events[n]=f};
for(const key of ['dockLiveStatus','dockLiveDetail','dockLiveRange','dock3dReset'])labels.set(key,{value:key==='dockLiveRange'?'live':'',textContent:''});
const listeners={},window={addEventListener:(n,f)=>listeners[n]=f};
const context={window,document:{getElementById:id=>id==='dockLiveCanvas'?canvas:labels.get(id),hidden:false},Math,Number,Array,console,setTimeout:fn=>timers.push(fn),fetch:async url=>{calls.push(url);return{ok:true,json:async()=>({points:[[.2,.1,3]],input_count:100,stale:false,age_s:.1,source_topic:'/velodyne_points_raw'})}}};
vm.createContext(context);vm.runInContext('/* Display-only XYZ:'+source,context);
const api=window.DockingLivePreview,p=api.project([.2,.1,0],1,600,360),z=api.project([.2,.1,3],1,600,360);
assert.notStrictEqual(p[1],z[1],'actual Z changes projection');
const rotated=api.project([.2,.1,0],1,600,360,1.1,.3);assert.notDeepStrictEqual(Array.from(p),Array.from(rotated),'orbit changes view');
for(const [w,h]of [[600,360],[280,360]]){const o=api.project([0,0,0],2,w,h),bow=api.project([1,0,0],2,w,h),port=api.project([0,1,0],2,w,h),starboard=api.project([0,-1,0],2,w,h);assert(bow[1]<o[1],'bow is up by default');assert(port[0]<o[0],'port is left by default');assert(starboard[0]>o[0],'starboard is right by default');}
const stale={points:[],stale:true};assert.strictEqual(api.choosePreview(stale,{points:[[1,1,0]],is2d:true}),stale,'no invented 3D from LaserScan');
setImmediate(()=>{
 assert.strictEqual(calls[0],'/api/mapping_pointcloud');assert(labels.get('dockLiveStatus').textContent.includes('1점'));
 assert(labels.get('dockLiveDetail').textContent.includes('높이 무제한'));
 labels.get('dockLiveRange').value='map';listeners['docking-map-update']({detail:{connected:true,map_points_xyz:[[1,2,5],[2,3,-4]],rc_path_xyz:[]}});labels.get('dockLiveRange').onchange();
 assert(labels.get('dockLiveStatus').textContent.includes('2점'));assert(labels.get('dockLiveDetail').textContent.includes('현재 yaw − 시작 yaw'));
 let prevented=false;events.wheel({preventDefault(){prevented=true;},deltaY:10});assert(prevented);
 console.log('MAP_3D_OK: XYZ height; orbit; real cloud only; live/map switching; wheel zoom');
});
