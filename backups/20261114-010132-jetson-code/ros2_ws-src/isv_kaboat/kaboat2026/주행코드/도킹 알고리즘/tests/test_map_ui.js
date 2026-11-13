/* Static JS unit test with a stub DOM. This is NOT a browser/rendering test. */
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=process.argv[2];if(!root)throw Error('Pass existing GUI asset directory');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const ids=new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]));
const nodes=new Map([...ids].map(id=>[id,{value:'',checked:true,disabled:false,textContent:''}]));
for(const [id,value]of Object.entries({dockOpacity:'.7',dockHopName:'hop1',dockHopZ:'0',dockHopRadius:'1',dockHopHold:'5',dockHopHeading:'',dockZMin:'-3',dockZMax:'6',dockRefYaw:'0'}))nodes.get(id).value=value;
const state={preview_schema:1,connected:true,controls_enabled:true,origin_lla:[35.12,129.1,0],
  status:{reference_id:'fixture',reference_version:1,phase:'REFERENCE_READY',initial_mapping_state:'FROZEN',rc_ready:true,state:'REFERENCE_LOADED',hopping_config:{}},
  map_points:[],current_scan:[],rc_path:[],hopping:null};
const requests=[];
const context={window:{},document:{getElementById:id=>nodes.get(id)||null},console,Math,Date,JSON,Number,
  setTimeout:()=>0,confirm:()=>true,fetch:async(url,options)=>{if(options)requests.push(JSON.parse(options.body));return {ok:true,json:async()=>options?{success:true,request_id:'fixture-request'}:state};}};
vm.createContext(context);
const app=fs.readFileSync(path.join(root,'app.js'),'utf8');
vm.runInContext(app.split('/* END SATELLITE MAP */')[0],context);
vm.runInContext(fs.readFileSync(path.join(root,'docking_map.js'),'utf8'),context);
const Satellite=vm.runInContext('SatelliteMap',context),ui=context.window.DockingMapUI;
async function main(){
  await new Promise(resolve=>setImmediate(resolve));
  const map={center:Satellite.project([35.12,129.1]),zoom:19,width:800,height:500,schedule(){},
    screen(ll){const q=Satellite.project(ll),s=256*2**this.zoom;return [this.width/2+Satellite.delta(q[0]-this.center[0])*s,this.height/2+(q[1]-this.center[1])*s];}};
  let cases=0;
  for(const zoom of [16,19,22])for(const center of [[35.12,129.1],[35.1201,129.1002]])for(const ll of [[35.12,129.1],[35.12001,129.10002]]){
    map.zoom=zoom;map.center=Satellite.project(center);const pixel=map.screen(ll),back=ui.clickToLatLon(map,...pixel);
    assert(Math.abs(ll[0]-back[0])<1e-10&&Math.abs(ll[1]-back[1])<1e-10);cases++;
  }
  ui.bindMap(map);
  assert.strictEqual(ui.mapClick(map,400,250),false,'normal map clicks must not be intercepted');
  nodes.get('dockHopAdd').onclick();assert.strictEqual(ui.mapClick(map,400,250),true);
  assert.strictEqual(ui.mapClick(map,400,250),false,'hopping click mode must finish after one click');
  nodes.get('dockHopZ').value='0';nodes.get('dockHopApply').onclick();
  await Promise.resolve();await Promise.resolve();
  const payload=requests[0];assert(payload&&payload.action==='set_hopping');
  assert(Math.abs(payload.latitude-35.1201)<1e-10&&Math.abs(payload.longitude-129.1002)<1e-10);
  assert.strictEqual(payload.height_enu_m,0);assert.strictEqual(payload.reference_id,'fixture');
  console.log(`STATIC_JS_OK: ${cases} projection/pan/zoom roundtrips; click mode isolation; bound hopping payload`);
}
main().catch(error=>{console.error(error);process.exitCode=1;});
