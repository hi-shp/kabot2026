// Exercise UI rendering and commands in a synthetic DOM (not a browser render).
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=path.join(__dirname,'..','web'),html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const ctx=new Proxy({}, {get(target,key){return target[key]??(()=>{});},set(target,key,value){target[key]=value;return true;}});
function element(){return {value:'',textContent:'',disabled:false,hidden:false,children:[],dataset:{},classList:{toggle(){}},append(x){this.children.push(x)},replaceChildren(){this.children=[]},getBoundingClientRect(){return {width:500,height:270}},getContext(){return ctx},click(){}}}
const elements={};for(const match of html.matchAll(/id="([^"]+)"/g))elements[match[1]]=element();
const captures=[0,1,2].map(i=>Object.assign(element(),{dataset:{capture:String(i)}}));
const base={stamp:100,state:'WP1_TO_WP2',stage:'WP1_TO_WP2',wp_index:1,
 waypoints:[[35,128],[35.0002,128],[35.0004,128]],position:{latitude:35.0001,longitude:128,sigma:.4},
 target_label:'red triangle',target:null,output:{enabled:true,active:false},report_age:.1,camera_age:null,
 intended_pwm:[1600,1600],feedback_pwm:[1600,1600],history:[{pwm:[1500,1500]},{pwm:[1600,1600]}],events:[],
 hold_remaining:10,distance_to_wp:11,yaw:1.57,heading_error_deg:0,gps_age:.1,scan_age:.1,heading_age:.1,telemetry_age:.1};
const requests=[];
const context={document:{getElementById:id=>{assert(elements[id],id);return elements[id]},createElement:element,
 querySelectorAll:selector=>selector==='[data-capture]'?captures:[]},window:{addEventListener(){}},devicePixelRatio:1,
 setTimeout(){},setInterval(){},console,Date,Math,Number,JSON,Blob,URL,
 fetch:async(url,options)=>{if(options)requests.push(JSON.parse(options.body));return {ok:true,json:async()=>base}}};
vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8'),context);
(async()=>{
 await new Promise(resolve=>setImmediate(resolve));
 for(const [state,stage]of [['AVOID_OBSTACLE','WP1_TO_WP2'],['WP2_HOLD','WP2_HOLD'],['ENTER_REGION','DOCK_TO_WP3'],['MISSION_COMPLETE','COMPLETE']]){
  context.sample={...base,state,stage};vm.runInContext('render(sample)',context);
  assert(elements.state.textContent.length>0);
 }
 context.sample={...base};vm.runInContext('render(sample)',context);
 assert.strictEqual(elements.start.disabled,false);
 elements.stop.onclick();await new Promise(resolve=>setImmediate(resolve));
 assert.strictEqual(requests.at(-1).action,'stop');
 elements.applyRoute.onclick();await new Promise(resolve=>setImmediate(resolve));
 assert.strictEqual(requests.at(-1).action,'set_route');
 assert.strictEqual(requests.at(-1).waypoints.length,3);
 console.log('UI_RENDER_AND_BUTTON_LOGIC_OK: four mission stages, stop and waypoint settings');
})().catch(error=>{console.error(error);process.exitCode=1});
