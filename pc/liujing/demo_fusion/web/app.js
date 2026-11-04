import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {encodeMapPLY} from './map-export.js';
import {installHeightColors} from './height-color.js';
import {backendLabel,plScope,mapTelemetry} from './view-state.js';

const $=id=>document.getElementById(id),canvas=$('scene');
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.setClearColor(0x101819);
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.03,1000);
camera.up.set(0,0,1);camera.position.set(4,-5,3);
const controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.dampingFactor=.12;
const limit=2000000,pos=new Float32Array(limit*3),col=new Float32Array(limit*3),rgb=new Uint32Array(limit),geometry=new THREE.BufferGeometry();
geometry.setAttribute('position',new THREE.BufferAttribute(pos,3).setUsage(THREE.DynamicDrawUsage));
geometry.setAttribute('color',new THREE.BufferAttribute(col,3).setUsage(THREE.DynamicDrawUsage));geometry.setDrawRange(0,0);
const dot=document.createElement('canvas');dot.width=dot.height=32;
const ctx=dot.getContext('2d');ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(16,16,15,0,Math.PI*2);ctx.fill();
const material=new THREE.PointsMaterial({size:.032,sizeAttenuation:true,vertexColors:true,map:new THREE.CanvasTexture(dot),alphaTest:.45});
const heightColors=installHeightColors(material);let heightRangeReady=false;
const cloud=new THREE.Points(geometry,material);cloud.frustumCulled=false;scene.add(cloud);
const marker=new THREE.Mesh(new THREE.SphereGeometry(.035,12,8),new THREE.MeshBasicMaterial({color:0xd7bd7c}));marker.visible=false;scene.add(marker);
const trajectoryGeometry=new THREE.BufferGeometry(),trajectoryPositions=new Float32Array(15000);
trajectoryGeometry.setAttribute('position',new THREE.BufferAttribute(trajectoryPositions,3).setUsage(THREE.DynamicDrawUsage));trajectoryGeometry.setDrawRange(0,0);
const trajectory=new THREE.Line(trajectoryGeometry,new THREE.LineBasicMaterial({color:0xbdac77,transparent:true,opacity:.7}));trajectory.visible=false;trajectory.frustumCulled=false;scene.add(trajectory);
let grid=null,trajectoryCount=0,displayMode='color',snapshot=false,snapshotLabel='',lastStatusAt=0,lastPoseAt=0,lastCameraAt=0,latestMeta=null;
let session='',version=0,count=0,pending=null,ack=null,connected=false,fitOnce=false,follow=false,state={},poses=[],events=[],gaps=[],frames=[],frameGaps=[],drawEvents=[],previousFrame=0,lastReport=0,lastCamera=0,sourceAge=null,lastMapAt=0,skipped=0,clock=null,visibilityEpoch=0,lastExtent=0,lastExtentVersion=-1;
const clientId=crypto.randomUUID(),worker=new Worker('/color-worker.js');let decodeResolve,renderCommit=null;
worker.onmessage=e=>{if(decodeResolve){decodeResolve(e.data);decodeResolve=null;}};
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('toast').hidden=true,3200);}
function resize(){const box=$('viewport');renderer.setSize(box.clientWidth,box.clientHeight,false);camera.aspect=box.clientWidth/box.clientHeight;camera.updateProjectionMatrix();}
new ResizeObserver(resize).observe($('viewport'));resize();
function bounds(){const min=new THREE.Vector3(Infinity,Infinity,Infinity),max=new THREE.Vector3(-Infinity,-Infinity,-Infinity);for(let i=0;i<count*3;i+=3){min.x=Math.min(min.x,pos[i]);min.y=Math.min(min.y,pos[i+1]);min.z=Math.min(min.z,pos[i+2]);max.x=Math.max(max.x,pos[i]);max.y=Math.max(max.y,pos[i+1]);max.z=Math.max(max.z,pos[i+2]);}return new THREE.Box3(min,max);}
function updateExtent(){if(!count)return;const box=bounds(),size=box.getSize(new THREE.Vector3());$('extent').textContent=[size.x,size.y,size.z].map(n=>n.toFixed(1)).join(' × ')+' m';if(grid){grid.position.set(Math.round((box.min.x+box.max.x)/2),Math.round((box.min.y+box.max.y)/2),Math.floor(box.min.z));}updateHeightRange(box);lastExtentVersion=version;}
function updateHeightRange(box){
 // Expand in half-metre steps; never renormalize on camera motion or shrinking bounds.
 let low=Math.floor(box.min.z*2)/2,high=Math.ceil(box.max.z*2)/2;if(high-low<.5){low-=.25;high+=.25;}
 const range=heightColors.heightRange.value;
 if(heightRangeReady){low=Math.min(low,range.x);high=Math.max(high,range.y);}
 range.set(low,high);heightRangeReady=true;
 $('heightLow').textContent=low.toFixed(2)+' m';$('heightHigh').textContent=high.toFixed(2)+' m';
 $('heightLegend').hidden=displayMode!=='points'||!count;
}
function setFollow(value){follow=value;$('follow').classList.toggle('selected',follow);$('free').classList.toggle('selected',!follow);$('follow').setAttribute('aria-pressed',String(follow));$('free').setAttribute('aria-pressed',String(!follow));$('viewMode').textContent=follow?'跟随设备':'自由观察';if(!follow)camera.up.set(0,0,1);}
function fit(direction=new THREE.Vector3(1,-1,.65)){
 if(!count){toast('收到地图后即可调整视角');return;}resize();setFollow(false);
 const box=bounds(),center=box.getCenter(new THREE.Vector3()),toward=direction.clone().normalize();
 camera.up.set(0,0,1);if(Math.abs(toward.z)>.99&&Math.abs(toward.x)+Math.abs(toward.y)<.01)camera.up.set(0,1,0);
 const right=new THREE.Vector3().crossVectors(camera.up,toward).normalize(),up=new THREE.Vector3().crossVectors(toward,right).normalize();
 const tanV=Math.tan(camera.fov*Math.PI/360),tanH=tanV*camera.aspect;let distance=.5;
 // Fit all eight corners using the actual viewport aspect. A bounding sphere
 // wastes most of the view on elongated rooms; no map point is discarded here.
 for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z]){
  const corner=new THREE.Vector3(x,y,z).sub(center),depth=corner.dot(toward);
  distance=Math.max(distance,depth+Math.max(Math.abs(corner.dot(right))/tanH,Math.abs(corner.dot(up))/tanV)*1.08);
 }
 controls.target.copy(center);camera.position.copy(center).addScaledVector(toward,distance);camera.near=Math.max(.005,distance/5000);camera.far=Math.max(100,distance*20);camera.updateProjectionMatrix();controls.update();updateExtent();
}
function display(mode){displayMode=mode;const points=mode==='points';material.vertexColors=true;material.color.set(0xffffff);heightColors.heightEnabled.value=points;$('heightLegend').hidden=!points||!count;material.sizeAttenuation=!points;material.size=Number($('pointSize').value)*(points?1.25:.016);material.needsUpdate=true;$('colorMode').classList.toggle('selected',!points);$('pointMode').classList.toggle('selected',points);$('colorMode').setAttribute('aria-pressed',String(!points));$('pointMode').setAttribute('aria-pressed',String(points));$('viewLabel').textContent=points?'高度着色点云':'彩色重建地图';}
function reset(s){heightRangeReady=false;$('heightLegend').hidden=true;session=s;version=0;count=0;geometry.setDrawRange(0,0);fitOnce=false;poses=[];events=[];gaps=[];lastMapAt=0;sourceAge=null;clock=null;trajectoryCount=0;trajectoryGeometry.setDrawRange(0,0);marker.visible=false;latestMeta=null;lastExtentVersion=-1;$('emptyState').hidden=false;$('export').disabled=true;}
$('fit').onclick=()=>fit();$('free').onclick=()=>setFollow(false);$('follow').onclick=()=>{if(snapshot||performance.now()-lastPoseAt>1500){toast('等待有效的实时设备位姿');return;}setFollow(true);};
controls.addEventListener('start',()=>setFollow(false));$('colorMode').onclick=()=>display('color');$('pointMode').onclick=()=>display('points');
$('topView').onclick=()=>fit(new THREE.Vector3(0,0,1));$('frontView').onclick=()=>fit(new THREE.Vector3(0,-1,.05));$('sideView').onclick=()=>fit(new THREE.Vector3(1,0,.05));
$('pointSize').oninput=()=>{$('pointSizeValue').value=Number($('pointSize').value).toFixed(1);material.size=Number($('pointSize').value)*(displayMode==='points'?1.25:.016);};
$('gridToggle').onchange=()=>{if(!grid){grid=new THREE.GridHelper(30,30,0x4b6051,0x293c31);grid.rotation.x=Math.PI/2;grid.material.transparent=true;grid.material.opacity=.45;scene.add(grid);updateExtent();}grid.visible=$('gridToggle').checked;$('gridNote').hidden=!grid.visible;};
$('trajectoryToggle').onchange=()=>{trajectory.visible=$('trajectoryToggle').checked;if(trajectory.visible&&!trajectoryCount)toast('轨迹将在收到有效设备位姿后显示');};
$('sidebarToggle').onclick=()=>{const space=document.querySelector('.workspace');let expanded;if(innerWidth<=820){expanded=space.classList.toggle('mobile-inspector');}else{expanded=!space.classList.toggle('sidebar-hidden');}const label=expanded?'收起信息栏':'展开信息栏';$('sidebarToggle').title=label;$('sidebarToggle').setAttribute('aria-label',label);$('sidebarToggle').setAttribute('aria-expanded',String(expanded));};
$('tech').onclick=()=>$('detailsDialog').showModal();$('closeDetails').onclick=()=>$('detailsDialog').close();$('detailsDialog').addEventListener('click',e=>{if(e.target===$('detailsDialog')){const r=$('detailsDialog').getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)$('detailsDialog').close();}});
$('help').onclick=()=>$('helpDialog').showModal();$('closeHelp').onclick=()=>$('helpDialog').close();
$('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}catch{toast('此浏览器暂不支持全屏，可使用浏览器的全屏功能');}};
document.addEventListener('fullscreenchange',()=>{$('fullscreen').textContent=document.fullscreenElement?'退出全屏':'全屏';});
function zoom(factor){if(!count){toast('收到地图后即可缩放');return;}setFollow(false);const offset=camera.position.clone().sub(controls.target);offset.setLength(THREE.MathUtils.clamp(offset.length()*factor,.05,camera.far*.8));camera.position.copy(controls.target).add(offset);controls.update();}
$('zoomIn').onclick=()=>zoom(.8);$('zoomOut').onclick=()=>zoom(1.25);
document.addEventListener('keydown',e=>{if(e.ctrlKey||e.metaKey||e.altKey||e.repeat||/INPUT|TEXTAREA|SELECT|BUTTON/.test(e.target.tagName)||e.target.isContentEditable||document.querySelector('dialog[open]'))return;const actions={f:()=>fit(),'1':()=>$('topView').click(),'2':()=>$('frontView').click(),'3':()=>$('sideView').click(),g:()=>{$('gridToggle').checked=!$('gridToggle').checked;$('gridToggle').dispatchEvent(new Event('change'));}};const action=actions[e.key.toLowerCase()];if(action){e.preventDefault();action();}});
// Display preferences never change map data or fusion parameters.
try{const pref=JSON.parse(localStorage.getItem('liujing-view-preferences')||'{}');if(Number.isFinite(pref.pointSize)&&pref.pointSize>=.6&&pref.pointSize<=4)$('pointSize').value=pref.pointSize;if(typeof pref.grid==='boolean')$('gridToggle').checked=pref.grid;if(typeof pref.trajectory==='boolean')$('trajectoryToggle').checked=pref.trajectory;}catch{}
function savePreferences(){try{localStorage.setItem('liujing-view-preferences',JSON.stringify({pointSize:Number($('pointSize').value),grid:$('gridToggle').checked,trajectory:$('trajectoryToggle').checked}));}catch{}}
$('pointSize').addEventListener('input',savePreferences);$('gridToggle').addEventListener('change',savePreferences);$('trajectoryToggle').addEventListener('change',savePreferences);
$('pointSize').oninput();trajectory.visible=$('trajectoryToggle').checked;
if(innerWidth<=820){$('sidebarToggle').setAttribute('aria-expanded','false');$('sidebarToggle').setAttribute('aria-label','展开信息栏');$('sidebarToggle').title='展开信息栏';}

$('export').onclick=()=>{if(!count)return;const bytes=encodeMapPLY(pos,rgb,count,version),url=URL.createObjectURL(new Blob([bytes],{type:'application/octet-stream'})),a=document.createElement('a');a.href=url;a.download='liujing-map-v'+version+'.ply';a.click();setTimeout(()=>URL.revokeObjectURL(url),30000);toast('已准备导出 '+count.toLocaleString()+' 个重建点');};

async function colors(){while(true){if(document.hidden){await sleep(250);continue;}try{const viewEpoch=visibilityEpoch,start=performance.now(),r=await fetch('/api/color?after='+version,{cache:'no-store',signal:AbortSignal.timeout(version===0?15000:3500)});if(r.status===204){await sleep(snapshot?1000:20);continue;}if(!r.ok)throw Error('stream');const bytes=await r.arrayBuffer(),decoded=await new Promise(resolve=>{decodeResolve=resolve;worker.postMessage(bytes,[bytes]);});if(decoded.error)throw Error(decoded.error);if(viewEpoch!==visibilityEpoch||document.hidden)continue;const m=decoded.meta,full=m.full===true||m.full==='true';if(m.session!==session){reset(m.session);if(!full)continue;}if(!full&&Number(m.base_version)!==version){version=0;continue;}if(Number(m.version)===version)continue;pending={...decoded,requestStart:start,received:performance.now(),full};await new Promise(resolve=>ack=resolve);}catch{connected=false;await sleep(400);}}}
function apply(now){if(!pending)return;const p=pending,m=p.meta;pending=null;if(m.session!==session){if(ack){ack();ack=null;}return;}const before=version;if(p.full)geometry.setDrawRange(0,0);for(let i=0;i<p.ids.length;i++){const id=p.ids[i];for(let k=0;k<3;k++){pos[id*3+k]=p.positions[i*3+k];col[id*3+k]=p.colors[i*3+k];}rgb[id]=p.packedColors[i];}
 if(p.ids.length)for(const key of ['position','color']){const a=geometry.getAttribute(key);a.clearUpdateRanges();let begin=p.ids[0],end=begin;for(let i=1;i<p.ids.length;i++){const id=p.ids[i];if(id<=end+16)end=id;else{a.addUpdateRange(begin*3,(end-begin+1)*3);begin=end=id;}}a.addUpdateRange(begin*3,(end-begin+1)*3);a.needsUpdate=true;}
 count=Number(m.map_count);geometry.setDrawRange(0,count);version=Number(m.version);latestMeta=m;skipped+=before?Math.max(0,version-before-1):0;sourceAge=Number(m.source_age_s)*1000+(clock?Math.max(0,now+clock.offset-Number(m.send_mono)*1000)+clock.rtt/2:now-p.requestStart);renderCommit={at:now,version,source_age_upper_ms:sourceAge,source_stamp:Number(m.source_stamp),rgb_id:m.rgb_id,depth_id:m.depth_id};if(events.length>10000)events.shift();if(gaps.length>10000)gaps.shift();connected=true;$('emptyState').hidden=!!count;$('export').disabled=!count;if(!fitOnce&&count){fit();fitOnce=true;}if(ack){ack();ack=null;}}

async function status(){while(true){if(document.hidden){await sleep(500);continue;}try{const start=performance.now(),r=await fetch('/api/status',{signal:AbortSignal.timeout(1800)});if(!r.ok)throw Error();state=await r.json();lastStatusAt=performance.now();const rtt=lastStatusAt-start;snapshot=state.display_mode==='snapshot';snapshotLabel=state.snapshot_label||'';if(!clock||rtt<clock.rtt||lastStatusAt-clock.at>15000)clock={offset:Number(state.mono)*1000-(start+lastStatusAt)/2,rtt,at:lastStatusAt};if(state.session&&state.session!==session)reset(state.session);$('backend').textContent=backendLabel(state.backend);$('backendScope').hidden=!String(state.backend||'').startsWith('pl_');$('backendScope').textContent=plScope;$('snapshotTag').hidden=!snapshot;$('dataSource').textContent=snapshot?'已提交地图 · 离线快照':'RGB-D 已提交地图';if(snapshot){$('overviewCaption').textContent=snapshotLabel||'已保存的融合结果，当前未连接实时传感器。';$('cameraPlaceholder').lastElementChild.textContent='离线快照无相机画面';$('cameraStatus').textContent='离线';}
 const xyz=state.pose_xyz?.map(Number),stamp=Number(state.pose_stamp),age=Number(state.pose_age_s);if(!snapshot&&xyz?.length===3&&xyz.every(Number.isFinite)&&age>=0&&age<.6&&(!poses.length||stamp>poses.at(-1).stamp)){const matrix=state.pose_matrix?.map(Number),rotation=matrix?.length===16?new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().set(...matrix)):null;poses.push({at:performance.now()-age*1000,stamp,xyz,rotation});lastPoseAt=performance.now();if(poses.length>32)poses.shift();if(trajectoryCount<5000){trajectoryPositions.set(xyz,trajectoryCount*3);trajectoryCount++;trajectoryGeometry.setDrawRange(0,trajectoryCount);trajectoryGeometry.getAttribute('position').needsUpdate=true;}}
 const now=performance.now();if(!snapshot&&now-lastCamera>650){lastCamera=now;const cameraResponse=await fetch('/api/camera.jpg',{signal:AbortSignal.timeout(1800)});if(cameraResponse.ok){const old=$('camera').src;$('camera').src=URL.createObjectURL(await cameraResponse.blob());$('camera').hidden=false;$('cameraPlaceholder').hidden=true;$('cameraFrame').classList.add('has-image');lastCameraAt=performance.now();if(old.startsWith('blob:'))URL.revokeObjectURL(old);}}
 }catch{connected=false;}await sleep(snapshot?1000:150);}}
function percentile(a,p){if(!a.length)return null;const x=[...a].sort((a,b)=>a-b);return x[Math.min(x.length-1,Math.ceil(x.length*p)-1)];}
function updateUI(now,metrics){const alive=now-lastStatusAt<2500&&lastStatusAt>0,paused=!snapshot&&count&&now-lastMapAt>1500;connected=alive;let health,connection,mode;if(snapshot){health='离线快照 · 可旋转、缩放和导出';connection='离线快照';mode='snapshot';}else if(!alive){health=count?'连接已断开 · 保留最后提交地图':'等待设备连接';connection='未连接';mode='offline';}else if(!count){health='已连接 · 等待首个融合版本';connection='设备已连接';mode='paused';}else if(state.failure){health='运行异常 · 保留最后提交地图';connection='运行异常';mode='paused';}else if(paused){health='等待新地图 · 当前显示最后提交版本';connection='更新暂停';mode='paused';}else{health='正在接收已融合地图';connection='实时连接';mode='live';}
 $('connectionText').textContent=connection;$('connection').dataset.state=mode;$('healthDot').dataset.state=mode;$('health').textContent=health;$('emptyMessage').textContent=alive?'设备已连接，正在等待第一版融合地图。':'连接设备后，融合完成的三维地图将在此呈现。';$('commitHz').textContent=snapshot||!alive?'—':metrics.display_hz_10s.toFixed(1);$('version').textContent=version;$('count').textContent=count.toLocaleString();$('hz').textContent=metrics.fusion_commit_hz_10s===null?'—':metrics.fusion_commit_hz_10s.toFixed(1);const age=sourceAge===null||snapshot?null:sourceAge+now-lastMapAt;$('age').textContent=age===null?'—':age>9999?(age/1000).toFixed(1)+' s':age.toFixed(0);$('age').nextElementSibling.textContent=age>9999?'':'ms';$('footerNote').textContent=snapshot?'离线结果 · 无实时更新':'只展示已完成融合的地图';$('cameraStatus').textContent=snapshot?'离线':!lastCameraAt?'未连接':now-lastCameraAt>2500?'最后画面':'实时参考';
 if(!snapshot){const bins=Array.from({length:30},(_,i)=>events.filter(e=>e.at>now-(30-i)*1000&&e.at<=now-(29-i)*1000).length),top=Math.max(10,...bins);$('rateLine').setAttribute('d',bins.map((n,i)=>(i?'L':'M')+(i*248/29).toFixed(1)+','+(42-n/top*38).toFixed(1)).join(' '));}else $('rateLine').setAttribute('d','');
 if(count&&version!==lastExtentVersion&&now-lastExtent>5000){updateExtent();lastExtent=now;}const mapCapacity=snapshot?limit:Number(state.map_capacity)||1000000;$('capacityBar').max=mapCapacity;$('capacityBar').value=count;$('capacityText').textContent=(count/10000).toFixed(1)+' / '+(mapCapacity/10000)+' 万';$('capacityNote').textContent=snapshot?'离线快照 · 浏览器支持 200 万点':count>=mapCapacity?'已达地图容量上限 · 新区域无法继续扩展':count>=mapCapacity*.9?'接近容量上限 · 建议及时导出点云':'当前板端容量 · '+(mapCapacity/10000)+' 万重建点';$('capacityNote').classList.toggle('warning',!snapshot&&count>=mapCapacity*.9);
 $('details').textContent=JSON.stringify({'连接状态':connection,'地图版本':version,'重建点数':count,'容量上限':mapCapacity,'实际融合更新_Hz':metrics.fusion_commit_hz_10s,'数据时延估计_ms':age===null?null:Math.round(age),'连接往返_ms':snapshot?null:clock?Math.round(clock.rtt):null,'运行异常':state.failure||null},null,2);
}

// Translate camera and orbit target together; never change geometry or device pose.
const moveKeys=new Set();let moveTime=0;
canvas.tabIndex=0;canvas.addEventListener('pointerdown',()=>canvas.focus({preventScroll:true}));
function clearMovement(){moveKeys.clear();moveTime=0;}
window.addEventListener('blur',clearMovement);document.addEventListener('visibilitychange',clearMovement);
document.addEventListener('focusin',e=>{if(e.target!==canvas)clearMovement();});
document.addEventListener('keyup',e=>moveKeys.delete(e.code));
document.addEventListener('keydown',e=>{if(!['KeyW','KeyA','KeyS','KeyD'].includes(e.code)||e.ctrlKey||e.altKey||e.metaKey||document.querySelector('dialog[open]')||/INPUT|TEXTAREA|SELECT|BUTTON/.test(e.target.tagName)||e.target.isContentEditable)return;e.preventDefault();if(count){setFollow(false);moveKeys.add(e.code);}});
function moveView(now){const dt=moveTime?Math.min((now-moveTime)/1000,.05):0;moveTime=now;if(!moveKeys.size||!count||document.querySelector('dialog[open]')){moveKeys.clear();return;}
 const forward=camera.getWorldDirection(new THREE.Vector3()),right=new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld,0),move=new THREE.Vector3();
 move.addScaledVector(forward,Number(moveKeys.has('KeyW'))-Number(moveKeys.has('KeyS')));move.addScaledVector(right,Number(moveKeys.has('KeyD'))-Number(moveKeys.has('KeyA')));
 if(move.lengthSq()){move.normalize().multiplyScalar(dt*THREE.MathUtils.clamp(camera.position.distanceTo(controls.target)*.4,.2,8));camera.position.add(move);controls.target.add(move);}
}

function animate(now){requestAnimationFrame(animate);if(document.hidden)return;apply(now);marker.visible=!snapshot&&now-lastPoseAt<1500;const target=now-400;for(let i=1;i<poses.length;i++){const a=poses[i-1],b=poses[i];if(a.at<=target&&target<=b.at&&b.at-a.at<=300){const t=(target-a.at)/(b.at-a.at),v=new THREE.Vector3(...a.xyz).lerp(new THREE.Vector3(...b.xyz),t);marker.position.copy(v);if(follow&&a.rotation&&b.rotation&&marker.visible){const rotation=a.rotation.clone().slerp(b.rotation,t),forward=new THREE.Vector3(0,0,1).applyQuaternion(rotation),up=new THREE.Vector3(0,-1,0).applyQuaternion(rotation);camera.up.copy(up);camera.position.copy(v).addScaledVector(forward,-1.2).addScaledVector(up,.3);controls.target.copy(v).addScaledVector(forward,2);}break;}}
 moveView(now);controls.enableDamping=!follow;controls.update();renderer.render(scene,camera);if(renderCommit){const submitted=performance.now();renderCommit.source_age_upper_ms+=submitted-renderCommit.at;renderCommit.at=submitted;if(lastMapAt)gaps.push(submitted-lastMapAt);lastMapAt=submitted;sourceAge=renderCommit.source_age_upper_ms;events.push(renderCommit);renderCommit=null;}frames.push(now);if(previousFrame){frameGaps.push(now-previousFrame);drawEvents.push({at:now,gap_ms:now-previousFrame});if(drawEvents.length>1000)drawEvents.shift();}previousFrame=now;while(frames.length&&frames[0]<now-10000)frames.shift();if(frameGaps.length>10000)frameGaps.shift();
 if(now-lastReport>1000){const telemetry=mapTelemetry({state,session,now,lastStatusAt,lastMapAt,events,lastReport,snapshot}),metrics={client_id:clientId,client_kind:snapshot?'offline_snapshot':telemetry.status_mode==='unknown'?'unclassified_browser':navigator.webdriver?'automated_browser':'interactive_browser',user_agent:navigator.userAgent,viewport:[innerWidth,innerHeight],device_pixel_ratio:devicePixelRatio,client_mono_ms:now,follow_delay_ms:400,board_clock_offset_ms:snapshot?null:clock?.offset,session,visible:!document.hidden,connected:now-lastStatusAt<2500&&lastStatusAt>0,version,map_count:count,display_mode:displayMode,snapshot,status_mode:telemetry.status_mode,fusion_commit_hz_10s:telemetry.fusion_commit_hz_10s,display_hz_10s:telemetry.display_hz_10s,display_gap_p95_ms:percentile(gaps,.95),display_gap_max_ms:gaps.length?Math.max(...gaps):null,source_age_upper_p95_ms:telemetry.live_source_age?percentile(events.map(e=>e.source_age_upper_ms),.95):null,frame_gap_p95_ms:percentile(frameGaps,.95),fps_10s:frames.length/10,skipped_versions:skipped,clock_rtt_ms:snapshot?null:clock?.rtt,source_age_scope:telemetry.live_source_age?'board source age plus estimated send-to-render with RTT half-width':'not applicable',new_versions:telemetry.new_versions,draw_intervals:drawEvents.splice(0),board:state};updateUI(now,metrics);fetch('/client-metrics',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(metrics)}).catch(()=>{});lastReport=now;}}
// Visibility changes may discard a pending update, never replay it as a fresh map.
document.addEventListener('visibilitychange',()=>{++visibilityEpoch;previousFrame=0;frames=[];pending=null;if(ack){ack();ack=null;}});
$('gridToggle').onchange();
colors();status();requestAnimationFrame(animate);
