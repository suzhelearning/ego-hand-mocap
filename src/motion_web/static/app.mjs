import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {nearestFrame, timelineRange} from './timeline.mjs';
import {createMeshGeometry} from './mesh.mjs';

const $ = id => document.getElementById(id);
const state = {ego: null, mocap: null, time: 0, range: [0, 0], playing: false};
const layers = {}, revisions = {ego: 0, mocap: 0};
let renderer, scene, camera, controls;
const sphere = new THREE.SphereGeometry(.004, 12, 8);
const cylinder = new THREE.CylinderGeometry(.0028, .0028, 1, 10);
const axis = new THREE.Vector3(0, 1, 0);
const offset = () => Number($('offset').value) || 0;
function notice(message, error = false) {
  $('notice').textContent = message;
  $('notice').classList.toggle('error', error);
}
function overview() {
  const loaded = ['ego', 'mocap'].filter(r => state[r]);
  $('empty').hidden = loaded.length > 0;
  $('demo-badge').hidden = !loaded.some(r => state[r].synthetic);
  if (!loaded.length) return notice('等待数据 · 可以先加载模拟动作体验双路叠加');
  const warnings = new Set(loaded.flatMap(r => state[r].warnings));
  if (loaded.some(r => state[r].synthetic)) warnings.add('含模拟数据，仅用于界面调试，非真实精度评测。');
  if (state.ego && state.mocap && state.ego.coordinate_system !== state.mocap.coordinate_system)
    warnings.add('两路坐标系不同，当前原样叠加不可直接比较；请先离线变换到相同坐标系。');
  if (state.ego && state.mocap) {
    if (state.ego.session_id && state.mocap.session_id && state.ego.session_id !== state.mocap.session_id)
      warnings.add('两路数据来自不同会话，不能作为同次动作进行评估。');
    if (!state.ego.timeline_id || state.ego.timeline_id !== state.mocap.timeline_id || offset() !== 0)
      warnings.add('两路未使用相同配对时间轴或存在人工偏移；世界坐标统一不代表时间同步。');
  }
  notice([...warnings].join('\n'));
}
function stop() {state.playing = false; $('play').textContent = '播放';}
function recalculate() {
  stop();
  state.range = timelineRange(state.ego, state.mocap, offset());
  state.time = state.range[0];
  const [start, end] = state.range;
  $('timeline').min = start; $('timeline').max = end;
  $('timeline').disabled = end <= start;
  $('play').disabled = end <= start;
  $('range-start').textContent = `${start.toFixed(3)} s`;
  $('range-end').textContent = `${end.toFixed(3)} s`;
  overview();
}
function clearLayer(role) {
  if (!layers[role]) return;
  scene.remove(layers[role].group);
  layers[role].mesh?.geometry.dispose();
  layers[role].material.dispose();
  delete layers[role];
}
function setData(role, data, name) {
  clearLayer(role);
  state[role] = data;
  const material = new THREE.MeshStandardMaterial({color: role === 'ego' ? 0x6cdece : 0xb29bff,
    roughness: .35, metalness: .12, transparent: role === 'mocap',
    opacity: role === 'ego' ? 1 : Number($('opacity').value), depthWrite: role === 'ego',
    depthTest: role === 'ego', side: THREE.DoubleSide});
  const group = new THREE.Group();
  let mesh = null;
  if (data.kind === 'mesh') {
    const geometry = createMeshGeometry(THREE, data);
    mesh = new THREE.Mesh(geometry, material);
    mesh.frustumCulled = false;
    group.add(mesh);
  }
  const points = mesh ? [] : data.frames[0].points.map(() => {
    const mesh = new THREE.Mesh(sphere, material); group.add(mesh); return mesh;
  });
  const bones = data.bones.map(() => {
    const mesh = new THREE.Mesh(cylinder, material); group.add(mesh); return mesh;
  });
  // Ground-truth overlay intentionally stays visible through the solid EGO layer.
  if (role === 'mocap') group.traverse(obj => {obj.renderOrder = 2;});
  scene.add(group); layers[role] = {group, material, points, bones, mesh};
  $(role + '-info').textContent = `${name}\n${data.frames.length} 帧 · ${data.frames[0].points.length} 点 · ${data.frames.at(-1).time_s.toFixed(3)} s\n${data.coordinate_system} · ${data.kind} · m`;
  recalculate(); fitView();
}
function fitView() {
  const bounds = new THREE.Box3();
  for (const role of ['ego', 'mocap']) {
    const data = state[role]; if (!data) continue;
    for (const frame of data.frames) if (frame.valid)
      for (const point of frame.points) if (point) bounds.expandByPoint(new THREE.Vector3(...point));
  }
  const center = bounds.isEmpty() ? new THREE.Vector3(0, .05, 0) : bounds.getCenter(new THREE.Vector3());
  const size = bounds.isEmpty() ? .6 : Math.max(.15, bounds.getSize(new THREE.Vector3()).length());
  controls.target.copy(center);
  const distance = size / Math.min(1, camera.aspect) * 1.25;
  camera.position.copy(center).add(new THREE.Vector3(.35, .25, 1).normalize().multiplyScalar(distance));
  camera.near = Math.max(.0001, distance / 10000); camera.far = Math.max(100, distance * 20);
  camera.updateProjectionMatrix(); controls.update();
}
function updateLayer(role) {
  const data = state[role], layer = layers[role];
  if (!data || !layer) return `${role.toUpperCase()} —`;
  const shift = role === 'mocap' ? offset() : 0;
  const idx = nearestFrame(data, state.time, shift);
  const frame = idx < 0 ? null : data.frames[idx];
  const delta = frame ? frame.time_s + shift - state.time : 0;
  const tolerance = Math.max(1, Math.min(1000, Number($('tolerance').value) || 100)) / 1000;
  layer.group.visible = $(role + '-visible').checked && !!frame?.valid && Math.abs(delta) <= tolerance;
  if (!frame) return `${role.toUpperCase()} 超出范围`;
  if (!frame.valid) return `${role.toUpperCase()} #${idx + 1} 无效帧`;
  if (Math.abs(delta) > tolerance) return `${role.toUpperCase()} 无邻近样本`;
  if (layer.mesh && layer.lastFrame !== idx) {
    const positions = layer.mesh.geometry.attributes.position;
    frame.points.forEach((point, i) => positions.setXYZ(i, ...point));
    positions.needsUpdate = true;
    layer.mesh.geometry.computeVertexNormals();
    layer.lastFrame = idx;
  }
  for (let i = 0; i < layer.points.length; i++) {
    const p = frame.points[i], mesh = layer.points[i];
    mesh.visible = p !== null; if (p) mesh.position.set(...p);
  }
  for (let i = 0; i < layer.bones.length; i++) {
    const [a, b] = data.bones[i].map(j => frame.points[j]), mesh = layer.bones[i];
    mesh.visible = !!a && !!b;
    if (!mesh.visible) continue;
    const start = new THREE.Vector3(...a), end = new THREE.Vector3(...b), direction = end.clone().sub(start);
    const length = direction.length(); mesh.visible = length > 1e-9;
    if (!mesh.visible) continue;
    mesh.position.copy(start.add(end).multiplyScalar(.5));
    mesh.scale.set(1, length, 1); mesh.quaternion.setFromUnitVectors(axis, direction.normalize());
  }
  return `${role.toUpperCase()} #${idx + 1} · Δt ${(delta * 1000).toFixed(1)} ms`;
}
async function fetchJSON(url, options) {
  const response = await fetch(url, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
function assetQuery(file) {
  const query = new URLSearchParams({path: file.path});
  if (file.version) query.set('version', file.version);
  if (file.stage) query.set('stage', file.stage);
  return query.toString();
}
async function load(role, file) {
  const revision = ++revisions[role];
  try {
    const path = $(role + '-select').value;
    if (!file && !path) throw new Error('请先选择文件。');
    if (file && file.size > 64 * 1024 * 1024) throw new Error('文件超过 64 MiB，请裁剪动作片段。');
    notice(`正在加载 ${role.toUpperCase()} …`);
    const request = new URLSearchParams(file ? {name: file.name} : path);
    request.set('target', $('target-frame').value);
    if ($('alignment-version').value) {
      const selected = JSON.parse($('alignment-version').value);
      request.set('alignment_version', selected.version);
      request.set('alignment_session', selected.session_id);
    }
    const data = file
      ? await fetchJSON('/api/upload?' + request, {method: 'POST', body: file})
      : await fetchJSON('/api/load?' + request);
    if (revision !== revisions[role]) return;
    const query = new URLSearchParams(path);
    const label = [query.get('path'), query.get('stage'), query.get('version')].filter(Boolean).join(' · ');
    setData(role, data, file?.name || label);
  } catch (error) {
    if (revision === revisions[role]) notice(`${role.toUpperCase()} 加载失败：${error.message}`, true);
  } finally {$(role + '-file').value = '';}
}
function step(direction) {
  stop();
  const role = state.ego ? 'ego' : 'mocap', data = state[role];
  if (!data) return;
  const shift = role === 'mocap' ? offset() : 0;
  const times = data.frames.map(f => f.time_s + shift);
  const target = direction > 0 ? times.find(t => t > state.time + 1e-9) : times.findLast(t => t < state.time - 1e-9);
  if (target !== undefined) state.time = target;
}
async function refreshCatalog() {
  $('refresh').disabled = true;
  try {
    const {files, root} = await fetchJSON('/api/files');
    $('data-root').textContent = root || 'data/';
    const alignment = $('alignment-version'), oldAlignment = alignment.value;
    alignment.replaceChildren(new Option('同会话唯一完成版本（自动）', ''));
    for (const file of files.filter(f => f.asset_type === 'mocap_aligned_h5' && f.version))
      alignment.append(new Option(file.session_id + ' · ' + file.version,
        JSON.stringify({session_id: file.session_id, version: file.version})));
    if ([...alignment.options].some(o => o.value === oldAlignment)) alignment.value = oldAlignment;
    for (const role of ['ego', 'mocap']) {
      const select = $(role + '-select'), previous = select.value;
      select.replaceChildren(new Option(role === 'ego' ? '选择 EGO 视觉结果' : '选择 MOCAP 数据 / 处理版本', ''));
      const kinds = role === 'ego' ? ['ego_hand_motion', 'ego_vision_h5']
        : ['mocap_raw_h5', 'mocap_aligned_h5', 'mocap_mano_h5'];
      const groups = new Map();
      for (const file of files.filter(f => kinds.includes(f.asset_type))) {
        const session = file.session_id || ('未分配 · ' + (file.record_id || '历史数据'));
        if (!groups.has(session)) {
          const group = document.createElement('optgroup'); group.label = session;
          groups.set(session, group); select.append(group);
        }
        const label = {ego_vision_h5: 'EGO 视觉', ego_hand_motion: 'EGO 手部 JSON',
          mocap_mano_h5: 'MANO 网格', mocap_aligned_h5: '时间对齐刚体', mocap_raw_h5: '原始 marker / 刚体'}[file.asset_type];
        groups.get(session).append(new Option([label, file.version || file.run_id,
          file.path.split('/').at(-1)].filter(Boolean).join(' · '), assetQuery(file)));
      }
      if ([...select.options].some(o => o.value === previous)) select.value = previous;
      if (!state[role]) $(role + '-info').textContent = groups.size ? '已发现会话数据，请选择具体结果和版本。'
        : role === 'ego' ? '尚无 EGO 视觉动作结果。原始 ego.h5 图像/IMU 不作为手部轨迹。' : '尚未发现动捕结果。';
    }
    if (!state.ego && !state.mocap) notice('数据目录已更新，请按会话选择文件和处理版本；不会自动选择或配对不同录制。');
  } catch (error) {notice('无法读取数据目录：' + error.message, true);}
  finally {$('refresh').disabled = false;}
}
async function init() {
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(40, 1, .001, 1000);
  renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  $('viewport').prepend(renderer.domElement);
  renderer.domElement.setAttribute('aria-label', 'EGO 与 MOCAP 三维动作叠加视图');
  controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xe2f7ff, 0x26384a, 2.5));
  const light = new THREE.DirectionalLight(0xffffff, 3); light.position.set(1, 3, 4); scene.add(light);
  const grid = new THREE.GridHelper(2, 40, 0x425b72, 0x233347); grid.position.y = -.18; scene.add(grid);
  scene.add(new THREE.AxesHelper(.1));
  new ResizeObserver(() => {
    const {width, height} = $('viewport').getBoundingClientRect();
    renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix();
  }).observe($('viewport'));
  fitView();
  for (const role of ['ego', 'mocap']) {
    $(role + '-file').addEventListener('change', event => {if (event.target.files[0]) load(role, event.target.files[0]);});
    $(role + '-load').onclick = () => load(role);
    $(role + '-clear').onclick = () => {
      ++revisions[role]; clearLayer(role); state[role] = null;
      $(role + '-info').textContent = '尚未加载'; recalculate(); fitView();
    };
  }
  $('demo').onclick = async () => {
    const revision = {ego: ++revisions.ego, mocap: ++revisions.mocap};
    try {
      const data = await fetchJSON('/api/demo');
      if (revisions.ego !== revision.ego || revisions.mocap !== revision.mocap) return;
      $('offset').value = 0;
      for (const role of ['ego', 'mocap']) setData(role, data[role], data[role].name);
    } catch (error) {notice(error.message, true);}
  };
  $('opacity').oninput = () => {
    const value = Number($('opacity').value); $('opacity-value').textContent = `${Math.round(value * 100)}%`;
    if (layers.mocap) layers.mocap.material.opacity = value;
  };
  $('offset').onchange = () => {
    $('offset').value = Math.max(-3600, Math.min(3600, offset())); recalculate();
  };
  $('reset').onclick = fitView;
  $('play').onclick = () => {
    if (state.playing) stop();
    else {if (state.time >= state.range[1]) state.time = state.range[0]; state.playing = true; $('play').textContent = '暂停';}
  };
  $('prev').onclick = () => step(-1); $('next').onclick = () => step(1);
  $('timeline').oninput = () => {stop(); state.time = Number($('timeline').value);};
  let last = performance.now();
  renderer.setAnimationLoop(now => {
    const elapsed = Math.min((now - last) / 1000, .1); last = now;
    if (state.playing) {
      state.time += elapsed * Number($('speed').value);
      if (state.time > state.range[1]) {
        if ($('loop').checked) state.time = state.range[0];
        else {state.time = state.range[1]; stop();}
      }
    }
    $('timeline').value = state.time;
    $('time').textContent = `${state.time.toFixed(3)} / ${state.range[1].toFixed(3)} s`;
    $('frame-info').textContent = ['ego', 'mocap'].map(updateLayer).join('\n');
    controls.update(); renderer.render(scene, camera);
  });
  for (const id of ['target-frame', 'alignment-version']) $(id).onchange = () => {
    for (const role of ['ego', 'mocap']) {
      ++revisions[role]; clearLayer(role); state[role] = null;
      $(role + '-info').textContent = '转换设置已更改，请重新加载所选数据。';
    }
    recalculate();
    notice('转换设置已更改，请重新加载两路数据；原始文件不变。');
  };
  $('refresh').onclick = refreshCatalog;
  await refreshCatalog();
}
init().catch(error => notice('三维视图初始化失败，请检查浏览器 WebGL 支持：' + error.message, true));
