"""
Cesium 3D 地圖組件
使用 Cesium.js + QWebEngineView 實現 3D 互動式地圖

特色：
- 完整 3D 地形 + 衛星影像
- 固定翼三階段路徑含高度視覺化（起飛爬升 / 掃描 / 降落）
- 多旋翼路徑等高度顯示
- SITL 即時飛行位置更新預留接口
- 禁航區 3D 柱體顯示
- 與 2D MapWidget 相同的 pyqt:// URL bridge
"""

import json
import math
from typing import List, Tuple, Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings
from PyQt6.QtCore import pyqtSignal, QUrl, Qt
from urllib.parse import unquote

from utils.logger import get_logger

logger = get_logger()

# ── 顏色常數（與 MapWidget 一致）─────────────────────────────────────
_REGION_COLORS = ['#08EC91', '#FF6B35', '#3D87FF', '#FFD700', '#FF69B4', '#9B59B6']
_DRONE_COLORS  = ['#E53935', '#1E88E5', '#43A047', '#FB8C00',
                  '#8E24AA', '#00ACC1', '#F4511E', '#3949AB']


# ══════════════════════════════════════════════════════════════════════
#  Cesium HTML 模板
# ══════════════════════════════════════════════════════════════════════
_CESIUM_HTML = r"""<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="utf-8">
<title>AeroPlan Studio — 3D View</title>
<script src="__CESIUM_JS__"></script>
<link href="__CESIUM_CSS__" rel="stylesheet">
<style>
  html,body{width:100%;height:100%;margin:0;padding:0;overflow:hidden;background:#0d1117;font-family:sans-serif}
  #cesiumContainer{position:absolute;top:0;left:0;width:100%;height:100%}

  /* 右上工具列 */
  #toolbar{
    position:absolute;top:8px;right:8px;z-index:999;
    display:flex;flex-direction:column;gap:4px;
  }
  #toolbar button{
    background:rgba(30,40,55,0.88);color:#e0e6f0;border:1px solid #3a4a60;
    border-radius:6px;padding:5px 10px;cursor:pointer;font-size:11px;
    transition:background 0.15s;
  }
  #toolbar button:hover{background:rgba(30,90,180,0.85);}

  /* UAV 狀態列（SITL 用） */
  #sitlStatus{
    position:absolute;bottom:36px;left:8px;z-index:999;
    background:rgba(10,15,25,0.82);color:#4fc3f7;
    padding:5px 12px;border-radius:6px;font-size:11px;
    font-family:monospace;display:none;border:1px solid #1565C0;
  }

  /* 碰撞預警面板 */
  #collisionAlert{
    position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);z-index:1000;
    padding:10px 24px;border-radius:8px;font-size:14px;font-weight:bold;
    font-family:sans-serif;display:none;text-align:center;
    pointer-events:none;animation:alertPulse 0.8s infinite;
  }
  #collisionAlert.warn{
    background:rgba(255,193,7,0.88);color:#000;border:2px solid #FFC107;
  }
  #collisionAlert.danger{
    background:rgba(244,67,54,0.92);color:#FFF;border:2px solid #F44336;
  }
  @keyframes alertPulse{
    0%,100%{opacity:1} 50%{opacity:0.6}
  }

  /* 模式標籤 */
  #modeLabel{
    position:absolute;top:8px;left:8px;z-index:999;
    background:rgba(10,15,25,0.72);color:#80cbc4;
    padding:3px 10px;border-radius:5px;font-size:10px;pointer-events:none;
  }

  /* 左下圖例 */
  #legend{
    position:absolute;bottom:36px;right:8px;z-index:999;
    background:rgba(10,15,25,0.82);color:#cfd8dc;
    padding:6px 12px;border-radius:6px;font-size:10px;
    border:1px solid #37474f;display:none;max-width:160px;
  }
  #legend .item{display:flex;align-items:center;gap:6px;margin:2px 0}
  #legend .dot{width:10px;height:10px;border-radius:50%;flex-shrink:0}

  /* 圖層過濾器面板 */
  #layerPanel{
    position:absolute;top:8px;left:8px;z-index:999;
    background:rgba(10,15,25,0.88);color:#cfd8dc;
    padding:8px 12px;border-radius:8px;font-size:11px;
    border:1px solid #3a4a60;min-width:130px;
    user-select:none;
  }
  #layerPanel .lp-title{
    font-size:12px;font-weight:bold;color:#90caf9;
    margin-bottom:6px;border-bottom:1px solid #37474f;padding-bottom:4px;
  }
  #layerPanel label{
    display:flex;align-items:center;gap:6px;
    padding:3px 0;cursor:pointer;transition:color 0.15s;
  }
  #layerPanel label:hover{color:#fff}
  #layerPanel input[type="checkbox"]{
    accent-color:#42a5f5;width:14px;height:14px;cursor:pointer;
  }
</style>
</head>
<body>
<div id="cesiumContainer"></div>

<div id="layerPanel">
  <div class="lp-title">📋 圖層過濾</div>
  <label><input type="checkbox" id="lyr_aircraft" checked onchange="toggleLayer('aircraft')">✈ 飛機</label>
  <label><input type="checkbox" id="lyr_trail" checked onchange="toggleLayer('trail')">〰 軌跡</label>
  <label><input type="checkbox" id="lyr_path" checked onchange="toggleLayer('path')">📍 路徑</label>
  <label><input type="checkbox" id="lyr_waypoint" onchange="toggleLayer('waypoint')">🔢 航點標記</label>
  <label><input type="checkbox" id="lyr_imagery" checked onchange="toggleLayer('imagery')">🗺️ 地圖</label>
  <label><input type="checkbox" id="lyr_dem" checked onchange="toggleLayer('dem')">🏔️ DEM</label>
</div>

<div id="toolbar">
  <button onclick="flyToScene()">🎯 飛向路徑</button>
  <button onclick="toggleClamp()">🏔️ 地形吸附</button>
  <button onclick="setMode3D()">🌐 3D</button>
  <button onclick="setMode2D()">🗺️ 2D</button>
  <button onclick="setModeColumbus()">📐 哥倫布</button>
  <button onclick="clearUAV()">✖ 清除 UAV</button>
</div>

<div id="sitlStatus"></div>
<div id="collisionAlert"></div>
<div id="legend" id="legend"></div>

<script>
// ── Cesium Ion Token（使用者可設定，空字串 = 免費基本模式）────────────
Cesium.Ion.defaultAccessToken = '__CESIUM_TOKEN__';

// ── 初始化 Viewer ─────────────────────────────────────────────────────
const viewer = new Cesium.Viewer('cesiumContainer', {
  terrainProvider: new Cesium.EllipsoidTerrainProvider(),  // 無 Token 用平面地形
  baseLayerPicker: false,
  navigationHelpButton: false,
  homeButton: false,
  sceneModePicker: false,
  geocoder: false,
  timeline: true,      // SITL: 時間軸（預留）
  animation: true,     // SITL: 動畫控制（預留）
  fullscreenButton: false,
  infoBox: true,
  selectionIndicator: false,
  shadows: false,
  skyBox: false,
  skyAtmosphere: new Cesium.SkyAtmosphere(),
  requestRenderMode: false,
});

// 深色背景
viewer.scene.backgroundColor = Cesium.Color.fromCssColorString('#0d1117');

// 加入 Google 衛星影像（與 2D 地圖一致）
try {
  viewer.imageryLayers.removeAll();
  viewer.imageryLayers.addImageryProvider(
    new Cesium.UrlTemplateImageryProvider({
      url: 'https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
      subdomains: ['0','1','2','3'],
      credit: 'Google Satellite',
      maximumLevel: 22,
    })
  );
} catch(e) {
  console.warn('Google 衛星影像載入失敗，使用預設底圖', e);
}

// 抑制 imagery tile 失敗的紅色錯誤訊息（避免 console 洗版）
viewer.scene.globe.tileLoadProgressEvent.addEventListener(function(){});
if (viewer._cesiumWidget && viewer._cesiumWidget._showRenderLoopErrors !== undefined) {
  viewer._cesiumWidget._showRenderLoopErrors = false;
}
// 攔截 imageryLayer 錯誤事件，只 warn 不丟例外
viewer.imageryLayers.layerAdded.addEventListener(function(layer){
  if (layer.imageryProvider && layer.imageryProvider.errorEvent) {
    layer.imageryProvider.errorEvent.addEventListener(function(err){
      // 只在 console 顯示一次，不再持續刷錯誤
      if (!window._tileWarned) {
        console.warn('部分圖磚載入失敗（網路或伺服器問題），已自動略過');
        window._tileWarned = true;
      }
    });
  }
});

// 嘗試載入世界地形（需要 Cesium Ion Token）
if (Cesium.Ion.defaultAccessToken && Cesium.Ion.defaultAccessToken !== '__CESIUM_TOKEN__') {
  try {
    viewer.terrainProvider = Cesium.createWorldTerrain({ requestWaterMask: false });
  } catch(e) {
    console.warn('地形載入失敗，使用平面模式');
  }
}

// ── 狀態變數 ─────────────────────────────────────────────────────────
let pathEntities   = [];   // 飛行路徑
let markerEntities = [];   // 起終點 / 航點標記
let cornerEntities = [];   // 邊界角點
let polygonEntity  = null; // 邊界多邊形
let circleEntity   = null; // 圓心掃描範圍
let homeEntity     = null; // 起飛點
let uavEntity      = null; // 單機 UAV 即時位置（保留舊 API 相容）
let uavEntities    = {};   // 多機 UAV 字典：{ sysid: entity }
let nfzEntities    = [];   // 禁航區
let clampToGround  = false;
let legendItems    = [];   // 圖例資料
let wpLabelEntities = [];  // 航點標記 (WP1, WP2, ...)

// ── 圖層可見度控制 ───────────────────────────────────────────────
function toggleLayer(layer) {
  const checked = document.getElementById('lyr_' + layer).checked;
  switch(layer) {
    case 'aircraft':
      Object.values(uavEntities).forEach(e => {
        e.show = checked;
        if (e._wingChild) e._wingChild.show = checked;
      });
      break;
    case 'trail':
      Object.values(uavTrails).forEach(e => { e.show = checked; });
      break;
    case 'path':
      pathEntities.forEach(e => { e.show = checked; });
      markerEntities.forEach(e => { e.show = checked; });
      break;
    case 'waypoint':
      wpLabelEntities.forEach(e => { e.show = checked; });
      break;
    case 'imagery':
      for (var i = 0; i < viewer.imageryLayers.length; i++) {
        viewer.imageryLayers.get(i).show = checked;
      }
      break;
    case 'dem':
      if (demOverlayEntity) demOverlayEntity.show = checked;
      if (demOutlineEntity) demOutlineEntity.show = checked;
      demLabelEntities.forEach(e => { e.show = checked; });
      if (demTerrainPrimitive) demTerrainPrimitive.show = checked;
      break;
  }
}

// ── 工具函數 ─────────────────────────────────────────────────────────
function hexColor(hex, alpha) {
  return Cesium.Color.fromCssColorString(hex).withAlpha(alpha !== undefined ? alpha : 0.92);
}

function toPts(arr) {
  // arr: [{lat, lon, alt}, ...]
  return arr.map(p => Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.alt || 0));
}

function midPoint(pts) {
  if (!pts.length) return null;
  return pts[Math.floor(pts.length / 2)];
}

// ── 路徑操作 ─────────────────────────────────────────────────────────
function clearPaths() {
  pathEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  pathEntities = [];
  markerEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  markerEntities = [];
  wpLabelEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  wpLabelEntities = [];
  document.getElementById('legend').style.display = 'none';
  legendItems = [];
}

/**
 * 加入一條 3D 折線
 * @param {string} ptsJson  JSON 陣列：[{lat,lon,alt}, ...]
 * @param {string} colorHex 十六進位顏色
 * @param {number} width    線寬
 * @param {string} label    路徑標籤（用於圖例、起終點 tooltip）
 * @param {boolean} dashed  是否虛線
 */
function addPath(ptsJson, colorHex, width, label, dashed, forceClamp) {
  const pts = JSON.parse(ptsJson);
  if (pts.length < 2) return;
  const positions = toPts(pts);
  const useClamp = (forceClamp === true) || clampToGround;
  const color = hexColor(colorHex);

  let material;
  if (dashed) {
    material = new Cesium.PolylineDashMaterialProperty({
      color: color,
      dashLength: 16.0,
    });
  } else {
    material = new Cesium.PolylineGlowMaterialProperty({
      glowPower: 0.08,
      color: color,
    });
  }

  const e = viewer.entities.add({
    name: label || 'path',
    polyline: {
      positions: positions,
      width: width || 10,
      material: material,
      clampToGround: useClamp,
      arcType: useClamp ? Cesium.ArcType.GEODESIC : Cesium.ArcType.NONE,
    }
  });
  pathEntities.push(e);

  // 起點標記
  const startE = viewer.entities.add({
    position: positions[0],
    point: {
      pixelSize: 10,
      color: Cesium.Color.LIMEGREEN,
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      heightReference: Cesium.HeightReference.NONE,
    },
    label: label ? {
      text: label + '\n起點',
      font: '11px "Segoe UI",sans-serif',
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      outlineWidth: 2,
      outlineColor: Cesium.Color.BLACK,
      fillColor: Cesium.Color.WHITE,
      pixelOffset: new Cesium.Cartesian2(0, -16),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
      show: false,  // hover 才顯示（避免雜亂）
    } : undefined,
  });
  const endE = viewer.entities.add({
    position: positions[positions.length - 1],
    point: {
      pixelSize: 10,
      color: Cesium.Color.TOMATO,
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      heightReference: Cesium.HeightReference.NONE,
    },
  });
  markerEntities.push(startE, endE);

  // 航點標記 (WP0, WP1, WP2, ...)
  const showWp = document.getElementById('lyr_waypoint').checked;
  for (var wi = 0; wi < positions.length; wi++) {
    var wpE = viewer.entities.add({
      position: positions[wi],
      point: {
        pixelSize: 7,
        color: hexColor(colorHex, 0.9),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 1.5,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: 'WP' + wi,
        font: 'bold 11px "Segoe UI",sans-serif',
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -14),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        scale: 0.9,
      },
      show: showWp,
    });
    wpLabelEntities.push(wpE);
  }

  // 更新圖例
  if (label) {
    legendItems.push({ color: colorHex, label: label });
    _updateLegend();
  }
}

function _updateLegend() {
  const el = document.getElementById('legend');
  if (!legendItems.length) { el.style.display = 'none'; return; }
  el.style.display = 'block';
  el.innerHTML = '<b style="font-size:11px;color:#90caf9">航線圖例</b>' +
    legendItems.map(item =>
      `<div class="item"><div class="dot" style="background:${item.color}"></div><span>${item.label}</span></div>`
    ).join('');
}

// ── 角點 / 邊界 ──────────────────────────────────────────────────────
function clearCorners() {
  cornerEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  cornerEntities = [];
  if (polygonEntity) { viewer.entities.remove(polygonEntity); polygonEntity = null; }
}

function setCorners(cornersJson) {
  clearCorners();
  const corners = JSON.parse(cornersJson);
  if (!corners.length) return;

  corners.forEach((c, i) => {
    const e = viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(c.lon, c.lat, 1),
      point: {
        pixelSize: 13,
        color: hexColor('#E53935'),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: String(i + 1),
        font: 'bold 11px sans-serif',
        pixelOffset: new Cesium.Cartesian2(0, -18),
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
    cornerEntities.push(e);
  });

  if (corners.length >= 3) {
    const degs = corners.flatMap(c => [c.lon, c.lat]);
    polygonEntity = viewer.entities.add({
      polygon: {
        hierarchy: Cesium.Cartesian3.fromDegreesArray(degs),
        material: hexColor('#2196F3', 0.15),
        outline: true,
        outlineColor: hexColor('#2196F3', 0.9),
        height: 0,
      },
    });
  }
}

// ── 圓形覆蓋 ─────────────────────────────────────────────────────────
function setCircleOverlay(lat, lon, radius, colorHex) {
  if (circleEntity) { viewer.entities.remove(circleEntity); circleEntity = null; }
  if (radius <= 0) return;
  circleEntity = viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(lon, lat, 0),
    ellipse: {
      semiMajorAxis: radius,
      semiMinorAxis: radius,
      material: hexColor(colorHex || '#2196F3', 0.1),
      outline: true,
      outlineColor: hexColor(colorHex || '#2196F3', 0.85),
      outlineWidth: 2,
      height: 0,
    },
    label: {
      text: '⊙ 掃描圓心',
      font: '11px sans-serif',
      fillColor: hexColor(colorHex || '#2196F3', 1.0),
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -20),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
}

// ── 起飛點 ───────────────────────────────────────────────────────────
function setHomePoint(lat, lon) {
  if (homeEntity) { viewer.entities.remove(homeEntity); homeEntity = null; }
  homeEntity = viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(lon, lat, 3),
    point: {
      pixelSize: 15,
      color: hexColor('#FF9800'),
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    label: {
      text: '🏠 Home',
      font: '12px sans-serif',
      fillColor: Cesium.Color.ORANGE,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -22),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
}

// ── 禁航區 NFZ ───────────────────────────────────────────────────────
function clearNFZ() {
  nfzEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  nfzEntities = [];
}

function addNFZPolygon(vertsJson, name) {
  const verts = JSON.parse(vertsJson);
  const degs  = verts.flatMap(v => [v.lon, v.lat]);
  const e = viewer.entities.add({
    name: name || 'NFZ',
    polygon: {
      hierarchy: Cesium.Cartesian3.fromDegreesArray(degs),
      material: Cesium.Color.RED.withAlpha(0.22),
      outline: true,
      outlineColor: Cesium.Color.RED.withAlpha(0.9),
      height: 0,
      extrudedHeight: 300,
    },
  });
  nfzEntities.push(e);
}

function addNFZCircle(lat, lon, radius, name) {
  const e = viewer.entities.add({
    name: name || 'NFZ',
    position: Cesium.Cartesian3.fromDegrees(lon, lat, 0),
    ellipse: {
      semiMajorAxis: radius,
      semiMinorAxis: radius,
      material: Cesium.Color.RED.withAlpha(0.22),
      outline: true,
      outlineColor: Cesium.Color.RED.withAlpha(0.9),
      height: 0,
      extrudedHeight: 300,
    },
  });
  nfzEntities.push(e);
}

// ── 相機操作 ─────────────────────────────────────────────────────────
function flyToScene() {
  const all = [...pathEntities, ...cornerEntities].filter(e => viewer.entities.contains(e));
  if (all.length) {
    viewer.flyTo(all, { offset: new Cesium.HeadingPitchRange(0, Cesium.Math.toRadians(-55), 0) });
  }
}

function flyToPos(lat, lon, alt, range) {
  viewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(lon, lat, (alt || 0) + (range || 600)),
    orientation: {
      heading: 0,
      pitch: Cesium.Math.toRadians(-50),
      roll: 0,
    },
    duration: 1.5,
  });
}

function setMode3D()      { viewer.scene.mode = Cesium.SceneMode.SCENE3D; }
function setMode2D()      { viewer.scene.mode = Cesium.SceneMode.SCENE2D; }
function setModeColumbus(){ viewer.scene.mode = Cesium.SceneMode.COLUMBUS_VIEW; }

function toggleClamp() {
  clampToGround = !clampToGround;
  console.log('[3D] 地形吸附:', clampToGround);
}

// ── SITL 即時 UAV 位置更新（多機支援）────────────────────────────────
// 呼叫方式：updateUAVPosition(lat, lon, alt, hdg, spd, sysid, mode, armed)
const _UAV_COLORS = ['#FFD700','#E53935','#43A047','#1E88E5','#8E24AA','#00ACC1','#FB8C00','#3949AB'];
const _PLANE_MODEL_URI = '__PLANE_MODEL_URI__';
const _COPTER_MODEL_URI = '__COPTER_MODEL_URI__';

// 多旋翼 SVG（X 型四軸 + 中央機身）
function _uavSvgCopter(colorHex) {
  return 'data:image/svg+xml,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">' +
    '<line x1="8" y1="8" x2="40" y2="40" stroke="#222" stroke-width="3"/>' +
    '<line x1="40" y1="8" x2="8" y2="40" stroke="#222" stroke-width="3"/>' +
    '<circle cx="8" cy="8" r="6" fill="' + colorHex + '" stroke="#222" stroke-width="1.5"/>' +
    '<circle cx="40" cy="8" r="6" fill="' + colorHex + '" stroke="#222" stroke-width="1.5"/>' +
    '<circle cx="8" cy="40" r="6" fill="' + colorHex + '" stroke="#222" stroke-width="1.5"/>' +
    '<circle cx="40" cy="40" r="6" fill="' + colorHex + '" stroke="#222" stroke-width="1.5"/>' +
    '<rect x="18" y="18" width="12" height="12" rx="2" fill="#FFF" stroke="#222" stroke-width="1.5"/>' +
    '<polygon points="24,12 28,20 20,20" fill="#E53935"/>' +  // 朝前指示三角
    '</svg>'
  );
}

// 固定翼 SVG（飛機俯視圖）
function _uavSvgPlane(colorHex) {
  return 'data:image/svg+xml,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">' +
    '<polygon points="24,2 28,22 46,28 46,32 28,30 28,40 34,44 34,46 24,44 14,46 14,44 20,40 20,30 2,32 2,28 20,22" ' +
    'fill="' + colorHex + '" stroke="#1a1a1a" stroke-width="1.5" stroke-linejoin="round"/>' +
    '<circle cx="24" cy="14" r="2" fill="#FFF"/>' +
    '</svg>'
  );
}

function _uavSvg(colorHex, vtype) {
  const vt = (vtype || '').toUpperCase();
  if (vt.indexOf('PLANE') >= 0 || vt.indexOf('FIXED') >= 0 || vt.indexOf('VTOL') >= 0) {
    return _uavSvgPlane(colorHex);
  }
  return _uavSvgCopter(colorHex);
}

if (typeof uavTrails === 'undefined') { var uavTrails = {}; var uavTrailPts = {}; }
function _appendTrail(sysid, lat, lon, alt, colorHex) {
  if (!uavTrailPts[sysid]) uavTrailPts[sysid] = [];
  const arr = uavTrailPts[sysid];
  arr.push(lon, lat, alt || 0);
  // 限制最多 1500 點 (= 500 個 3D 點)
  if (arr.length > 1500 * 3) arr.splice(0, 3);
  if (arr.length < 6) return;
  if (uavTrails[sysid]) {
    try { viewer.entities.remove(uavTrails[sysid]); } catch(_) {}
  }
  uavTrails[sysid] = viewer.entities.add({
    name: 'UAV-trail-' + sysid,
    polyline: {
      positions: Cesium.Cartesian3.fromDegreesArrayHeights(arr),
      width: 4,
      material: new Cesium.PolylineGlowMaterialProperty({
        glowPower: 0.15,
        color: Cesium.Color.fromCssColorString(colorHex).withAlpha(0.9),
      }),
      arcType: Cesium.ArcType.NONE,
    }
  });
}

// ── 碰撞預警系統 ────────────────────────────────────────────────
const COLLISION_WARN_DIST = 80;   // 預警距離 (m) — 黃色
const COLLISION_DANGER_DIST = 30; // 危險距離 (m) — 紅色
let collisionWarnings = {};       // {key: entity} 預警連線
let _lastCollisionAudio = 0;

function _uavDistance(pos1, pos2) {
  return Cesium.Cartesian3.distance(pos1, pos2);
}

function _checkCollisions() {
  // 清除舊預警
  Object.values(collisionWarnings).forEach(e => {
    try { viewer.entities.remove(e); } catch(_) {}
  });
  collisionWarnings = {};

  const ids = Object.keys(uavEntities);
  if (ids.length < 2) {
    document.getElementById('collisionAlert').style.display = 'none';
    return;
  }

  let worstLevel = 0; // 0=safe, 1=warn, 2=danger
  let worstDist = Infinity;
  let worstPair = '';

  for (let i = 0; i < ids.length; i++) {
    for (let j = i + 1; j < ids.length; j++) {
      const eA = uavEntities[ids[i]], eB = uavEntities[ids[j]];
      if (!eA || !eB || !eA.position || !eB.position) continue;
      const pA = eA.position.getValue(Cesium.JulianDate.now());
      const pB = eB.position.getValue(Cesium.JulianDate.now());
      if (!pA || !pB) continue;
      const dist = _uavDistance(pA, pB);

      if (dist < COLLISION_WARN_DIST) {
        const isDanger = dist < COLLISION_DANGER_DIST;
        const level = isDanger ? 2 : 1;
        if (level > worstLevel || (level === worstLevel && dist < worstDist)) {
          worstLevel = level;
          worstDist = dist;
          worstPair = 'UAV' + ids[i] + '↔UAV' + ids[j];
        }
        // 繪製預警連線
        const lineColor = isDanger
          ? Cesium.Color.RED.withAlpha(0.9)
          : Cesium.Color.YELLOW.withAlpha(0.7);
        const key = ids[i] + '-' + ids[j];
        collisionWarnings[key] = viewer.entities.add({
          polyline: {
            positions: [pA, pB],
            width: isDanger ? 5 : 3,
            material: new Cesium.PolylineDashMaterialProperty({
              color: lineColor,
              dashLength: isDanger ? 8 : 14,
            }),
            arcType: Cesium.ArcType.NONE,
          },
          label: {
            text: '⚠ ' + dist.toFixed(1) + 'm',
            font: 'bold 13px sans-serif',
            fillColor: isDanger ? Cesium.Color.RED : Cesium.Color.YELLOW,
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 3,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            position: Cesium.Cartesian3.midpoint(pA, pB, new Cesium.Cartesian3()),
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            showBackground: true,
            backgroundColor: new Cesium.Color(0, 0, 0, 0.7),
          },
          position: Cesium.Cartesian3.midpoint(pA, pB, new Cesium.Cartesian3()),
        });
      }
    }
  }

  // 更新警告面板
  const alertEl = document.getElementById('collisionAlert');
  if (worstLevel > 0) {
    alertEl.style.display = 'block';
    alertEl.className = worstLevel === 2 ? 'danger' : 'warn';
    alertEl.textContent = (worstLevel === 2 ? '🚨 碰撞危險' : '⚠️ 接近預警') +
      '  ' + worstPair + '  ' + worstDist.toFixed(1) + 'm';
  } else {
    alertEl.style.display = 'none';
  }
}

function updateUAVPosition(lat, lon, alt, headingDeg, speedMs, sysid, mode, armed, vehicleType, pitchDeg, rollDeg) {
  sysid = sysid || 1;
  const pos = Cesium.Cartesian3.fromDegrees(lon, lat, alt || 0);
  const color = _UAV_COLORS[(sysid - 1) % _UAV_COLORS.length];
  _appendTrail(sysid, lat, lon, alt, color);
  const _vt = (vehicleType || '').toUpperCase();
  const isPlane = _vt.indexOf('PLANE') >= 0 || _vt.indexOf('FIXED') >= 0
                  || _vt.indexOf('VTOL') >= 0;   // VTOL (QuadPlane) 使用固定翼模型
  const icon = isPlane ? '✈' : '🚁';
  const labelTxt = icon + ' UAV' + sysid + (mode ? ' [' + mode + ']' : '') + (armed ? ' ●ARM' : '');

  // 計算 orientation（含姿態：heading + pitch + roll）
  const isPlaneGlb = isPlane && _PLANE_MODEL_URI && _PLANE_MODEL_URI.length > 0;
  const isCopterGlb = !isPlane && _COPTER_MODEL_URI && _COPTER_MODEL_URI.length > 0;
  let headingOffset = 0, pitchOffset = 0;
  if (isPlaneGlb) { headingOffset = 90; pitchOffset = 90; }
  if (isCopterGlb) { headingOffset = -90; pitchOffset = 90; }
  const hpr = new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians((headingDeg || 0) + headingOffset),
    Cesium.Math.toRadians((pitchDeg || 0) + pitchOffset),
    Cesium.Math.toRadians(rollDeg || 0)
  );
  const orientation = Cesium.Transforms.headingPitchRollQuaternion(pos, hpr);

  let entity = uavEntities[sysid];
  if (!entity) {
    const useGlbModel = (isPlane && _PLANE_MODEL_URI && _PLANE_MODEL_URI.length > 0)
                     || (!isPlane && _COPTER_MODEL_URI && _COPTER_MODEL_URI.length > 0);
    const entityOpts = {
      name: 'UAV-' + sysid,
      position: new Cesium.ConstantPositionProperty(pos),
      orientation: new Cesium.ConstantProperty(orientation),
      billboard: {
        image: _uavSvg(color, vehicleType),
        scale: 1.1,
        rotation: Cesium.Math.toRadians(-(headingDeg || 0)),
        alignedAxis: Cesium.Cartesian3.UNIT_Z,
        heightReference: Cesium.HeightReference.NONE,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        distanceDisplayCondition: useGlbModel
          ? new Cesium.DistanceDisplayCondition(2000.0, 1.0e8)
          : new Cesium.DistanceDisplayCondition(800.0, 1.0e8),
      },
      label: {
        text: labelTxt,
        font: 'bold 12px "Segoe UI",sans-serif',
        fillColor: Cesium.Color.fromCssColorString(color),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2.5,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -32),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    };

    if (isPlaneGlb) {
      console.log('[3D] 載入固定翼 GLB 模型:', _PLANE_MODEL_URI);
      entityOpts.model = {
        uri: _PLANE_MODEL_URI,
        minimumPixelSize: 32,
        maximumScale: 50,
        scale: 0.01,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 2000.0),
      };
    } else if (isCopterGlb) {
      console.log('[3D] 載入多旋翼 GLB 模型:', _COPTER_MODEL_URI);
      entityOpts.model = {
        uri: _COPTER_MODEL_URI,
        minimumPixelSize: 32,
        maximumScale: 50,
        scale: 0.01,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 2000.0),
      };
    } else if (isPlane) {
      entityOpts.box = {
        dimensions: new Cesium.Cartesian3(2.0, 12.0, 1.2),
        material: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
        outline: true, outlineColor: Cesium.Color.BLACK,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 800.0),
      };
    } else {
      entityOpts.ellipsoid = {
        radii: new Cesium.Cartesian3(8, 8, 2),
        material: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
        outline: true, outlineColor: Cesium.Color.BLACK,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 800.0),
      };
    }

    entity = viewer.entities.add(entityOpts);
    uavEntities[sysid] = entity;
    if (sysid === 1) uavEntity = entity;  // 舊 API 相容

    // 為固定翼 fallback 加上機翼（無 GLB 時）
    if (isPlane && !useGlbModel) {
      const wingEntity = viewer.entities.add({
        position: new Cesium.ConstantPositionProperty(pos),
        orientation: new Cesium.ConstantProperty(orientation),
        box: {
          dimensions: new Cesium.Cartesian3(16.0, 2.0, 0.4),
          material: Cesium.Color.fromCssColorString(color).withAlpha(0.85),
          outline: true, outlineColor: Cesium.Color.BLACK,
          distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 800.0),
        },
      });
      entity._wingChild = wingEntity;
    }
  } else {
    entity.position = new Cesium.ConstantPositionProperty(pos);
    entity.orientation = new Cesium.ConstantProperty(orientation);
    if (entity.billboard) entity.billboard.rotation = Cesium.Math.toRadians(-(headingDeg || 0));
    if (entity.label) entity.label.text = labelTxt;
    if (entity._wingChild) {
      entity._wingChild.position = new Cesium.ConstantPositionProperty(pos);
      entity._wingChild.orientation = new Cesium.ConstantProperty(orientation);
    }
  }

  // 碰撞檢測（每次位置更新都檢查）
  _checkCollisions();

  // 狀態列：顯示主控（sysid=1）的資訊
  if (sysid === 1) {
    const el = document.getElementById('sitlStatus');
    el.style.display = 'block';
    el.textContent =
      '✈ SITL[' + sysid + ']  ' + (mode || '') + (armed ? ' ARM' : ' DIS') +
      '  lat:' + lat.toFixed(5) + '  lon:' + lon.toFixed(5) +
      '  alt:' + (alt||0).toFixed(1) + 'm  hdg:' + (headingDeg||0).toFixed(0) + '°  ' +
      'pitch:' + (pitchDeg||0).toFixed(1) + '° roll:' + (rollDeg||0).toFixed(1) + '°  ' +
      'spd:' + (speedMs||0).toFixed(1) + 'm/s';
  }
}

function clearUAV(sysid) {
  function _rm(e) {
    try { if (e._wingChild) viewer.entities.remove(e._wingChild); } catch(_){}
    try { viewer.entities.remove(e); } catch(_){}
  }
  function _rmTrail(sid) {
    if (uavTrails[sid]) { try { viewer.entities.remove(uavTrails[sid]); } catch(_){} delete uavTrails[sid]; }
    if (uavTrailPts[sid]) delete uavTrailPts[sid];
  }
  if (sysid !== undefined && sysid !== null) {
    if (uavEntities[sysid]) { _rm(uavEntities[sysid]); delete uavEntities[sysid]; }
    _rmTrail(sysid);
  } else {
    Object.values(uavEntities).forEach(_rm);
    uavEntities = {};
    uavEntity = null;
    Object.keys(uavTrails).forEach(_rmTrail);
  }
  document.getElementById('sitlStatus').style.display = 'none';
}

// ── 滑鼠點擊（對應 MapWidget 的 pyqt://click 橋接）──────────────────
const clickHandler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
clickHandler.setInputAction(function(e) {
  const ray = viewer.camera.getPickRay(e.position);
  const cartesian = viewer.scene.globe.pick(ray, viewer.scene);
  if (cartesian) {
    const carto = Cesium.Ellipsoid.WGS84.cartesianToCartographic(cartesian);
    const lat = Cesium.Math.toDegrees(carto.latitude);
    const lon = Cesium.Math.toDegrees(carto.longitude);
    window.location.href = 'pyqt://click/' + lat + '/' + lon;
  }
}, Cesium.ScreenSpaceEventType.LEFT_CLICK);

// ── 初始相機（會由 Python 端在載入後呼叫 setInitialView 覆寫）────────
// ── DEM 地形疊加 ──────────────────────────────────────────────
var demOverlayEntity = null;
var demOutlineEntity = null;
var demLabelEntities = [];
function clearDemOverlay() {
  if (demOverlayEntity) { viewer.entities.remove(demOverlayEntity); demOverlayEntity = null; }
  if (demOutlineEntity) { viewer.entities.remove(demOutlineEntity); demOutlineEntity = null; }
  demLabelEntities.forEach(e => { try { viewer.entities.remove(e); } catch(_) {} });
  demLabelEntities = [];
}
function showDemOverlay(latMin, latMax, lonMin, lonMax, pngDataUrl, minE, maxE, alpha) {
  clearDemOverlay();
  var rect = Cesium.Rectangle.fromDegrees(lonMin, latMin, lonMax, latMax);
  // 半透明高度 heatmap 貼在地面
  demOverlayEntity = viewer.entities.add({
    name: 'DEM_Overlay',
    rectangle: {
      coordinates: rect,
      material: new Cesium.ImageMaterialProperty({
        image: pngDataUrl,
        transparent: true,
        color: new Cesium.Color(1, 1, 1, alpha || 0.65),
      }),
      classificationType: Cesium.ClassificationType.TERRAIN,
    },
  });
  // 紅色外框
  var ring = [
    Cesium.Cartesian3.fromDegrees(lonMin, latMin),
    Cesium.Cartesian3.fromDegrees(lonMax, latMin),
    Cesium.Cartesian3.fromDegrees(lonMax, latMax),
    Cesium.Cartesian3.fromDegrees(lonMin, latMax),
    Cesium.Cartesian3.fromDegrees(lonMin, latMin),
  ];
  demOutlineEntity = viewer.entities.add({
    polyline: {
      positions: ring,
      width: 3,
      material: new Cesium.PolylineDashMaterialProperty({
        color: Cesium.Color.fromCssColorString('#FFEB3B'),
        dashLength: 14,
      }),
      clampToGround: true,
    },
  });
  // 四角高程標籤
  var corners = [
    [latMin, lonMin, 'SW'], [latMin, lonMax, 'SE'],
    [latMax, lonMin, 'NW'], [latMax, lonMax, 'NE'],
  ];
  corners.forEach(c => {
    var e = viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(c[1], c[0], 50),
      label: {
        text: c[2],
        font: '11px sans-serif',
        fillColor: Cesium.Color.YELLOW,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
    demLabelEntities.push(e);
  });
  // 中央資訊標籤
  var cLat = (latMin + latMax) / 2;
  var cLon = (lonMin + lonMax) / 2;
  var info = viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(cLon, cLat, 200),
    label: {
      text: 'DEM  ' + minE.toFixed(0) + 'm ~ ' + maxE.toFixed(0) + 'm',
      font: 'bold 13px sans-serif',
      fillColor: Cesium.Color.fromCssColorString('#FFEB3B'),
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 3,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      showBackground: true,
      backgroundColor: new Cesium.Color(0, 0, 0, 0.6),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
  demLabelEntities.push(info);
  // 飛到 DEM 區域
  viewer.camera.flyTo({
    destination: Cesium.Rectangle.fromDegrees(lonMin, latMin, lonMax, latMax),
    duration: 1.2,
  });
  console.log('[DEM] overlay 已載入: ' + minE.toFixed(1) + '~' + maxE.toFixed(1) + 'm');
}

// ── DEM 3D 地形（CustomHeightmapTerrainProvider 平滑版）─────
var demTerrainPrimitive = null;
var demHeightData = null;  // {data: Float32Array, rows, cols, latMin, latMax, lonMin, lonMax}
var demOriginalTerrain = null;
function clearDemTerrain() {
  if (demTerrainPrimitive) {
    viewer.scene.primitives.remove(demTerrainPrimitive);
    demTerrainPrimitive = null;
  }
  if (demOriginalTerrain) {
    viewer.terrainProvider = demOriginalTerrain;
    demOriginalTerrain = null;
  }
  demHeightData = null;
}
// 採樣 DEM 高程（雙線性插值）
function sampleDem(lat, lon) {
  if (!demHeightData) return 0;
  var d = demHeightData;
  if (lat < d.latMin || lat > d.latMax || lon < d.lonMin || lon > d.lonMax) return 0;
  var fy = (d.latMax - lat) / (d.latMax - d.latMin) * (d.rows - 1);
  var fx = (lon - d.lonMin) / (d.lonMax - d.lonMin) * (d.cols - 1);
  var y0 = Math.floor(fy), y1 = Math.min(y0 + 1, d.rows - 1);
  var x0 = Math.floor(fx), x1 = Math.min(x0 + 1, d.cols - 1);
  var dy = fy - y0, dx = fx - x0;
  var h00 = d.data[y0 * d.cols + x0];
  var h10 = d.data[y0 * d.cols + x1];
  var h01 = d.data[y1 * d.cols + x0];
  var h11 = d.data[y1 * d.cols + x1];
  return h00 * (1-dx)*(1-dy) + h10 * dx*(1-dy) + h01 * (1-dx)*dy + h11 * dx*dy;
}
// 建立 CustomHeightmapTerrainProvider 並套用
function applyDemTerrainProvider() {
  if (!demHeightData) return;
  if (!demOriginalTerrain) demOriginalTerrain = viewer.terrainProvider;
  var TILE_W = 32, TILE_H = 32;
  var provider = new Cesium.CustomHeightmapTerrainProvider({
    width: TILE_W,
    height: TILE_H,
    tilingScheme: new Cesium.GeographicTilingScheme(),
    callback: function(x, y, level) {
      // 計算 tile 地理範圍
      var rect = provider.tilingScheme.tileXYToRectangle(x, y, level);
      var west = Cesium.Math.toDegrees(rect.west);
      var east = Cesium.Math.toDegrees(rect.east);
      var south = Cesium.Math.toDegrees(rect.south);
      var north = Cesium.Math.toDegrees(rect.north);
      var arr = new Float32Array(TILE_W * TILE_H);
      for (var j = 0; j < TILE_H; j++) {
        var lat = north - (north - south) * j / (TILE_H - 1);
        for (var i = 0; i < TILE_W; i++) {
          var lon = west + (east - west) * i / (TILE_W - 1);
          arr[j * TILE_W + i] = sampleDem(lat, lon);
        }
      }
      return arr;
    },
  });
  viewer.terrainProvider = provider;
  viewer.scene.globe.enableLighting = true;
  console.log('[DEM3D] CustomHeightmapTerrainProvider 已套用');
}
// 新版：接收整份 DEM 陣列 → 套用 CustomHeightmapTerrainProvider
function showDemTerrainHeightmap(latMin, latMax, lonMin, lonMax, rows, cols, flatHeights) {
  clearDemTerrain();
  demHeightData = {
    data: new Float32Array(flatHeights),
    rows: rows, cols: cols,
    latMin: latMin, latMax: latMax, lonMin: lonMin, lonMax: lonMax,
  };
  applyDemTerrainProvider();
  viewer.camera.flyTo({
    destination: Cesium.Rectangle.fromDegrees(lonMin, latMin, lonMax, latMax),
    duration: 1.2,
  });
}
// 舊版方塊模式（保留以供比對）
function showDemTerrain3D(latMin, latMax, lonMin, lonMax, cellsFlat, minE, maxE) {
  clearDemTerrain();
  var instances = [];
  var rng = Math.max(maxE - minE, 1.0);
  function rampColor(t) {
    if (t < 0.25) { var k = t/0.25; return new Cesium.Color(0, k, 1, 1); }
    if (t < 0.5)  { var k = (t-0.25)/0.25; return new Cesium.Color(0, 1, 1-k, 1); }
    if (t < 0.75) { var k = (t-0.5)/0.25;  return new Cesium.Color(k, 1, 0, 1); }
    var k = (t-0.75)/0.25; return new Cesium.Color(1, 1-k, 0, 1);
  }
  for (var i = 0; i < cellsFlat.length; i += 5) {
    var la0 = cellsFlat[i];
    var lo0 = cellsFlat[i+1];
    var la1 = cellsFlat[i+2];
    var lo1 = cellsFlat[i+3];
    var h   = cellsFlat[i+4];
    var t = Math.max(0, Math.min(1, (h - minE) / rng));
    var color = rampColor(t);
    var rect = Cesium.Rectangle.fromDegrees(lo0, la0, lo1, la1);
    var geom = new Cesium.RectangleGeometry({
      rectangle: rect,
      height: Math.max(h, 0),
      vertexFormat: Cesium.PerInstanceColorAppearance.VERTEX_FORMAT,
    });
    instances.push(new Cesium.GeometryInstance({
      geometry: geom,
      attributes: {
        color: Cesium.ColorGeometryInstanceAttribute.fromColor(color),
      },
    }));
  }
  demTerrainPrimitive = viewer.scene.primitives.add(new Cesium.Primitive({
    geometryInstances: instances,
    appearance: new Cesium.PerInstanceColorAppearance({
      flat: false,
      translucent: false,
    }),
    asynchronous: false,
  }));
  // 飛到區域
  viewer.camera.flyTo({
    destination: Cesium.Rectangle.fromDegrees(lonMin, latMin, lonMax, latMax),
    duration: 1.2,
  });
  console.log('[DEM3D] 已載入 ' + instances.length + ' 格, 高程 ' + minE.toFixed(0) + '~' + maxE.toFixed(0) + 'm');
}

function setInitialView(lat, lon, height) {
  viewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(lon, lat, height || 2000),
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-55), roll: 0 },
    duration: 0,
  });
}
setInitialView(__INIT_LAT__, __INIT_LON__, __INIT_HEIGHT__);

// ══════════════════════════════════════════════════════════════════════
//  模組一：FSDM 高程切片分析 (Elevation Slicing)
//  基於「特徵選擇決策模型」，將地形依高度區間分色渲染，
//  識別低空突防盲區走廊。
// ══════════════════════════════════════════════════════════════════════
let _elevSlicerActive = false;

/**
 * 更新高程切片視覺化
 * @param {number} minAlt  最低安全高度 (m)
 * @param {number} maxAlt  最高突防高度 (m)
 *
 * 原理：利用 Cesium Globe 的 material 修改，
 * 將地表依高程分為三個色帶：
 *   低於 minAlt → 暗灰色（遮蔽地形）
 *   minAlt ~ maxAlt → 半透明安全綠色（盲區走廊）
 *   高於 maxAlt → 暗黑色（障礙地形）
 *
 * Cesium 1.115 使用 Globe.material 搭配自訂 fabric 實現
 * ElevationBand 效果。
 */
function updateElevationSlicer(minAlt, maxAlt) {
  const globe = viewer.scene.globe;

  if (minAlt === undefined || maxAlt === undefined || minAlt >= maxAlt) {
    // 關閉切片：還原為預設材質
    globe.material = undefined;
    _elevSlicerActive = false;
    console.log('[FSDM] 高程切片已關閉');
    return;
  }

  // 使用 Cesium Fabric 定義自訂 Globe Material
  // 依片元高度（positionMC.z 近似橢球高度）分色
  globe.material = new Cesium.Material({
    fabric: {
      type: 'ElevationSlicer',
      uniforms: {
        minAlt: minAlt,
        maxAlt: maxAlt,
        // 安全走廊色：半透明綠
        safeColor: new Cesium.Color(0.0, 0.9, 0.3, 0.45),
        // 遮蔽色：暗灰黑
        blockedColor: new Cesium.Color(0.12, 0.12, 0.15, 0.85),
      },
      source:
        'uniform float minAlt;\n' +
        'uniform float maxAlt;\n' +
        'uniform vec4 safeColor;\n' +
        'uniform vec4 blockedColor;\n' +
        'czm_material czm_getMaterial(czm_materialInput materialInput) {\n' +
        '  czm_material material = czm_getDefaultMaterial(materialInput);\n' +
        '  // 取得片元的橢球高度（公尺）\n' +
        '  float height = materialInput.height;\n' +
        '  if (height >= minAlt && height <= maxAlt) {\n' +
        '    // 盲區走廊：安全綠色半透明覆蓋\n' +
        '    material.diffuse = safeColor.rgb;\n' +
        '    material.alpha = safeColor.a;\n' +
        '  } else {\n' +
        '    // 超出走廊範圍：暗色遮蔽\n' +
        '    material.diffuse = blockedColor.rgb;\n' +
        '    material.alpha = blockedColor.a;\n' +
        '  }\n' +
        '  return material;\n' +
        '}\n',
    },
  });

  // 備援方案：若自訂 fabric 不支援 materialInput.height，
  // 改用 ElevationBand 色帶（Cesium 1.115 支援）
  try {
    const layers = [
      // 低於最低安全高度 → 暗灰黑
      { entries: [
        { height: -500.0, color: new Cesium.Color(0.12, 0.12, 0.15, 0.85) },
        { height: minAlt - 0.1, color: new Cesium.Color(0.12, 0.12, 0.15, 0.85) },
      ]},
      // 安全走廊 → 綠色半透明
      { entries: [
        { height: minAlt, color: new Cesium.Color(0.0, 0.9, 0.3, 0.45) },
        { height: maxAlt, color: new Cesium.Color(0.0, 0.9, 0.3, 0.45) },
      ]},
      // 高於最高突防高度 → 暗黑
      { entries: [
        { height: maxAlt + 0.1, color: new Cesium.Color(0.08, 0.08, 0.1, 0.9) },
        { height: 9000.0, color: new Cesium.Color(0.08, 0.08, 0.1, 0.9) },
      ]},
    ];
    globe.material = Cesium.createElevationBandMaterial({
      scene: viewer.scene,
      layers: layers,
    });
  } catch(e) {
    console.warn('[FSDM] ElevationBand fallback 失敗，使用基本材質', e);
  }

  _elevSlicerActive = true;
  console.log('[FSDM] 高程切片啟用: ' + minAlt + 'm ~ ' + maxAlt + 'm');
}

/**
 * 清除高程切片，還原正常地圖
 */
function clearElevationSlicer() {
  viewer.scene.globe.material = undefined;
  _elevSlicerActive = false;
  console.log('[FSDM] 高程切片已清除');
}


// ══════════════════════════════════════════════════════════════════════
//  模組二：FOV 光錐 + 動態搜救機率熱力圖 (SAR Probability Heatmap)
//  目標發現機率遵循指數探測定律：P_detect = 1 - exp(-W * q)
//  累積發現機率 (COS) 隨掃過次數遞增
// ══════════════════════════════════════════════════════════════════════
let fovConeEntity = null;       // FOV 光錐 Entity
let fovOutlineEntity = null;    // FOV 地面投影圓環 Entity
let heatmapEntities = [];       // 熱力圖網格 Entity 陣列
let heatmapGrid = null;         // 網格狀態 {rows, cols, cells: [{lat, lon, cos, entity}]}
let _heatmapSweepWidth = 50.0;  // 掃描有效寬度 W (公尺)
let _heatmapQuality = 0.8;      // 探測品質 q (0~1)

// FOV 光錐的即時狀態（供 CallbackProperty 讀取，避免每幀重建 Entity）
let _fovState = {
  lat: 0, lon: 0, alt: 100,
  radius: 50, heading: 0,
  pitch: 0, roll: 0,
  active: false,
  lastUpdate: 0,
};

/**
 * 更新 FOV 光錐狀態
 *
 * 設計改良：
 * 1. 使用 CallbackProperty 讓 Cesium 自行在渲染迴圈中讀取最新狀態，
 *    避免每幀重建 ConstantPositionProperty 造成的跳動。
 * 2. 圓錐保持垂直（不套用 pitch/roll），僅用 heading 旋轉。
 *    pitch/roll 改為微幅偏移地面投影圓心，模擬感測器斜視效果。
 * 3. 內建 100ms 節流，避免高頻遙測導致 GPU 過載。
 *
 * @param {number} lat  UAV 緯度
 * @param {number} lon  UAV 經度
 * @param {number} alt  UAV 高度 (m)
 * @param {number} fovRadius  地面投影半徑 (m)
 * @param {number} headingDeg  航向角 (度)
 * @param {number} pitchDeg    俯仰角 (度)
 * @param {number} rollDeg     滾轉角 (度)
 */
function updateFOVCone(lat, lon, alt, fovRadius, headingDeg, pitchDeg, rollDeg) {
  if (!lat || !lon || !alt || alt <= 0) {
    if (fovConeEntity) {
      viewer.entities.remove(fovConeEntity);
      fovConeEntity = null;
    }
    if (fovOutlineEntity) {
      viewer.entities.remove(fovOutlineEntity);
      fovOutlineEntity = null;
    }
    _fovState.active = false;
    return;
  }

  // 節流：100ms 內不重複更新，避免高頻遙測造成 GPU 閃爍
  const now = Date.now();
  if (now - _fovState.lastUpdate < 100 && fovConeEntity) {
    // 只更新狀態數值，讓 CallbackProperty 下一幀自動讀取
    _fovState.lat = lat;
    _fovState.lon = lon;
    _fovState.alt = alt;
    _fovState.radius = fovRadius || (alt * 0.6);
    _fovState.heading = headingDeg || 0;
    _fovState.pitch = pitchDeg || 0;
    _fovState.roll = rollDeg || 0;
    return;
  }
  _fovState.lastUpdate = now;
  _fovState.lat = lat;
  _fovState.lon = lon;
  _fovState.alt = alt;
  _fovState.radius = fovRadius || (alt * 0.6);
  _fovState.heading = headingDeg || 0;
  _fovState.pitch = pitchDeg || 0;
  _fovState.roll = rollDeg || 0;
  _fovState.active = true;

  if (!fovConeEntity) {
    // 首次建立：使用 CallbackProperty 綁定位置和尺寸
    fovConeEntity = viewer.entities.add({
      name: 'FOV-Cone',
      // CallbackProperty：每渲染幀自動讀取 _fovState，無需手動刷新
      position: new Cesium.CallbackProperty(function() {
        // 圓錐中心在 UAV 與地面之間的中點
        return Cesium.Cartesian3.fromDegrees(
          _fovState.lon, _fovState.lat, _fovState.alt / 2.0
        );
      }, false),
      // 只用 heading 旋轉（不套用 pitch/roll 避免劇烈晃動）
      orientation: new Cesium.CallbackProperty(function() {
        var pos = Cesium.Cartesian3.fromDegrees(
          _fovState.lon, _fovState.lat, _fovState.alt / 2.0
        );
        var hpr = new Cesium.HeadingPitchRoll(
          Cesium.Math.toRadians(_fovState.heading), 0, 0
        );
        return Cesium.Transforms.headingPitchRollQuaternion(pos, hpr);
      }, false),
      cylinder: {
        // CallbackProperty 控制長度與半徑，隨高度自動調整
        length: new Cesium.CallbackProperty(function() {
          return Math.max(_fovState.alt, 10);
        }, false),
        topRadius: 0.5,  // 頂端微小（近似尖頂，但不為 0 避免渲染瑕疵）
        bottomRadius: new Cesium.CallbackProperty(function() {
          return _fovState.radius;
        }, false),
        material: Cesium.Color.YELLOW.withAlpha(0.15),
        outline: true,
        outlineColor: Cesium.Color.YELLOW.withAlpha(0.45),
        numberOfVerticalLines: 8,
        slices: 24,
      },
    });

    // 地面投影圓環：顯示光錐在地面的覆蓋範圍
    fovOutlineEntity = viewer.entities.add({
      name: 'FOV-Ground-Ring',
      position: new Cesium.CallbackProperty(function() {
        // pitch/roll 偏移地面投影中心，模擬感測器斜視
        // 偏移量 = alt × tan(pitch) 沿航向方向
        var pitchRad = Cesium.Math.toRadians(_fovState.pitch || 0);
        var hdgRad = Cesium.Math.toRadians(_fovState.heading || 0);
        var offsetM = _fovState.alt * Math.tan(Math.min(Math.abs(pitchRad), 0.6));
        var dLat = offsetM * Math.cos(hdgRad) / 111320.0;
        var dLon = offsetM * Math.sin(hdgRad) / (111320.0 * Math.cos(Cesium.Math.toRadians(_fovState.lat)));
        return Cesium.Cartesian3.fromDegrees(
          _fovState.lon + dLon, _fovState.lat + dLat, 1.0
        );
      }, false),
      ellipse: {
        semiMajorAxis: new Cesium.CallbackProperty(function() {
          return _fovState.radius;
        }, false),
        semiMinorAxis: new Cesium.CallbackProperty(function() {
          return _fovState.radius;
        }, false),
        material: Cesium.Color.YELLOW.withAlpha(0.08),
        outline: true,
        outlineColor: Cesium.Color.YELLOW.withAlpha(0.6),
        outlineWidth: 2,
        height: 0.5,
      },
    });

    console.log('[FOV] 光錐 Entity 建立（CallbackProperty 模式）');
  }
  // 後續更新不需手動設定 Entity 屬性，CallbackProperty 會自動讀取 _fovState
}

/**
 * 清除 FOV 光錐
 */
function clearFOVCone() {
  if (fovConeEntity) {
    viewer.entities.remove(fovConeEntity);
    fovConeEntity = null;
  }
  if (fovOutlineEntity) {
    viewer.entities.remove(fovOutlineEntity);
    fovOutlineEntity = null;
  }
  _fovState.active = false;
}

/**
 * 初始化搜救機率熱力圖網格
 * 在指定矩形區域建立 rows x cols 的網格覆蓋
 *
 * @param {number} latMin  南邊界緯度
 * @param {number} latMax  北邊界緯度
 * @param {number} lonMin  西邊界經度
 * @param {number} lonMax  東邊界經度
 * @param {number} rows    行數
 * @param {number} cols    列數
 * @param {number} sweepWidth  掃描有效寬度 W (公尺)，用於探測機率計算
 * @param {number} quality     探測品質 q (0~1)
 */
function initSARHeatmap(latMin, latMax, lonMin, lonMax, rows, cols, sweepWidth, quality) {
  clearSARHeatmap();

  _heatmapSweepWidth = sweepWidth || 50.0;
  _heatmapQuality = quality || 0.8;
  rows = rows || 20;
  cols = cols || 20;

  const dLat = (latMax - latMin) / rows;
  const dLon = (lonMax - lonMin) / cols;

  heatmapGrid = {
    rows: rows,
    cols: cols,
    latMin: latMin, latMax: latMax,
    lonMin: lonMin, lonMax: lonMax,
    dLat: dLat, dLon: dLon,
    cells: [],
  };

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const cellLatMin = latMin + r * dLat;
      const cellLonMin = lonMin + c * dLon;
      const cellLatMax = cellLatMin + dLat;
      const cellLonMax = cellLonMin + dLon;

      // 初始累積發現機率 COS = 0（紅色 = 高殘餘機率 = 未搜索）
      const entity = viewer.entities.add({
        name: 'SAR-Cell-' + r + '-' + c,
        rectangle: {
          coordinates: Cesium.Rectangle.fromDegrees(cellLonMin, cellLatMin, cellLonMax, cellLatMax),
          // 初始：紅色 = 目標可能在此（未搜索）
          material: Cesium.Color.RED.withAlpha(0.5),
          height: 0.5,
          classificationType: Cesium.ClassificationType.TERRAIN,
        },
      });
      heatmapEntities.push(entity);
      heatmapGrid.cells.push({
        r: r, c: c,
        latCenter: (cellLatMin + cellLatMax) / 2.0,
        lonCenter: (cellLonMin + cellLonMax) / 2.0,
        cos: 0.0,        // 累積發現機率 (Cumulative probability Of Success)
        sweepCount: 0,    // 被光錐掃過的次數
        entity: entity,
      });
    }
  }

  console.log('[SAR] 熱力圖初始化: ' + rows + 'x' + cols +
              ' 網格, W=' + _heatmapSweepWidth + ', q=' + _heatmapQuality);
}

/**
 * 更新搜救熱力圖：當 UAV 光錐掃過某網格時，該網格 COS 遞增
 *
 * 指數探測定律：
 *   單次探測機率 P_single = 1 - exp(-W * q)
 *   累積發現機率 COS(n) = 1 - (1 - P_single)^n
 *     其中 n = 掃過次數
 *
 * 顏色映射：
 *   COS ≈ 0 → 紅色（高殘餘機率，未搜索）
 *   COS ≈ 0.5 → 黃色（部分搜索）
 *   COS ≈ 1.0 → 藍色/透明（已充分搜索，目標不太可能在此）
 *
 * @param {number} uavLat    UAV 緯度
 * @param {number} uavLon    UAV 經度
 * @param {number} fovRadius FOV 地面投影半徑 (m)
 */
function updateHeatmap(uavLat, uavLon, fovRadius) {
  if (!heatmapGrid || !heatmapGrid.cells.length) return;
  if (!uavLat || !uavLon) return;

  const R = fovRadius || 50.0;
  // 近似：1 度緯度 ≈ 111320 m, 1 度經度 ≈ 111320 * cos(lat) m
  const mPerDegLat = 111320.0;
  const mPerDegLon = 111320.0 * Math.cos(Cesium.Math.toRadians(uavLat));

  // 單次探測機率：P_single = 1 - exp(-W * q)
  // 這裡 W 正規化為 (掃描寬度/格子寬度)，q 為探測品質
  const cellWidthM = heatmapGrid.dLon * mPerDegLon;
  const normalizedW = _heatmapSweepWidth / Math.max(cellWidthM, 1.0);
  const pSingle = 1.0 - Math.exp(-normalizedW * _heatmapQuality);

  for (let i = 0; i < heatmapGrid.cells.length; i++) {
    const cell = heatmapGrid.cells[i];

    // 計算 UAV 到網格中心的距離（公尺）
    const dLatM = (cell.latCenter - uavLat) * mPerDegLat;
    const dLonM = (cell.lonCenter - uavLon) * mPerDegLon;
    const dist = Math.sqrt(dLatM * dLatM + dLonM * dLonM);

    if (dist <= R) {
      // 光錐覆蓋此網格 → 更新 COS
      cell.sweepCount += 1;

      // COS(n) = 1 - (1 - P_single)^n
      // 等價遞推：COS_new = 1 - (1 - COS_old) * (1 - P_single)
      cell.cos = 1.0 - (1.0 - cell.cos) * (1.0 - pSingle);

      // 顏色映射：紅(0) → 黃(0.5) → 青(0.8) → 藍(1.0)
      const t = Math.min(cell.cos, 1.0);
      let r, g, b, a;
      if (t < 0.5) {
        // 紅 → 黃
        const k = t / 0.5;
        r = 1.0; g = k; b = 0.0;
      } else if (t < 0.8) {
        // 黃 → 青
        const k = (t - 0.5) / 0.3;
        r = 1.0 - k; g = 1.0; b = k;
      } else {
        // 青 → 藍
        const k = (t - 0.8) / 0.2;
        r = 0.0; g = 1.0 - k * 0.7; b = 1.0;
      }
      // 已搜索越多 → 越透明
      a = Math.max(0.08, 0.55 - t * 0.45);

      cell.entity.rectangle.material = new Cesium.Color(r, g, b, a);
    }
  }
}

/**
 * 清除搜救熱力圖
 */
function clearSARHeatmap() {
  heatmapEntities.forEach(function(e) {
    try { viewer.entities.remove(e); } catch(_) {}
  });
  heatmapEntities = [];
  heatmapGrid = null;
}

/**
 * 重置搜救熱力圖（保留網格，重設 COS 為 0）
 */
function resetSARHeatmap() {
  if (!heatmapGrid) return;
  for (let i = 0; i < heatmapGrid.cells.length; i++) {
    var cell = heatmapGrid.cells[i];
    cell.cos = 0.0;
    cell.sweepCount = 0;
    cell.entity.rectangle.material = Cesium.Color.RED.withAlpha(0.5);
  }
  console.log('[SAR] 熱力圖已重置');
}


// ══════════════════════════════════════════════════════════════════════
//  模組三：3D 雷達威脅穹頂 + RCS 敏感度動態渲染
//  雷達威脅成本呈指數衰減：cost = exp(-d / δ)
//  UAV 不同角度的 RCS 敏感度不同，正面 RCS 最大
// ══════════════════════════════════════════════════════════════════════
let radarDomeEntities = [];     // 雷達穹頂 Entity 陣列
let radarSources = [];          // 雷達源資訊：[{lat, lon, alt, radius, entity, ringEntity}]

/**
 * 新增一個雷達威脅穹頂（半球體 + 發光邊緣）
 *
 * @param {number} lat     雷達緯度
 * @param {number} lon     雷達經度
 * @param {number} alt     雷達高度 (m)，通常為 0
 * @param {number} radius  探測半徑 (m)
 * @param {string} name    雷達名稱（可選）
 * @returns {number} 雷達索引 ID
 */
function addRadarDome(lat, lon, alt, radius, name) {
  alt = alt || 0;
  radius = radius || 5000;
  const pos = Cesium.Cartesian3.fromDegrees(lon, lat, alt);

  // 半球體 Entity：使用 ellipsoid 搭配 minimumClock/maximumClock
  // 限制只顯示上半球（地面以上）
  const domeEntity = viewer.entities.add({
    name: name || ('Radar-' + (radarSources.length + 1)),
    position: pos,
    ellipsoid: {
      radii: new Cesium.Cartesian3(radius, radius, radius),
      // 只顯示上半球：俯仰角 0° ~ 90°
      minimumCone: 0.0,                            // 頂部（正上方）
      maximumCone: Cesium.Math.toRadians(90),      // 赤道（地平面）
      // 半透明紅色材質
      material: Cesium.Color.RED.withAlpha(0.08),
      outline: true,
      outlineColor: Cesium.Color.RED.withAlpha(0.5),
      outlineWidth: 2.0,
      slicePartitions: 36,
      stackPartitions: 18,
    },
  });
  radarDomeEntities.push(domeEntity);

  // 地面圓環：標示雷達探測範圍邊界（發光環）
  const ringPositions = [];
  for (let deg = 0; deg <= 360; deg += 5) {
    const rad = Cesium.Math.toRadians(deg);
    const rLat = lat + (radius / 111320.0) * Math.cos(rad);
    const rLon = lon + (radius / (111320.0 * Math.cos(Cesium.Math.toRadians(lat)))) * Math.sin(rad);
    ringPositions.push(Cesium.Cartesian3.fromDegrees(rLon, rLat, alt + 5));
  }
  const ringEntity = viewer.entities.add({
    name: (name || 'Radar') + '-Ring',
    polyline: {
      positions: ringPositions,
      width: 4,
      material: new Cesium.PolylineGlowMaterialProperty({
        glowPower: 0.3,
        color: Cesium.Color.RED.withAlpha(0.8),
      }),
    },
  });
  radarDomeEntities.push(ringEntity);

  // 雷達中心標記
  const centerEntity = viewer.entities.add({
    name: (name || 'Radar') + '-Center',
    position: Cesium.Cartesian3.fromDegrees(lon, lat, alt + 20),
    billboard: {
      image: 'data:image/svg+xml,' + encodeURIComponent(
        '<svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">' +
        '<circle cx="16" cy="16" r="14" fill="rgba(244,67,54,0.7)" stroke="#fff" stroke-width="2"/>' +
        '<text x="16" y="21" text-anchor="middle" fill="#fff" font-size="14" font-weight="bold">R</text>' +
        '</svg>'
      ),
      scale: 1.2,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    label: {
      text: (name || 'Radar') + '\nR=' + (radius/1000).toFixed(1) + 'km',
      font: 'bold 11px sans-serif',
      fillColor: Cesium.Color.fromCssColorString('#FF5252'),
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -28),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
  radarDomeEntities.push(centerEntity);

  const idx = radarSources.length;
  radarSources.push({
    lat: lat, lon: lon, alt: alt,
    radius: radius,
    entity: domeEntity,
    ringEntity: ringEntity,
    centerEntity: centerEntity,
  });

  console.log('[Radar] 新增威脅穹頂 #' + idx + ': ' + (name||'Radar') +
              ' (' + lat.toFixed(4) + ',' + lon.toFixed(4) + ') R=' + radius + 'm');
  return idx;
}

/**
 * 清除所有雷達穹頂
 */
function clearRadarDomes() {
  radarDomeEntities.forEach(function(e) {
    try { viewer.entities.remove(e); } catch(_) {}
  });
  radarDomeEntities = [];
  radarSources = [];
  console.log('[Radar] 所有威脅穹頂已清除');
}

/**
 * 更新 UAV 的 RCS 敏感度渲染
 *
 * 威脅成本公式：threat = exp(-d / δ)
 *   d = UAV 與雷達之距離 (m)
 *   δ = 雷達有效衰減距離（= radius * 0.3，約 30% 半徑處威脅降至 1/e）
 *
 * RCS 角度敏感度：
 *   正面朝向雷達 (θ ≈ 0°) → RCS 最大 → 威脅加成 × 1.5
 *   側面 (θ ≈ 90°) → RCS 中等 → 威脅加成 × 1.0
 *   尾部 (θ ≈ 180°) → RCS 最小 → 威脅加成 × 0.5
 *   角度權重 w(θ) = 1.5 - cos(θ)（θ=0 時 w=0.5, θ=180 時 w=2.5 再 clamp）
 *   簡化：w(θ) = 0.5 + (1 - cos(θ)) * 0.5 → 正面=0.5(低), 尾部=1.5(高)
 *   但軍事上正面 RCS 大 → w 反轉：w = 1.5 - (1 - cos(θ)) * 0.5 = 1 + 0.5*cos(θ)
 *
 * 最終威脅 T = Σ_radars [ exp(-d_i / δ_i) × w(θ_i) ]
 * T → 0 安全（藍/綠冷色）, T → 1+ 危險（紅色熱色）
 *
 * @param {number} uavLat     UAV 緯度
 * @param {number} uavLon     UAV 經度
 * @param {number} uavAlt     UAV 高度
 * @param {number} uavHeading UAV 航向角 (度)
 * @param {number} sysid      UAV sysid（用於定位對應 Entity）
 */
function updateRCSSensitivity(uavLat, uavLon, uavAlt, uavHeading, sysid) {
  sysid = sysid || 1;
  const entity = uavEntities[sysid];
  if (!entity || !radarSources.length) return;

  const mPerDegLat = 111320.0;
  const mPerDegLon = 111320.0 * Math.cos(Cesium.Math.toRadians(uavLat));

  let totalThreat = 0.0;

  for (let i = 0; i < radarSources.length; i++) {
    const radar = radarSources[i];

    // 計算 UAV 到雷達的 3D 距離
    const dLatM = (uavLat - radar.lat) * mPerDegLat;
    const dLonM = (uavLon - radar.lon) * mPerDegLon;
    const dAltM = (uavAlt || 0) - (radar.alt || 0);
    const dist = Math.sqrt(dLatM * dLatM + dLonM * dLonM + dAltM * dAltM);

    // δ = 有效衰減距離（雷達半徑的 30%）
    const delta = radar.radius * 0.3;

    // 指數衰減威脅：threat_i = exp(-d / δ)
    const threatDist = Math.exp(-dist / Math.max(delta, 1.0));

    // 計算 UAV 相對於雷達的方位角（從雷達看 UAV 的方向）
    const bearingToUav = Math.atan2(dLonM, dLatM);  // 弧度，北=0
    // UAV 航向（弧度）
    const uavHdgRad = Cesium.Math.toRadians(uavHeading || 0);
    // θ = UAV 航向與「雷達→UAV 方向」的夾角
    // θ ≈ 0° 表示 UAV 正面朝向雷達（RCS 最大）
    // θ ≈ 180° 表示 UAV 尾部朝向雷達（RCS 最小）
    const theta = Math.abs(bearingToUav - uavHdgRad);
    const cosTheta = Math.cos(theta);

    // 角度權重：正面(θ≈0, cos≈1) → w 大；尾部(θ≈π, cos≈-1) → w 小
    // w(θ) = 1.0 + 0.5 * cosθ → 範圍 [0.5, 1.5]
    const angleWeight = 1.0 + 0.5 * cosTheta;

    totalThreat += threatDist * angleWeight;
  }

  // 將總威脅 clamp 到 [0, 1]
  totalThreat = Math.min(totalThreat, 1.0);

  // 顏色映射：安全(0)→藍/綠 → 危險(1)→紅
  let r, g, b;
  if (totalThreat < 0.3) {
    // 安全：藍色 → 青色
    const k = totalThreat / 0.3;
    r = 0.0; g = k; b = 1.0;
  } else if (totalThreat < 0.6) {
    // 中等：青 → 黃
    const k = (totalThreat - 0.3) / 0.3;
    r = k; g = 1.0; b = 1.0 - k;
  } else {
    // 危險：黃 → 紅
    const k = (totalThreat - 0.6) / 0.4;
    r = 1.0; g = 1.0 - k; b = 0.0;
  }

  const rcsColor = new Cesium.Color(r, g, b, 0.95);

  // 套用到 UAV Entity 的各個幾何屬性
  // 1. billboard tint（SVG 圖標染色）
  if (entity.billboard) {
    entity.billboard.color = rcsColor;
  }
  // 2. model color blend（GLB 3D 模型染色）
  if (entity.model) {
    entity.model.color = rcsColor;
    entity.model.colorBlendMode = Cesium.ColorBlendMode.MIX;
    entity.model.colorBlendAmount = 0.6;  // 60% 混合 RCS 顏色
  }
  // 3. ellipsoid / box fallback
  if (entity.ellipsoid) {
    entity.ellipsoid.material = rcsColor;
  }
  if (entity.box) {
    entity.box.material = rcsColor;
  }
  // 4. 機翼子 Entity
  if (entity._wingChild && entity._wingChild.box) {
    entity._wingChild.box.material = rcsColor;
  }
}

/**
 * 清除 RCS 渲染，還原 UAV 原始顏色
 * @param {number} sysid  UAV sysid
 */
function clearRCSSensitivity(sysid) {
  sysid = sysid || 1;
  const entity = uavEntities[sysid];
  if (!entity) return;

  // 還原預設顏色
  const color = _UAV_COLORS[(sysid - 1) % _UAV_COLORS.length];
  if (entity.billboard) {
    entity.billboard.color = Cesium.Color.WHITE;  // 無染色
  }
  if (entity.model) {
    entity.model.color = Cesium.Color.WHITE;
    entity.model.colorBlendAmount = 0.0;
  }
  if (entity.ellipsoid) {
    entity.ellipsoid.material = Cesium.Color.fromCssColorString(color).withAlpha(0.95);
  }
  if (entity.box) {
    entity.box.material = Cesium.Color.fromCssColorString(color).withAlpha(0.95);
  }
  if (entity._wingChild && entity._wingChild.box) {
    entity._wingChild.box.material = Cesium.Color.fromCssColorString(color).withAlpha(0.85);
  }
}

/**
 * 模擬雷達掃描動畫：穹頂半徑脈衝動態效果
 * @param {number} radarIdx  雷達索引
 * @param {number} duration  動畫持續時間 (ms)
 */
function animateRadarScan(radarIdx, duration) {
  if (radarIdx < 0 || radarIdx >= radarSources.length) return;
  const radar = radarSources[radarIdx];
  const baseRadius = radar.radius;
  const startTime = Date.now();
  duration = duration || 2000;

  function _pulse() {
    const elapsed = Date.now() - startTime;
    if (elapsed > duration) {
      // 恢復原始大小
      radar.entity.ellipsoid.radii = new Cesium.Cartesian3(baseRadius, baseRadius, baseRadius);
      return;
    }
    // 脈衝效果：半徑在 80%~120% 之間振盪
    const t = elapsed / duration;
    const scale = 1.0 + 0.2 * Math.sin(t * Math.PI * 6);
    const r = baseRadius * scale;
    radar.entity.ellipsoid.radii = new Cesium.Cartesian3(r, r, r);
    requestAnimationFrame(_pulse);
  }
  _pulse();
}

// ═══════════════════════════════════════════════════════════════════════
//  戰術模組四：UCAV 蜂群分佈式協同打擊與末端俯衝視覺化
//  (Swarm Distributed Strike & Terminal Dive Visualization)
// ═══════════════════════════════════════════════════════════════════════

// ── 打擊模組狀態變數 ─────────────────────────────────────────────────
let strikeTargetEntities = [];     // 打擊目標標記 entities
let strikeTrailEntities  = [];     // 軌跡折線 entities
let strikeUavEntities    = [];     // UAV 動畫 entities (模型/billboard)
let strikeLaserEntities  = [];     // 目標鎖定射線 entities
let strikeBlastEntities  = [];     // 爆炸特效 entities
let _strikeAnimRunning   = false;  // 動畫是否進行中
let _strikeAnimHandles   = [];     // requestAnimationFrame handles
let _strikeMarkingMode   = false;  // 是否正在標記目標
let _strikeMarkedTargets = [];     // 已標記的目標 [{lat, lon}]

/**
 * 切換打擊目標標記模式
 * 啟用後，地圖左鍵點擊會新增敵方目標而非邊界角點
 */
function strikeSetMarkingMode(enabled) {
  _strikeMarkingMode = !!enabled;
  // 更新模式標籤提示
  const modeEl = document.getElementById('modeLabel');
  if (modeEl) {
    if (_strikeMarkingMode) {
      modeEl.textContent = '🎯 打擊目標標記模式';
      modeEl.style.color = '#FF5252';
      modeEl.style.display = 'block';
    } else {
      modeEl.style.display = 'none';
    }
  }
}

/**
 * 新增一個打擊目標標記到 3D 地圖
 * @param {number} lat  緯度
 * @param {number} lon  經度
 * @param {number} idx  目標編號
 */
function strikeAddTarget(lat, lon, idx) {
  const pos = Cesium.Cartesian3.fromDegrees(lon, lat, 5);
  // 菱形紅色標記 + 脈衝動畫
  const entity = viewer.entities.add({
    position: pos,
    name: 'TGT-' + idx,
    // 地面紅十字標記
    billboard: {
      image: _strikeTargetSvg(idx),
      scale: 1.0,
      verticalOrigin: Cesium.VerticalOrigin.CENTER,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    // 目標區域環
    ellipse: {
      semiMajorAxis: 30,
      semiMinorAxis: 30,
      material: Cesium.Color.RED.withAlpha(0.15),
      outline: true,
      outlineColor: Cesium.Color.RED.withAlpha(0.6),
      outlineWidth: 2,
      height: 0,
    },
    label: {
      text: 'TGT-' + idx,
      font: 'bold 12px monospace',
      fillColor: Cesium.Color.RED,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -32),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
  strikeTargetEntities.push(entity);
}

/**
 * 生成打擊目標 SVG 圖示（紅色瞄準十字）
 */
function _strikeTargetSvg(idx) {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">
    <circle cx="24" cy="24" r="20" fill="none" stroke="#FF1744" stroke-width="2.5" stroke-dasharray="6,3"/>
    <circle cx="24" cy="24" r="10" fill="none" stroke="#FF1744" stroke-width="1.5"/>
    <line x1="24" y1="2" x2="24" y2="14" stroke="#FF1744" stroke-width="2"/>
    <line x1="24" y1="34" x2="24" y2="46" stroke="#FF1744" stroke-width="2"/>
    <line x1="2" y1="24" x2="14" y2="24" stroke="#FF1744" stroke-width="2"/>
    <line x1="34" y1="24" x2="46" y2="24" stroke="#FF1744" stroke-width="2"/>
    <circle cx="24" cy="24" r="3" fill="#FF1744"/>
  </svg>`;
  return 'data:image/svg+xml,' + encodeURIComponent(svg);
}

/**
 * 清除所有打擊相關視覺化
 */
function strikeClearAll() {
  // 停止動畫
  _strikeAnimRunning = false;
  _strikeAnimHandles.forEach(h => cancelAnimationFrame(h));
  _strikeAnimHandles = [];

  // 移除所有打擊 entities
  [strikeTargetEntities, strikeTrailEntities, strikeUavEntities,
   strikeLaserEntities, strikeBlastEntities].forEach(arr => {
    arr.forEach(e => { try { viewer.entities.remove(e); } catch(_){} });
    arr.length = 0;
  });

  _strikeMarkedTargets = [];
  _strikeMarkingMode = false;

  // 重置時間軸
  viewer.clock.shouldAnimate = false;

  const modeEl = document.getElementById('modeLabel');
  if (modeEl) modeEl.style.display = 'none';
}

/**
 * 執行蜂群打擊動畫 — 核心入口
 *
 * 接收後台 Python TerminalStrikePlanner 計算完成的 JSON 資料，
 * 在 Cesium 3D 場景中播放完整的俯衝攻擊動畫序列。
 *
 * @param {string} dataJson - JSON 字串，包含：
 *   - targets: 目標陣列
 *   - trajectories: 各 UCAV 軌跡（含 cruise/dive 分段標記）
 *   - dive_initiation_dist: 俯衝起始距離
 *   - max_dive_angle: 最大俯衝角
 * @param {number} animSpeedMultiplier - 動畫速度倍率（預設 3.0）
 */
function strikeExecuteAnimation(dataJson, animSpeedMultiplier) {
  const data = JSON.parse(dataJson);
  const targets = data.targets;
  const trajectories = data.trajectories;
  const speedMul = animSpeedMultiplier || 3.0;

  // 清除先前的動畫（保留目標標記）
  _strikeAnimRunning = false;
  _strikeAnimHandles.forEach(h => cancelAnimationFrame(h));
  _strikeAnimHandles = [];
  [strikeTrailEntities, strikeUavEntities, strikeLaserEntities, strikeBlastEntities].forEach(arr => {
    arr.forEach(e => { try { viewer.entities.remove(e); } catch(_){} });
    arr.length = 0;
  });

  if (!trajectories || trajectories.length === 0) return;

  // ── 計算動畫時間範圍 ──────────────────────────────────────────────
  // 找出所有軌跡中最大的任務時間
  let maxTimeSec = 0;
  trajectories.forEach(tr => {
    if (tr.total_time_sec > maxTimeSec) maxTimeSec = tr.total_time_sec;
  });

  // 動畫實際時長（秒）= 任務時間 / 速度倍率
  const animDurationSec = maxTimeSec / speedMul;

  // 設定 Cesium 時鐘為動畫模式
  const startTime = Cesium.JulianDate.now();
  const endTime = Cesium.JulianDate.addSeconds(startTime, animDurationSec, new Cesium.JulianDate());
  viewer.clock.startTime = startTime.clone();
  viewer.clock.stopTime = endTime.clone();
  viewer.clock.currentTime = startTime.clone();
  viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;
  viewer.clock.multiplier = 1.0;  // 實際速率由 speedMul 控制
  viewer.clock.shouldAnimate = true;
  viewer.timeline.zoomTo(startTime, endTime);

  // ── UCAV 顏色盤 ──────────────────────────────────────────────────
  const uavColors = ['#E53935','#1E88E5','#43A047','#FB8C00',
                     '#8E24AA','#00ACC1','#F4511E','#3949AB',
                     '#C62828','#0D47A1','#2E7D32','#E65100'];

  // ── 為每條軌跡建立 SampledPositionProperty + 模型 Entity ─────────
  trajectories.forEach((tr, trIdx) => {
    const colorHex = uavColors[trIdx % uavColors.length];
    const cesiumColor = Cesium.Color.fromCssColorString(colorHex);
    const wps = tr.waypoints;
    if (!wps || wps.length < 2) return;

    // ── 建立 SampledPositionProperty ──────────────────────────────
    // 將每個航點的 time_sec 映射到 JulianDate（依 speedMul 壓縮）
    const positionProperty = new Cesium.SampledPositionProperty();
    positionProperty.setInterpolationOptions({
      interpolationDegree: 2,
      interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
    });

    wps.forEach(wp => {
      // 動畫時間 = 任務時間 / 速度倍率
      const animT = wp.time_sec / speedMul;
      const julianT = Cesium.JulianDate.addSeconds(startTime, animT, new Cesium.JulianDate());
      const cartPos = Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt);
      positionProperty.addSample(julianT, cartPos);
    });

    // ── 特徵 1：真實的俯衝姿態 (Velocity Orientation) ──────────────
    // 使用 VelocityOrientationProperty 將 UAV 3D 模型綁定到動態軌跡上
    // 當 UAV 進入「末端俯衝段」時，機頭會自動朝下呈現真實的俯衝攻擊姿態
    const orientationProperty = new Cesium.VelocityOrientationProperty(positionProperty);

    // ── 建立 UAV Entity（使用 GLB 模型 + fallback）──────────────────
    const uavEntity = viewer.entities.add({
      name: tr.uav_name || ('UCAV-' + tr.uav_id),
      position: positionProperty,
      orientation: orientationProperty,
      // 使用固定翼 GLB 模型
      model: (_PLANE_MODEL_URI && _PLANE_MODEL_URI.length > 5) ? {
        uri: _PLANE_MODEL_URI,
        minimumPixelSize: 48,
        maximumScale: 200,
        scale: 0.015,
        color: cesiumColor.withAlpha(0.95),
        colorBlendMode: Cesium.ColorBlendMode.MIX,
        colorBlendAmount: 0.4,
        silhouetteColor: cesiumColor,
        silhouetteSize: 2.0,
      } : undefined,
      // fallback: 如果沒有 GLB 模型，使用 point + label
      point: (!_PLANE_MODEL_URI || _PLANE_MODEL_URI.length <= 5) ? {
        pixelSize: 14,
        color: cesiumColor,
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      } : undefined,
      label: {
        text: tr.uav_name || ('UCAV-' + tr.uav_id),
        font: 'bold 11px monospace',
        fillColor: cesiumColor,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -24),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        scale: 0.85,
      },

      // ── 特徵 2：動態戰術軌跡尾跡 (Tactical Fading Trails) ────────
      // 使用 PathGraphics 繪製飛行軌跡
      // 巡航階段：帶透明度的綠色；俯衝階段：高亮血紅色
      path: {
        // 軌跡顯示：向前 0 秒（不預覽），向後顯示全部已飛過的路程
        leadTime: 0,
        trailTime: animDurationSec,
        width: 4,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.15,
          color: cesiumColor.withAlpha(0.6),
        }),
      },
    });
    strikeUavEntities.push(uavEntity);

    // ── 靜態軌跡折線（分色顯示：巡航段綠色 + 俯衝段紅色）────────────
    // 巡航段航點
    const cruiseWps = wps.filter(wp => wp.segment === 'cruise');
    const diveWps   = wps.filter(wp => wp.segment === 'dive');

    // 巡航段折線 — 帶透明度的綠色
    if (cruiseWps.length >= 2) {
      const cruisePositions = cruiseWps.map(wp =>
        Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt)
      );
      // 銜接：加入俯衝段第一個點（如果有）
      if (diveWps.length > 0) {
        cruisePositions.push(
          Cesium.Cartesian3.fromDegrees(diveWps[0].lon, diveWps[0].lat, diveWps[0].alt)
        );
      }
      const cruiseLine = viewer.entities.add({
        name: tr.uav_name + ' 巡航段',
        polyline: {
          positions: cruisePositions,
          width: 6,
          material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.1,
            color: Cesium.Color.fromCssColorString('rgba(0, 255, 100, 0.5)'),
          }),
          arcType: Cesium.ArcType.NONE,
        },
      });
      strikeTrailEntities.push(cruiseLine);
    }

    // 俯衝段折線 — 高亮血紅色加粗，強烈暗示攻擊行為
    if (diveWps.length >= 2) {
      const divePositions = diveWps.map(wp =>
        Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt)
      );
      const diveLine = viewer.entities.add({
        name: tr.uav_name + ' 俯衝段',
        polyline: {
          positions: divePositions,
          width: 10,
          material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.25,
            color: Cesium.Color.RED.withAlpha(0.9),
          }),
          arcType: Cesium.ArcType.NONE,
        },
      });
      strikeTrailEntities.push(diveLine);
    }

    // ── 特徵 3 & 4：目標鎖定射線 + 命中爆炸 ─────────────────────────
    // 這些效果需要在動畫進行中動態觸發，使用 onTick 監聽器
    const targetInfo = targets.find(t => t.id === tr.target_id);
    if (!targetInfo) return;

    const tgtPos = Cesium.Cartesian3.fromDegrees(targetInfo.lon, targetInfo.lat, targetInfo.alt || 0);
    const diveStartTimeSec = (tr.dive_start_index >= 0 && tr.dive_start_index < wps.length)
      ? wps[tr.dive_start_index].time_sec : 0;
    const impactTimeSec = (tr.impact_index >= 0 && tr.impact_index < wps.length)
      ? wps[tr.impact_index].time_sec : maxTimeSec;

    // 鎖定射線 entity（初始隱藏）
    // 使用 PolylineDashMaterialProperty 模擬雷射導引/光電鎖定視覺效果
    const laserEntity = viewer.entities.add({
      name: tr.uav_name + ' 鎖定射線',
      polyline: {
        positions: new Cesium.CallbackProperty(function() {
          // 動態取得 UAV 當前位置
          const now = viewer.clock.currentTime;
          const uavPos = positionProperty.getValue(now);
          if (!uavPos) return [tgtPos, tgtPos];
          return [uavPos, tgtPos];
        }, false),
        width: 3,
        material: new Cesium.PolylineDashMaterialProperty({
          color: Cesium.Color.RED.withAlpha(0.85),
          dashLength: 16.0,
          dashPattern: parseInt('1100110011001100', 2),
        }),
        arcType: Cesium.ArcType.NONE,
      },
      show: false,  // 進入俯衝段時才顯示
    });
    strikeLaserEntities.push(laserEntity);

    // 爆炸特效 entity（初始隱藏）
    // 使用擴張的 EllipseGraphics 模擬巡飛彈命中目標的爆炸衝擊波
    const blastEntity = viewer.entities.add({
      name: tr.uav_name + ' 命中爆炸',
      position: Cesium.Cartesian3.fromDegrees(targetInfo.lon, targetInfo.lat, 2),
      ellipse: {
        semiMajorAxis: 10,
        semiMinorAxis: 10,
        material: Cesium.Color.ORANGERED.withAlpha(0.8),
        outline: true,
        outlineColor: Cesium.Color.YELLOW.withAlpha(0.6),
        outlineWidth: 3,
        height: 0,
      },
      show: false,
    });
    strikeBlastEntities.push(blastEntity);

    // ── 動畫更新回調：控制鎖定射線、爆炸特效的觸發時機 ──────────────
    // 儲存此軌跡的動畫狀態
    const _animState = {
      laserShown: false,   // 鎖定射線是否已顯示
      impacted: false,     // 是否已命中
      blastPhase: 0,       // 爆炸擴張進度
    };

    // 每個 UCAV 的時間軸監聽器
    const tickListener = function(clock) {
      const now = clock.currentTime;
      // 計算當前動畫對應的任務時間（秒）
      const elapsedAnimSec = Cesium.JulianDate.secondsDifference(now, startTime);
      const currentMissionTime = elapsedAnimSec * speedMul;

      // ── 俯衝階段判定：進入俯衝時顯示鎖定射線 ────────────────────
      if (currentMissionTime >= diveStartTimeSec && !_animState.laserShown) {
        // UAV 進入俯衝段 → 啟動目標鎖定射線
        laserEntity.show = true;
        _animState.laserShown = true;

        // 切換此 UAV 的軌跡尾跡為紅色（動態視覺強調）
        if (uavEntity.path) {
          uavEntity.path.material = new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.3,
            color: Cesium.Color.RED.withAlpha(0.9),
          });
          uavEntity.path.width = 8;
        }
      }

      // ── 命中判定：到達目標座標 → 隱藏 UAV + 觸發爆炸特效 ──────────
      if (currentMissionTime >= impactTimeSec && !_animState.impacted) {
        _animState.impacted = true;

        // 隱藏 UAV 模型（模擬爆炸損毀）
        uavEntity.show = false;
        // 隱藏鎖定射線
        laserEntity.show = false;

        // 觸發爆炸擴張動畫
        blastEntity.show = true;
        _strikeAnimateExplosion(blastEntity, targetInfo);

        // 移除此軌跡的 tick 監聽器
        viewer.clock.onTick.removeEventListener(tickListener);
      }
    };

    viewer.clock.onTick.addEventListener(tickListener);
    // 記錄以便清除
    if (!window._strikeTickListeners) window._strikeTickListeners = [];
    window._strikeTickListeners.push(tickListener);
  });

  // ── 飛向場景 ──────────────────────────────────────────────────────
  _strikeAnimRunning = true;
  viewer.flyTo(viewer.entities, { duration: 1.5 });
}

/**
 * 命中爆炸擴張動畫
 *
 * 在目標位置生成擴張的 EllipseGraphics，模擬巡飛彈命中目標的
 * 爆炸衝擊波視覺效果：亮橘黃色 → 漸變透明 → 消散
 *
 * @param {Cesium.Entity} blastEntity  爆炸 entity
 * @param {object} targetInfo          目標資訊 {lat, lon, alt}
 */
function _strikeAnimateExplosion(blastEntity, targetInfo) {
  const maxRadius = 120;       // 衝擊波最大半徑 (m)
  const expandDuration = 1500; // 擴張持續時間 (ms)
  const fadeDuration = 1000;   // 消散時間 (ms)
  const startMs = Date.now();

  function _expand() {
    const elapsed = Date.now() - startMs;

    if (elapsed < expandDuration) {
      // 階段一：快速擴張（easeOut 曲線）
      const t = elapsed / expandDuration;
      const eased = 1 - Math.pow(1 - t, 3);  // cubic easeOut
      const radius = 10 + (maxRadius - 10) * eased;
      const alpha = 0.85 * (1 - t * 0.3);    // 輕微透明化

      blastEntity.ellipse.semiMajorAxis = radius;
      blastEntity.ellipse.semiMinorAxis = radius;
      blastEntity.ellipse.material = Cesium.Color.ORANGERED.withAlpha(alpha);
      blastEntity.ellipse.outlineColor = Cesium.Color.YELLOW.withAlpha(alpha * 0.8);

      requestAnimationFrame(_expand);
    } else if (elapsed < expandDuration + fadeDuration) {
      // 階段二：維持最大半徑，漸變透明消散
      const fadeT = (elapsed - expandDuration) / fadeDuration;
      const alpha = 0.6 * (1 - fadeT);

      blastEntity.ellipse.semiMajorAxis = maxRadius;
      blastEntity.ellipse.semiMinorAxis = maxRadius;
      blastEntity.ellipse.material = Cesium.Color.ORANGE.withAlpha(alpha);
      blastEntity.ellipse.outlineColor = Cesium.Color.YELLOW.withAlpha(alpha * 0.5);

      requestAnimationFrame(_expand);
    } else {
      // 動畫結束：保留淡色殘留效果（戰場標記）
      blastEntity.ellipse.semiMajorAxis = maxRadius;
      blastEntity.ellipse.semiMinorAxis = maxRadius;
      blastEntity.ellipse.material = Cesium.Color.DARKRED.withAlpha(0.1);
      blastEntity.ellipse.outlineColor = Cesium.Color.RED.withAlpha(0.2);
    }
  }
  _expand();
}

/**
 * 清除打擊 tick 監聽器
 */
function _strikeClearTickListeners() {
  if (window._strikeTickListeners) {
    window._strikeTickListeners.forEach(fn => {
      try { viewer.clock.onTick.removeEventListener(fn); } catch(_){}
    });
    window._strikeTickListeners = [];
  }
}

// 覆寫 strikeClearAll 以包含 tick 清理
const _origStrikeClear = strikeClearAll;
strikeClearAll = function() {
  _strikeClearTickListeners();
  _origStrikeClear();
};

// ══════════════════════════════════════════════════════════════════════
//  VTOL 群飛 3D 軌跡視覺化（垂直起降 + 固定翼巡航 + 轉場標記）
// ══════════════════════════════════════════════════════════════════════
var _vtolEntities = [];           // 所有 VTOL 視覺化 Entity
var _vtolTransitionAnims = [];    // 轉場點閃爍動畫計時器

/**
 * drawVTOLSwarmPaths — 繪製 VTOL 群飛三階段 3D 軌跡
 *
 * @param {string} uavDataJson — JSON 陣列，每個元素格式：
 *   { uav_id, color, home:{lat,lon}, transition_alt,
 *     takeoff_segment:[{lat,lon,alt},...],
 *     cruise_segment:[{lat,lon,alt},...],
 *     landing_segment:[{lat,lon,alt},...],
 *     fw_transition_pt:{lat,lon,alt},
 *     mc_transition_pt:{lat,lon,alt} }
 */
function drawVTOLSwarmPaths(uavDataJson) {
  // 清除舊的 VTOL 視覺化
  vtolClearAll();

  const uavList = (typeof uavDataJson === 'string')
    ? JSON.parse(uavDataJson) : uavDataJson;
  if (!uavList || !uavList.length) return;

  uavList.forEach(function(uav) {
    const color = Cesium.Color.fromCssColorString(uav.color);
    const uavLabel = 'VTOL-' + uav.uav_id;

    // ── 1. 垂直起飛段：黃色粗實線 + 向上箭頭標記 ─────────────────
    if (uav.takeoff_segment && uav.takeoff_segment.length >= 2) {
      const tkPts = [];
      uav.takeoff_segment.forEach(function(p) {
        tkPts.push(p.lon, p.lat, p.alt || 0);
      });
      // 黃色粗實線
      var tkEntity = viewer.entities.add({
        name: uavLabel + ' 起飛段',
        polyline: {
          positions: Cesium.Cartesian3.fromDegreesArrayHeights(tkPts),
          width: 6,
          material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.3,
            color: Cesium.Color.YELLOW,
          }),
          clampToGround: false,
        },
      });
      _vtolEntities.push(tkEntity);

      // 起飛底部：向上箭頭標記
      var tkBase = uav.takeoff_segment[0];
      var arrowUp = viewer.entities.add({
        name: uavLabel + ' 起飛↑',
        position: Cesium.Cartesian3.fromDegrees(tkBase.lon, tkBase.lat, (tkBase.alt || 0) + 5),
        billboard: {
          image: _vtolArrowSvg('up'),
          scale: 0.7,
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
          text: '▲ VTOL 起飛',
          font: 'bold 11px sans-serif',
          fillColor: Cesium.Color.YELLOW,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cesium.Cartesian2(0, -40),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      });
      _vtolEntities.push(arrowUp);
    }

    // ── 2. 固定翼巡航段：UAV 專屬顏色科技感虛線 ─────────────────
    if (uav.cruise_segment && uav.cruise_segment.length >= 2) {
      var crPts = [];
      uav.cruise_segment.forEach(function(p) {
        crPts.push(p.lon, p.lat, p.alt || 0);
      });
      var crEntity = viewer.entities.add({
        name: uavLabel + ' 巡航段',
        polyline: {
          positions: Cesium.Cartesian3.fromDegreesArrayHeights(crPts),
          width: 3,
          material: new Cesium.PolylineDashMaterialProperty({
            color: color.withAlpha(0.9),
            dashLength: 16.0,
            dashPattern: 255,       // 0xFF = 科技感短虛線
          }),
          clampToGround: false,
        },
      });
      _vtolEntities.push(crEntity);

      // 巡航航點小圓點
      uav.cruise_segment.forEach(function(p, i) {
        if (i % 3 !== 0 && i !== uav.cruise_segment.length - 1) return; // 每3點標一個
        var wpDot = viewer.entities.add({
          position: Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.alt || 0),
          point: {
            pixelSize: 4,
            color: color.withAlpha(0.7),
            outlineColor: Cesium.Color.WHITE,
            outlineWidth: 1,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
        });
        _vtolEntities.push(wpDot);
      });
    }

    // ── 3. 垂直降落段：黃色粗實線 + 向下箭頭標記 ─────────────────
    if (uav.landing_segment && uav.landing_segment.length >= 2) {
      var ldPts = [];
      uav.landing_segment.forEach(function(p) {
        ldPts.push(p.lon, p.lat, p.alt || 0);
      });
      var ldEntity = viewer.entities.add({
        name: uavLabel + ' 降落段',
        polyline: {
          positions: Cesium.Cartesian3.fromDegreesArrayHeights(ldPts),
          width: 6,
          material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.3,
            color: Cesium.Color.YELLOW,
          }),
          clampToGround: false,
        },
      });
      _vtolEntities.push(ldEntity);

      // 降落底部：向下箭頭標記
      var ldEnd = uav.landing_segment[uav.landing_segment.length - 1];
      var arrowDn = viewer.entities.add({
        name: uavLabel + ' 降落↓',
        position: Cesium.Cartesian3.fromDegrees(ldEnd.lon, ldEnd.lat, (ldEnd.alt || 0) + 5),
        billboard: {
          image: _vtolArrowSvg('down'),
          scale: 0.7,
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
          text: '▼ VTOL 降落',
          font: 'bold 11px sans-serif',
          fillColor: Cesium.Color.YELLOW,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cesium.Cartesian2(0, -40),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      });
      _vtolEntities.push(arrowDn);
    }

    // ── 4. 轉場點標記：閃爍菱形 — FW 轉換點（藍）與 MC 轉換點（橘）──
    if (uav.fw_transition_pt) {
      _addVTOLTransitionMarker(uav.fw_transition_pt, 'FW',
        Cesium.Color.DODGERBLUE, uavLabel + ' →FW');
    }
    if (uav.mc_transition_pt) {
      _addVTOLTransitionMarker(uav.mc_transition_pt, 'MC',
        Cesium.Color.ORANGERED, uavLabel + ' →MC');
    }
  }); // forEach uav

  // 鏡頭飛往第一架 UAV 的 Home
  if (uavList.length > 0 && uavList[0].home) {
    var h = uavList[0].home;
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(h.lon, h.lat, 800),
      orientation: { heading: 0, pitch: Cesium.Math.toRadians(-45), roll: 0 },
      duration: 1.5,
    });
  }
  console.log('[VTOL] drawVTOLSwarmPaths: 已繪製', uavList.length, '架 VTOL 軌跡');
}

/**
 * _addVTOLTransitionMarker — 在轉換點放置閃爍菱形標記
 */
function _addVTOLTransitionMarker(pt, modeName, markerColor, entityName) {
  var pos = Cesium.Cartesian3.fromDegrees(pt.lon, pt.lat, pt.alt || 0);

  // 閃爍橢球（菱形外觀）
  var blinkEntity = viewer.entities.add({
    name: entityName,
    position: pos,
    // 3D 菱形：壓扁的橢球
    ellipsoid: {
      radii: new Cesium.Cartesian3(15, 15, 30),
      material: markerColor.withAlpha(0.6),
      outline: true,
      outlineColor: markerColor,
      outlineWidth: 2,
    },
    label: {
      text: '⚡ ' + modeName + ' 轉換',
      font: 'bold 12px "Segoe UI",sans-serif',
      fillColor: markerColor,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2.5,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -50),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
  _vtolEntities.push(blinkEntity);

  // 閃爍動畫：週期性切換 alpha
  var _blinkOn = true;
  var blinkTimer = setInterval(function() {
    if (!blinkEntity || !blinkEntity.ellipsoid) {
      clearInterval(blinkTimer);
      return;
    }
    _blinkOn = !_blinkOn;
    try {
      blinkEntity.ellipsoid.material = _blinkOn
        ? markerColor.withAlpha(0.6)
        : markerColor.withAlpha(0.15);
    } catch(_e) { clearInterval(blinkTimer); }
  }, 500);
  _vtolTransitionAnims.push(blinkTimer);
}

/**
 * _vtolArrowSvg — 產生向上或向下箭頭 SVG data URI
 */
function _vtolArrowSvg(dir) {
  var path = dir === 'up'
    ? 'M24,44 L24,8 M16,16 L24,4 L32,16'   // 向上箭頭
    : 'M24,4 L24,40 M16,32 L24,44 L32,32';  // 向下箭頭
  return 'data:image/svg+xml,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48">' +
    '<path d="' + path + '" stroke="#FFD600" stroke-width="4" ' +
    'fill="none" stroke-linecap="round" stroke-linejoin="round"/>' +
    '</svg>'
  );
}

/**
 * vtolClearAll — 清除所有 VTOL 視覺化 Entity 與動畫
 */
function vtolClearAll() {
  _vtolTransitionAnims.forEach(function(t) { clearInterval(t); });
  _vtolTransitionAnims = [];
  _vtolEntities.forEach(function(e) {
    try { viewer.entities.remove(e); } catch(_){}
  });
  _vtolEntities = [];
  console.log('[VTOL] vtolClearAll: 已清除所有 VTOL 軌跡');
}

// ══════════════════════════════════════════════════════════════════
//  Terminal Dive 末端俯衝視覺化模組
//  功能：逼真俯衝姿態 + 戰術軌跡變色 + 目標鎖定射線 + 命中爆炸衝擊波
// ══════════════════════════════════════════════════════════════════

// 全域管理：儲存所有 Terminal Dive 相關的 Entity 與清理 handles
var _terminalDiveEntities = {};  // uavId → { uav, laser, path, shockwave, ... }

/**
 * renderTerminalDive — 末端俯衝完整視覺化
 *
 * @param {string|number} uavId            - UAV 識別碼
 * @param {Array} trajectoryData           - 航跡資料陣列
 *   格式：[{ time: ISO8601字串, lat: 度, lon: 度, alt: 公尺 }, ...]
 *   時間必須為連續遞增的 ISO 8601 格式（如 "2026-01-01T00:00:00Z"）
 * @param {{ lat: number, lon: number, alt: number }} targetPos - 地面目標座標
 * @param {number} diveThreshold           - 俯衝觸發距離門檻（公尺）
 *
 * 視覺特徵：
 *   1. VelocityOrientationProperty — 機頭自動對齊速度向量，俯衝時真實朝下
 *   2. 動態軌跡變色 — 巡航=半透明藍，俯衝=高亮血紅+加粗
 *   3. 目標鎖定射線 — 進入俯衝範圍後顯示紅色虛線雷射
 *   4. 命中爆炸衝擊波 — 撞擊地面後環形擴散動畫
 */
function renderTerminalDive(uavId, trajectoryData, targetPos, diveThreshold) {

  // ── 清除同 ID 的舊資料 ──────────────────────────────────────
  clearTerminalDive(uavId);

  if (!trajectoryData || trajectoryData.length < 2) {
    console.warn('[TerminalDive] trajectoryData 不足 2 點，無法渲染');
    return;
  }

  // ── 將目標座標轉為 Cartesian3 ──────────────────────────────
  const targetCart = Cesium.Cartesian3.fromDegrees(
    targetPos.lon, targetPos.lat, targetPos.alt || 0
  );

  // ════════════════════════════════════════════════════════════
  //  階段 1：建立 SampledPositionProperty（時間插值位置）
  // ════════════════════════════════════════════════════════════
  // SampledPositionProperty 會根據 Cesium Clock 的當前時間，
  // 在取樣點之間進行三次 Hermite 插值，產生平滑的飛行軌跡。
  const positionProperty = new Cesium.SampledPositionProperty();

  // 設定插值選項：Hermite 三次插值，5 點窗格
  // 讓俯衝段的急劇高度變化也能平滑過渡，避免跳動
  positionProperty.setInterpolationOptions({
    interpolationDegree: 3,
    interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
  });

  // 將航跡資料逐點加入 SampledPositionProperty
  const startTime = Cesium.JulianDate.fromIso8601(trajectoryData[0].time);
  const stopTime  = Cesium.JulianDate.fromIso8601(
    trajectoryData[trajectoryData.length - 1].time
  );

  for (let i = 0; i < trajectoryData.length; i++) {
    const pt = trajectoryData[i];
    const julianTime = Cesium.JulianDate.fromIso8601(pt.time);
    const cartesian  = Cesium.Cartesian3.fromDegrees(pt.lon, pt.lat, pt.alt);
    positionProperty.addSample(julianTime, cartesian);
  }

  // ════════════════════════════════════════════════════════════
  //  階段 2：VelocityOrientationProperty — 速度向量姿態
  // ════════════════════════════════════════════════════════════
  // VelocityOrientationProperty 會根據 positionProperty 的時間微分
  // （即瞬時速度向量），自動計算 Entity 的 orientation (Quaternion)。
  //
  // 關鍵效果：當 UAV 進入俯衝航段（Z 軸急劇下降）時，
  // 速度向量會指向地面 → 飛機機頭自然朝下俯衝，
  // 完美貼合物理運動方向，無需手動計算 heading/pitch/roll。
  //
  // 注意：GLB 模型預設機頭朝向可能需要校正，
  // 此處使用 modelMatrix 或 Entity.orientation 直接套用即可。
  const orientationProperty = new Cesium.VelocityOrientationProperty(
    positionProperty
  );

  // ════════════════════════════════════════════════════════════
  //  狀態追蹤變數（閉包內共享，供 CallbackProperty 即時讀取）
  // ════════════════════════════════════════════════════════════
  // _impacted: 是否已命中目標（一旦 true 不可逆）
  // _currentDistance: UAV 到目標的即時距離（效能優化：每幀只算一次）
  let _impacted = false;
  let _currentDistance = Infinity;
  let _lastComputeTime = 0; // 節流用：避免 CallbackProperty 每幀重複計算

  // 快取計算用的暫存向量（避免每幀 new Cartesian3 造成 GC 壓力）
  const _scratchCart = new Cesium.Cartesian3();

  /**
   * 即時計算 UAV 與目標的距離（效能最佳化版本）
   * - 使用 Cesium.Cartesian3.distance 計算三維歐式距離
   * - 節流：同一幀只計算一次，多個 CallbackProperty 共享結果
   */
  function _computeDistance(time) {
    const ts = Cesium.JulianDate.toDate(time).getTime();
    if (Math.abs(ts - _lastComputeTime) < 16) return _currentDistance; // 60fps 節流
    _lastComputeTime = ts;

    const uavPos = positionProperty.getValue(time, _scratchCart);
    if (!uavPos) { _currentDistance = Infinity; return _currentDistance; }
    _currentDistance = Cesium.Cartesian3.distance(uavPos, targetCart);
    return _currentDistance;
  }

  // ════════════════════════════════════════════════════════════
  //  階段 3：建立 UAV Entity（含 3D 模型 + 速度姿態）
  // ════════════════════════════════════════════════════════════
  const uavColor = _UAV_COLORS[
    (typeof uavId === 'number' ? uavId : parseInt(uavId) || 0) % _UAV_COLORS.length
  ];

  const uavEntity = viewer.entities.add({
    id: 'termDive_uav_' + uavId,
    name: 'Terminal Dive UAV-' + uavId,
    position: positionProperty,

    // VelocityOrientationProperty：機頭永遠對齊飛行方向
    orientation: orientationProperty,

    // 3D 模型（使用專案內的固定翼 GLB）
    model: (_PLANE_MODEL_URI && _PLANE_MODEL_URI.length > 0) ? {
      uri: _PLANE_MODEL_URI,
      minimumPixelSize: 48,
      maximumScale: 80,
      scale: 0.015,
      // GLB 姿態校正：模型原始朝向 → heading+90°, pitch+90°
      // VelocityOrientationProperty 已處理方向，此處用 nodeTransformations
      // 若模型已正確朝向則不需額外校正
    } : undefined,

    // 備用：無 GLB 時用 billboard
    billboard: (!_PLANE_MODEL_URI || _PLANE_MODEL_URI.length === 0) ? {
      image: _uavSvgPlane(uavColor),
      scale: 1.2,
      alignedAxis: Cesium.Cartesian3.UNIT_Z,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    } : undefined,

    label: {
      text: '💀 UAV' + uavId + ' [DIVE]',
      font: 'bold 13px "Segoe UI",sans-serif',
      fillColor: Cesium.Color.RED,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 3,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -40),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
      // 命中後隱藏 label
      show: new Cesium.CallbackProperty(function() { return !_impacted; }, false),
    },

    // 命中後整體隱藏
    show: new Cesium.CallbackProperty(function() { return !_impacted; }, false),

    // ════════════════════════════════════════════════════════
    //  階段 4：動態戰術軌跡尾跡 (Tactical Trajectory Color Shift)
    // ════════════════════════════════════════════════════════
    // CallbackProperty 即時計算：
    //   距離 > diveThreshold → 半透明藍（巡航階段）
    //   距離 ≤ diveThreshold → 高亮血紅（俯衝攻擊階段）
    path: {
      leadTime: 0,
      trailTime: 600, // 顯示最近 600 秒的軌跡

      // 線寬動態切換：巡航=3px，俯衝=6px
      width: new Cesium.CallbackProperty(function(time) {
        _computeDistance(time);
        return (_currentDistance <= diveThreshold && !_impacted) ? 6 : 3;
      }, false),

      // 材質動態切換：巡航=藍色半透明，俯衝=血紅色
      // CallbackProperty 包裝 ColorMaterialProperty 的 color
      material: new Cesium.ColorMaterialProperty(
        new Cesium.CallbackProperty(function(time) {
          _computeDistance(time);
          if (_impacted) return Cesium.Color.GRAY.withAlpha(0.3);
          if (_currentDistance <= diveThreshold) {
            // 俯衝階段：血紅色，越近越亮
            const ratio = Math.max(0, _currentDistance / diveThreshold);
            return Cesium.Color.RED.withAlpha(0.7 + 0.3 * (1 - ratio));
          }
          // 巡航階段：半透明藍
          return Cesium.Color.DODGERBLUE.withAlpha(0.6);
        }, false)
      ),
    },
  });

  // ════════════════════════════════════════════════════════════
  //  階段 5：末端目標鎖定射線 (Target Lock-on Laser)
  // ════════════════════════════════════════════════════════════
  // 使用 CallbackProperty 動態更新 polyline 的兩端座標：
  //   一端 = UAV 即時位置
  //   另一端 = 地面目標座標
  //
  // PolylineDashMaterialProperty：紅色虛線，模擬光電追蹤雷射
  // show 動態切換：僅在俯衝範圍內且尚未命中時顯示
  const laserEntity = viewer.entities.add({
    id: 'termDive_laser_' + uavId,
    name: 'Lock-on Laser UAV-' + uavId,

    // 顯示條件：距離 ≤ 門檻 且 尚未命中
    show: new Cesium.CallbackProperty(function(time) {
      _computeDistance(time);
      return (_currentDistance <= diveThreshold) && !_impacted;
    }, false),

    polyline: {
      // 動態座標：[UAV 當前位置, 目標位置]
      positions: new Cesium.CallbackProperty(function(time) {
        const uavPos = positionProperty.getValue(time);
        if (!uavPos) return [targetCart, targetCart]; // fallback
        return [uavPos, targetCart];
      }, false),

      width: 2,

      // 紅色虛線材質 — 模擬光電鎖定雷射
      material: new Cesium.PolylineDashMaterialProperty({
        color: Cesium.Color.RED.withAlpha(0.85),
        dashLength: 16,   // 虛線段長（像素）
        dashPattern: 255,  // 0xFF = 實線段佔比
      }),
    },
  });

  // ════════════════════════════════════════════════════════════
  //  階段 6：地面目標標記（十字靶心）
  // ════════════════════════════════════════════════════════════
  const targetEntity = viewer.entities.add({
    id: 'termDive_target_' + uavId,
    name: 'Target-' + uavId,
    position: targetCart,
    point: {
      pixelSize: 14,
      color: Cesium.Color.RED,
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    label: {
      text: '🎯 TARGET',
      font: 'bold 11px monospace',
      fillColor: Cesium.Color.ORANGERED,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -22),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
      show: new Cesium.CallbackProperty(function() { return !_impacted; }, false),
    },
  });

  // ════════════════════════════════════════════════════════════
  //  階段 7：命中偵測 + 爆炸衝擊波 (Impact & Detonation)
  // ════════════════════════════════════════════════════════════
  //
  // 偵測策略：監聽 Cesium Clock tick，每幀檢查：
  //   1. 當前時間是否超過軌跡終點時間
  //   2. UAV 與目標距離是否 < 5 公尺（直接命中判定）
  //
  // 命中後：
  //   - 隱藏 UAV Entity (show=false) + 移除鎖定射線
  //   - 在目標位置生成衝擊波圓盤 (EllipseGraphics)
  //   - 半徑 1m → 50m，透明度 1.0 → 0.0，歷時 1.5 秒
  //   - 動畫結束後自動移除

  const IMPACT_PROXIMITY = 5.0;  // 命中判定距離（公尺）
  const SHOCKWAVE_DURATION = 1.5; // 衝擊波動畫持續時間（秒）
  const SHOCKWAVE_MAX_RADIUS = 50.0; // 衝擊波最大半徑（公尺）

  // 衝擊波動畫狀態
  let _shockwaveStartTime = null;
  let _shockwaveEntity = null;

  /**
   * 觸發爆炸衝擊波動畫
   * 建立 EllipseGraphics 貼地圓盤，透過 CallbackProperty 驅動：
   *   - semiMajorAxis / semiMinorAxis：1m → 50m（1.5 秒）
   *   - material alpha：1.0 → 0.0（同步消散）
   */
  function _triggerExplosion() {
    _impacted = true;
    _shockwaveStartTime = Date.now();

    console.log('[TerminalDive] UAV-' + uavId + ' 命中目標！觸發爆炸衝擊波');

    _shockwaveEntity = viewer.entities.add({
      id: 'termDive_boom_' + uavId,
      name: 'Shockwave-' + uavId,
      position: targetCart,

      ellipse: {
        // 半徑動態擴張：CallbackProperty 每幀計算
        semiMajorAxis: new Cesium.CallbackProperty(function() {
          const elapsed = (Date.now() - _shockwaveStartTime) / 1000;
          const t = Math.min(elapsed / SHOCKWAVE_DURATION, 1.0);
          // ease-out 曲線：初期快速擴張，後期減速
          const eased = 1 - Math.pow(1 - t, 3);
          return 1.0 + (SHOCKWAVE_MAX_RADIUS - 1.0) * eased;
        }, false),

        semiMinorAxis: new Cesium.CallbackProperty(function() {
          const elapsed = (Date.now() - _shockwaveStartTime) / 1000;
          const t = Math.min(elapsed / SHOCKWAVE_DURATION, 1.0);
          const eased = 1 - Math.pow(1 - t, 3);
          return 1.0 + (SHOCKWAVE_MAX_RADIUS - 1.0) * eased;
        }, false),

        // 材質：橘紅色半透明 → 漸變消散
        material: new Cesium.ColorMaterialProperty(
          new Cesium.CallbackProperty(function() {
            const elapsed = (Date.now() - _shockwaveStartTime) / 1000;
            const t = Math.min(elapsed / SHOCKWAVE_DURATION, 1.0);
            // 顏色從亮橘紅漸變到深紅
            const r = 1.0;
            const g = 0.4 * (1 - t);
            const b = 0.1 * (1 - t);
            const alpha = 1.0 - t;  // 透明度：1.0 → 0.0
            return new Cesium.Color(r, g, b, alpha);
          }, false)
        ),

        // 貼地顯示
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        outline: true,
        outlineColor: new Cesium.CallbackProperty(function() {
          const elapsed = (Date.now() - _shockwaveStartTime) / 1000;
          const t = Math.min(elapsed / SHOCKWAVE_DURATION, 1.0);
          return Cesium.Color.ORANGERED.withAlpha(Math.max(0, 1.0 - t * 1.5));
        }, false),
        outlineWidth: 2,
      },
    });

    // 衝擊波生命週期管理：1.5 秒後自動移除
    setTimeout(function() {
      if (_shockwaveEntity) {
        try { viewer.entities.remove(_shockwaveEntity); } catch(_) {}
        _shockwaveEntity = null;
      }
      console.log('[TerminalDive] UAV-' + uavId + ' 衝擊波動畫結束，已清理');
    }, SHOCKWAVE_DURATION * 1000 + 200); // +200ms 緩衝
  }

  // ── Clock tick 監聽器：偵測命中 ──────────────────────────
  function _onClockTick(clock) {
    if (_impacted) return; // 已命中，不再檢查

    const currentTime = clock.currentTime;

    // 檢查 1：是否超過軌跡終點時間
    if (Cesium.JulianDate.greaterThan(currentTime, stopTime)) {
      _triggerExplosion();
      return;
    }

    // 檢查 2：UAV 是否已非常接近目標（直接命中判定）
    _computeDistance(currentTime);
    if (_currentDistance < IMPACT_PROXIMITY) {
      _triggerExplosion();
      return;
    }
  }

  // 註冊 Clock tick 監聽
  const _tickRemover = viewer.clock.onTick.addEventListener(_onClockTick);

  // ── 設定 Clock 時間範圍 ──────────────────────────────────
  viewer.clock.startTime = Cesium.JulianDate.clone(startTime);
  viewer.clock.stopTime  = Cesium.JulianDate.addSeconds(
    stopTime, 3, new Cesium.JulianDate()  // 多留 3 秒看爆炸
  );
  viewer.clock.currentTime = Cesium.JulianDate.clone(startTime);
  viewer.clock.clockRange = Cesium.ClockRange.CLAMPED; // 到終點停住
  viewer.clock.multiplier = 1.0; // 即時速度播放
  viewer.clock.shouldAnimate = true;

  // 自動追蹤此 UAV
  viewer.trackedEntity = uavEntity;

  // ── 儲存所有 Entity 以供清理 ────────────────────────────
  _terminalDiveEntities[uavId] = {
    uav: uavEntity,
    laser: laserEntity,
    target: targetEntity,
    shockwave: null, // 爆炸時才建立
    tickRemover: _tickRemover,
    positionProperty: positionProperty,
  };

  console.log(
    '[TerminalDive] UAV-' + uavId + ' 已初始化：' +
    trajectoryData.length + ' 航點，俯衝門檻=' + diveThreshold + 'm'
  );
}

/**
 * clearTerminalDive — 清除指定 UAV 的所有末端俯衝視覺化
 */
function clearTerminalDive(uavId) {
  const data = _terminalDiveEntities[uavId];
  if (!data) return;

  // 移除 Clock tick 監聽
  if (data.tickRemover) { data.tickRemover(); }

  // 移除所有 Entity
  ['uav', 'laser', 'target', 'shockwave'].forEach(function(key) {
    if (data[key]) {
      try { viewer.entities.remove(data[key]); } catch(_) {}
    }
  });

  delete _terminalDiveEntities[uavId];
  console.log('[TerminalDive] UAV-' + uavId + ' 已清除');
}

/**
 * clearAllTerminalDives — 清除所有末端俯衝視覺化
 */
function clearAllTerminalDives() {
  Object.keys(_terminalDiveEntities).forEach(function(id) {
    clearTerminalDive(id);
  });
  console.log('[TerminalDive] 已清除所有末端俯衝');
}


console.log('[Cesium 3D] 初始化完成（含戰術視覺化 + 蜂群打擊 + VTOL 軌跡 + 末端俯衝模組）');
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════════════
#  URL Bridge（重用 ClickCapturePage 邏輯）
# ══════════════════════════════════════════════════════════════════════
class CesiumPage(QWebEnginePage):
    def __init__(self, parent, widget):
        super().__init__(parent)
        self.widget = widget

    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        prefix = {0: 'ℹ️', 1: '⚠️', 2: '❌'}.get(level, '🔹')
        print(f"{prefix} [Cesium JS] {message}")

    def acceptNavigationRequest(self, url, nav_type, is_main_frame):
        url_str = url.toString()
        if url_str.startswith('pyqt://click/'):
            try:
                parts = url_str.replace('pyqt://click/', '').split('/')
                lat, lon = float(parts[0]), float(parts[1])
                self.widget.on_map_clicked(lat, lon)
            except Exception as e:
                print(f'❌ [Cesium] 解析點擊失敗: {e}')
            return False
        if url_str.startswith('pyqt://circle_draw/'):
            try:
                parts = url_str.replace('pyqt://circle_draw/', '').split('/')
                lat, lon, radius = float(parts[0]), float(parts[1]), float(parts[2])
                self.widget.on_circle_draw_complete(lat, lon, radius)
            except Exception as e:
                print(f'❌ [Cesium] 解析圓形失敗: {e}')
            return False
        if url_str.startswith('pyqt://nfz_poly_done/'):
            encoded = unquote(url_str[len('pyqt://nfz_poly_done/'):])
            vertices = []
            for pair in encoded.split('|'):
                parts = pair.split(',')
                if len(parts) == 2:
                    try:
                        vertices.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        pass
            self.widget.on_nfz_poly_done(vertices)
            return False
        return True


# ══════════════════════════════════════════════════════════════════════
#  CesiumMapWidget
# ══════════════════════════════════════════════════════════════════════
class CesiumMapWidget(QWidget):
    """3D 地圖組件，介面與 MapWidget 相容"""

    # ── 與 MapWidget 相同的信號 ───────────────────────────────────────
    corner_added       = pyqtSignal(float, float)
    corner_moved       = pyqtSignal(int, float, float)
    circle_defined     = pyqtSignal(float, float, float)
    nfz_polygon_drawn  = pyqtSignal(list)
    nfz_circle_drawn   = pyqtSignal(float, float, float)
    strike_target_added = pyqtSignal(float, float)  # 打擊目標標記

    # ── Cesium Ion Token（可在設定中覆寫）────────────────────────────
    CESIUM_TOKEN: str = ''   # 留空 = 無地形 Token，衛星影像仍正常

    def __init__(self, parent=None):
        super().__init__(parent)
        self.corners: List[Tuple[float, float]] = []
        self.paths: list = []
        self.path_colors: list = []
        self._path_tooltips: list = []
        self.transit_paths: list = []
        self._circle_center: Optional[Tuple[float, float]] = None
        self._circle_radius_m: float = 0.0
        self._home_point: Optional[Tuple[float, float]] = None
        self._nfz_zones: list = []
        self._swarm_data: Optional[dict] = None
        self._fw_result: Optional[dict] = None   # fw_mission_result，用於 3D 高度
        self._last_altitude: float = 50.0        # 多旋翼路徑高度
        self._clamp_basic: bool = False          # 基本演算法 grid 是否貼地
        self._strike_marking: bool = False       # 打擊目標標記模式

        self._cesium_html_file = None

        self._init_ui()
        self._load_cesium()
        logger.info('Cesium 3D 地圖組件初始化完成')

    def __del__(self):
        if self._cesium_html_file:
            try:
                import os
                os.unlink(self._cesium_html_file.name)
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────
    # UI 初始化
    # ─────────────────────────────────────────────────────────────────
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.web_view = QWebEngineView()
        self.web_view.setMinimumSize(400, 400)
        self.web_view.setSizePolicy(
            self.web_view.sizePolicy().horizontalPolicy(),
            self.web_view.sizePolicy().verticalPolicy(),
        )
        self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)

        self._page = CesiumPage(self.web_view, self)
        self.web_view.setPage(self._page)

        ws = self._page.settings()
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.LocalStorageEnabled, True)
        ws.setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)

        layout.addWidget(self.web_view)

    def _load_cesium(self):
        import os
        from pathlib import Path

        # 讀取 2D 地圖預設座標保持一致
        try:
            from config.settings import get_settings
            s = get_settings()
            init_lat = s.map.default_lat
            init_lon = s.map.default_lon
        except Exception:
            init_lat, init_lon = 23.702732, 120.419333
        init_height = 2000.0

        # ── 本地 / CDN 路徑解析 ──
        project_root = Path(__file__).resolve().parent.parent.parent
        local_cesium = project_root / 'assets' / 'cesium' / 'Build' / 'Cesium'
        local_js     = local_cesium / 'Cesium.js'
        local_css    = local_cesium / 'Widgets' / 'widgets.css'

        if local_js.exists() and local_css.exists():
            cesium_js  = local_js.as_uri()
            cesium_css = local_css.as_uri()
            base_url   = QUrl.fromLocalFile(str(local_cesium) + os.sep)
            logger.info(f'[Cesium] 使用本地資源: {local_cesium}')
        else:
            cesium_js  = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Cesium.js'
            cesium_css = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Widgets/widgets.css'
            base_url   = QUrl('https://cesium.com/')
            logger.info('[Cesium] 使用 CDN 線上資源')

        # ── GLB 3D 模型路徑 ──
        plane_glb = project_root / 'ui' / 'resources' / 'P25-ID.glb'
        plane_model_uri = plane_glb.as_uri() if plane_glb.exists() else ''
        logger.info(f'[Cesium] 固定翼 GLB 模型: {plane_model_uri or "未找到，使用 fallback"}')

        copter_glb = project_root / 'ui' / 'resources' / 'copter.glb'
        copter_model_uri = copter_glb.as_uri() if copter_glb.exists() else ''
        logger.info(f'[Cesium] 多旋翼 GLB 模型: {copter_model_uri or "未找到，使用 fallback"}')

        html = (_CESIUM_HTML
                .replace('__CESIUM_JS__',   cesium_js)
                .replace('__CESIUM_CSS__',  cesium_css)
                .replace('__CESIUM_TOKEN__', self.CESIUM_TOKEN)
                .replace('__INIT_LAT__',    str(init_lat))
                .replace('__INIT_LON__',    str(init_lon))
                .replace('__INIT_HEIGHT__', str(init_height))
                .replace('__PLANE_MODEL_URI__', plane_model_uri)
                .replace('__COPTER_MODEL_URI__', copter_model_uri))

        # 將 HTML 寫入暫存檔案並用 setUrl 載入，確保 file:// URI 能正常存取本地 GLB
        import tempfile
        self._cesium_html_file = tempfile.NamedTemporaryFile(
            mode='w', suffix='.html', dir=str(project_root),
            delete=False, encoding='utf-8'
        )
        self._cesium_html_file.write(html)
        self._cesium_html_file.close()
        html_url = QUrl.fromLocalFile(self._cesium_html_file.name)
        logger.info(f'[Cesium] 載入 HTML: {html_url.toString()}')
        self.web_view.setUrl(html_url)

    # ─────────────────────────────────────────────────────────────────
    # JS 橋接：Python → Cesium
    # ─────────────────────────────────────────────────────────────────
    def _js(self, script: str):
        """執行 JavaScript（非同步）"""
        self._page.runJavaScript(script)

    @staticmethod
    def _pts_to_json(pts, default_alt: float = 50.0) -> str:
        """將 [(lat,lon)] 或 [(lat,lon,alt)] 序列化為 JSON"""
        result = []
        for p in pts:
            if len(p) >= 3:
                result.append({'lat': p[0], 'lon': p[1], 'alt': p[2]})
            else:
                result.append({'lat': p[0], 'lon': p[1], 'alt': default_alt})
        return json.dumps(result)

    # ─────────────────────────────────────────────────────────────────
    # URL Bridge 回調（由 CesiumPage 呼叫）
    # ─────────────────────────────────────────────────────────────────
    def on_map_clicked(self, lat: float, lon: float):
        if self._strike_marking:
            self.strike_target_added.emit(lat, lon)
            return
        self.corner_added.emit(lat, lon)

    def on_circle_draw_complete(self, lat: float, lon: float, radius: float):
        self.circle_defined.emit(lat, lon, radius)

    def on_nfz_poly_done(self, vertices):
        self.nfz_polygon_drawn.emit(vertices)

    # ─────────────────────────────────────────────────────────────────
    # 狀態同步（由 DualMapWidget 呼叫，切換到 3D 時同步 2D 狀態）
    # ─────────────────────────────────────────────────────────────────
    def sync_state(self, corners, paths, path_colors, path_tooltips,
                   transit_paths, circle_center, circle_radius,
                   home_point, nfz_zones, swarm_data):
        """同步 MapWidget 狀態到 3D 地圖"""
        self.corners       = list(corners)
        self.paths         = list(paths)
        self.path_colors   = list(path_colors)
        self._path_tooltips = list(path_tooltips)
        self.transit_paths  = list(transit_paths)
        self._circle_center = circle_center
        self._circle_radius_m = circle_radius
        self._home_point    = home_point
        self._nfz_zones     = list(nfz_zones)
        self._swarm_data    = swarm_data
        self._refresh_all()

    def _refresh_all(self):
        """重繪 3D 地圖上的所有元素"""
        self._refresh_corners()
        self._refresh_paths()
        self._refresh_overlays()
        self._refresh_nfz()

    def _refresh_corners(self):
        corners_data = [{'lat': lat, 'lon': lon} for lat, lon in self.corners]
        corners_json = json.dumps(corners_data)
        self._js(f'setCorners({json.dumps(corners_json)})')

    def _refresh_paths(self):
        self._js('clearPaths()')
        for idx, path in enumerate(self.paths):
            if len(path) < 2:
                continue
            color = (self.path_colors[idx]
                     if idx < len(self.path_colors) else _REGION_COLORS[0])
            tooltip = (self._path_tooltips[idx]
                       if idx < len(self._path_tooltips) else '')

            # 取得高度（固定翼用 _fw_result；多旋翼用 _last_altitude）
            alt = self._resolve_altitude_for_path(idx, tooltip)
            pts_json = self._pts_to_json(path, default_alt=alt)

            dashed = '轉移' in tooltip or 'transfer' in tooltip.lower()
            label_escaped = json.dumps(tooltip)
            clamp = 'true' if getattr(self, '_clamp_basic', False) else 'false'
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},10,{label_escaped},{str(dashed).lower()},{clamp})')

        # 轉場路徑（虛線）
        for tp in self.transit_paths:
            t_path = tp.get('path', [])
            t_alt  = tp.get('altitude', self._last_altitude)
            t_idx  = tp.get('region_index', 0)
            if len(t_path) < 2:
                continue
            color = _REGION_COLORS[t_idx % len(_REGION_COLORS)]
            pts_json = self._pts_to_json(t_path, default_alt=t_alt)
            label = f'轉場→區域{t_idx+1}'
            self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},7,{json.dumps(label)},true)')

    def _resolve_altitude_for_path(self, idx: int, tooltip: str) -> float:
        """從 fw_result 解析高度（固定翼），否則回傳 _last_altitude"""
        if self._fw_result:
            if '起飛' in tooltip:
                tp = self._fw_result.get('takeoff_path', [])
                if tp and len(tp[0]) >= 3:
                    return tp[0][2]  # 起飛起始高度
            elif '降落' in tooltip or '進場' in tooltip:
                lp = self._fw_result.get('landing_path', [])
                if lp and len(lp[0]) >= 3:
                    return lp[0][2]
            else:
                mp = self._fw_result.get('mission_path', [])
                if mp and len(mp[0]) >= 3:
                    return mp[0][2]
        return self._last_altitude

    def _refresh_overlays(self):
        if self._circle_center:
            lat, lon = self._circle_center
            self._js(f'setCircleOverlay({lat},{lon},{self._circle_radius_m},"#2196F3")')
        if self._home_point:
            h_lat, h_lon = self._home_point
            self._js(f'setHomePoint({h_lat},{h_lon})')

    def _refresh_nfz(self):
        self._js('clearNFZ()')
        for nfz in self._nfz_zones:
            name = json.dumps(nfz.get('name', 'NFZ'))
            if nfz['type'] == 'polygon':
                verts = [{'lat': la, 'lon': lo} for la, lo in nfz['vertices']]
                self._js(f'addNFZPolygon({json.dumps(json.dumps(verts))},{name})')
            elif nfz['type'] == 'circle':
                clat, clon = nfz['center']
                r = nfz['radius']
                self._js(f'addNFZCircle({clat},{clon},{r},{name})')

    # ─────────────────────────────────────────────────────────────────
    # 固定翼 3D 路徑（高度完整資訊）
    # ─────────────────────────────────────────────────────────────────
    def set_fw_result(self, result: dict):
        """接收 fw_mission_result，用於提取 (lat,lon,alt) 3D 路徑"""
        self._fw_result = result
        self._render_fw_3d(result)

    def _render_fw_3d(self, result: dict):
        """在 3D 地圖上用完整高度資訊繪製固定翼三階段路徑

        為避免段與段之間出現視覺斷層：
        - 在 takeoff 末端附加 mission 起點
        - 在 mission 末端附加 landing 起點
        """
        self._js('clearPaths()')

        takeoff = list(result.get('takeoff_path', []) or [])
        mission = list(result.get('mission_path', []) or [])
        landing = list(result.get('landing_path', []) or [])

        # 銜接：takeoff → mission
        if takeoff and mission:
            takeoff = takeoff + [mission[0]]
        # 銜接：mission → landing
        if mission and landing:
            mission = mission + [landing[0]]

        # 起飛段（金黃）
        if len(takeoff) >= 2:
            pts_json = self._pts_to_json(takeoff, default_alt=50.0)
            self._js(f'addPath({json.dumps(pts_json)},"#FFD700",12,"起飛爬升",false)')

        # 任務段（藍）
        if len(mission) >= 2:
            pts_json = self._pts_to_json(mission, default_alt=100.0)
            self._js(f'addPath({json.dumps(pts_json)},"#3D87FF",10,"任務掃描",false)')

        # 降落段（橙）
        if len(landing) >= 2:
            pts_json = self._pts_to_json(landing, default_alt=80.0)
            self._js(f'addPath({json.dumps(pts_json)},"#FF6B35",10,"五邊進場",false)')

        # 飛向路徑
        self._js('flyToScene()')

    # ─────────────────────────────────────────────────────────────────
    # 與 MapWidget 相容的公開方法
    # ─────────────────────────────────────────────────────────────────
    def display_path(self, path: List[Tuple], altitude: float = 50.0):
        self._last_altitude = altitude
        self.paths = [path]
        self.path_colors = [_REGION_COLORS[0]]
        self._path_tooltips = []
        self._fw_result = None
        self._clamp_basic = True
        self._refresh_paths()
        self._clamp_basic = False
        self._js('flyToScene()')

    def display_paths(self, paths_list: List[List[Tuple]], altitude: float = 50.0):
        self._last_altitude = altitude
        self.paths = [p for p in paths_list if p and len(p) >= 2]
        self.path_colors = [_REGION_COLORS[i % len(_REGION_COLORS)]
                            for i in range(len(self.paths))]
        self._path_tooltips = [f'掃描區域 {i+1}' for i in range(len(self.paths))]
        self._fw_result = None
        self._clamp_basic = True
        self._refresh_paths()
        self._clamp_basic = False
        self._js('flyToScene()')

    def display_fw_paths(self,
                         takeoff: List[Tuple],
                         mission,
                         landing: List[Tuple]):
        """固定翼三階段（2D 座標版；高度用 _fw_result 補充，若無則用常數）"""
        # 正規化 mission
        if not mission:
            mission_paths = []
        elif mission and isinstance(mission[0], tuple):
            mission_paths = [mission]
        else:
            mission_paths = [m for m in mission if m and len(m) >= 2]

        _SCAN_COLORS = ['#3D87FF', '#08EC91', '#FF69B4', '#9B59B6', '#00BCD4', '#FF4500']

        self.paths = []
        self.path_colors = []
        self._path_tooltips = []

        if takeoff and len(takeoff) >= 2:
            self.paths.append(takeoff)
            self.path_colors.append('#FFD700')
            self._path_tooltips.append('起飛爬升路徑')

        for i, mp in enumerate(mission_paths):
            label = f'掃描區域 {i+1}' if len(mission_paths) > 1 else '任務掃描路徑'
            self.paths.append(mp)
            self.path_colors.append(_SCAN_COLORS[i % len(_SCAN_COLORS)])
            self._path_tooltips.append(label)

        if landing and len(landing) >= 2:
            self.paths.append(landing)
            self.path_colors.append('#FF6B35')
            self._path_tooltips.append('五邊進場路徑')

        # 如果已有 fw_result，用高度完整版渲染
        if self._fw_result:
            self._render_fw_3d(self._fw_result)
        else:
            self._refresh_paths()
            self._js('flyToScene()')

    def display_nfz_zones(self, nfz_zones: list):
        self._nfz_zones = list(nfz_zones) if nfz_zones else []
        self._refresh_nfz()

    def draw_circle_overlay(self, center_lat: float, center_lon: float,
                             radius_m: float, color: str = '#2196F3'):
        self._circle_center   = (center_lat, center_lon)
        self._circle_radius_m = radius_m
        self._js(f'setCircleOverlay({center_lat},{center_lon},{radius_m},{json.dumps(color)})')

    def clear_circle_overlay(self):
        self._circle_center   = None
        self._circle_radius_m = 0.0
        self._js('if(circleEntity){viewer.entities.remove(circleEntity);circleEntity=null;}')

    def set_home_point_overlay(self, lat: float, lon: float):
        self._home_point = (lat, lon)
        self._js(f'setHomePoint({lat},{lon})')

    def clear_home_point_overlay(self):
        self._home_point = None
        self._js('if(homeEntity){viewer.entities.remove(homeEntity);homeEntity=null;}')

    def display_swarm_coverage(self, swarm_mission, coverage_paths=None):
        """群飛路徑顯示（3D 版）"""
        try:
            self._js('clearPaths()')
            for drone in swarm_mission.drones:
                drone_id = drone.drone_id
                color    = _DRONE_COLORS[(drone_id - 1) % len(_DRONE_COLORS)]
                if not drone.mission or not drone.mission.waypoints:
                    continue
                nav_wps = [wp for wp in drone.mission.waypoints.waypoints if wp.command == 16]
                if len(nav_wps) < 2:
                    continue
                pts = [{'lat': wp.lat, 'lon': wp.lon, 'alt': getattr(wp, 'alt', 50.0)}
                       for wp in nav_wps]
                pts_json = json.dumps(pts)
                label    = f'Drone {drone_id}'
                self._js(f'addPath({json.dumps(pts_json)},{json.dumps(color)},6,{json.dumps(label)},false)')
            self._js('flyToScene()')
        except Exception as e:
            logger.error(f'Cesium 群飛顯示失敗: {e}')

    def display_swarm_raw(self, swarm_data: dict):
        """群飛原始資料顯示（3D 版）— 支援 DCCPP 五階段分色：
        TAKEOFF（綠）/ ENTRY（青）/ OPERATION（無人機主色）/
        TRANSFER（主色長虛線）/ LANDING（橘 dash-dot）。
        點可為 (lat, lon) 或 (lat, lon, alt)。
        """
        try:
            self._js('clearPaths()')
            drones = swarm_data.get('drones', [])

            def _to_pts(seg, default_alt):
                return [{'lat': p[0], 'lon': p[1],
                         'alt': (p[2] if len(p) > 2 else default_alt)}
                        for p in seg]

            def _add(seg, color, width, label, dashed, default_alt=80.0):
                if not seg or len(seg) < 2:
                    return
                pts = _to_pts(seg, default_alt)
                self._js(
                    f'addPath({json.dumps(json.dumps(pts))},'
                    f'{json.dumps(color)},{width},{json.dumps(label)},'
                    f'{"true" if dashed else "false"})'
                )

            for drone_info in drones:
                drone_id = drone_info.get('drone_id', 1)
                color    = _DRONE_COLORS[(drone_id - 1) % len(_DRONE_COLORS)]

                # TAKEOFF — 金黃實線
                for seg in drone_info.get('takeoff_paths', []) or []:
                    _add(seg, '#FFD700', 8, f'D{drone_id} 起飛', False, 30.0)

                # ENTRY — 青色實線
                for seg in drone_info.get('entry_paths', []) or []:
                    _add(seg, '#26C6DA', 6, f'D{drone_id} 進入', False)

                # OPERATION — 無人機主色，最粗實線
                for seg in drone_info.get('operation_paths', []) or []:
                    _add(seg, color, 8, f'D{drone_id} 作業', False)

                # TRANSFER — 主色虛線
                for seg in drone_info.get('transfer_paths', []) or []:
                    _add(seg, color, 5, f'D{drone_id} 轉移', True)

                # LANDING — 橘色實線
                for seg in drone_info.get('landing_paths', []) or []:
                    _add(seg, '#FF6B35', 7, f'D{drone_id} 降落', False, 40.0)

            self._js('flyToScene()')
        except Exception as e:
            logger.error(f'Cesium 群飛原始顯示失敗: {e}')

    def display_transit_paths(self, transit_paths: list):
        self.transit_paths = list(transit_paths)
        self._refresh_paths()

    def clear_transit_paths(self):
        self.transit_paths = []
        self._refresh_paths()

    def set_edit_mode(self, enabled: bool):
        """編輯模式（3D 地圖無需特別處理）"""
        pass

    def add_corner(self, lat: float, lon: float):
        if len(self.corners) < 100:
            self.corners.append((lat, lon))
            self._refresh_corners()

    def move_corner(self, index: int, lat: float, lon: float):
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            self._refresh_corners()

    def clear_all(self):
        self.corners.clear()
        self.paths.clear()
        self.path_colors.clear()
        self._path_tooltips.clear()
        self.transit_paths.clear()
        self._circle_center   = None
        self._circle_radius_m = 0.0
        self._home_point      = None
        self._nfz_zones.clear()
        self._swarm_data      = None
        self._fw_result       = None
        self._js('clearCorners(); clearPaths(); clearNFZ();')
        self._js('if(circleEntity){viewer.entities.remove(circleEntity);circleEntity=null;}')
        self._js('if(homeEntity){viewer.entities.remove(homeEntity);homeEntity=null;}')
        self._js('clearDemOverlay()')

    # ─────────────────────────────────────────────────────────────────
    # DEM 地形視覺化
    # ─────────────────────────────────────────────────────────────────
    def show_dem_overlay(self, dem_manager, alpha: float = 0.65,
                          max_size: int = 256) -> bool:
        """
        將 DEM 資料以高度 heatmap 方式疊加在 3D 地圖上。
        dem_manager: 已載入的 DEMTerrainManager
        """
        try:
            import numpy as np
            import base64
            from io import BytesIO
            data = getattr(dem_manager, '_data', None)
            if data is None or not getattr(dem_manager, '_loaded', False):
                logger.warning('[DEM] 尚未載入任何 DEM 資料')
                return False

            lat_min = float(dem_manager._lat_min)
            lat_max = float(dem_manager._lat_max)
            lon_min = float(dem_manager._lon_min)
            lon_max = float(dem_manager._lon_max)

            # 過濾 nodata
            arr = np.array(data, dtype=np.float64)
            nodata = float(getattr(dem_manager, '_nodata', -9999.0))
            mask = (arr > nodata + 1) & np.isfinite(arr)
            if not mask.any():
                logger.warning('[DEM] 全為 nodata')
                return False
            min_e = float(arr[mask].min())
            max_e = float(arr[mask].max())
            rng = max(max_e - min_e, 1.0)

            # 下採樣
            h, w = arr.shape
            step_r = max(1, h // max_size)
            step_c = max(1, w // max_size)
            sub = arr[::step_r, ::step_c]
            sub_mask = (sub > nodata + 1) & np.isfinite(sub)

            # 正規化 0~1 → 彩虹色帶
            norm = np.clip((sub - min_e) / rng, 0.0, 1.0)
            rgba = np.zeros((*sub.shape, 4), dtype=np.uint8)
            # 色帶：藍→青→綠→黃→紅
            def ramp(t):
                if t < 0.25:
                    k = t / 0.25;   return (0, int(255 * k), 255)
                if t < 0.5:
                    k = (t - 0.25) / 0.25; return (0, 255, int(255 * (1 - k)))
                if t < 0.75:
                    k = (t - 0.5) / 0.25; return (int(255 * k), 255, 0)
                k = (t - 0.75) / 0.25; return (255, int(255 * (1 - k)), 0)
            it = np.nditer(norm, flags=['multi_index'])
            while not it.finished:
                r, g, b = ramp(float(it[0]))
                i, j = it.multi_index
                rgba[i, j] = (r, g, b, 230 if sub_mask[i, j] else 0)
                it.iternext()

            # 轉 PNG base64
            try:
                from PIL import Image
                img = Image.fromarray(rgba, 'RGBA')
                buf = BytesIO()
                img.save(buf, format='PNG')
                png_bytes = buf.getvalue()
            except ImportError:
                # 退而求其次：用 numpy 自行寫 PNG — 需 PIL 才可行
                logger.error('[DEM] 需要 Pillow 才能生成 heatmap: pip install pillow')
                return False

            b64 = base64.b64encode(png_bytes).decode('ascii')
            data_url = f'data:image/png;base64,{b64}'

            self._js(
                f'showDemOverlay({lat_min},{lat_max},{lon_min},{lon_max},'
                f'{json.dumps(data_url)},{min_e},{max_e},{alpha})'
            )
            logger.info(
                f'[DEM] 疊加完成: {sub.shape[1]}x{sub.shape[0]} 點, '
                f'範圍 {min_e:.1f}~{max_e:.1f}m'
            )
            return True
        except Exception as e:
            logger.error(f'[DEM] 疊加失敗: {e}', exc_info=True)
            return False

    def clear_dem_overlay(self):
        self._js('clearDemOverlay()')

    def show_dem_terrain_3d(self, dem_manager, max_size: int = 256) -> bool:
        """把 DEM 以平滑 3D 地形顯示 (Cesium CustomHeightmapTerrainProvider)。

        整個地球地形會被替換成 DEM 資料，用雙線性插值採樣。
        max_size: 下採樣後每邊最大解析度（256×256 ≈ 夠平滑）
        """
        try:
            import numpy as np
            data = getattr(dem_manager, '_data', None)
            if data is None or not getattr(dem_manager, '_loaded', False):
                logger.warning('[DEM3D] 尚未載入任何 DEM 資料')
                return False

            lat_min = float(dem_manager._lat_min)
            lat_max = float(dem_manager._lat_max)
            lon_min = float(dem_manager._lon_min)
            lon_max = float(dem_manager._lon_max)

            arr = np.array(data, dtype=np.float32)
            nodata = float(getattr(dem_manager, '_nodata', -9999.0))
            arr = np.where((arr > nodata + 1) & np.isfinite(arr), arr, 0.0)

            # 下採樣
            h, w = arr.shape
            step_r = max(1, h // max_size)
            step_c = max(1, w // max_size)
            sub = arr[::step_r, ::step_c]
            sh, sw = sub.shape

            flat = sub.flatten().tolist()

            self._js(
                f'showDemTerrainHeightmap({lat_min},{lat_max},{lon_min},{lon_max},'
                f'{sh},{sw},{json.dumps(flat)})'
            )
            logger.info(
                f'[DEM3D] CustomHeightmapTerrainProvider 已載入 {sh}x{sw} 解析度'
            )
            return True
        except Exception as e:
            logger.error(f'[DEM3D] 建立 3D 地形失敗: {e}', exc_info=True)
            return False

    def clear_dem_terrain_3d(self):
        self._js('clearDemTerrain()')

    # ─────────────────────────────────────────────────────────────────
    # SITL 接口（公開方法，供外部 MAVLink 執行緒呼叫）
    # ─────────────────────────────────────────────────────────────────
    def update_uav_position(self, lat: float, lon: float, alt: float,
                             heading_deg: float = 0.0, speed_ms: float = 0.0,
                             sysid: int = 1, mode: str = '', armed: bool = False,
                             vehicle_type: str = '',
                             pitch_deg: float = 0.0, roll_deg: float = 0.0):
        """更新 UAV 即時位置（多機支援，含姿態同步）"""
        mode_js = json.dumps(mode or '')
        vt_js = json.dumps(vehicle_type or '')
        armed_js = 'true' if armed else 'false'
        self._js(
            f'updateUAVPosition({lat},{lon},{alt},{heading_deg},{speed_ms},'
            f'{sysid},{mode_js},{armed_js},{vt_js},{pitch_deg},{roll_deg})'
        )

    def clear_uav(self, sysid: int = None):
        """清除 UAV 位置標記（不指定 sysid 則清全部）"""
        if sysid is None:
            self._js('clearUAV()')
        else:
            self._js(f'clearUAV({sysid})')

    def fly_to_position(self, lat: float, lon: float,
                         alt: float = 0.0, range_m: float = 600.0):
        """相機飛向指定位置"""
        self._js(f'flyToPos({lat},{lon},{alt},{range_m})')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組一：FSDM 高程切片分析
    # ─────────────────────────────────────────────────────────────────
    def update_elevation_slicer(self, min_alt: float, max_alt: float):
        """更新高程切片視覺化（FSDM 地形盲區走廊）"""
        self._js(f'updateElevationSlicer({min_alt},{max_alt})')

    def clear_elevation_slicer(self):
        """清除高程切片"""
        self._js('clearElevationSlicer()')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組二：FOV 光錐 + SAR 搜救機率熱力圖
    # ─────────────────────────────────────────────────────────────────
    def update_fov_cone(self, lat: float, lon: float, alt: float,
                        fov_radius: float = 50.0,
                        heading_deg: float = 0.0,
                        pitch_deg: float = 0.0,
                        roll_deg: float = 0.0):
        """更新 FOV 光錐位置與姿態"""
        self._js(
            f'updateFOVCone({lat},{lon},{alt},{fov_radius},'
            f'{heading_deg},{pitch_deg},{roll_deg})'
        )

    def clear_fov_cone(self):
        """清除 FOV 光錐"""
        self._js('clearFOVCone()')

    def init_sar_heatmap(self, lat_min: float, lat_max: float,
                         lon_min: float, lon_max: float,
                         rows: int = 20, cols: int = 20,
                         sweep_width: float = 50.0,
                         quality: float = 0.8):
        """初始化搜救機率熱力圖"""
        self._js(
            f'initSARHeatmap({lat_min},{lat_max},{lon_min},{lon_max},'
            f'{rows},{cols},{sweep_width},{quality})'
        )

    def update_heatmap(self, uav_lat: float, uav_lon: float,
                       fov_radius: float = 50.0):
        """更新搜救熱力圖（UAV 光錐掃過時更新 COS）"""
        self._js(f'updateHeatmap({uav_lat},{uav_lon},{fov_radius})')

    def clear_sar_heatmap(self):
        """清除搜救熱力圖"""
        self._js('clearSARHeatmap()')

    def reset_sar_heatmap(self):
        """重置搜救熱力圖 COS"""
        self._js('resetSARHeatmap()')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組三：3D 雷達威脅穹頂 + RCS 敏感度
    # ─────────────────────────────────────────────────────────────────
    def add_radar_dome(self, lat: float, lon: float, alt: float = 0.0,
                       radius: float = 5000.0, name: str = ''):
        """新增雷達威脅穹頂"""
        name_js = json.dumps(name or '')
        self._js(f'addRadarDome({lat},{lon},{alt},{radius},{name_js})')

    def clear_radar_domes(self):
        """清除所有雷達穹頂"""
        self._js('clearRadarDomes()')

    def update_rcs_sensitivity(self, uav_lat: float, uav_lon: float,
                               uav_alt: float, uav_heading: float = 0.0,
                               sysid: int = 1):
        """更新 UAV RCS 敏感度渲染"""
        self._js(
            f'updateRCSSensitivity({uav_lat},{uav_lon},{uav_alt},'
            f'{uav_heading},{sysid})'
        )

    def clear_rcs_sensitivity(self, sysid: int = 1):
        """清除 RCS 渲染"""
        self._js(f'clearRCSSensitivity({sysid})')

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組四：蜂群打擊 (Swarm Strike) 前端接口
    # ─────────────────────────────────────────────────────────────────
    def strike_set_marking_mode(self, enabled: bool):
        """切換打擊目標標記模式"""
        self._strike_marking = enabled
        self._js(f'strikeSetMarkingMode({"true" if enabled else "false"})')

    def strike_add_target(self, lat: float, lon: float, idx: int):
        """在 3D 地圖上新增打擊目標標記"""
        self._js(f'strikeAddTarget({lat},{lon},{idx})')

    def strike_execute_animation(self, data_json: str, anim_speed: float = 3.0):
        """執行蜂群打擊動畫"""
        self._js(f'strikeExecuteAnimation({json.dumps(data_json)},{anim_speed})')

    def strike_clear_all(self):
        """清除所有打擊視覺化"""
        self._strike_marking = False
        self._js('strikeClearAll()')

    # ── VTOL 3D 軌跡視覺化 ────────────────────────────────────────
    def draw_vtol_swarm_paths(self, uav_data_list: list):
        """繪製 VTOL 群飛三階段 3D 軌跡（垂直黃線+巡航虛線+轉換點閃爍）

        Parameters
        ----------
        uav_data_list : list[dict]
            VTOLMissionExporter.to_cesium_data() 回傳的資料列表。
        """
        import json as _json
        self._js(f'drawVTOLSwarmPaths({_json.dumps(uav_data_list)})')

    def vtol_clear_all(self):
        """清除所有 VTOL 軌跡視覺化"""
        self._js('vtolClearAll()')

    def animate_radar_scan(self, radar_idx: int = 0, duration_ms: int = 2000):
        """播放雷達掃描動畫"""
        self._js(f'animateRadarScan({radar_idx},{duration_ms})')

    # ── Terminal Dive 末端俯衝視覺化 ──────────────────────────
    def render_terminal_dive(self,
                             uav_id: int,
                             trajectory: list,
                             target_lat: float,
                             target_lon: float,
                             target_alt: float = 0.0,
                             dive_threshold: float = 500.0):
        """渲染末端俯衝全套視覺化（俯衝姿態 + 軌跡變色 + 鎖定射線 + 爆炸）

        Parameters
        ----------
        uav_id : int
            UAV 識別碼
        trajectory : list
            航跡資料，每筆格式:
              { 'time': 'ISO8601', 'lat': float, 'lon': float, 'alt': float }
            或 tuple: (iso_time_str, lat, lon, alt)
        target_lat, target_lon, target_alt : float
            地面目標座標（WGS84 度、公尺）
        dive_threshold : float
            俯衝觸發距離門檻（公尺），預設 500m
        """
        # 將 trajectory 正規化為 JS 可用的 JSON 陣列
        traj_data = []
        for pt in trajectory:
            if isinstance(pt, dict):
                traj_data.append({
                    'time': pt['time'],
                    'lat':  float(pt['lat']),
                    'lon':  float(pt['lon']),
                    'alt':  float(pt.get('alt', 0)),
                })
            elif isinstance(pt, (list, tuple)) and len(pt) >= 4:
                traj_data.append({
                    'time': str(pt[0]),
                    'lat':  float(pt[1]),
                    'lon':  float(pt[2]),
                    'alt':  float(pt[3]),
                })

        traj_json = json.dumps(traj_data, ensure_ascii=False)
        target_json = json.dumps({
            'lat': target_lat, 'lon': target_lon, 'alt': target_alt
        })

        self._js(
            f'renderTerminalDive({uav_id}, {traj_json}, {target_json}, {dive_threshold})'
        )
        logger.info(
            f'[TerminalDive] UAV-{uav_id}: {len(traj_data)} 航點, '
            f'目標=({target_lat:.5f},{target_lon:.5f}), 俯衝門檻={dive_threshold}m'
        )

    def clear_terminal_dive(self, uav_id: int):
        """清除指定 UAV 的末端俯衝視覺化"""
        self._js(f'clearTerminalDive({uav_id})')

    def clear_all_terminal_dives(self):
        """清除所有末端俯衝視覺化"""
        self._js('clearAllTerminalDives()')
