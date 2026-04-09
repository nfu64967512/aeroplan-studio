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
<title>3D UAV Path Planner</title>
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
</style>
</head>
<body>
<div id="cesiumContainer"></div>

<div id="modeLabel">🌐 3D 視圖</div>

<div id="toolbar">
  <button onclick="flyToScene()">🎯 飛向路徑</button>
  <button onclick="toggleClamp()">🏔️ 地形吸附</button>
  <button onclick="setMode3D()">🌐 3D</button>
  <button onclick="setMode2D()">🗺️ 2D</button>
  <button onclick="setModeColumbus()">📐 哥倫布</button>
  <button onclick="clearUAV()">✖ 清除 UAV</button>
</div>

<div id="sitlStatus"></div>
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

function updateUAVPosition(lat, lon, alt, headingDeg, speedMs, sysid, mode, armed, vehicleType) {
  sysid = sysid || 1;
  const pos = Cesium.Cartesian3.fromDegrees(lon, lat, alt || 0);
  const color = _UAV_COLORS[(sysid - 1) % _UAV_COLORS.length];
  _appendTrail(sysid, lat, lon, alt, color);
  const isPlane = (vehicleType || '').toUpperCase().indexOf('PLANE') >= 0
                  || (vehicleType || '').toUpperCase().indexOf('FIXED') >= 0;
  const icon = isPlane ? '✈' : '🚁';
  const labelTxt = icon + ' UAV' + sysid + (mode ? ' [' + mode + ']' : '') + (armed ? ' ●ARM' : '');

  // 計算 orientation（用於 3D 立體模型旋轉）
  const hpr = new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians(headingDeg || 0), 0, 0
  );
  const orientation = Cesium.Transforms.headingPitchRollQuaternion(pos, hpr);

  let entity = uavEntities[sysid];
  if (!entity) {
    entity = viewer.entities.add({
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
        // 距離縮放：近距離用立體模型，遠處用 billboard
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(800.0, 1.0e8),
      },
      // 立體幾何（近距離顯示）：固定翼用機身+機翼，多旋翼用十字+四螺旋
      ellipsoid: isPlane ? undefined : {
        radii: new Cesium.Cartesian3(8, 8, 2),
        material: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
        outline: true, outlineColor: Cesium.Color.BLACK,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 800.0),
      },
      box: isPlane ? {
        // 機身：長 12m × 寬 1.5m × 高 1.2m；機翼透過 plane 額外加（簡化合成）
        dimensions: new Cesium.Cartesian3(2.0, 12.0, 1.2),
        material: Cesium.Color.fromCssColorString(color).withAlpha(0.95),
        outline: true, outlineColor: Cesium.Color.BLACK,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 800.0),
      } : undefined,
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
    });
    uavEntities[sysid] = entity;
    if (sysid === 1) uavEntity = entity;  // 舊 API 相容

    // 為固定翼加上機翼（兩個側向長條）
    if (isPlane) {
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

  // 狀態列：顯示主控（sysid=1）的資訊
  if (sysid === 1) {
    const el = document.getElementById('sitlStatus');
    el.style.display = 'block';
    el.textContent =
      '✈ SITL[' + sysid + ']  ' + (mode || '') + (armed ? ' ARM' : ' DIS') +
      '  lat:' + lat.toFixed(5) + '  lon:' + lon.toFixed(5) +
      '  alt:' + (alt||0).toFixed(1) + 'm  hdg:' + (headingDeg||0).toFixed(0) + '°  ' +
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

console.log('[Cesium 3D] 初始化完成');
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

        self._init_ui()
        self._load_cesium()
        logger.info('Cesium 3D 地圖組件初始化完成')

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

        html = (_CESIUM_HTML
                .replace('__CESIUM_JS__',   cesium_js)
                .replace('__CESIUM_CSS__',  cesium_css)
                .replace('__CESIUM_TOKEN__', self.CESIUM_TOKEN)
                .replace('__INIT_LAT__',    str(init_lat))
                .replace('__INIT_LON__',    str(init_lon))
                .replace('__INIT_HEIGHT__', str(init_height)))
        self.web_view.setHtml(html, base_url)

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
                             vehicle_type: str = ''):
        """更新 UAV 即時位置（多機支援）"""
        mode_js = json.dumps(mode or '')
        vt_js = json.dumps(vehicle_type or '')
        armed_js = 'true' if armed else 'false'
        self._js(
            f'updateUAVPosition({lat},{lon},{alt},{heading_deg},{speed_ms},'
            f'{sysid},{mode_js},{armed_js},{vt_js})'
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
