/*
 * AeroWind - AeroPlan Studio wind visualisation engine (shared by the 2D Leaflet page and the 3D Cesium page)
 *
 * 純腳本：不 import／export、不依賴外部函式庫、不連網；只定義 window.AeroWind。可用 <script src="file:///..."> 載入，
 * 也可以整檔內嵌（本檔刻意不含結束 script 標籤字串、Jinja 的雙大括號／百分號／井號開頭、__大寫__ 佔位字樣）。
 *
 * 氣象慣例：dir_from_deg = 風「吹來」的方向（自真北順時針）。u = 空氣往東的分量、v = 往北的分量（m/s）：
 *   u = -s·sin(dir)、v = -s·cos(dir)；風從 270°（西風）⇒ 往東吹 ⇒ u = +s、v = 0。
 *
 * AeroWind.create(host, adapter) -> 實例
 *   host    : DOM 元素（2D：map.getContainer()；3D：viewer.container）。引擎在 host 裡加「一個」canvas
 *             （position:absolute; 0,0; 100%×100%; pointer-events:none; z-index = options.zIndex，預設 450），
 *             每一幀檢查 host 尺寸／devicePixelRatio，有變就重設像素尺寸。粒子軌跡與箭頭／圖例各自畫在
 *             不掛進 DOM 的離屏 canvas，每幀合成到這一個可見 canvas。
 *   adapter : { project(lat, lon, h_m) -> {x, y}（host 的 CSS 像素）或 null（看不到）,
 *               viewKey() -> 字串（平移／縮放／旋轉就要變） }
 *             透視投影的 adapter 可以多回傳 w（裁切空間的 w，> 0；3D 頁的快速路徑有）：格網四角都有 w 時，
 *             引擎改在齊次座標內插 (x·w, y·w, w) 再相除——平面在透視下的投影對齊次座標是線性的，所以靠近鏡頭、
 *             一格跨幾百公尺時也準（只內插 x、y 的話，追機時 p90 誤差 40–55 px）。沒有 w（2D 頁）⇒ 照舊雙線性。
 *             ⚠ 建議畫面外的點照樣回傳座標、只有在地球背面／相機後方才回 null；回 null 也能用
 *               （引擎會加密取樣找可見區），只是放大到很深時找區塊比較費工。
 *
 * 實例方法（id 可省略，省略時用 'default'）：
 *   setField(field) | setField(id, field)
 *       field = {lats:[遞增], lons:[遞增], u:[[...]], v:[[...]], height_m, label, mode:'flow'|'arrows'|'both',
 *                bbox?:[latMin, lonMin, latMax, lonMax], particles?:int, group?:string, groupLabel?:string}
 *       u[i][j] 對應 lats[i]、lons[j]；null／NaN 格點略過。1×1 格點 = 均勻風場，範圍取 bbox
 *       （沒給 bbox 時是 ±0.01°）。遞減的 lats／lons 會自動翻轉。沒有任何有效格點 ⇒ 移除該 id、回傳 false。
 *       group：同組的多個風場（例：分層風場 10／80／120 m 各一個 id）在圖例只佔「一個區塊」——
 *       groupLabel（可用換行分多行）＋全組最大風速，不再每層一行。沒給 group 的風場照舊一層一行。
 *       particleShare（0–1）：粒子數＝依可見面積的自動值 × share（同組幾層分一個自動預算，總工作量≈單一風場），
 *         至少 particlesMin（但不超過面積密度上限 max(20, area/25) × share），particles 有給時變成**上限**而不是固定數。
 *       faint：true ⇒ 這層的粒子畫淡、畫細（分層風場 3D 只強調一層，其餘層淡化）。同 id 再 setField 只換樣式／
 *         資料時粒子陣列沿用（不重生）。
 *   clearField(id?)            省略 id = 全部移除
 *   setArrows(rows, opts?) | setArrows(id, rows, opts?)
 *       rows = [[sysid, lat, lon, alt_rel, dir_from_deg, speed_mps, label, color?], ...]
 *          或 [{lat, lon, h_m|alt_rel, dir_from_deg, speed_mps, label?, color?, sysid?}, ...]
 *       每列一支箭頭，指向空氣運動方向（dir_from + 180°），長度 ∝ 風速（夾在 arrowMinPx–arrowMaxPx），
 *       文字 = label + 風速（沒有 label 時是「x.x m/s ddd°」），顏色 = speedColor(風速, limitMps)；opts.label = 圖例名稱。
 *   clearArrows(id?)           省略 id = 全部移除
 *   clear()                    全部移除
 *   setOptions({...})          camelCase（snake_case 別名也收）：
 *       animate(true) limitMps(3.0) pxPerMps(0.6) maxAge(90) fade(0.92) particles(0=自動) opacity(1)
 *       zIndex(450) showLegend(true) legendCorner('top-right') legendMargin([10,10]) lineWidth(1.5)
 *       particleAlpha(0.9) arrowPxPerMps(12) arrowMinPx(18) arrowMaxPx(70) arrowSpacingPx(60) visible(true) seed(null)
 *       trailsFollowView(false)：true ⇒ 視角變時不清軌跡，改用每顆粒子最近 TRAIL_HIST 個經緯度位置以新視角重畫
 *         （3D 追機／旋轉時軌跡跟著場景，見下方「粒子」）；false ⇒ 舊行為（2D 頁用這個）。
 *       不認得的鍵或無效的值：console.warn 一次並忽略（不靜默吃掉）。
 *   setMask(rects)             rects = [[x, y, w, h], ...]（host 的 CSS 像素，最多 32 個）：這些框裡不畫粒子軌跡
 *                              （每幀清掉；例：3D 剖面柱文字底下挖空，粒子不蓋字）。空陣列＝取消
 *   options()  redraw()  interpAt(id, lat, lon) -> {u, v}|null  state()  destroy()
 *
 * 粒子：在 lat/lon 上依內插 (u, v) 平流，但每步長度在螢幕上量：用 adapter 的投影量出局部「公尺／像素」
 * （粒子位置與順風 10 m 外一點），使螢幕速度 = 風速 × pxPerMps（與縮放無關）。投影只在 viewKey 變（或資料變）
 * 時重算：引擎把可見區的經緯度格網投影一次，之後每幀用這張格網雙線性換算（adapter 有給 w 時在齊次座標內插；
 * 格網碰到 null 的邊緣格才逐點呼叫 adapter.project）。離開範圍／看不到／超過 maxAge 的粒子重生（初始年齡隨機）。
 * 視角不變的幀：軌跡畫布乘上 fade 再疊上這一步的線段（累積畫布）。視角一變：
 *   trailsFollowView=false ⇒ 軌跡全清、那一幀不畫（舊行為；2D 頁）。
 *   trailsFollowView=true  ⇒ 清掉畫布後，把每顆粒子最近 TRAIL_HIST 個經緯度位置（每幀記一個）用新視角投影、
 *     每隔 TRAIL_STRIDE 點連成折線重畫，透明度 = particleAlpha × fade^年齡（跟累積畫布的拖尾同一條淡出曲線）。
 *     優先用 WebGL（每段一個四邊形、逐頂點透明度；Canvas 2D 描寬線的成本跟總線長成正比，最大化追機會掉到 13 FPS），
 *     拿不到 WebGL 才用 Canvas 2D（透明度分 TRAIL_BANDS 段）。畫好的拖尾放進軌跡畫布，之後視角沒變的幀照常淡出、疊新段。
 *     追機時鏡頭每幀都在動，累積畫布每幀都被清 ⇒ 只剩點；這樣軌跡錨在世界座標、跟著場景走。
 *     單顆粒子上一段的螢幕長度跟記錄時差超過 TRAIL_JUMP 倍（拉近拉遠、瞬間換視角）⇒ 只丟那顆的歷史，不畫被拉長的舊線。
 * animate=false：不畫粒子，風場一律以箭頭格網顯示，只在 viewKey 或資料變時重畫。
 *
 * AeroWind.speedColor(speed, limit)：綠 < 0.7·limit、琥珀 < limit、紅 ≥ limit（limit 無效時用 3.0）。
 */
(function (root) {
  'use strict';

  if (root.AeroWind && root.AeroWind.VERSION) {
    return;
  }

  var VERSION = '1.0.0';
  var D2R = Math.PI / 180;
  var R2D = 180 / Math.PI;
  var M_PER_DEG = 111320;          // 只用來造 ~10 m 的探針點；步長本身在螢幕上量
  var PROBE_M = 10;
  var DEFAULT_ID = 'default';
  var DEFAULT_LIMIT = 3.0;
  var CAUTION_FRAC = 0.7;
  var UNIFORM_HALF_DEG = 0.01;     // 1×1 風場沒給 bbox 時的半寬
  var EDGE_PX = 10;                // 粒子允許超出畫面的邊界
  var LATTICE_PX = 48;             // 投影格網目標格寬（螢幕像素）
  var MAX_PARTICLES = 20000;
  var FAINT_ALPHA = 0.3;           // faint 風場：粒子不透明度 × 0.3、線寬 × 0.7
  var FAINT_WIDTH = 0.7;
  var MAX_MASK_RECTS = 32;
  var DENSE_THROTTLE_MS = 400;
  var TRAIL_HIST = 25;             // trailsFollowView：每顆粒子記的位置數（含現在，≈ 24 幀的拖尾）
  var TRAIL_STRIDE = 4;            // 重畫時每隔幾個位置取一點（風場平滑，6 段折線看不出差別；投影次數是逐點的 1/4）
  var TRAIL_BANDS = 4;             // 透明度分段：每段 TRAIL_BAND_FRAMES 幀，透明度 = particleAlpha × fade^段首年齡
  var TRAIL_BAND_FRAMES = (TRAIL_HIST - 1) / TRAIL_BANDS;   // 6：年齡 0–5、6–11、12–17、18–23 各一段（各 3 條線段）
  var TRAIL_JUMP = 1.6;            // 上一段螢幕長度 / 記錄時長度超過這個倍數（或低於倒數）⇒ 丟掉那顆粒子的歷史
  var HAS_PATH2D = typeof root.Path2D === 'function';
  var GL_FLOATS = 8;               // WebGL 拖尾每個頂點：x, y（CSS px）、side（−1／+1 橫跨線寬）、半線寬（裝置 px）、r, g, b, a
  var GL_CHUNK = 16383;            // 一次 drawElements 最多幾個四邊形（16 位元索引：4 × 16383 < 65536）
  var COLORS = { ok: '#2fbf71', caution: '#f5a623', warn: '#d03b3b', unknown: '#9e9e9e' };
  var CLASS_COLORS = [COLORS.ok, COLORS.caution, COLORS.warn];
  var CLASS_RGB = [[47 / 255, 191 / 255, 113 / 255], [245 / 255, 166 / 255, 35 / 255], [208 / 255, 59 / 255, 59 / 255]];
  var TEXT_COLOR = '#e8eef5';
  var WARN_TEXT = '#ff8a80';
  var FONT = '12px "Microsoft JhengHei", "PingFang TC", "Noto Sans TC", "Noto Sans CJK TC", sans-serif';
  var LEGEND_CORNERS = ['top-right', 'top-left', 'bottom-left', 'bottom-right'];
  var INVALID = { invalid: true };
  var PREPARED = (typeof WeakSet === 'function') ? new WeakSet() : null;

  // ───────────────────────────── 小工具 ─────────────────────────────
  function num(x) {
    if (typeof x === 'number') { return x; }
    if (typeof x === 'string' && x.trim() !== '') { return Number(x); }
    return NaN;
  }
  function clamp(x, a, b) { return x < a ? a : (x > b ? b : x); }
  function clean0(x) { return Math.abs(x) < 1e-12 ? 0 : x; }
  function now() {
    return (root.performance && typeof root.performance.now === 'function') ? root.performance.now() : Date.now();
  }
  function fmt1(x) { return (Math.round(x * 10) / 10).toFixed(1); }
  function niceCeil(x) { return Math.ceil(x * 2 - 1e-9) / 2; }
  function isRowLike(x) { return x !== null && typeof x === 'object' && typeof x.length === 'number'; }
  function mulberry32(seed) {
    var a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function upperBound(cum, r) {           // 第一個 cum[i] > r 的 i
    var lo = 0, hi = cum.length - 1;
    while (lo < hi) {
      var mid = (lo + hi) >> 1;
      if (cum[mid] > r) { hi = mid; } else { lo = mid + 1; }
    }
    return lo;
  }
  function bsearch(a, x) {                // 最大的 i ∈ [0, n-2] 使 a[i] <= x
    var lo = 0, hi = a.length - 2;
    while (lo < hi) {
      var mid = (lo + hi + 1) >> 1;
      if (a[mid] <= x) { lo = mid; } else { hi = mid - 1; }
    }
    return lo < 0 ? 0 : lo;
  }

  // ───────────────────────────── 顏色 ─────────────────────────────
  function speedClass(s, lim) {
    if (s >= lim) { return 2; }
    if (s >= CAUTION_FRAC * lim) { return 1; }
    return 0;
  }
  function speedColor(speed, limit) {
    var lim = num(limit);
    if (!(lim > 0 && isFinite(lim))) { lim = DEFAULT_LIMIT; }
    var s = Math.abs(num(speed));
    if (!isFinite(s)) { return COLORS.unknown; }
    return CLASS_COLORS[speedClass(s, lim)];
  }

  // ───────────────────────────── 風向換算 ─────────────────────────────
  function dirToUV(dirFromDeg, speedMps) {
    var d = num(dirFromDeg), s = num(speedMps), u = NaN, v = NaN;
    if (isFinite(d) && isFinite(s)) {
      u = clean0(-s * Math.sin(d * D2R));
      v = clean0(-s * Math.cos(d * D2R));
    }
    var r = [u, v];
    r.u = u;
    r.v = v;
    return r;
  }
  function uvToDir(u, v) {              // -> [dir_from_deg, speed]
    u = num(u);
    v = num(v);
    var r;
    if (!isFinite(u) || !isFinite(v)) {
      r = [NaN, NaN];
    } else {
      var s = Math.sqrt(u * u + v * v);
      var d = s > 0 ? Math.atan2(-u, -v) * R2D : 0;
      d = ((d % 360) + 360) % 360;
      r = [clean0(d) === 360 ? 0 : clean0(d), s];
    }
    r.dir = r[0];
    r.speed = r[1];
    return r;
  }

  // ───────────────────────────── 風場整理與內插 ─────────────────────────────
  function toNumList(a) {
    if (a === null || a === undefined) { return []; }
    if (typeof a === 'number' || typeof a === 'string') { return [num(a)]; }
    if (typeof a.length !== 'number') { return []; }
    var out = new Array(a.length);
    for (var i = 0; i < a.length; i++) { out[i] = num(a[i]); }
    return out;
  }
  function monotonic(a) {                 // 1 = 遞增、-1 = 遞減、0 = 都不是
    var asc = true, desc = true;
    for (var i = 1; i < a.length; i++) {
      if (a[i] < a[i - 1]) { asc = false; }
      if (a[i] > a[i - 1]) { desc = false; }
    }
    return asc ? 1 : (desc ? -1 : 0);
  }
  function readGrid(src, ni, nj, flipI, flipJ) {
    var n = ni * nj, out = new Float64Array(n), i, j;
    for (i = 0; i < n; i++) { out[i] = NaN; }
    if (src === null || src === undefined) { return out; }
    if (typeof src === 'number' || typeof src === 'string') {
      var c = num(src);
      for (i = 0; i < n; i++) { out[i] = c; }
      return out;
    }
    if (typeof src.length !== 'number') { return out; }
    var put = function (ii, jj, val) {
      if (ii < ni && jj < nj) {
        out[(flipI ? ni - 1 - ii : ii) * nj + (flipJ ? nj - 1 - jj : jj)] = num(val);
      }
    };
    var rows = false;
    for (i = 0; i < src.length; i++) {
      if (isRowLike(src[i])) { rows = true; break; }
    }
    if (rows) {
      for (i = 0; i < Math.min(src.length, ni); i++) {
        var row = src[i];
        if (isRowLike(row)) {
          for (j = 0; j < Math.min(row.length, nj); j++) { put(i, j, row[j]); }
        } else if (nj === 1) {
          put(i, 0, row);
        }
      }
    } else if (src.length === n) {
      for (i = 0; i < ni; i++) {
        for (j = 0; j < nj; j++) { put(i, j, src[i * nj + j]); }
      }
    }
    return out;
  }
  function parseBBox(b) {
    if (!isRowLike(b) || b.length !== 4) { return null; }
    var q = [num(b[0]), num(b[1]), num(b[2]), num(b[3])];
    for (var i = 0; i < 4; i++) { if (!isFinite(q[i])) { return null; } }
    if (q[0] > q[2] || q[1] > q[3] || q[0] < -90 || q[2] > 90) { return null; }
    return q;
  }
  function prepareField(f) {
    if (!f || typeof f !== 'object') { return null; }
    if (PREPARED && PREPARED.has(f)) { return f; }
    var lats = toNumList(f.lats), lons = toNumList(f.lons), ni = lats.length, nj = lons.length, i;
    if (!ni || !nj) { return null; }
    for (i = 0; i < ni; i++) { if (!isFinite(lats[i])) { return null; } }
    for (i = 0; i < nj; i++) { if (!isFinite(lons[i])) { return null; } }
    var mi = ni > 1 ? monotonic(lats) : 1, mj = nj > 1 ? monotonic(lons) : 1;
    if (!mi || !mj) { return null; }
    var flipI = mi < 0, flipJ = mj < 0;
    var U = readGrid(f.u, ni, nj, flipI, flipJ), V = readGrid(f.v, ni, nj, flipI, flipJ);
    if (flipI) { lats.reverse(); }
    if (flipJ) { lons.reverse(); }
    var bb = parseBBox(f.bbox), latMin, latMax, lonMin, lonMax;
    if (ni > 1) { latMin = lats[0]; latMax = lats[ni - 1]; }
    else if (bb) { latMin = bb[0]; latMax = bb[2]; }
    else { latMin = lats[0] - UNIFORM_HALF_DEG; latMax = lats[0] + UNIFORM_HALF_DEG; }
    if (nj > 1) { lonMin = lons[0]; lonMax = lons[nj - 1]; }
    else if (bb) { lonMin = bb[1]; lonMax = bb[3]; }
    else { lonMin = lons[0] - UNIFORM_HALF_DEG; lonMax = lons[0] + UNIFORM_HALF_DEG; }
    if (!(latMax > latMin) || !(lonMax > lonMin)) { return null; }
    var nValid = 0, maxSpeed = 0;
    for (i = 0; i < U.length; i++) {
      var a = U[i], b = V[i];
      if (isFinite(a) && isFinite(b)) {
        nValid++;
        var s = Math.sqrt(a * a + b * b);
        if (s > maxSpeed) { maxSpeed = s; }
      } else {
        U[i] = NaN;
        V[i] = NaN;
      }
    }
    return {
      lats: Float64Array.from(lats), lons: Float64Array.from(lons), ni: ni, nj: nj, U: U, V: V,
      latMin: latMin, latMax: latMax, lonMin: lonMin, lonMax: lonMax, nValid: nValid, maxSpeed: maxSpeed
    };
  }
  function interpInto(P, lat, lon, out) {
    if (!(lat >= P.latMin && lat <= P.latMax && lon >= P.lonMin && lon <= P.lonMax)) { return false; }
    var i0 = 0, i1 = 0, fi = 0, j0 = 0, j1 = 0, fj = 0, d;
    if (P.ni > 1) {
      i0 = bsearch(P.lats, lat);
      i1 = i0 + 1;
      d = P.lats[i1] - P.lats[i0];
      fi = d > 0 ? clamp((lat - P.lats[i0]) / d, 0, 1) : 0;
    }
    if (P.nj > 1) {
      j0 = bsearch(P.lons, lon);
      j1 = j0 + 1;
      d = P.lons[j1] - P.lons[j0];
      fj = d > 0 ? clamp((lon - P.lons[j0]) / d, 0, 1) : 0;
    }
    var nj = P.nj, U = P.U, V = P.V;
    var a = i0 * nj + j0, b = i0 * nj + j1, c = i1 * nj + j0, e = i1 * nj + j1;
    var u00 = U[a], u01 = U[b], u10 = U[c], u11 = U[e];
    if (u00 !== u00 || u01 !== u01 || u10 !== u10 || u11 !== u11) { return false; }   // 四角有 NaN／null ⇒ 略過
    var w00 = (1 - fi) * (1 - fj), w01 = (1 - fi) * fj, w10 = fi * (1 - fj), w11 = fi * fj;
    out.u = u00 * w00 + u01 * w01 + u10 * w10 + u11 * w11;
    out.v = V[a] * w00 + V[b] * w01 + V[c] * w10 + V[e] * w11;
    return true;
  }
  function interp(field, lat, lon) {
    var P = prepareField(field);
    if (!P || !P.nValid) { return null; }
    var o = { u: 0, v: 0 };
    return interpInto(P, num(lat), num(lon), o) ? o : null;
  }
  function prepare(field) {
    var P = prepareField(field);
    if (P && PREPARED) { PREPARED.add(P); }
    return P;
  }
  function parseArrow(r) {
    var sysid = null, lat, lon, h, dir, s, label, color;
    if (isRowLike(r)) {
      sysid = r[0] === undefined ? null : r[0];
      lat = num(r[1]); lon = num(r[2]); h = num(r[3]); dir = num(r[4]); s = num(r[5]); label = r[6]; color = r[7];
    } else if (r && typeof r === 'object') {
      sysid = r.sysid !== undefined ? r.sysid : (r.id !== undefined ? r.id : null);
      lat = num(r.lat); lon = num(r.lon);
      h = num(r.h_m !== undefined ? r.h_m : (r.alt_rel !== undefined ? r.alt_rel : r.h));
      dir = num(r.dir_from_deg); s = num(r.speed_mps); label = r.label; color = r.color;
    } else {
      return null;
    }
    if (!(isFinite(lat) && lat >= -90 && lat <= 90 && isFinite(lon) && isFinite(dir) && isFinite(s) && s >= 0)) {
      return null;
    }
    return {
      sysid: sysid, lat: lat, lon: lon, h: isFinite(h) ? h : 0, dir: ((dir % 360) + 360) % 360, speed: s,
      name: (label === null || label === undefined || label === '') ? '' : String(label),
      color: (typeof color === 'string' && color) ? color : null
    };
  }
  function arrowText(it) {
    var sp = fmt1(it.speed) + ' m/s';
    if (it.name) { return it.name + ' ' + sp; }
    return sp + ' ' + (Math.round(it.dir) % 360) + '°';
  }

  // ───────────────────────────── 選項 ─────────────────────────────
  var DEFAULTS = {
    animate: true, limitMps: DEFAULT_LIMIT, pxPerMps: 0.6, maxAge: 90, fade: 0.92, particles: 0, opacity: 1,
    zIndex: 450, showLegend: true, legendCorner: 'top-right', legendMargin: [10, 10], lineWidth: 1.5,
    particleAlpha: 0.9, arrowPxPerMps: 12, arrowMinPx: 18, arrowMaxPx: 70, arrowSpacingPx: 60, visible: true, seed: null,
    trailsFollowView: false
  };
  var OPT_ALIASES = {
    limit_mps: 'limitMps', px_per_mps: 'pxPerMps', max_age: 'maxAge', z_index: 'zIndex', show_legend: 'showLegend',
    legend: 'showLegend', legend_corner: 'legendCorner', legend_margin: 'legendMargin', line_width: 'lineWidth',
    particle_alpha: 'particleAlpha', arrow_px_per_mps: 'arrowPxPerMps', arrow_min_px: 'arrowMinPx',
    arrow_max_px: 'arrowMaxPx', arrow_spacing_px: 'arrowSpacingPx', trails_follow_view: 'trailsFollowView'
  };
  function vBool(v) { return (typeof v === 'boolean' || v === 0 || v === 1) ? !!v : INVALID; }
  function vPos(v) { v = num(v); return (isFinite(v) && v > 0) ? v : INVALID; }
  function vUnit(v) { v = num(v); return (isFinite(v) && v >= 0 && v <= 1) ? v : INVALID; }
  var VALIDATORS = {
    animate: vBool, showLegend: vBool, visible: vBool, trailsFollowView: vBool,
    limitMps: vPos, pxPerMps: vPos, lineWidth: vPos, arrowPxPerMps: vPos, arrowMinPx: vPos, arrowMaxPx: vPos,
    arrowSpacingPx: function (v) { v = num(v); return (isFinite(v) && v >= 10) ? v : INVALID; },
    maxAge: function (v) { v = num(v); return (isFinite(v) && v >= 1) ? Math.round(v) : INVALID; },
    fade: function (v) { v = num(v); return (isFinite(v) && v > 0 && v <= 1) ? v : INVALID; },
    particles: function (v) { v = num(v); return (isFinite(v) && v >= 0) ? Math.min(MAX_PARTICLES, Math.round(v)) : INVALID; },
    opacity: vUnit, particleAlpha: vUnit,
    zIndex: function (v) { v = num(v); return isFinite(v) ? Math.round(v) : INVALID; },
    legendCorner: function (v) { return LEGEND_CORNERS.indexOf(v) >= 0 ? v : INVALID; },
    legendMargin: function (v) {
      if (!isRowLike(v) || v.length !== 2) { return INVALID; }
      var a = num(v[0]), b = num(v[1]);
      return (isFinite(a) && isFinite(b) && a >= 0 && b >= 0) ? [a, b] : INVALID;
    },
    seed: function (v) {
      if (v === null || v === undefined) { return null; }
      v = num(v);
      return isFinite(v) ? v : INVALID;
    }
  };
  function copyOpts(o) {
    var r = {};
    for (var k in o) {
      if (Object.prototype.hasOwnProperty.call(o, k)) { r[k] = isRowLike(o[k]) ? Array.prototype.slice.call(o[k]) : o[k]; }
    }
    return r;
  }

  // ───────────────────────────── 繪圖小件 ─────────────────────────────
  function drawArrow(ctx, x, y, ux, uy, L, color) {
    var hl = Math.min(10, L * 0.45), hw = hl * 0.55;
    var tx = x - ux * L / 2, ty = y - uy * L / 2, hx = x + ux * L / 2, hy = y + uy * L / 2;
    var bx = hx - ux * hl, by = hy - uy * hl, px = -uy, py = ux;
    var tri = function () {
      ctx.beginPath();
      ctx.moveTo(hx, hy);
      ctx.lineTo(bx + px * hw, by + py * hw);
      ctx.lineTo(bx - px * hw, by - py * hw);
      ctx.closePath();
    };
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.strokeStyle = 'rgba(0,0,0,0.6)';         // 深色光暈
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(bx, by);
    ctx.stroke();
    tri();
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(tx, ty);
    ctx.lineTo(bx + ux, by + uy);
    ctx.stroke();
    ctx.fillStyle = color;
    tri();
    ctx.fill();
    return [tx, ty, hx, hy];
  }
  function drawCalm(ctx, x, y, color) {
    ctx.lineWidth = 4;
    ctx.strokeStyle = 'rgba(0,0,0,0.6)';
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.stroke();
    ctx.lineWidth = 2;
    ctx.strokeStyle = color;
    ctx.stroke();
  }
  function drawLabel(ctx, text, x, y, align, color) {
    ctx.font = FONT;
    ctx.textAlign = align || 'center';
    ctx.textBaseline = 'top';
    ctx.lineJoin = 'round';
    ctx.lineWidth = 3;
    ctx.strokeStyle = 'rgba(0,0,0,0.75)';
    ctx.strokeText(text, x, y);
    ctx.fillStyle = color || '#ffffff';
    ctx.fillText(text, x, y);
  }
  function fitText(ctx, s, maxW) {
    if (ctx.measureText(s).width <= maxW) { return s; }
    while (s.length > 1 && ctx.measureText(s + '…').width > maxW) { s = s.slice(0, -1); }
    return s + '…';
  }
  function roundRectPath(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.arcTo(x + w, y, x + w, y + r, r);
    ctx.lineTo(x + w, y + h - r);
    ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
    ctx.lineTo(x + r, y + h);
    ctx.arcTo(x, y + h, x, y + h - r, r);
    ctx.lineTo(x, y + r);
    ctx.arcTo(x, y, x + r, y, r);
    ctx.closePath();
  }
  function allocParticles(old, n) {
    var pt = { n: n, lat: new Float64Array(n), lon: new Float64Array(n), age: new Int32Array(n), alive: new Uint8Array(n),
               hist: null };
    if (old) {
      var m = Math.min(old.n, n);
      pt.lat.set(old.lat.subarray(0, m));
      pt.lon.set(old.lon.subarray(0, m));
      pt.age.set(old.age.subarray(0, m));
      pt.alive.set(old.alive.subarray(0, m));
      if (old.hist) {
        allocHistory(pt);
        var H = pt.hist, G = old.hist;
        H.lat.set(G.lat.subarray(0, m * TRAIL_HIST));
        H.lon.set(G.lon.subarray(0, m * TRAIL_HIST));
        H.head.set(G.head.subarray(0, m));
        H.cnt.set(G.cnt.subarray(0, m));
        H.step.set(G.step.subarray(0, m));
        H.cls.set(G.cls.subarray(0, m));
      }
    }
    return pt;
  }
  // trailsFollowView 的位置歷史（環狀）：第 k 顆的第 i 格在 k*TRAIL_HIST + i；head = 最新那格、cnt = 有效格數；
  // step = 最新一步記錄時的螢幕長度（px，判斷視角是否跳太大）；cls = 最新一步的風速等級（0／1／2，拖尾整條用這個顏色）。
  // 只在選項打開時才配置（2D 頁不付這個成本）。
  function allocHistory(pt) {
    var n = pt.n;
    pt.hist = { lat: new Float64Array(n * TRAIL_HIST), lon: new Float64Array(n * TRAIL_HIST), head: new Uint8Array(n),
                cnt: new Uint8Array(n), step: new Float32Array(n), cls: new Uint8Array(n) };
    return pt.hist;
  }
  function histReset(H, k, lat, lon) {
    var b = k * TRAIL_HIST;
    H.head[k] = 0;
    H.cnt[k] = 1;
    H.step[k] = 0;
    H.lat[b] = lat;
    H.lon[b] = lon;
  }
  function histPush(H, k, lat, lon, stepPx, cls) {
    var h = H.head[k] + 1;
    if (h >= TRAIL_HIST) { h = 0; }
    var i = k * TRAIL_HIST + h;
    H.head[k] = h;
    if (H.cnt[k] < TRAIL_HIST) { H.cnt[k]++; }
    H.step[k] = stepPx;
    H.cls[k] = cls;
    H.lat[i] = lat;
    H.lon[i] = lon;
  }
  function expandBox(b, f, full) {
    var dla = (b[2] - b[0]) * f, dlo = (b[3] - b[1]) * f;
    return [Math.max(full[0], b[0] - dla), Math.max(full[1], b[1] - dlo),
            Math.min(full[2], b[2] + dla), Math.min(full[3], b[3] + dlo)];
  }
  var EMPTY_REGION = { nCells: 0, cells: null, cum: null, total: 0, area: 0, sub: null, bbox: null };

  // ───────────────────────────── 實例 ─────────────────────────────
  function Wind(host, adapter) {
    if (!host || typeof host.appendChild !== 'function') { throw new TypeError('AeroWind.create：host 必須是 DOM 元素'); }
    if (!adapter || typeof adapter.project !== 'function') { throw new TypeError('AeroWind.create：adapter.project 必須是函式'); }
    this.host = host;
    this.adapter = adapter;
    this.opts = copyOpts(DEFAULTS);
    this.fields = new Map();
    this.arrowLayers = new Map();
    this.frames = 0;
    this.w = 0;
    this.h = 0;
    this.dpr = 0;
    this._rng = Math.random;
    this._lastKey = null;
    this._raf = 0;
    this._overlayDirty = true;
    this._errLogged = false;
    this._warned = {};
    this._destroyed = false;
    this._px = 0;
    this._pw = NaN;                  // 上一次 _proj 的 w（adapter 沒給 ⇒ NaN）
    this._py = 0;
    this._dx = 0;
    this._dy = 0;
    this._sawNull = false;
    this._lastDense = -1e12;
    this._scratch = { u: 0, v: 0 };
    this._segs = [[], [], []];
    this._segsF = [[], [], []];      // faint 風場的軌跡（先畫、較淡）
    this._trail = { mode: 'none', segments: 0, jumps: 0, renderer: null };   // 上一幀的軌跡畫法（測試／診斷用）
    this._mask = [];
    this._legend = [];
    this._geom = [];
    var cv = document.createElement('canvas');
    cv.className = 'aerowind-canvas';
    cv.style.cssText = 'position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;';
    cv.style.zIndex = String(this.opts.zIndex);
    this.canvas = cv;
    this._trailCanvas = document.createElement('canvas');     // 離屏：粒子軌跡
    this._overlayCanvas = document.createElement('canvas');   // 離屏：箭頭＋文字＋圖例
    this._ctx = cv.getContext('2d');
    this._ctxT = this._trailCanvas.getContext('2d');
    this._ctxO = this._overlayCanvas.getContext('2d');
    try {
      if (root.getComputedStyle && root.getComputedStyle(host).position === 'static') { host.style.position = 'relative'; }
    } catch (e) { /* 取不到樣式就算了 */ }
    host.appendChild(cv);
    var self = this;
    this._tickBound = function () { self._tick(); };
    this._syncSize();
  }

  Wind.prototype._warnOnce = function (key, msg) {
    if (this._warned[key]) { return; }
    this._warned[key] = true;
    try { root.console.warn(msg); } catch (e) { /* 沒有 console */ }
  };

  // ── 資料 ──
  Wind.prototype.setField = function (a, b) {
    if (this._destroyed) { return false; }
    var id, f;
    if (b === undefined && (a === null || a === undefined || typeof a === 'object')) { id = DEFAULT_ID; f = a; }
    else { id = String(a); f = b; }
    var P = null;
    try { P = prepareField(f); } catch (e) { P = null; }
    if (!P || !P.nValid) {
      if (f !== null && f !== undefined) {
        this._warnOnce('field:' + id, '[AeroWind] 風場「' + id + '」沒有可用資料（空格點、座標無效或全為 null），已移除該層');
      }
      this.clearField(id);
      return false;
    }
    var mode = f.mode === undefined || f.mode === null ? 'flow' : f.mode;
    if (mode !== 'flow' && mode !== 'arrows' && mode !== 'both') {
      this._warnOnce('mode:' + mode, '[AeroWind] 不認得的 mode「' + mode + '」，改用 flow');
      mode = 'flow';
    }
    var h = num(f.height_m), np = num(f.particles), old = this.fields.get(id);
    var sh = num(f.particleShare), nm = num(f.particlesMin);
    this.fields.set(id, {
      id: id, P: P, mode: mode, h: isFinite(h) ? h : 0,
      label: (f.label === null || f.label === undefined || f.label === '') ? null : String(f.label),
      group: (typeof f.group === 'string' && f.group) ? f.group : null,
      groupLabel: (f.groupLabel === null || f.groupLabel === undefined || f.groupLabel === '') ? null : String(f.groupLabel),
      nReq: (isFinite(np) && np > 0) ? Math.min(MAX_PARTICLES, Math.round(np)) : 0,
      share: (isFinite(sh) && sh > 0 && sh <= 1) ? sh : 0,
      nMin: (isFinite(nm) && nm > 0) ? Math.min(MAX_PARTICLES, Math.round(nm)) : 0,
      faint: f.faint === true,
      pt: old ? old.pt : null, region: old ? old.region : null, lat: null, regionDirty: true, pendingDense: false,
      alive: 0, stat: { dx: 0, dy: 0, len: 0, n: 0 }
    });
    this._overlayDirty = true;
    this._afterChange();
    return true;
  };

  Wind.prototype.clearField = function (id) {
    if (this._destroyed) { return false; }
    var had;
    if (id === undefined || id === null) { had = this.fields.size > 0; this.fields.clear(); }
    else { had = this.fields.delete(String(id)); }
    if (had) {
      var flowLeft = false;
      this.fields.forEach(function (F) { if (F.mode !== 'arrows') { flowLeft = true; } });
      if (!flowLeft) { this._clearCanvas(this._ctxT, this._trailCanvas); }
      this._overlayDirty = true;
      this._afterChange();
    }
    return had;
  };

  Wind.prototype.setArrows = function (a, b, c) {
    if (this._destroyed) { return 0; }
    var id, rows, opts;
    if (typeof a === 'string' || typeof a === 'number') { id = String(a); rows = b; opts = c; }
    else { id = DEFAULT_ID; rows = a; opts = b; }
    var items = [], bad = 0, mx = 0;
    if (isRowLike(rows)) {
      for (var i = 0; i < rows.length; i++) {
        var it = parseArrow(rows[i]);
        if (it) { items.push(it); if (it.speed > mx) { mx = it.speed; } } else { bad++; }
      }
    }
    if (bad) { this._warnOnce('arrows:' + id, '[AeroWind] 箭頭層「' + id + '」有 ' + bad + ' 列資料無效（經緯度／風向／風速），已略過'); }
    if (!items.length) { this.clearArrows(id); return 0; }
    this.arrowLayers.set(id, {
      id: id, items: items, max: mx,
      label: (opts && opts.label !== undefined && opts.label !== null && opts.label !== '') ? String(opts.label) : null
    });
    this._overlayDirty = true;
    this._afterChange();
    return items.length;
  };

  Wind.prototype.clearArrows = function (id) {
    if (this._destroyed) { return false; }
    var had;
    if (id === undefined || id === null) { had = this.arrowLayers.size > 0; this.arrowLayers.clear(); }
    else { had = this.arrowLayers.delete(String(id)); }
    if (had) { this._overlayDirty = true; this._afterChange(); }
    return had;
  };

  Wind.prototype.clear = function () {
    var a = this.clearField(), b = this.clearArrows();
    return a || b;
  };

  Wind.prototype.setOptions = function (o) {
    if (this._destroyed || !o || typeof o !== 'object') { return this.options(); }
    var O = this.opts, wasAnimate = O.animate;
    for (var k in o) {
      if (!Object.prototype.hasOwnProperty.call(o, k)) { continue; }
      var key = OPT_ALIASES[k] || k;
      if (!Object.prototype.hasOwnProperty.call(VALIDATORS, key)) {
        this._warnOnce('opt:' + k, '[AeroWind] 不認得的選項「' + k + '」，已忽略');
        continue;
      }
      var val = VALIDATORS[key](o[k]);
      if (val === INVALID) {
        this._warnOnce('optv:' + k, '[AeroWind] 選項「' + k + '」的值無效（' + String(o[k]) + '），保留 ' + String(O[key]));
        continue;
      }
      O[key] = val;
      if (key === 'seed') { this._rng = val === null ? Math.random : mulberry32(val); }
    }
    this.canvas.style.zIndex = String(O.zIndex);
    this.canvas.style.opacity = String(O.opacity);
    this.canvas.style.display = O.visible ? '' : 'none';
    if (wasAnimate && !O.animate) {
      this._clearCanvas(this._ctxT, this._trailCanvas);
      this.fields.forEach(function (F) { F.alive = 0; if (F.pt && F.pt.hist) { F.pt.hist.cnt.fill(0); } });
    }
    this._overlayDirty = true;
    this._afterChange();
    return this.options();
  };

  Wind.prototype.options = function () { return copyOpts(this.opts); };

  Wind.prototype.redraw = function () {
    if (this._destroyed) { return; }
    this._overlayDirty = true;
    this.fields.forEach(function (F) { F.regionDirty = true; });
    this._afterChange();
  };

  Wind.prototype.interpAt = function (id, lat, lon) {
    var F = this.fields.get(id === undefined || id === null ? DEFAULT_ID : String(id));
    if (!F) { return null; }
    var o = { u: 0, v: 0 };
    return interpInto(F.P, num(lat), num(lon), o) ? o : null;
  };

  // ── 迴圈 ──
  Wind.prototype._hasContent = function () { return this.fields.size > 0 || this.arrowLayers.size > 0; };

  Wind.prototype._stopLoop = function () {
    if (this._raf) {
      try { root.cancelAnimationFrame(this._raf); } catch (e) { /* 無 */ }
      this._raf = 0;
    }
  };

  Wind.prototype._afterChange = function () {
    if (this._destroyed) { return; }
    if (!this._hasContent()) {
      this._stopLoop();
      this._clearCanvas(this._ctxT, this._trailCanvas);
      this._clearCanvas(this._ctxO, this._overlayCanvas);
      this._clearCanvas(this._ctx, this.canvas);
      this._legend = [];
      this._geom = [];
      return;
    }
    if (!this.opts.visible) { this._stopLoop(); return; }
    if (!this._raf) { this._raf = root.requestAnimationFrame(this._tickBound); }
  };

  Wind.prototype._tick = function () {
    this._raf = 0;
    if (this._destroyed) { return; }
    try {
      this._frame();
    } catch (e) {
      if (!this._errLogged) {
        this._errLogged = true;
        try { root.console.error('[AeroWind] 畫面更新失敗（之後同類錯誤不再列印）', e); } catch (e2) { /* 無 */ }
      }
    }
    if (!this._destroyed && this.opts.visible && this._hasContent()) {
      this._raf = root.requestAnimationFrame(this._tickBound);
    }
  };

  Wind.prototype._syncSize = function () {
    var w = this.host.clientWidth | 0, h = this.host.clientHeight | 0, dpr = root.devicePixelRatio || 1;
    if (w === this.w && h === this.h && dpr === this.dpr) { return false; }
    this.w = w;
    this.h = h;
    this.dpr = dpr;
    var pw = Math.max(1, Math.round(w * dpr)), ph = Math.max(1, Math.round(h * dpr));
    var cs = [this.canvas, this._trailCanvas, this._overlayCanvas];
    for (var i = 0; i < cs.length; i++) { cs[i].width = pw; cs[i].height = ph; }
    return true;
  };

  Wind.prototype._viewKey = function () {
    var a = this.adapter;
    if (typeof a.viewKey !== 'function') { return ''; }
    try { return String(a.viewKey()); } catch (e) { return ''; }
  };

  Wind.prototype._proj = function (lat, lon, h) {      // 呼叫 adapter；成功時結果放在 _px/_py
    var p;
    try { p = this.adapter.project(lat, lon, h); } catch (e) { return false; }
    if (!p) { return false; }
    var x, y, w = NaN;
    if (typeof p.x === 'number') { x = p.x; y = p.y; if (typeof p.w === 'number' && p.w > 0 && isFinite(p.w)) { w = p.w; } }
    else if (isRowLike(p)) { x = p[0]; y = p[1]; }
    else { return false; }
    if (!(typeof x === 'number' && typeof y === 'number' && isFinite(x) && isFinite(y))) { return false; }
    this._px = x;
    this._py = y;
    this._pw = w;
    return true;
  };

  Wind.prototype._onScreen = function () {
    return this._px >= -EDGE_PX && this._px <= this.w + EDGE_PX && this._py >= -EDGE_PX && this._py <= this.h + EDGE_PX;
  };

  Wind.prototype._clearCanvas = function (ctx, cv) {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalCompositeOperation = 'source-over';
    ctx.globalAlpha = 1;
    ctx.clearRect(0, 0, cv.width, cv.height);
  };

  Wind.prototype._frame = function () {
    if (this._destroyed) { return; }
    var resized = this._syncSize();
    var key = this._viewKey();
    var viewChanged = resized || key !== this._lastKey;
    if (viewChanged) {
      this._lastKey = key;
      this._clearCanvas(this._ctxT, this._trailCanvas);
      this._overlayDirty = true;
      this.fields.forEach(function (F) { F.regionDirty = true; });
    }
    var o = this.opts, self = this, stepped = false;
    if (o.visible && o.animate && this.w >= 2 && this.h >= 2) {
      if (!viewChanged) { this._fade(); }
      var segs = this._segs, segsF = this._segsF;
      segs[0].length = 0;
      segs[1].length = 0;
      segs[2].length = 0;
      segsF[0].length = 0;
      segsF[1].length = 0;
      segsF[2].length = 0;
      var follow = o.trailsFollowView && viewChanged, gl = follow ? this._glInit() : null;
      if (follow && !gl && !HAS_PATH2D) { follow = false; }
      var record = o.trailsFollowView && (gl !== null || HAS_PATH2D);
      var paths = (follow && !gl) ? this._newTrailPaths() : null, nHist = 0, nJump = 0;
      if (gl) { this._glN[0] = 0; this._glN[1] = 0; }
      this.fields.forEach(function (F) {
        F.trailSegs = 0;
        if (F.mode === 'arrows') { F.alive = 0; return; }
        if (F.regionDirty) { self._computeRegion(F); }
        self._stepParticles(F, !viewChanged, record, follow);
        if (follow) {
          F.trailSegs = gl ? self._traceHistoryGL(F) : self._traceHistory(F, paths);
          nHist += F.trailSegs;
          nJump += F.trailJumps;
        }
      });
      if (follow) { if (gl) { this._glDraw(); } else { this._strokeTrails(paths); } }
      this._drawSegs();
      this._applyMask();
      if (this._hasContent()) { this.frames++; }
      this._trail = { mode: follow ? 'follow' : (viewChanged ? 'cleared' : 'accumulate'), segments: nHist, jumps: nJump,
                      renderer: follow ? (gl ? 'webgl' : 'canvas2d') : null };
      stepped = true;
    }
    var overlayChanged = false;
    if (this._overlayDirty) {
      this._overlayDirty = false;
      this._drawOverlay();
      overlayChanged = true;
    }
    if (stepped || overlayChanged || viewChanged) { this._composite(); }
  };

  Wind.prototype._composite = function () {
    var ctx = this._ctx;
    this._clearCanvas(ctx, this.canvas);
    if (!this.opts.visible) { return; }
    if (this.opts.animate) { ctx.drawImage(this._trailCanvas, 0, 0); }
    ctx.drawImage(this._overlayCanvas, 0, 0);
  };

  // ── 可見區與投影格網 ──
  // 在 [la0,la1]×[lo0,lo1] 上取 (N+1)² 個點投影；回傳格點座標與「可見格」（任一角投影成功且外框碰到畫面）。
  Wind.prototype._sample = function (F, la0, lo0, la1, lo1, N) {
    var NP = N + 1, n = NP * NP, xs = new Float64Array(n), ys = new Float64Array(n), ok = new Uint8Array(n);
    var ws = new Float64Array(n), anyW = false;
    var dla = (la1 - la0) / N, dlo = (lo1 - lo0) / N, nulls = false, a, b, k, q;
    for (a = 0; a < NP; a++) {
      for (b = 0; b < NP; b++) {
        k = a * NP + b;
        if (this._proj(la0 + a * dla, lo0 + b * dlo, F.h)) {
          ok[k] = 1; xs[k] = this._px; ys[k] = this._py; ws[k] = this._pw;
          if (this._pw === this._pw) { anyW = true; }
        } else { ws[k] = NaN; nulls = true; }
      }
    }
    if (nulls) { this._sawNull = true; }
    var W = this.w, H = this.h, M = EDGE_PX, offs = [0, 1, NP, NP + 1];
    var vis = new Uint8Array(N * N), cw = new Float64Array(N * N), csz = new Float64Array(N * N), full = new Uint8Array(N * N);
    var cnt = 0, amin = N, amax = -1, bmin = N, bmax = -1;
    for (a = 0; a < N; a++) {
      for (b = 0; b < N; b++) {
        var k0 = a * NP + b, c = 0, x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        for (q = 0; q < 4; q++) {
          k = k0 + offs[q];
          if (!ok[k]) { continue; }
          c++;
          if (xs[k] < x0) { x0 = xs[k]; }
          if (xs[k] > x1) { x1 = xs[k]; }
          if (ys[k] < y0) { y0 = ys[k]; }
          if (ys[k] > y1) { y1 = ys[k]; }
        }
        if (!c || x1 < -M || x0 > W + M || y1 < -M || y0 > H + M) { continue; }
        var ci = a * N + b, ww = Math.min(x1, W) - Math.max(x0, 0), hh = Math.min(y1, H) - Math.max(y0, 0);
        vis[ci] = 1;
        full[ci] = c === 4 ? 1 : 0;
        cw[ci] = (ww > 0 ? ww : 0) * (hh > 0 ? hh : 0);
        csz[ci] = Math.max(x1 - x0, y1 - y0);
        cnt++;
        if (a < amin) { amin = a; }
        if (a > amax) { amax = a; }
        if (b < bmin) { bmin = b; }
        if (b > bmax) { bmax = b; }
      }
    }
    return { N: N, NP: NP, la0: la0, lo0: lo0, dla: dla, dlo: dlo, xs: xs, ys: ys, ws: anyW ? ws : null, ok: ok, vis: vis, cw: cw, csz: csz,
             full: full, cnt: cnt, nulls: nulls, amin: amin, amax: amax, bmin: bmin, bmax: bmax };
  };

  // 從 box 開始反覆縮小到「可見格的聯集」（有 null 時外擴一圈：畫面外回 null 的轉接器，邊上的格子可能部分可見）。
  Wind.prototype._refine = function (F, box, N0) {
    var la0 = box[0], lo0 = box[1], la1 = box[2], lo1 = box[3], N = N0, found = null;
    for (var it = 0; it < 8; it++) {
      var S = this._sample(F, la0, lo0, la1, lo1, N);
      if (!S.cnt) { break; }
      var ring = S.nulls ? 1 : 0;
      var amin = Math.max(0, S.amin - ring), amax = Math.min(N - 1, S.amax + ring);
      var bmin = Math.max(0, S.bmin - ring), bmax = Math.min(N - 1, S.bmax + ring);
      found = [la0 + amin * S.dla, lo0 + bmin * S.dlo, la0 + (amax + 1) * S.dla, lo0 + (bmax + 1) * S.dlo];
      if ((amax - amin + 1) > 0.6 * N && (bmax - bmin + 1) > 0.6 * N) { break; }
      la0 = found[0]; lo0 = found[1]; la1 = found[2]; lo1 = found[3];
      N = 12;
    }
    return found;
  };

  Wind.prototype._computeRegion = function (F) {
    F.regionDirty = false;
    var P = F.P, full = [P.latMin, P.lonMin, P.latMax, P.lonMax];
    var hint = (F.region && F.region.sub) ? F.region.sub : null, sub = null, t;
    F.region = EMPTY_REGION;
    F.lat = null;
    if (this.w < 2 || this.h < 2) { return; }
    this._sawNull = false;
    if (F.pendingDense) {
      t = now();
      if (t - this._lastDense < DENSE_THROTTLE_MS) { F.regionDirty = true; return; }
      F.pendingDense = false;
      this._lastDense = t;
      sub = this._refine(F, full, 128);
    } else {
      sub = this._refine(F, full, 12);
      if (!sub && this._sawNull && hint) { sub = this._refine(F, expandBox(hint, 1.0, full), 12); }
      if (!sub && this._sawNull) {
        sub = this._refine(F, full, 48);
        if (!sub) {
          t = now();
          if (t - this._lastDense >= DENSE_THROTTLE_MS) { this._lastDense = t; sub = this._refine(F, full, 128); }
          else { F.pendingDense = true; F.regionDirty = true; }
        }
      }
    }
    if (sub) { this._buildLattice(F, sub); }
  };

  Wind.prototype._buildLattice = function (F, sub) {
    var S = this._sample(F, sub[0], sub[1], sub[2], sub[3], 12);
    if (!S.cnt) { return; }
    var sizes = [], i;
    for (i = 0; i < S.vis.length; i++) { if (S.vis[i]) { sizes.push(S.csz[i]); } }
    sizes.sort(function (p, q) { return p - q; });
    var med = sizes[sizes.length >> 1] || 0, G = 12 * clamp(Math.ceil(med / LATTICE_PX), 1, 3);
    if (G > 12) {
      var S2 = this._sample(F, sub[0], sub[1], sub[2], sub[3], G);
      if (S2.cnt) { S = S2; }
    }
    var N = S.N, cells = [], wts = [], area = 0, wsum = 0, a, b, ci;
    for (ci = 0; ci < N * N; ci++) {
      if (!S.vis[ci]) { continue; }
      cells.push(ci);
      wts.push(S.cw[ci] > 1 ? S.cw[ci] : 1);
      area += S.cw[ci];
      wsum += S.cw[ci] > 1 ? S.cw[ci] : 1;
    }
    if (S.nulls) {
      // 畫面外回 null 的轉接器：有角投影失敗的邊緣格只量得到畫面內那幾個角 ⇒ 權重被低估、邊上會變稀。
      // 邊緣格與外圍一圈鄰格都給「完整格」的中位數權重，多撒到畫面外的點由 _spawn 的可見檢查剔除。
      var fw = [];
      for (i = 0; i < cells.length; i++) { if (S.full[cells[i]]) { fw.push(wts[i]); } }
      fw.sort(function (p, q) { return p - q; });
      var avg = fw.length ? fw[fw.length >> 1] : (cells.length ? wsum / cells.length : 1), seen = new Uint8Array(N * N);
      for (i = 0; i < cells.length; i++) {
        seen[cells[i]] = 1;
        if (!S.full[cells[i]] && wts[i] < avg) { wts[i] = avg; }
      }
      var base = cells.length;
      for (i = 0; i < base; i++) {
        a = (cells[i] / N) | 0;
        b = cells[i] - a * N;
        for (var da = -1; da <= 1; da++) {
          for (var db = -1; db <= 1; db++) {
            var na = a + da, nb = b + db;
            if (na < 0 || nb < 0 || na >= N || nb >= N) { continue; }
            var nk = na * N + nb;
            if (seen[nk]) { continue; }
            seen[nk] = 1;
            cells.push(nk);
            wts.push(avg);
          }
        }
      }
    }
    var cum = new Float64Array(cells.length), tot = 0;
    for (i = 0; i < cells.length; i++) { tot += wts[i]; cum[i] = tot; }
    var bx0 = Infinity, by0 = Infinity, bx1 = -Infinity, by1 = -Infinity;
    for (i = 0; i < S.ok.length; i++) {
      if (!S.ok[i]) { continue; }
      var x = clamp(S.xs[i], 0, this.w), y = clamp(S.ys[i], 0, this.h);
      if (x < bx0) { bx0 = x; }
      if (x > bx1) { bx1 = x; }
      if (y < by0) { by0 = y; }
      if (y > by1) { by1 = y; }
    }
    if (S.nulls && bx1 >= bx0) {
      // 畫面外回 null 的轉接器：邊緣格只量得到畫面內那幾個角 ⇒ 面積用「可見格點外框外擴一格」估
      var cs = [];
      for (i = 0; i < S.vis.length; i++) { if (S.vis[i]) { cs.push(S.csz[i]); } }
      cs.sort(function (p, q) { return p - q; });
      var md = cs[cs.length >> 1] || 0;
      var ea = (Math.min(this.w, bx1 + md) - Math.max(0, bx0 - md)) * (Math.min(this.h, by1 + md) - Math.max(0, by0 - md));
      if (ea > area) { area = ea; }
    }
    F.lat = S;
    F.region = {
      nCells: cells.length, cells: Int32Array.from(cells), cum: cum, total: tot,
      area: Math.min(area, this.w * this.h), sub: sub.slice(), bbox: [bx0, by0, bx1, by1]
    };
  };

  // 經緯度 → 螢幕：可見區格網雙線性；格子有角投影失敗才逐點問 adapter。
  Wind.prototype._mapLL = function (F, lat, lon) {
    var L = F.lat;
    if (!L) { return false; }
    var fi = (lat - L.la0) / L.dla, fj = (lon - L.lo0) / L.dlo, N = L.N;
    if (!(fi >= 0 && fi <= N && fj >= 0 && fj <= N)) { return false; }
    var i = fi | 0, j = fj | 0;
    if (i >= N) { i = N - 1; }
    if (j >= N) { j = N - 1; }
    var ti = fi - i, tj = fj - j, NP = L.NP, k = i * NP + j, ok = L.ok;
    if (ok[k] && ok[k + 1] && ok[k + NP] && ok[k + NP + 1]) {
      var xs = L.xs, ys = L.ys, ws = L.ws;
      var w00 = (1 - ti) * (1 - tj), w01 = (1 - ti) * tj, w10 = ti * (1 - tj), w11 = ti * tj;
      if (ws) {                                          // 透視：在齊次座標內插（四角都要有 w）
        var a0 = ws[k] * w00, a1 = ws[k + 1] * w01, a2 = ws[k + NP] * w10, a3 = ws[k + NP + 1] * w11, cw = a0 + a1 + a2 + a3;
        if (cw > 0) {                                    // 有 NaN ⇒ cw 是 NaN ⇒ 不成立 ⇒ 下面照舊雙線性
          this._px = (xs[k] * a0 + xs[k + 1] * a1 + xs[k + NP] * a2 + xs[k + NP + 1] * a3) / cw;
          this._py = (ys[k] * a0 + ys[k + 1] * a1 + ys[k + NP] * a2 + ys[k + NP + 1] * a3) / cw;
          return true;
        }
      }
      this._px = xs[k] * w00 + xs[k + 1] * w01 + xs[k + NP] * w10 + xs[k + NP + 1] * w11;
      this._py = ys[k] * w00 + ys[k + 1] * w01 + ys[k + NP] * w10 + ys[k + NP + 1] * w11;
      return true;
    }
    return this._proj(lat, lon, F.h);
  };

  // ── 粒子 ──
  function areaCount(area) {             // 依可見面積的自動粒子數：每 1200 px² 一顆（400–1500），且不密於每 25 px² 一顆
    var n = clamp(Math.round(area / 1200), 400, 1500);
    return Math.min(n, Math.max(20, Math.round(area / 25)));
  }

  Wind.prototype._targetCount = function (F) {
    var area = F.region ? F.region.area : 0;
    if (F.share > 0) {                     // 同組分一個自動預算；particles 有給時是上限
      if (!(area > 0)) { return F.pt ? F.pt.n : 0; }
      var auto = this.opts.particles > 0 ? this.opts.particles : areaCount(area);
      // 下限 particlesMin，但不超過「面積密度上限 × share」：拉得很遠時全組加起來也不密於單一風場
      var n = Math.round(auto * F.share), floor = Math.min(F.nMin, Math.max(1, Math.round(Math.max(20, Math.round(area / 25)) * F.share)));
      if (n < floor) { n = floor; }
      if (F.nReq > 0 && n > F.nReq) { n = F.nReq; }
      return n;
    }
    if (F.nReq > 0) { return F.nReq; }
    if (this.opts.particles > 0) { return this.opts.particles; }
    if (!(area > 0)) { return F.pt ? F.pt.n : 0; }
    return areaCount(area);
  };

  Wind.prototype._spawn = function (F, k, randomAge) {
    var R = F.region, pt = F.pt, L = F.lat, sc = this._scratch, rng = this._rng;
    if (!R || !R.nCells || !L) { pt.alive[k] = 0; return false; }
    for (var t = 0; t < 6; t++) {
      var ci = R.cells[upperBound(R.cum, rng() * R.total)], a = (ci / L.N) | 0, b = ci - a * L.N;
      var lat = L.la0 + (a + rng()) * L.dla, lon = L.lo0 + (b + rng()) * L.dlo;
      if (!interpInto(F.P, lat, lon, sc)) { continue; }
      if (!this._mapLL(F, lat, lon) || !this._onScreen()) { continue; }
      pt.lat[k] = lat;
      pt.lon[k] = lon;
      pt.alive[k] = 1;
      pt.age[k] = randomAge ? Math.floor(rng() * this.opts.maxAge) : 0;
      if (pt.hist) { histReset(pt.hist, k, lat, lon); }
      return true;
    }
    pt.alive[k] = 0;
    if (pt.hist) { pt.hist.cnt[k] = 0; }
    return false;
  };

  // draw：把這一步的線段排進累積畫布（視角沒變的幀）。record：記位置歷史（trailsFollowView）。
  // follow：這一幀視角變了、要用歷史重畫 ⇒ 先檢查每顆粒子上一段的螢幕長度有沒有跳（TRAIL_JUMP）。
  Wind.prototype._stepParticles = function (F, draw, record, follow) {
    var target = this._targetCount(F), pt = F.pt, explicit = F.share > 0 ? false : (F.nReq > 0 || this.opts.particles > 0);
    if (!pt || (pt.n !== target && (explicit || Math.abs(target - pt.n) > 0.15 * pt.n))) { F.pt = pt = allocParticles(pt, target); }
    F.alive = 0;
    F.stat = { dx: 0, dy: 0, len: 0, n: 0 };
    F.trailJumps = 0;
    var R = F.region;
    if (!R || !R.nCells || !pt.n) { pt.alive.fill(0); if (pt.hist) { pt.hist.cnt.fill(0); } return; }
    if (record && !pt.hist) { allocHistory(pt); }         // 剛打開：歷史從空的開始（cnt 0 ⇒ 下一步只記一點）
    if (!record && pt.hist) { pt.hist = null; }
    var H = pt.hist, jumps = 0;
    var P = F.P, o = this.opts, sc = this._scratch, lim = o.limitMps, cf = CAUTION_FRAC * lim, pxm = o.pxPerMps;
    var maxAge = o.maxAge, segs = F.faint ? this._segsF : this._segs, sdx = 0, sdy = 0, sl = 0, sn = 0, alive = 0;
    for (var k = 0; k < pt.n; k++) {
      if (!pt.alive[k]) { if (this._spawn(F, k, true)) { alive++; } continue; }
      if (pt.age[k] >= maxAge) { if (this._spawn(F, k, false)) { alive++; } continue; }
      var lat = pt.lat[k], lon = pt.lon[k];
      if (!interpInto(P, lat, lon, sc) || !this._mapLL(F, lat, lon) || !this._onScreen()) {
        if (this._spawn(F, k, true)) { alive++; }
        continue;
      }
      var x0 = this._px, y0 = this._py, u = sc.u, v = sc.v, s = Math.sqrt(u * u + v * v);
      if (H) {
        if (!H.cnt[k]) { histReset(H, k, lat, lon); }
        else if (follow && H.cnt[k] >= 2 && H.step[k] > 0.5) {
          // 上一段（前一格 → 現在）在新視角下的長度 vs 記錄時：跳太多 ⇒ 舊歷史在新視角裡會被拉長／壓扁，丟掉
          var hp = H.head[k] - 1;
          if (hp < 0) { hp += TRAIL_HIST; }
          var jL = -1;
          if (this._mapLL(F, H.lat[k * TRAIL_HIST + hp], H.lon[k * TRAIL_HIST + hp])) {
            jL = Math.sqrt((this._px - x0) * (this._px - x0) + (this._py - y0) * (this._py - y0)) / H.step[k];
          }
          if (!(jL <= TRAIL_JUMP && jL >= 1 / TRAIL_JUMP)) { histReset(H, k, lat, lon); jumps++; }
        }
      }
      if (s < 1e-9) {                                     // 無風：不動也不畫；歷史只留現在這一點（不留一條凍住的舊線）
        pt.age[k]++;
        alive++;
        if (H) { histReset(H, k, lat, lon); }
        continue;
      }
      var cosLat = Math.cos(lat * D2R);
      if (cosLat < 1e-6) { cosLat = 1e-6; }
      var dLat = (v / s) * PROBE_M / M_PER_DEG, dLon = (u / s) * PROBE_M / (M_PER_DEG * cosLat), wx, wy;
      if (this._mapLL(F, lat + dLat, lon + dLon)) { wx = this._px - x0; wy = this._py - y0; }
      else if (this._mapLL(F, lat - dLat, lon - dLon)) { wx = x0 - this._px; wy = y0 - this._py; }
      else { if (this._spawn(F, k, true)) { alive++; } continue; }
      var wl = Math.sqrt(wx * wx + wy * wy);
      if (!(wl > 1e-12)) { if (this._spawn(F, k, true)) { alive++; } continue; }
      var stepPx = s * pxm, frac = stepPx / wl, x1 = x0 + wx * frac, y1 = y0 + wy * frac, cls = s >= lim ? 2 : (s >= cf ? 1 : 0);
      if (draw) { segs[cls].push(x0, y0, x1, y1); }
      sdx += x1 - x0;
      sdy += y1 - y0;
      sl += stepPx;
      sn++;
      alive++;
      pt.lat[k] = lat + dLat * frac;
      pt.lon[k] = lon + dLon * frac;
      pt.age[k]++;
      if (H) { histPush(H, k, pt.lat[k], pt.lon[k], stepPx, cls); }
    }
    F.alive = alive;
    F.stat = { dx: sn ? sdx / sn : 0, dy: sn ? sdy / sn : 0, len: sn ? sl / sn : 0, n: sn };
    F.trailJumps = jumps;
  };

  // trailsFollowView、視角變了的那一幀：把每顆活粒子的位置歷史用「現在的」投影連成折線，依年齡分段放進
  // paths[faint][等級][段]（Path2D），最後由 _strokeTrails 一段一段描（舊的先畫、淡；新的後畫）。
  // 投影走 _mapLL（可見區格網雙線性）；某一點投影失敗就在那裡截斷（更舊的不畫）。回傳畫了幾段。
  Wind.prototype._traceHistory = function (F, paths) {
    var pt = F.pt, H = pt ? pt.hist : null;
    if (!H || !F.lat) { return 0; }
    var fi = F.faint ? 1 : 0, fp = paths.p[fi], used = paths.used[fi], nSeg = 0;
    for (var k = 0; k < pt.n; k++) {
      var c = H.cnt[k];
      if (!pt.alive[k] || c < 2) { continue; }
      var base = k * TRAIL_HIST, head = H.head[k], cl = H.cls[k], cp = fp[cl], px = 0, py = 0, prev = -1;
      for (var a = 0; a < c; a += TRAIL_STRIDE) {
        if (a + TRAIL_STRIDE >= c) { a = c - 1; }        // 最後一點用最舊那格（不一定落在步距上）
        var j = head - a;
        if (j < 0) { j += TRAIL_HIST; }
        if (!this._mapLL(F, H.lat[base + j], H.lon[base + j])) { break; }
        if (prev >= 0) {                                  // 這一段的「新端」年齡 prev 決定透明度分段
          var p = cp[Math.min(TRAIL_BANDS - 1, (prev / TRAIL_BAND_FRAMES) | 0)];
          p.moveTo(px, py);
          p.lineTo(this._px, this._py);
          nSeg++;
        }
        px = this._px;
        py = this._py;
        prev = a;
      }
      if (prev > 0) { used[cl] = 1; }
    }
    return nSeg;
  };

  Wind.prototype._newTrailPaths = function () {
    var out = { p: [], used: [[0, 0, 0], [0, 0, 0]] };
    for (var f = 0; f < 2; f++) {
      var byCls = [];
      for (var c = 0; c < 3; c++) {
        var bands = [];
        for (var b = 0; b < TRAIL_BANDS; b++) { bands.push(new root.Path2D()); }
        byCls.push(bands);
      }
      out.p.push(byCls);
    }
    return out;
  };

  Wind.prototype._strokeTrails = function (paths) {
    var ctx = this._ctxT, o = this.opts;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.globalCompositeOperation = 'source-over';
    ctx.lineCap = 'round';
    for (var f = 1; f >= 0; f--) {                       // faint 先畫
      ctx.lineWidth = f ? o.lineWidth * FAINT_WIDTH : o.lineWidth;
      var a0 = f ? o.particleAlpha * FAINT_ALPHA : o.particleAlpha;
      for (var b = TRAIL_BANDS - 1; b >= 0; b--) {        // 舊的先畫
        ctx.globalAlpha = a0 * Math.pow(o.fade, b * TRAIL_BAND_FRAMES);
        for (var c = 0; c < 3; c++) {
          if (!paths.used[f][c]) { continue; }
          ctx.strokeStyle = CLASS_COLORS[c];
          ctx.stroke(paths.p[f][c][b]);
        }
      }
    }
    ctx.globalAlpha = 1;
  };

  // ── WebGL 拖尾（trailsFollowView 的重畫，優先用）──
  // 為什麼不用 Canvas 2D：Chromium 的 2D canvas 描「寬度 > 1 裝置像素」的反鋸齒線，成本跟總線長成正比（2026-10-21
  // 在 QtWebEngine 最大化 2560×1369、dpr 1.5 實測：每幀 1,500 段 × 30 px ⇒ 43 FPS、12,000 段 ⇒ 13 FPS）。靜止時便宜是因為
  // 每幀只畫 3 px 的新段、靠淡出累積；重畫整條拖尾就要畫全長。WebGL 每段一個四邊形，GPU 畫幾萬段不到 1 ms；
  // 透明度逐頂點給（= particleAlpha × fade^年齡，與累積畫布的淡出同一條曲線）、橫向用距離算覆蓋率（反鋸齒）。
  // 畫好之後在同一個 task 裡 drawImage 到軌跡畫布，之後的靜止幀照常在上面淡出、疊新段。
  // 拿不到 WebGL（或 context 掉了、著色器編不過）⇒ 回 null，改用 Canvas 2D（_traceHistory／_strokeTrails）。
  var GL_VS = 'attribute vec2 p; attribute vec2 s; attribute vec4 c; uniform vec2 sz; varying vec2 vs; varying vec4 vc;' +
              'void main() { vs = s; vc = c; gl_Position = vec4(p.x / sz.x * 2.0 - 1.0, 1.0 - p.y / sz.y * 2.0, 0.0, 1.0); }';
  var GL_FS = 'precision mediump float; varying vec2 vs; varying vec4 vc;' +
              'void main() { float d = abs(vs.x) * (vs.y + 1.0); float a = vc.a * clamp(vs.y + 0.5 - d, 0.0, 1.0);' +
              ' gl_FragColor = vec4(vc.rgb * a, a); }';

  Wind.prototype._glInit = function () {
    if (this._gl !== undefined) { return this._gl; }       // null = 不可用（之後都走 Canvas 2D）
    this._gl = null;
    if (this._noGL) { return null; }
    var self = this, cv, gl, why = '';
    try {
      cv = document.createElement('canvas');
      gl = cv.getContext('webgl', { alpha: true, premultipliedAlpha: true, antialias: false, depth: false, stencil: false,
                                    preserveDrawingBuffer: false });
      if (!gl) { why = '拿不到 WebGL'; }
    } catch (e) { gl = null; why = String(e); }
    var prog = null;
    if (gl) {
      var sh = function (type, src) {
        var o = gl.createShader(type);
        gl.shaderSource(o, src);
        gl.compileShader(o);
        if (!gl.getShaderParameter(o, gl.COMPILE_STATUS)) { why = '著色器：' + gl.getShaderInfoLog(o); return null; }
        return o;
      };
      var vs = sh(gl.VERTEX_SHADER, GL_VS), fs = vs ? sh(gl.FRAGMENT_SHADER, GL_FS) : null;
      if (vs && fs) {
        prog = gl.createProgram();
        gl.attachShader(prog, vs);
        gl.attachShader(prog, fs);
        gl.linkProgram(prog);
        if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) { why = '連結：' + gl.getProgramInfoLog(prog); prog = null; }
      }
    }
    if (!prog) {
      this._warnOnce('gl', '[AeroWind] 拖尾改用 Canvas 2D（' + (why || 'WebGL 不可用') + '）：大畫面追機時可能掉幀');
      return null;
    }
    var idx = new Uint16Array(GL_CHUNK * 6);
    for (var q = 0; q < GL_CHUNK; q++) {
      var v = q * 4, i = q * 6;
      idx[i] = v; idx[i + 1] = v + 1; idx[i + 2] = v + 2; idx[i + 3] = v + 2; idx[i + 4] = v + 1; idx[i + 5] = v + 3;
    }
    var ibo = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibo);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, idx, gl.STATIC_DRAW);
    cv.addEventListener('webglcontextlost', function () {
      self._gl = null;                                       // 之後改用 Canvas 2D；不再重建
      self._warnOnce('gllost', '[AeroWind] WebGL context 遺失，拖尾改用 Canvas 2D');
    }, false);
    this._glBuf = [new Float32Array(4096 * 4 * GL_FLOATS), new Float32Array(4096 * 4 * GL_FLOATS)];
    this._glN = [0, 0];
    this._gl = { cv: cv, gl: gl, prog: prog, vbo: gl.createBuffer(), ibo: ibo,
                 aP: gl.getAttribLocation(prog, 'p'), aS: gl.getAttribLocation(prog, 's'),
                 aC: gl.getAttribLocation(prog, 'c'), uSz: gl.getUniformLocation(prog, 'sz') };
    return this._gl;
  };

  function glVert(B, w, x, y, side, hw, rgb, a) {
    B[w] = x; B[w + 1] = y; B[w + 2] = side; B[w + 3] = hw; B[w + 4] = rgb[0]; B[w + 5] = rgb[1]; B[w + 6] = rgb[2]; B[w + 7] = a;
    return w + GL_FLOATS;
  }

  // 跟 _traceHistory 同樣的點（每隔 TRAIL_STRIDE 格、最後一點用最舊那格、投影失敗就截斷），每段一個四邊形放進
  // _glBuf[faint ? 1 : 0]（兩端各帶自己年齡的透明度）。回傳畫了幾段。
  Wind.prototype._traceHistoryGL = function (F) {
    var pt = F.pt, H = pt ? pt.hist : null;
    if (!H || !F.lat) { return 0; }
    var o = this.opts, fi = F.faint ? 1 : 0, a0 = o.particleAlpha * (fi ? FAINT_ALPHA : 1);
    var hw = o.lineWidth * (fi ? FAINT_WIDTH : 1) * this.dpr / 2, ext = (hw + 1) / this.dpr;   // 半線寬（裝置 px）、外擴（CSS px）
    var fpow = this._fpow, nSeg = 0;
    if (!fpow || this._fpowOf !== o.fade) {
      fpow = this._fpow = new Float64Array(TRAIL_HIST);
      this._fpowOf = o.fade;
      for (var t = 0; t < TRAIL_HIST; t++) { fpow[t] = Math.pow(o.fade, t); }
    }
    for (var k = 0; k < pt.n; k++) {
      var c = H.cnt[k];
      if (!pt.alive[k] || c < 2) { continue; }
      var base = k * TRAIL_HIST, head = H.head[k], rgb = CLASS_RGB[H.cls[k]], px = 0, py = 0, prev = -1;
      for (var a = 0; a < c; a += TRAIL_STRIDE) {
        if (a + TRAIL_STRIDE >= c) { a = c - 1; }
        var j = head - a;
        if (j < 0) { j += TRAIL_HIST; }
        if (!this._mapLL(F, H.lat[base + j], H.lon[base + j])) { break; }
        var x = this._px, y = this._py;
        if (prev >= 0) {
          var dx = x - px, dy = y - py, L = Math.sqrt(dx * dx + dy * dy);
          if (L > 1e-6) {
            var nx = -dy / L * ext, ny = dx / L * ext, n = this._glN[fi], need = (n + 1) * 4 * GL_FLOATS, B = this._glBuf[fi];
            if (B.length < need) {
              var nb = new Float32Array(Math.max(need, B.length * 2));
              nb.set(B);
              B = this._glBuf[fi] = nb;
            }
            var w = n * 4 * GL_FLOATS, ah = a0 * fpow[prev], at = a0 * fpow[a];
            w = glVert(B, w, px + nx, py + ny, 1, hw, rgb, ah);
            w = glVert(B, w, px - nx, py - ny, -1, hw, rgb, ah);
            w = glVert(B, w, x + nx, y + ny, 1, hw, rgb, at);
            glVert(B, w, x - nx, y - ny, -1, hw, rgb, at);
            this._glN[fi] = n + 1;
            nSeg++;
          }
        }
        px = x;
        py = y;
        prev = a;
      }
    }
    return nSeg;
  };

  // faint 先畫、再畫實線層；畫完在同一個 task 裡 drawImage 到（這一幀已清空的）軌跡畫布。
  Wind.prototype._glDraw = function () {
    var G = this._gl, gl = G.gl, cv = G.cv, tw = this._trailCanvas.width, th = this._trailCanvas.height;
    if (cv.width !== tw || cv.height !== th) { cv.width = tw; cv.height = th; }
    gl.viewport(0, 0, tw, th);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    if (!this._glN[0] && !this._glN[1]) { return; }
    gl.useProgram(G.prog);
    gl.uniform2f(G.uSz, this.w, this.h);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);           // 輸出是預乘 alpha
    gl.bindBuffer(gl.ARRAY_BUFFER, G.vbo);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, G.ibo);
    gl.enableVertexAttribArray(G.aP);
    gl.enableVertexAttribArray(G.aS);
    gl.enableVertexAttribArray(G.aC);
    var stride = GL_FLOATS * 4;
    for (var fi = 1; fi >= 0; fi--) {
      var n = this._glN[fi];
      if (!n) { continue; }
      gl.bufferData(gl.ARRAY_BUFFER, this._glBuf[fi].subarray(0, n * 4 * GL_FLOATS), gl.STREAM_DRAW);
      for (var q0 = 0; q0 < n; q0 += GL_CHUNK) {
        var off = q0 * 4 * stride;
        gl.vertexAttribPointer(G.aP, 2, gl.FLOAT, false, stride, off);
        gl.vertexAttribPointer(G.aS, 2, gl.FLOAT, false, stride, off + 8);
        gl.vertexAttribPointer(G.aC, 4, gl.FLOAT, false, stride, off + 16);
        gl.drawElements(gl.TRIANGLES, Math.min(GL_CHUNK, n - q0) * 6, gl.UNSIGNED_SHORT, 0);
      }
    }
    var ctx = this._ctxT;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.globalCompositeOperation = 'source-over';
    ctx.drawImage(cv, 0, 0);
  };

  Wind.prototype._fade = function () {
    var ctx = this._ctxT;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.globalCompositeOperation = 'destination-in';
    ctx.fillStyle = 'rgba(0,0,0,' + this.opts.fade + ')';
    ctx.fillRect(0, 0, this._trailCanvas.width, this._trailCanvas.height);
    ctx.globalCompositeOperation = 'source-over';
  };

  Wind.prototype._drawSegs = function () {
    var ctx = this._ctxT, o = this.opts;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.globalCompositeOperation = 'source-over';
    ctx.lineCap = 'round';
    var passes = [[this._segsF, o.particleAlpha * FAINT_ALPHA, o.lineWidth * FAINT_WIDTH],
                  [this._segs, o.particleAlpha, o.lineWidth]];
    for (var p = 0; p < passes.length; p++) {
      var segs = passes[p][0];
      ctx.globalAlpha = passes[p][1];
      ctx.lineWidth = passes[p][2];
      for (var b = 0; b < 3; b++) {
        var a = segs[b];
        if (!a.length) { continue; }
        ctx.strokeStyle = CLASS_COLORS[b];
        ctx.beginPath();
        for (var i = 0; i < a.length; i += 4) {
          ctx.moveTo(a[i], a[i + 1]);
          ctx.lineTo(a[i + 2], a[i + 3]);
        }
        ctx.stroke();
      }
    }
    ctx.globalAlpha = 1;
  };

  Wind.prototype._applyMask = function () {
    var m = this._mask;
    if (!m.length) { return; }
    var ctx = this._ctxT;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    for (var i = 0; i < m.length; i++) { ctx.clearRect(m[i][0], m[i][1], m[i][2], m[i][3]); }
  };

  Wind.prototype.setMask = function (rects) {
    if (this._destroyed) { return 0; }
    var out = [];
    if (isRowLike(rects)) {
      for (var i = 0; i < rects.length && out.length < MAX_MASK_RECTS; i++) {
        var r = rects[i];
        if (!isRowLike(r) || r.length !== 4) { continue; }
        var x = num(r[0]), y = num(r[1]), w = num(r[2]), h = num(r[3]);
        if (isFinite(x) && isFinite(y) && isFinite(w) && isFinite(h) && w > 0 && h > 0) { out.push([x, y, w, h]); }
      }
    }
    this._mask = out;
    this._applyMask();
    return out.length;
  };

  // ── 箭頭與圖例 ──
  Wind.prototype._arrowLen = function (s) {
    var o = this.opts, lo = Math.min(o.arrowMinPx, o.arrowMaxPx), hi = Math.max(o.arrowMinPx, o.arrowMaxPx);
    return clamp(s * o.arrowPxPerMps, lo, hi);
  };

  // (x0, y0) 是 (lat, lon) 的投影；算出順風方向在螢幕上的單位向量放進 _dx/_dy。
  Wind.prototype._screenDir = function (h, lat, lon, u, v, x0, y0) {
    var s = Math.sqrt(u * u + v * v);
    if (!(s > 0)) { return false; }
    var cosLat = Math.max(Math.cos(lat * D2R), 1e-6);
    var dLat = (v / s) * PROBE_M / M_PER_DEG, dLon = (u / s) * PROBE_M / (M_PER_DEG * cosLat), dx, dy;
    if (this._proj(lat + dLat, lon + dLon, h)) { dx = this._px - x0; dy = this._py - y0; }
    else if (this._proj(lat - dLat, lon - dLon, h)) { dx = x0 - this._px; dy = y0 - this._py; }
    else { dx = u / s; dy = -v / s; }
    var l = Math.sqrt(dx * dx + dy * dy);
    if (!(l > 1e-12)) { dx = u / s; dy = -v / s; l = 1; }
    this._dx = dx / l;
    this._dy = dy / l;
    return true;
  };

  // 'arrows' 模式（或 animate=false）的稀疏箭頭：先用格點（跨步抽樣），不足 4 支再用錨定在風場範圍的虛擬格網；彼此 ≥ arrowSpacingPx。
  Wind.prototype._fieldSites = function (F) {
    if (F.regionDirty || !F.region) { this._computeRegion(F); }
    var R = F.region, P = F.P, out = [];
    if (!R || !R.nCells || !R.sub) { return out; }
    var self = this, sp = this.opts.arrowSpacingPx, W = this.w, H = this.h, inset = 10, hash = {}, sc = this._scratch;
    var add = function (lat, lon, u, v) {
      if (!(isFinite(u) && isFinite(v))) { return; }
      if (!self._proj(lat, lon, F.h)) { return; }
      var x = self._px, y = self._py;
      if (x < inset || x > W - inset || y < inset || y > H - inset) { return; }
      var cx = Math.floor(x / sp), cy = Math.floor(y / sp);
      for (var i = -1; i <= 1; i++) {
        for (var j = -1; j <= 1; j++) {
          var lst = hash[(cx + i) + ',' + (cy + j)];
          if (!lst) { continue; }
          for (var m = 0; m < lst.length; m++) {
            var ddx = lst[m].x - x, ddy = lst[m].y - y;
            if (ddx * ddx + ddy * ddy < sp * sp) { return; }
          }
        }
      }
      var site = { lat: lat, lon: lon, u: u, v: v, x: x, y: y }, hk = cx + ',' + cy;
      if (!hash[hk]) { hash[hk] = []; }
      hash[hk].push(site);
      out.push(site);
    };
    var sub = R.sub, i, j, a, b;
    if (P.ni >= 2 && P.nj >= 2) {
      var ia = bsearch(P.lats, sub[0]), ib = Math.min(P.ni - 1, bsearch(P.lats, sub[2]) + 1);
      var ja = bsearch(P.lons, sub[1]), jb = Math.min(P.nj - 1, bsearch(P.lons, sub[3]) + 1);
      var maxAlong = Math.ceil(Math.max(W, H) / sp) * 2 + 2;
      var si = Math.max(1, Math.ceil((ib - ia + 1) / maxAlong)), sj = Math.max(1, Math.ceil((jb - ja + 1) / maxAlong));
      for (i = ia; i <= ib; i += si) {
        for (j = ja; j <= jb; j += sj) {
          var kk = i * P.nj + j;
          add(P.lats[i], P.lons[j], P.U[kk], P.V[kk]);
        }
      }
    }
    if (out.length < 4) {
      var mLat = (sub[0] + sub[2]) / 2, mLon = (sub[1] + sub[3]) / 2, spanLon = NaN, spanLat = NaN, x0, y0;
      if (this._proj(mLat, sub[1], F.h)) {
        x0 = this._px; y0 = this._py;
        if (this._proj(mLat, sub[3], F.h)) { spanLon = Math.sqrt((this._px - x0) * (this._px - x0) + (this._py - y0) * (this._py - y0)); }
      }
      if (this._proj(sub[0], mLon, F.h)) {
        x0 = this._px; y0 = this._py;
        if (this._proj(sub[2], mLon, F.h)) { spanLat = Math.sqrt((this._px - x0) * (this._px - x0) + (this._py - y0) * (this._py - y0)); }
      }
      var bb = R.bbox;
      if (!isFinite(spanLon)) { spanLon = bb ? bb[2] - bb[0] : W; }
      if (!isFinite(spanLat)) { spanLat = bb ? bb[3] - bb[1] : H; }
      // 步距量化成「整個範圍 / 2^k」⇒ 平移時箭頭不會游移，只在縮放跨級時換格
      var fullLa = P.latMax - P.latMin, fullLo = P.lonMax - P.lonMin;
      var rawLa = (sub[2] - sub[0]) * sp * 1.1 / Math.max(spanLat, 1), rawLo = (sub[3] - sub[1]) * sp * 1.1 / Math.max(spanLon, 1);
      var stLa = fullLa / Math.pow(2, clamp(Math.floor(Math.log(fullLa / rawLa) / Math.LN2), 0, 30));
      var stLo = fullLo / Math.pow(2, clamp(Math.floor(Math.log(fullLo / rawLo) / Math.LN2), 0, 30));
      var a0 = Math.max(0, Math.floor((sub[0] - P.latMin) / stLa)), a1 = Math.ceil((sub[2] - P.latMin) / stLa);
      var b0 = Math.max(0, Math.floor((sub[1] - P.lonMin) / stLo)), b1 = Math.ceil((sub[3] - P.lonMin) / stLo);
      if ((a1 - a0 + 1) * (b1 - b0 + 1) <= 6400) {
        for (a = a0; a <= a1; a++) {
          var lat = P.latMin + (a + 0.5) * stLa;
          if (lat > P.latMax) { break; }
          for (b = b0; b <= b1; b++) {
            var lon = P.lonMin + (b + 0.5) * stLo;
            if (lon > P.lonMax) { break; }
            if (interpInto(P, lat, lon, sc)) { add(lat, lon, sc.u, sc.v); }
          }
        }
      }
    }
    return out;
  };

  Wind.prototype._drawOverlay = function () {
    var ctx = this._ctxO, o = this.opts, self = this, lim = o.limitMps;
    this._clearCanvas(ctx, this._overlayCanvas);
    this._geom = [];
    this._legend = [];
    if (!o.visible || this.w < 2 || this.h < 2 || !this._hasContent()) { return; }
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    this.fields.forEach(function (F, id) {
      if (o.animate && F.mode === 'flow') { return; }
      var sites = self._fieldSites(F);
      for (var i = 0; i < sites.length; i++) {
        var st = sites[i], s = Math.sqrt(st.u * st.u + st.v * st.v);
        if (!self._screenDir(F.h, st.lat, st.lon, st.u, st.v, st.x, st.y)) { continue; }
        var L = self._arrowLen(s), col = speedColor(s, lim);
        var g = drawArrow(ctx, st.x, st.y, self._dx, self._dy, L, col);
        self._geom.push({ kind: 'field', layer: id, lat: st.lat, lon: st.lon, x: st.x, y: st.y, tail: [g[0], g[1]],
                          tip: [g[2], g[3]], length: L, speed: s, dirFrom: uvToDir(st.u, st.v)[0], color: col, label: null });
      }
    });
    this.arrowLayers.forEach(function (A, id) {
      for (var i = 0; i < A.items.length; i++) {
        var it = A.items[i];
        if (!self._proj(it.lat, it.lon, it.h)) { continue; }
        var x = self._px, y = self._py;
        if (x < -20 || x > self.w + 20 || y < -20 || y > self.h + 20) { continue; }
        var col = it.color || speedColor(it.speed, lim), text = arrowText(it), L = 0, g, uv = dirToUV(it.dir, 1);
        if (it.speed < 0.05 || !self._screenDir(it.h, it.lat, it.lon, uv[0], uv[1], x, y)) {
          drawCalm(ctx, x, y, col);
          g = [x, y, x, y];
        } else {
          L = self._arrowLen(it.speed);
          g = drawArrow(ctx, x, y, self._dx, self._dy, L, col);
        }
        drawLabel(ctx, text, x, Math.max(g[1], g[3], y + 5) + 6, 'center');
        self._geom.push({ kind: 'arrow', layer: id, index: i, sysid: it.sysid, lat: it.lat, lon: it.lon, x: x, y: y,
                          tail: [g[0], g[1]], tip: [g[2], g[3]], length: L, speed: it.speed, dirFrom: it.dir,
                          color: col, label: text });
      }
    });
    if (o.showLegend) { this._drawLegend(ctx); }
  };

  Wind.prototype._drawLegend = function (ctx) {
    var o = this.opts, lim = o.limitMps, entries = [], maxShown = 0, groups = {};
    this.fields.forEach(function (F, id) {
      var mx = F.P.maxSpeed;
      if (mx > maxShown) { maxShown = mx; }
      if (F.group) {                          // 同組的風場合成一個區塊（佔第一個成員的位置）
        var g = groups[F.group];
        if (!g) { groups[F.group] = { label: F.groupLabel || F.group, mx: mx, at: entries.length }; entries.push(null); }
        else if (mx > g.mx) { g.mx = mx; }
        return;
      }
      entries.push({ lab: F.label || (id === DEFAULT_ID ? '風場' : id), mx: mx });
    });
    Object.keys(groups).forEach(function (k) { var g = groups[k]; entries[g.at] = { lab: g.label, mx: g.mx }; });
    var lines = [], i, j;
    for (i = 0; i < entries.length; i++) {    // label 可用換行分多行：第一行接最大風速（超過上限變紅），其餘照常
      var parts = String(entries[i].lab).split('\n');
      lines.push({ t: parts[0] + '　最大 ' + fmt1(entries[i].mx) + ' m/s', warn: entries[i].mx >= lim });
      for (j = 1; j < parts.length; j++) { if (parts[j]) { lines.push({ t: parts[j], warn: false }); } }
    }
    this.arrowLayers.forEach(function (A, id) {
      var lab = A.label || (id === DEFAULT_ID ? '各機風向' : id);
      lines.push({ t: lab + '　' + A.items.length + ' 點　最大 ' + fmt1(A.max) + ' m/s', warn: A.max >= lim });
      if (A.max > maxShown) { maxShown = A.max; }
    });
    if (!lines.length) { return; }
    ctx.font = FONT;
    var pad = 8, lh = 16, rampW = 160, rampH = 8, maxTextW = 360;
    var title = '風速 m/s（上限 ' + fmt1(lim) + '）';
    var texts = [], tw = rampW;
    for (i = 0; i < lines.length; i++) {
      texts.push(fitText(ctx, lines[i].t, maxTextW));
      tw = Math.max(tw, ctx.measureText(texts[i]).width);
    }
    tw = Math.max(tw, ctx.measureText(title).width);
    var boxW = Math.ceil(tw) + 2 * pad, boxH = pad + lines.length * lh + lh + rampH + 3 + 14 + pad;
    var m = o.legendMargin, corner = o.legendCorner;
    var x0 = corner.indexOf('right') >= 0 ? this.w - m[0] - boxW : m[0];
    var y0 = corner.indexOf('bottom') >= 0 ? this.h - m[1] - boxH : m[1];
    ctx.fillStyle = 'rgba(15,20,28,0.72)';
    roundRectPath(ctx, x0, y0, boxW, boxH, 6);
    ctx.fill();
    var y = y0 + pad, drawn = [];
    for (i = 0; i < texts.length; i++) {
      drawLabel(ctx, texts[i], x0 + pad, y, 'left', lines[i].warn ? WARN_TEXT : TEXT_COLOR);
      drawn.push(texts[i]);
      y += lh;
    }
    drawLabel(ctx, title, x0 + pad, y, 'left', TEXT_COLOR);
    drawn.push(title);
    y += lh;
    var R = niceCeil(Math.max(lim * 1.5, maxShown * 1.05)), rx = x0 + pad;
    var xc = rx + rampW * (CAUTION_FRAC * lim / R), xl = rx + rampW * (lim / R);
    ctx.fillStyle = COLORS.ok;
    ctx.fillRect(rx, y, xc - rx, rampH);
    ctx.fillStyle = COLORS.caution;
    ctx.fillRect(xc, y, xl - xc, rampH);
    ctx.fillStyle = COLORS.warn;
    ctx.fillRect(xl, y, rx + rampW - xl, rampH);
    ctx.strokeStyle = 'rgba(255,255,255,0.6)';
    ctx.lineWidth = 1;
    ctx.strokeRect(rx + 0.5, y + 0.5, rampW - 1, rampH - 1);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(xl, y - 3);
    ctx.lineTo(xl, y + rampH + 3);
    ctx.stroke();
    var ty = y + rampH + 3, lt = fmt1(lim) + ' 上限', lw = ctx.measureText(lt).width;
    var zt = '0', zw = ctx.measureText(zt).width, rt = fmt1(R), rw = ctx.measureText(rt).width;
    if (xl - lw / 2 > rx + zw + 4) { drawLabel(ctx, zt, rx, ty, 'left', TEXT_COLOR); drawn.push(zt); }
    drawLabel(ctx, lt, xl, ty, 'center', TEXT_COLOR);
    drawn.push(lt);
    if (xl + lw / 2 < rx + rampW - rw - 4) { drawLabel(ctx, rt, rx + rampW, ty, 'right', TEXT_COLOR); drawn.push(rt); }
    this._legend = drawn;
    this._legendBox = [x0, y0, boxW, boxH];
  };

  // ── 狀態／清除／測試掛勾 ──
  Wind.prototype.state = function () {
    var fields = {}, any = false, label = null, alive = 0, cells = 0, maxSpeed = null, self = this;
    var def = this.fields.get(DEFAULT_ID);
    this.fields.forEach(function (F, id) {
      var P = F.P, R = F.region || EMPTY_REGION;
      fields[id] = {
        n_particles: F.pt ? F.pt.n : 0, n_alive: F.alive || 0, max_speed: P.maxSpeed,
        extent: [P.latMin, P.lonMin, P.latMax, P.lonMax], mode: F.mode, label: F.label, height_m: F.h,
        group: F.group, group_label: F.groupLabel, n_requested: F.nReq, share: F.share, n_min: F.nMin, faint: F.faint,
        grid: [P.ni, P.nj], cells: P.nValid, visible_area_px: R.area, region_cells: R.nCells,
        last_step: { dx: F.stat.dx, dy: F.stat.dy, len: F.stat.len, n: F.stat.n },
        trail_segments: F.trailSegs || 0, trail_jumps: F.trailJumps || 0
      };
      if (!any) { label = F.label; }
      any = true;
      alive += F.alive || 0;
      cells += P.nValid;
      maxSpeed = maxSpeed === null ? P.maxSpeed : Math.max(maxSpeed, P.maxSpeed);
    });
    if (def) { label = def.label; }
    var arrowLayers = {}, nArrows = 0;
    this.arrowLayers.forEach(function (A, id) { arrowLayers[id] = A.items.length; nArrows += A.items.length; });
    return {
      field: any, label: label, arrows: nArrows, particles: alive,
      viewKey: this._lastKey === null ? this._viewKey() : this._lastKey, cells: cells, maxSpeed: maxSpeed,
      fields: fields, arrowLayers: arrowLayers, frames: this.frames,
      animating: !this._destroyed && this.opts.animate && this.opts.visible && this._raf !== 0 && this._hasContent(),
      legend: this._legend.slice(), legendBox: this._legendBox ? this._legendBox.slice() : null,
      arrowGeom: this._geom.map(function (g) {
        var c = {};
        for (var k in g) { if (Object.prototype.hasOwnProperty.call(g, k)) { c[k] = isRowLike(g[k]) ? g[k].slice() : g[k]; } }
        return c;
      }),
      options: this.options(), size: [self.w, self.h, self.dpr], destroyed: this._destroyed,
      mask: this._mask.map(function (r) { return r.slice(); }),
      trail: { mode: this._trail.mode, segments: this._trail.segments, jumps: this._trail.jumps,
               renderer: this._trail.renderer }
    };
  };

  Wind.prototype.destroy = function () {
    if (this._destroyed) { return; }
    this._stopLoop();
    this._destroyed = true;
    this.fields.clear();
    this.arrowLayers.clear();
    if (this.canvas.parentNode) { this.canvas.parentNode.removeChild(this.canvas); }
    if (this._gl) {                                          // 拖尾的 WebGL context 交還（一頁的 context 數有上限）
      try { var ext = this._gl.gl.getExtension('WEBGL_lose_context'); if (ext) { ext.loseContext(); } } catch (e) { /* 無 */ }
      this._gl = null;
    }
    this._legend = [];
    this._geom = [];
  };

  // 測試用：同步跑 n 幀（與 rAF 迴圈走同一條 _frame）
  Wind.prototype._step = function (n) {
    n = Math.max(1, num(n) | 0);
    for (var i = 0; i < n; i++) { this._frame(); }
    return this.state();
  };

  // 測試用：粒子與可見區細節
  Wind.prototype._debug = function (id) {
    var F = this.fields.get(id === undefined || id === null ? DEFAULT_ID : String(id));
    if (!F) { return null; }
    var pt = F.pt, R = F.region || EMPTY_REGION, H = pt ? pt.hist : null;
    return {
      n: pt ? pt.n : 0,
      lat: pt ? Array.from(pt.lat) : [], lon: pt ? Array.from(pt.lon) : [],
      age: pt ? Array.from(pt.age) : [], alive: pt ? Array.from(pt.alive) : [],
      hist_cnt: H ? Array.from(H.cnt) : null,
      region: { nCells: R.nCells, area: R.area, sub: R.sub ? R.sub.slice() : null, bbox: R.bbox ? R.bbox.slice() : null,
                lattice: F.lat ? F.lat.N : 0 }
    };
  };

  // 測試用：第 k 顆粒子的位置歷史（trailsFollowView），新的在前：[[lat, lon], …]；沒有歷史 ⇒ []
  Wind.prototype._history = function (id, k) {
    var F = this.fields.get(id === undefined || id === null ? DEFAULT_ID : String(id)), out = [];
    var pt = F ? F.pt : null, H = pt ? pt.hist : null;
    if (!H || !(k >= 0 && k < pt.n)) { return out; }
    for (var a = 0; a < H.cnt[k]; a++) {
      var j = H.head[k] - a;
      if (j < 0) { j += TRAIL_HIST; }
      out.push([H.lat[k * TRAIL_HIST + j], H.lon[k * TRAIL_HIST + j]]);
    }
    return out;
  };

  root.AeroWind = {
    VERSION: VERSION,
    COLORS: Object.freeze({ ok: COLORS.ok, caution: COLORS.caution, warn: COLORS.warn, unknown: COLORS.unknown }),
    CAUTION_FRAC: CAUTION_FRAC,
    DEFAULT_LIMIT: DEFAULT_LIMIT,
    create: function (host, adapter) { return new Wind(host, adapter); },
    dirToUV: dirToUV,
    uvToDir: uvToDir,
    interp: interp,
    prepare: prepare,
    speedColor: speedColor
  };
}(typeof window !== 'undefined' ? window : this));
