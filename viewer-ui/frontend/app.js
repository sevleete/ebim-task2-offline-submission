// Copyright (c) 2026 The EBiM Benchmark Contributors
// SPDX-License-Identifier: Apache-2.0
// viewer-ui 前端:多视图 + 实时控制(关节滑条 / 3D 孪生 / 控制动作)。
"use strict";
const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);
const PI = Math.PI;
const LEFT_J = [1, 2, 3, 4, 5, 6, 7].map((i) => `left_fr3v2_joint${i}`);
const RIGHT_J = [1, 2, 3, 4, 5, 6, 7].map((i) => `right_fr3v2_joint${i}`);
let view = "overview", camMounted = false, lastCams = {};
let twinMode = "real";
const simVals = {}, realMap = {};

/* 导航 */
$$(".nav-item").forEach((el) => el.addEventListener("click", () => {
  view = el.dataset.view;
  $$(".nav-item").forEach((n) => n.classList.toggle("active", n === el));
  $$(".view").forEach((v) => v.classList.toggle("active", v.id === "view-" + view));
  if (view === "cameras") mountCams(lastCams); else unmountCams();
  if (view === "control") buildSliders();
}));

function setConn(ok, mode) {
  const c = $("#conn");
  c.textContent = ok ? "● 在线" : "● 断线";
  c.className = "pill " + (ok ? "pill-on" : "pill-off");
  if (mode !== undefined) $("#mode-tag").textContent = mode ? "模式:" + mode : "";
}
function renderAlerts(a) {
  const box = $("#alerts");
  a = a || [];
  const sig = JSON.stringify(a);
  if (box._sig === sig) return;          // 内容没变不重绘,保住滚动位置
  box._sig = sig;
  if (!a.length) { box.innerHTML = ""; box.classList.remove("has"); return; }
  box.classList.add("has");
  const ne = a.filter((x) => x.level === "error").length;
  box.innerHTML =
    `<div class="sa-head"><span class="dot"></span>WARNING · ${a.length}` +
    (ne ? `<span class="sa-err">${ne}</span>` : "") + `</div>` +
    `<div class="sa-list">` + a.map((x) =>
      `<div class="sa-item ${x.level === "error" ? "err" : "warn"}">` +
      `<span class="d"></span><span>${x.msg}</span></div>`).join("") + `</div>`;
}

/* ---- 概览 ---- */
const kv = (k, v, c) => `<div class="kv"><span class="k">${k}</span><span class="v ${c || ""}">${v}</span></div>`;
function tile(l, v, s, c) {
  return `<div class="tile"><div class="tl">${l}</div><div class="tv ${c || ""}">${v}</div><div class="ts ${c || ""}">${s || ""}</div></div>`;
}
function renderOverview(ssh, ros) {
  const H = ssh.hosts || {}, S = ssh.services || {}, A = ssh.arm || {}, C = ros.cameras || {};
  const svcUp = (s) => s.running && s.healthy !== false;   // 进程在且未判不健康
  const hu = Object.values(H).filter((h) => h.reachable).length;
  const su = Object.values(S).filter(svcUp).length;
  const sbad = Object.values(S).filter((s) => s.running && s.healthy === false).length;
  const cu = Object.values(C).filter((c) => c.connected).length;
  const at = (side) => { const a = A[side]; const lab = (side === "left" ? "左" : "右") + "臂";
    return (!a || !a.ok) ? tile(lab, "—", "无状态") : tile(lab, a.mode_name, a.locked ? "抱死/报错" : "正常", a.locked ? "err" : "ok"); };
  $("#tiles").innerHTML = tile("主机", `${hu}/${Object.keys(H).length}`, hu === Object.keys(H).length ? "全在线" : "有离线", hu === Object.keys(H).length ? "ok" : "err")
    + tile("服务", `${su}/${Object.keys(S).length}`, sbad ? `${sbad} 异常` : "可用", sbad ? "warn" : (su ? "ok" : "")) + at("left") + at("right")
    + tile("相机", `${cu}/${Object.keys(C).length || 0}`, "有画面", cu ? "ok" : "warn");
  // 主机 + 服务 合并:一台主机一列,IP + 在线徽标 + 该机服务灯(三态)
  $("#hostsvc").innerHTML = Object.entries(H).map(([hn, h]) => {
    const svcs = Object.entries(S).filter(([, s]) => s.host === hn);
    const up = svcs.filter(([, s]) => svcUp(s)).length;
    return `<div class="hostcol ${h.reachable ? "up" : "down"}">` +
      `<div class="hc-head"><span class="hc-led"></span>` +
      `<div class="hc-title"><b>${hn}</b><span class="hc-ip">${h.addr || ""}</span></div>` +
      `<span class="hc-badge">${h.reachable ? "在线" : "离线"}</span></div>` +
      `<div class="hc-stat">服务 ${up}/${svcs.length} 可用</div>` +
      `<div class="hc-svcs">${svcs.map(([n, s]) => {
        const cls = !s.running ? "down" : (s.healthy === false ? "bad" : "up");
        const tag = (s.running && s.healthy === false) ? '<span class="sx">异常</span>' : "";
        return `<div class="svc ${cls}"><span class="b"></span>${n}${tag}</div>`;
      }).join("")}</div>` +
      `</div>`;
  }).join("") || `<div class="ph-body">等待 SSH 状态首轮轮询…</div>`;
  $("#controllers").innerHTML = Object.entries(ssh.controllers || {}).map(([n, c]) => { const k = (c.active || []).length; return kv(n + " 臂", c.ok ? k + " active" : "查询失败", c.ok && k ? "ok" : "warn"); }).join("") || kv("控制器", "无数据", "warn");
}

/* ---- 相机 ---- */
function mountCams(cams) {
  const box = $("#cameras"), keys = Object.keys(cams || {});
  if (!camMounted && keys.length) {
    box.innerHTML = keys.map((k) => `<div class="cam" id="cam-${k}" onclick="this.classList.toggle('zoom')"><img src="/stream?cam=${k}" alt="${k}"><div class="ov"><span class="cn">${k}</span><span class="cf" id="cf-${k}">—</span></div></div>`).join("");
    camMounted = true;
  }
  keys.forEach((k) => { const el = $("#cam-" + k); if (el) el.classList.toggle("dead", !cams[k].connected); const cf = $("#cf-" + k); if (cf) cf.textContent = `${cams[k].res} · ${cams[k].rate} Hz`; });
}
function unmountCams() { if (camMounted) { $("#cameras").innerHTML = ""; camMounted = false; } }

/* ---- 实时控制:双臂滑条(上下都展开,无 tab) ---- */
function jlimit(n) { const m = window.__twin && window.__twin.movable[n]; return m ? [m.lower, m.upper] : [-3.14, 3.14]; }
function buildSliders() {
  const mk = (names) => names.map((n, i) => {
    const [lo, hi] = jlimit(n);
    const v = simVals[n] != null ? simVals[n] : (realMap[n] || 0);
    return `<div class="srow"><span class="sn">J${i + 1}</span><input type="range" min="${lo}" max="${hi}" step="0.001" value="${v}" data-j="${n}"><span class="sv" id="sv-${n}">${(+v).toFixed(3)}</span></div>`;
  }).join("");
  $("#sliders-left").innerHTML = mk(LEFT_J);
  $("#sliders-right").innerHTML = mk(RIGHT_J);
  $$(".sliders input[data-j]").forEach((inp) => inp.addEventListener("input", (e) => {
    const n = e.target.dataset.j; simVals[n] = parseFloat(e.target.value);
    $("#sv-" + n).textContent = simVals[n].toFixed(3);
    setMode("sim"); applyTwin();
  }));
}
function setMode(m) {
  twinMode = m;
  $("#mode-real").classList.toggle("active", m === "real");
  $("#mode-sim").classList.toggle("active", m === "sim");
}
function applyTwin() {
  if (!window.__twin) return;
  if (twinMode === "sim") window.__twin.setJoints(simVals);
  else window.__twin.setJoints(realMap);
}

/* ---- 控制动作 ---- */
async function doControl(act, params) {
  const t = $("#ctrl-toast");
  $$(".btn").forEach((b) => b.disabled = true);
  t.className = "toast show"; t.textContent = "执行中…";
  try {
    const d = await (await fetch("/api/control", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(Object.assign({ action: act }, params || {})) })).json();
    t.className = "toast show " + (d.ok ? "ok" : "err");
    t.textContent = (d.ok ? "✓ " : "✗ ") + (d.output || act);
  } catch (e) { t.className = "toast show err"; t.textContent = "✗ " + e; }
  finally { $$(".btn").forEach((b) => b.disabled = false); }
}
document.addEventListener("click", (e) => {
  const b = e.target.closest(".btn[data-act]"); if (!b) return;
  const act = b.dataset.act;
  if (act === "spine") doControl("spine", { value: parseFloat($("#spine-val").value) });
  else if (act === "down_arms") { if (confirm("停机械臂控制器服务?")) doControl("down_arms"); }
  else doControl(act);
});
function sendArm(side) {
  const names = side === "left" ? LEFT_J : RIGHT_J;
  const joints = names.map((n) => simVals[n] != null ? simVals[n] : (realMap[n] || 0));
  if (confirm(`发送${side === "left" ? "左" : "右"}臂到当前滑条位形?(PTP,平滑)`))
    doControl(side === "left" ? "arm_left" : "arm_right", { joints });
}
$("#send-left").addEventListener("click", () => sendArm("left"));
$("#send-right").addEventListener("click", () => sendArm("right"));
$("#spd").addEventListener("input", (e) => $("#spd-v").textContent = e.target.value + "%");
$("#mode-real").addEventListener("click", () => { setMode("real"); if (view === "control") buildSliders(); applyTwin(); });
$("#mode-sim").addEventListener("click", () => { setMode("sim"); Object.assign(simVals, realMap); if (view === "control") buildSliders(); applyTwin(); });

/* ---- 状态读数 ---- */
function renderState(ssh, ros) {
  const A = ssh.arm || {}, od = ros.odom || {};
  let h = "";
  ["left", "right"].forEach((s) => { const a = A[s]; h += kv((s === "left" ? "左" : "右") + "臂模式", a && a.ok ? a.mode_name : "—", a && a.locked ? "bad" : (a && a.ok ? "ok" : "")); });
  h += kv("底盘", od.ok ? (od.moving ? "移动中" : "静止") : "—", od.moving ? "ok" : "");
  const sp = ssh.spine;
  h += kv("升降柱", sp && sp.ok ? sp.position.toFixed(3) + " m" : "—", sp && sp.ok ? "ok" : "");
  Object.entries(ros.gello || {}).forEach(([k, g]) => h += kv("GELLO " + k, g.ok ? g.rate + " Hz" : "未发布", g.ok ? "ok" : ""));
  $("#statebox").innerHTML = h;
}

/* ---- 主循环 ---- */
async function tick() {
  try {
    const s = await (await fetch("/api/status", { cache: "no-store" })).json();
    const ssh = s.ssh || {}, ros = s.ros || {};
    lastCams = ros.cameras || {};
    setConn(true, s.demo ? "DEMO" : "");
    renderAlerts(s.alerts);
    renderOverview(ssh, ros);
    if (view === "control") renderState(ssh, ros);
    if (view === "cameras") mountCams(lastCams);
  } catch (e) { setConn(false); }
}

/* ---- 快循环(10Hz):关节 + spine → 孪生/滑条,不碰其余 DOM ---- */
let fastBusy = false;
async function tickFast() {
  if (fastBusy) return;             // 上一个请求还在路上就跳过,防积压
  fastBusy = true;
  try {
    const d = await (await fetch("/api/joints", { cache: "no-store" })).json();
    const j = d.joints || {};
    LEFT_J.forEach((n, i) => { const v = j.left && j.left[i]; if (v != null) realMap[n] = v; });
    RIGHT_J.forEach((n, i) => { const v = j.right && j.right[i]; if (v != null) realMap[n] = v; });
    if (d.spine != null) realMap["franka_spine_vertical_joint"] = d.spine;
    if (view === "control" && twinMode === "real") {
      applyTwin();
      LEFT_J.concat(RIGHT_J).forEach((n) => { const inp = $(`.sliders input[data-j="${n}"]`); const sv = $("#sv-" + n); if (inp) { inp.value = realMap[n]; if (sv) sv.textContent = (realMap[n] || 0).toFixed(3); } });
    }
  } catch (e) { /* 断线由 tick() 的 setConn 提示 */ }
  finally { fastBusy = false; }
}
setInterval(() => $("#clock").textContent = new Date().toTimeString().slice(0, 8), 1000);
setInterval(tick, 1000); tick();
setInterval(tickFast, 100); tickFast();
// 3D 看门狗:6 秒还没初始化成功就把原因亮出来(模块/URDF加载失败时 window.__twin 不会被赋值)
setTimeout(() => {
  const el = $("#twin-err");
  if (el && !window.__twin && !el.textContent)
    el.textContent = "3D 未初始化:three.js 模块或 URDF 加载失败(F12 控制台看红色报错)";
}, 6000);
