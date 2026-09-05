// Copyright (c) 2026 The EBiM Benchmark Contributors
// SPDX-License-Identifier: Apache-2.0
// 3D 数字孪生:解析完整 URDF(带 visual origin)→ Three.js FK → 加载真实 glb mesh。
// glb 由 franka .dae 抽面转成(小且快);GLTFLoader 不自动转轴,配 URDF visual
// origin 即可正确组装。整机 ROS Z-up → Three Y-up(root 绕 X -90)。
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

function quatFromRPY(r, p, y) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), r);
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), p);
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), y);
  return qz.multiply(qy).multiply(qx);
}
function nums(s) { return (s || "0 0 0").trim().split(/\s+/).map(Number); }

export class Twin {
  constructor(canvas) {
    const w = canvas.clientWidth || 600, h = canvas.clientHeight || 500;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0f16);
    this.cam = new THREE.PerspectiveCamera(50, w / h, 0.01, 100);
    this.cam.position.set(1.6, 1.2, 1.6);
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    } catch (e) {
      try {   // 有些环境抗锯齿上下文创建会失败,降级再试一次
        this.renderer = new THREE.WebGLRenderer({
          canvas, antialias: false, powerPreference: "low-power" });
      } catch (e2) {
        throw new Error(
          "WebGL 上下文创建失败。多半是浏览器关闭了硬件加速:" +
          "chrome://settings 搜「硬件加速」打开后重启浏览器;" +
          "chrome://gpu 查看 WebGL2 是否 Enabled。");
      }
    }
    this.renderer.setPixelRatio(devicePixelRatio);
    this.renderer.setSize(w, h, false);
    this.ctrl = new OrbitControls(this.cam, canvas);
    this.ctrl.target.set(0, 0.7, 0);
    this.scene.add(new THREE.HemisphereLight(0xcfe2ff, 0x1a2230, 1.3));
    const d = new THREE.DirectionalLight(0xffffff, 1.4); d.position.set(3, 6, 4);
    this.scene.add(d);
    const d2 = new THREE.DirectionalLight(0x88aaff, 0.5); d2.position.set(-3, 2, -3);
    this.scene.add(d2);
    this.scene.add(new THREE.GridHelper(6, 24, 0x1e2a3a, 0x141d29));
    this.links = {};
    this.movable = {};
    this._raf();
    addEventListener("resize", () => this._resize(canvas));
  }

  async load(url) {
    const xml = new DOMParser().parseFromString(
      await (await fetch(url)).text(), "application/xml");

    const joints = [...xml.querySelectorAll("joint")].map((j) => {
      const o = j.querySelector("origin"), ax = j.querySelector("axis"),
        lim = j.querySelector("limit");
      return {
        name: j.getAttribute("name"), type: j.getAttribute("type"),
        parent: j.querySelector("parent").getAttribute("link"),
        child: j.querySelector("child").getAttribute("link"),
        xyz: nums(o && o.getAttribute("xyz")),
        rpy: nums(o && o.getAttribute("rpy")),
        axis: nums(ax ? ax.getAttribute("xyz") : "1 0 0"),
        lower: lim ? Number(lim.getAttribute("lower")) : -3.14,
        upper: lim ? Number(lim.getAttribute("upper")) : 3.14,
      };
    });

    const visuals = {};
    [...xml.querySelectorAll("link")].forEach((link) => {
      const name = link.getAttribute("name");
      this.links[name] = new THREE.Group();
      const vis = link.querySelector("visual");
      const mesh = vis && vis.querySelector("geometry mesh");
      if (mesh) {
        const o = vis.querySelector("origin");
        visuals[name] = { file: mesh.getAttribute("filename"),
          xyz: nums(o && o.getAttribute("xyz")),
          rpy: nums(o && o.getAttribute("rpy")) };
      }
    });

    const children = new Set(joints.map((j) => j.child));
    joints.forEach((j) => {
      const origin = new THREE.Group();
      origin.position.set(...j.xyz);
      origin.quaternion.copy(quatFromRPY(...j.rpy));
      const motion = new THREE.Group();
      origin.add(motion);
      if (!this.links[j.child]) this.links[j.child] = new THREE.Group();
      motion.add(this.links[j.child]);
      if (!this.links[j.parent]) this.links[j.parent] = new THREE.Group();
      this.links[j.parent].add(origin);
      if (["revolute", "continuous", "prismatic"].includes(j.type))
        this.movable[j.name] = { motion, axis: new THREE.Vector3(...j.axis),
          type: j.type, lower: j.lower, upper: j.upper };
    });

    // 所有无父的 link 都是根(正常是一个);整机 ROS Z-up → Three Y-up
    [...Object.keys(this.links)].filter((n) => !children.has(n)).forEach((r) => {
      this.links[r].rotation.x = -Math.PI / 2;
      this.scene.add(this.links[r]);
    });
    this._loadMeshes(visuals);
    this._fit();
  }

  _loadMeshes(visuals) {
    const loader = new GLTFLoader();
    Object.entries(visuals).forEach(([name, v]) => {
      const url = v.file
        .replace("package://franka_description/", "/assets/franka_description/")
        .replace(/\.dae$/i, ".glb");
      const inner = new THREE.Group();
      inner.position.set(...v.xyz);
      inner.quaternion.copy(quatFromRPY(...v.rpy));
      this.links[name].add(inner);
      loader.load(url, (g) => {
        g.scene.traverse((o) => {
          if (o.isMesh && o.material) {
            const m = o.material;
            m.side = THREE.DoubleSide;
            // 金属材质无环境贴图会渲染成黑;降 metalness 让漫反射色(白)显出来
            if ("metalness" in m) m.metalness = Math.min(m.metalness ?? 0.5, 0.2);
            if ("roughness" in m) m.roughness = Math.max(m.roughness ?? 0.5, 0.55);
            m.needsUpdate = true;
          }
        });
        inner.add(g.scene);
        this._fit();
      }, undefined, () => console.warn("[twin] mesh 加载失败:", url));
    });
  }

  setJoints(map) {
    for (const [name, val] of Object.entries(map || {})) {
      const m = this.movable[name];
      if (!m) continue;
      if (m.type === "prismatic")
        m.motion.position.copy(m.axis.clone().multiplyScalar(val));
      else m.motion.quaternion.setFromAxisAngle(m.axis, val);
    }
  }

  _fit() {
    const box = new THREE.Box3().setFromObject(this.scene);
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3()).length();
    // 某些抽面 glb 可能含退化/NaN 几何 → bbox 非有限 → 相机 NaN 会导致全黑;跳过
    if (!Number.isFinite(c.x) || !Number.isFinite(c.y) ||
        !Number.isFinite(c.z) || !Number.isFinite(size) || size <= 0) return;
    this.ctrl.target.copy(c);
    this.cam.position.set(c.x + size * 0.5, c.y + size * 0.35, c.z + size * 0.5);
  }
  _resize(canvas) {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    this.cam.aspect = w / h; this.cam.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }
  _raf() {
    const loop = () => {
      // 画布 CSS 尺寸变化(切视图/等高弹性布局)时同步渲染缓冲
      const c = this.renderer.domElement;
      const w = c.clientWidth, h = c.clientHeight;
      if (w && h && (c.width !== Math.round(w * devicePixelRatio) ||
                     c.height !== Math.round(h * devicePixelRatio))) {
        this.cam.aspect = w / h; this.cam.updateProjectionMatrix();
        this.renderer.setSize(w, h, false);
      }
      this.ctrl.update();
      this.renderer.render(this.scene, this.cam);
      requestAnimationFrame(loop); };
    loop();
  }
}
