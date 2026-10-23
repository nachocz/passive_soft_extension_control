// 3D view (three.js): the UR10 drawn from its DH frames, the soft extension as a tube that
// follows the frames of the rod, the two tendons (exterior spans from the anchors a_i to the
// entry points b_i, interior paths drawn apart from the centerline for visibility), the
// frames {B} and {D}, the body points p_j, targets, obstacles, the fulcrum with its cone, the
// tip path, and the moving tip reference. Labels are HTML placed at projected points.
import * as THREE from '../../vendor/three/three.module.min.js';
import { OrbitControls } from '../../vendor/three/OrbitControls.js';
import { palette } from './theme.js';

const V = (p) => new THREE.Vector3(p[0], p[1], p[2]);
const RING = 14;                 // vertices around the tube
const LINK_RADIUS = [0.075, 0.062, 0.052, 0.045, 0.045, 0.045];

function cylinderBetween(mesh, a, b, radius) {
    const dir = new THREE.Vector3().subVectors(b, a), len = dir.length();
    mesh.visible = len > 1e-6;
    if (!mesh.visible) return;
    mesh.position.copy(a).addScaledVector(dir, 0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
    mesh.scale.set(radius, len, radius);
}

export class RobotScene {
    constructor(container, { trailCapacity = 8000, rodRadius = 0.009, view = 'ee' } = {}) {
        this.container = container;
        try {
            this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
        } catch {
            container.append(Object.assign(document.createElement('p'), { className: 'view3d-hint', textContent: 'WebGL is not available in this browser.' }));
            this.failed = true;
            return;
        }
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        container.prepend(this.renderer.domElement);
        this.labelLayer = Object.assign(document.createElement('div'), { className: 'view3d-labels' });
        container.append(this.labelLayer);
        this.labels = [];
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(32, 1, 0.01, 30);
        this.camera.up.set(0, 0, 1);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.12;
        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x50555e, 2.0));
        const sun = new THREE.DirectionalLight(0xffffff, 1.7);
        sun.position.set(-1.2, -1.6, 2.4);
        this.scene.add(sun);
        this.rodRadius = rodRadius;
        this.trailCapacity = trailCapacity;
        this.mat = {};
        this.build();
        this.dirty = true;
        new ResizeObserver(() => this.resize()).observe(container);
        this.controls.addEventListener('change', () => { this.dirty = true; });
        this.frameView(view);
        const loop = () => {
            if (this.disposed) return;
            if (this.controls.update() || this.dirty) { this.renderer.render(this.scene, this.camera); this.placeLabels(); this.dirty = false; }
            requestAnimationFrame(loop);
        };
        requestAnimationFrame(loop);
    }

    resize() {
        if (this.failed) return;
        const w = this.container.clientWidth, h = this.container.clientHeight;
        if (!w || !h) return;
        this.renderer.setSize(w, h, false);
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.dirty = true;
    }

    // ---------------------------------------------------------------- build
    material(key, color, opts = {}) {
        const m = new THREE.MeshLambertMaterial({ color: new THREE.Color(color), ...opts });
        this.mat[key] = m;
        return m;
    }

    build() {
        const p = palette();
        this.scene.background = new THREE.Color(p.scene);
        const unitCyl = new THREE.CylinderGeometry(1, 1, 1, 28);
        const sphere = new THREE.SphereGeometry(1, 20, 14);
        this.unitCyl = unitCyl; this.unitSphere = sphere;
        // floor grid below the workspace
        this.grid = new THREE.GridHelper(2.4, 24, 0x8a929c, 0x8a929c);
        this.grid.rotation.x = Math.PI / 2;
        this.grid.position.set(-0.4, 0, -0.06);
        this.grid.material.transparent = true;
        this.grid.material.opacity = 0.12;
        this.scene.add(this.grid);
        // arm: links between the DH origins, joint housings along z_{i-1}, flange at {B}
        // translucent, so that the anchors on the link axes and the tendons stay visible
        const armMat = this.material('arm', p.arm, { transparent: true, opacity: 0.5, depthWrite: false });
        const jointMat = this.material('joint', p.joint, { transparent: true, opacity: 0.62, depthWrite: false });
        this.links = LINK_RADIUS.map((r) => { const m = new THREE.Mesh(unitCyl, armMat); this.scene.add(m); return { m, r }; });
        this.housings = LINK_RADIUS.map((r) => { const m = new THREE.Mesh(unitCyl, jointMat); this.scene.add(m); return { m, r }; });
        this.pedestal = new THREE.Mesh(unitCyl, jointMat);
        this.scene.add(this.pedestal);
        this.flange = new THREE.Mesh(unitCyl, jointMat);
        this.scene.add(this.flange);
        // soft extension
        this.tube = this.makeTube(this.material('rod', p.rod));
        this.ghost = this.makeTube(this.material('ghost', p.rod, { transparent: true, opacity: 0.22, depthWrite: false }));
        this.ghost.mesh.visible = false;
        // tendons
        this.tendons = [p.tendon1, p.tendon2].map((c, i) => {
            const m = this.material(`tendon${i}`, c);
            const span = new THREE.Mesh(unitCyl, m), anchor = new THREE.Mesh(sphere, m), entry = new THREE.Mesh(sphere, m);
            anchor.scale.setScalar(0.011); entry.scale.setScalar(0.0045);
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 64), 3));
            const inside = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: new THREE.Color(c) }));
            inside.frustumCulled = false;
            this.scene.add(span, anchor, entry, inside);
            return { span, anchor, entry, inside };
        });
        // body points and tip
        this.bodyMat = this.material('body', '#ffffff');
        this.bodies = [];
        // frames {B}, {D}
        this.frames = ['B', 'D'].map(() => {
            const g = new THREE.Group();
            const axes = [p.axisX, p.axisY, p.axisZ].map((c, i) => {
                const m = new THREE.Mesh(unitCyl, this.material(`axis${i}`, c));
                g.add(m);
                return m;
            });
            this.scene.add(g);
            return { g, axes };
        });
        // targets, obstacles, fulcrum, trail, reference
        this.targetGroup = new THREE.Group(); this.scene.add(this.targetGroup);
        this.obstacleGroup = new THREE.Group(); this.scene.add(this.obstacleGroup);
        this.fulcrumGroup = new THREE.Group(); this.scene.add(this.fulcrumGroup);
        const trail = new THREE.BufferGeometry();
        this.trailPos = new Float32Array(this.trailCapacity * 3);
        trail.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3));
        trail.setDrawRange(0, 0);
        this.trail = new THREE.Line(trail, new THREE.LineBasicMaterial({ color: new THREE.Color(p.path) }));
        this.trail.frustumCulled = false;
        this.trailCount = 0;
        this.scene.add(this.trail);
        this.reference = new THREE.Mesh(sphere, this.material('reference', p.reference));
        this.reference.scale.setScalar(0.004);
        this.reference.visible = false;
        this.scene.add(this.reference);
        this.tip = new THREE.Mesh(sphere, this.material('tip', p.path));
        this.tip.scale.setScalar(0.0065);
        this.scene.add(this.tip);
    }

    makeTube(material, samples = 26) {
        const geometry = new THREE.BufferGeometry();
        const pos = new Float32Array(samples * RING * 3);
        geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        const index = [];
        for (let k = 0; k < samples - 1; k++) for (let j = 0; j < RING; j++) {
            const a = k * RING + j, b = k * RING + (j + 1) % RING, c = a + RING, d = b + RING;
            index.push(a, c, b, b, c, d);
        }
        geometry.setIndex(index);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.frustumCulled = false;
        this.scene.add(mesh);
        return { mesh, pos, samples };
    }

    // centerline points and rod rotations (world); radius in m
    updateTube(tube, points, rotations, radius) {
        const { pos } = tube;
        for (let k = 0; k < tube.samples; k++) {
            const p = points[Math.min(k, points.length - 1)], R = rotations[Math.min(k, rotations.length - 1)];
            for (let j = 0; j < RING; j++) {
                const a = 2 * Math.PI * j / RING, c = Math.cos(a) * radius, s = Math.sin(a) * radius;
                const i = 3 * (k * RING + j);
                pos[i] = p[0] + R[0][0] * c + R[0][1] * s;
                pos[i + 1] = p[1] + R[1][0] * c + R[1][1] * s;
                pos[i + 2] = p[2] + R[2][0] * c + R[2][1] * s;
            }
        }
        tube.mesh.geometry.attributes.position.needsUpdate = true;
        tube.mesh.geometry.computeVertexNormals();
    }

    // ---------------------------------------------------------------- state
    // state: {frames, anchors, entries, centerline, rotations (local), bodyPoints, R_B, p_B, p_D, R_D}
    setState(state, { tendonOffsets = null } = {}) {
        if (this.failed) return;
        const F = state.frames;
        for (let i = 0; i < 6; i++) {
            cylinderBetween(this.links[i].m, V(F[i].p), V(F[i + 1].p), this.links[i].r * 0.72);
            const z = new THREE.Vector3(F[i].R[0][2], F[i].R[1][2], F[i].R[2][2]);
            const c = V(F[i].p), r = this.housings[i].r;
            cylinderBetween(this.housings[i].m, c.clone().addScaledVector(z, -0.9 * r), c.clone().addScaledVector(z, 0.9 * r), r);
        }
        cylinderBetween(this.pedestal, new THREE.Vector3(0, 0, -0.06), new THREE.Vector3(0, 0, 0.02), 0.09);
        const zB = new THREE.Vector3(F[6].R[0][2], F[6].R[1][2], F[6].R[2][2]);
        cylinderBetween(this.flange, V(F[6].p).addScaledVector(zB, -0.012), V(F[6].p), 0.032);
        const world = state.rotations.map((R) => mul3(state.R_B, R));
        this.updateTube(this.tube, state.centerline, world, this.rodRadius);
        this.tendons.forEach((t, i) => {
            const a = V(state.anchors[i]), b = V(state.entries[i]);
            cylinderBetween(t.span, a, b, 0.0022);
            t.anchor.position.copy(a);
            t.entry.position.copy(b);
            const offset = tendonOffsets ? tendonOffsets[i] : null;
            const attr = t.inside.geometry.attributes.position, n = state.centerline.length;
            for (let k = 0; k < n; k++) {
                const p = state.centerline[k], R = world[k];
                const o = offset ? [offset[0] * 1.9, offset[1] * 1.9, 0] : [0, 0, 0];
                attr.setXYZ(k, p[0] + R[0][0] * o[0] + R[0][1] * o[1], p[1] + R[1][0] * o[0] + R[1][1] * o[1], p[2] + R[2][0] * o[0] + R[2][1] * o[1]);
            }
            t.inside.geometry.setDrawRange(0, n);
            attr.needsUpdate = true;
        });
        while (this.bodies.length < state.bodyPoints.length - 1) {
            const m = new THREE.Mesh(this.unitSphere, this.bodyMat);
            m.scale.setScalar(0.0062);
            this.scene.add(m);
            this.bodies.push(m);
        }
        this.bodies.forEach((m, k) => { m.position.copy(V(state.bodyPoints[k])); m.visible = k > 0; });
        this.tip.position.copy(V(state.p_D));
        this.setFrame(0, F[6].R, F[6].p);
        this.setFrame(1, state.R_D, state.p_D);
        this.state = state;
        this.dirty = true;
    }

    setFrame(k, R, p, length = 0.06) {
        const { axes } = this.frames[k];
        axes.forEach((m, i) => {
            const d = new THREE.Vector3(R[0][i], R[1][i], R[2][i]);
            cylinderBetween(m, V(p), V(p).addScaledVector(d, length), 0.0018);
        });
    }

    showFrames(flag) { this.frames.forEach((f) => { f.g.visible = flag; }); this.dirty = true; }

    // the soft extension transported at fixed shape (centerline and rotations in the world), or null
    setGhost(points, rotations) {
        if (this.failed) return;
        this.ghost.mesh.visible = !!points;
        if (points) this.updateTube(this.ghost, points, rotations, this.rodRadius * 0.98);
        this.dirty = true;
    }

    // ---------------------------------------------------------------- scenario
    setTargets(targets, active = -1, infeasible = [], candidate = null) {
        if (this.failed) return;
        this.targetGroup.clear();
        this.activeRing = null;
        if (candidate) {
            const m = new THREE.Mesh(this.unitSphere, new THREE.MeshBasicMaterial({ color: new THREE.Color(palette().reference), wireframe: true }));
            m.scale.setScalar(0.007);
            m.position.copy(V(candidate));
            this.targetGroup.add(m);
        }
        this.targetLabels = (targets || []).map((p, k) => {
            const isActive = k === active;
            const m = new THREE.Mesh(this.unitSphere, new THREE.MeshLambertMaterial({ color: new THREE.Color(palette().target),
                transparent: !isActive, opacity: isActive ? 1 : 0.45 }));
            m.scale.setScalar(isActive ? 0.0055 : 0.0042);
            m.position.copy(V(p));
            this.targetGroup.add(m);
            if (isActive) {
                const ring = new THREE.Mesh(new THREE.TorusGeometry(0.011, 0.0012, 8, 32), new THREE.MeshBasicMaterial({ color: new THREE.Color(palette().reference) }));
                ring.position.copy(V(p));
                ring.lookAt(this.camera.position);
                this.activeRing = ring;
                this.targetGroup.add(ring);
            }
            return { p, text: `T${k + 1}${infeasible.includes(k + 1) ? '*' : ''}`, cls: isActive ? 'active' : '' };
        });
        if (candidate) this.targetLabels.push({ p: candidate, text: 'new', cls: 'active' });
        this.dirty = true;
    }

    setObstacles(obstacles) {
        if (this.failed) return;
        this.obstacleGroup.clear();
        const color = new THREE.Color(palette().obstacle);
        for (const o of obstacles || []) {
            const material = new THREE.MeshLambertMaterial({ color, transparent: true, opacity: 0.28, depthWrite: false });
            let m;
            if (o.kind === 'sphere') {
                m = new THREE.Mesh(this.unitSphere, material);
                m.scale.setScalar(Array.isArray(o.size) ? o.size[0] : o.size);
            } else {
                const s = Array.isArray(o.size) ? o.size : [o.size, o.size, o.size];
                m = new THREE.Mesh(new THREE.BoxGeometry(s[0], s[1], s[2]), material);
                const edges = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 }));
                m.add(edges);
            }
            m.position.copy(V(o.center));
            this.obstacleGroup.add(m);
        }
        this.dirty = true;
    }

    // fulcrum point c, the cone of half-angle theta about the aperture axis, and the aperture rim
    setFulcrum(c, { halfAngle = 35 * Math.PI / 180, apertureRadius = 0, height = 0.2 } = {}) {
        if (this.failed) return;
        this.fulcrumGroup.clear();
        this.fulcrumPoint = c;
        if (!c) { this.dirty = true; return; }
        const color = new THREE.Color(palette().fulcrum);
        const dot = new THREE.Mesh(this.unitSphere, new THREE.MeshBasicMaterial({ color }));
        dot.scale.setScalar(0.005);
        dot.position.copy(V(c));
        this.fulcrumGroup.add(dot);
        const pts = [];
        const r = height * Math.tan(halfAngle);
        for (let k = 0; k <= 64; k++) {
            const a = 2 * Math.PI * k / 64;
            pts.push(new THREE.Vector3(c[0] + r * Math.cos(a), c[1] + r * Math.sin(a), c[2] + height));
        }
        const rim = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color, dashSize: 0.012, gapSize: 0.008 }));
        rim.computeLineDistances();
        this.fulcrumGroup.add(rim);
        for (let k = 0; k < 4; k++) {
            const a = Math.PI / 4 + k * Math.PI / 2;
            const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([V(c), new THREE.Vector3(c[0] + r * Math.cos(a), c[1] + r * Math.sin(a), c[2] + height)]),
                new THREE.LineDashedMaterial({ color, dashSize: 0.012, gapSize: 0.008 }));
            line.computeLineDistances();
            this.fulcrumGroup.add(line);
        }
        const plate = new THREE.Mesh(new THREE.RingGeometry(Math.max(apertureRadius, 0.012), 0.09, 48),
            new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }));
        plate.position.copy(V(c));
        this.fulcrumGroup.add(plate);
        this.dirty = true;
    }

    setReference(p) {
        if (this.failed) return;
        this.reference.visible = !!p;
        if (p) this.reference.position.copy(V(p));
        this.dirty = true;
    }

    clearTrail() { if (this.failed) return; this.trailCount = 0; this.trail.geometry.setDrawRange(0, 0); this.dirty = true; }

    pushTrail(p) {
        if (this.failed) return;
        const cap = this.trailCapacity;
        if (this.trailCount === cap) { this.trailPos.copyWithin(0, 3 * Math.floor(cap / 4)); this.trailCount = cap - Math.floor(cap / 4); }
        const i = this.trailCount++;
        this.trailPos[3 * i] = p[0]; this.trailPos[3 * i + 1] = p[1]; this.trailPos[3 * i + 2] = p[2];
        this.trail.geometry.attributes.position.needsUpdate = true;
        this.trail.geometry.setDrawRange(0, this.trailCount);
        this.dirty = true;
    }

    // ---------------------------------------------------------------- labels and camera
    setLabels(items) { this.extraLabels = items; this.dirty = true; }

    placeLabels() {
        const items = [...(this.targetLabels || []), ...(this.extraLabels || [])];
        while (this.labels.length < items.length) {
            const el = document.createElement('span');
            this.labelLayer.append(el);
            this.labels.push(el);
        }
        const w = this.container.clientWidth, h = this.container.clientHeight, v = new THREE.Vector3();
        this.labels.forEach((el, k) => {
            const item = items[k];
            if (!item) { el.style.display = 'none'; return; }
            v.set(item.p[0], item.p[1], item.p[2]).project(this.camera);
            if (v.z > 1) { el.style.display = 'none'; return; }
            el.style.display = '';
            el.className = `label3d ${item.cls || ''}`;
            el.textContent = item.text;
            if (item.color) el.style.color = item.color; else el.style.removeProperty('color');
            el.style.transform = `translate(${((v.x + 1) / 2 * w + (item.dx ?? 7)).toFixed(1)}px, ${((1 - v.y) / 2 * h + (item.dy ?? -7)).toFixed(1)}px)`;
        });
        if (this.activeRing) this.activeRing.lookAt(this.camera.position);
    }

    // 'ee': the soft extension and its workspace; 'arm': the whole manipulator
    frameView(kind = 'ee') {
        if (kind === 'arm') {
            this.camera.position.set(0.25, -2.35, 1.05);
            this.controls.target.set(-0.42, -0.05, 0.33);
        } else {
            this.camera.position.set(-0.02, -0.98, 0.62);
            this.controls.target.set(-0.74, -0.08, 0.3);
        }
        this.controls.update();
        this.dirty = true;
    }

    refreshTheme() {
        if (this.failed) return;
        const p = palette();
        this.scene.background = new THREE.Color(p.scene);
        const set = (key, c) => this.mat[key]?.color.set(c);
        set('arm', p.arm); set('joint', p.joint); set('rod', p.rod); set('ghost', p.rod);
        set('tendon0', p.tendon1); set('tendon1', p.tendon2); set('reference', p.reference); set('tip', p.path);
        set('axis0', p.axisX); set('axis1', p.axisY); set('axis2', p.axisZ);
        this.tendons[0].inside.material.color.set(p.tendon1);
        this.tendons[1].inside.material.color.set(p.tendon2);
        this.trail.material.color.set(p.path);
        this.dirty = true;
    }

    snapshot() { return this.failed ? null : this.renderer.domElement; }
}

function mul3(A, B) {
    const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    return C;
}
