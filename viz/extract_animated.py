#!/usr/bin/env python3
"""Extract point-cloud frames from a rosbag and generate a lightweight Three.js animation."""
import sys
import struct
import json
import numpy as np


def read_points_from_msg(msg, max_points=100000):
    """Parse PointCloud2 message into numpy array of [x, y, z]."""
    field_map = {}
    for f in msg.fields:
        field_map[f.name] = (f.offset, f.datatype)

    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return np.array([])

    points = []
    point_step = msg.point_step
    data = msg.data
    n_points = min(msg.width * msg.height, max_points * 2)  # read more, filter later

    for i in range(n_points):
        offset = i * point_step
        x = struct.unpack_from('f', data, offset + field_map['x'][0])[0]
        y = struct.unpack_from('f', data, offset + field_map['y'][0])[0]
        z = struct.unpack_from('f', data, offset + field_map['z'][0])[0]

        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
            continue
        points.append([x, y, z])

    pts = np.array(points) if points else np.array([])
    if len(pts) > max_points:
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts = pts[idx]
    return pts


def generate_animated_html(all_frames, output_path, fps=5):
    """Generate Three.js animated visualization."""
    # Compute global Z range across all frames for consistent coloring
    global_min_z = min(pts[:, 2].min() for pts in all_frames)
    global_max_z = max(pts[:, 2].max() for pts in all_frames)

    # Convert frames to compact format: list of list of [x,y,z]
    frames_data = []
    for pts in all_frames:
        frames_data.append(pts.round(3).tolist())

    html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Point Cloud Animation</title>
<style>
body { margin: 0; overflow: hidden; background: #1a1a2e; font-family: monospace; }
#info { position: absolute; top: 10px; left: 10px; color: #eee; z-index: 100; font-size: 14px; }
#controls { position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%);
            color: #eee; z-index: 100; text-align: center; }
button { padding: 8px 16px; cursor: pointer; background: #16213e; color: #eee;
         border: 1px solid #555; margin: 0 4px; font-size: 14px; }
button:hover { background: #0f3460; }
#slider { width: 400px; margin: 10px; }
</style>
</head><body>
<div id="info">Frame: <span id="frameNum">0</span> / <span id="totalFrames">0</span>
 | Points: <span id="ptCount">0</span> | <span id="playState">Playing</span></div>
<div id="controls">
    <button id="prevBtn">Prev</button>
    <button id="playBtn">Pause</button>
    <button id="nextBtn">Next</button>
    <br>
    <input type="range" id="slider" min="0" max="0" value="0">
    <br>
    Speed: <input type="range" id="speedSlider" min="1" max="20" value=\"""" + str(fps) + """">
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const FRAMES = """ + json.dumps(frames_data) + """;
const GLOBAL_MIN_Z = """ + str(round(float(global_min_z), 3)) + """;
const GLOBAL_MAX_Z = """ + str(round(float(global_max_z), 3)) + """;
const GLOBAL_RANGE_Z = GLOBAL_MAX_Z - GLOBAL_MIN_Z || 1;

let scene, camera, renderer, controls, pointCloud, geometry;
let currentFrame = 0;
let playing = true;
let fps = """ + str(fps) + """;
let lastTime = 0;

document.getElementById('totalFrames').textContent = FRAMES.length - 1;
document.getElementById('slider').max = FRAMES.length - 1;

function init() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.1, 1000);
    camera.position.set(20, 20, 20);

    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x1a1a2e);
    document.body.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    scene.add(new THREE.GridHelper(50, 50, 0x333333, 0x222222));
    scene.add(new THREE.AxesHelper(5));

    geometry = new THREE.BufferGeometry();
    const mat = new THREE.PointsMaterial({size: 0.05, vertexColors: true, sizeAttenuation: true});
    pointCloud = new THREE.Points(geometry, mat);
    scene.add(pointCloud);

    updateFrame(0);

    // Controls
    document.getElementById('playBtn').onclick = () => {
        playing = !playing;
        document.getElementById('playBtn').textContent = playing ? 'Pause' : 'Play';
        document.getElementById('playState').textContent = playing ? 'Playing' : 'Paused';
    };
    document.getElementById('prevBtn').onclick = () => {
        playing = false;
        document.getElementById('playBtn').textContent = 'Play';
        document.getElementById('playState').textContent = 'Paused';
        updateFrame((currentFrame - 1 + FRAMES.length) % FRAMES.length);
    };
    document.getElementById('nextBtn').onclick = () => {
        playing = false;
        document.getElementById('playBtn').textContent = 'Play';
        document.getElementById('playState').textContent = 'Paused';
        updateFrame((currentFrame + 1) % FRAMES.length);
    };
    document.getElementById('slider').oninput = (e) => {
        updateFrame(parseInt(e.target.value));
    };
    document.getElementById('speedSlider').oninput = (e) => {
        fps = parseInt(e.target.value);
    };

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    animate(0);
}

function updateFrame(idx) {
    currentFrame = idx;
    const pts = FRAMES[idx];

    const positions = new Float32Array(pts.length * 3);
    const colors = new Float32Array(pts.length * 3);

    pts.forEach((p, j) => {
        positions[j*3] = p[0];
        positions[j*3+1] = p[2];
        positions[j*3+2] = -p[1];

        const t = (p[2] - GLOBAL_MIN_Z) / GLOBAL_RANGE_Z;
        const c = new THREE.Color().setHSL(0.6 - t * 0.6, 0.9, 0.5);
        colors[j*3] = c.r;
        colors[j*3+1] = c.g;
        colors[j*3+2] = c.b;
    });

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.attributes.position.needsUpdate = true;
    geometry.attributes.color.needsUpdate = true;
    geometry.computeBoundingSphere();

    document.getElementById('frameNum').textContent = idx;
    document.getElementById('ptCount').textContent = pts.length;
    document.getElementById('slider').value = idx;
}

function animate(time) {
    requestAnimationFrame(animate);

    if (playing && time - lastTime > 1000 / fps) {
        lastTime = time;
        updateFrame((currentFrame + 1) % FRAMES.length);
    }

    controls.update();
    renderer.render(scene, camera);
}

init();
</script></body></html>"""

    with open(output_path, 'w') as f:
        f.write(html)
    print(f"Saved {len(all_frames)} frames to {output_path}")


if __name__ == '__main__':
    import rosbag

    bag_path = sys.argv[1] if len(sys.argv) > 1 else '/ws/bags/example.bag'
    topic = sys.argv[2] if len(sys.argv) > 2 else '/robot/livox_front/points'
    output = sys.argv[3] if len(sys.argv) > 3 else '/ws/src/lidar_preprocessing_pipeline/viz/points_animated.html'
    max_frames = int(sys.argv[4]) if len(sys.argv) > 4 else 100

    print(f"Reading {bag_path} topic {topic} (sampling {max_frames} frames across full bag)...")
    bag = rosbag.Bag(bag_path)

    # First pass: count total frames
    total = bag.get_message_count(topic_filters=[topic])
    step = max(1, total // max_frames)
    print(f"  Total frames: {total}, sampling every {step}th frame")

    # Second pass: extract evenly sampled frames
    frames = []
    idx = 0
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        if idx % step == 0:
            pts = read_points_from_msg(msg, max_points=100000)
            if len(pts) > 0:
                frames.append(pts)
                print(f"  Sampled frame {len(frames)}/{max_frames} (bag frame {idx}/{total})", end='\r')
                if len(frames) >= max_frames:
                    break
        idx += 1
    bag.close()

    if not frames:
        print("No point cloud data found!")
        sys.exit(1)

    print(f"\nExtracted {len(frames)} frames")
    generate_animated_html(frames, output, fps=5)
