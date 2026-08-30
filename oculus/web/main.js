// RoboProtocol Oculus Monitor -- read-only WebXR dashboard (Option 0).
// Renders a video quad + a floating telemetry HUD panel in VR, both fed
// by JPEG frames / telemetry JSON pushed over a single WebSocket by
// oculus-gateway. No control path: this page never sends anything on the
// socket beyond the initial handshake.

import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

const statusEl = document.getElementById("status");
const previewCanvas = document.getElementById("preview");
const previewCtx = previewCanvas.getContext("2d");
const enterVrButton = document.getElementById("enter-vr");

// --- Video + HUD canvases, shared between the 2D preview and the VR textures ---

const videoCanvas = document.createElement("canvas");
videoCanvas.width = 640;
videoCanvas.height = 360;
const videoCtx = videoCanvas.getContext("2d");

const hudCanvas = document.createElement("canvas");
hudCanvas.width = 640;
hudCanvas.height = 400;
const hudCtx = hudCanvas.getContext("2d");

let latestStatus = { phase: "connecting", robot_id: null, dof_count: null, camera: null, estopped: false };
let latestTelemetry = null;
let wsConnected = false;

function drawHud() {
  const ctx = hudCtx;
  ctx.fillStyle = "#0b0d10";
  ctx.fillRect(0, 0, hudCanvas.width, hudCanvas.height);
  ctx.fillStyle = "#d7dee5";
  ctx.font = "28px system-ui, sans-serif";
  let y = 40;
  const line = (text, color) => {
    ctx.fillStyle = color || "#d7dee5";
    ctx.fillText(text, 20, y);
    y += 38;
  };

  line(`gateway: ${wsConnected ? "connected" : "disconnected"}`, wsConnected ? "#6fd18f" : "#e07a5f");
  line(`phase: ${latestStatus.phase}`);
  line(`robot: ${latestStatus.robot_id ?? "-"}  dof: ${latestStatus.dof_count ?? "-"}`);
  line(`camera: ${latestStatus.camera ?? "-"}`);
  line(`e-stop: ${latestStatus.estopped ? "LATCHED" : "clear"}`, latestStatus.estopped ? "#e07a5f" : "#6fd18f");

  if (latestTelemetry) {
    const t = latestTelemetry;
    line(`battery: ${t.battery}%`);
    line(`roll/pitch/yaw: ${t.roll.toFixed(1)} / ${t.pitch.toFixed(1)} / ${t.yaw.toFixed(1)}`);
    line(`rtt: ${t.rtt_ms != null ? t.rtt_ms.toFixed(1) + "ms" : "-"}  seq: ${t.seq}`);
  } else {
    line("telemetry: waiting for first frame...", "#5c6773");
  }

  ctx.fillStyle = "#5c6773";
  ctx.font = "18px system-ui, sans-serif";
  ctx.fillText("monitor-only -- no control path (RoboProtocol discussion #12, option 0)", 20, hudCanvas.height - 20);
}
drawHud();

function drawPreview() {
  previewCtx.drawImage(videoCanvas, 0, 0, previewCanvas.width, previewCanvas.height);
  previewCtx.drawImage(hudCanvas, 0, 0, previewCanvas.width * 0.4, previewCanvas.height * 0.4 * (hudCanvas.height / hudCanvas.width));
}

// --- WebSocket: binary = JPEG video frame, text = JSON status/telemetry event ---

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.binaryType = "arraybuffer";

  ws.onopen = () => {
    wsConnected = true;
    statusEl.textContent = "connected -- waiting for robot session";
    drawHud();
  };

  ws.onclose = () => {
    wsConnected = false;
    statusEl.textContent = "gateway disconnected, retrying...";
    drawHud();
    setTimeout(connect, 1500);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = async (ev) => {
    if (typeof ev.data === "string") {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "status") {
          latestStatus = msg;
          if (msg.phase === "operating") enterVrButton.removeAttribute("disabled");
        } else if (msg.type === "telemetry") {
          latestTelemetry = msg;
        }
        drawHud();
        statusEl.textContent = `${latestStatus.phase}${latestStatus.robot_id ? " -- " + latestStatus.robot_id : ""}`;
      } catch (e) {
        console.warn("bad JSON event", e);
      }
      return;
    }

    try {
      const bitmap = await createImageBitmap(new Blob([ev.data], { type: "image/jpeg" }));
      if (videoCanvas.width !== bitmap.width || videoCanvas.height !== bitmap.height) {
        videoCanvas.width = bitmap.width;
        videoCanvas.height = bitmap.height;
      }
      videoCtx.drawImage(bitmap, 0, 0);
      bitmap.close();
      if (videoTexture) videoTexture.needsUpdate = true;
    } catch (e) {
      console.warn("failed to decode video frame", e);
    }
  };
}

// --- Three.js / WebXR scene ---

let videoTexture = null;
let renderer, scene, camera, hudTexture;

function initScene() {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.xr.enabled = true;
  // Default reference space ('local') puts the origin at the headset's
  // own position when tracking starts -- roughly at eye height, not the
  // floor -- so content placed at realistic "eye level" world-space
  // heights (y ~ 1.0-1.6) ends up far above the actual view and never
  // enters the frustum. 'local-floor' puts the origin at the floor
  // instead, matching how the scene below is laid out. Must be set
  // before the XR session starts.
  renderer.xr.setReferenceSpaceType("local-floor");
  document.body.appendChild(renderer.domElement);
  renderer.domElement.style.position = "fixed";
  renderer.domElement.style.inset = "0";
  renderer.domElement.style.zIndex = "-1"; // behind the 2D overlay until VR starts

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0d10);
  camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.05, 50);

  videoTexture = new THREE.CanvasTexture(videoCanvas);
  videoTexture.colorSpace = THREE.SRGBColorSpace;
  const videoMat = new THREE.MeshBasicMaterial({ map: videoTexture });
  const videoGeo = new THREE.PlaneGeometry(1.6, 0.9);
  const videoQuad = new THREE.Mesh(videoGeo, videoMat);
  videoQuad.position.set(0, 1.5, -2.2);
  scene.add(videoQuad);

  hudTexture = new THREE.CanvasTexture(hudCanvas);
  hudTexture.colorSpace = THREE.SRGBColorSpace;
  const hudMat = new THREE.MeshBasicMaterial({ map: hudTexture, transparent: true });
  const hudGeo = new THREE.PlaneGeometry(0.9, 0.56);
  const hudPanel = new THREE.Mesh(hudGeo, hudMat);
  hudPanel.position.set(1.05, 1.0, -1.6);
  hudPanel.rotation.y = -0.35;
  scene.add(hudPanel);

  scene.add(new THREE.AmbientLight(0xffffff, 1.0));

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  renderer.xr.addEventListener("sessionstart", () => {
    document.getElementById("overlay").style.display = "none";
    renderer.domElement.style.zIndex = "0";
  });
  renderer.xr.addEventListener("sessionend", () => {
    document.getElementById("overlay").style.display = "flex";
  });

  renderer.setAnimationLoop(() => {
    if (hudTexture) hudTexture.needsUpdate = true;
    renderer.render(scene, camera);
    drawPreview();
  });
}

async function initVrButton() {
  if (!navigator.xr) {
    statusEl.textContent = "WebXR not available in this browser";
    return;
  }
  const supported = await navigator.xr.isSessionSupported("immersive-vr").catch(() => false);
  if (!supported) {
    statusEl.textContent = "immersive-vr not supported on this device";
    return;
  }
  const vrButton = VRButton.createButton(renderer);
  vrButton.id = "vr-button-real";
  vrButton.style.position = "static";
  vrButton.style.padding = enterVrButton.style.padding;
  enterVrButton.replaceWith(vrButton);
  vrButton.disabled = latestStatus.phase !== "operating";
}

initScene();
initVrButton();
connect();
setInterval(drawPreview, 200);
