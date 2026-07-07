const canvas = document.getElementById("fieldCanvas");
const ctx = canvas.getContext("2d");
const video = document.getElementById("videoFeed");
const videoFallback = document.getElementById("videoFallback");
const statusPill = document.getElementById("statusPill");
const fpsValue = document.getElementById("fpsValue");
const poseValue = document.getElementById("poseValue");
const tagValue = document.getElementById("tagValue");
const pieceCount = document.getElementById("pieceCount");
const poseLine = document.getElementById("poseLine");
const timingLine = document.getElementById("timingLine");
const yoloLine = document.getElementById("yoloLine");
const pieceList = document.getElementById("pieceList");

const fieldImage = new Image();
fieldImage.src = "/api/field-image";
let fieldImageReady = false;
fieldImage.onload = () => {
  fieldImageReady = true;
};

let latestState = null;

function resizeCanvas() {
  const bounds = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const nextWidth = Math.max(1, Math.floor(bounds.width * dpr));
  const nextHeight = Math.max(1, Math.floor(bounds.height * dpr));
  if (canvas.width !== nextWidth || canvas.height !== nextHeight) {
    canvas.width = nextWidth;
    canvas.height = nextHeight;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function fieldBounds(state) {
  const field = state.field;
  const bounds = {
    minX: 0,
    maxX: field.lengthM,
    minY: 0,
    maxY: field.widthM,
  };
  const mapMode = state.display?.mapMode || "field";
  if (mapMode !== "auto") {
    return bounds;
  }

  const include = (x, y) => {
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return;
    }
    bounds.minX = Math.min(bounds.minX, x);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.maxY = Math.max(bounds.maxY, y);
  };
  if (state.cameraPose) {
    include(state.cameraPose.x, state.cameraPose.y);
  }
  for (const piece of state.pieces || []) {
    include(piece.x, piece.y);
  }

  const spanX = Math.max(1, bounds.maxX - bounds.minX);
  const spanY = Math.max(1, bounds.maxY - bounds.minY);
  const margin = Math.max(0.75, Math.max(spanX, spanY) * 0.08);
  bounds.minX -= margin;
  bounds.maxX += margin;
  bounds.minY -= margin;
  bounds.maxY += margin;
  return bounds;
}

function fieldMapper(field, width, height, bounds) {
  const padding = 26;
  const spanX = bounds.maxX - bounds.minX;
  const spanY = bounds.maxY - bounds.minY;
  const scale = Math.min(
    (width - padding * 2) / spanX,
    (height - padding * 2) / spanY,
  );
  const worldWidthPx = spanX * scale;
  const worldHeightPx = spanY * scale;
  const left = (width - worldWidthPx) / 2;
  const top = (height - worldHeightPx) / 2;
  return {
    scale,
    left,
    top,
    width: worldWidthPx,
    height: worldHeightPx,
    bounds,
    point(x, y) {
      return [left + (x - bounds.minX) * scale, top + worldHeightPx - (y - bounds.minY) * scale];
    },
  };
}

function drawField(state) {
  resizeCanvas();
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#151816";
  ctx.fillRect(0, 0, width, height);

  if (!state || !state.field) {
    return;
  }

  const field = state.field;
  const map = fieldMapper(field, width, height, fieldBounds(state));
  ctx.save();
  const [fieldLeft, fieldBottom] = map.point(0, 0);
  const [fieldRight, fieldTop] = map.point(field.lengthM, field.widthM);
  const fieldX = Math.min(fieldLeft, fieldRight);
  const fieldY = Math.min(fieldTop, fieldBottom);
  const fieldW = Math.abs(fieldRight - fieldLeft);
  const fieldH = Math.abs(fieldBottom - fieldTop);

  ctx.fillStyle = "#19201c";
  ctx.fillRect(map.left, map.top, map.width, map.height);
  ctx.fillStyle = "#224934";
  ctx.fillRect(fieldX, fieldY, fieldW, fieldH);

  if (fieldImageReady) {
    ctx.globalAlpha = 0.6;
    ctx.drawImage(fieldImage, fieldX, fieldY, fieldW, fieldH);
    ctx.globalAlpha = 1;
  }

  ctx.strokeStyle = "rgba(223, 236, 218, 0.72)";
  ctx.lineWidth = 2;
  ctx.strokeRect(fieldX, fieldY, fieldW, fieldH);

  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(223, 236, 218, 0.16)";
  for (let x = 1; x < field.lengthM; x += 1) {
    const [px] = map.point(x, 0);
    ctx.beginPath();
    ctx.moveTo(px, fieldY);
    ctx.lineTo(px, fieldY + fieldH);
    ctx.stroke();
  }
  for (let y = 1; y < field.widthM; y += 1) {
    const [, py] = map.point(0, y);
    ctx.beginPath();
    ctx.moveTo(fieldX, py);
    ctx.lineTo(fieldX + fieldW, py);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(102, 217, 239, 0.42)";
  const [midX] = map.point(field.lengthM / 2, 0);
  ctx.beginPath();
  ctx.moveTo(midX, fieldY);
  ctx.lineTo(midX, fieldY + fieldH);
  ctx.stroke();

  drawTags(field, map);
  drawCamera(state.cameraPose, map);
  drawPieces(state.pieces || [], map);
  ctx.restore();
}

function drawTags(field, map) {
  ctx.font = "11px Inter, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const tag of field.tags || []) {
    const [x, y] = map.point(tag.x, tag.y);
    ctx.fillStyle = "#ff7f6e";
    ctx.fillRect(x - 3, y - 3, 6, 6);
    ctx.fillStyle = "rgba(255, 255, 255, 0.82)";
    ctx.fillText(String(tag.id), x, y - 12);
  }
}

function drawCamera(pose, map) {
  if (!pose) {
    return;
  }
  const [x, y] = map.point(pose.x, pose.y);
  const yaw = (pose.yawDeg * Math.PI) / 180;
  const length = Math.max(24, map.scale * 0.55);
  const noseX = x + Math.cos(yaw) * length;
  const noseY = y - Math.sin(yaw) * length;
  ctx.strokeStyle = "#66d9ef";
  ctx.fillStyle = "#66d9ef";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(noseX, noseY);
  ctx.stroke();
}

function drawPieces(pieces, map) {
  ctx.font = "12px Inter, sans-serif";
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (const piece of pieces) {
    const [x, y] = map.point(piece.x, piece.y);
    ctx.fillStyle = piece.onField ? "#f6c343" : "#ff7f6e";
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "rgba(0, 0, 0, 0.76)";
    ctx.fillText(String(Math.round(piece.score * 100)), x + 11, y);
  }
}

function updateTelemetry(state) {
  statusPill.textContent = state.status || "unknown";
  fpsValue.textContent = String(state.fps ?? 0);
  poseValue.textContent = state.cameraPoseStale ? "stale" : "live";
  tagValue.textContent = state.cameraPose?.tagId ?? "-";
  pieceCount.textContent = String((state.pieces || []).length);
  yoloLine.textContent = `YOLO ${state.yoloStatus || "--"}`;
  const timing = state.timingMs || {};
  timingLine.textContent = `tag ${timing.tag ?? "--"} ms, yolo ${timing.yolo ?? "--"} ms, jpeg ${timing.jpeg ?? "--"} ms`;

  if (state.cameraPose) {
    const pose = state.cameraPose;
    poseLine.textContent = `x ${pose.x.toFixed(2)} m, y ${pose.y.toFixed(2)} m, z ${pose.z.toFixed(2)} m, yaw ${pose.yawDeg.toFixed(1)} deg`;
  } else {
    poseLine.textContent = "x --, y --, z --";
  }

  pieceList.replaceChildren();
  for (const piece of state.pieces || []) {
    const row = document.createElement("div");
    row.className = "piece-row";
    const name = document.createElement("span");
    name.textContent = `${piece.label} ${(piece.score * 100).toFixed(0)}%`;
    const position = document.createElement("span");
    position.textContent = `${piece.x.toFixed(2)}, ${piece.y.toFixed(2)}`;
    row.append(name, position);
    pieceList.append(row);
  }
}

async function pollState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    latestState = await response.json();
    updateTelemetry(latestState);
    video.src = `/api/frame.jpg?t=${Date.now()}`;
  } catch (error) {
    statusPill.textContent = "offline";
  }
}

video.onload = () => {
  videoFallback.style.display = "none";
};
video.onerror = () => {
  videoFallback.style.display = "grid";
};

function renderLoop() {
  drawField(latestState);
  requestAnimationFrame(renderLoop);
}

window.addEventListener("resize", resizeCanvas);
setInterval(pollState, 100);
pollState();
renderLoop();
