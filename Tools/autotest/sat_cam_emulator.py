#!/usr/bin/env python3

import argparse
import asyncio
import json
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path

import aiohttp
from aiohttp import web
import cv2
import numpy as np
from pymavlink import mavutil

DEFAULT_RESOLUTION = (1280, 720)
DEFAULT_FOV_DEGREES = 90
DEFAULT_FPS = 30
DEFAULT_SMOOTH_TAU_S = 0.2
ROS_IMAGE_TOPIC = "/camera/image_raw"
ROS_COMPRESSED_TOPIC = "/camera/image_raw/compressed"
ROS_FRAME_ID = "camera"
ROS_JPEG_QUALITY = 85
MJPEG_BIND_HOST = "127.0.0.1"
MIN_VIEW_ALT_M = 0.0
EARTH_RADIUS_M = 6378137.0

TILE_SIZE = 256
MAPBOX_TILE_URL = "https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.png?access_token={token}"
TILE_FETCH_CONCURRENCY = 24

CACHE_DIR = Path("/tmp/sat_cam_tiles")


def meters_per_pixel_at_zoom(lat_deg, zoom):
    """Web Mercator: ground meters per pixel at latitude and integer zoom z."""
    return (EARTH_RADIUS_M * 2 * math.pi * math.cos(math.radians(lat_deg))) / (256.0 * (2**zoom))


def lat_lon_to_frac_tile(lat, lon, zoom):
    n = 2.0**zoom
    x = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x, y


class FpsMeter:
    """Rolling output frame rate from monotonic timestamps."""

    def __init__(self, window_s: float = 1.0):
        self.window_s = max(0.25, float(window_s))
        self._samples: list[float] = []
        self.last_fps = 0.0

    def tick(self) -> float:
        now = time.monotonic()
        self._samples.append(now)
        cutoff = now - self.window_s
        while self._samples and self._samples[0] < cutoff:
            self._samples.pop(0)
        if len(self._samples) >= 2:
            span = self._samples[-1] - self._samples[0]
            if span > 0.0:
                self.last_fps = (len(self._samples) - 1) / span
        return self.last_fps


def sim_state_lat_lon_deg(msg) -> tuple[float, float]:
    """Decode SIM_STATE lat/lon: prefer lat_int/lon_int; else float fields (ArduPilot may scale by 1e7)."""
    lat_i = int(getattr(msg, "lat_int", 0) or 0)
    lon_i = int(getattr(msg, "lon_int", 0) or 0)
    if lat_i != 0 and lon_i != 0:
        return lat_i / 1.0e7, lon_i / 1.0e7
    la = float(msg.lat)
    lo = float(msg.lon)
    if abs(la) > 90.0:
        la /= 1.0e7
    if abs(lo) > 180.0:
        lo /= 1.0e7
    return la, lo


def ned_vel_to_mosaic_uv_rate(lat_deg: float, zoom: int, vel_n: float, vel_e: float) -> tuple[float, float]:
    """Map NED north/east (m/s) to mosaic pixel rates (u east, v south)."""
    mpp = meters_per_pixel_at_zoom(lat_deg, zoom)
    if mpp <= 0.0:
        return 0.0, 0.0
    return vel_e / mpp, (-vel_n) / mpp


class ViewSmoother:
    """Exponential smoothing with optional velocity feed-forward between MAVLink samples."""

    def __init__(self, tau_s: float = DEFAULT_SMOOTH_TAU_S):
        self.tau_s = max(0.0, float(tau_s))
        self.initialized = False
        self.u = 0.0
        self.v = 0.0
        self.alt = 0.0
        self.heading = 0.0

    def reset(self) -> None:
        self.initialized = False

    def _alpha(self, dt: float) -> float:
        if self.tau_s <= 0.0:
            return 1.0
        return 1.0 - math.exp(-max(dt, 1e-4) / self.tau_s)

    @staticmethod
    def _lerp_angle_deg(a: float, b: float, t: float) -> float:
        delta = ((b - a + 180.0) % 360.0) - 180.0
        return (a + t * delta) % 360.0

    def step(
        self,
        u: float,
        v: float,
        alt: float,
        heading: float,
        dt: float,
        vel_u: float = 0.0,
        vel_v: float = 0.0,
        vel_alt: float = 0.0,
    ) -> tuple[float, float, float, float]:
        if not self.initialized:
            self.u, self.v, self.alt, self.heading = u, v, alt, heading
            self.initialized = True
            return self.u, self.v, self.alt, self.heading

        alpha = self._alpha(dt)
        if self.tau_s > 0.0:
            self.u += vel_u * dt
            self.v += vel_v * dt
            self.alt += vel_alt * dt
            self.u += alpha * (u - self.u)
            self.v += alpha * (v - self.v)
            self.alt += alpha * (alt - self.alt)
            self.heading = self._lerp_angle_deg(self.heading, heading, alpha)
        else:
            self.u, self.v, self.alt, self.heading = u, v, alt, heading
        return self.u, self.v, self.alt, self.heading


class RosImagePublisher:
    """Publish ROS images on a worker thread so the compose loop stays real-time."""

    def __init__(
        self,
        topic: str,
        frame_id: str,
        publish_size: tuple[int, int] | None = None,
        compressed: bool = False,
        jpeg_quality: int = 85,
    ):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSPresetProfiles
            from sensor_msgs.msg import CompressedImage, Image
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 publish requested but rclpy is not available. "
                "Source your ROS 2 install (e.g. /opt/ros/humble/setup.bash)."
            ) from exc

        self._rclpy = rclpy
        self._Image = Image
        self._CompressedImage = CompressedImage
        self._compressed = compressed
        self._jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self._publish_size = publish_size
        self._owns_context = False
        if not rclpy.ok():
            rclpy.init()
            self._owns_context = True

        class _SatCamRosNode(Node):
            def __init__(self, node_name: str) -> None:
                super().__init__(node_name)

        self._node = _SatCamRosNode("sat_cam_emulator")
        pub_qos = QoSPresetProfiles.SENSOR_DATA.value
        msg_type = CompressedImage if compressed else Image
        self._pub = self._node.create_publisher(msg_type, topic, pub_qos)
        self._frame_id = frame_id
        self._stop = threading.Event()
        self._queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=1)
        self._spin_thread = threading.Thread(target=self._spin_loop, name="sat_cam_ros_spin", daemon=True)
        self._publish_thread = threading.Thread(
            target=self._publish_loop, name="sat_cam_ros_pub", daemon=True
        )
        self._spin_thread.start()
        self._publish_thread.start()
        mode = "compressed_jpeg" if compressed else "bgr8"
        if publish_size is not None:
            mode = f"{mode}@{publish_size[0]}x{publish_size[1]}"
        print(
            f"[ROS] {topic} frame_id={frame_id} qos=sensor_data mode={mode} "
            "(async publish, queue depth 1)",
            flush=True,
        )

    def _spin_loop(self) -> None:
        while not self._stop.is_set():
            self._rclpy.spin_once(self._node, timeout_sec=0.01)

    def _prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        if self._publish_size is not None:
            target_w, target_h = self._publish_size
            if frame.shape[1] != target_w or frame.shape[0] != target_h:
                frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)
        return frame

    def _publish_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if frame is None:
                break
            frame = self._prepare_frame(frame)
            stamp = self._node.get_clock().now().to_msg()
            if self._compressed:
                ok, buf = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
                )
                if not ok:
                    continue
                msg = self._CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_id
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self._pub.publish(msg)
                continue

            h, w = frame.shape[:2]
            msg = self._Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = int(h)
            msg.width = int(w)
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(w * 3)
            msg.data = frame.tobytes()
            self._pub.publish(msg)

    def publish(self, frame: np.ndarray) -> None:
        if frame is None or frame.size == 0:
            return
        try:
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
            self._queue.put_nowait(frame)
        except queue.Full:
            pass

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(None)
            except queue.Full:
                pass
        if self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def draw_fps_overlay(frame, out_fps: float, cap_fps: float) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    label = f"OUT {out_fps:.1f} fps (cap {cap_fps:.0f})"
    scale = 0.55
    thick = 1
    tw, th = cv2.getTextSize(label, font, scale, thick)[0]
    x = max(10, frame.shape[1] - tw - 16)
    y = max(th + 12, frame.shape[0] - 12)
    cv2.rectangle(frame, (x - 8, y - th - 8), (x + tw + 8, y + 8), (0, 0, 0), -1)
    cv2.putText(frame, label, (x, y), font, scale, (0, 255, 0), thick, cv2.LINE_AA)


class MAVLinkConnection:
    def __init__(self, port: int = 14550, pose_source: str = "sim", sim_view_agl_m: float = 30.0):
        self.port = port
        self.pose_source = str(pose_source).strip().lower()
        if self.pose_source not in ("sim", "ekf"):
            raise ValueError(f"pose_source must be sim or ekf, got {pose_source!r}")
        self.sim_view_agl_m = max(0.5, float(sim_view_agl_m))
        self.connection = None
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.heading = 0.0
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.vel_up = 0.0
        self.last_update = 0.0
        self.last_local_alt_t = 0.0
        self.has_fix = False

    async def connect(self, position_stream_hz: int = 20):
        """Wait for heartbeat, then MAV_CMD_SET_MESSAGE_INTERVAL for pose stream (Hz clamped 2-50)."""
        url = f"udp:127.0.0.1:{self.port}"
        print(f"[MAVLink] Connecting to {url}...")
        self.connection = mavutil.mavlink_connection(url)

        def _wait_heartbeat():
            self.connection.wait_heartbeat()

        await asyncio.to_thread(_wait_heartbeat)
        tsys = self.connection.target_system
        tcomp = self.connection.target_component
        print(f"[MAVLink] Heartbeat system={tsys} component={tcomp}")

        hz = max(2, min(50, int(position_stream_hz)))
        interval_us = max(1, int(1_000_000.0 / float(hz)))
        cmd = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        if self.pose_source == "sim":
            msg_names = ("SIM_STATE",)
        else:
            msg_names = ("GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "ATTITUDE")
        for msg_name in msg_names:
            mid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}")
            self.connection.mav.command_long_send(
                tsys,
                tcomp,
                cmd,
                0,
                float(mid),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        print(
            f"[MAVLink] SET_MESSAGE_INTERVAL {hz} Hz ({interval_us} us) for {', '.join(msg_names)} "
            f"(pose_source={self.pose_source})",
            flush=True,
        )

    def drain(self):
        if self.connection is None:
            return
        while True:
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                break
            self._handle_message(msg)

    def _handle_message(self, msg):
        msg_type = msg.get_type()

        if self.pose_source == "sim" and msg_type in (
            "GLOBAL_POSITION_INT",
            "LOCAL_POSITION_NED",
            "ATTITUDE",
        ):
            return

        if msg_type == "SIM_STATE" and self.pose_source == "sim":
            self.lat, self.lon = sim_state_lat_lon_deg(msg)
            self.alt = self.sim_view_agl_m
            self.vel_n = float(msg.vn)
            self.vel_e = float(msg.ve)
            self.vel_up = -float(msg.vd)
            self.heading = math.degrees(float(msg.yaw)) % 360.0
            self.has_fix = True
            self.last_update = time.time()
            return

        if msg_type == "GLOBAL_POSITION_INT":
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            if self.last_local_alt_t <= 0.0 or (time.time() - self.last_local_alt_t) > 0.5:
                self.alt = msg.relative_alt / 1000.0
            if msg.hdg != 65535:
                self.heading = msg.hdg / 100.0
            self.vel_n = msg.vx / 100.0
            self.vel_e = msg.vy / 100.0
            self.vel_up = -msg.vz / 100.0
            self.has_fix = True
            self.last_update = time.time()

        elif msg_type == "LOCAL_POSITION_NED":
            self.alt = -float(msg.z)
            self.vel_n = float(msg.vx)
            self.vel_e = float(msg.vy)
            self.vel_up = -float(msg.vz)
            self.last_local_alt_t = time.time()
            self.last_update = time.time()

        elif msg_type == "ATTITUDE":
            self.heading = math.degrees(msg.yaw) % 360.0

    def is_stale(self, timeout=5.0):
        if self.last_update <= 0:
            return True
        return time.time() - self.last_update > timeout


class TileFetcher:
    def __init__(self, api_key):
        self.api_key = api_key
        self.cache_dir = CACHE_DIR
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.session = None
        self._http_sem = asyncio.Semaphore(TILE_FETCH_CONCURRENCY)

    async def start(self):
        self.session = aiohttp.ClientSession()

    async def stop(self):
        if self.session:
            await self.session.close()
            self.session = None

    async def fetch_tile(self, zoom, x, y):
        cache_path = self.cache_dir / f"{zoom}_{x}_{y}.png"

        if cache_path.exists():
            img = cv2.imread(str(cache_path))
            if img is not None:
                return img

        if self.session is None:
            return None

        url = MAPBOX_TILE_URL.format(z=zoom, x=x, y=y, token=self.api_key)

        try:
            async with self._http_sem:
                async with self.session.get(url, timeout=aiohttp.ClientTimeout(total=10)) as resp:
                    if resp.status == 200:
                        data = await resp.read()
                        cache_path.write_bytes(data)
                        img_arr = np.frombuffer(data, dtype=np.uint8)
                        return cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
                    print(f"http {resp.status}: {zoom}/{x}/{y}")
        except asyncio.TimeoutError:
            print(f"timeout: {zoom}/{x}/{y}")
        except Exception as e:
            print(f"error: {e}")

        return None


def lat_lon_to_tile(lat, lon, zoom):
    n = 2**zoom
    x = int((lon + 180) / 360 * n)
    lat_rad = math.radians(lat)
    y = int((1 - math.asinh(math.tan(lat_rad)) / math.pi) / 2 * n)
    return x, y


async def build_airfield_mosaic(fetcher, center_lat, center_lon, radius_m, zoom, progress=None):
    """Stitch Mapbox tiles covering a ground disk around center."""
    mpp = meters_per_pixel_at_zoom(center_lat, zoom)
    tile_ground_m = mpp * TILE_SIZE
    half = int(math.ceil(radius_m / tile_ground_m)) + 2
    cx, cy = lat_lon_to_tile(center_lat, center_lon, zoom)
    n = 2**zoom
    min_tx = max(0, cx - half)
    max_tx = min(n - 1, cx + half)
    min_ty = max(0, cy - half)
    max_ty = min(n - 1, cy + half)

    nw = (max_tx - min_tx + 1) * TILE_SIZE
    nh = (max_ty - min_ty + 1) * TILE_SIZE
    mosaic = np.zeros((nh, nw, 3), dtype=np.uint8)
    n_tot = (max_tx - min_tx + 1) * (max_ty - min_ty + 1)

    if progress is not None:
        progress["phase"] = "downloading"
        progress["current"] = 0
        progress["total"] = n_tot
        progress["tiles_ok"] = 0
        progress["mosaic_px"] = f"{nw}x{nh}"
        progress["zoom"] = zoom
        progress["started"] = time.time()

    print(
        f"[Airfield] Mosaic z={zoom} tiles x=[{min_tx},{max_tx}] y=[{min_ty},{max_ty}] "
        f"({n_tot} tiles, {nw}x{nh}px)",
        flush=True,
    )

    pairs = [(tx, ty) for tx in range(min_tx, max_tx + 1) for ty in range(min_ty, max_ty + 1)]
    log_every = max(1, min(50, n_tot // 40))
    print(
        f"[Airfield] fetching {n_tot} tiles in parallel (max {TILE_FETCH_CONCURRENCY} HTTP), "
        f"log every ~{log_every}...",
        flush=True,
    )

    async def fetch_pair(tx, ty):
        return tx, ty, await fetcher.fetch_tile(zoom, tx, ty)

    tasks = [asyncio.create_task(fetch_pair(tx, ty)) for tx, ty in pairs]
    n_ok = 0
    for i, fut in enumerate(asyncio.as_completed(tasks), start=1):
        tx, ty, tile = await fut
        if tile is not None:
            n_ok += 1
            ix = (tx - min_tx) * TILE_SIZE
            iy = (ty - min_ty) * TILE_SIZE
            mosaic[iy : iy + TILE_SIZE, ix : ix + TILE_SIZE] = tile
        if progress is not None:
            progress["current"] = i
            progress["tiles_ok"] = n_ok
        if i == 1 or i == n_tot or i % log_every == 0:
            print(f"[Airfield] tiles {i}/{n_tot} ok={n_ok}", flush=True)
        if i % 32 == 0:
            await asyncio.sleep(0)

    if n_ok < max(1, n_tot // 2):
        print(f"[Airfield] WARNING: only {n_ok}/{n_tot} tiles loaded", flush=True)

    print(f"[Airfield] Mosaic ready ({n_ok} tiles)", flush=True)
    if progress is not None:
        progress["phase"] = "mosaic_done"
        progress["current"] = n_tot
    return mosaic, min_tx, min_ty, zoom


def save_airfield_cache(out_dir, mosaic, min_tx, min_ty, zoom, center_lat, center_lon, radius_m):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / "airfield_mosaic.png"
    meta_path = out_dir / "airfield_meta.json"
    cv2.imwrite(str(png_path), mosaic)
    meta = {
        "zoom": zoom,
        "min_tile_x": min_tx,
        "min_tile_y": min_ty,
        "height_px": int(mosaic.shape[0]),
        "width_px": int(mosaic.shape[1]),
        "center_lat": center_lat,
        "center_lon": center_lon,
        "radius_m": radius_m,
    }
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"[Airfield] Saved {png_path} and {meta_path}")


def load_airfield_cache(out_dir):
    out_dir = Path(out_dir)
    png_path = out_dir / "airfield_mosaic.png"
    meta_path = out_dir / "airfield_meta.json"
    if not png_path.is_file() or not meta_path.is_file():
        return None
    img = cv2.imread(str(png_path))
    if img is None:
        return None
    meta = json.loads(meta_path.read_text())
    return img, meta


class AirfieldSlidingComposer:
    """Sliding window on a fixed-z mosaic. Footprint uses max(min_view_alt_m, rel AGL); rel AGL is MAV rel alt clamped to >= 0. HUD shows true MAV rel alt."""

    def __init__(self, mosaic_bgr, min_tx, min_ty, zoom, resolution, fov_degrees, min_view_alt_m, show_hud=True):
        self.mosaic = mosaic_bgr
        self.min_tx = min_tx
        self.min_ty = min_ty
        self.zoom = zoom
        self.resolution = resolution
        self.fov_degrees = fov_degrees
        self.min_view_alt_m = min_view_alt_m
        self.show_hud = show_hud
        self.mosaic_h, self.mosaic_w = mosaic_bgr.shape[:2]

    def _mosaic_uv(self, lat, lon):
        fx, fy = lat_lon_to_frac_tile(lat, lon, self.zoom)
        u = (fx - self.min_tx) * TILE_SIZE
        v = (fy - self.min_ty) * TILE_SIZE
        return u, v

    def compose(self, lat, lon, altitude_m, heading, view_uv=None, view_alt=None, view_heading=None):
        rw, rh = self.resolution
        rel_agl = max(0.0, float(altitude_m))
        view_agl = rel_agl if view_alt is None else max(0.0, float(view_alt))
        alt_view = max(self.min_view_alt_m, view_agl)
        ground_w_m = 2.0 * alt_view * math.tan(math.radians(self.fov_degrees / 2.0))
        ground_h_m = ground_w_m * (rh / max(rw, 1))

        mpp = meters_per_pixel_at_zoom(lat, self.zoom)
        cw0 = max(32.0, ground_w_m / mpp)
        ch0 = max(32.0, ground_h_m / mpp)

        if view_uv is None:
            uc, vc = self._mosaic_uv(lat, lon)
        else:
            uc, vc = view_uv
        h_view = heading if view_heading is None else float(view_heading)
        # Square overscan: max edge of rotated FOV AABB is hypot(cw0,ch0). Constant S
        # avoids patch size jumping with yaw (which looked like squeeze/breathe).
        S = int(max(32, math.ceil(math.hypot(cw0, ch0))))
        x0 = int(round(uc - S / 2.0))
        y0 = int(round(vc - S / 2.0))
        x1 = x0 + S
        y1 = y0 + S

        patch = np.zeros((S, S, 3), dtype=np.uint8)

        sx = max(0, x0)
        sy = max(0, y0)
        ex = min(self.mosaic_w, x1)
        ey = min(self.mosaic_h, y1)

        if ex > sx and ey > sy:
            dx0 = sx - x0
            dy0 = sy - y0
            patch[dy0 : dy0 + (ey - sy), dx0 : dx0 + (ex - sx)] = self.mosaic[sy:ey, sx:ex]

        cx = uc - float(x0)
        cy = vc - float(y0)
        M = cv2.getRotationMatrix2D((cx, cy), h_view, 1.0)
        patch = cv2.warpAffine(
            patch, M, (S, S), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE
        )

        cw_i = max(1, min(int(round(cw0)), S))
        ch_i = max(1, min(int(round(ch0)), S))
        x_crop = int(round(cx - cw_i / 2.0))
        y_crop = int(round(cy - ch_i / 2.0))
        x_crop = max(0, min(x_crop, S - cw_i))
        y_crop = max(0, min(y_crop, S - ch_i))
        patch = patch[y_crop : y_crop + ch_i, x_crop : x_crop + cw_i]

        interp = cv2.INTER_AREA if patch.shape[1] >= rw and patch.shape[0] >= rh else cv2.INTER_LINEAR
        frame = cv2.resize(patch, (rw, rh), interpolation=interp)
        if not self.show_hud:
            return frame
        return self._overlay(frame, lat, lon, altitude_m, heading, alt_view)

    def _overlay(self, frame, lat, lon, altitude_m, heading, alt_view):
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 118), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        font = cv2.FONT_HERSHEY_SIMPLEX
        col = (255, 255, 255)
        y0 = 30
        cv2.putText(
            frame,
            f"AIRFIELD z{self.zoom}  footprint@{alt_view:.0f}m",
            (20, y0),
            font,
            0.45,
            col,
            1,
        )
        cv2.putText(frame, f"Lat: {lat:.6f}", (20, y0 + 22), font, 0.5, col, 1)
        cv2.putText(frame, f"Lon: {lon:.6f}", (20, y0 + 42), font, 0.5, col, 1)
        cv2.putText(
            frame,
            f"Alt: {altitude_m:.2f}m (MAV rel)  HDG: {heading:.0f}",
            (20, y0 + 62),
            font,
            0.45,
            col,
            1,
        )
        return frame


class NoFixPlaceholder:
    def __init__(self, resolution=DEFAULT_RESOLUTION):
        self.resolution = resolution

    def generate(self):
        rw, rh = self.resolution
        frame = np.full((rh, rw, 3), (40, 40, 40), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        main = "NO GPS FIX"
        sub = "Arm and fly for satellite view"
        for text, scale, thick, y_off, color in (
            (main, 1.0, 2, 0, (255, 255, 255)),
            (sub, 0.55, 1, 50, (180, 180, 180)),
        ):
            tw = cv2.getTextSize(text, font, scale, thick)[0][0]
            cv2.putText(
                frame,
                text,
                ((rw - tw) // 2, rh // 2 + y_off),
                font,
                scale,
                color,
                thick,
            )
        return frame


class NoMosaicPlaceholder:
    def __init__(self, resolution=DEFAULT_RESOLUTION, progress=None):
        self.resolution = resolution
        self.progress = progress

    def generate(self):
        rw, rh = self.resolution
        frame = np.full((rh, rw, 3), (35, 35, 50), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        pr = self.progress or {}

        phase = pr.get("phase", "")
        if phase == "waiting_gps":
            lines = ("AIRFIELD MOSAIC", "Waiting for GPS fix...")
        elif phase == "loading_cache":
            lines = ("AIRFIELD MOSAIC", "Loading cached PNG from disk...")
        elif phase == "cache_ready":
            lines = ("AIRFIELD MOSAIC", "Cache loaded, starting view...")
        elif phase == "saving":
            lines = ("AIRFIELD MOSAIC", "Saving cache to disk...")
        elif phase == "downloading":
            lines = ("AIRFIELD MOSAIC", "Downloading Mapbox tiles...")
        else:
            lines = ("AIRFIELD MOSAIC", "Preparing...")

        y = rh // 2 - 100
        for line in lines:
            tw = cv2.getTextSize(line, font, 0.7, 2)[0][0]
            cv2.putText(frame, line, ((rw - tw) // 2, y), font, 0.7, (220, 220, 255), 2)
            y += 42

        tot = int(pr.get("total", 0) or 0)
        cur = int(pr.get("current", 0) or 0)
        ok = int(pr.get("tiles_ok", 0) or 0)
        if tot > 0:
            line = f"Tiles {cur} / {tot}   decoded {ok}"
            tw = cv2.getTextSize(line, font, 0.55, 1)[0][0]
            cv2.putText(frame, line, ((rw - tw) // 2, y + 10), font, 0.55, (200, 255, 200), 1)
            pct = min(1.0, cur / float(tot))
            x0, x1 = rw // 8, 7 * rw // 8
            bar_y1, bar_y2 = y + 35, y + 58
            cv2.rectangle(frame, (x0, bar_y1), (x1, bar_y2), (70, 70, 100), 2)
            inner_w = x1 - x0 - 4
            fill = int(inner_w * pct)
            if fill > 0:
                cv2.rectangle(frame, (x0 + 2, bar_y1 + 2), (x0 + 2 + fill, bar_y2 - 2), (100, 160, 255), -1)

        mpx = pr.get("mosaic_px")
        if mpx:
            line = f"Mosaic {mpx}  z{pr.get('zoom', '')}"
            tw = cv2.getTextSize(line, font, 0.5, 1)[0][0]
            cv2.putText(frame, line, ((rw - tw) // 2, y + 85), font, 0.5, (180, 180, 200), 1)

        t0 = pr.get("started")
        if t0 and tot > 0 and cur > 0:
            dt = time.time() - t0
            rate = cur / max(dt, 0.001)
            eta = (tot - cur) / max(rate, 0.001)
            line = f"Elapsed {dt:.0f}s  ~{eta:.0f}s left  ({rate:.1f} tiles/s)"
            tw = cv2.getTextSize(line, font, 0.48, 1)[0][0]
            cv2.putText(frame, line, ((rw - tw) // 2, rh - 60), font, 0.48, (160, 160, 180), 1)

        return frame


class MjpegHttpServer:
    def __init__(self, host, port, fps_limit=30.0):
        self.host = host
        self.port = port
        self.fps_limit = fps_limit
        self._jpeg = None
        self._lock = asyncio.Lock()
        self._runner = None
        self._site = None

    async def start(self):
        app = web.Application()
        app.router.add_get("/video", self._handle_video)
        app.router.add_get("/", self._handle_index)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, self.host, self.port)
        await self._site.start()
        print(f"[MJPEG] http://{self.host}:{self.port}/video")

    async def stop(self):
        if self._site:
            await self._site.stop()
        if self._runner:
            await self._runner.cleanup()
        self._site = self._runner = None

    async def set_frame(self, frame):
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 78])
        data = buf.tobytes()
        async with self._lock:
            self._jpeg = data

    async def _handle_index(self, request):
        body = (
            f"<html><body><h1>Sat cam</h1>"
            f"<img src=\"/video\" width=\"960\"/><p><a href=\"/video\">/video</a></p></body></html>"
        )
        return web.Response(text=body, content_type="text/html")

    async def _handle_video(self, request):
        resp = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
            },
        )
        await resp.prepare(request)
        period = 1.0 / max(self.fps_limit, 1.0)
        try:
            while True:
                async with self._lock:
                    chunk = self._jpeg
                if chunk is None:
                    await asyncio.sleep(period)
                    continue
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + chunk + b"\r\n")
                await asyncio.sleep(period)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp


async def frame_generator_airfield(mavlink, composer_box, wait_ph, fix_ph, fps, smooth_tau_s):
    """composer_box[0] is set when mosaic build finishes."""
    period = 1.0 / max(fps, 1.0)
    smoother = ViewSmoother(smooth_tau_s)
    last_t = time.monotonic()

    while True:
        mavlink.drain()
        now = time.monotonic()
        dt = now - last_t
        last_t = now
        comp = composer_box[0]
        if comp is None:
            smoother.reset()
            yield wait_ph.generate()
            await asyncio.sleep(period)
            continue

        if not mavlink.has_fix or mavlink.is_stale():
            smoother.reset()
            yield fix_ph.generate()
            await asyncio.sleep(period)
            continue

        lat, lon, alt, hdg = mavlink.lat, mavlink.lon, mavlink.alt, mavlink.heading
        target_u, target_v = comp._mosaic_uv(lat, lon)
        vel_u, vel_v = ned_vel_to_mosaic_uv_rate(lat, comp.zoom, mavlink.vel_n, mavlink.vel_e)
        view_u, view_v, view_alt, view_hdg = smoother.step(
            target_u,
            target_v,
            alt,
            hdg,
            dt,
            vel_u,
            vel_v,
            mavlink.vel_up,
        )
        yield comp.compose(
            lat,
            lon,
            alt,
            hdg,
            view_uv=(view_u, view_v),
            view_alt=view_alt,
            view_heading=view_hdg,
        )
        await asyncio.sleep(period)


async def airfield_build_task(mavlink, fetcher, opts, resolution, composer_box, progress):
    progress["phase"] = "waiting_gps"
    progress["current"] = 0
    progress["total"] = 0
    progress["tiles_ok"] = 0
    print("[Airfield] Waiting for position fix to anchor mosaic (GPS or SIM_STATE)...", flush=True)
    while True:
        mavlink.drain()
        if mavlink.has_fix and not mavlink.is_stale():
            break
        await asyncio.sleep(0.05)

    cache_dir = Path(opts.airfield_cache_dir)
    loaded = None
    if opts.airfield_reuse_cache:
        progress["phase"] = "loading_cache"
        progress["started"] = time.time()
        loaded = load_airfield_cache(cache_dir)

    if loaded is not None:
        mosaic, meta = loaded
        z = int(meta["zoom"])
        mtx = int(meta["min_tile_x"])
        mty = int(meta["min_tile_y"])
        progress["phase"] = "cache_ready"
        progress["total"] = 0
        progress["current"] = 0
        progress["tiles_ok"] = 0
        progress["mosaic_px"] = f"{meta.get('width_px', mosaic.shape[1])}x{meta.get('height_px', mosaic.shape[0])}"
        progress["zoom"] = z
        print(f"[Airfield] Loaded cache from {cache_dir} (z={z})", flush=True)
    else:
        mosaic, mtx, mty, z = await build_airfield_mosaic(
            fetcher,
            mavlink.lat,
            mavlink.lon,
            float(opts.airfield_radius_m),
            int(opts.airfield_zoom),
            progress,
        )
        if opts.airfield_save_cache:
            progress["phase"] = "saving"
            progress["total"] = 0
            progress["current"] = 0
            progress["tiles_ok"] = 0
            save_airfield_cache(
                cache_dir,
                mosaic,
                mtx,
                mty,
                z,
                mavlink.lat,
                mavlink.lon,
                float(opts.airfield_radius_m),
            )
            progress["current"] = 1

    composer_box[0] = AirfieldSlidingComposer(
        mosaic,
        mtx,
        mty,
        z,
        resolution,
        DEFAULT_FOV_DEGREES,
        float(opts.min_view_alt_m),
        show_hud=not opts.no_hud,
    )
    if opts.pose_source == "sim":
        print(
            "[Airfield] View: lat/lon/heading from SIM_STATE (SITL truth); footprint AGL from "
            f"--sim-view-agl-m ({opts.sim_view_agl_m:.1f} m); "
            f"floor {opts.min_view_alt_m}m via --min-view-alt-m.",
            flush=True,
        )
    else:
        print(
            "[Airfield] View: footprint from MAV rel AGL (>= 0), "
            f"optional floor {opts.min_view_alt_m}m via --min-view-alt-m; "
            "HUD shows MAVLink relative altitude.",
            flush=True,
        )


async def main_async(opts):
    api_key = opts.api_key or os.environ.get("MAPBOX_API_KEY")
    if not api_key:
        print("ERROR: MAPBOX_API_KEY not set!")
        print("Get a free key at: https://account.mapbox.com/access-tokens/")
        print("Set it with: export MAPBOX_API_KEY='pk.xxx'")
        print("Or pass it with: --api-key 'pk.xxx'")
        return 1

    try:
        width, height = map(int, opts.resolution.split("x"))
        resolution = (width, height)
    except ValueError:
        print(f"ERROR: Invalid resolution format: {opts.resolution}")
        print("Expected format: WxH (e.g., 1280x720)")
        return 1

    fps = max(1, int(opts.fps))
    pos_hz = max(2, min(50, fps))
    smooth_tau_s = max(0.0, float(DEFAULT_SMOOTH_TAU_S))

    print(f"[Config] Resolution: {resolution}")
    print(f"[Config] MAVLink UDP port: {opts.port}")
    print(f"[Config] FPS cap: {fps}")
    print(f"[Config] MAVLink position/attitude request: {pos_hz} Hz (matches FPS cap)")
    print(f"[Config] View smoothing tau: {smooth_tau_s:.2f}s (always on)")
    print(f"[Config] frame HUD overlay: {not opts.no_hud}")
    print(f"[Config] local display: {not opts.no_display}")
    print(f"[Config] ros image publish: {opts.ros}")
    ros_publish_size = None
    if opts.ros_size:
        try:
            ros_w, ros_h = map(int, opts.ros_size.lower().split("x"))
            ros_publish_size = (ros_w, ros_h)
        except ValueError:
            print(f"ERROR: Invalid --ros-size format: {opts.ros_size} (expected WxH, e.g. 640x360)")
            return 1
    ros_topic = ROS_IMAGE_TOPIC
    if opts.ros and opts.ros_compressed:
        ros_topic = ROS_COMPRESSED_TOPIC
    if opts.ros:
        print(f"[Config] ros topic: {ros_topic} frame_id={ROS_FRAME_ID}")
        print(
            f"[Config] ros mode: "
            f"{'compressed_jpeg' if opts.ros_compressed else 'bgr8'}"
            + (f" size={ros_publish_size[0]}x{ros_publish_size[1]}" if ros_publish_size else "")
            + (f" jpeg_q={ROS_JPEG_QUALITY}" if opts.ros_compressed else "")
            + " qos=sensor_data"
        )
    print(f"[Config] pose-source: {opts.pose_source}")
    if opts.pose_source == "sim":
        print(f"[Config] sim-view-agl-m: {opts.sim_view_agl_m} (camera footprint; SIM_STATE.alt is MSL)")

    ros_pub = None
    if opts.ros:
        try:
            ros_pub = RosImagePublisher(
                ros_topic,
                ROS_FRAME_ID,
                publish_size=ros_publish_size,
                compressed=opts.ros_compressed,
                jpeg_quality=ROS_JPEG_QUALITY,
            )
        except RuntimeError as exc:
            print(f"ERROR: {exc}")
            return 1

    mavlink = MAVLinkConnection(
        port=opts.port,
        pose_source=opts.pose_source,
        sim_view_agl_m=float(opts.sim_view_agl_m),
    )
    fetcher = TileFetcher(api_key=api_key)
    placeholder = NoFixPlaceholder(resolution=resolution)
    airfield_progress = {}
    wait_mosaic = NoMosaicPlaceholder(resolution=resolution, progress=airfield_progress)

    mjpeg = None
    if opts.http_mjpeg_port > 0:
        mjpeg = MjpegHttpServer(MJPEG_BIND_HOST, opts.http_mjpeg_port, fps_limit=float(fps))
        await mjpeg.start()

    await fetcher.start()
    await mavlink.connect(position_stream_hz=pos_hz)

    composer_box = [None]
    asyncio.create_task(
        airfield_build_task(mavlink, fetcher, opts, resolution, composer_box, airfield_progress)
    )
    gen = frame_generator_airfield(
        mavlink, composer_box, wait_mosaic, placeholder, float(fps), smooth_tau_s
    )

    fps_meter = FpsMeter()
    fps_log_t = time.monotonic()

    try:
        if not opts.no_display:
            print("[Display] Press 'q' to quit")
            cv2.namedWindow("Satellite Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Satellite Camera", min(960, width), min(540, height))

        async for frame in gen:
            out_fps = fps_meter.tick()
            if not opts.no_hud:
                draw_fps_overlay(frame, out_fps, float(fps))
            now = time.monotonic()
            if now - fps_log_t >= 2.0:
                print(f"[FPS] out={out_fps:.1f} cap={fps}", flush=True)
                fps_log_t = now

            if not opts.no_display:
                cv2.imshow("Satellite Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if mjpeg:
                await mjpeg.set_frame(frame)
            if ros_pub:
                ros_pub.publish(frame)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted")
    finally:
        if ros_pub:
            ros_pub.shutdown()
        if mjpeg:
            await mjpeg.stop()
        await fetcher.stop()
        if not opts.no_display:
            cv2.destroyAllWindows()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Satellite camera emulator for ArduPilot SITL: fixed airfield Mapbox mosaic + sliding viewport.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
After the first position fix (GPS or SIM_STATE), Mapbox tiles are stitched into one mosaic, then a sliding viewport
warps that image at the output rate. Ground footprint uses max(--min-view-alt-m, AGL). With --pose-source sim,
AGL for the camera footprint is --sim-view-agl-m (SIM_STATE.alt is MSL, not height above ground).

Examples:
  python3 sat_cam_emulator.py --port 14550
  python3 sat_cam_emulator.py --port 14550 --pose-source ekf
  python3 sat_cam_emulator.py --airfield-radius-m 1500 --airfield-zoom 19
  python3 sat_cam_emulator.py --airfield-cache-dir /tmp/my_af --airfield-save-cache
  python3 sat_cam_emulator.py --airfield-reuse-cache --airfield-cache-dir /tmp/my_af

NGPS / ROS 2 (DDS): use SITL ground truth so the camera does not chase EKF fed by NGPS (loopback):
    source /opt/ros/humble/setup.bash
    python3 sat_cam_emulator.py --port 14550 --fps 30 --ros --no-hud --no-display --pose-source sim
    python3 sat_cam_emulator.py --port 14550 --fps 30 --ros --ros-compressed --ros-size 640x360 --no-hud --no-display

Legacy MJPEG + ap_ngps_ros2 bridge (optional browser preview):
    python3 sat_cam_emulator.py --port 14550 --http-mjpeg-port 8090 --fps 30 --no-hud --no-display
    ros2 launch ap_ngps_ros2 mjpeg_sat_cam_bridge.launch.py mjpeg_url:=http://127.0.0.1:8090/video

Tune --fps for output rate; first mosaic build needs a valid position (GPS fix or SIM_STATE in SITL).
""",
    )

    parser.add_argument("--port", type=int, default=14550, help="MAVLink UDP listen port (default: 14550)")
    parser.add_argument(
        "--pose-source",
        choices=("sim", "ekf"),
        default="sim",
        help="sim: SITL truth from MAVLink SIM_STATE (avoids EKF/NGPS loopback on lat/lon). "
        "ekf: GLOBAL_POSITION_INT + LOCAL_POSITION_NED + ATTITUDE (legacy).",
    )
    parser.add_argument(
        "--sim-view-agl-m",
        type=float,
        default=30.0,
        help="With pose-source=sim, AGL (m) used only for satellite footprint sizing (SIM_STATE.alt is MSL). Default: 30",
    )
    parser.add_argument("--http-mjpeg-port", type=int, default=0, help="MJPEG TCP port, 0=off (binds 127.0.0.1)")
    parser.add_argument("--resolution", default="1280x720", help="Camera WxH (default: 1280x720)")
    parser.add_argument(
        "--fps",
        type=int,
        default=DEFAULT_FPS,
        help="Max output rate for display, MJPEG /video, and MAVLink pose stream (default: 30). Use 24-30 for smoother motion.",
    )
    parser.add_argument("--api-key", default=None, help="Mapbox token (or MAPBOX_API_KEY)")
    parser.add_argument("--no-display", action="store_true", help="Disable local OpenCV preview window")
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Publish sensor_msgs/Image on the composed frame loop (source ROS 2 + rclpy)",
    )
    parser.add_argument(
        "--ros-compressed",
        action="store_true",
        help="Publish sensor_msgs/CompressedImage (jpeg) instead of raw bgr8",
    )
    parser.add_argument(
        "--ros-size",
        default=None,
        help="Downscale before ROS publish, WxH (e.g. 640x360). Applies to raw and compressed modes.",
    )
    parser.add_argument(
        "--no-hud",
        action="store_true",
        help="No on-frame pose or fps overlays (clean MJPEG for NGPS / ROS bridge)",
    )
    parser.add_argument(
        "--airfield-radius-m",
        type=float,
        default=1200.0,
        help="Radius (m) around first fix for mosaic (default: 1200)",
    )
    parser.add_argument(
        "--airfield-zoom",
        type=int,
        default=19,
        help="Mapbox z for mosaic (default: 19)",
    )
    parser.add_argument(
        "--airfield-cache-dir",
        default="/tmp/sat_cam_airfield",
        help="Dir for airfield_mosaic.png + airfield_meta.json",
    )
    parser.add_argument(
        "--airfield-save-cache",
        action="store_true",
        help="Save PNG+JSON after build",
    )
    parser.add_argument(
        "--airfield-reuse-cache",
        action="store_true",
        help="Load PNG+JSON if present (skip download)",
    )
    parser.add_argument(
        "--min-view-alt-m",
        type=float,
        default=MIN_VIEW_ALT_M,
        help="Min AGL (m) for footprint sizing (default: 0); footprint uses max(this, MAV rel AGL, 0)",
    )

    opts = parser.parse_args()

    try:
        return asyncio.run(main_async(opts))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
