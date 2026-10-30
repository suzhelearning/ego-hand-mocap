"""Original-image observations and native-fisheye reconstruction overlays."""

from pathlib import Path
import subprocess
import numpy as np

EDGES = [(0, base) for base in (1, 5, 9, 13, 17)] + [
    (base+i, base+i+1) for base in (1, 5, 9, 13, 17) for i in range(3)]


def render_overlays(left, right, obs, results, cameras, T_left_right, output):
    import cv2
    inverse = np.linalg.inv(T_left_right)
    times = np.asarray(obs["timestamps_s"])
    fps = float(1 / np.median(np.diff(times))) if len(times) > 1 else float(left.get("source_fps", 30))
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("Invalid overlay timestamps")
    colors = {"stereo": (0, 255, 255), "mano": (255, 180, 20), "mano_temporal": (255, 50, 255)}
    mesh_stage = next(reversed(results))
    final = results[mesh_stage]
    faces = np.asarray(final.get("faces", []), dtype=np.int32)
    mesh_edges = None
    if faces.ndim == 2 and faces.shape[1] == 3 and "vertices_m" in final:
        mesh_edges = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]],
                                                       faces[:, [2, 0]]]), axis=1), axis=0)
    for c, (sequence, camera) in enumerate(zip((left, right), cameras)):
        path = Path(output) / ("overlay_left.mp4" if c == 0 else "overlay_right.mp4")
        command = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                   "-s", f"{camera.width}x{camera.height}", "-r", str(fps), "-i", "pipe:0",
                   "-an", "-c:v", "libx264", "-threads", "2", "-crf", "20",
                   "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
        log = path.with_suffix(".encode.log").open("wb")
        writer = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        by_id = {frame["frame_id"]: frame for frame in sequence["frames"]}
        try:
            for t in range(len(times)):
                if "camera_frame_id" in obs:
                    frame_id = int(obs["camera_frame_id"][t, c])
                    frame = by_id.get(frame_id)
                else:
                    frame = sequence["frames"][t] if t < len(sequence["frames"]) else None
                    frame_id = frame["frame_id"] if frame else -1
                image = (cv2.imread(frame["image_path"]) if frame else
                         np.zeros((camera.height, camera.width, 3), np.uint8))
                if image is None or image.shape[:2] != (camera.height, camera.width):
                    raise ValueError(f"Missing/mismatched original image: {frame}")
                labels = []
                for h, side in enumerate(("L", "R")):
                    if mesh_edges is not None and np.asarray(final["valid"])[t, h]:
                        vertices = np.array(final["vertices_m"][t, h], copy=True)
                        if c:
                            vertices = vertices @ inverse[:3, :3].T + inverse[:3, 3]
                        mesh_pixels = camera.project(vertices)
                        mesh_valid = np.isfinite(mesh_pixels).all(-1)
                        mesh_valid &= (np.abs(mesh_pixels) < [camera.width * 4, camera.height * 4]).all(-1)
                        edges = mesh_edges[mesh_valid[mesh_edges].all(-1)]
                        if len(edges):
                            layer = image.copy()
                            lines = np.rint(mesh_pixels[edges]).astype(np.int32)
                            cv2.polylines(layer, lines, False, colors[mesh_stage], 1, cv2.LINE_AA)
                            image = cv2.addWeighted(layer, .28, image, .72, 0)
                    observed = np.asarray(obs["keypoints2d"])[t, c, h]
                    valid = np.asarray(obs["joint_valid"])[t, c, h]
                    for xy in observed[valid & np.isfinite(observed).all(-1)]:
                        cv2.circle(image, tuple(np.rint(xy).astype(int)), 4, (40, 255, 40), -1)
                    for name, result in results.items():
                        points = np.array(result["joints3d_m"][t, h], copy=True)
                        mask = np.asarray(result["valid"])[t, h]
                        mask = np.broadcast_to(mask, (21,))
                        if c:
                            points = points @ inverse[:3, :3].T + inverse[:3, 3]
                        pixels = camera.project(points)
                        good = mask & np.isfinite(pixels).all(-1)
                        for a, b in EDGES:
                            if good[a] and good[b]:
                                cv2.line(image, tuple(np.rint(pixels[a]).astype(int)),
                                         tuple(np.rint(pixels[b]).astype(int)), colors[name], 2)
                        source = str(np.asarray(result.get("source_status", [["stereo"]*2]*len(times)))[t, h])
                        if not good.any():
                            source = "invalid/geometry_or_missing"
                        labels.append(f"{side}/{name}: {int(good.sum())}/21 {source}")
                skew = np.asarray(obs.get("skew_ms", np.zeros(len(times))))[t]
                texts = [f"{sequence['camera_id']} frame={frame_id} t={times[t]:.6f}s skew={np.max(np.abs(skew)):.3f}ms",
                          "green=2D obs yellow=stereo blue=MANO magenta=temporal"
                          + (" (wireframe)" if mesh_edges is not None else "")] + labels
                for i, text in enumerate(texts):
                    cv2.putText(image, text, (12, 25+24*i), cv2.FONT_HERSHEY_SIMPLEX, .48, (255,255,255), 1, cv2.LINE_AA)
                writer.stdin.write(np.ascontiguousarray(image).tobytes())
        except BaseException:
            writer.terminate()
            raise
        finally:
            writer.stdin.close()
            code = writer.wait()
            log.close()
        if code:
            raise RuntimeError(f"ffmpeg exited {code}; see {path.with_suffix('.encode.log')}")


def plot_losses(result, prefix):
    history = result.get("loss_curves", result.get("loss_history", result.get("losses")))
    if history is None:
        return
    from deployment.infer import atomic_json
    serial = history.tolist() if isinstance(history, np.ndarray) else history
    atomic_json(Path(str(prefix) + "_losses.json"), serial)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    if isinstance(history, dict):
        for key, values in history.items():
            ax.plot(values, label=key)
    elif len(history) and isinstance(history[0], dict):
        for key in history[0]:
            if key not in ("stage", "iteration"):
                ax.plot([row.get(key, np.nan) for row in history], label=key)
    else:
        ax.plot(history)
    ax.set(xlabel="optimizer iteration", ylabel="loss (component-specific units)")
    if ax.get_legend_handles_labels()[0]:
        ax.legend()
    fig.savefig(str(prefix) + "_losses.png")
    plt.close(fig)
