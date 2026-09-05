import importlib

for m in ("numpy", "yaml", "cv2", "rclpy", "cv_bridge", "rich"):
    importlib.import_module(m)
try:
    import ruckig  # noqa: F401
    smoothing = "ruckig"
except Exception:
    smoothing = "clamp-fallback"
import yaml

yaml.safe_load(open("/app/deploy/inference/config.yaml"))
print(f"ebim-task2-offline health: PASS (smoothing={smoothing})")
