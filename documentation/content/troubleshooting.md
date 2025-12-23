---
title: 4. Troubleshooting
---

Common issues and their solutions.

### `ModuleNotFoundError: No module named 'carla'`
*   **Cause:** The `PYTHONPATH` is not set correctly, or the `.so` file in `site-packages` was not renamed.
*   **Fix:** Ensure you exported `PYTHONPATH` pointing to your `venv/lib/python3.12/site-packages` and that `carla.so` exists in that folder (see Installation Guide Step 4).

### `ImportError: ... module compiled using NumPy 1.x cannot be run in NumPy 2.x`
*   **Cause:** You have `numpy` version 2.0 or greater installed, but ROS 2 Jazzy's `cv_bridge` requires NumPy 1.x.
*   **Fix:** Run `pip install "numpy<2"`.

### `NameError: name 'da_path' is not defined`
*   **Cause:** Older code version in `depth_anything.py`.
*   **Fix:** Pull the latest changes or ensure `da_path` is defined as `self.da_path` in the class `__init__` method.
