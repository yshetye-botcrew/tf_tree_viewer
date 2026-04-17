# tf_tree_viewer
Tf tree visualizer without using ros2

This mimics a *snapshot* of what ROS 2’s TF tree viewers show; it does not run as a continuous interactive GUI.

## Build

Requires **message_manager** installed and discoverable by CMake.

**Botcrew layout:** dependencies (including `message_manager`) are often under **`/opt/botcrew`**. Point CMake at that prefix:

```bash
export CMAKE_PREFIX_PATH="/opt/botcrew:${CMAKE_PREFIX_PATH}"
cmake -S . -B build
cmake --build build
```

Other installs (custom prefix):

```bash
export CMAKE_PREFIX_PATH="/path/to/message-manager/install:${CMAKE_PREFIX_PATH}"
cmake -S . -B build
cmake --build build
```

Binary: `build/tf_tree_viewer`

## Run

Default **domain 0**, topics **`tf_static`** and **`tf`** (match your DDS topic names—sometimes `rt/tf_static` depending on bridge).

```bash
./build/tf_tree_viewer --duration 5 --dot tf.dot
```

**PDF in one step** (same mechanism ROS 2’s `tf2_tools` / `view_frames` uses: Graphviz `dot -Tpdf`; requires `dot` on `PATH`):

```bash
./build/tf_tree_viewer --duration 5 --pdf tf_tree.pdf
sudo apt install graphviz   # if needed
```

Graphviz layout defaults to **vertical** (`--rankdir TB`, root at top). Use **`--rankdir LR`** for a **horizontal** chain (left to right).

Quick snapshot from static TF only (exits once frames appear):

```bash
./build/tf_tree_viewer --static-only --dot tf_static.dot
```

Subtree from a known root (e.g. `odom` or `camera_init`):

```bash
./build/tf_tree_viewer --root camera_init --duration 2
```

Other domain / topic names:

```bash
./build/tf_tree_viewer --domain-id 0 --static-topic tf_static --dynamic-topic tf --duration 3
```

Full options: `./build/tf_tree_viewer --help`

## How to visualize the TF tree

### 1. PDF directly (closest to `ros2 run tf2_tools view_frames`)

Use **`--pdf out.pdf`**: the tool writes a temporary `.dot` and runs **`dot -Tpdf`** (same core mechanism as typical ROS TF PDF generators). Requires the **graphviz** package (`dot` on `PATH`).

### 2. Graphviz by hand

After generating `tf.dot` (or use `--pdf` above instead):

```bash
# PNG (ImageMagick or any viewer)
dot -Tpng tf.dot -o tf.png
xdg-open tf.png

# SVG (good in browser)
dot -Tsvg tf.dot -o tf.svg
xdg-open tf.svg

# PDF
dot -Tpdf tf.dot -o tf.pdf
```

Install Graphviz if needed (Ubuntu/Debian):

```bash
sudo apt install graphviz
```

### 3. Interactive `.dot` viewer

```bash
sudo apt install xdot
xdot tf.dot
```

### 4. Text tree only

Omit `--dot`; the tool prints an indented tree to the terminal (one section per discovered root frame).

## Limitations

- **Snapshot**: collects for `--duration` seconds (or `--static-only` first data), then exits. Re-run to refresh.
- **Graph** is built from **parent → child** edges only; it does not validate transform consistency or timestamps.
- Topic strings must match your **actual** Fast DDS topic names (check Foxglove / `ros2 topic list` on a bridge).
