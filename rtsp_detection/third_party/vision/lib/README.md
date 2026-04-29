Place `libvision.so` here for standalone `rtsp_detection` builds.

Expected default path:

- `applications/rtsp_detection/third_party/vision/lib/libvision.so`

If your shared library is elsewhere, configure with:

```bash
cmake -S . -B build -DVISION_LIBRARY=/path/to/libvision.so
```
