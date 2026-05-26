# DeepStream Quadron Detector

This folder contains a minimal DeepStream `nvinfer` setup for `tools/quadron_1280.onnx`.

Build the custom parser on Jetson:

```bash
cd ~/ivank/Berdanka-test/tools/deepstream_quadron
make
```

If the build cannot find `cuda_runtime_api.h`, ensure that the CUDA
development package is installed, or pass the CUDA Toolkit location:

```bash
make CUDA_DIR=/usr/local/cuda-11.4
```

If `deepstream-app` fails to load `libgstrtspserver-1.0.so.0`, install
the missing GStreamer RTSP runtime package:

```bash
sudo apt install libgstrtspserver-1.0-0
```

Run a camera smoke test:

```bash
deepstream-app -c deepstream_app_quadron.txt
```

The parser expects the model output shape used by `quadron_1280.onnx`:

```text
[1, 33600, 6] = [cx, cy, w, h, object_conf, class_score]
```

The preprocessing matches the working C++ OpenCV path:

```text
input size: 1280x1280
scale: 1 / 122
offsets: 120;120;120
color: RGB
```

If DeepStream is installed in a non-default directory, pass it to make:

```bash
make DEEPSTREAM_DIR=/opt/nvidia/deepstream/deepstream-6.1
```

To find the correct include path on Jetson:

```bash
sudo find /opt/nvidia -name nvdsinfer_custom_impl.h
```
