from __future__ import annotations

import ctypes
import ctypes.util
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


class CudaError(RuntimeError):
    pass


class CudaRuntime:
    """Small CUDA Runtime wrapper so Jetson does not need pycuda/cuda-python."""

    def __init__(self) -> None:
        candidates = [
            ctypes.util.find_library("cudart"),
            "libcudart.so",
            "libcudart.so.11.0",
            "/usr/local/cuda/lib64/libcudart.so",
            "/usr/lib/aarch64-linux-gnu/libcudart.so.11.0",
        ]
        self.lib = None
        errors = []
        for candidate in candidates:
            if not candidate:
                continue
            try:
                self.lib = ctypes.CDLL(candidate)
                break
            except OSError as exc:
                errors.append(f"{candidate}: {exc}")
        if self.lib is None:
            raise CudaError("Cannot load CUDA Runtime library: " + "; ".join(errors))

        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMalloc.restype = ctypes.c_int
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaFree.restype = ctypes.c_int
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaMemcpyAsync.restype = ctypes.c_int
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamCreate.restype = ctypes.c_int
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.restype = ctypes.c_int
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.restype = ctypes.c_int

    def check(self, status: int, operation: str) -> None:
        if status == 0:
            return
        raw = self.lib.cudaGetErrorString(status)
        detail = raw.decode("utf-8", errors="replace") if raw else f"CUDA error {status}"
        raise CudaError(f"{operation}: {detail}")

    def malloc(self, size: int) -> int:
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size), "cudaMalloc")
        return int(pointer.value or 0)

    def free(self, pointer: int) -> None:
        if pointer:
            self.check(self.lib.cudaFree(ctypes.c_void_p(pointer)), "cudaFree")

    def create_stream(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def copy_async(self, dst: int, src: int, size: int, kind: int, stream: ctypes.c_void_p) -> None:
        self.check(
            self.lib.cudaMemcpyAsync(
                ctypes.c_void_p(dst),
                ctypes.c_void_p(src),
                size,
                kind,
                stream,
            ),
            "cudaMemcpyAsync",
        )

    def synchronize(self, stream: ctypes.c_void_p) -> None:
        self.check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")

    def destroy_stream(self, stream: ctypes.c_void_p) -> None:
        if stream and stream.value:
            self.check(self.lib.cudaStreamDestroy(stream), "cudaStreamDestroy")


@dataclass
class Binding:
    index: int
    name: str
    shape: tuple[int, ...]
    dtype: Any
    host: np.ndarray
    device: int
    is_input: bool


class TensorRTEngine:
    def __init__(self, engine_path: str) -> None:
        try:
            import tensorrt as trt
        except Exception as exc:
            raise RuntimeError("TensorRT Python module is unavailable") from exc

        self.trt = trt
        self.cuda = CudaRuntime()
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.stream = self.cuda.create_stream()
        self.bindings: list[Binding] = []
        self.binding_pointers: list[int] = []
        self.closed = False

        with open(engine_path, "rb") as engine_file:
            self.engine = self.runtime.deserialize_cuda_engine(engine_file.read())
        if self.engine is None:
            self.close()
            raise RuntimeError(f"TensorRT cannot deserialize engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            self.close()
            raise RuntimeError("TensorRT cannot create execution context")

        try:
            dtype_map = {
                trt.float32: np.float32,
                trt.float16: np.float16,
                trt.int8: np.int8,
                trt.int32: np.int32,
                trt.bool: np.bool_,
            }
            for index in range(self.engine.num_bindings):
                shape = tuple(int(value) for value in self.engine.get_binding_shape(index))
                if not shape or any(value <= 0 for value in shape):
                    raise RuntimeError(
                        f"Only static TensorRT bindings are supported; "
                        f"{self.engine.get_binding_name(index)} has shape {shape}"
                    )
                trt_dtype = self.engine.get_binding_dtype(index)
                if trt_dtype not in dtype_map:
                    raise RuntimeError(f"Unsupported TensorRT binding dtype: {trt_dtype}")
                dtype = np.dtype(dtype_map[trt_dtype])
                host = np.empty(shape, dtype=dtype)
                device = self.cuda.malloc(host.nbytes)
                binding = Binding(
                    index=index,
                    name=self.engine.get_binding_name(index),
                    shape=shape,
                    dtype=dtype,
                    host=host,
                    device=device,
                    is_input=bool(self.engine.binding_is_input(index)),
                )
                self.bindings.append(binding)
                self.binding_pointers.append(device)
        except Exception:
            self.close()
            raise

        self.inputs = [binding for binding in self.bindings if binding.is_input]
        self.outputs = [binding for binding in self.bindings if not binding.is_input]
        if len(self.inputs) != 1 or not self.outputs:
            self.close()
            raise RuntimeError(
                f"Expected one input and at least one output, got "
                f"{len(self.inputs)} input(s), {len(self.outputs)} output(s)"
            )

    @property
    def input_shape(self) -> tuple[int, ...]:
        return self.inputs[0].shape

    @property
    def output_shapes(self) -> list[tuple[int, ...]]:
        return [binding.shape for binding in self.outputs]

    def infer(self, input_array: np.ndarray) -> list[np.ndarray]:
        if self.closed:
            raise RuntimeError("TensorRT engine is closed")

        input_binding = self.inputs[0]
        array = np.ascontiguousarray(input_array, dtype=input_binding.dtype)
        if array.shape != input_binding.shape:
            raise ValueError(f"Expected input {input_binding.shape}, got {array.shape}")

        self.cuda.copy_async(
            input_binding.device,
            int(array.ctypes.data),
            array.nbytes,
            CUDA_MEMCPY_HOST_TO_DEVICE,
            self.stream,
        )
        ok = self.context.execute_async_v2(
            bindings=self.binding_pointers,
            stream_handle=int(self.stream.value or 0),
        )
        if not ok:
            raise RuntimeError("TensorRT execute_async_v2 returned false")

        for binding in self.outputs:
            self.cuda.copy_async(
                int(binding.host.ctypes.data),
                binding.device,
                binding.host.nbytes,
                CUDA_MEMCPY_DEVICE_TO_HOST,
                self.stream,
            )
        self.cuda.synchronize(self.stream)
        return [binding.host.copy() for binding in self.outputs]

    def close(self) -> None:
        if getattr(self, "closed", False):
            return
        self.closed = True
        for binding in getattr(self, "bindings", []):
            try:
                self.cuda.free(binding.device)
            except Exception:
                pass
        self.bindings = []
        try:
            self.cuda.destroy_stream(getattr(self, "stream", ctypes.c_void_p()))
        except Exception:
            pass
        self.context = None
        self.engine = None
        self.runtime = None


def preprocess_quadron(frame: np.ndarray, image_size: int = 1280) -> np.ndarray:
    # Keep the exact normalization used by the existing ONNX path.
    return cv2.dnn.blobFromImage(
        frame,
        1.0 / 122.0,
        (image_size, image_size),
        (120, 120, 120),
        True,
        False,
        cv2.CV_32F,
    )


def vectorized_quadron_nms(
    output: np.ndarray,
    frame_width: int,
    frame_height: int,
    confidence_threshold: float,
    minimum_box_px: int,
    previous_center: tuple[int, int] | None,
    image_size: int = 1280,
    nms_threshold: float = 0.5,
) -> dict[str, Any]:
    predictions = np.asarray(output).reshape(-1, np.asarray(output).shape[-1])
    if predictions.shape[1] < 6:
        raise ValueError(f"Unexpected Quadron output shape: {np.asarray(output).shape}")

    class_scores = predictions[:, 5:].max(axis=1)
    confidence = predictions[:, 4] * class_scores
    mask = (predictions[:, 4] >= confidence_threshold) & (confidence >= confidence_threshold)
    filtered = predictions[mask]
    confidence = confidence[mask]
    raw_count = int(filtered.shape[0])
    if raw_count == 0:
        return {"raw_count": 0, "kept_count": 0, "candidates": [], "selected": None}

    xywh = filtered[:, :4].astype(np.float32, copy=True)
    xywh[:, 0] -= xywh[:, 2] * 0.5
    xywh[:, 1] -= xywh[:, 3] * 0.5
    keep = cv2.dnn.NMSBoxes(
        xywh.tolist(),
        confidence.astype(float).tolist(),
        float(confidence_threshold),
        float(nms_threshold),
    )
    if len(keep) == 0:
        return {"raw_count": raw_count, "kept_count": 0, "candidates": [], "selected": None}

    indices = np.asarray(keep).reshape(-1).astype(np.int32)
    kept_boxes = xywh[indices]
    kept_confidence = confidence[indices]
    scale_x = frame_width / float(image_size)
    scale_y = frame_height / float(image_size)

    x1 = np.clip(np.rint(kept_boxes[:, 0] * scale_x), 0, max(0, frame_width - 1)).astype(np.int32)
    y1 = np.clip(np.rint(kept_boxes[:, 1] * scale_y), 0, max(0, frame_height - 1)).astype(np.int32)
    x2 = np.clip(
        np.rint((kept_boxes[:, 0] + kept_boxes[:, 2]) * scale_x),
        0,
        max(0, frame_width - 1),
    ).astype(np.int32)
    y2 = np.clip(
        np.rint((kept_boxes[:, 1] + kept_boxes[:, 3]) * scale_y),
        0,
        max(0, frame_height - 1),
    ).astype(np.int32)
    widths = x2 - x1
    heights = y2 - y1
    areas = widths * heights
    valid = (widths >= minimum_box_px) & (heights >= minimum_box_px) & (areas > 0)

    x1, y1, x2, y2 = x1[valid], y1[valid], x2[valid], y2[valid]
    areas = areas[valid]
    kept_confidence = kept_confidence[valid]
    if len(areas) == 0:
        return {
            "raw_count": raw_count,
            "kept_count": int(len(indices)),
            "candidates": [],
            "selected": None,
        }

    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    ranking = kept_confidence * 1000.0 + np.sqrt(areas.astype(np.float32)) * 0.7
    if previous_center is not None:
        distance = np.hypot(center_x - previous_center[0], center_y - previous_center[1])
        ranking -= np.minimum(distance, 1000.0) * 0.6
        ranking += (distance <= 260.0) * 180.0

    candidates = [
        {
            "box": (int(ax1), int(ay1), int(ax2), int(ay2)),
            "confidence": float(conf),
            "area": int(area),
        }
        for ax1, ay1, ax2, ay2, conf, area in zip(x1, y1, x2, y2, kept_confidence, areas)
    ]
    selected_index = int(np.argmax(ranking))
    return {
        "raw_count": raw_count,
        "kept_count": int(len(indices)),
        "candidates": candidates,
        "selected": candidates[selected_index],
    }


class TensorRTInferenceWorker:
    def __init__(self, engine: TensorRTEngine, image_size: int = 1280) -> None:
        self.engine = engine
        self.image_size = image_size
        self.jobs: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=1)
        self.results: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.previous_center: tuple[int, int] | None = None
        self.thread = threading.Thread(target=self._run, name="quadron-tensorrt", daemon=True)
        self.thread.start()

    @staticmethod
    def _replace(queue_object: queue.Queue, value: Any) -> None:
        try:
            queue_object.put_nowait(value)
            return
        except queue.Full:
            pass
        try:
            queue_object.get_nowait()
        except queue.Empty:
            pass
        try:
            queue_object.put_nowait(value)
        except queue.Full:
            pass

    def submit(
        self,
        frame: np.ndarray,
        confidence_threshold: float,
        minimum_box_px: int,
        single: bool = False,
    ) -> None:
        job = {
            "frame": frame.copy(),
            "confidence_threshold": confidence_threshold,
            "minimum_box_px": minimum_box_px,
            "single": single,
            "submitted_at": time.perf_counter(),
        }
        self._replace(self.jobs, job)

    def latest_result(self) -> dict[str, Any] | None:
        latest = None
        while True:
            try:
                latest = self.results.get_nowait()
            except queue.Empty:
                return latest

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                job = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            if job is None:
                break
            started = time.perf_counter()
            try:
                frame = job["frame"]
                blob = preprocess_quadron(frame, self.image_size)
                preprocess_done = time.perf_counter()
                outputs = self.engine.infer(blob)
                inference_done = time.perf_counter()
                detection = vectorized_quadron_nms(
                    outputs[0],
                    frame.shape[1],
                    frame.shape[0],
                    job["confidence_threshold"],
                    job["minimum_box_px"],
                    self.previous_center,
                    self.image_size,
                )
                selected = detection["selected"]
                if selected is not None:
                    x1, y1, x2, y2 = selected["box"]
                    self.previous_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                else:
                    self.previous_center = None
                result = {
                    **detection,
                    "frame_size": (frame.shape[1], frame.shape[0]),
                    "single": job["single"],
                    "preprocess_ms": (preprocess_done - started) * 1000.0,
                    "inference_ms": (inference_done - preprocess_done) * 1000.0,
                    "total_ms": (time.perf_counter() - started) * 1000.0,
                    "queue_ms": (started - job["submitted_at"]) * 1000.0,
                    "error": None,
                }
            except Exception as exc:
                result = {
                    "single": job.get("single", False),
                    "error": str(exc),
                    "frame_size": None,
                }
            self._replace(self.results, result)

    def close(self) -> None:
        self.stop_event.set()
        self._replace(self.jobs, None)
        self.thread.join(timeout=2.0)
        self.engine.close()
