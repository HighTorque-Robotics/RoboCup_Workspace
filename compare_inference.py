"""
对比 Python (ONNX Runtime) 与 C++ (TensorRT) 推理结果
使用方法: python3 compare_inference.py
"""
import cv2
import numpy as np
import onnxruntime as ort

# === 参数（与 C++ yolo26.hpp 一致）===
INPUT_W = 640
INPUT_H = 640
CLASS_NUM = 8
NUM_ANCHORS = 8400
THRESHOLD = 0.25
NMS_THRESHOLD = 0.3
MODEL_PATH = "models/best_yolo26s.onnx"

CLASS_NAMES = ["ball", "goalpost", "robot", "L-Intersection",
               "T-Intersection", "X-Intersection", "crossbar", "obstacle"]

COLORS = [
    (0, 255, 0), (0, 255, 255), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (255, 0, 0), (128, 255, 0), (0, 128, 255),
]


def preprocess_img(img, input_w, input_h):
    """与 C++ preprocess_img (utils.cpp) 完全一致"""
    r_w = input_w / (img.shape[1] * 1.0)
    r_h = input_h / (img.shape[0] * 1.0)
    if r_h > r_w:
        w = input_w
        h = int(r_w * img.shape[0])
        x = 0
        y = (input_h - h) // 2
    else:
        w = int(r_h * img.shape[1])
        h = input_h
        x = (input_w - w) // 2
        y = 0
    re = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    out = np.full((input_h, input_w, 3), 128, dtype=np.uint8)
    out[y:y+h, x:x+w] = re
    return out


def prepare_input(img):
    """BGR + normalize，与 C++ yolo26::detect() 一致"""
    # Handle stereo camera: crop right half if image is very wide
    if img.shape[1] > img.shape[0] * 2:
        half_w = img.shape[1] // 2
        img = img[:, half_w:]  # take right half
    pr_img = preprocess_img(img, INPUT_W, INPUT_H)
    # Keep BGR, normalize to [0, 1], NCHW layout
    data = pr_img.astype(np.float32) / 255.0
    data = np.transpose(data, (2, 0, 1))  # HWC to CHW
    data = np.expand_dims(data, axis=0)  # add batch dim
    return data.astype(np.float32)


def decode_output(output, img_w, img_h, threshold, nms_threshold):
    """与 C++ yolo26::decodeOutput() 完全一致"""
    # Compute letterbox scale/offset (same as preprocess_img)
    r_w = INPUT_W / (img_w * 1.0)
    r_h = INPUT_H / (img_h * 1.0)
    scale = min(r_w, r_h)
    pad_x = (INPUT_W - img_w * scale) / 2.0
    pad_y = (INPUT_H - img_h * scale) / 2.0

    detections = []
    for i in range(NUM_ANCHORS):
        cx = output[0, 0, i]
        cy = output[0, 1, i]
        w = output[0, 2, i]
        h = output[0, 3, i]

        # Find max class score
        scores = output[0, 4:4+CLASS_NUM, i]
        best_class = int(np.argmax(scores))
        max_score = float(scores[best_class])

        if max_score < threshold:
            continue

        # Convert from letterbox back to original image coords
        x1 = (cx - w / 2.0 - pad_x) / scale
        y1 = (cy - h / 2.0 - pad_y) / scale
        x2 = (cx + w / 2.0 - pad_x) / scale
        y2 = (cy + h / 2.0 - pad_y) / scale

        # Clip to image bounds
        x1 = max(0.0, min(x1, float(img_w) - 1.0))
        y1 = max(0.0, min(y1, float(img_h) - 1.0))
        x2 = max(0.0, min(x2, float(img_w) - 1.0))
        y2 = max(0.0, min(y2, float(img_h) - 1.0))

        if x2 <= x1 or y2 <= y1:
            continue

        # Store as cx, cy, w, h in original image space
        out_cx = (x1 + x2) / 2.0
        out_cy = (y1 + y2) / 2.0
        out_w = x2 - x1
        out_h = y2 - y1

        detections.append({
            "bbox": [out_cx, out_cy, out_w, out_h],
            "conf": max_score,
            "class_id": best_class,
        })

    # NMS (same logic as C++ yolo26::decodeOutput)
    detections.sort(key=lambda d: d["conf"], reverse=True)
    suppressed = [False] * len(detections)
    result = []

    for i in range(len(detections)):
        if suppressed[i]:
            continue
        result.append(detections[i])

        ib = detections[i]["bbox"]
        i_area = ib[2] * ib[3]

        for j in range(i + 1, len(detections)):
            if suppressed[j]:
                continue
            if detections[j]["class_id"] != detections[i]["class_id"]:
                continue

            jb = detections[j]["bbox"]
            inter_x1 = max(ib[0] - ib[2] / 2.0, jb[0] - jb[2] / 2.0)
            inter_y1 = max(ib[1] - ib[3] / 2.0, jb[1] - jb[3] / 2.0)
            inter_x2 = min(ib[0] + ib[2] / 2.0, jb[0] + jb[2] / 2.0)
            inter_y2 = min(ib[1] + ib[3] / 2.0, jb[1] + jb[3] / 2.0)

            if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
                continue

            inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
            j_area = jb[2] * jb[3]
            iou = inter_area / (i_area + j_area - inter_area)

            if iou > nms_threshold:
                suppressed[j] = True

    return result


def draw_detections(img, detections):
    """与 C++ draw_detections (带 class_labels 版本) 一致"""
    display = img.copy()
    for det in detections:
        bbox = det["bbox"]
        x1 = int(bbox[0] - bbox[2] / 2)
        y1 = int(bbox[1] - bbox[3] / 2)
        x2 = int(bbox[0] + bbox[2] / 2)
        y2 = int(bbox[1] + bbox[3] / 2)

        cls_id = det["class_id"]
        color = COLORS[cls_id % len(COLORS)]
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)

        label = f"{int(det['conf'] * 100)}%+{CLASS_NAMES[cls_id]}"
        cv2.putText(display, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)

    return display


def main():
    print(f"Loading model: {MODEL_PATH}")
    session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])

    # Grab image from camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Cannot open camera, trying /dev/video1...")
        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("No camera found!")
        return

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Camera: {actual_w}x{actual_h}")
    print("Press 'q' to quit, 'c' to capture and compare")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Preprocess
        input_data = prepare_input(frame)

        # Inference
        outputs = session.run(None, {"images": input_data})
        output = outputs[0]  # shape [1, 12, 8400]

        # Decode
        detections = decode_output(output, frame.shape[1], frame.shape[0],
                                   THRESHOLD, NMS_THRESHOLD)

        # Draw
        display = draw_detections(frame, detections)

        # Show info
        info = f"Python ONNX | detections={len(detections)} | threshold={THRESHOLD}"
        cv2.putText(display, info, (10, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 255, 255), 2)

        cv2.namedWindow("Python ONNX Inference", cv2.WINDOW_NORMAL)
        cv2.imshow("Python ONNX Inference", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            ts = cv2.getTickCount()
            fname = f"/tmp/compare_{int(ts)}.jpg"
            cv2.imwrite(fname, frame)
            print(f"Saved: {fname}")
            # Print detection details
            for det in detections:
                print(f"  {CLASS_NAMES[det['class_id']]}: "
                      f"conf={det['conf']:.3f} "
                      f"bbox=[{det['bbox'][0]:.1f}, {det['bbox'][1]:.1f}, "
                      f"{det['bbox'][2]:.1f}, {det['bbox'][3]:.1f}]")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
