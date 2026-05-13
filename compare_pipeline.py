"""
C++ 与 Python 推理管线对比
用法: python3 compare_pipeline.py <图片路径>
从 C++ zed_detect 按 'c' 截图后，用此脚本对同一张图做 Python ONNX 推理并对比坐标
"""
import cv2
import numpy as np
import onnxruntime as ort
import sys

INPUT_W = 640
INPUT_H = 640
CLASS_NUM = 8
NUM_ANCHORS = 8400
THRESHOLD = 0.35
NMS_THRESHOLD = 0.3

CLASS_NAMES = ["ball", "goalpost", "robot", "L-Intersection",
               "T-Intersection", "X-Intersection", "crossbar", "obstacle"]

# === C++ preprocess_img (utils.cpp) ===
def preprocess_img(img, input_w, input_h):
    r_w = input_w / (img.shape[1] * 1.0)
    r_h = input_h / (img.shape[0] * 1.0)
    if r_h > r_w:
        w, h = input_w, int(r_w * img.shape[0])
        x, y = 0, (input_h - h) // 2
    else:
        w, h = int(r_h * img.shape[1]), input_h
        x, y = (input_w - w) // 2, 0
    re = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    out = np.full((input_h, input_w, 3), 128, dtype=np.uint8)
    out[y:y+h, x:x+w] = re
    return out

# === C++ yolo26::decodeOutput() ===
def decode_output(output, img_w, img_h):
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

        scores = output[0, 4:4+CLASS_NUM, i]
        best_class = int(np.argmax(scores))
        max_score = float(scores[best_class])
        if max_score < THRESHOLD:
            continue

        x1 = (cx - w / 2.0 - pad_x) / scale
        y1 = (cy - h / 2.0 - pad_y) / scale
        x2 = (cx + w / 2.0 - pad_x) / scale
        y2 = (cy + h / 2.0 - pad_y) / scale

        x1 = max(0.0, min(x1, float(img_w) - 1.0))
        y1 = max(0.0, min(y1, float(img_h) - 1.0))
        x2 = max(0.0, min(x2, float(img_w) - 1.0))
        y2 = max(0.0, min(y2, float(img_h) - 1.0))
        if x2 <= x1 or y2 <= y1:
            continue

        out_cx = (x1 + x2) / 2.0
        out_cy = (y1 + y2) / 2.0
        out_w = x2 - x1
        out_h = y2 - y1

        detections.append({
            "bbox": [out_cx, out_cy, out_w, out_h],
            "conf": max_score,
            "class_id": best_class,
        })

    # NMS
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
            if iou > NMS_THRESHOLD:
                suppressed[j] = True
    return result


# === C++ get_rect() ===
def get_rect_cpp(img, bbox):
    """完全仿照 C++ common.cpp get_rect()"""
    r_w = INPUT_W / (img.shape[1] * 1.0)
    r_h = INPUT_H / (img.shape[0] * 1.0)
    l = bbox[0] - bbox[2] / 2.0
    r = bbox[0] + bbox[2] / 2.0
    t = bbox[1] - bbox[3] / 2.0
    b = bbox[1] + bbox[3] / 2.0
    l = l / r_w
    r = r / r_w
    t = t / r_w   # BUG: C++ uses r_w, should be r_h
    b = b / r_w   # BUG: C++ uses r_w, should be r_h
    return (int(l), int(t), int(r - l), int(b - t))


def get_rect_correct(img, bbox):
    """修正版 get_rect：Y 轴用 r_h"""
    r_w = INPUT_W / (img.shape[1] * 1.0)
    r_h = INPUT_H / (img.shape[0] * 1.0)
    l = bbox[0] - bbox[2] / 2.0
    r = bbox[0] + bbox[2] / 2.0
    t = bbox[1] - bbox[3] / 2.0
    b = bbox[1] + bbox[3] / 2.0
    l = l / r_w
    r = r / r_w
    t = t / r_h   # FIXED
    b = b / r_h   # FIXED
    return (int(l), int(t), int(r - l), int(b - t))


def get_rect_direct(bbox):
    """直接转换（bbox 已在原图空间，无需缩放）"""
    x1 = int(bbox[0] - bbox[2] / 2)
    y1 = int(bbox[1] - bbox[3] / 2)
    x2 = int(bbox[0] + bbox[2] / 2)
    y2 = int(bbox[1] + bbox[3] / 2)
    return (x1, y1, x2 - x1, y2 - y1)


def main():
    if len(sys.argv) < 2:
        print("用法: python3 compare_pipeline.py <图片路径>")
        sys.exit(1)

    img_path = sys.argv[1]
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法读取图片: {img_path}")
        sys.exit(1)

    img_h, img_w = img.shape[:2]
    print(f"图片大小: {img_w}x{img_h}")
    print(f"r_w = {INPUT_W} / {img_w} = {INPUT_W/img_w:.3f}")
    print(f"r_h = {INPUT_H} / {img_h} = {INPUT_H/img_h:.3f}")
    print(f"scale = {min(INPUT_W/img_w, INPUT_H/img_h):.3f}")
    pad_y = (INPUT_H - img_h * min(INPUT_W/img_w, INPUT_H/img_h)) / 2.0
    print(f"pad_y = {pad_y:.1f}")

    # 预处理
    pr_img = preprocess_img(img, INPUT_W, INPUT_H)
    data = pr_img.astype(np.float32) / 255.0
    data = np.transpose(data, (2, 0, 1))
    data = np.expand_dims(data, axis=0).astype(np.float32)

    # ONNX 推理
    import os
    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models/best_yolo26s.onnx")
    session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
    outputs = session.run(None, {"images": data})
    output = outputs[0]

    detections = decode_output(output, img_w, img_h)

    print(f"\n=== 检测结果 ({len(detections)} 个) ===")
    for i, det in enumerate(detections):
        bbox = det["bbox"]
        r_cpp = get_rect_cpp(img, bbox)
        r_correct = get_rect_correct(img, bbox)
        r_direct = get_rect_direct(bbox)

        diff_correct = (r_cpp[0] - r_correct[0], r_cpp[1] - r_correct[1],
                        r_cpp[2] - r_correct[2], r_cpp[3] - r_correct[3])
        diff_direct = (r_cpp[0] - r_direct[0], r_cpp[1] - r_direct[1],
                       r_cpp[2] - r_direct[2], r_cpp[3] - r_direct[3])

        print(f"\n  [{i}] {CLASS_NAMES[det['class_id']]} conf={det['conf']:.3f}")
        print(f"      bbox [cx,cy,w,h]: [{bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f}]")
        print(f"      C++ get_rect:     {r_cpp}")
        print(f"      Fixed get_rect:   {r_correct}  diff={diff_correct}")
        print(f"      Direct (no r):    {r_direct}  diff={diff_direct}")

    # 可视化：三列对比
    display_cpp = img.copy()
    display_correct = img.copy()
    display_direct = img.copy()

    for det in detections:
        bbox = det["bbox"]
        cls_id = det["class_id"]
        conf = det["conf"]
        label = f"{int(conf*100)}%{CLASS_NAMES[cls_id]}"

        r_cpp = get_rect_cpp(img, bbox)
        cv2.rectangle(display_cpp, (r_cpp[0], r_cpp[1]),
                      (r_cpp[0]+r_cpp[2], r_cpp[1]+r_cpp[3]), (0, 255, 0), 2)
        cv2.putText(display_cpp, f"C++: {label}", (r_cpp[0], r_cpp[1]-4),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 255, 0), 2)

        r_correct = get_rect_correct(img, bbox)
        cv2.rectangle(display_correct, (r_correct[0], r_correct[1]),
                      (r_correct[0]+r_correct[2], r_correct[1]+r_correct[3]), (255, 0, 0), 2)
        cv2.putText(display_correct, f"Fixed: {label}", (r_correct[0], r_correct[1]-4),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 0, 0), 2)

        r_direct = get_rect_direct(bbox)
        cv2.rectangle(display_direct, (r_direct[0], r_direct[1]),
                      (r_direct[0]+r_direct[2], r_direct[1]+r_direct[3]), (0, 0, 255), 2)
        cv2.putText(display_direct, f"Direct: {label}", (r_direct[0], r_direct[1]-4),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2)

    comparison = np.hstack([display_cpp, display_correct, display_direct])
    cv2.putText(comparison, "C++ get_rect (GREEN)", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(comparison, "Fixed get_rect (BLUE)", (img_w + 10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(comparison, "Direct bbox (RED)", (img_w * 2 + 10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.namedWindow("C++ vs Fixed vs Direct", cv2.WINDOW_NORMAL)
    cv2.imshow("C++ vs Fixed vs Direct", comparison)
    print(f"\n图像大小: {img_w}x{img_h}")
    print("绿色 = C++ get_rect | 蓝色 = Fixed get_rect | 红色 = Direct bbox")
    print("按任意键退出...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
