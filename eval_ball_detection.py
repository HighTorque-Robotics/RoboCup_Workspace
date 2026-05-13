"""
足球识别评估脚本
使用方法:
  1. 运行 zed_detect 并用 'a' 键自动保存测试图片
  2. 运行本脚本分析结果:
     python3 eval_ball_detection.py <图片文件夹路径>

  或分析单张图片:
     python3 eval_ball_detection.py <图片路径>

  交互验证模式（逐张标记检测是否正确）:
     python3 eval_ball_detection.py <图片文件夹> --verify
"""

import cv2
import numpy as np
import os
import sys
import json
from pathlib import Path
from collections import Counter


# YOLO 绘制检测框使用的颜色 (BGR)
YOLO_GREEN_LOWER = np.array([30, 100, 30])
YOLO_GREEN_UPPER = np.array([100, 255, 100])

# 红色圆圈的 HSV 范围（球检测器画的红色圆圈）
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])


def find_green_boxes(gray):
    """通过轮廓检测找绿色矩形框（需传入彩色图上的绿色掩码区域）"""
    # 先用 Canny 边缘检测
    edges = cv2.Canny(gray, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        area = w * h
        # 过滤太小的框（噪声）
        if area < 100:
            continue
        # 过滤太大的框（几乎整个图像）
        if area > gray.shape[0] * gray.shape[1] * 0.8:
            continue
        # 过滤长宽比极端的
        aspect_ratio = w / h if h > 0 else 0
        if aspect_ratio > 5 or aspect_ratio < 0.2:
            continue
        boxes.append({"x": x, "y": y, "w": w, "h": h, "area": area, "cx": x + w//2, "cy": y + h//2})

    # 合并重叠框
    merged = merge_overlapping_boxes(boxes)
    return merged


def merge_overlapping_boxes(boxes):
    """合并重叠的边框"""
    if not boxes:
        return []

    boxes = sorted(boxes, key=lambda b: b["area"], reverse=True)
    merged = []

    for box in boxes:
        overlapped = False
        for m in merged:
            # 计算 IOU
            ix = max(box["x"], m["x"])
            iy = max(box["y"], m["y"])
            iw = min(box["x"] + box["w"], m["x"] + m["w"]) - ix
            ih = min(box["y"] + box["h"], m["y"] + m["h"]) - iy
            if iw > 0 and ih > 0:
                intersection = iw * ih
                union = box["area"] + m["area"] - intersection
                iou = intersection / union if union > 0 else 0
                if iou > 0.3:
                    overlapped = True
                    break
        if not overlapped:
            merged.append(box)

    return merged


def find_red_circles(hsv):
    """通过 HSV 颜色检测红色圆圈"""
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 形态学操作去除噪声
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 20:
            continue
        # 计算圆度
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity > 0.3:  # 接近圆形
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circles.append({
                "cx": int(cx), "cy": int(cy),
                "radius": int(radius),
                "area": area,
                "circularity": circularity
            })

    return circles


def analyze_image(image_path):
    """分析单张图片，返回检测统计"""
    img = cv2.imread(str(image_path))
    if img is None:
        return None

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 寻找绿色框（YOLO 检测）
    green_mask = cv2.inRange(img, YOLO_GREEN_LOWER, YOLO_GREEN_UPPER)
    green_boxes = find_green_boxes(green_mask)

    # 寻找红色圆圈（球检测器）
    red_circles = find_red_circles(hsv)

    return {
        "filename": os.path.basename(image_path),
        "width": w,
        "height": h,
        "yolo_detections": len(green_boxes),
        "ball_circles": len(red_circles),
        "yolo_boxes": green_boxes,
        "red_circles": red_circles,
    }


def generate_report(results, output_dir=None, verify_mode=False):
    """生成统计报告"""
    total = len(results)
    if total == 0:
        print("没有找到图片。")
        return

    yolo_counts = [r["yolo_detections"] for r in results]
    circle_counts = [r["ball_circles"] for r in results]

    yolo_dist = Counter(yolo_counts)
    circle_dist = Counter(circle_counts)

    print("=" * 60)
    print("             足球识别评估报告")
    print("=" * 60)
    print(f"总图片数:              {total}")
    print(f"")
    print("--- YOLO 检测结果 ---")
    print(f"平均每帧检测数:        {np.mean(yolo_counts):.2f}")
    print(f"检测数中位数:          {np.median(yolo_counts):.1f}")
    print(f"最多检测数:            {max(yolo_counts)}")
    print(f"最少检测数:            {min(yolo_counts)}")
    print(f"有检测的图片数:        {sum(1 for c in yolo_counts if c > 0)}"
          f" ({sum(1 for c in yolo_counts if c > 0)/total*100:.1f}%)")
    print(f"无检测的图片数:        {yolo_dist.get(0, 0)}"
          f" ({yolo_dist.get(0, 0)/total*100:.1f}%)")
    print(f"")
    print("检测数分布:")
    for k in sorted(yolo_dist):
        print(f"  {k} 个检测: {yolo_dist[k]:4d} 张 ({yolo_dist[k]/total*100:.1f}%)")

    print(f"")
    print("--- 球检测器（红色圆圈）---")
    print(f"平均每帧圆圈数:        {np.mean(circle_counts):.2f}")
    print(f"有圆圈的图片数:        {sum(1 for c in circle_counts if c > 0)}"
          f" ({sum(1 for c in circle_counts if c > 0)/total*100:.1f}%)")

    # 有 YOLO 框但同时有红圈的关联分析
    both = sum(1 for y, c in zip(yolo_counts, circle_counts) if y > 0 and c > 0)
    yolo_only = sum(1 for y, c in zip(yolo_counts, circle_counts) if y > 0 and c == 0)
    circle_only = sum(1 for y, c in zip(yolo_counts, circle_counts) if y == 0 and c > 0)
    print(f"")
    print("--- 两种检测器对比 ---")
    print(f"YOLO + 球检测器一致:  {both} 张 ({both/total*100:.1f}%)")
    print(f"仅有 YOLO 检测:       {yolo_only} 张 ({yolo_only/total*100:.1f}%)")
    print(f"仅有球检测器圆圈:     {circle_only} 张 ({circle_only/total*100:.1f}%)")

    # 保存 CSV
    if output_dir:
        import csv
        csv_path = os.path.join(output_dir, "detection_report.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["filename", "yolo_detections", "ball_circles",
                           "width", "height"])
            for r in results:
                writer.writerow([r["filename"], r["yolo_detections"],
                               r["ball_circles"], r["width"], r["height"]])
        print(f"\nCSV 报告已保存: {csv_path}")

        # 保存 JSON 详细结果
        json_path = os.path.join(output_dir, "detection_details.json")
        with open(json_path, "w") as f:
            json.dump(results, f, indent=2)
        print(f"详细数据已保存: {json_path}")


def verify_mode(image_dir):
    """交互验证模式：逐张检查检测是否正确"""
    extensions = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for ext in extensions:
        files.extend(Path(image_dir).glob(ext))
        files.extend(Path(image_dir).glob(ext.upper()))
    files = sorted(files)

    if not files:
        print(f"在 {image_dir} 中没有找到图片")
        return

    results = []
    print(f"\n交互验证模式 — 逐张判断检测是否正确")
    print(f"按键:  y=正确检测  n=误报  m=漏报  q=退出 其他=跳过\n")
    print(f"共 {len(files)} 张图片\n")

    for fpath in files:
        img = cv2.imread(str(fpath))
        if img is None:
            continue

        h, w = img.shape[:2]
        # 在图片上显示路径
        display = img.copy()

        analysis = analyze_image(fpath)
        has_detection = analysis["yolo_detections"] > 0

        status_text = "有检测" if has_detection else "无检测"
        cv2.putText(display, f"{fpath.name} - {status_text}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        cv2.imshow("Verification", display)
        key = cv2.waitKey(0) & 0xFF

        label = None
        if key == ord("y"):
            label = "correct"
        elif key == ord("n"):
            label = "false_positive"
        elif key == ord("m"):
            label = "false_negative"
        elif key == ord("q"):
            break
        else:
            label = "skipped"

        results.append({
            "filename": fpath.name,
            "has_detection": has_detection,
            "label": label,
        })
        print(f"  {fpath.name}: {label}")

    cv2.destroyAllWindows()

    # 统计
    total = len(results)
    correct = sum(1 for r in results if r["label"] == "correct")
    fp = sum(1 for r in results if r["label"] == "false_positive")
    fn = sum(1 for r in results if r["label"] == "false_negative")
    skipped = sum(1 for r in results if r["label"] == "skipped")

    print("\n" + "=" * 60)
    print("             验证结果")
    print("=" * 60)
    print(f"总标注数:    {total}")
    print(f"正确检测:    {correct}")
    print(f"误报:        {fp}")
    print(f"漏报:        {fn}")
    print(f"跳过:        {skipped}")

    labeled = total - skipped
    if labeled > 0 and (correct + fp) > 0:
        precision = correct / (correct + fp) * 100
        recall = correct / (correct + fn) * 100 if (correct + fn) > 0 else 0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
        print(f"\n正确检测:    {correct}/{labeled}")
        print(f"精确率:      {precision:.1f}%")
        print(f"召回率:      {recall:.1f}%")
        print(f"F1 分数:     {f1:.1f}")

    # 保存结果
    output_path = os.path.join(image_dir, "verification_results.json")
    with open(output_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n结果已保存: {output_path}")


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 eval_ball_detection.py <图片文件夹路径>")
        print("  python3 eval_ball_detection.py <单张图片路径>")
        print("  python3 eval_ball_detection.py <图片文件夹> --verify")
        sys.exit(1)

    path = sys.argv[1]
    is_verify = "--verify" in sys.argv

    if is_verify:
        verify_mode(path)
        return

    if os.path.isfile(path):
        result = analyze_image(path)
        if result:
            print(json.dumps(result, indent=2))
        else:
            print(f"无法读取图片: {path}")
        return

    # 处理文件夹
    extensions = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for ext in extensions:
        files.extend(Path(path).glob(ext))
        files.extend(Path(path).glob(ext.upper()))

    if not files:
        print(f"在 {path} 中没有找到图片文件")
        sys.exit(1)

    print(f"找到 {len(files)} 张图片，正在分析...")
    results = []
    for i, fpath in enumerate(files):
        result = analyze_image(fpath)
        if result:
            results.append(result)
        if (i + 1) % 20 == 0:
            print(f"  已处理 {i+1}/{len(files)}...")

    generate_report(results, output_dir=path)
    print("\n完成！")


if __name__ == "__main__":
    main()
