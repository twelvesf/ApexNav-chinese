import re

import cv2
import numpy as np
from vlm.coco_classes import COCO_CLASSES
from vlm.detector.yolov7 import YOLOv7Client
from vlm.segmentor.sam import MobileSAMClient
from vlm.detector.grounding_dino import GroundingDINOClient
from vlm.itm.blip2itm import BLIP2ITMClient
from vlm.utils.get_itm_message import get_itm_message

yolov7_detector = YOLOv7Client(port=12184)
blip2_itm = BLIP2ITMClient(port=12182)
sam_segmentor = MobileSAMClient(port=12183)
dino_detector = GroundingDINOClient(port=12181)


def _normalize_label(text):
    text = str(text).lower().replace("#", "")
    text = re.sub(r"[^a-z0-9\s]+", " ", text)
    return " ".join(text.split())


def _label_tokens(text):
    return {token for token in _normalize_label(text).split() if len(token) >= 2}

# 把输入的目标字符串去重，得到一个干净的目标词列表
def _dedupe_labels(labels):
    unique = []
    seen = set()
    for label in labels:
        norm = _normalize_label(label)
        if not norm or norm in seen:
            continue
        seen.add(norm)
        unique.append(str(label).strip())
    return unique

# 把检测器输出的类别名 label_detected，匹配到“主目标列表 + 相似物列表”里，并返回一个统一的标签索引
# 0：主目标 1,2,3：相似物列表里的第几个 None：没匹配上
def _match_label_index(label_detected, primary_labels, all_labels):
    detected_norm = _normalize_label(label_detected)
    detected_tokens = _label_tokens(label_detected)
    if not detected_tokens:
        return None

    primary_norms = [_normalize_label(label) for label in primary_labels]
    all_norms = [_normalize_label(label) for label in all_labels]

    for norm in primary_norms:
        if detected_norm == norm:
            return 0

    for idx, norm in enumerate(all_norms):
        if detected_norm == norm:
            return 0 if idx < len(primary_labels) else idx - len(primary_labels) + 1
# 看和主目标有没有词重叠，只要检测标签和主目标在词语上有交集，就优先归到主目标
    best_primary_overlap = max(
        (len(detected_tokens & _label_tokens(label)) for label in primary_labels),
        default=0,
    )
    if best_primary_overlap > 0:
        return 0
# 找与所有候选里词重叠最多的那个
    best_idx = None
    best_overlap = 0
    for idx, label in enumerate(all_labels):
        overlap = len(detected_tokens & _label_tokens(label))
        if overlap > best_overlap:
            best_overlap = overlap
            best_idx = idx

    if best_idx is None or best_overlap == 0:
        return None
    return 0 if best_idx < len(primary_labels) else best_idx - len(primary_labels) + 1

# 把某个检测框交给 SAM 做分割，得到目标 mask，并顺手把分割结果和检测框画到可视化图上。
# 输入：
# segmented_img用来画可视化结果的图 idx当前处理的是第几个检测结果
# detections检测器输出（框、分数、类别） img原始图像
# label当前检测到的类别名 score当前检测分数 color画框和轮廓时用的颜色

# 输出：
# segmented_img：带有框、轮廓、标签的图
# object_mask：当前目标的 mask
def get_segmentation(segmented_img, idx, detections, img, label, score, color):
    object_mask = np.zeros((480, 640), dtype=np.uint8)
    # 把归一化框变回图像像素坐标
    bbox_denorm = detections.boxes[idx] * np.array(
        [img.shape[1], img.shape[0], img.shape[1], img.shape[0]]
    )
    x1, y1, x2, y2 = [int(v) for v in bbox_denorm]
    bbox_area = (x2 - x1) * (y2 - y1)
    img_area = img.shape[0] * img.shape[1]
# 如果框面积接近整张图，就不做 SAM 分割了，这种不靠谱，无意义。
    if bbox_area / img_area < 0.99:
        object_mask = sam_segmentor.segment_bbox(img, bbox_denorm.tolist())
        # 把 mask 轮廓画到图上
        contours, _ = cv2.findContours(
            object_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            cv2.drawContours(segmented_img, [contour], 0, color, 4)
        # 画检测框
        cv2.rectangle(
            segmented_img,
            (x1, y1),
            (x2, y2),
            color,
            2,
        )
        # 把标签和分数写在图上
        label_text = f"{label} ({score:.2f})"
        (text_width, text_height), _ = cv2.getTextSize(
            label_text, cv2.FONT_HERSHEY_DUPLEX, 0.7, 2
        )
        label_x = x1
        label_y = y1 - text_height
        cv2.rectangle(
            segmented_img,
            (label_x, label_y - 30),
            (label_x + text_width, label_y + text_height),
            color,
            2,
        )
        cv2.putText(
            segmented_img,
            label_text,
            (label_x, label_y),
            cv2.FONT_HERSHEY_DUPLEX,
            0.7,
            (255, 255, 255),
            1,
        )

    return segmented_img, object_mask

#根据目标词和相似词，在一张图里做目标检测 + 分割，并把结果整理成后续点云生成能用的格式。
# 输入：目标词，输入图像，检测器配置，相似物列表
# 输出：画好了检测/分割结果的可视化图，每个检测结果的分数，每个检测结果对应的mask，每个结果对应的标签索引
def get_object(right_label, img, cfg, similar_answer):
    score_list = []
    object_masks_list = []
    segmented_img = img.copy()
    label_list = []
    coco_label = []
    dino_label = []
# 把输入的目标字符串 right_label 按 | 切开，
# 去掉每个词两边空格，再去重，得到一个干净的目标词列表
    right_label_list = _dedupe_labels(map(str.strip, right_label.split('|')))
    # print(f"right_label_list: {right_label_list}")
    all_answer = _dedupe_labels(right_label_list + similar_answer)
    for label in all_answer:
        if label in COCO_CLASSES:
            coco_label.append(label)
        else:
            dino_label.append(label)
# 若主目标中至少有一个需要用groundingdino处理，则让dino处理所有标签，yolo处理本来属于coco的主目标词
    if any(item in dino_label for item in right_label_list):
        dino_label = all_answer
        coco_label = []
        for label in right_label_list:
            if label in COCO_CLASSES:
                coco_label.append(label)

# 用 YOLO 检测 COCO 类别目标，筛出和主目标/相似物匹配的框，再用 SAM 分割，并保存分数、mask 和标签索引
    if coco_label:
        # yolo做检测
        detections = yolov7_detector.predict(img, agnostic_nms=cfg.yolo.agnostic_nms, 
                                            conf_thres=cfg.yolo.confidence_threshold_yolo, iou_thres=cfg.yolo.iou_threshold_yolo)
        # 遍历每个检测结果
        for idx in range(len(detections.logits)):
            # 取出这个框的类别和分数
            label_detected = detections.phrases[idx]
            score = detections.logits[idx].item()
            # 判断这个类别属于主目标还是相似物还是无关
            matched_idx = _match_label_index(label_detected, right_label_list, all_answer)
            if matched_idx == 0:#真正目标
                # 对保留下来的结果做sam分割 主目标红色 相似物绿色
                # 返回更新后的可视化图，对应mask
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(255, 0, 0)
                )
                score_list.append(score)
                object_masks_list.append(object_mask)
                label_list.append(0)
            elif matched_idx is not None:
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(0, 255, 0)
                )
                score_list.append(score)
                object_masks_list.append(object_mask)
                label_list.append(matched_idx)
# 用 dino 检测 开放词汇 类别目标，筛出和主目标/相似物匹配的框，再用 SAM 分割，并保存分数、mask 和标签索引
    if dino_label:
        # 把标签列表拼成 DINO 的文本 prompt
        caption = ' '.join(f'{item}.  ' for item in dino_label)
        detections = dino_detector.predict(img, caption=caption, 
                                        box_threshold=cfg.groundingDINO.confidence_threshold_dino, text_threshold=cfg.groundingDINO.text_threshold)
        for idx in range(len(detections.logits)):
            label_detected = detections.phrases[idx]
            score = detections.logits[idx].item()
            matched_idx = _match_label_index(label_detected, right_label_list, all_answer)
            if matched_idx == 0:
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(255, 0, 0)
                )
                score_list.append(score)
                object_masks_list.append(object_mask)
                label_list.append(0)
            elif matched_idx is not None:
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(0, 255, 0)
                )
                score_list.append(score)
                object_masks_list.append(object_mask)
                label_list.append(matched_idx)

    return segmented_img, score_list, object_masks_list, label_list

def get_object_with_itm(label, img, cfg):
    score_list = []
    object_masks_list = []
    cosine_list = []
    itm_score_list = []
    segmented_img = img.copy()
    if label in COCO_CLASSES:
        detections = yolov7_detector.predict(img, agnostic_nms=cfg.yolo.agnostic_nms,
                                             conf_thres=cfg.yolo.confidence_threshold_yolo, iou_thres=cfg.yolo.iou_threshold_yolo)
        for idx in range(len(detections.logits)):
            label_detected = detections.phrases[idx]
            score = detections.logits[idx].item()
            if detections.phrases[idx] == label:
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(255, 0, 0)
                )
                img_detected = crop_and_expand_box(img, detections, idx)
                # cv2.imshow(f"img_detected{idx}", img_detected)
                cosine, itm_score = get_itm_message(img_detected, label)
                print(f"cosine: {cosine:.3f}, itm_score: {itm_score:.3f}")
                score_list.append(score)
                object_masks_list.append(object_mask)
                cosine_list.append(cosine)
                itm_score_list.append(itm_score)

    else:
        detections = dino_detector.predict(img, caption=label, 
                                           box_threshold=cfg.groundingDINO.confidence_threshold_dino, text_threshold=cfg.groundingDINO.text_threshold)
        for idx in range(len(detections.logits)):
            label_detected = detections.phrases[idx]
            score = detections.logits[idx].item()
            if score > cfg.groundingDINO.confidence_threshold_dino:
                segmented_img, object_mask = get_segmentation(
                    segmented_img, idx, detections, img, label_detected, score, color=(255, 0, 0)
                )
                score_list.append(score)
                object_masks_list.append(object_mask)
                img_detected = crop_and_expand_box(img, detections, idx)
                # cv2.imshow(f"img_detected{idx}", img_detected)
                cosine, itm_score = get_itm_message(img_detected, label)
                print(f"cosine: {cosine}, itm_score: {itm_score}")
                cosine_list.append(cosine)
                itm_score_list.append(itm_score)
    
    return segmented_img, score_list, object_masks_list, cosine_list, itm_score_list


def crop_and_expand_box(img, detections, idx, expand_pixels=0.4):
    # Get bounding box coordinates in [x_min, y_min, x_max, y_max] format
    x_min, y_min, x_max, y_max = detections.boxes[idx]
    x_min = int(x_min * img.shape[1])
    y_min = int(y_min * img.shape[0])
    x_max = int(x_max * img.shape[1])
    y_max = int(y_max * img.shape[0])

    # Expand the box outward; clamp to image boundaries
    x_min = max(int(x_min*(1-expand_pixels)), 0)
    y_min = max(int(y_min*(1-expand_pixels)), 0)
    x_max = min(int(x_max*(1+expand_pixels)), img.shape[1] - 1)
    y_max = min(int(y_max*(1+expand_pixels)), img.shape[0] - 1)

    # Crop the image to keep only the box region
    img_detected = img[y_min:y_max+1, x_min:x_max+1]

    return img_detected
