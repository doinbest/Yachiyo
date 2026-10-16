#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颜色粗定位/色环精定位 + 数字同心圆定位 + 三环/码堆姿态纠偏程序。

颜色命令沿用原程序的 HSV 粗定位与色环精定位流程；数字命令先在
整幅灰度图中识别数字 1、2、3 并输出粗中心，对准后再在准心附近
使用 Hough 多重同心圆精定位。颜色模式沿用可配置准心（默认 357,324），
数字同心圆和三环姿态模式固定使用现场记录的准心（367,316）。
所有模式沿用B2帧；色环0D与码堆黑线0E使用独立请求及回复目标。

摄像头固定请求 V4L2 + MJPG，默认 640x480、120 FPS。
采集线程持续覆盖最新帧，主线程不会处理摄像头积压的旧帧。

USB CDC：用 --usb-port 指定实际设备路径，timeout=0.1，write_timeout=0.2。
例如 python3 change.py --usb-port /dev/serial/by-id/实际设备名。
115200仅为CDC线路编码参数，USB数据速率由USB总线决定。
上电后不进行识别；收到 FF B2 Command FF 后才开始识别。

Color：
    0x01 红色
    0x02 黄色
    0x03 蓝色
    0x04 绿色
    0x05 黑色
    0x06 浅蓝

数字同心圆：
    0x0A 数字 1 所在的同心圆
    0x0B 数字 2 所在的同心圆
    0x0C 数字 3 所在的同心圆

三环姿态：
    0x0D 三环中心连线姿态；参考中环中心 (367,316)，参考角度 0°

码堆姿态：
    0x0E 检测色环前方、画面底部横穿左右的黑线。
    与0x0D共用图像参考角（默认视觉X轴0°），右端向下为正。

坐标发送帧固定为 14 字节：
    FF B2 Command Valid CX_H CX_L CY_H CY_L DX_H DX_L DY_H DY_L SUM FE

    Valid：1=检测有效，0=当前未检测到目标
    CX/CY：目标中心像素坐标，无符号 16 位，大端序
    DX：CX - 准心X，有符号 16 位，大端序，向右为正
    DY：准心Y - CY，有符号 16 位，大端序，向上为正
    SUM：从 B2 到 DY_L 所有字节求和后取低 8 位

0x0D 仍使用同一个 14 字节帧，但四个 16 位槽均按有符号大端整数解释：
    EX, EY, EA_X100, STOP
    EA_X100：R1 到 R3 连线角度误差乘 100
    STOP：角度误差绝对值不超过 5° 时为 1，否则为 0

0x0E 复用同一种姿态帧，前两个16位槽固定为0，后两个槽为：
    EA_X100, STOP
    漏检时Valid=0且四槽全零，历史线仅用于显示；重获后重新三帧确认。
    图像参考角相同不表示透视/车体角映射已经实测标定。

按 q 或 Esc 退出。
"""

import argparse
import glob
from itertools import combinations
import math
import os
import struct
import threading
import time

import cv2
import numpy as np
import serial


CAMERA_COLOR_RED = 0x01
CAMERA_COLOR_YELLOW = 0x02
CAMERA_COLOR_BLUE = 0x03
CAMERA_COLOR_GREEN = 0x04
CAMERA_COLOR_BLACK = 0x05
CAMERA_COLOR_LIGHT_BLUE = 0x06

CAMERA_DIGIT_RING_1 = 0x0A
CAMERA_DIGIT_RING_2 = 0x0B
CAMERA_DIGIT_RING_3 = 0x0C
CAMERA_THREE_RING_POSE = 0x0D
CAMERA_PILE_LINE_POSE = 0x0E

VALID_COLOR_IDS = {
    CAMERA_COLOR_RED,
    CAMERA_COLOR_YELLOW,
    CAMERA_COLOR_BLUE,
    CAMERA_COLOR_GREEN,
    CAMERA_COLOR_BLACK,
    CAMERA_COLOR_LIGHT_BLUE,
}

DIGIT_RING_COMMAND_TO_DIGIT = {
    CAMERA_DIGIT_RING_1: 1,
    CAMERA_DIGIT_RING_2: 2,
    CAMERA_DIGIT_RING_3: 3,
}

VALID_COMMAND_IDS = (
    VALID_COLOR_IDS
    | set(DIGIT_RING_COMMAND_TO_DIGIT)
    | {CAMERA_THREE_RING_POSE, CAMERA_PILE_LINE_POSE}
)

COLOR_NAMES = {
    CAMERA_COLOR_RED: "red",
    CAMERA_COLOR_YELLOW: "yellow",
    CAMERA_COLOR_BLUE: "blue",
    CAMERA_COLOR_GREEN: "green",
    CAMERA_COLOR_BLACK: "black",
    CAMERA_COLOR_LIGHT_BLUE: "light_blue",
}

COMMAND_NAMES = {
    **COLOR_NAMES,
    CAMERA_DIGIT_RING_1: "digit_ring_1",
    CAMERA_DIGIT_RING_2: "digit_ring_2",
    CAMERA_DIGIT_RING_3: "digit_ring_3",
    CAMERA_THREE_RING_POSE: "three_ring_pose",
    CAMERA_PILE_LINE_POSE: "pile_line_pose",
}

DRAW_COLORS = {
    CAMERA_COLOR_RED: (0, 0, 255),
    CAMERA_COLOR_YELLOW: (0, 255, 255),
    CAMERA_COLOR_BLUE: (255, 0, 0),
    CAMERA_COLOR_GREEN: (0, 255, 0),
    CAMERA_COLOR_BLACK: (255, 255, 255),
    CAMERA_COLOR_LIGHT_BLUE: (255, 255, 0),
}

# 粗定位使用 wuliao.py 原有的 HSV 范围。
MATERIAL_HSV_RANGES = {
    CAMERA_COLOR_RED: [
        (np.array([0, 100, 70]), np.array([10, 255, 255])),
        (np.array([170, 100, 70]), np.array([180, 255, 255])),
    ],
    CAMERA_COLOR_YELLOW: [
        (np.array([18, 80, 80]), np.array([38, 255, 255])),
    ],
    CAMERA_COLOR_BLUE: [
        (np.array([100, 120, 40]), np.array([135, 255, 255])),
    ],
    CAMERA_COLOR_GREEN: [
        (np.array([40, 55, 40]), np.array([88, 255, 255])),
    ],
    CAMERA_COLOR_BLACK: [
        (np.array([0, 0, 0]), np.array([180, 255, 70])),
    ],
    CAMERA_COLOR_LIGHT_BLUE: [
        (np.array([82, 25, 110]), np.array([105, 150, 255])),
    ],
}

# 精定位使用当前色环识别逻辑的 HSV 范围。
CIRCLE_HSV_RANGES = {
    CAMERA_COLOR_RED: [
        (np.array([0, 130, 130]), np.array([10, 255, 255])),
        (np.array([150, 30, 70]), np.array([180, 255, 255])),
    ],
    CAMERA_COLOR_YELLOW: [
        (np.array([18, 80, 80]), np.array([38, 255, 255])),
    ],
    CAMERA_COLOR_BLUE: [
        (np.array([100, 100, 60]), np.array([132, 255, 255])),
    ],
    CAMERA_COLOR_GREEN: [
        (np.array([40, 30, 50]), np.array([90, 255, 255])),
    ],
    CAMERA_COLOR_BLACK: [
        (np.array([0, 0, 0]), np.array([180, 255, 70])),
    ],
    CAMERA_COLOR_LIGHT_BLUE: [
        (np.array([82, 25, 110]), np.array([105, 150, 255])),
    ],
}


# 数字同心圆检测参数，沿用第一份程序的配置。
GAUSSIAN_SIZE = 3
GAUSSIAN_SIGMA = 0

HOUGH_DP = 1.0
HOUGH_MIN_DIST = 18
HOUGH_PARAM1 = 120
HOUGH_PARAM2 = 0.65
HOUGH_MIN_RADIUS = 15
HOUGH_MAX_RADIUS = 90

CENTER_DISTANCE_THRESHOLD = 12
MIN_RING_COUNT = 3
RADIUS_MERGE_THRESHOLD = 3.0
MIN_GROUP_RADIUS = 25
MAX_GROUP_RADIUS = 90

DIGIT_IMAGE_SIZE = 48
DIGIT_ROI_INNER_RADIUS_RATIO = 0.78
DIGIT_ROI_OUTER_RADIUS_RATIO = 0.50
DIGIT_ROI_MIN_RADIUS = 10
DIGIT_MIN_SCORE = 0.50
DIGIT_MIN_MARGIN = 0.015

DIGIT_COARSE_MIN_HEIGHT_RATIO = 0.065
DIGIT_COARSE_MAX_HEIGHT_RATIO = 0.35
DIGIT_COARSE_MAX_WIDTH_RATIO = 0.28
DIGIT_COARSE_MIN_ASPECT = 0.22
DIGIT_COARSE_MAX_ASPECT = 0.92
DIGIT_COARSE_REFERENCE_AREA = 640.0 * 480.0
DIGIT_COARSE_MIN_AREA_640X480 = 350.0
DIGIT_COARSE_RING_MIN_COUNT = 2
DIGIT_COARSE_RING_CENTER_TOLERANCE = 20.0
COLOR_REFERENCE_CENTER_X = 357
COLOR_REFERENCE_CENTER_Y = 324
DIGIT_RING_REFERENCE_CENTER_X = 367
DIGIT_RING_REFERENCE_CENTER_Y = 316
THREE_RING_REFERENCE_CENTER_X = 367
THREE_RING_REFERENCE_CENTER_Y = 316
THREE_RING_REFERENCE_YAW_DEG = 0.0
THREE_RING_ANGLE_DEADZONE_DEG = 5.0
PILE_LINE_ROI_TOP_RATIO = 0.70
PILE_LINE_ROI_BOTTOM_RATIO = 0.97
PILE_LINE_MIN_SPAN_RATIO = 0.70
PILE_LINE_SIDE_MARGIN_RATIO = 0.15
PILE_LINE_MAX_ABS_ANGLE_DEG = 30.0
PILE_LINE_MAX_GRAY = 145
PILE_LINE_MIN_DARK_SUPPORT_RATIO = 0.75
PILE_LINE_PROFILE_RADIUS = 32
PILE_LINE_MIN_BLACK_CONTRAST = 20.0
PILE_LINE_MIN_BLACK_BAND_SUPPORT_RATIO = 0.60
PILE_LINE_ANGLE_DEADZONE_DEG = 5.0
PILE_LINE_STABLE_FRAMES = 3
PILE_LINE_MAX_ANGLE_JITTER_DEG = 2.0
PILE_LINE_MAX_MISSED_FRAMES = 2
DIGIT_RING_SWITCH_TOLERANCE = 20
DIGIT_RING_ALIGNED_CONFIRM_FRAMES = 3
DIGIT_RING_ROI_HALF_SIZE = 110
DIGIT_RING_HOLD_MISS_FRAMES = 5
DIGIT_RING_ACQUIRE_MISS_FRAMES = 12


def point_distance(point_1, point_2):
    delta_x = point_1[0] - point_2[0]
    delta_y = point_1[1] - point_2[1]
    return math.hypot(delta_x, delta_y)


def remove_duplicate_circles(circles):
    """合并 Hough 在同一条圆边附近产生的重复结果。"""
    circles = sorted(circles, key=lambda circle: circle["radius"])
    result = []

    for circle in circles:
        duplicate = any(
            point_distance(circle["center"], saved["center"])
            <= CENTER_DISTANCE_THRESHOLD
            and abs(circle["radius"] - saved["radius"])
            <= RADIUS_MERGE_THRESHOLD
            for saved in result
        )
        if not duplicate:
            result.append(circle)

    return result


def remove_duplicate_ring_radii(circles):
    """同一同心圆组内只保留相互分离的半径层。"""
    result = []
    for circle in sorted(circles, key=lambda item: item["radius"]):
        duplicate_radius = any(
            abs(circle["radius"] - saved["radius"])
            <= RADIUS_MERGE_THRESHOLD
            for saved in result
        )
        if not duplicate_radius:
            result.append(circle)
    return result


def group_concentric_circles(circles):
    """把圆心距离不超过阈值的圆聚为同心圆候选组。"""
    groups = []
    circles = sorted(circles, key=lambda circle: circle["radius"], reverse=True)

    for circle in circles:
        best_group = None
        best_distance = float("inf")

        for group in groups:
            distance = point_distance(circle["center"], group["center"])
            if distance <= CENTER_DISTANCE_THRESHOLD and distance < best_distance:
                best_group = group
                best_distance = distance

        if best_group is None:
            groups.append({"center": circle["center"], "circles": [circle]})
            continue

        best_group["circles"].append(circle)
        best_group["center"] = (
            float(np.mean([item["center"][0] for item in best_group["circles"]])),
            float(np.mean([item["center"][1] for item in best_group["circles"]])),
        )

    return groups


def validate_ring_groups(groups, min_ring_count=MIN_RING_COUNT):
    """保留达到指定圆边数量且尺寸合规的同心圆组。"""
    valid_groups = []
    min_ring_count = max(1, int(min_ring_count))

    for group in groups:
        circles = remove_duplicate_ring_radii(group["circles"])
        if len(circles) < min_ring_count:
            continue

        circles = sorted(circles, key=lambda circle: circle["radius"])
        max_radius = max(circle["radius"] for circle in circles)
        if max_radius < MIN_GROUP_RADIUS or max_radius > MAX_GROUP_RADIUS:
            continue

        center_x = float(np.mean([circle["center"][0] for circle in circles]))
        center_y = float(np.mean([circle["center"][1] for circle in circles]))
        valid_groups.append(
            {
                "center": (center_x, center_y),
                "circles": circles,
                "min_radius": float(min(circle["radius"] for circle in circles)),
                "max_radius": float(max_radius),
                "detected_digit": None,
                "digit_score": 0.0,
                "digit_margin": 0.0,
            }
        )

    return valid_groups


def detect_hough_circles(gray):
    """使用第一份程序的 HOUGH_GRADIENT_ALT 参数检测圆形边缘。"""
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT_ALT,
        HOUGH_DP,
        HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=HOUGH_MIN_RADIUS,
        maxRadius=HOUGH_MAX_RADIUS,
    )
    if circles is None:
        return []

    return [
        {
            "center": (float(center_x), float(center_y)),
            "radius": float(radius),
        }
        for center_x, center_y, radius in circles.reshape(-1, 3)
    ]


def select_target_ring(valid_groups, target_digit):
    """选择中央数字匹配的最佳同心圆。"""
    candidates = [
        group
        for group in valid_groups
        if group.get("detected_digit") == target_digit
    ]
    if not candidates:
        return None

    return max(
        candidates,
        key=lambda group: (
            group["digit_score"],
            group["digit_margin"],
            group["max_radius"],
        ),
    )


def resolve_command_mode(command):
    """把统一 B2 命令映射到颜色、数字圆环或姿态模式。"""
    if command in VALID_COLOR_IDS:
        return "color", command
    if command in DIGIT_RING_COMMAND_TO_DIGIT:
        return "digit_ring", DIGIT_RING_COMMAND_TO_DIGIT[command]
    if command == CAMERA_THREE_RING_POSE:
        return "three_ring_pose", None
    if command == CAMERA_PILE_LINE_POSE:
        return "pile_line_pose", None
    raise ValueError(f"不支持的 B2 命令：0x{command:02X}")


def build_three_ring_pose_detection(
    centers,
    reference_center=(
        THREE_RING_REFERENCE_CENTER_X,
        THREE_RING_REFERENCE_CENTER_Y,
    ),
    reference_yaw_deg=THREE_RING_REFERENCE_YAW_DEG,
    deadzone_deg=THREE_RING_ANGLE_DEADZONE_DEG,
    r2_center=None,
    ring_labels=None,
):
    """由两或三个圆心计算中环位置误差、连线角度误差和停止标志。"""
    normalized_centers = tuple(
        (float(x), float(y)) for x, y in centers
    )
    if len(normalized_centers) not in (2, 3):
        raise ValueError("姿态计算需要两个或三个圆心")

    if ring_labels is None:
        ordered = tuple(sorted(
            normalized_centers,
            key=lambda point: point[0],
        ))
        labels = (
            ("R1", "R2", "R3")
            if len(ordered) == 3
            else ("R1", "R2")
        )
    else:
        labels = tuple(str(label) for label in ring_labels)
        if len(labels) != len(normalized_centers):
            raise ValueError("圆环标签数量必须与圆心数量一致")
        ordered_items = sorted(
            zip(normalized_centers, labels),
            key=lambda item: item[0][0],
        )
        ordered = tuple(item[0] for item in ordered_items)
        labels = tuple(item[1] for item in ordered_items)
    first, third = ordered[0], ordered[-1]
    if r2_center is None:
        if "R2" not in labels:
            raise ValueError("姿态计算结果中必须包含R2")
        middle = ordered[labels.index("R2")]
    else:
        middle = tuple(float(value) for value in r2_center)
    yaw_deg = math.degrees(
        math.atan2(third[1] - first[1], third[0] - first[0])
    )
    angle_error_deg = (
        yaw_deg - float(reference_yaw_deg) + 180.0
    ) % 360.0 - 180.0
    center_x = int(round(middle[0]))
    center_y = int(round(middle[1]))
    reference_x, reference_y = reference_center
    return {
        "detected": True,
        "state": "three_ring_pose_detected",
        "mode": "three_ring_pose",
        "next_mode": "three_ring_pose",
        "target": "three_ring_pose",
        "center": (center_x, center_y),
        "ring_centers": ordered,
        "ring_count": len(ordered),
        "ring_labels": labels,
        "station_yaw_deg": float(yaw_deg),
        "angle_error_deg": float(angle_error_deg),
        "angle_error_cdeg": int(round(angle_error_deg * 100.0)),
        "stop_correction": abs(angle_error_deg) <= float(deadzone_deg),
        "error_x": int(center_x - reference_x),
        "error_y": int(reference_y - center_y),
        "area": 0.0,
    }


def build_digit_ring_detection(target_group, target_digit, reference_center):
    """把数字同心圆候选转换为与颜色检测一致的坐标结果。"""
    target_name = f"digit_ring_{target_digit}"
    if target_group is None:
        return {
            "detected": False,
            "state": "digit_ring_not_found",
            "mode": "digit_ring",
            "next_mode": "digit_ring",
            "target": target_name,
            "center": None,
            "error_x": 0,
            "error_y": 0,
            "area": 0.0,
            "radius": 0.0,
            "circle_count": 0,
            "digit": target_digit,
            "digit_score": 0.0,
            "digit_margin": 0.0,
        }

    center_x = int(round(target_group["center"][0]))
    center_y = int(round(target_group["center"][1]))
    reference_x, reference_y = reference_center
    return {
        "detected": True,
        "state": "digit_ring_detected",
        "mode": "digit_ring",
        "next_mode": "digit_ring",
        "target": target_name,
        "center": (center_x, center_y),
        "error_x": int(center_x - reference_x),
        "error_y": int(reference_y - center_y),
        "area": 0.0,
        "radius": float(target_group["max_radius"]),
        "circle_count": len(target_group["circles"]),
        "digit": target_digit,
        "digit_score": float(target_group["digit_score"]),
        "digit_margin": float(target_group["digit_margin"]),
    }


class Digit123Recognizer:
    """使用合成字体模板和 HOG 相似度识别圆心内的数字 1、2、3。"""

    def __init__(self, coarse_min_area=DIGIT_COARSE_MIN_AREA_640X480):
        self.coarse_min_area = max(0.0, float(coarse_min_area))
        self.hog = cv2.HOGDescriptor(
            (DIGIT_IMAGE_SIZE, DIGIT_IMAGE_SIZE),
            (16, 16),
            (8, 8),
            (8, 8),
            9,
        )
        self.template_features, self.template_labels = self._build_templates()

    @staticmethod
    def _normalize_glyph(binary):
        binary = np.where(binary > 0, 255, 0).astype(np.uint8)
        points = cv2.findNonZero(binary)
        if points is None:
            return None

        x, y, width, height = cv2.boundingRect(points)
        if width < 1 or height < 3:
            return None

        glyph = binary[y:y + height, x:x + width]
        padding = 5
        available = DIGIT_IMAGE_SIZE - 2 * padding
        scale = min(available / width, available / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
        glyph = cv2.resize(
            glyph,
            (resized_width, resized_height),
            interpolation=interpolation,
        )
        _, glyph = cv2.threshold(glyph, 127, 255, cv2.THRESH_BINARY)

        normalized = np.zeros(
            (DIGIT_IMAGE_SIZE, DIGIT_IMAGE_SIZE),
            dtype=np.uint8,
        )
        offset_x = (DIGIT_IMAGE_SIZE - resized_width) // 2
        offset_y = (DIGIT_IMAGE_SIZE - resized_height) // 2
        normalized[
            offset_y:offset_y + resized_height,
            offset_x:offset_x + resized_width,
        ] = glyph
        return normalized

    def _feature(self, normalized_glyph):
        feature = self.hog.compute(normalized_glyph).reshape(-1).astype(np.float32)
        norm = float(np.linalg.norm(feature))
        if norm <= 1e-8:
            return None
        return feature / norm

    def _minimum_coarse_contour_area(self, frame_area):
        return (
            self.coarse_min_area
            * float(frame_area)
            / DIGIT_COARSE_REFERENCE_AREA
        )

    @staticmethod
    def _render_seven_segment(digit, thickness):
        canvas = np.zeros((96, 72), dtype=np.uint8)
        left, right = 15, 56
        top, middle, bottom = 10, 47, 84
        segments = {
            "a": ((left, top), (right, top)),
            "b": ((right, top), (right, middle)),
            "c": ((right, middle), (right, bottom)),
            "d": ((left, bottom), (right, bottom)),
            "e": ((left, middle), (left, bottom)),
            "f": ((left, top), (left, middle)),
            "g": ((left, middle), (right, middle)),
        }
        active_segments = {1: "bc", 2: "abged", 3: "abgcd"}[digit]

        for segment_name in active_segments:
            start, end = segments[segment_name]
            cv2.line(canvas, start, end, 255, thickness, cv2.LINE_AA)
        return canvas

    def _append_rotated_templates(self, images, labels, image, digit):
        height, width = image.shape
        center = (width / 2.0, height / 2.0)

        for base_angle in (0, 180):
            for offset_angle in (-12, -6, 0, 6, 12):
                matrix = cv2.getRotationMatrix2D(
                    center,
                    base_angle + offset_angle,
                    1.0,
                )
                rotated = cv2.warpAffine(
                    image,
                    matrix,
                    (width, height),
                    flags=cv2.INTER_LINEAR,
                    borderValue=0,
                )
                _, rotated = cv2.threshold(
                    rotated,
                    80,
                    255,
                    cv2.THRESH_BINARY,
                )
                normalized = self._normalize_glyph(rotated)
                if normalized is not None:
                    images.append(normalized)
                    labels.append(digit)

    def _build_templates(self):
        images = []
        labels = []
        fonts = (
            cv2.FONT_HERSHEY_SIMPLEX,
            cv2.FONT_HERSHEY_DUPLEX,
            cv2.FONT_HERSHEY_COMPLEX,
            cv2.FONT_HERSHEY_TRIPLEX,
            cv2.FONT_HERSHEY_PLAIN,
        )

        for digit in (1, 2, 3):
            for font in fonts:
                for thickness in (1, 2, 3, 4):
                    canvas = np.zeros((96, 96), dtype=np.uint8)
                    text = str(digit)
                    scale = 2.3 if font == cv2.FONT_HERSHEY_PLAIN else 2.0
                    (text_width, text_height), baseline = cv2.getTextSize(
                        text,
                        font,
                        scale,
                        thickness,
                    )
                    origin = (
                        (canvas.shape[1] - text_width) // 2,
                        (canvas.shape[0] + text_height - baseline) // 2,
                    )
                    cv2.putText(
                        canvas,
                        text,
                        origin,
                        font,
                        scale,
                        255,
                        thickness,
                        cv2.LINE_AA,
                    )
                    self._append_rotated_templates(
                        images,
                        labels,
                        canvas,
                        digit,
                    )

            for thickness in (4, 6, 8, 10):
                seven_segment = self._render_seven_segment(digit, thickness)
                self._append_rotated_templates(
                    images,
                    labels,
                    seven_segment,
                    digit,
                )

        features = []
        feature_labels = []
        for image, label in zip(images, labels):
            feature = self._feature(image)
            if feature is not None:
                features.append(feature)
                feature_labels.append(label)

        return (
            np.asarray(features, dtype=np.float32),
            np.asarray(feature_labels, dtype=np.int32),
        )

    @staticmethod
    def _extract_candidate(binary):
        binary = binary.copy()
        height, width = binary.shape
        border = max(2, min(height, width) // 20)
        binary[:border, :] = 0
        binary[-border:, :] = 0
        binary[:, :border] = 0
        binary[:, -border:] = 0

        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best_contour = None
        best_score = -1.0
        roi_area = float(width * height)
        roi_center_x = width / 2.0
        roi_center_y = height / 2.0

        for contour in contours:
            x, y, box_width, box_height = cv2.boundingRect(contour)
            area = float(cv2.contourArea(contour))
            if area < roi_area * 0.006 or area > roi_area * 0.55:
                continue
            if box_height < height * 0.28 or box_height > height * 0.96:
                continue
            if box_width < 2 or box_width > width * 0.90:
                continue

            center_distance = math.hypot(
                (x + box_width / 2.0 - roi_center_x) / max(1.0, width),
                (y + box_height / 2.0 - roi_center_y) / max(1.0, height),
            )
            if center_distance > 0.34:
                continue

            contour_score = (
                box_height / max(1.0, height)
                + 0.35 * area / roi_area
                - 0.8 * center_distance
            )
            if contour_score > best_score:
                best_score = contour_score
                best_contour = contour

        if best_contour is None:
            return None

        selection_mask = np.zeros_like(binary)
        cv2.drawContours(selection_mask, [best_contour], -1, 255, -1)
        return cv2.bitwise_and(binary, selection_mask)

    @staticmethod
    def _threshold_candidates(roi):
        roi = cv2.GaussianBlur(roi, (3, 3), 0)
        candidates = []

        for threshold_type in (cv2.THRESH_BINARY_INV, cv2.THRESH_BINARY):
            _, otsu = cv2.threshold(
                roi,
                0,
                255,
                threshold_type | cv2.THRESH_OTSU,
            )
            candidates.append(otsu)

        min_side = min(roi.shape)
        block_size = max(11, (min_side // 2) | 1)
        if block_size >= min_side:
            block_size = min_side - 1 if min_side % 2 == 0 else min_side - 2
        if block_size >= 3:
            candidates.append(
                cv2.adaptiveThreshold(
                    roi,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV,
                    block_size,
                    5,
                )
            )

        return candidates

    def _classify_normalized(self, normalized):
        feature = self._feature(normalized)
        if feature is None:
            return None, 0.0, 0.0

        similarities = self.template_features @ feature
        digit_scores = {}
        for digit in (1, 2, 3):
            values = similarities[self.template_labels == digit]
            top_count = min(6, len(values))
            if top_count == 0:
                digit_scores[digit] = 0.0
                continue
            digit_scores[digit] = float(np.mean(np.partition(values, -top_count)[-top_count:]))

        ranked = sorted(digit_scores.items(), key=lambda item: item[1], reverse=True)
        best_digit, best_score = ranked[0]
        return best_digit, best_score, best_score - ranked[1][1]

    def recognize_group(self, gray, group):
        center_x = int(round(group["center"][0]))
        center_y = int(round(group["center"][1]))
        roi_radius = int(
            round(
                max(
                    DIGIT_ROI_MIN_RADIUS,
                    min(
                        group["min_radius"] * DIGIT_ROI_INNER_RADIUS_RATIO,
                        group["max_radius"] * DIGIT_ROI_OUTER_RADIUS_RATIO,
                    ),
                )
            )
        )

        x_1 = max(0, center_x - roi_radius)
        y_1 = max(0, center_y - roi_radius)
        x_2 = min(gray.shape[1], center_x + roi_radius + 1)
        y_2 = min(gray.shape[0], center_y + roi_radius + 1)
        roi = gray[y_1:y_2, x_1:x_2]
        if roi.shape[0] < 12 or roi.shape[1] < 12:
            return None, 0.0, 0.0

        best_digit = None
        best_score = 0.0
        best_margin = 0.0
        for threshold_image in self._threshold_candidates(roi):
            candidate = self._extract_candidate(threshold_image)
            if candidate is None:
                continue
            normalized = self._normalize_glyph(candidate)
            if normalized is None:
                continue
            digit, score, margin = self._classify_normalized(normalized)
            if score + 0.35 * margin > best_score + 0.35 * best_margin:
                best_digit = digit
                best_score = score
                best_margin = margin

        if (
            best_digit is None
            or best_score < DIGIT_MIN_SCORE
            or best_margin < DIGIT_MIN_MARGIN
        ):
            return None, best_score, best_margin
        return best_digit, best_score, best_margin

    def find_target_candidates(self, gray, target_digit):
        """在整幅灰度图中直接寻找指定数字，不依赖圆环候选。"""
        if target_digit not in (1, 2, 3):
            return []

        height, width = gray.shape
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        threshold_images = []
        _, otsu = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
        )
        threshold_images.append(otsu)

        block_size = max(15, (min(height, width) // 8) | 1)
        if block_size >= min(height, width):
            block_size = min(height, width) - 1
            if block_size % 2 == 0:
                block_size -= 1
        if block_size >= 3:
            threshold_images.append(
                cv2.adaptiveThreshold(
                    blurred,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV,
                    block_size,
                    7,
                )
            )

        candidates = []
        frame_area = float(height * width)
        min_contour_area = self._minimum_coarse_contour_area(frame_area)
        min_height = max(12, int(round(height * DIGIT_COARSE_MIN_HEIGHT_RATIO)))
        max_height = max(min_height + 1, int(round(height * DIGIT_COARSE_MAX_HEIGHT_RATIO)))
        max_width = max(4, int(round(width * DIGIT_COARSE_MAX_WIDTH_RATIO)))

        for binary in threshold_images:
            contours, _ = cv2.findContours(
                binary,
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                x, y, box_width, box_height = cv2.boundingRect(contour)
                if box_height < min_height or box_height > max_height:
                    continue
                if box_width < 3 or box_width > max_width:
                    continue
                aspect = box_width / max(1.0, box_height)
                if (
                    aspect < DIGIT_COARSE_MIN_ASPECT
                    or aspect > DIGIT_COARSE_MAX_ASPECT
                ):
                    continue

                contour_area = float(cv2.contourArea(contour))
                box_area = float(box_width * box_height)
                if contour_area < min_contour_area:
                    continue
                if contour_area / max(1.0, box_area) < 0.06:
                    continue

                # 只分类当前轮廓本身。若直接截取整个矩形，圆环内部的
                # 其它边缘也会混入 HOG，完整圆环可能被误认成数字 3。
                contour_mask = np.zeros(
                    (box_height, box_width),
                    dtype=np.uint8,
                )
                shifted_contour = contour - np.asarray(
                    [[[x, y]]],
                    dtype=contour.dtype,
                )
                cv2.drawContours(
                    contour_mask,
                    [shifted_contour],
                    -1,
                    255,
                    -1,
                )
                normalized = self._normalize_glyph(contour_mask)
                if normalized is None:
                    continue
                digit, score, margin = self._classify_normalized(normalized)
                if (
                    digit != target_digit
                    or score < DIGIT_MIN_SCORE
                    or margin < DIGIT_MIN_MARGIN
                ):
                    continue

                center = (
                    x + box_width / 2.0,
                    y + box_height / 2.0,
                )
                candidate = {
                    "center": center,
                    "bbox": (x, y, box_width, box_height),
                    "contour": contour.copy(),
                    "area": contour_area,
                    "digit": int(digit),
                    "digit_score": float(score),
                    "digit_margin": float(margin),
                }

                duplicate_index = next(
                    (
                        index
                        for index, saved in enumerate(candidates)
                        if point_distance(saved["center"], center)
                        <= max(4.0, 0.20 * box_height)
                    ),
                    None,
                )
                if duplicate_index is None:
                    candidates.append(candidate)
                elif (
                    score + 0.35 * margin
                    > candidates[duplicate_index]["digit_score"]
                    + 0.35 * candidates[duplicate_index]["digit_margin"]
                ):
                    candidates[duplicate_index] = candidate

        return candidates


class StableDigitRingTracker:
    """对数字同心圆进行多帧确认、邻近跟踪和短时漏检保持。"""

    def __init__(
        self,
        confirm_frames=3,
        hold_miss_frames=3,
        max_center_jump=40.0,
        max_radius_jump=12.0,
        center_alpha=0.30,
        radius_alpha=0.25,
    ):
        self.confirm_frames = max(1, int(confirm_frames))
        self.hold_miss_frames = max(0, int(hold_miss_frames))
        self.max_center_jump = float(max_center_jump)
        self.max_radius_jump = float(max_radius_jump)
        self.center_alpha = float(center_alpha)
        self.radius_alpha = float(radius_alpha)
        self.reset()

    def reset(self):
        self.locked = False
        self.pending_groups = []
        self.filtered_center = None
        self.filtered_radius = None
        self.last_group = None
        self.miss_count = 0

    @staticmethod
    def _distance(group, center):
        return float(
            np.hypot(
                group["center"][0] - center[0],
                group["center"][1] - center[1],
            )
        )

    @staticmethod
    def _quality(group):
        return (
            group["digit_score"],
            group["digit_margin"],
            group["max_radius"],
        )

    def _within_gate(self, group, center, radius):
        dynamic_center_gate = max(self.max_center_jump, float(radius) * 0.70)
        return (
            self._distance(group, center) <= dynamic_center_gate
            and abs(group["max_radius"] - radius) <= self.max_radius_jump
        )

    @staticmethod
    def _stabilized_group(group, center, radius):
        stabilized = dict(group)
        stabilized["center"] = (float(center[0]), float(center[1]))
        stabilized["max_radius"] = float(radius)
        return stabilized

    def _begin_pending(self, group):
        self.pending_groups = [group]

    def _update_pending(self, candidates):
        if not candidates:
            self.pending_groups = []
            return None, "confirming"

        if not self.pending_groups:
            self._begin_pending(max(candidates, key=self._quality))
        else:
            previous = self.pending_groups[-1]
            nearby = [
                group
                for group in candidates
                if self._within_gate(
                    group,
                    previous["center"],
                    previous["max_radius"],
                )
            ]
            if nearby:
                chosen = min(
                    nearby,
                    key=lambda group: (
                        self._distance(group, previous["center"]),
                        abs(group["max_radius"] - previous["max_radius"]),
                        -group["digit_score"],
                    ),
                )
                self.pending_groups.append(chosen)
            else:
                self._begin_pending(max(candidates, key=self._quality))

        if len(self.pending_groups) < self.confirm_frames:
            return None, "confirming"

        recent = self.pending_groups[-self.confirm_frames:]
        self.filtered_center = np.median(
            np.asarray([group["center"] for group in recent], dtype=np.float64),
            axis=0,
        )
        self.filtered_radius = float(
            np.median([group["max_radius"] for group in recent])
        )
        self.locked = True
        self.miss_count = 0
        self.last_group = self._stabilized_group(
            recent[-1],
            self.filtered_center,
            self.filtered_radius,
        )
        self.pending_groups = []
        return self.last_group, "locked"

    def _handle_miss(self):
        self.miss_count += 1
        if self.last_group is not None and self.miss_count <= self.hold_miss_frames:
            return self.last_group, "held"

        self.reset()
        return None, "lost"

    def update(self, candidates):
        if not self.locked:
            return self._update_pending(candidates)

        nearby = [
            group
            for group in candidates
            if self._within_gate(
                group,
                self.filtered_center,
                self.filtered_radius,
            )
        ]
        if not nearby:
            return self._handle_miss()

        chosen = min(
            nearby,
            key=lambda group: (
                self._distance(group, self.filtered_center),
                abs(group["max_radius"] - self.filtered_radius),
                -group["digit_score"],
            ),
        )
        raw_center = np.asarray(chosen["center"], dtype=np.float64)
        self.filtered_center = (
            self.filtered_center
            + self.center_alpha * (raw_center - self.filtered_center)
        )
        self.filtered_radius += self.radius_alpha * (
            chosen["max_radius"] - self.filtered_radius
        )
        self.miss_count = 0
        self.last_group = self._stabilized_group(
            chosen,
            self.filtered_center,
            self.filtered_radius,
        )
        return self.last_group, "tracked"


class DigitRingDetector:
    """先按数字粗定位，再在准心附近用同心圆精定位。"""

    def __init__(
        self,
        center_x=DIGIT_RING_REFERENCE_CENTER_X,
        center_y=DIGIT_RING_REFERENCE_CENTER_Y,
        recognizer=None,
        tracker=None,
        digit_min_area=DIGIT_COARSE_MIN_AREA_640X480,
        switch_tolerance=DIGIT_RING_SWITCH_TOLERANCE,
        aligned_confirm_frames=DIGIT_RING_ALIGNED_CONFIRM_FRAMES,
        ring_roi_half_size=DIGIT_RING_ROI_HALF_SIZE,
        ring_hold_miss_frames=DIGIT_RING_HOLD_MISS_FRAMES,
        ring_acquire_miss_frames=DIGIT_RING_ACQUIRE_MISS_FRAMES,
    ):
        self.center_x = int(center_x)
        self.center_y = int(center_y)
        self.switch_tolerance = max(1, int(switch_tolerance))
        self.aligned_confirm_frames = max(1, int(aligned_confirm_frames))
        self.ring_roi_half_size = max(60, int(ring_roi_half_size))
        self.ring_acquire_miss_frames = max(1, int(ring_acquire_miss_frames))
        self.target_digit = None
        self.recognizer = recognizer or Digit123Recognizer(
            coarse_min_area=digit_min_area,
        )
        self.tracker = tracker or StableDigitRingTracker(
            confirm_frames=3,
            hold_miss_frames=ring_hold_miss_frames,
        )
        self.phase = "digit_coarse"
        self.aligned_count = 0
        self.coarse_filter = AdaptiveCenterFilter()
        self.last_digit_center = None
        self.ring_acquire_miss_count = 0

    def set_digit(self, digit):
        if digit not in (1, 2, 3):
            raise ValueError(f"不支持的数字同心圆：{digit}")
        self.target_digit = int(digit)
        self._reset_tracking()

    def reset(self):
        self.target_digit = None
        self._reset_tracking()

    def _reset_tracking(self):
        self.phase = "digit_coarse"
        self.aligned_count = 0
        self.coarse_filter = AdaptiveCenterFilter()
        self.last_digit_center = None
        self.ring_acquire_miss_count = 0
        self.tracker.reset()

    def draw_crosshair(self, frame):
        half_length = 14
        cv2.line(
            frame,
            (self.center_x - half_length, self.center_y),
            (self.center_x + half_length, self.center_y),
            (0, 255, 0),
            1,
        )
        cv2.line(
            frame,
            (self.center_x, self.center_y - half_length),
            (self.center_x, self.center_y + half_length),
            (0, 255, 0),
            1,
        )
        cv2.circle(frame, (self.center_x, self.center_y), 2, (0, 255, 0), -1)

    def _empty_detection(self, state, phase, next_phase=None):
        return {
            "detected": False,
            "state": state,
            "mode": "digit_ring",
            "next_mode": "digit_ring",
            "phase": phase,
            "next_phase": next_phase or phase,
            "target": f"digit_ring_{self.target_digit}",
            "center": None,
            "error_x": 0,
            "error_y": 0,
            "area": 0.0,
            "radius": 0.0,
            "circle_count": 0,
            "digit": self.target_digit,
            "digit_score": 0.0,
            "digit_margin": 0.0,
            "hough_count": 0,
            "ring_group_count": 0,
            "aligned_count": 0,
            "aligned_required": self.aligned_confirm_frames,
            "miss_count": 0,
        }

    @staticmethod
    def _candidate_quality(candidate):
        return (
            candidate.get("coarse_ring_count", 0),
            candidate["digit_score"],
            candidate["digit_margin"],
            candidate["area"],
        )

    def _confirm_candidates_with_concentric_rings(self, gray, candidates):
        """只保留中心附近至少存在两条同心圆边的数字候选。"""
        height, width = gray.shape
        confirmed = []

        for candidate in candidates:
            candidate_x, candidate_y = candidate["center"]
            x_1 = max(0, int(round(candidate_x)) - self.ring_roi_half_size)
            y_1 = max(0, int(round(candidate_y)) - self.ring_roi_half_size)
            x_2 = min(
                width,
                int(round(candidate_x)) + self.ring_roi_half_size + 1,
            )
            y_2 = min(
                height,
                int(round(candidate_y)) + self.ring_roi_half_size + 1,
            )
            roi = gray[y_1:y_2, x_1:x_2]
            if roi.shape[0] < 2 * HOUGH_MIN_RADIUS or roi.shape[1] < 2 * HOUGH_MIN_RADIUS:
                continue

            blurred = cv2.GaussianBlur(
                roi,
                (GAUSSIAN_SIZE, GAUSSIAN_SIZE),
                GAUSSIAN_SIGMA,
            )
            local_circles = detect_hough_circles(blurred)
            circles = [
                {
                    "center": (
                        circle["center"][0] + x_1,
                        circle["center"][1] + y_1,
                    ),
                    "radius": circle["radius"],
                }
                for circle in local_circles
            ]
            groups = validate_ring_groups(
                group_concentric_circles(circles),
                min_ring_count=DIGIT_COARSE_RING_MIN_COUNT,
            )
            nearby_groups = []
            for group in groups:
                nearby_circles = remove_duplicate_ring_radii(
                    [
                        circle
                        for circle in group["circles"]
                        if point_distance(
                            circle["center"],
                            candidate["center"],
                        )
                        <= DIGIT_COARSE_RING_CENTER_TOLERANCE
                    ]
                )
                if len(nearby_circles) < DIGIT_COARSE_RING_MIN_COUNT:
                    continue

                nearby_groups.append(
                    {
                        **group,
                        "center": (
                            float(
                                np.mean(
                                    [
                                        circle["center"][0]
                                        for circle in nearby_circles
                                    ]
                                )
                            ),
                            float(
                                np.mean(
                                    [
                                        circle["center"][1]
                                        for circle in nearby_circles
                                    ]
                                )
                            ),
                        ),
                        "circles": nearby_circles,
                        "min_radius": float(
                            min(circle["radius"] for circle in nearby_circles)
                        ),
                        "max_radius": float(
                            max(circle["radius"] for circle in nearby_circles)
                        ),
                    }
                )
            if not nearby_groups:
                continue

            best_group = min(
                nearby_groups,
                key=lambda group: (
                    point_distance(group["center"], candidate["center"]),
                    -len(group["circles"]),
                    -group["max_radius"],
                ),
            )
            confirmed_candidate = dict(candidate)
            confirmed_candidate["coarse_ring_count"] = len(
                best_group["circles"]
            )
            confirmed_candidate["coarse_ring_center"] = best_group["center"]
            confirmed_candidate["coarse_circles"] = best_group["circles"]
            confirmed.append(confirmed_candidate)

        return confirmed

    def _select_digit_candidate(self, candidates):
        if not candidates:
            return None
        if self.last_digit_center is None:
            return max(candidates, key=self._candidate_quality)

        nearby = [
            candidate
            for candidate in candidates
            if point_distance(candidate["center"], self.last_digit_center)
            <= max(55.0, self.switch_tolerance * 3.0)
        ]
        if not nearby:
            return max(candidates, key=self._candidate_quality)
        return min(
            nearby,
            key=lambda candidate: (
                point_distance(candidate["center"], self.last_digit_center),
                -candidate["digit_score"],
                -candidate["digit_margin"],
            ),
        )

    def _process_digit_coarse(self, frame, gray):
        result = frame.copy()
        candidates = self.recognizer.find_target_candidates(
            gray,
            self.target_digit,
        )
        candidates = self._confirm_candidates_with_concentric_rings(
            gray,
            candidates,
        )
        candidate = self._select_digit_candidate(candidates)
        if candidate is None:
            self.aligned_count = 0
            detection = self._empty_detection(
                "digit_search",
                "digit_coarse",
            )
            self.draw_crosshair(result)
            return result, detection

        self.last_digit_center = candidate["center"]
        filtered_x, filtered_y = self.coarse_filter.update(candidate["center"])
        center = (int(round(filtered_x)), int(round(filtered_y)))
        error_x = center[0] - self.center_x
        error_y = self.center_y - center[1]

        within_tolerance = (
            abs(error_x) <= self.switch_tolerance
            and abs(error_y) <= self.switch_tolerance
        )
        if within_tolerance:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        state = "digit_aligning"
        next_phase = "digit_coarse"
        if self.aligned_count >= self.aligned_confirm_frames:
            state = "digit_aligned"
            next_phase = "ring_refine"
            self.phase = "ring_refine"
            self.ring_acquire_miss_count = 0
            self.tracker.reset()

        cv2.drawContours(result, [candidate["contour"]], -1, (255, 0, 255), 2)
        for circle in candidate["coarse_circles"]:
            self._draw_circle(result, circle, (0, 165, 255), 1)
        x, y, width, height = candidate["bbox"]
        cv2.rectangle(result, (x, y), (x + width, y + height), (255, 0, 255), 1)
        cv2.circle(result, center, 4, (0, 0, 255), -1)
        self.draw_crosshair(result)

        return result, {
            "detected": True,
            "state": state,
            "mode": "digit_ring",
            "next_mode": "digit_ring",
            "phase": "digit_coarse",
            "next_phase": next_phase,
            "target": f"digit_ring_{self.target_digit}",
            "center": center,
            "error_x": int(error_x),
            "error_y": int(error_y),
            "area": float(candidate["area"]),
            "radius": 0.0,
            "circle_count": 0,
            "digit": self.target_digit,
            "digit_score": float(candidate["digit_score"]),
            "digit_margin": float(candidate["digit_margin"]),
            "coarse_ring_count": int(candidate["coarse_ring_count"]),
            "hough_count": int(candidate["coarse_ring_count"]),
            "ring_group_count": 0,
            "aligned_count": self.aligned_count,
            "aligned_required": self.aligned_confirm_frames,
            "miss_count": 0,
        }

    def _detect_local_ring_groups(self, gray):
        height, width = gray.shape
        x_1 = max(0, self.center_x - self.ring_roi_half_size)
        y_1 = max(0, self.center_y - self.ring_roi_half_size)
        x_2 = min(width, self.center_x + self.ring_roi_half_size + 1)
        y_2 = min(height, self.center_y + self.ring_roi_half_size + 1)
        roi = gray[y_1:y_2, x_1:x_2]
        blurred = cv2.GaussianBlur(
            roi,
            (GAUSSIAN_SIZE, GAUSSIAN_SIZE),
            GAUSSIAN_SIGMA,
        )
        local_circles = detect_hough_circles(blurred)
        circles = [
            {
                "center": (
                    circle["center"][0] + x_1,
                    circle["center"][1] + y_1,
                ),
                "radius": circle["radius"],
            }
            for circle in local_circles
        ]
        groups = validate_ring_groups(group_concentric_circles(circles))
        max_center_distance = max(45.0, self.switch_tolerance * 3.0)
        nearby_groups = [
            group
            for group in groups
            if point_distance(
                group["center"],
                (self.center_x, self.center_y),
            )
            <= max_center_distance
        ]
        return circles, groups, nearby_groups

    @staticmethod
    def _draw_circle(frame, circle, color, thickness):
        center = (
            int(round(circle["center"][0])),
            int(round(circle["center"][1])),
        )
        cv2.circle(
            frame,
            center,
            max(1, int(round(circle["radius"]))),
            color,
            thickness,
        )

    def _select_refine_tracking_candidates(self, nearby_groups):
        """首次精定位只尝试最接近准心的圆组，锁定后再邻近跟踪。"""
        if self.tracker.locked or len(nearby_groups) <= 1:
            return nearby_groups

        closest = min(
            nearby_groups,
            key=lambda group: (
                point_distance(
                    group["center"],
                    (self.center_x, self.center_y),
                ),
                -len(group["circles"]),
                -group["max_radius"],
            ),
        )
        return [closest]

    def _process_ring_refine(self, frame, gray):
        result = frame.copy()
        hough_circles, valid_groups, nearby_groups = self._detect_local_ring_groups(
            gray
        )

        for circle in hough_circles:
            self._draw_circle(result, circle, (150, 150, 150), 1)
        for group in valid_groups:
            for circle in group["circles"]:
                self._draw_circle(result, circle, (0, 165, 255), 1)

        tracking_candidates = self._select_refine_tracking_candidates(
            nearby_groups
        )

        if self.tracker.locked:
            self.ring_acquire_miss_count = 0
        else:
            self.ring_acquire_miss_count += 1

        target_group, tracking_state = self.tracker.update(tracking_candidates)
        if target_group is None:
            acquisition_timed_out = (
                tracking_state == "confirming"
                and self.ring_acquire_miss_count
                >= self.ring_acquire_miss_frames
            )
            if tracking_state == "lost" or acquisition_timed_out:
                self.phase = "digit_coarse"
                self.aligned_count = 0
                self.coarse_filter = AdaptiveCenterFilter()
                self.last_digit_center = None
                self.ring_acquire_miss_count = 0
                detection = self._empty_detection(
                    "ring_lost_reacquire",
                    "ring_refine",
                    next_phase="digit_coarse",
                )
            else:
                detection = self._empty_detection(
                    "ring_refining",
                    "ring_refine",
                )
        else:
            detection = build_digit_ring_detection(
                target_group,
                target_digit=self.target_digit,
                reference_center=(self.center_x, self.center_y),
            )
            detection["phase"] = "ring_refine"
            detection["next_phase"] = "ring_refine"
            detection["state"] = f"ring_{tracking_state}"

            stable_center = detection["center"]
            for circle in target_group["circles"]:
                cv2.circle(
                    result,
                    stable_center,
                    max(1, int(round(circle["radius"]))),
                    (0, 0, 255),
                    2,
                )
            cv2.drawMarker(
                result,
                stable_center,
                (0, 0, 255),
                cv2.MARKER_CROSS,
                16,
                1,
            )

        detection["hough_count"] = len(hough_circles)
        detection["ring_group_count"] = len(valid_groups)
        detection["tracking_state"] = tracking_state
        detection["miss_count"] = (
            self.ring_acquire_miss_count
            if not self.tracker.locked
            else self.tracker.miss_count
        )
        self.draw_crosshair(result)
        return result, detection

    def process(self, frame):
        if self.target_digit not in (1, 2, 3):
            raise RuntimeError("尚未收到有效数字同心圆命令")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.phase == "digit_coarse":
            result, detection = self._process_digit_coarse(frame, gray)
        else:
            result, detection = self._process_ring_refine(frame, gray)
        return result, gray, detection


def _three_ring_extract_observations(frame):
    """提取可能属于同心圆的椭圆轮廓。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(blurred, 60, 160)
    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_NONE,
    )
    observations = []
    for contour in contours:
        if len(contour) < 40:
            continue
        center, axes, angle = cv2.fitEllipse(contour)
        major_axis = float(max(axes))
        minor_axis = float(min(axes))
        if (
            minor_axis < 25.0
            or major_axis > 320.0
            or minor_axis / major_axis < 0.25
        ):
            continue
        observations.append(
            {
                "center": np.asarray(center, dtype=np.float64),
                "ellipse": (
                    tuple(float(value) for value in center),
                    tuple(float(value) for value in axes),
                    float(angle),
                ),
                "major_axis": major_axis,
            }
        )
    return observations, edges


def _three_ring_root(parent, index):
    if parent[index] != index:
        parent[index] = _three_ring_root(parent, parent[index])
    return parent[index]


def _three_ring_candidates(observations):
    """把圆心相近的椭圆轮廓聚合成同心圆候选。"""
    parent = list(range(len(observations)))
    for first_index in range(len(observations)):
        for second_index in range(first_index):
            distance = np.linalg.norm(
                observations[first_index]["center"]
                - observations[second_index]["center"]
            )
            if distance > 22.0:
                continue
            first_root = _three_ring_root(parent, first_index)
            second_root = _three_ring_root(parent, second_index)
            if first_root != second_root:
                parent[second_root] = first_root

    groups = {}
    for index, observation in enumerate(observations):
        root = _three_ring_root(parent, index)
        groups.setdefault(root, []).append(observation)

    candidates = []
    for group in groups.values():
        diameters = sorted(item["major_axis"] for item in group)
        diameter_bands = []
        for diameter in diameters:
            if not diameter_bands or diameter - diameter_bands[-1] > 5.0:
                diameter_bands.append(diameter)
        if len(group) < 5 or len(diameter_bands) < 4:
            continue

        center = np.median(
            np.asarray([item["center"] for item in group]),
            axis=0,
        )
        effective_diameter = float(np.median(diameters[-min(4, len(diameters)):]))
        center_tolerance = max(6.0, effective_diameter * 0.08)
        centered = [
            item for item in group
            if np.linalg.norm(item["center"] - center) <= center_tolerance
        ] or group
        outer = max(centered, key=lambda item: item["major_axis"])
        candidates.append(
            {
                "center": center,
                "support": len(group),
                "diameter_bands": len(diameter_bands),
                "ellipse": outer["ellipse"],
                "effective_diameter": effective_diameter,
            }
        )
    return candidates


def _select_three_ring_triple(candidates, image_width):
    """从所有同心圆候选中选择跨度足够且最接近共线的一组三环。"""
    best = None
    for triple in combinations(candidates, 3):
        points = np.asarray([item["center"] for item in triple])
        span, first_index, third_index = max(
            (
                float(np.linalg.norm(points[first] - points[second])),
                first,
                second,
            )
            for first in range(3)
            for second in range(first)
        )
        if span < image_width * 0.14:
            continue
        middle_index = 3 - first_index - third_index
        line = points[third_index] - points[first_index]
        offset = points[middle_index] - points[first_index]
        fraction = float(np.dot(offset, line) / (span * span))
        cross_product = line[0] * offset[1] - line[1] * offset[0]
        perpendicular = abs(float(cross_product)) / span
        collinearity = perpendicular / span
        if not 0.2 < fraction < 0.8 or collinearity > 0.07:
            continue

        ordered = tuple(sorted(triple, key=lambda item: item["center"][0]))
        spacing_12 = float(
            np.linalg.norm(ordered[1]["center"] - ordered[0]["center"])
        )
        spacing_23 = float(
            np.linalg.norm(ordered[2]["center"] - ordered[1]["center"])
        )
        balance = abs(math.log(spacing_12 / spacing_23))
        diameter_penalty = 0.0
        for item, spacing in zip(
            ordered,
            (spacing_12, (spacing_12 + spacing_23) / 2.0, spacing_23),
        ):
            ratio = item["effective_diameter"] / spacing
            if ratio < 0.25:
                diameter_penalty += 0.25 - ratio
            elif ratio > 1.10:
                diameter_penalty += ratio - 1.10
        support = sum(item["support"] for item in ordered)
        score = (
            8.0 * collinearity
            + 0.35 * balance
            + 2.0 * diameter_penalty
            - 0.015 * support
        )
        if best is None or score < best[0]:
            best = (score, ordered, collinearity)

    if best is None:
        return None
    _, ordered, collinearity = best
    spacing_12 = float(
        np.linalg.norm(ordered[1]["center"] - ordered[0]["center"])
    )
    spacing_23 = float(
        np.linalg.norm(ordered[2]["center"] - ordered[1]["center"])
    )
    perspective = spacing_12 / spacing_23
    geometry = math.exp(
        -10.0 * collinearity - 0.5 * abs(math.log(perspective))
    )
    support = min(item["support"] for item in ordered) / 10.0
    bands = min(item["diameter_bands"] for item in ordered) / 6.0
    confidence = min(
        1.0,
        0.55 * geometry
        + 0.25 * min(support, 1.0)
        + 0.20 * min(bands, 1.0),
    )
    return {
        "centers": tuple(
            tuple(float(value) for value in item["center"])
            for item in ordered
        ),
        "ellipses": tuple(item["ellipse"] for item in ordered),
        "ring_labels": ("R1", "R2", "R3"),
        "r2_center": tuple(
            float(value) for value in ordered[1]["center"]
        ),
        "confidence": float(confidence),
        "collinearity": float(collinearity),
        "perspective": float(perspective),
    }


def _select_two_ring_pair(candidates, image_width, reference_center):
    """选择包含R2的相邻两环；跨度过大的R1+R3组合会被拒绝。"""
    reference = np.asarray(reference_center, dtype=np.float64)
    best = None
    for pair in combinations(candidates, 2):
        ordered = tuple(sorted(pair, key=lambda item: item["center"][0]))
        first_center = np.asarray(ordered[0]["center"], dtype=np.float64)
        second_center = np.asarray(ordered[1]["center"], dtype=np.float64)
        spacing = float(np.linalg.norm(second_center - first_center))
        if not image_width * 0.08 <= spacing <= image_width * 0.42:
            continue

        mean_diameter = float(np.mean([
            item["effective_diameter"] for item in ordered
        ]))
        diameter_spacing_ratio = mean_diameter / spacing
        if not 0.32 <= diameter_spacing_ratio <= 1.10:
            continue

        reference_distances = tuple(
            float(np.linalg.norm(np.asarray(item["center"]) - reference))
            for item in ordered
        )
        if math.isclose(
            reference_distances[0],
            reference_distances[1],
            abs_tol=1e-6,
        ):
            continue
        r2_index = int(np.argmin(reference_distances))
        labels = (
            ("R1", "R2")
            if r2_index == 1
            else ("R2", "R3")
        )

        support = min(item["support"] for item in ordered) / 10.0
        bands = min(item["diameter_bands"] for item in ordered) / 6.0
        spacing_quality = math.exp(
            -0.35 * abs(math.log(diameter_spacing_ratio / 0.50))
        )
        confidence = min(
            1.0,
            0.40 * spacing_quality
            + 0.30 * min(support, 1.0)
            + 0.30 * min(bands, 1.0),
        )
        score = (
            reference_distances[r2_index] / max(float(image_width), 1.0)
            + abs(math.log(diameter_spacing_ratio / 0.50))
            - 0.01 * sum(item["support"] for item in ordered)
        )
        if best is None or score < best[0]:
            best = (
                score,
                ordered,
                labels,
                r2_index,
                confidence,
                diameter_spacing_ratio,
            )

    if best is None:
        return None
    _, ordered, labels, r2_index, confidence, ratio = best
    return {
        "centers": tuple(
            tuple(float(value) for value in item["center"])
            for item in ordered
        ),
        "ellipses": tuple(item["ellipse"] for item in ordered),
        "ring_labels": labels,
        "r2_center": tuple(
            float(value) for value in ordered[r2_index]["center"]
        ),
        "confidence": float(confidence),
        "diameter_spacing_ratio": float(ratio),
    }


def _measure_black_line_band_support(
    gray,
    line_points,
    profile_radius=PILE_LINE_PROFILE_RADIUS,
    maximum_line_gray=PILE_LINE_MAX_GRAY,
    minimum_contrast=PILE_LINE_MIN_BLACK_CONTRAST,
):
    """测量候选线是否是被较亮背景从上下夹住的黑色细带。"""
    (x1, y1), (x2, y2) = line_points
    delta_x = float(x2 - x1)
    delta_y = float(y2 - y1)
    line_length = max(math.hypot(delta_x, delta_y), 1.0)
    normal_x = -delta_y / line_length
    normal_y = delta_x / line_length
    sample_count = 64
    valid_count = 0
    radius = max(8, int(profile_radius))

    for line_ratio in np.linspace(0.05, 0.95, sample_count):
        base_x = x1 + line_ratio * delta_x
        base_y = y1 + line_ratio * delta_y
        profile = []
        for offset in range(-radius, radius + 1):
            sample_x = int(round(base_x + normal_x * offset))
            sample_y = int(round(base_y + normal_y * offset))
            sample_x = max(0, min(gray.shape[1] - 1, sample_x))
            sample_y = max(0, min(gray.shape[0] - 1, sample_y))
            profile.append(float(gray[sample_y, sample_x]))

        profile = np.convolve(
            np.asarray(profile, dtype=np.float32),
            np.asarray((0.25, 0.50, 0.25), dtype=np.float32),
            mode="same",
        )
        # Hough可能落在黑带任一边缘。沿法向找到完整暗带，再检查其
        # 两侧背景；不把固定±6像素当成所有观察高度下的黑带宽度。
        dark = profile <= float(maximum_line_gray)
        index = 0
        while index < len(profile):
            if not dark[index]:
                index += 1
                continue
            start = index
            while index < len(profile) and dark[index]:
                index += 1
            end = index
            if start < 4 or end + 4 > len(profile):
                continue
            if start > radius + 6 or end - 1 < radius - 6:
                continue
            line_gray = float(np.median(profile[start:end]))
            upper_gray = float(np.median(profile[start - 4:start - 1]))
            lower_gray = float(np.median(profile[end + 1:end + 4]))
            if min(upper_gray, lower_gray) - line_gray >= float(minimum_contrast):
                valid_count += 1
                break

    return valid_count / max(sample_count, 1)


def _extract_pile_line(
    frame,
    roi_top_ratio=PILE_LINE_ROI_TOP_RATIO,
    roi_bottom_ratio=PILE_LINE_ROI_BOTTOM_RATIO,
    minimum_span_ratio=PILE_LINE_MIN_SPAN_RATIO,
    side_margin_ratio=PILE_LINE_SIDE_MARGIN_RATIO,
    maximum_abs_angle_deg=PILE_LINE_MAX_ABS_ANGLE_DEG,
):
    """在画面底部选择横跨左右两侧的最长近水平黑线。"""
    height, width = frame.shape[:2]
    roi_top = max(0, min(height - 1, int(round(height * roi_top_ratio))))
    roi_bottom = max(
        roi_top + 1,
        min(height, int(round(height * roi_bottom_ratio))),
    )
    full_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = full_gray[roi_top:roi_bottom, :]
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges_roi = cv2.Canny(blurred, 40, 120)
    edges = np.zeros((height, width), dtype=np.uint8)
    edges[roi_top:roi_bottom, :] = edges_roi

    lines = cv2.HoughLinesP(
        edges_roi,
        1,
        np.pi / 180.0,
        threshold=max(20, int(round(width * 0.05))),
        minLineLength=max(20, int(round(width * 0.25))),
        maxLineGap=max(8, int(round(width * 0.12))),
    )
    if lines is None:
        return None, edges, roi_top

    minimum_span = float(width) * float(minimum_span_ratio)
    left_limit = float(width) * float(side_margin_ratio)
    right_limit = float(width) * (1.0 - float(side_margin_ratio))
    segments = []
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1) + roi_top
        y2 = int(y2) + roi_top
        if x2 < x1:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        span = float(x2 - x1)
        if span < width * 0.20:
            continue
        angle_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))
        if abs(angle_deg) > float(maximum_abs_angle_deg):
            continue
        sample_count = max(16, min(96, int(round(span / 8.0))))
        dark_sample_count = 0
        for sample_index in range(sample_count):
            ratio = sample_index / max(sample_count - 1, 1)
            sample_x = int(round(x1 + ratio * (x2 - x1)))
            sample_y = int(round(y1 + ratio * (y2 - y1))) - roi_top
            x_start = max(0, sample_x - 2)
            x_end = min(width, sample_x + 3)
            y_start = max(0, sample_y - 3)
            y_end = min(gray.shape[0], sample_y + 4)
            patch = gray[y_start:y_end, x_start:x_end]
            if patch.size and int(np.min(patch)) <= PILE_LINE_MAX_GRAY:
                dark_sample_count += 1
        dark_support_ratio = dark_sample_count / max(sample_count, 1)
        if dark_support_ratio < PILE_LINE_MIN_DARK_SUPPORT_RATIO:
            continue
        slope = (y2 - y1) / max(float(x2 - x1), 1.0)
        segments.append(
            {
                "points": ((x1, y1), (x2, y2)),
                "angle_deg": float(angle_deg),
                "length": math.hypot(x2 - x1, y2 - y1),
                "center_y": y1 + (width * 0.5 - x1) * slope,
                "dark_support_ratio": float(dark_support_ratio),
            }
        )

    if not segments:
        return None, edges, roi_top

    y_tolerance = max(8.0, height * 0.025)
    fitted_candidates = []
    for seed in segments:
        cluster = [
            segment
            for segment in segments
            if abs(segment["angle_deg"] - seed["angle_deg"]) <= 4.0
            and abs(segment["center_y"] - seed["center_y"]) <= y_tolerance
        ]
        all_points = [
            point
            for segment in cluster
            for point in segment["points"]
        ]
        minimum_x = min(point[0] for point in all_points)
        maximum_x = max(point[0] for point in all_points)
        aggregate_span = float(maximum_x - minimum_x)
        if (
            aggregate_span < minimum_span
            or minimum_x > left_limit
            or maximum_x < right_limit
        ):
            continue

        fit_points = np.asarray(all_points, dtype=np.float32).reshape(-1, 1, 2)
        vx, vy, x0, y0 = (
            float(value)
            for value in cv2.fitLine(
                fit_points,
                cv2.DIST_L2,
                0,
                0.01,
                0.01,
            ).reshape(-1)
        )
        if abs(vx) < 1e-6:
            continue
        left_x = 0
        right_x = width - 1
        left_y = y0 + (left_x - x0) * vy / vx
        right_y = y0 + (right_x - x0) * vy / vx
        left_y = int(round(max(0.0, min(height - 1.0, left_y))))
        right_y = int(round(max(0.0, min(height - 1.0, right_y))))
        # 角度保留浮点拟合精度；整数端点只供绘图和黑带验证使用。
        angle_deg = math.degrees(math.atan(vy / vx))
        if abs(angle_deg) > float(maximum_abs_angle_deg):
            continue
        line_points = ((left_x, left_y), (right_x, right_y))
        fitted_candidates.append(
            {
                "line_points": line_points,
                "angle_deg": float(angle_deg),
                "center": (
                    int(round((left_x + right_x) / 2.0)),
                    int(round((left_y + right_y) / 2.0)),
                ),
                "span_ratio": aggregate_span / max(float(width), 1.0),
                "score": (
                    aggregate_span,
                    sum(segment["length"] for segment in cluster),
                    float(np.mean([
                        segment["dark_support_ratio"]
                        for segment in cluster
                    ])),
                ),
            }
        )

    if not fitted_candidates:
        return None, edges, roi_top
    for selected in sorted(
        fitted_candidates,
        key=lambda candidate: candidate["score"],
        reverse=True,
    ):
        black_band_support_ratio = _measure_black_line_band_support(
            full_gray,
            selected["line_points"],
        )
        if black_band_support_ratio < PILE_LINE_MIN_BLACK_BAND_SUPPORT_RATIO:
            continue
        selected.pop("score", None)
        selected["black_band_support_ratio"] = float(
            black_band_support_ratio
        )
        return selected, edges, roi_top
    return None, edges, roi_top


class PileLinePoseDetector:
    """用画面底部最长黑线相对视觉X轴的角度进行码堆姿态纠偏。"""

    def __init__(
        self,
        reference_yaw_deg=THREE_RING_REFERENCE_YAW_DEG,
        deadzone_deg=PILE_LINE_ANGLE_DEADZONE_DEG,
        stable_frame_count=PILE_LINE_STABLE_FRAMES,
        maximum_angle_jitter_deg=PILE_LINE_MAX_ANGLE_JITTER_DEG,
        maximum_missed_frames=PILE_LINE_MAX_MISSED_FRAMES,
        roi_top_ratio=PILE_LINE_ROI_TOP_RATIO,
    ):
        self.reference_yaw_deg = float(reference_yaw_deg)
        self.deadzone_deg = float(deadzone_deg)
        self.stable_frame_count = max(2, int(stable_frame_count))
        self.maximum_angle_jitter_deg = float(maximum_angle_jitter_deg)
        self.maximum_missed_frames = max(0, int(maximum_missed_frames))
        self.roi_top_ratio = float(roi_top_ratio)
        self.reset()

    def reset(self):
        self.samples = []
        self.miss_count = 0
        self.last_stable_detection = None

    def _empty_detection(self, state):
        return {
            "detected": False,
            "state": state,
            "mode": "pile_line_pose",
            "next_mode": "pile_line_pose",
            "target": "pile_line_pose",
            "center": None,
            "line_points": (),
            "station_yaw_deg": 0.0,
            "angle_error_deg": 0.0,
            "angle_error_cdeg": 0,
            "stop_correction": False,
            "error_x": 0,
            "error_y": 0,
            "area": 0.0,
            "stable_count": len(self.samples),
            "stable_required": self.stable_frame_count,
            "miss_count": self.miss_count,
            "span_ratio": 0.0,
        }

    def _build_detection(self, line_points, angle_deg, detected, state, span_ratio):
        angle_error_deg = (
            float(angle_deg) - self.reference_yaw_deg + 180.0
        ) % 360.0 - 180.0
        left, right = line_points
        center = (
            int(round((left[0] + right[0]) / 2.0)),
            int(round((left[1] + right[1]) / 2.0)),
        )
        return {
            "detected": bool(detected),
            "state": state,
            "mode": "pile_line_pose",
            "next_mode": "pile_line_pose",
            "target": "pile_line_pose",
            "center": center,
            "line_points": tuple(tuple(int(value) for value in point) for point in line_points),
            "station_yaw_deg": float(angle_deg),
            "angle_error_deg": float(angle_error_deg),
            "angle_error_cdeg": int(round(angle_error_deg * 100.0)),
            "stop_correction": bool(
                detected and abs(angle_error_deg) <= self.deadzone_deg
            ),
            "error_x": 0,
            "error_y": 0,
            "area": 0.0,
            "stable_count": len(self.samples),
            "stable_required": self.stable_frame_count,
            "miss_count": self.miss_count,
            "span_ratio": float(span_ratio),
        }

    @staticmethod
    def _draw_line(frame, line_points):
        if not line_points:
            return
        left, right = line_points
        center_y = int(round((left[1] + right[1]) / 2.0))
        cv2.line(
            frame,
            (0, center_y),
            (frame.shape[1] - 1, center_y),
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.line(frame, left, right, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.circle(frame, left, 5, (255, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, right, 5, (255, 0, 255), -1, cv2.LINE_AA)

    def process(self, frame):
        result = frame.copy()
        raw, edges, roi_top = _extract_pile_line(
            frame,
            roi_top_ratio=self.roi_top_ratio,
        )
        cv2.line(
            result,
            (0, roi_top),
            (result.shape[1] - 1, roi_top),
            (255, 255, 0),
            1,
            cv2.LINE_AA,
        )

        if raw is None:
            self.miss_count += 1
            self.samples.clear()
            if (
                self.last_stable_detection is not None
                and self.miss_count <= self.maximum_missed_frames
            ):
                detection = dict(self.last_stable_detection)
                detection["state"] = "pile_line_holding"
                detection["detected"] = False
                detection["stop_correction"] = False
                detection["stable_count"] = 0
                detection["miss_count"] = self.miss_count
                self._draw_line(result, detection["line_points"])
                return result, edges, detection

            self.samples.clear()
            self.last_stable_detection = None
            return result, edges, self._empty_detection("pile_line_search")

        self.miss_count = 0
        self.samples.append(raw)
        self.samples = self.samples[-self.stable_frame_count:]
        median_angle = float(np.median([
            sample["angle_deg"] for sample in self.samples
        ]))
        median_points = tuple(
            (
                int(round(np.median([
                    sample["line_points"][point_index][0]
                    for sample in self.samples
                ]))),
                int(round(np.median([
                    sample["line_points"][point_index][1]
                    for sample in self.samples
                ]))),
            )
            for point_index in (0, 1)
        )
        maximum_jitter = max(
            abs(sample["angle_deg"] - median_angle)
            for sample in self.samples
        )
        stable = (
            len(self.samples) == self.stable_frame_count
            and maximum_jitter <= self.maximum_angle_jitter_deg
        )
        detection = self._build_detection(
            median_points,
            median_angle,
            stable,
            "pile_line_detected" if stable else "pile_line_confirming",
            float(np.median([
                sample["span_ratio"] for sample in self.samples
            ])),
        )
        if stable:
            self.last_stable_detection = dict(detection)
        self._draw_line(result, median_points)
        return result, edges, detection


class ThreeRingPoseDetector:
    """优先识别三环；只见相邻两环时仍输出中环位置和姿态误差。"""

    def __init__(
        self,
        reference_center=(
            THREE_RING_REFERENCE_CENTER_X,
            THREE_RING_REFERENCE_CENTER_Y,
        ),
        reference_yaw_deg=THREE_RING_REFERENCE_YAW_DEG,
        deadzone_deg=THREE_RING_ANGLE_DEADZONE_DEG,
        stable_frame_count=3,
        maximum_center_jitter_px=4.0,
        minimum_confidence=0.90,
        maximum_missed_frames=2,
    ):
        self.center_x = int(reference_center[0])
        self.center_y = int(reference_center[1])
        self.reference_yaw_deg = float(reference_yaw_deg)
        self.deadzone_deg = float(deadzone_deg)
        self.stable_frame_count = max(2, int(stable_frame_count))
        self.maximum_center_jitter_px = float(maximum_center_jitter_px)
        self.minimum_confidence = float(minimum_confidence)
        self.maximum_missed_frames = max(0, int(maximum_missed_frames))
        self.reset()

    def reset(self):
        self.samples = []
        self.miss_count = 0
        self.last_stable_detection = None
        self.last_stable_ellipses = ()

    def _empty_detection(self, state, **extra):
        detection = {
            "detected": False,
            "state": state,
            "mode": "three_ring_pose",
            "next_mode": "three_ring_pose",
            "target": "three_ring_pose",
            "center": None,
            "ring_centers": (),
            "ring_count": 0,
            "ring_labels": (),
            "station_yaw_deg": 0.0,
            "angle_error_deg": 0.0,
            "angle_error_cdeg": 0,
            "stop_correction": False,
            "error_x": 0,
            "error_y": 0,
            "area": 0.0,
            "stable_count": len(self.samples),
            "stable_required": self.stable_frame_count,
            "confidence": 0.0,
            "miss_count": self.miss_count,
        }
        detection.update(extra)
        return detection

    def _hold_or_search(self, result, edges):
        """短时漏检沿用最近稳定结果，连续超限后才解除锁定。"""
        self.miss_count += 1
        if (
            self.last_stable_detection is not None
            and self.miss_count <= self.maximum_missed_frames
        ):
            detection = dict(self.last_stable_detection)
            detection["state"] = "three_ring_pose_holding"
            detection["miss_count"] = self.miss_count
            self._draw_observation(
                result,
                detection["ring_centers"],
                self.last_stable_ellipses,
                detection.get("ring_labels"),
            )
            self.draw_crosshair(result)
            return result, edges, detection

        self.samples.clear()
        self.last_stable_detection = None
        self.last_stable_ellipses = ()
        self.draw_crosshair(result)
        return result, edges, self._empty_detection("three_ring_search")

    def draw_crosshair(self, frame):
        cv2.drawMarker(
            frame,
            (self.center_x, self.center_y),
            (0, 255, 0),
            cv2.MARKER_CROSS,
            28,
            1,
        )

    @staticmethod
    def _draw_observation(frame, centers, ellipses, ring_labels=None):
        labels = tuple(ring_labels or (
            f"R{index}" for index in range(1, len(centers) + 1)
        ))
        color_by_label = {
            "R1": (0, 180, 255),
            "R2": (0, 255, 0),
            "R3": (255, 160, 0),
        }
        points = []
        for center, ellipse, label in zip(centers, ellipses, labels):
            color = color_by_label.get(label, (180, 180, 180))
            cv2.ellipse(frame, ellipse, color, 2, cv2.LINE_AA)
            point = tuple(int(round(value)) for value in center)
            points.append(point)
            cv2.drawMarker(frame, point, color, cv2.MARKER_CROSS, 24, 2)
            cv2.putText(
                frame,
                label,
                (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2,
                cv2.LINE_AA,
            )
        cv2.polylines(
            frame,
            [np.asarray(points, dtype=np.int32)],
            False,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def process(self, frame):
        result = frame.copy()
        observations, edges = _three_ring_extract_observations(frame)
        candidates = _three_ring_candidates(observations)
        raw = _select_three_ring_triple(candidates, frame.shape[1])
        if raw is None and len(candidates) == 2:
            raw = _select_two_ring_pair(
                candidates,
                frame.shape[1],
                (self.center_x, self.center_y),
            )
        if raw is None or raw["confidence"] < self.minimum_confidence:
            return self._hold_or_search(result, edges)

        if self.last_stable_detection is not None:
            current_by_label = dict(zip(
                raw["ring_labels"],
                raw["centers"],
            ))
            previous_by_label = dict(zip(
                self.last_stable_detection["ring_labels"],
                self.last_stable_detection["ring_centers"],
            ))
            common_labels = set(current_by_label) & set(previous_by_label)
            if "R2" not in common_labels:
                return self._hold_or_search(result, edges)
            tracking_jitter = max(
                point_distance(
                    current_by_label[label],
                    previous_by_label[label],
                )
                for label in common_labels
            )
            if tracking_jitter > self.maximum_center_jitter_px:
                return self._hold_or_search(result, edges)

        self.miss_count = 0
        if (
            self.samples
            and self.samples[-1]["ring_labels"] != raw["ring_labels"]
        ):
            self.samples.clear()
        self.samples.append(raw)
        self.samples = self.samples[-self.stable_frame_count:]
        centers = []
        for ring_index in range(len(raw["centers"])):
            centers.append(
                (
                    float(np.median([
                        sample["centers"][ring_index][0]
                        for sample in self.samples
                    ])),
                    float(np.median([
                        sample["centers"][ring_index][1]
                        for sample in self.samples
                    ])),
                )
            )
        centers = tuple(centers)
        maximum_jitter = max(
            point_distance(sample["centers"][ring_index], centers[ring_index])
            for sample in self.samples
            for ring_index in range(len(raw["centers"]))
        )
        stable = self.last_stable_detection is not None or (
            len(self.samples) == self.stable_frame_count
            and maximum_jitter <= self.maximum_center_jitter_px
        )

        detection = build_three_ring_pose_detection(
            centers,
            reference_center=(self.center_x, self.center_y),
            reference_yaw_deg=self.reference_yaw_deg,
            deadzone_deg=self.deadzone_deg,
            r2_center=centers[raw["ring_labels"].index("R2")],
            ring_labels=raw["ring_labels"],
        )
        detection["detected"] = bool(stable)
        detection["state"] = (
            "three_ring_pose_detected"
            if stable
            else "three_ring_pose_confirming"
        )
        detection["stable_count"] = len(self.samples)
        detection["stable_required"] = self.stable_frame_count
        detection["confidence"] = float(
            np.median([sample["confidence"] for sample in self.samples])
        )
        detection["miss_count"] = 0
        detection["stop_correction"] = bool(
            stable and detection["stop_correction"]
        )
        if stable:
            self.last_stable_detection = dict(detection)
            self.last_stable_ellipses = tuple(raw["ellipses"])

        self._draw_observation(
            result,
            centers,
            raw["ellipses"],
            raw["ring_labels"],
        )
        self.draw_crosshair(result)
        return result, edges, detection


class AdaptiveCenterFilter:
    """静止时抑制像素抖动，快速移动时降低滤波延迟。"""

    def __init__(self, steady_alpha=0.35, moving_alpha=0.85, fast_distance=12.0):
        self.steady_alpha = float(steady_alpha)
        self.moving_alpha = float(moving_alpha)
        self.fast_distance = float(fast_distance)
        self.history = []
        self.value = None

    def update(self, point):
        raw = np.asarray(point, dtype=np.float64)
        if self.value is None:
            self.value = raw
            self.history = [raw]
            return tuple(self.value)

        distance = float(np.linalg.norm(raw - self.value))
        if distance >= self.fast_distance:
            target = raw
            alpha = self.moving_alpha
            self.history = [raw]
        else:
            self.history.append(raw)
            self.history = self.history[-3:]
            target = np.median(np.stack(self.history), axis=0)
            alpha = self.steady_alpha

        self.value = self.value + alpha * (target - self.value)
        return tuple(self.value)


def configure_processing_runtime():
    """启用 OpenCV 优化，并允许进程使用全部在线 CPU 核心。"""
    logical_cpu_count = max(1, os.cpu_count() or 1)
    available_cpu_count = logical_cpu_count

    if hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, set(range(logical_cpu_count)))
            available_cpu_count = len(os.sched_getaffinity(0))
        except (OSError, PermissionError):
            try:
                available_cpu_count = len(os.sched_getaffinity(0))
            except OSError:
                available_cpu_count = logical_cpu_count

    available_cpu_count = max(1, available_cpu_count)
    cv2.setUseOptimized(True)
    cv2.setNumThreads(available_cpu_count)

    print(
        f"OpenCV optimized={cv2.useOptimized()}, "
        f"threads={cv2.getNumThreads()}, "
        f"available CPU cores={available_cpu_count}"
    )


class LatestFrameCamera:
    """后台持续采集，只向处理线程交付摄像头的最新帧。"""

    def __init__(self, camera_index=0, width=640, height=480, fps=120.0):
        self.camera_index = int(camera_index)
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.requested_fps = float(fps)

        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"无法通过 V4L2 打开摄像头 {self.camera_index}"
            )

        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.requested_fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.actual_width = int(round(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.actual_height = int(round(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.actual_fps = float(self.capture.get(cv2.CAP_PROP_FPS))
        self.actual_fourcc = self.decode_fourcc(
            self.capture.get(cv2.CAP_PROP_FOURCC)
        )

        if self.actual_fourcc != "MJPG":
            self.capture.release()
            raise RuntimeError(
                "摄像头没有接受 MJPG："
                f"实际格式={self.actual_fourcc or 'UNKNOWN'}；程序禁止使用 YUYV"
            )
        if (
            self.actual_width != self.requested_width
            or self.actual_height != self.requested_height
        ):
            self.capture.release()
            raise RuntimeError(
                "摄像头没有接受指定分辨率："
                f"实际={self.actual_width}x{self.actual_height}，"
                f"请求={self.requested_width}x{self.requested_height}"
            )
        if self.actual_fps > 0 and abs(self.actual_fps - self.requested_fps) > 0.5:
            self.capture.release()
            raise RuntimeError(
                "摄像头没有接受指定帧率："
                f"实际={self.actual_fps:.3f}，请求={self.requested_fps:.3f}"
            )

        self.condition = threading.Condition()
        self.running = False
        self.frame = None
        self.sequence = 0
        self.reader_error = None
        self.thread = None

    @staticmethod
    def decode_fourcc(value):
        value = int(round(value))
        return "".join(
            chr((value >> (8 * index)) & 0xFF)
            for index in range(4)
        ).rstrip("\x00 ")

    def start(self):
        if self.running:
            return self

        self.running = True
        self.thread = threading.Thread(
            target=self._reader_loop,
            name="camera-latest-frame",
            daemon=True,
        )
        self.thread.start()
        return self

    def _reader_loop(self):
        try:
            while self.running:
                ok, frame = self.capture.read()
                if not self.running:
                    break
                if not ok or frame is None:
                    time.sleep(0.002)
                    continue

                with self.condition:
                    self.frame = frame
                    self.sequence += 1
                    self.condition.notify_all()
        except BaseException as error:
            self.reader_error = error
        finally:
            self.running = False
            with self.condition:
                self.condition.notify_all()

    def read_latest(self, previous_sequence=-1, timeout=2.0):
        deadline = time.monotonic() + timeout

        with self.condition:
            while (
                self.running
                and self.reader_error is None
                and (self.frame is None or self.sequence == previous_sequence)
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)

            if self.reader_error is not None:
                raise RuntimeError("摄像头采集线程异常") from self.reader_error
            if self.frame is None or self.sequence == previous_sequence:
                return False, None, previous_sequence

            return True, self.frame.copy(), self.sequence

    def stop(self):
        self.running = False
        with self.condition:
            self.condition.notify_all()

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        self.capture.release()


class USBController:
    """USB CDC 接收统一 B2 目标命令，并持续发送识别坐标帧。"""

    RX_HEADER = bytes((0xFF, 0xB2))
    RX_FRAME_LENGTH = 4
    TX_TYPE = 0xB2

    def __init__(
        self,
        port,
        baudrate=115200,
        reconnect_interval=0.5,
        serial_factory=None,
        port_finder=None,
    ):
        # STM32 reset/烧录时 CDC 会从 /dev 暂时消失，因此串口由后台线程
        # 打开并自动重连。baudrate 只是 CDC 线路编码参数，不是 USB 速率。
        self.requested_port = str(port)
        self.baudrate = int(baudrate)
        self.reconnect_interval = max(0.01, float(reconnect_interval))
        self.serial_factory = serial_factory or serial.Serial
        self.port_finder = port_finder or self._discover_port
        self.serial = None
        self.connected_port = None
        self.running = False
        self.thread = None
        self.reader_error = None
        self.rx_buffer = bytearray()
        self.state_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.connection_lock = threading.Lock()
        self._waiting_announced = False
        self._last_connect_error = None
        self.disconnect_requested_for = None
        self.disconnect_request_error = None

        # 上电默认不识别；只有收到合法命令后才设置目标。
        self.active_command = None
        self.command_version = 0

    def _discover_port(self):
        """优先使用指定设备，其次自动寻找 STM32 by-id 和 ttyACM。"""
        candidates = []
        if self.requested_port:
            candidates.append(self.requested_port)
        candidates.extend(
            sorted(
                glob.glob(
                    "/dev/serial/by-id/"
                    "usb-STMicroelectronics_STM32_Virtual_ComPort_*-if00"
                )
            )
        )
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))

        seen = set()
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            if os.path.exists(candidate):
                return candidate
        return None

    def _current_serial(self):
        with self.connection_lock:
            return self.serial

    def is_connected(self):
        serial_connection = self._current_serial()
        return serial_connection is not None and getattr(
            serial_connection,
            "is_open",
            True,
        )

    def _try_connect(self):
        if self.is_connected():
            return True

        port = self.port_finder()
        if not port:
            if not self._waiting_announced:
                print("未检测到 STM32 USB CDC，程序保持运行并等待设备")
                self._waiting_announced = True
            return False

        try:
            serial_connection = self.serial_factory(
                port=port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
                write_timeout=0.2,
            )
        except (serial.SerialException, OSError) as error:
            error_key = (port, str(error))
            if error_key != self._last_connect_error:
                print(f"USB CDC暂时无法打开，继续重试: {port}: {error}")
                self._last_connect_error = error_key
            self._waiting_announced = True
            return False

        with self.connection_lock:
            if not self.running:
                publish_connection = False
                old_connection = None
            else:
                publish_connection = True
                old_connection = self.serial
                self.serial = serial_connection
                self.connected_port = port
                self.disconnect_requested_for = None
                self.disconnect_request_error = None

        if not publish_connection:
            try:
                serial_connection.close()
            except (serial.SerialException, OSError):
                pass
            return False

        # 只有读取线程会打开连接和处理接收缓冲，避免与发送线程并发修改。
        self.rx_buffer.clear()

        if old_connection is not None and old_connection is not serial_connection:
            try:
                old_connection.close()
            except (serial.SerialException, OSError):
                pass

        self._waiting_announced = False
        self._last_connect_error = None
        print(f"USB CDC设备连接成功: {port}, {self.baudrate}, 8N1")
        return True

    def start(self):
        if self.running:
            return self

        self.running = True
        self.thread = threading.Thread(
            target=self._reader_loop,
            name="usb-target-command",
            daemon=True,
        )
        self.thread.start()
        return self

    def _reader_loop(self):
        try:
            while self.running:
                serial_connection = self._current_serial()
                if serial_connection is not None:
                    requested_error = self._take_disconnect_request(
                        serial_connection
                    )
                    if requested_error is not None:
                        self._mark_disconnected(
                            serial_connection,
                            requested_error,
                        )
                        continue

                if not self._try_connect():
                    time.sleep(self.reconnect_interval)
                    continue

                serial_connection = self._current_serial()
                if serial_connection is None:
                    continue

                try:
                    waiting = serial_connection.in_waiting
                    data = serial_connection.read(waiting if waiting > 0 else 1)
                    if not data:
                        continue

                    self.rx_buffer.extend(data)
                    self._parse_rx_buffer()
                except (serial.SerialException, OSError) as error:
                    self._mark_disconnected(serial_connection, error)
        except BaseException as error:
            self.reader_error = error
        finally:
            self.running = False

    def _parse_rx_buffer(self):
        """从任意分段或粘连的字节流中提取 FF B2 Command FF。"""
        while True:
            header_index = self.rx_buffer.find(self.RX_HEADER)

            if header_index < 0:
                # 最后一个 FF 可能是下一帧的帧头，需要保留。
                if self.rx_buffer[-1:] == b"\xFF":
                    self.rx_buffer[:] = b"\xFF"
                else:
                    self.rx_buffer.clear()
                return

            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < self.RX_FRAME_LENGTH:
                return

            if self.rx_buffer[3] != 0xFF:
                del self.rx_buffer[0]
                continue

            command = int(self.rx_buffer[2])
            del self.rx_buffer[:self.RX_FRAME_LENGTH]

            if command not in VALID_COMMAND_IDS:
                print(f"收到无效目标命令: 0x{command:02X}")
                continue

            with self.state_lock:
                # 姿态模式依赖连续三帧稳定窗口；下位机周期性重发命令时
                # 只保持当前模式。其他旧命令仍沿用“每帧重新激活”的行为。
                command_changed = (
                    command != self.active_command
                    or command not in {
                        CAMERA_THREE_RING_POSE,
                        CAMERA_PILE_LINE_POSE,
                    }
                )
                self.active_command = command
                if command_changed:
                    self.command_version += 1

            if command_changed:
                print(
                    "收到目标命令: "
                    f"FF B2 {command:02X} FF -> {COMMAND_NAMES[command]}"
                )

    def get_command(self):
        if self.reader_error is not None:
            raise RuntimeError("USB CDC接收线程异常") from self.reader_error

        with self.state_lock:
            return self.active_command, self.command_version

    @staticmethod
    def clamp_int16(value):
        return max(-32768, min(32767, int(round(value))))

    @staticmethod
    def clamp_uint16(value):
        return max(0, min(65535, int(round(value))))

    def _mark_disconnected(self, serial_connection, error):
        """由读取线程释放失效句柄，并保留当前目标命令。"""
        was_current = False
        with self.connection_lock:
            if self.serial is serial_connection:
                self.serial = None
                self.connected_port = None
                if self.disconnect_requested_for is serial_connection:
                    self.disconnect_requested_for = None
                    self.disconnect_request_error = None
                was_current = True

        # 接收缓冲只归读取线程所有；发送线程不能在解析期间清空它。
        if was_current:
            self.rx_buffer.clear()

        # 等待正在执行的 write() 结束后再关闭，避免 pyserial 的 fd 在
        # 写入中途被置空并抛出未捕获的 TypeError。
        try:
            with self.write_lock:
                serial_connection.close()
        except (serial.SerialException, OSError):
            pass

        if was_current:
            self._waiting_announced = True
            print(f"USB CDC已断开，等待自动重连: {error}")

    def _request_disconnect(self, serial_connection, error):
        """发送线程只提交断线请求，实际关闭和清缓冲由读取线程完成。"""
        with self.connection_lock:
            if self.serial is serial_connection:
                self.disconnect_requested_for = serial_connection
                self.disconnect_request_error = error

    def _take_disconnect_request(self, serial_connection):
        with self.connection_lock:
            if self.disconnect_requested_for is not serial_connection:
                return None
            error = self.disconnect_request_error
            self.disconnect_requested_for = None
            self.disconnect_request_error = None
            return error

    def send_detection(self, command, detection):
        """发送固定14字节结果帧；姿态命令使用有符号误差与停止标志。"""
        if command not in VALID_COMMAND_IDS:
            return False

        serial_connection = self._current_serial()
        if serial_connection is None or not getattr(
            serial_connection,
            "is_open",
            True,
        ):
            return False

        valid = 1 if detection.get("detected", False) else 0
        center = detection.get("center")

        if command == CAMERA_THREE_RING_POSE:
            if valid:
                error_x = self.clamp_int16(detection.get("error_x", 0))
                error_y = self.clamp_int16(detection.get("error_y", 0))
                angle_error = self.clamp_int16(
                    detection.get("angle_error_cdeg", 0)
                )
                stop_correction = 1 if detection.get(
                    "stop_correction", False
                ) else 0
            else:
                error_x = 0
                error_y = 0
                angle_error = 0
                stop_correction = 0
            body = struct.pack(
                ">BBBBhhhh",
                0xFF,
                self.TX_TYPE,
                int(command),
                valid,
                error_x,
                error_y,
                angle_error,
                stop_correction,
            )
        elif command == CAMERA_PILE_LINE_POSE:
            if valid:
                angle_error = self.clamp_int16(
                    detection.get("angle_error_cdeg", 0)
                )
                stop_correction = 1 if detection.get(
                    "stop_correction", False
                ) else 0
            else:
                angle_error = 0
                stop_correction = 0
            body = struct.pack(
                ">BBBBhhhh",
                0xFF,
                self.TX_TYPE,
                int(command),
                valid,
                0,
                0,
                angle_error,
                stop_correction,
            )
        elif valid and center is not None:
            center_x = self.clamp_uint16(center[0])
            center_y = self.clamp_uint16(center[1])
            error_x = self.clamp_int16(detection.get("error_x", 0))
            error_y = self.clamp_int16(detection.get("error_y", 0))
            body = struct.pack(
                ">BBBBHHhh",
                0xFF,
                self.TX_TYPE,
                int(command),
                valid,
                center_x,
                center_y,
                error_x,
                error_y,
            )
        else:
            center_x = 0
            center_y = 0
            error_x = 0
            error_y = 0
            body = struct.pack(
                ">BBBBHHhh",
                0xFF,
                self.TX_TYPE,
                int(command),
                valid,
                center_x,
                center_y,
                error_x,
                error_y,
            )
        checksum = sum(body[1:]) & 0xFF
        packet = body + bytes((checksum, 0xFE))

        try:
            with self.write_lock:
                if (
                    self._current_serial() is not serial_connection
                    or not getattr(serial_connection, "is_open", True)
                ):
                    return False
                written = serial_connection.write(packet)
                if written != len(packet):
                    raise OSError(
                        f"USB CDC 写入不完整: {written}/{len(packet)}"
                    )
        except (serial.SerialException, OSError) as error:
            self._request_disconnect(serial_connection, error)
            return False
        return True

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        with self.connection_lock:
            serial_connection = self.serial
            self.serial = None
            self.connected_port = None
            self.disconnect_requested_for = None
            self.disconnect_request_error = None

        if serial_connection is not None and getattr(
            serial_connection,
            "is_open",
            True,
        ):
            serial_connection.close()


class ColorCircleDetector:
    """先进行颜色轮廓粗定位，再进行色环精定位。"""

    def __init__(
        self,
        center_x=COLOR_REFERENCE_CENTER_X,
        center_y=COLOR_REFERENCE_CENTER_Y,
        min_contour_area=2000,
        min_color_area=150,
        switch_tolerance=50,
        circle_lost_limit=5,
    ):
        self.kernel = np.ones((5, 5), dtype=np.uint8)
        self.min_contour_area = float(min_contour_area)
        self.min_color_area = int(min_color_area)
        self.switch_tolerance = int(switch_tolerance)
        self.circle_lost_limit = max(1, int(circle_lost_limit))
        self.center_x = int(center_x)
        self.center_y = int(center_y)
        self.choose_color = None
        self.mode = "coarse"
        self.blue_circle_delta_x = -1
        self.blue_circle_delta_y = 2
        # 摄像头抬高后目标会变小；面积、圆度和包围圆填充率共同抑制小噪点。
        self.min_circle_radius = 8.0
        self.max_circle_radius = 180.0
        self.min_circle_circularity = 0.45
        self.min_enclosing_fill = 0.76
        self.coarse_filter = AdaptiveCenterFilter()
        self.center_filter = AdaptiveCenterFilter()
        self.last_circle_center = None
        self.last_circle_radius = None
        self.circle_miss_count = 0

    def set_color(self, color):
        if color not in VALID_COLOR_IDS:
            raise ValueError(f"不支持的颜色编号：{color}")
        self.choose_color = int(color)
        self.mode = "coarse"
        self.coarse_filter = AdaptiveCenterFilter()
        self.center_filter = AdaptiveCenterFilter()
        self.last_circle_center = None
        self.last_circle_radius = None
        self.circle_miss_count = 0

    def reset(self):
        self.choose_color = None
        self.mode = "coarse"
        self.coarse_filter = AdaptiveCenterFilter()
        self.center_filter = AdaptiveCenterFilter()
        self.last_circle_center = None
        self.last_circle_radius = None
        self.circle_miss_count = 0

    def make_color_mask(self, frame, ranges, hsv=None):
        if self.choose_color not in VALID_COLOR_IDS:
            raise RuntimeError("尚未收到有效颜色命令，不能执行 HSV 识别")

        if hsv is None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in ranges[self.choose_color]:
            one_range_mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_or(mask, one_range_mask)

        return mask

    def make_material_mask(self, frame, hsv=None):
        """使用 wuliao.py 的颜色范围及形态学处理生成粗定位掩码。"""
        mask = self.make_color_mask(frame, MATERIAL_HSV_RANGES, hsv=hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def make_circle_mask(self, frame, hsv=None):
        """生成色环候选区域的颜色校验掩码。"""
        return self.make_color_mask(frame, CIRCLE_HSV_RANGES, hsv=hsv)

    def is_color(self, frame):
        """候选圆 ROI 中指定颜色的像素面积是否达到阈值。"""
        if frame.size == 0:
            return False, 0

        color_area = int(cv2.countNonZero(self.make_circle_mask(frame)))
        return color_area > self.min_color_area, color_area

    def find_circle_candidates(self, mask):
        """在原分辨率颜色掩码上提取并精修圆形候选。"""
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates = []

        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area < max(20.0, self.min_color_area * 0.5):
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue

            circularity = float(4.0 * np.pi * contour_area / (perimeter * perimeter))
            if circularity < self.min_circle_circularity:
                continue

            (_, _), radius = cv2.minEnclosingCircle(contour)
            if not self.min_circle_radius <= radius <= self.max_circle_radius:
                continue

            enclosing_fill = float(contour_area / (np.pi * radius * radius))
            if enclosing_fill < self.min_enclosing_fill:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            center_x = float(moments["m10"] / moments["m00"])
            center_y = float(moments["m01"] / moments["m00"])
            x, y, width, height = cv2.boundingRect(contour)
            color_area = int(cv2.countNonZero(mask[y:y + height, x:x + width]))
            if color_area <= self.min_color_area:
                continue

            candidates.append(
                {
                    "center": (center_x, center_y),
                    "radius": float(radius),
                    "contour_area": contour_area,
                    "color_area": color_area,
                    "circularity": circularity,
                    "enclosing_fill": enclosing_fill,
                }
            )

        return candidates

    @staticmethod
    def delete_superfluous_circles(circles):
        """与源程序一致：同一圆心只保留半径最大的候选圆。"""
        remaining = np.asarray(circles).reshape((-1, 3))
        filtered = []

        while remaining.shape[0] != 0:
            same_center = np.where(
                (remaining[:, 0] == remaining[0, 0])
                & (remaining[:, 1] == remaining[0, 1])
            )[0]
            concentric = remaining[same_center]
            filtered.append(concentric[np.argmax(concentric[:, 2])])
            remaining = np.delete(remaining, same_center, axis=0)

        return np.asarray(filtered)

    @staticmethod
    def circle_squares(frame, circles):
        """截取各候选圆的外接方形 ROI。"""
        squares = []
        frame_height, frame_width = frame.shape[:2]

        for x, y, radius in circles:
            x1 = max(0, int(x - radius))
            y1 = max(0, int(y - radius))
            x2 = min(frame_width, int(x + radius))
            y2 = min(frame_height, int(y + radius))
            roi = frame[y1:y2, x1:x2]
            if roi.size != 0:
                squares.append((roi, (x, y, radius)))

        return squares

    def draw_crosshair(self, frame):
        # 小十字准星固定在配置的中心点。
        half_length = 14
        cv2.line(
            frame,
            (self.center_x - half_length, self.center_y),
            (self.center_x + half_length, self.center_y),
            (0, 255, 0),
            1,
        )
        cv2.line(
            frame,
            (self.center_x, self.center_y - half_length),
            (self.center_x, self.center_y + half_length),
            (0, 255, 0),
            1,
        )
        cv2.circle(frame, (self.center_x, self.center_y), 2, (0, 255, 0), -1)

    def make_waiting_frame(self, frame):
        result = frame.copy()
        self.draw_crosshair(result)
        return result

    def process(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.mode == "coarse":
            return self.process_material(frame, hsv=hsv)
        return self.process_circle(frame, hsv=hsv)

    def process_material(self, frame, hsv=None):
        """通过最大颜色轮廓粗定位物块；进入阈值范围后切换模式。"""
        result = frame.copy()
        mask = self.make_material_mask(frame, hsv=hsv)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        detected = False
        center = None
        area = 0.0
        max_contour = None
        error_x = 0
        error_y = 0
        state = "coarse_not_found"

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(max_contour))

            if area > self.min_contour_area:
                moments = cv2.moments(max_contour)
                if moments["m00"] != 0:
                    raw_center_x = float(moments["m10"] / moments["m00"])
                    raw_center_y = float(moments["m01"] / moments["m00"])
                    filtered_x, filtered_y = self.coarse_filter.update(
                        (raw_center_x, raw_center_y)
                    )
                    center_x = int(round(filtered_x))
                    center_y = int(round(filtered_y))
                    center = (center_x, center_y)
                    error_x = center_x - self.center_x
                    error_y = self.center_y - center_y
                    detected = True
                    state = "coarse_detected"

                    if (
                        abs(error_x) <= self.switch_tolerance
                        and abs(error_y) <= self.switch_tolerance
                    ):
                        self.mode = "circle"
                        state = "coarse_aligned"

        if detected and max_contour is not None and center is not None:
            cv2.drawContours(
                result,
                [max_contour],
                -1,
                DRAW_COLORS[self.choose_color],
                2,
            )
            cv2.circle(result, center, 5, (0, 0, 255), -1)

        self.draw_crosshair(result)

        return result, mask, {
            "detected": detected,
            "state": state,
            "mode": "coarse",
            "next_mode": self.mode,
            "color_id": self.choose_color,
            "color": COLOR_NAMES[self.choose_color],
            "target": COLOR_NAMES[self.choose_color],
            "center": center,
            "error_x": int(error_x),
            "error_y": int(error_y),
            "area": area,
        }

    def process_circle(self, frame, hsv=None):
        """在全分辨率颜色掩码上快速、精确地定位色环圆心。"""
        result = frame.copy()
        mask = self.make_circle_mask(frame, hsv=hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        color_circle_targets = self.find_circle_candidates(mask)
        detected = False
        center = None
        area = 0.0
        error_x = 0
        error_y = 0
        radius = 0.0
        state = "circle_not_found"

        target = None
        if color_circle_targets:
            if self.last_circle_center is None:
                # 首次进入精定位时沿用原程序行为：选择 Y 最大的同色圆。
                target = max(
                    color_circle_targets,
                    key=lambda item: item["center"][1],
                )
            else:
                # 跟踪阶段选择最接近上一帧的候选，避免多个同色目标之间跳变。
                nearest = min(
                    color_circle_targets,
                    key=lambda item: (
                        (item["center"][0] - self.last_circle_center[0]) ** 2
                        + (item["center"][1] - self.last_circle_center[1]) ** 2
                    ),
                )
                jump_distance = float(
                    np.hypot(
                        nearest["center"][0] - self.last_circle_center[0],
                        nearest["center"][1] - self.last_circle_center[1],
                    )
                )
                max_jump = max(35.0, float(self.last_circle_radius or 0.0) * 1.8)
                if jump_distance <= max_jump:
                    target = nearest

        if target is not None:
            target_x, target_y = target["center"]
            radius = target["radius"]
            area = target["color_area"]

            if self.choose_color == CAMERA_COLOR_BLUE:
                target_x += self.blue_circle_delta_x
                target_y += self.blue_circle_delta_y

            filtered_x, filtered_y = self.center_filter.update(
                (target_x, target_y)
            )
            center_x = int(round(filtered_x))
            center_y = int(round(filtered_y))
            center = (center_x, center_y)
            self.last_circle_center = (float(target_x), float(target_y))
            self.last_circle_radius = float(radius)
            self.circle_miss_count = 0
            error_x = center_x - self.center_x
            error_y = self.center_y - center_y
            detected = True
            state = "circle_detected"

            cv2.circle(
                result,
                center,
                int(round(radius)),
                DRAW_COLORS[self.choose_color],
                2,
            )
            cv2.circle(result, center, 4, (0, 0, 255), -1)
        else:
            self.circle_miss_count += 1
            if self.circle_miss_count >= self.circle_lost_limit:
                self.mode = "coarse"
                self.last_circle_center = None
                self.last_circle_radius = None
                self.center_filter = AdaptiveCenterFilter()
                state = "circle_lost_reacquire"

        self.draw_crosshair(result)

        return result, mask, {
            "detected": detected,
            "state": state,
            "mode": "circle",
            "next_mode": self.mode,
            "color_id": self.choose_color,
            "color": COLOR_NAMES[self.choose_color],
            "target": COLOR_NAMES[self.choose_color],
            "center": center,
            "error_x": int(error_x),
            "error_y": int(error_y),
            "area": float(area),
            "radius": float(radius),
            "method": "full_res_mask_refine",
        }


# 保留旧类名，避免其他代码若曾导入 MaterialDetector 时失效。
MaterialDetector = ColorCircleDetector


def activate_command(
    command,
    color_detector,
    digit_ring_detector,
    three_ring_detector=None,
    pile_line_detector=None,
):
    """切换统一 B2 命令，并清除其他检测模式留下的状态。"""
    mode, target = resolve_command_mode(command)
    if mode == "color":
        digit_ring_detector.reset()
        if three_ring_detector is not None:
            three_ring_detector.reset()
        if pile_line_detector is not None:
            pile_line_detector.reset()
        color_detector.set_color(target)
    elif mode == "digit_ring":
        color_detector.reset()
        if three_ring_detector is not None:
            three_ring_detector.reset()
        if pile_line_detector is not None:
            pile_line_detector.reset()
        digit_ring_detector.set_digit(target)
    elif mode == "three_ring_pose":
        if three_ring_detector is None:
            raise ValueError("三环姿态模式缺少检测器")
        color_detector.reset()
        digit_ring_detector.reset()
        if pile_line_detector is not None:
            pile_line_detector.reset()
        three_ring_detector.reset()
    else:
        if pile_line_detector is None:
            raise ValueError("码堆黑线姿态模式缺少检测器")
        color_detector.reset()
        digit_ring_detector.reset()
        if three_ring_detector is not None:
            three_ring_detector.reset()
        pile_line_detector.reset()
    return mode


def process_detection_mode(
    active_mode,
    frame,
    color_detector,
    digit_ring_detector,
    three_ring_detector,
    pile_line_detector=None,
):
    """运行当前已激活的检测器，并保持各模式的返回格式一致。"""
    if active_mode == "color":
        return color_detector.process(frame)
    if active_mode == "digit_ring":
        return digit_ring_detector.process(frame)
    if active_mode == "three_ring_pose":
        return three_ring_detector.process(frame)
    if active_mode == "pile_line_pose":
        if pile_line_detector is None:
            raise ValueError("码堆黑线姿态模式缺少检测器")
        return pile_line_detector.process(frame)
    raise ValueError(f"未知检测模式：{active_mode}")


def draw_status(frame, detection, fps):
    if detection["state"] == "waiting_usb":
        state_color = (180, 180, 180)
    elif detection["detected"]:
        state_color = (0, 255, 0)
    else:
        state_color = (0, 255, 255)

    center_text = (
        f"({detection['center'][0]}, {detection['center'][1]})"
        if detection["center"] is not None
        else "none"
    )
    mode = detection.get("mode", "unknown")
    next_mode = detection.get("next_mode", mode)
    mode_text = mode if next_mode == mode else f"{mode} -> {next_mode}"
    target_name = detection.get("target", detection.get("color", "none"))
    lines = [
        f"Target: {target_name}  State: {detection['state']}",
        f"Mode: {mode_text}",
        f"Center: {center_text}",
        f"Error X: {detection['error_x']}",
        f"Error Y: {detection['error_y']}",
    ]
    if mode == "digit_ring":
        phase = detection.get("phase", "digit_coarse")
        next_phase = detection.get("next_phase", phase)
        phase_text = phase if next_phase == phase else f"{phase} -> {next_phase}"
        lines[1] = f"Mode: digit_ring  Stage: {phase_text}"
        if phase == "digit_coarse":
            lines.append(
                f"Digit score: {detection.get('digit_score', 0.0):.3f}  "
                f"Area: {detection.get('area', 0.0):.0f}  "
                f"Rings: {detection.get('coarse_ring_count', 0)}  "
                f"Align: {detection.get('aligned_count', 0)}/"
                f"{detection.get('aligned_required', DIGIT_RING_ALIGNED_CONFIRM_FRAMES)}"
            )
        else:
            lines.append(
                f"Hough: {detection.get('hough_count', 0)}  "
                f"Groups: {detection.get('ring_group_count', 0)}  "
                f"Rings: {detection.get('circle_count', 0)}  "
                f"Miss: {detection.get('miss_count', 0)}"
            )
    elif mode == "three_ring_pose":
        lines[1] = (
            "Mode: three_ring_pose  "
            f"Stable: {detection.get('stable_count', 0)}/"
            f"{detection.get('stable_required', 3)}"
        )
        lines.append(
            f"Yaw: {detection.get('station_yaw_deg', 0.0):+.2f} deg  "
            f"Angle error: {detection.get('angle_error_deg', 0.0):+.2f} deg"
        )
        lines.append(
            "Correction: "
            + ("STOP" if detection.get("stop_correction", False) else "RUN")
            + f"  Deadzone: +/-{THREE_RING_ANGLE_DEADZONE_DEG:.1f} deg  "
            f"Confidence: {detection.get('confidence', 0.0):.3f}"
        )
    elif mode == "pile_line_pose":
        lines[1] = (
            "Mode: pile_line_pose  "
            f"Stable: {detection.get('stable_count', 0)}/"
            f"{detection.get('stable_required', PILE_LINE_STABLE_FRAMES)}"
        )
        lines.append(
            f"Line angle: {detection.get('station_yaw_deg', 0.0):+.2f} deg  "
            f"Angle error: {detection.get('angle_error_deg', 0.0):+.2f} deg"
        )
        lines.append(
            "Correction: "
            + ("STOP" if detection.get("stop_correction", False) else "RUN")
            + f"  Deadzone: +/-{PILE_LINE_ANGLE_DEADZONE_DEG:.1f} deg  "
            f"Span: {detection.get('span_ratio', 0.0) * 100.0:.0f}%"
        )
    else:
        lines.append(f"Area: {detection['area']:.0f}")
    lines.append(f"FPS: {fps:.1f}")

    for index, text in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (10, 28 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.66,
            state_color,
            2,
            cv2.LINE_AA,
        )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="HSV color-circle + Hough digit-concentric-ring detector"
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=120.0)
    parser.add_argument(
        "--center-x",
        type=int,
        default=COLOR_REFERENCE_CENTER_X,
        help="颜色模式准心 X，默认 357；数字同心圆固定使用 367",
    )
    parser.add_argument(
        "--center-y",
        type=int,
        default=COLOR_REFERENCE_CENTER_Y,
        help="颜色模式准心 Y，默认 324；数字同心圆固定使用 316",
    )
    parser.add_argument(
        "--coarse-min-area",
        type=float,
        default=2000.0,
        help="minimum contour area for coarse positioning, default: 2000",
    )
    parser.add_argument(
        "--min-area",
        type=int,
        default=150,
        help="minimum target-color pixel area inside a circle ROI, default: 150",
    )
    parser.add_argument(
        "--digit-min-area",
        type=float,
        default=DIGIT_COARSE_MIN_AREA_640X480,
        help=(
            "minimum digit contour area normalized to 640x480, "
            "default: 350"
        ),
    )
    parser.add_argument(
        "--switch-tolerance",
        type=int,
        default=50,
        help="switch to circle mode when both axis errors are within this value",
    )
    parser.add_argument("--usb-port", "--uart-port", dest="usb_port", required=True,
                        help="实际 STM32 USB CDC 设备路径，建议使用 /dev/serial/by-id/...")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--send-hz",
        type=float,
        default=50.0,
        help="USB CDC coordinate send rate, default: 50 Hz",
    )
    parser.add_argument("--print-interval", type=float, default=1.0)
    return parser.parse_args()


def build_detectors(args):
    """按模式分别建立检测器，避免颜色准心覆盖数字圆环准心。"""
    color_detector = ColorCircleDetector(
        center_x=args.center_x,
        center_y=args.center_y,
        min_contour_area=args.coarse_min_area,
        min_color_area=args.min_area,
        switch_tolerance=args.switch_tolerance,
    )
    digit_ring_detector = DigitRingDetector(
        center_x=DIGIT_RING_REFERENCE_CENTER_X,
        center_y=DIGIT_RING_REFERENCE_CENTER_Y,
        digit_min_area=args.digit_min_area,
    )
    return color_detector, digit_ring_detector


def main():
    args = parse_arguments()
    configure_processing_runtime()

    color_detector, digit_ring_detector = build_detectors(args)
    three_ring_detector = ThreeRingPoseDetector()
    pile_line_detector = PileLinePoseDetector()
    active_mode = None
    usb = USBController(
        port=args.usb_port,
        baudrate=args.baudrate,
    ).start()
    camera = None

    previous_sequence = -1
    previous_frame_time = time.monotonic()
    last_print_time = 0.0
    last_command_version = 0
    next_send_time = 0.0
    display_fps = 0.0
    send_interval = 1.0 / max(1.0, args.send_hz)

    try:
        camera = LatestFrameCamera(
            camera_index=args.camera,
            width=args.width,
            height=args.height,
            fps=args.fps,
        ).start()

        print(
            f"Camera {args.camera}: "
            f"{camera.actual_width}x{camera.actual_height}, "
            f"{camera.actual_fourcc}, {camera.actual_fps:.3f} FPS"
        )
        print(
            "等待 USB CDC 命令：FF B2 Command FF；"
            "01-06=颜色，0A-0C=数字同心圆，"
            "0D=三环姿态，0E=码堆黑线姿态"
        )
        print("按 q 或 Esc 退出")

        while True:
            ok, frame, sequence = camera.read_latest(
                previous_sequence,
                timeout=2.0,
            )
            if not ok:
                print("等待摄像头最新帧超时")
                continue

            previous_sequence = sequence
            active_command, command_version = usb.get_command()

            if command_version != last_command_version:
                active_mode = activate_command(
                    active_command,
                    color_detector,
                    digit_ring_detector,
                    three_ring_detector,
                    pile_line_detector,
                )
                last_command_version = command_version
                next_send_time = 0.0

            if active_command is None:
                # 上电等待阶段：只显示原图、准星和状态，不执行识别。
                result = color_detector.make_waiting_frame(frame)
                detection = {
                    "detected": False,
                    "state": "waiting_usb",
                    "mode": "waiting",
                    "next_mode": "waiting",
                    "color_id": None,
                    "color": "none",
                    "target": "none",
                    "center": None,
                    "error_x": 0,
                    "error_y": 0,
                    "area": 0.0,
                }
            else:
                result, _, detection = process_detection_mode(
                    active_mode,
                    frame,
                    color_detector,
                    digit_ring_detector,
                    three_ring_detector,
                    pile_line_detector,
                )

            if active_command is not None:
                detection["command_id"] = active_command
                detection.setdefault("target", COMMAND_NAMES[active_command])

            completed_time = time.monotonic()
            elapsed = completed_time - previous_frame_time
            previous_frame_time = completed_time
            if elapsed > 0:
                instant_fps = 1.0 / elapsed
                display_fps = (
                    instant_fps
                    if display_fps <= 0
                    else 0.9 * display_fps + 0.1 * instant_fps
                )

            draw_status(result, detection, display_fps)

            # 收到首个目标命令后持续发送；未检测到时发送 Valid=0 和零坐标。
            if active_command is not None and completed_time >= next_send_time:
                usb.send_detection(active_command, detection)
                next_send_time = completed_time + send_interval

            if completed_time - last_print_time >= args.print_interval:
                target_name = detection.get(
                    "target",
                    detection.get("color", "none"),
                )
                if active_command is None:
                    print("等待 USB CDC 目标命令，当前未执行识别")
                elif detection["detected"]:
                    print(
                        f"[{detection['mode']}] {target_name} "
                        f"中心={detection['center']} "
                        f"误差=({detection['error_x']}, "
                        f"{detection['error_y']}) "
                        f"面积={detection['area']:.0f} "
                        f"FPS={display_fps:.1f}"
                    )
                else:
                    print(
                        f"[{detection['mode']}] "
                        f"未检测到 {target_name}  "
                        f"FPS={display_fps:.1f}"
                    )
                last_print_time = completed_time

            cv2.imshow("Color + digit ring detection", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        if camera is not None:
            camera.stop()
        usb.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
