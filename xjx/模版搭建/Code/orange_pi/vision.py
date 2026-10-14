#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颜色粗定位/色环精定位 + 数字同心圆定位程序。

颜色命令沿用原程序的 HSV 粗定位与色环精定位流程；数字命令使用
Hough 多重同心圆检测，并识别圆心内的数字 1、2、3。两种模式共用
物料可配置准心（默认 372,345）、编号环独立准心（默认 378,333）及 B2 通信协议。

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

坐标发送帧固定为 14 字节：
    FF B2 Command Valid CX_H CX_L CY_H CY_L DX_H DX_L DY_H DY_L SUM FE

    Valid：1=检测有效，0=当前未检测到目标
    CX/CY：目标中心像素坐标，无符号 16 位，大端序
    DX：CX - 准心X，有符号 16 位，大端序，向右为正
    DY：准心Y - CY，有符号 16 位，大端序，向上为正
    SUM：从 B2 到 DY_L 所有字节求和后取低 8 位

按 q 或 Esc 退出。
"""

import argparse
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

VALID_COMMAND_IDS = VALID_COLOR_IDS | set(DIGIT_RING_COMMAND_TO_DIGIT)

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
HOUGH_PARAM2 = 0.72
HOUGH_MIN_RADIUS = 18
HOUGH_MAX_RADIUS = 65

CENTER_DISTANCE_THRESHOLD = 12
MIN_RING_COUNT = 3
RADIUS_MERGE_THRESHOLD = 3.0
MIN_GROUP_RADIUS = 25
MAX_GROUP_RADIUS = 65

DIGIT_IMAGE_SIZE = 48
DIGIT_ROI_INNER_RADIUS_RATIO = 0.78
DIGIT_ROI_OUTER_RADIUS_RATIO = 0.50
DIGIT_ROI_MIN_RADIUS = 10
DIGIT_MIN_SCORE = 0.50
DIGIT_MIN_MARGIN = 0.015


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


def validate_ring_groups(groups):
    """保留至少包含三个不同半径、尺寸合规的同心圆组。"""
    valid_groups = []

    for group in groups:
        circles = remove_duplicate_circles(group["circles"])
        if len(circles) < MIN_RING_COUNT:
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
    """把统一 B2 命令映射到颜色模式或数字同心圆模式。"""
    if command in VALID_COLOR_IDS:
        return "color", command
    if command in DIGIT_RING_COMMAND_TO_DIGIT:
        return "digit_ring", DIGIT_RING_COMMAND_TO_DIGIT[command]
    raise ValueError(f"不支持的 B2 命令：0x{command:02X}")


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

    def __init__(self):
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

        for angle in (-12, -6, 0, 6, 12):
            matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            rotated = cv2.warpAffine(
                image,
                matrix,
                (width, height),
                flags=cv2.INTER_LINEAR,
                borderValue=0,
            )
            _, rotated = cv2.threshold(rotated, 80, 255, cv2.THRESH_BINARY)
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


class DigitRingDetector:
    """检测多重同心圆，并按中央数字 1、2、3 选择目标。"""

    def __init__(self, center_x=378, center_y=333, recognizer=None):
        self.center_x = int(center_x)
        self.center_y = int(center_y)
        self.target_digit = None
        self.recognizer = recognizer or Digit123Recognizer()

    def set_digit(self, digit):
        if digit not in (1, 2, 3):
            raise ValueError(f"不支持的数字同心圆：{digit}")
        self.target_digit = int(digit)

    def reset(self):
        self.target_digit = None

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

    def process(self, frame):
        if self.target_digit not in (1, 2, 3):
            raise RuntimeError("尚未收到有效数字同心圆命令")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(
            gray,
            (GAUSSIAN_SIZE, GAUSSIAN_SIZE),
            GAUSSIAN_SIGMA,
        )
        hough_circles = detect_hough_circles(blurred)
        valid_groups = validate_ring_groups(
            group_concentric_circles(hough_circles)
        )

        for group in valid_groups:
            digit, score, margin = self.recognizer.recognize_group(gray, group)
            group["detected_digit"] = digit
            group["digit_score"] = float(score)
            group["digit_margin"] = float(margin)

        target_group = select_target_ring(valid_groups, self.target_digit)
        detection = build_digit_ring_detection(
            target_group,
            target_digit=self.target_digit,
            reference_center=(self.center_x, self.center_y),
        )
        detection["hough_count"] = len(hough_circles)
        detection["ring_group_count"] = len(valid_groups)

        result = frame.copy()
        if target_group is not None:
            for circle in target_group["circles"]:
                circle_center = (
                    int(round(circle["center"][0])),
                    int(round(circle["center"][1])),
                )
                cv2.circle(
                    result,
                    circle_center,
                    int(round(circle["radius"])),
                    (0, 0, 255),
                    2,
                )

            target_center = detection["center"]
            cv2.circle(result, target_center, 4, (0, 0, 255), -1)

        self.draw_crosshair(result)
        return result, gray, detection


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

    def __init__(self, port, baudrate=115200):
        # pyserial 打开 USB CDC 虚拟串口；baudrate 是线路编码参数，不是 USB 传输速率。
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.1,
            write_timeout=0.2,
        )

        print(f"USB CDC设备打开成功: {port}, {baudrate}, 8N1")

        self.running = False
        self.thread = None
        self.reader_error = None
        self.rx_buffer = bytearray()
        self.state_lock = threading.Lock()
        self.write_lock = threading.Lock()

        # 上电默认不识别；只有收到合法命令后才设置目标。
        self.active_command = None
        self.command_version = 0

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
                waiting = self.serial.in_waiting
                data = self.serial.read(waiting if waiting > 0 else 1)
                if not data:
                    continue

                self.rx_buffer.extend(data)
                self._parse_rx_buffer()
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
                self.active_command = command
                self.command_version += 1

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

    def send_detection(self, command, detection):
        """发送固定 14 字节坐标帧；识别无效时坐标和误差均发送 0。"""
        if command not in VALID_COMMAND_IDS:
            return

        valid = 1 if detection.get("detected", False) else 0
        center = detection.get("center")

        if valid and center is not None:
            center_x = self.clamp_uint16(center[0])
            center_y = self.clamp_uint16(center[1])
            error_x = self.clamp_int16(detection.get("error_x", 0))
            error_y = self.clamp_int16(detection.get("error_y", 0))
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

        with self.write_lock:
            written = self.serial.write(packet)
            if written != len(packet):
                raise OSError(f"USB CDC 写入不完整: {written}/{len(packet)}")

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.serial.is_open:
            self.serial.close()


class ColorCircleDetector:
    """先进行颜色轮廓粗定位，再进行色环精定位。"""

    def __init__(
        self,
        center_x=372,
        center_y=345,
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


def activate_command(command, color_detector, digit_ring_detector):
    """切换统一 B2 命令，并清除另一种检测模式留下的状态。"""
    mode, target = resolve_command_mode(command)
    if mode == "color":
        digit_ring_detector.reset()
        color_detector.set_color(target)
    else:
        color_detector.reset()
        digit_ring_detector.set_digit(target)
    return mode


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
        lines.append(
            f"Circles: {detection.get('circle_count', 0)}  "
            f"Score: {detection.get('digit_score', 0.0):.3f}"
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
        default=372,
        help="物料颜色准心 X，默认 372",
    )
    parser.add_argument(
        "--center-y",
        type=int,
        default=345,
        help="物料颜色准心 Y，默认 345",
    )
    parser.add_argument("--ring-center-x", type=int, default=378,
                        help="编号色环独立准心 X，默认 378")
    parser.add_argument("--ring-center-y", type=int, default=333,
                        help="编号色环独立准心 Y，默认 333")
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


def main():
    args = parse_arguments()
    configure_processing_runtime()

    center_x = args.center_x
    center_y = args.center_y

    color_detector = ColorCircleDetector(
        center_x=center_x,
        center_y=center_y,
        min_contour_area=args.coarse_min_area,
        min_color_area=args.min_area,
        switch_tolerance=args.switch_tolerance,
    )
    digit_ring_detector = DigitRingDetector(
        center_x=args.ring_center_x,
        center_y=args.ring_center_y,
    )
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
            "01-06=颜色，0A-0C=数字同心圆"
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
            elif active_mode == "color":
                result, _, detection = color_detector.process(frame)
            else:
                result, _, detection = digit_ring_detector.process(frame)

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
