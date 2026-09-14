#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颜色粗定位 + 色环精定位程序。

检测分为两个阶段：
    1. 收到颜色命令后，先用 wuliao.py 的 HSV 最大轮廓算法粗定位；
    2. 粗定位的 X、Y 误差都不超过 50 像素时，切换到色环精定位；
    3. 色环阶段使用 HoughCircles 检测圆并去除重复同心圆；
    4. 截取候选圆 ROI，用 HSV 像素面积确认目标颜色；
    5. 有多个同色圆时选择 Y 坐标最大的圆；
    6. 蓝色色环增加 (-1, +2) 像素补偿；
    7. 收到新的颜色命令时重新从粗定位开始。

摄像头固定请求 V4L2 + MJPG，默认 640x480、120 FPS。
采集线程持续覆盖最新帧，主线程不会处理摄像头积压的旧帧。

USB CDC：用 --usb-port 指定实际设备路径，timeout=0.1，write_timeout=0.2。
例如 python3 change.py --usb-port /dev/serial/by-id/实际设备名。
115200仅为CDC线路编码参数，USB数据速率由USB总线决定。
上电后不进行识别；收到 FF B2 Color FF 后才开始两阶段识别。

Color：
    0x01 红色
    0x02 黄色
    0x03 蓝色
    0x04 绿色
    0x05 黑色
    0x06 浅蓝

坐标发送帧固定为 14 字节：
    FF B2 Color Valid CX_H CX_L CY_H CY_L DX_H DX_L DY_H DY_L SUM FE

    Valid：1=检测有效，0=当前未检测到目标
    CX/CY：目标中心像素坐标，无符号 16 位，大端序
    DX：CX - 准心X，有符号 16 位，大端序，向右为正
    DY：准心Y - CY，有符号 16 位，大端序，向上为正
    SUM：从 B2 到 DY_L 所有字节求和后取低 8 位

按 q 或 Esc 退出。
"""

import argparse
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

VALID_COLOR_IDS = {
    CAMERA_COLOR_RED,
    CAMERA_COLOR_YELLOW,
    CAMERA_COLOR_BLUE,
    CAMERA_COLOR_GREEN,
    CAMERA_COLOR_BLACK,
    CAMERA_COLOR_LIGHT_BLUE,
}

COLOR_NAMES = {
    CAMERA_COLOR_RED: "red",
    CAMERA_COLOR_YELLOW: "yellow",
    CAMERA_COLOR_BLUE: "blue",
    CAMERA_COLOR_GREEN: "green",
    CAMERA_COLOR_BLACK: "black",
    CAMERA_COLOR_LIGHT_BLUE: "light_blue",
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
    """USB CDC 接收颜色命令，并持续发送识别坐标帧。"""

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

        # 上电默认不识别；只有收到合法命令后才设置颜色。
        self.active_color = None
        self.command_version = 0

    def start(self):
        if self.running:
            return self

        self.running = True
        self.thread = threading.Thread(
            target=self._reader_loop,
            name="usb-color-command",
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
        """从任意分段或粘连的字节流中提取 FF B2 Color FF。"""
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

            color = int(self.rx_buffer[2])
            del self.rx_buffer[:self.RX_FRAME_LENGTH]

            if color not in VALID_COLOR_IDS:
                print(f"收到无效颜色命令: 0x{color:02X}")
                continue

            with self.state_lock:
                self.active_color = color
                self.command_version += 1

            print(
                "收到颜色命令: "
                f"FF B2 {color:02X} FF -> {COLOR_NAMES[color]}"
            )

    def get_command(self):
        if self.reader_error is not None:
            raise RuntimeError("USB CDC接收线程异常") from self.reader_error

        with self.state_lock:
            return self.active_color, self.command_version

    @staticmethod
    def clamp_int16(value):
        return max(-32768, min(32767, int(round(value))))

    @staticmethod
    def clamp_uint16(value):
        return max(0, min(65535, int(round(value))))

    def send_detection(self, color, detection):
        """发送固定 14 字节坐标帧；识别无效时坐标和误差均发送 0。"""
        if color not in VALID_COLOR_IDS:
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
            int(color),
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
        center_x=334,
        center_y=338,
        min_contour_area=2000,
        min_color_area=150,
        switch_tolerance=50,
    ):
        self.kernel = np.ones((5, 5), dtype=np.uint8)
        self.min_contour_area = float(min_contour_area)
        self.min_color_area = int(min_color_area)
        self.switch_tolerance = int(switch_tolerance)
        self.center_x = int(center_x)
        self.center_y = int(center_y)
        self.choose_color = None
        self.mode = "coarse"
        self.blue_circle_delta_x = -1
        self.blue_circle_delta_y = 2

    def set_color(self, color):
        if color not in VALID_COLOR_IDS:
            raise ValueError(f"不支持的颜色编号：{color}")
        self.choose_color = int(color)
        self.mode = "coarse"

    def reset(self):
        self.choose_color = None
        self.mode = "coarse"

    def make_color_mask(self, frame, ranges):
        if self.choose_color not in VALID_COLOR_IDS:
            raise RuntimeError("尚未收到有效颜色命令，不能执行 HSV 识别")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in ranges[self.choose_color]:
            one_range_mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_or(mask, one_range_mask)

        return mask

    def make_material_mask(self, frame):
        """使用 wuliao.py 的颜色范围及形态学处理生成粗定位掩码。"""
        mask = self.make_color_mask(frame, MATERIAL_HSV_RANGES)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def make_circle_mask(self, frame):
        """生成色环候选区域的颜色校验掩码。"""
        return self.make_color_mask(frame, CIRCLE_HSV_RANGES)

    def is_color(self, frame):
        """候选圆 ROI 中指定颜色的像素面积是否达到阈值。"""
        if frame.size == 0:
            return False, 0

        color_area = int(cv2.countNonZero(self.make_circle_mask(frame)))
        return color_area > self.min_color_area, color_area

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
        if self.mode == "coarse":
            return self.process_material(frame)
        return self.process_circle(frame)

    def process_material(self, frame):
        """通过最大颜色轮廓粗定位物块；进入阈值范围后切换模式。"""
        result = frame.copy()
        mask = self.make_material_mask(frame)
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
                    center_x = int(moments["m10"] / moments["m00"])
                    center_y = int(moments["m01"] / moments["m00"])
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
            "center": center,
            "error_x": int(error_x),
            "error_y": int(error_y),
            "area": area,
        }

    def process_circle(self, frame):
        """使用当前 Hough + HSV 逻辑精定位色环圆心。"""
        result = frame.copy()
        mask = self.make_circle_mask(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gauss = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(
            gauss,
            cv2.HOUGH_GRADIENT_ALT,
            0.5,
            minDist=100,
            param1=100,
            param2=0.9,
            minRadius=20,
            maxRadius=180,
        )

        color_circle_targets = []
        detected = False
        center = None
        area = 0.0
        error_x = 0
        error_y = 0

        if circles is not None:
            circles = self.delete_superfluous_circles(circles)
            for roi, (circle_x, circle_y, radius) in self.circle_squares(
                frame,
                circles,
            ):
                color_matches, color_area = self.is_color(roi)
                if color_matches:
                    color_circle_targets.append(
                        (circle_x, circle_y, radius, color_area)
                    )

        if color_circle_targets:
            # 与源程序实际行为一致：选择 Y 值最大的同色圆。
            target_x, target_y, radius, area = max(
                color_circle_targets,
                key=lambda item: item[1],
            )

            if self.choose_color == CAMERA_COLOR_BLUE:
                target_x += self.blue_circle_delta_x
                target_y += self.blue_circle_delta_y

            center_x = int(round(target_x))
            center_y = int(round(target_y))
            center = (center_x, center_y)
            error_x = center_x - self.center_x
            error_y = self.center_y - center_y
            detected = True

            cv2.circle(result, center, 4, (0, 0, 255), -1)

        self.draw_crosshair(result)

        return result, mask, {
            "detected": detected,
            "state": "circle_detected" if detected else "circle_not_found",
            "mode": "circle",
            "next_mode": "circle",
            "color_id": self.choose_color,
            "color": COLOR_NAMES[self.choose_color],
            "center": center,
            "error_x": int(error_x),
            "error_y": int(error_y),
            "area": float(area),
        }


# 保留旧类名，避免其他代码若曾导入 MaterialDetector 时失效。
MaterialDetector = ColorCircleDetector


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
    lines = [
        f"Target: {detection['color']}  State: {detection['state']}",
        f"Mode: {mode_text}",
        f"Center: {center_text}",
        f"Error X: {detection['error_x']}",
        f"Error Y: {detection['error_y']}",
        f"Area: {detection['area']:.0f}",
        f"FPS: {fps:.1f}",
    ]

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
        description="HSV coarse positioning + Hough color-circle detector"
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=120.0)
    parser.add_argument("--center-x", type=int, default=334)
    parser.add_argument("--center-y", type=int, default=338)
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

    detector = ColorCircleDetector(
        center_x=center_x,
        center_y=center_y,
        min_contour_area=args.coarse_min_area,
        min_color_area=args.min_area,
        switch_tolerance=args.switch_tolerance,
    )
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
            "等待USB CDC颜色命令：FF B2 Color FF；"
            "收到命令后先粗定位，再进行色环精定位"
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
            active_color, command_version = usb.get_command()

            if command_version != last_command_version:
                detector.set_color(active_color)
                last_command_version = command_version
                next_send_time = 0.0

            if active_color is None:
                # 上电等待阶段：只显示原图、准星和状态，不执行识别。
                result = detector.make_waiting_frame(frame)
                detection = {
                    "detected": False,
                    "state": "waiting_usb",
                    "mode": "waiting",
                    "next_mode": "waiting",
                    "color_id": None,
                    "color": "none",
                    "center": None,
                    "error_x": 0,
                    "error_y": 0,
                    "area": 0.0,
                }
            else:
                result, _, detection = detector.process(frame)

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

            # 收到首个颜色命令后持续发送；未检测到时发送 Valid=0 和零坐标。
            if active_color is not None and completed_time >= next_send_time:
                usb.send_detection(active_color, detection)
                next_send_time = completed_time + send_interval

            if completed_time - last_print_time >= args.print_interval:
                if active_color is None:
                    print("等待USB CDC颜色命令，当前未执行识别")
                elif detection["detected"]:
                    print(
                        f"[{detection['mode']}] {detection['color']} "
                        f"中心={detection['center']} "
                        f"误差=({detection['error_x']}, "
                        f"{detection['error_y']}) "
                        f"面积={detection['area']:.0f} "
                        f"FPS={display_fps:.1f}"
                    )
                else:
                    print(
                        f"[{detection['mode']}] "
                        f"未检测到 {detection['color']}  "
                        f"FPS={display_fps:.1f}"
                    )
                last_print_time = completed_time

            cv2.imshow("Coarse + circle detection", result)

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
