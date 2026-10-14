"""Unpack console radar diagnosis into USART2 bytes for native parser replay.

This tool checks transport CRC/identity and slices the recorded circle. It does
not implement another lidar parser or change the map coordinates.
"""
import argparse
import json
from pathlib import Path
import zlib


def unpack_capture(document):
    """Return the original RX window, optional full circle, and its metadata."""
    snapshots = document.get('snapshots', {})
    raw = snapshots.get('raw')
    if not isinstance(raw, dict) or not raw.get('download_complete'):
        raise ValueError('原始数据未完整下载，请使用诊断导出文件')
    try:
        data = bytes.fromhex(raw['hex'])
        expected_crc = int(raw['crc32'], 16)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('原始字节或CRC字段无效') from error
    if len(data) != raw.get('total'):
        raise ValueError('原始数据下载字节数不一致')
    if zlib.crc32(data) != expected_crc:
        raise ValueError('原始数据CRC不一致')
    circle = None
    cloud = snapshots.get('cloud')
    capture_complete = raw.get('complete') is True
    if capture_complete:
        window = raw.get('window')
        bounds = raw.get('circle')
        if (not isinstance(window, int) or not isinstance(bounds, list) or len(bounds) != 2
                or not all(isinstance(v, int) for v in bounds)
                or not window <= bounds[0] < bounds[1] <= window + len(data)):
            raise ValueError('完整圈字节范围没有被下载窗口覆盖')
        circle = data[bounds[0] - window:bounds[1] - window]
        if cloud is not None:
            if not cloud.get('complete') or any(
                    cloud.get(key) != raw.get(key) for key in ('session', 'origin', 'scan', 'map')):
                raise ValueError('原始字节与点云不属于同圈')
        # A completed automatic scan can have map=0 when start is unresolved.
        # Its bytes are still useful; do not substitute a historical map.
    metadata = {key: value for key, value in raw.items() if key != 'hex'}
    metadata.update(capture_complete=capture_complete,
                    point_count=len(cloud.get('points', [])) if cloud else 0,
                    circle_bytes=len(circle) if circle is not None else 0)
    return data, circle, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path, help='诊断导出JSON')
    parser.add_argument('--output', type=Path, help='保存二进制与元数据的目录')
    args = parser.parse_args()
    document = json.loads(args.snapshot.read_text(encoding='utf-8-sig'))
    data, circle, metadata = unpack_capture(document)
    output = args.output or Path('.embeddedskills/radar-replay') / args.snapshot.stem
    output.mkdir(parents=True, exist_ok=True)
    (output / 'raw.bin').write_bytes(data)
    if circle is not None:
        (output / 'circle.bin').write_bytes(circle)
    (output / 'capture.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8')
    cloud = document.get('snapshots', {}).get('cloud')
    if cloud and metadata['capture_complete']:
        (output / 'cloud.json').write_text(json.dumps(cloud, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(dict(output=str(output.resolve()), bytes=len(data),
                         circle_bytes=metadata['circle_bytes'], complete=metadata['capture_complete'],
                         point_count=metadata['point_count']), ensure_ascii=False))


if __name__ == '__main__':
    main()
