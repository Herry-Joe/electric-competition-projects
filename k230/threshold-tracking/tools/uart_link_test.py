#!/usr/bin/env python3
"""uart_link_test.py — PC 端串口链路自检 (配合 K230 threshold_lvgl.py 的 TEST 按钮)

用途: 验证 K230(USART2) -> STM32 -> PC 串口回传通道是否通畅。
  按下 K230 屏上的 TEST 按钮, 本脚本会收到一帧自检帧并解码打印。

用法:
    pip install pyserial
    python uart_link_test.py            # 自动列出可用串口
    python uart_link_test.py COMx 115200
    python uart_link_test.py /dev/ttyUSB0 115200

帧格式: 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC] 0x01 0x01  (10 字节)
"""
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial，请先: pip install pyserial")
    sys.exit(1)

FRAME = bytes([0x3C, 0x3B])
TAIL = bytes([0x01, 0x01])


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc & 0xFF


def list_ports():
    try:
        from serial.tools.list_ports import comports
        ports = list(comports())
    except Exception:
        ports = []
    if not ports:
        print("未发现可用串口")
        return
    print("可用串口:")
    for p in ports:
        print("  -", p.device, ":", p.description)


def main():
    if len(sys.argv) < 2:
        list_ports()
        print("\n用法: python uart_link_test.py <COM端口> [波特率, 默认115200]")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:
        print("打开串口失败:", e)
        return

    print(f"监听 {port} @ {baud} ... (按 K230 的 TEST 按钮发帧, Ctrl+C 退出)")
    buf = b""
    while True:
        try:
            chunk = ser.read(64)
        except KeyboardInterrupt:
            print("\n退出")
            return
        if not chunk:
            continue
        buf += chunk
        # 在缓冲中按帧头切分
        while True:
            i = buf.find(FRAME)
            if i < 0 or len(buf) - i < 10:
                break
            frame = buf[i:i + 10]
            buf = buf[i + 10:]
            if frame[-2:] != TAIL:
                continue
            x = (frame[2] << 8) | frame[3]
            y = (frame[4] << 8) | frame[5]
            flag = frame[6]
            crc = frame[7]
            calc = crc8(bytes([frame[2], frame[3], frame[4], frame[5], frame[6]]))
            ok = (crc == calc)
            parts = []
            if flag & 1:
                parts.append("valid")
            if flag & 2:
                parts.append("hit")
            if flag & 4:
                parts.append("shape")
            if flag & 8:
                parts.append("TEST")
            tag = "TEST" if (flag & 8) else "DATA"
            crc_str = "OK" if ok else ("BAD got=%s want=%s" % (hex(crc), hex(calc)))
            print("[%s] x=%d y=%d flag=%s (%s) crc=%s" % (
                tag, x, y, format(flag, "#04x"), ",".join(parts) or "-", crc_str))
        time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出")
