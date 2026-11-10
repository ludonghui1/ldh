import time
import serial
from servo_protocol import packet, word

class Bus:
    def __init__(self, port, baud, sid, log):
        self.io = serial.Serial(port, baud, timeout=0.02, write_timeout=0.3)
        self.sid, self.log = sid, log
        self.byte_delay = 0.0
        self.last_end = time.monotonic()
        time.sleep(0.15)

    def request(self, instruction, params=(), expected=0):
        tx = packet(self.sid, instruction, params)
        time.sleep(max(0, 0.05 - (time.monotonic() - self.last_end)))
        self.io.reset_input_buffer()
        self.log('发 → ' + tx.hex(' ').upper())
        if self.byte_delay:
            for byte in tx:
                self.io.write(bytes([byte]))
                time.sleep(self.byte_delay)
        else:
            self.io.write(tx)
        self.io.flush()
        self.last_end = time.monotonic()
        buf = bytearray()
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            buf.extend(self.io.read(max(1, self.io.in_waiting)))
            while True:
                start = buf.find(b'\xff\xff')
                if start < 0:
                    buf[:] = buf[-1:] if buf[-1:] == b'\xff' else b''
                    break
                del buf[:start]
                if len(buf) < 4:
                    break
                length = buf[3]
                if length < 2 or length > 64:
                    del buf[0]
                    continue
                size = length + 4
                if len(buf) < size:
                    break
                rx = bytes(buf[:size])
                del buf[:size]
                self.log('收 ← ' + rx.hex(' ').upper())
                if rx == tx:  # 部分转换器有本地回显
                    continue
                if sum(rx[2:]) & 255 != 255:
                    self.log('丢弃：校验错误')
                    continue
                if rx[2] != self.sid:
                    continue
                self.last_end = time.monotonic()
                if rx[4]:
                    names = ['电压', '角度', '温度', '范围', '校验', '堵转', '指令', '保留位']
                    errors = '、'.join(n for i, n in enumerate(names) if rx[4] & (1 << i))
                    raise RuntimeError(f'舵机状态 0x{rx[4]:02X}：{errors}')
                if length != expected + 2:
                    continue
                return rx[5:-1]
        raise TimeoutError('未收到完整有效应答；检查供电、ID、波特率、接线和返回级别。')

    def read(self, address, count):
        # 此设备实测支持2字节读取，4字节读取返回0x10；统一分段读取。
        result = bytearray()
        for offset in range(0, count, 2):
            size = min(2, count - offset)
            result.extend(self.request(2, [address + offset, size], size))
        return bytes(result)

    def write(self, address, values):
        # 写指令可能不返回应答，随后必须读回校验。
        try:
            self.request(3, [address, *values])
        except TimeoutError:
            self.log('写入无应答，尝试读回确认')
        if self.read(address, len(values)) != bytes(values):
            raise RuntimeError(f'地址 0x{address:02X} 写入读回不一致')

    def emergency(self):
        # 无需依赖应答，优先发出卸载扭矩，再将速度归零。
        for address, values in [(0x18, [0]), (0x20, [0, 0])]:
            tx = packet(self.sid, 3, [address, *values])
            self.io.write(tx)
            self.io.flush()
            self.log('停止发 → ' + tx.hex(' ').upper())
            time.sleep(0.08)
        self.last_end = time.monotonic()
        self.log('已发送停止和卸载扭矩；未确认实际停止，必要时断开舵机电源。')


