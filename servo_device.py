"""无界面舵机功能：依赖注入传输对象，便于模拟测试。"""
from servo_protocol import word
class ServoDevice:
    def __init__(self, bus): self.bus = bus
    def read_position_bytes(self): return self.bus.read(0x24, 2)
    def read_position(self): return int.from_bytes(self.read_position_bytes(), 'little')
    def read_speed(self): return int.from_bytes(self.bus.read(0x26, 2), 'little')
    def read_load(self): return int.from_bytes(self.bus.read(0x28, 2), 'little')
    def read_voltage_raw(self): return self.bus.read(0x2A, 1)[0]
    def read_voltage(self): return self.read_voltage_raw()/10
    def read_temperature(self): return self.bus.read(0x2B, 1)[0]
    def enable_torque(self): self.bus.write(0x18, [1])
    def disable_torque(self): self.bus.write(0x18, [0])
    def set_joint_mode(self): self.bus.write(6, [0,0,255,3])
    def set_wheel_mode(self): self.bus.write(6, [0,0,0,0])
    def set_target(self, position, speed): self.bus.write(0x1E, word(position)+word(speed))
