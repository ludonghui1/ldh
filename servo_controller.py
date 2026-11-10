"""通用总线舵机控制器。Python 3.10+，依赖 pyserial。"""
import csv
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
from serial.tools import list_ports


from servo_protocol import packet, word
from serial_transport import Bus
from servo_device import ServoDevice

class App:
    def __init__(self, root):
        self.root, self.bus = root, None
        self.events = queue.Queue()
        self.tasks = queue.Queue()
        self.halt = threading.Event()
        self.deadline = None
        self.swing_active = threading.Event()
        self.samples = []
        self.telemetry_stats = None
        self.next_telemetry = 0
        self.auto_feedback = False
        self.vars = {}
        root.title('总线舵机关节模式测试工具')
        root.geometry('1100x880')
        self.status = tk.StringVar(value='未连接')
        self.position = tk.StringVar(value='位置：未读取')
        self.progress = tk.StringVar(value='未运行')
        self.configure_theme()
        self.viewport = tk.Canvas(root, bg='white', highlightthickness=0)
        scroll = ttk.Scrollbar(root, orient='vertical', command=self.viewport.yview)
        scroll.pack(side='right', fill='y')
        self.viewport.pack(fill='both', expand=True)
        self.viewport.configure(yscrollcommand=scroll.set)
        self.content = ttk.Frame(self.viewport)
        self.content_window = self.viewport.create_window(0, 0, anchor='nw', window=self.content)
        self.content.bind('<Configure>', lambda e: self.viewport.configure(scrollregion=self.viewport.bbox('all')))
        self.viewport.bind('<Configure>', self.resize_layout)
        root.bind('<MouseWheel>', lambda e: self.viewport.yview_scroll(int(-e.delta / 120), 'units'))
        self.reflow_groups = []
        self.metric_frames = []
        self.wrap_labels = []
        self.narrow = None
        self.connection_page = ttk.Frame(self.content)
        self.connection_page.pack(fill='both', expand=True)
        self.control_page = ttk.Frame(self.content)
        ttk.Label(self.connection_page, text='连接舵机', style='Title.TLabel').pack(anchor='w', padx=24, pady=(24, 8))
        ttk.Label(self.control_page, text='关节模式测试', style='Title.TLabel').pack(anchor='w', padx=24, pady=(24, 8))
        top = ttk.LabelFrame(self.connection_page, text='串口连接（先关闭 SSCOM 的串口）', padding=10)
        top.pack(fill='x', padx=10, pady=6)
        self.port = ttk.Combobox(top, width=15)
        self.port.grid(row=0, column=0)
        ttk.Button(top, text='刷新串口', command=self.refresh).grid(row=0, column=1)
        self.field(top, '波特率', 'baud', '1000000', 0, 2)
        self.field(top, 'ID', 'sid', '1', 0, 4)
        ttk.Button(top, text='连接', command=self.connect).grid(row=0, column=6, padx=6)
        ttk.Button(top, text='断开', command=self.disconnect).grid(row=0, column=7)
        ttk.Label(top, textvariable=self.status).grid(row=1, column=0, columnspan=8, sticky='w')
        self.slow = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='兼容发送：每字节间隔1ms（仅断开后更改再连接）', variable=self.slow).grid(row=2, column=0, columnspan=8, sticky='w')
        control = ttk.Frame(self.control_page, padding=10)
        control.pack(fill='x')
        ttk.Button(control, text='返回连接页', command=lambda: self.disconnect(return_page=True)).pack(side='right', padx=3)
        ttk.Button(control, text='断开连接', command=lambda: self.disconnect(return_page=False)).pack(side='right', padx=3)
        for text, func in [('读取模式', self.read_mode), ('读取当前位置', self.read_test_position), ('开启扭矩', self.torque_on)]:
            ttk.Button(control, text=text, command=lambda f=func: self.submit(f)).pack(side='left', padx=3)
        tk.Button(control, text='停止并卸载扭矩', bg='#B42318', activebackground='#922018', fg='white', activeforeground='white', relief='flat', bd=0, padx=12, pady=8, font=(self.ui_font, -15), command=self.stop).pack(side='left', padx=3)
        ttk.Label(self.control_page, textvariable=self.position).pack(anchor='w', padx=20)
        monitor = ttk.LabelFrame(self.control_page, text='实时状态（连接后自动刷新）', padding=12)
        monitor.pack(fill='x', padx=10, pady=6)
        self.telemetry = tk.StringVar(value='当前速度原始值：—')
        self.telemetry_summary = tk.StringVar(value='等待采样')
        self.cards = {}
        for column, (key, title, color) in enumerate([
            ('voltage', '电源电压 / V', '#165DFF'),
            ('temperature', '温度 / ℃', '#165DFF'),
            ('actual', '实际位置 / 原始值', '#165DFF'),
            ('load', '当前负载 / 原始值', '#165DFF'),
        ]):
            monitor.columnconfigure(column, weight=1, uniform='card')
            card = tk.Frame(monitor, bg='#F5F7FA', highlightbackground='#F5F7FA', highlightthickness=0, padx=14, pady=10)
            card.grid(row=0, column=column, sticky='nsew', padx=5)
            self.metric_frames.append(card)
            tk.Label(card, text=title, bg='#F5F7FA', fg='#526174', font=(self.ui_font, -15)).pack(anchor='w')
            value = tk.StringVar(value='—')
            note = tk.StringVar(value='等待设备反馈')
            self.cards[key] = (value, note)
            tk.Label(card, textvariable=value, bg='#F5F7FA', fg=color, font=(self.ui_font, -30, 'bold')).pack(anchor='w', pady=(5, 2))
            tk.Label(card, textvariable=note, bg='#F5F7FA', fg='#718096', font=(self.ui_font, -13), wraplength=205, justify='left').pack(anchor='w')
        ttk.Label(monitor, textvariable=self.telemetry).grid(row=1, column=0, columnspan=4, sticky='w', padx=5, pady=(8, 0))
        ttk.Label(monitor, textvariable=self.telemetry_summary, wraplength=1000).grid(row=2, column=0, columnspan=4, sticky='w', padx=5)
        actions = ttk.Frame(monitor)
        actions.grid(row=3, column=0, columnspan=4, sticky='w', pady=(6,0))
        ttk.Button(actions, text='刷新反馈', command=lambda: self.submit(self.read_telemetry)).pack(side='left')
        ttk.Button(actions, text='导出记录', command=self.export_telemetry).pack(side='left', padx=6)
        self.operation_area = ttk.Frame(self.control_page)
        self.operation_area.pack(fill='both', expand=True, padx=10, pady=12)
        self.operation_area.columnconfigure(0, weight=1)
        self.operation_area.columnconfigure(1, weight=1)
        self.operation_area.rowconfigure(0, weight=1)
        self.operation_left = ttk.Frame(self.operation_area)
        self.operation_left.grid(row=0, column=0, sticky='nsew', padx=(0,12))
        self.operation_log = ttk.Frame(self.operation_area)
        self.operation_log.grid(row=0, column=1, sticky='nsew')
        ttk.Label(self.operation_log, text='收发日志', font=(self.ui_font, -15, 'bold')).pack(anchor='w', pady=(0,8))
        # 当前界面仅启用关节模式；车轮控制方法保留，未提供入口。
        joint = ttk.LabelFrame(self.operation_left, text='关节模式（绝对位置约 0–300°）', padding=12)
        joint.pack(fill='x', padx=10, pady=6)
        ttk.Button(joint, text='关节模式（切换）', command=lambda: self.switch('joint')).grid(row=0, column=0, columnspan=2)
        self.field(joint, '目标位置', 'target', '100', 1, 0)
        self.unit = ttk.Combobox(joint, values=['角度 °', '原始值 0–1023'], state='readonly', width=16)
        self.unit.set('角度 °')
        self.unit.grid(row=1, column=2)
        self.field(joint, '速度 1–1023', 'jspeed', '90', 1, 3)
        ttk.Button(joint, text='移动到目标位置', command=self.move).grid(row=2, column=0, columnspan=2, pady=6)
        ttk.Label(joint, text='关节速度单位约 0.111 rpm；不提供速度0，避免误设为最大速度。').grid(row=3, column=0, columnspan=6, sticky='w')
        swing = ttk.LabelFrame(self.operation_left, text='关节往返摆动', padding=8)
        swing.pack(fill='x', padx=10, pady=4)
        self.field(swing, '摆动速度1–1023', 'swing_speed', '90', 0, 0)
        self.field(swing, '往返次数', 'swing_count', '3', 0, 2)
        ttk.Button(swing, text='开始往返摆动', command=self.start_swing).grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(swing, text='停止摆动并卸载扭矩', command=self.stop).grid(row=1, column=2, columnspan=2)
        self.swing_progress = tk.StringVar(value='固定原始位置0↔1023；持续刷新反馈，每程约1秒后换向，0→1023→0计1次。')
        ttk.Label(swing, textvariable=self.swing_progress).grid(row=2, column=0, columnspan=6, sticky='w')
        ttk.Label(self.control_page, text='模式切换会修改 EEPROM 并断电保存；切换前卸载扭矩，关节切换时将目标对齐当前位置。').pack(anchor='w', padx=15)
        self.logbox = tk.Text(self.operation_log, width=35, wrap='word', height=20, font=('Consolas', -13), bg='#F5F7FA', fg='#475467', relief='flat', bd=0, padx=16, pady=12, state='disabled')
        log_scroll = ttk.Scrollbar(self.operation_log, orient='vertical', command=self.logbox.yview)
        log_scroll.pack(side='right', fill='y')
        self.logbox.configure(yscrollcommand=log_scroll.set)
        self.logbox.pack(fill='both', expand=True)
        for group in (top, joint, swing):
            self.reflow_groups.append([(w, dict(w.grid_info())) for w in group.winfo_children() if w.winfo_manager() == 'grid'])
        self.responsive_buttons = [control, actions]
        self.button_layouts = [(w, dict(w.pack_info())) for g in self.responsive_buttons for w in g.winfo_children() if w.winfo_manager() == 'pack']
        self.refresh()
        root.after(100, lambda: self.resize_layout())
        threading.Thread(target=self.worker, daemon=True).start()
        root.after(50, self.pump)
        root.protocol('WM_DELETE_WINDOW', self.close)

    def configure_theme(self):
        from tkinter import font
        families = set(font.families(self.root))
        self.ui_font = 'Inter' if 'Inter' in families else 'Microsoft YaHei UI'
        self.root.configure(bg='white')
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', font=(self.ui_font, -15), background='white', foreground='#1D2939')
        style.configure('TFrame', background='white')
        style.configure('TLabel', background='white', padding=0)
        style.configure('Title.TLabel', font=(self.ui_font, -24, 'bold'), foreground='#101828')
        style.configure('TLabelframe', background='white', borderwidth=0, relief='flat', padding=16)
        style.configure('TLabelframe.Label', font=(self.ui_font, -15, 'bold'), foreground='#101828', background='white')
        style.configure('TButton', background='#EEF3FF', foreground='#165DFF', borderwidth=0, padding=(14, 9), focusthickness=0)
        style.map('TButton', background=[('active', '#DFE9FF'), ('pressed', '#CDDDFF')])
        style.configure('TEntry', padding=8, fieldbackground='#F5F7FA', borderwidth=0)
        style.configure('TCombobox', padding=7, fieldbackground='#F5F7FA', borderwidth=0)
        style.configure('TCheckbutton', background='white', padding=(0, 8))

    def resize_layout(self, event=None):
        width = max(280, self.viewport.winfo_width())
        content_width = min(1200, width)
        self.viewport.itemconfigure(self.content_window, width=content_width)
        self.viewport.coords(self.content_window, max(0, (width-content_width)//2), 0)
        narrow = content_width < 760
        if hasattr(self, 'operation_left'):
            self.operation_left.grid_configure(row=0, column=0, columnspan=1 if content_width >= 1000 else 2)
            self.operation_log.grid_configure(row=0 if content_width >= 1000 else 1, column=1 if content_width >= 1000 else 0, columnspan=1 if content_width >= 1000 else 2)
        layout_key = (narrow, content_width >= 1000)
        if layout_key != self.narrow:
            self.narrow = layout_key
            for group in self.reflow_groups:
                compact = narrow or (content_width >= 1000 and group and group[0][0].master is not self.port.master)
                for index, (widget, original) in enumerate(group):
                    if compact:
                        widget.grid_configure(row=index, column=0, columnspan=1, sticky='ew', padx=4, pady=4)
                    else:
                        widget.grid_configure(**{k:v for k,v in original.items() if k != 'in'})
            for index, card in enumerate(self.metric_frames):
                card.grid_configure(row=index if narrow else 0, column=0 if narrow else index, columnspan=4 if narrow else 1, pady=5)
            for widget, original in self.button_layouts:
                if narrow:
                    widget.pack_configure(side='top', fill='x', padx=4, pady=4)
                else:
                    widget.pack_configure(**{k:v for k,v in original.items() if k != 'in'})
        def wrap(parent):
            for child in parent.winfo_children():
                if isinstance(child, ttk.Label):
                    child.configure(wraplength=max(200, content_width-80))
                if isinstance(child, (ttk.Frame, ttk.LabelFrame)):
                    wrap(child)
        wrap(self.content)
        self.viewport.configure(scrollregion=self.viewport.bbox('all'))

    def field(self, parent, label, key, value, row, col):
        ttk.Label(parent, text=label).grid(row=row, column=col, padx=5, pady=4)
        v = tk.StringVar(value=value)
        self.vars[key] = v
        ttk.Entry(parent, textvariable=v, width=11).grid(row=row, column=col+1, padx=5)

    def number(self, key, low, high, integer=False):
        v = float(self.vars[key].get())
        if not math.isfinite(v) or not low <= v <= high or (integer and v != int(v)):
            raise ValueError(f'{key} 必须在 {low}～{high} 范围内' + ('，且为整数' if integer else ''))
        return int(v) if integer else v

    def log(self, text):
        self.events.put(('log', time.strftime('%H:%M:%S') + ' ' + text))

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.port['values'] = ports
        if ports and not self.port.get():
            self.port.set(ports[0])

    def submit(self, function):
        if self.swing_active.is_set():
            messagebox.showinfo('摆动中', '请先停止摆动再执行其他操作')
            return
        if not self.tasks.empty():
            messagebox.showinfo('请稍候', '还有操作等待执行')
            return
        self.tasks.put(function)

    def connect(self):
        try:
            port = self.port.get()
            baud = self.number('baud', 1200, 3000000, True)
            sid = self.number('sid', 0, 253, True)
        except Exception as exc:
            messagebox.showerror('输入错误', str(exc)); return
        slow = self.slow.get()
        def job():
            if self.bus:
                raise RuntimeError('请先断开已有连接')
            self.bus = Bus(port, baud, sid, self.log)
            self.bus.byte_delay = 0.001 if slow else 0.0
            self.events.put(('status', f'串口已打开 {port} / {baud} / ID={sid}，8N1；等待验证'))
            self.bus.request(1)
            self.events.put(('status', f'Ping成功 {port} / {baud} / ID={sid}，8N1'))
            self.read_mode()
            self.auto_feedback = True
            self.events.put(('page', 'control'))
        self.submit(job)

    def disconnect(self, return_page=True):
        self.stop()
        def job():
            self.auto_feedback = False
            if self.bus:
                self.bus.io.close()
                self.bus = None
            self.events.put(('status', '未连接'))
            if return_page:
                self.events.put(('page', 'connection'))
        self.tasks.put(job)

    def require(self):
        if not self.bus:
            raise RuntimeError('请先连接串口')

    def read_mode(self):
        self.require()
        limits = self.bus.read(6, 4)
        lo, hi = int.from_bytes(limits[:2], 'little'), int.from_bytes(limits[2:], 'little')
        mode = 'wheel' if lo == hi == 0 else 'joint'
        self.log(f'模式：{"车轮" if mode == "wheel" else "关节"}，最小={lo}，最大={hi}')
        return mode, lo, hi

    def read_position(self):
        # 反馈原样显示，不以0–1023范围拦截。
        return self.read_test_position()

    def position_target(self):
        # 保留原始反馈；用作写入目标时单独限制到协议范围。
        raw = self.read_position()
        target = raw if 0 <= raw <= 1023 else None
        if target != raw:
            self.log(f'原始反馈{raw}保留显示；不将其自动复制为目标，请使用明确的目标位置启动运动')
        return target

    def read_telemetry(self):
        self.require()
        # 0x26速度(2字节)，0x28负载(2字节)，0x2A电压和0x2B温度。
        device = ServoDevice(self.bus)
        speed = device.read_speed()
        if self.halt.is_set():
            return
        load = device.read_load()
        if self.halt.is_set():
            return
        voltage_data = bytes([device.read_voltage_raw()])
        if self.halt.is_set():
            return
        env = voltage_data + bytes([device.read_temperature()])
        voltage, temperature = env[0] / 10, env[1]
        env_valid = env[0] != 0
        self.events.put(('cards', {
            'voltage': (f'{voltage:.1f}', '舵机反馈电压'),
            'temperature': (str(temperature), '舵机反馈温度'),
            'load': (str(load), f'0x{load:04X} · 未换算扭矩'),
        }))
        sample = (time.strftime('%Y-%m-%d %H:%M:%S'), temperature, voltage, speed, load)
        self.events.put(('sample', sample))
        stats = self.telemetry_stats
        if stats is None:
            stats = [temperature, voltage, voltage, speed, load, 0]
            self.telemetry_stats = stats
        stats[0] = max(stats[0], temperature)
        stats[1] = min(stats[1], voltage)
        stats[2] = max(stats[2], voltage)
        stats[3] = max(stats[3], speed)
        stats[4] = max(stats[4], load)
        stats[5] += 1
        self.events.put(('telemetry', f'温度 {temperature} ℃  |  电源 {voltage:.1f} V  |  当前速度原始值 {speed} (0x{speed:04X})  |  负载原始值 {load} (0x{load:04X})'))
        self.events.put(('telemetry_summary', f'采样{stats[5]}次：最高温度{stats[0]}℃，电压{stats[1]:.1f}–{stats[2]:.1f}V，速度原始最大值{stats[3]}，负载原始最大值{stats[4]}'))
        self.log(f'监测：{temperature}℃ / {voltage:.1f}V / 速度原始{speed} / 负载原始{load}')

    def export_telemetry(self):
        if not self.samples:
            messagebox.showinfo('无数据', '先读取监测数据或运行往返测试')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV文件', '*.csv')], initialfile='servo_test.csv')
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as stream:
                writer = csv.writer(stream)
                writer.writerow(['时间', '温度_C', '电源电压_V', '速度原始值', '负载原始值'])
                writer.writerows(list(self.samples))
            messagebox.showinfo('导出完成', path)
        except OSError as exc:
            messagebox.showerror('导出失败', str(exc))

    def torque_on(self):
        self.require()
        self.read_mode()
        # 只允许动作按钮有意启动运动；手动开扭矩前清除旧运动命令。
        ServoDevice(self.bus).disable_torque()
        if self.read_mode()[0] == 'joint':
            p = self.position_target()
            if p is None:
                return
            self.bus.write(0x1E, word(p) + word(90))
        else:
            self.bus.write(0x20, [0, 0])
        ServoDevice(self.bus).enable_torque()

    def switch(self, mode):
        if not messagebox.askokcancel('模式切换', '将卸载扭矩并修改 EEPROM 角度限制。请确保机械负载有支撑。继续？'):
            return
        self.stop()
        def job():
            self.require()
            current_mode, _, _ = self.read_mode()
            if current_mode == mode:
                self.read_test_position()
                self.log('当前已经是所选模式，无需重复切换；扭矩保持关闭。')
                return
            ServoDevice(self.bus).disable_torque()
            self.bus.write(0x20, [0, 0])
            if mode == 'joint':
                initial = self.position_target()
                if initial is not None:
                    self.bus.write(0x1E, word(initial) + word(90))
            self.bus.write(6, [0, 0, 0, 0] if mode == 'wheel' else [0, 0, 255, 3])
            if self.read_mode()[0] != mode:
                raise RuntimeError('模式读回不符')
            self.log('模式切换已确认；扭矩保持关闭，点击动作按钮才开始运动。')
        self.tasks.put(job)

    def move(self):
        try:
            target = self.number('target', 0, 300 if self.unit.get() == '角度 °' else 1023)
            if self.unit.get() == '角度 °':
                target = round(target * 1023 / 300)
            elif target != int(target):
                raise ValueError('原始位置必须是整数')
            target = int(target)
            speed = self.number('jspeed', 1, 1023, True)
        except Exception as exc:
            messagebox.showerror('输入错误', str(exc)); return
        def job():
            self.require()
            mode, lo, hi = self.read_mode()
            if mode != 'joint' or not lo <= target <= hi:
                raise RuntimeError('需要关节模式，且目标在角度限制内')
            ServoDevice(self.bus).disable_torque()
            ServoDevice(self.bus).set_target(target, speed)
            ServoDevice(self.bus).enable_torque()
            self.log('位置和速度已读回确认；应答不表示已到位，请读取当前位置。')
        self.submit(job)

    def start_swing(self):
        try:
            count = self.number('swing_count', 1, 100000, True)
            speed = self.number('swing_speed', 1, 1023, True)
        except Exception as exc:
            messagebox.showerror('输入错误', str(exc))
            return
        self.submit(lambda: self.run_swing(0, 1023, count, speed, 0, 1))

    def read_test_position(self):
        self.require()
        data = ServoDevice(self.bus).read_position_bytes()
        value = int.from_bytes(data, 'little')
        text = f'实际位置：低=0x{data[0]:02X} 高=0x{data[1]:02X} 原始值={value}'
        if value <= 1023:
            text += f'，约{value*300/1023:.2f}°'
        self.events.put(('position', text))
        note = f'约 {value*300/1023:.2f}°' if value <= 1023 else '原始反馈，未换算角度'
        self.events.put(('cards', {'actual': (str(value), note)}))
        self.log(text)
        return value

    def run_swing(self, a, b, count, speed, tolerance, timeout):
        self.swing_active.set()
        try:
            self.require()
            mode, lo, hi = self.read_mode()
            if mode != 'joint' or not (lo <= a <= hi and lo <= b <= hi):
                raise RuntimeError('需要关节模式，角度限制必须允许0–1023；请先切换关节模式')
            self.telemetry_stats = None
            self.events.put(('reset_samples', ''))
            if self.halt.is_set():
                return
            ServoDevice(self.bus).disable_torque()
            def command_and_sample(target, label, enable=False):
                if self.halt.is_set():
                    return False
                self.events.put(('swing_progress', label + '，实时读取反馈'))
                ServoDevice(self.bus).set_target(target, speed)
                if self.halt.is_set():
                    return False
                if enable:
                    ServoDevice(self.bus).enable_torque()
                until = time.monotonic() + 1.0
                while not self.halt.is_set():
                    actual = self.read_test_position()
                    self.events.put(('swing_progress', f'{label}：目标={target}，实际={actual}'))
                    if self.halt.is_set():
                        return False
                    self.read_telemetry()
                    if time.monotonic() >= until:
                        break
                    if self.halt.wait(0.02):
                        return False
                return not self.halt.is_set()
            if not command_and_sample(a, '初始定位0（不计次）', True):
                return
            for i in range(count):
                if not command_and_sample(b, f'第{i+1}/{count}次 0→1023'):
                    return
                if not command_and_sample(a, f'第{i+1}/{count}次 1023→0'):
                    return
                self.log(f'完成指令往返 {i+1}/{count} 次；不代表实际全程到位')
            self.events.put(('swing_progress', f'已完成{count}次指令往返，最后目标0，保持扭矩；实际位置见上方'))
        finally:
            self.swing_active.clear()
            if self.halt.is_set():
                self.events.put(('swing_progress', '测试已取消，正在停止并卸载扭矩'))

    def start_wheel(self, counted):
        try:
            speed = self.number('wspeed', 1, 1023, True)
            maximum = self.number('maxsec', 0.1, 3600)
            duration = maximum
            if counted:
                if not self.calibrated.get():
                    raise ValueError('请先在当前输出、方向和负载下实测 rpm，并勾选校准确认')
                turns = self.number('turns', 0.001, 10000)
                rpm = self.number('rpm', 0.01, 10000)
                duration = turns * 60 / rpm
                if duration > maximum:
                    raise ValueError(f'估算需要 {duration:.2f} 秒，超过最长运行时间')
            raw = speed + (1024 if self.direction.get().startswith('CW ') else 0)
        except Exception as exc:
            messagebox.showerror('输入错误', str(exc)); return
        def job():
            self.require()
            if self.read_mode()[0] != 'wheel':
                raise RuntimeError('请先切换车轮模式')
            ServoDevice(self.bus).disable_torque()
            self.bus.write(0x20, word(raw))
            self.deadline = time.monotonic() + duration  # 包括使能确认耗时
            ServoDevice(self.bus).enable_torque()
            self.log(f'车轮运行，输出值={raw}，最迟 {duration:.2f} 秒后卸载扭矩停止')
        self.submit(job)

    def stop(self):
        self.halt.set()
        while True:
            try:
                self.tasks.get_nowait()
            except queue.Empty:
                break

    def worker(self):
        while True:
            try:
                if self.halt.is_set() or (self.deadline is not None and time.monotonic() >= self.deadline):
                    self.halt.clear()
                    self.deadline = None
                    if self.bus:
                        self.bus.emergency()
                    self.events.put(('progress', '已发送停止并卸载扭矩'))
                job = self.tasks.get(timeout=0.03)
            except queue.Empty:
                if self.auto_feedback and self.bus and time.monotonic() >= self.next_telemetry:
                    try:
                        self.read_test_position()
                        if not self.halt.is_set():
                            self.read_telemetry()
                        self.next_telemetry = time.monotonic() + 0.1
                    except Exception as exc:
                        self.log('自动反馈读取失败：' + str(exc))
                        self.events.put(('telemetry_summary', '反馈读取失败，1秒后重试；数据保留为上次读数'))
                        self.next_telemetry = time.monotonic() + 1.0
                continue
            try:
                job()
            except Exception as exc:
                self.deadline = None
                self.log('错误：' + str(exc))
                if self.bus:
                    try:
                        self.bus.emergency()
                    except Exception as stop_exc:
                        self.log('停止发送失败：' + str(stop_exc))
                if not self.auto_feedback and self.bus:
                    try:
                        self.bus.io.close()
                    except Exception:
                        pass
                    self.bus = None
                    self.events.put(('status', '连接验证失败，请检查设置后重试'))
                self.events.put(('error', str(exc)))

    def pump(self):
        while True:
            try:
                kind, text = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'page':
                if text == 'control':
                    self.connection_page.pack_forget()
                    self.control_page.pack(fill='both', expand=True)
                else:
                    self.control_page.pack_forget()
                    self.connection_page.pack(fill='both', expand=True)
            elif kind == 'cards':
                for key, (value, note) in text.items():
                    self.cards[key][0].set(value)
                    self.cards[key][1].set(note)
            elif kind == 'reset_samples':
                self.samples.clear()
            elif kind == 'sample':
                self.samples.append(text)
            elif kind == 'log':
                self.logbox.configure(state='normal')
                self.logbox.insert('end', text + '\n')
                if int(self.logbox.index('end-1c').split('.')[0]) > 2000:
                    self.logbox.delete('1.0', '500.0')
                self.logbox.see('end')
                self.logbox.configure(state='disabled')
            elif kind == 'error':
                messagebox.showerror('操作失败', text)
            elif kind == 'closed':
                self.root.destroy()
                return
            else:
                getattr(self, kind).set(text)
        if self.deadline is not None:
            self.progress.set(f'估算剩余 {max(0, self.deadline-time.monotonic()):.1f} 秒（不保证精确圈数）')
        self.root.after(50, self.pump)

    def close(self):
        self.stop()
        # 由后台线程发完停止帧后关闭窗口，避免在主线程争用串口。
        def job():
            self.auto_feedback = False
            if self.bus:
                self.bus.io.close()
                self.bus = None
            self.events.put(('closed', ''))
        self.tasks.put(job)
        # 等待后台停止与关闭串口完成。


if __name__ == '__main__':
    App(tk.Tk()).root.mainloop()
