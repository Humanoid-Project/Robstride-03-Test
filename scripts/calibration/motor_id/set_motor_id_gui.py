#!/usr/bin/env python3
import argparse
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import can

HOST_ID = 0xFD
DEVICE_ID_DESTINATION = 0xFE
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019

MIN_CURRENT_ID = 0
MIN_NEW_ID = 1
MAX_MOTOR_ID = 127
LISTEN_SECONDS = 8.0
SCAN_QUERY_TIMEOUT = 0.05

REPLY_TYPES = (0x00, 0x02, 0x15, 0x18)

def parse_scan_max(value):
    try:
        scan_max = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("scan-max는 숫자여야 합니다.") from exc
    if not MIN_CURRENT_ID <= scan_max <= MAX_MOTOR_ID:
        raise argparse.ArgumentTypeError(
            f"scan-max는 {MIN_CURRENT_ID}~{MAX_MOTOR_ID} 범위여야 합니다."
        )
    return scan_max

def build_arb_set_id(new_id, host_id, target_id):
    data16 = ((new_id & 0xFF) << 8) | (host_id & 0xFF)
    return (0x07 << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)

def build_arb_read(host_id, target_id):
    return (0x11 << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)

def build_arb_get_id(host_id, target_id):
    return (0x00 << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)

def build_arb_stop(host_id, target_id):
    return (0x04 << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)

def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination

def _await_reply(bus, host_id, target_id, comm_type, timeout, index=None):
    expected_destination = DEVICE_ID_DESTINATION if comm_type == 0x00 else host_id
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        reply_type, data16, destination = parse_arb(msg.arbitration_id)
        responder = data16 & 0xFF
        payload = bytes(msg.data)
        if (reply_type == comm_type and destination == expected_destination
                and responder == target_id and responder != host_id):
            if index is not None and (len(payload) < 8
                    or int.from_bytes(payload[0:2], "little") != index):
                continue
            return payload
    return None

def probe(bus, host_id, target_id, timeout=0.3):
    bus.send(can.Message(arbitration_id=build_arb_get_id(host_id, target_id),
                         data=bytes(8), is_extended_id=True))
    uid = _await_reply(bus, host_id, target_id, 0x00, timeout)
    if uid is not None:
        return {"motor_id": target_id, "uid": uid.hex(), "source": "device-id"}

    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    bus.send(can.Message(arbitration_id=build_arb_read(host_id, target_id),
                         data=bytes(data), is_extended_id=True))
    payload = _await_reply(
        bus, host_id, target_id, 0x11, timeout, index=MECH_POS_INDEX)
    if payload is None:
        return None
    return {"motor_id": target_id, "uid": None, "source": "mechPos"}

def ping(bus, host_id, target_id, timeout=0.3):
    return probe(bus, host_id, target_id, timeout) is not None

def send_stop(bus, host_id, target_id):
    bus.send(can.Message(arbitration_id=build_arb_stop(host_id, target_id),
                         data=bytes(8), is_extended_id=True))

def send_set_id(bus, host_id, current_id, new_id):
    bus.send(can.Message(arbitration_id=build_arb_set_id(new_id, host_id, current_id),
                         data=bytes(8), is_extended_id=True))
    time.sleep(0.2)

def drain(bus):
    while bus.recv(timeout=0.0) is not None:
        pass

def scan_ids(bus, host_id, max_id=MAX_MOTOR_ID):
    found = {}
    drain(bus)
    for target_id in range(MIN_CURRENT_ID, max_id + 1):
        result = probe(bus, host_id, target_id, SCAN_QUERY_TIMEOUT)
        if result is not None:
            found[target_id] = result["uid"]
        time.sleep(0.002)
    return found

def listen_for_ids(bus, host_id, seconds=8.0):
    found = {}
    drain(bus)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            continue
        if msg.is_extended_id:
            comm_type, data16, _destination = parse_arb(msg.arbitration_id)
            responder = data16 & 0xFF
            if comm_type in REPLY_TYPES and responder != host_id:
                if MIN_CURRENT_ID <= responder <= MAX_MOTOR_ID:
                    found.setdefault(responder, "private")
        elif len(msg.data) == 8:
            responder = msg.data[0]
            if MIN_CURRENT_ID <= responder <= MAX_MOTOR_ID:
                found.setdefault(responder, "MIT")
    return found

def link_state(channel):
    try:
        with open(f"/sys/class/net/{channel}/operstate") as handle:
            return handle.read().strip().lower()
    except OSError:
        return None

class Worker(threading.Thread):

    def __init__(self, channel, interface, host_id, scan_max, cmd_q, out_q):
        super().__init__(daemon=True)
        self.channel = channel
        self.interface = interface
        self.host_id = host_id
        self.scan_max = scan_max
        self.cmd_q = cmd_q
        self.out_q = out_q
        self.bus = None
        self._stop_event = threading.Event()

    def emit(self, kind, **kw):
        self.out_q.put((kind, kw))

    def down_hint(self):
        return (f"{self.channel} 인터페이스가 DOWN 입니다  ->  "
                f"sudo ip link set {self.channel} up type can bitrate 1000000")

    def close_bus(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def ensure_bus(self):
        if self.bus is not None:
            return True
        if link_state(self.channel) == "down":
            self.emit("bus", ok=False, text=self.down_hint())
            return False
        try:
            self.bus = can.Bus(channel=self.channel, interface=self.interface)
        except Exception as exc:
            self.emit("bus", ok=False, text=f"{self.channel} 열기 실패: {exc}")
            return False
        self.emit("bus", ok=True, text=f"{self.channel} 연결됨")
        return True

    def fail(self, exc):
        text = str(exc)
        if "Network is down" in text or "No such device" in text:
            self.close_bus()
            self.emit("bus", ok=False, text=self.down_hint())
        self.emit("done", ok=False, new=None, text=f"오류: {text}")

    def run(self):
        self.ensure_bus()

        while not self._stop_event.is_set():
            try:
                cmd = self.cmd_q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if not self.ensure_bus():
                    self.emit("done", ok=False, new=None, text=f"오류: {self.down_hint()}")
                    continue
                action = cmd[0]
                if action == "check":
                    _, target = cmd
                    ok = ping(self.bus, self.host_id, target)
                    self.emit("check", target=target, ok=ok)
                elif action == "scan":
                    self.do_scan()
                elif action == "listen":
                    self.do_listen()
                elif action == "change":
                    _, current, new = cmd
                    self.do_change(current, new)
            except Exception as exc:
                self.fail(exc)

    def report_found(self, found, empty_hint):
        for motor_id in sorted(found):
            detail = found[motor_id]
            suffix = f"  ({detail})" if detail else ""
            self.emit("log", text=f"  발견: ID {motor_id} (0x{motor_id:02X}){suffix}")
        if not found:
            self.emit("log", text=f"  {empty_hint}")
        self.emit("scan", ids=sorted(found))

    def do_scan(self):
        self.emit("log", text=f"ID {MIN_CURRENT_ID}~{self.scan_max} 순차 스캔 중...")
        found = scan_ids(self.bus, self.host_id, max_id=self.scan_max)
        self.report_found(found, "응답한 모터가 없습니다. '전원 켜서 찾기'를 써보세요.")

    def do_listen(self):
        self.emit("log", text=f"{LISTEN_SECONDS:.0f}초간 수신 대기 -- 지금 모터 전원을 인가하세요.")
        found = listen_for_ids(self.bus, self.host_id, seconds=LISTEN_SECONDS)
        self.report_found(found, "프레임이 하나도 안 왔습니다. 전원/배선/종단저항/비트레이트를 확인하세요.")

    def do_change(self, current, new):
        self.emit("log", text=f"현재 ID {current} (0x{current:02X}) 응답 확인 중...")
        current_result = probe(self.bus, self.host_id, current)
        if current_result is None:
            self.emit("done", ok=False, new=None,
                      text=f"현재 ID {current}가 응답하지 않습니다. 배선/전원/ID를 확인하세요.")
            return

        self.emit("log", text=f"새 ID {new} 충돌 검사 중...")
        if probe(self.bus, self.host_id, new) is not None:
            self.emit("done", ok=False, new=None,
                      text=f"새 ID {new}가 이미 버스에서 응답합니다(충돌). 다른 값을 쓰세요.")
            return

        self.emit("log", text=f"ID {current} 정지 후 {new}로 변경 명령 전송...")
        send_stop(self.bus, self.host_id, current)
        time.sleep(0.05)
        send_set_id(self.bus, self.host_id, current, new)

        self.emit("log", text=f"새 ID {new} 검증 중...")
        new_result = None
        for _ in range(10):
            new_result = probe(self.bus, self.host_id, new)
            if new_result is not None:
                break
            time.sleep(0.2)

        old_result = probe(self.bus, self.host_id, current)
        if new_result is not None and old_result is None:
            previous_uid = current_result["uid"]
            if previous_uid and new_result["uid"] and previous_uid != new_result["uid"]:
                self.emit("done", ok=False, new=None,
                          text="새 ID에서 다른 UID가 응답했습니다. ID 충돌 가능성이 있습니다.")
                return
            self.emit("done", ok=True, new=new,
                      text=f"성공: 모터가 ID {new} (0x{new:02X})로만 응답합니다.")
        elif new_result is None and old_result is not None:
            self.emit("done", ok=False, new=None,
                      text=f"변경 실패: 모터가 여전히 이전 ID {current}로 응답합니다. ID는 바뀌지 않았습니다.")
        elif new_result is not None and old_result is not None:
            self.emit("done", ok=False, new=None,
                      text="이전 ID와 새 ID가 모두 응답합니다. 중복 모터 또는 ID 충돌 상태입니다.")
        else:
            self.emit("done", ok=False, new=None,
                      text=f"ID {new}, {current} 둘 다 무응답입니다. 전원을 재인가한 뒤 '전원 켜서 찾기'로 확인하세요.")

    def shutdown(self):
        self._stop_event.set()
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass

class App:
    def __init__(self, root, args):
        self.root = root
        self.cmd_q = queue.Queue()
        self.out_q = queue.Queue()
        self.worker = Worker(args.channel, args.interface, args.host_id, args.scan_max,
                             self.cmd_q, self.out_q)
        self.busy = False

        root.title("Robstride Set Motor ID")
        root.geometry("460x470")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        frm = ttk.Frame(root, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        self.bus_var = tk.StringVar(value=f"버스 여는 중... ({args.channel})")
        ttk.Label(frm, textvariable=self.bus_var, foreground="#555").pack(anchor="w", pady=(0, 10))

        cur = ttk.Frame(frm)
        cur.pack(fill=tk.X, pady=3)
        ttk.Label(cur, text="현재 ID", width=8).pack(side=tk.LEFT)
        self.current_var = tk.StringVar()
        self.current_box = ttk.Combobox(cur, textvariable=self.current_var, width=6, values=[])
        self.current_box.pack(side=tk.LEFT)
        self.current_box.bind("<Return>", lambda _e: self.check_current())
        self.scan_btn = ttk.Button(cur, text="스캔", command=self.scan)
        self.scan_btn.pack(side=tk.LEFT, padx=(6, 0))
        self.check_btn = ttk.Button(cur, text="확인", command=self.check_current)
        self.check_btn.pack(side=tk.LEFT, padx=(4, 0))
        self.check_var = tk.StringVar(value="-")
        ttk.Label(cur, textvariable=self.check_var).pack(side=tk.LEFT, padx=(8, 0))

        listen_row = ttk.Frame(frm)
        listen_row.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(listen_row, text="", width=8).pack(side=tk.LEFT)
        self.listen_btn = ttk.Button(listen_row, text=f"전원 켜서 찾기 ({LISTEN_SECONDS:.0f}s)",
                                     command=self.listen)
        self.listen_btn.pack(side=tk.LEFT)
        ttk.Label(listen_row, text="스캔이 실패할 때", foreground="#888").pack(side=tk.LEFT, padx=(8, 0))

        new = ttk.Frame(frm)
        new.pack(fill=tk.X, pady=3)
        ttk.Label(new, text="새 ID", width=8).pack(side=tk.LEFT)
        self.new_var = tk.StringVar()
        ttk.Entry(new, textvariable=self.new_var, width=8).pack(side=tk.LEFT)
        ttk.Label(new, text=f"({MIN_NEW_ID}~{MAX_MOTOR_ID}, 예: 5 또는 0x05)",
                  foreground="#888").pack(side=tk.LEFT, padx=(8, 0))

        self.change_btn = ttk.Button(frm, text="변경", command=self.change_id)
        self.change_btn.pack(fill=tk.X, pady=(8, 10))

        ttk.Label(frm, text="로그").pack(anchor="w")
        self.log = tk.Text(frm, height=8, wrap="word", state="disabled")
        self.log.pack(fill=tk.BOTH, expand=True)

        self.worker.start()
        self.poll()

    def parse_id(self, raw, field, minimum):
        raw = raw.strip()
        if not raw:
            messagebox.showerror("입력 오류", f"{field}를 입력하세요.")
            return None
        try:
            value = int(raw, 0)
        except ValueError:
            messagebox.showerror("입력 오류", f"{field}는 숫자여야 합니다 (예: 5 또는 0x05).")
            return None
        if not minimum <= value <= MAX_MOTOR_ID:
            messagebox.showerror("입력 오류",
                                 f"{field}는 {minimum}~{MAX_MOTOR_ID} 범위여야 합니다.")
            return None
        return value

    def scan(self):
        if self.busy:
            return
        self.set_busy(True)
        self.check_var.set("-")
        self.cmd_q.put(("scan",))

    def listen(self):
        if self.busy:
            return
        self.set_busy(True)
        self.check_var.set("-")
        self.append_log(f"[전원 켜서 찾기] {LISTEN_SECONDS:.0f}초 대기 -- 지금 모터 전원을 인가하세요.")
        self.cmd_q.put(("listen",))

    def check_current(self):
        if self.busy:
            return
        current = self.parse_id(self.current_var.get(), "현재 ID", MIN_CURRENT_ID)
        if current is None:
            return
        self.check_var.set("확인 중...")
        self.cmd_q.put(("check", current))

    def change_id(self):
        if self.busy:
            return
        current = self.parse_id(self.current_var.get(), "현재 ID", MIN_CURRENT_ID)
        if current is None:
            return
        new = self.parse_id(self.new_var.get(), "새 ID", MIN_NEW_ID)
        if new is None:
            return
        if current == new:
            messagebox.showinfo("변경", "현재 ID와 새 ID가 같습니다.")
            return
        if not messagebox.askyesno(
            "영구 변경 확인",
            f"모터 ID {current} (0x{current:02X}) -> {new} (0x{new:02X})\n"
            "영구적으로 변경합니다.\n\n"
            "같은 현재 ID의 모터가 여러 대 연결되어 있으면 모두 변경됩니다.\n"
            "대상 모터 한 대만 연결했는지 확인한 뒤 진행하세요.",
        ):
            return
        self.set_busy(True)
        self.append_log(f"[변경 시작] {current} -> {new}")
        self.cmd_q.put(("change", current, new))

    def set_busy(self, busy):
        self.busy = busy
        state = "disabled" if busy else "normal"
        for widget in (self.change_btn, self.scan_btn, self.check_btn, self.listen_btn):
            widget.configure(state=state)

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("[%H:%M:%S] ") + text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def poll(self):
        while True:
            try:
                kind, kw = self.out_q.get_nowait()
            except queue.Empty:
                break
            if kind == "bus":
                self.bus_var.set(kw["text"])
            elif kind == "check":
                self.check_var.set("응답함 ✅" if kw["ok"] else "무응답 ❌")
            elif kind == "scan":
                ids = kw["ids"]
                self.current_box.configure(values=[str(i) for i in ids])
                if len(ids) == 1:
                    self.current_var.set(str(ids[0]))
                    self.check_var.set("응답함 ✅")
                self.set_busy(False)
            elif kind == "log":
                self.append_log(kw["text"])
            elif kind == "error":
                self.append_log("오류: " + kw["text"])
            elif kind == "done":
                self.append_log(kw["text"])
                if kw["ok"] and kw.get("new") is not None:
                    self.current_var.set(str(kw["new"]))
                    self.new_var.set("")
                    self.check_var.set("응답함 ✅")
                elif self.check_var.get() == "확인 중...":
                    self.check_var.set("-")
                self.set_busy(False)
        self.root.after(80, self.poll)

    def on_close(self):
        self.worker.shutdown()
        self.worker.join(timeout=0.5)
        self.root.destroy()

def parse_args():
    parser = argparse.ArgumentParser(description="Change a Robstride motor's CAN ID from a small GUI.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can interface, default: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="host CAN ID used in the private protocol, default: 0xFD")
    parser.add_argument("--scan-max", type=parse_scan_max, default=127,
                        help="highest motor ID probed by Scan, default: %(default)s")
    return parser.parse_args()

def main():
    args = parse_args()
    root = tk.Tk()
    App(root, args)
    root.mainloop()

if __name__ == "__main__":
    main()
