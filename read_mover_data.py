import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from datetime import datetime
import pyads


class ADSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Beckhoff ADS Mover Control")
        self.root.geometry("1450x800")

        self.plc = None
        self.polling = False
        self.poll_thread = None

        # Update these defaults to match your setup
        self.ams_net_id = tk.StringVar(value="169.254.137.138.1.1")
        self.ams_port = tk.StringVar(value="852")
        self.local_ip = tk.StringVar(value="172.24.68.147")
        self.poll_ms = tk.StringVar(value="300")

        # PLC control symbols
        self.symbols = {
            "ResetPressed": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.ResetPressed",
            "StartPressed": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.StartPressed",
            "StopPressed": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.StopPressed",
            "ResetPermissive": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.ResetPermissive",
            "StartPermissive": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.StartPermissive",
            "StopPermissive": "MAIN.ControlSourceHMI.MainPMLControl_Simplified.StopPermissive",
            "InitComplete": "MAIN.ControlSourceHMI._InitComplete",
            "Busy": "MAIN.ControlSourceHMI._Busy",
            "Error": "MAIN.ControlSourceHMI._Error",
            "ErrorID": "MAIN.ControlSourceHMI._ErrorID",
            "bButton": "MAIN.bButton",
            "bReset": "MAIN.bReset",
            "bStart": "MAIN.bStart",
            "bStop": "MAIN.bStop",
            "bTriggerButton": "MAIN.bTriggerButton",
        }

        # Read-only variables
        self.read_only = {
            "InitComplete",
            "Busy",
            "Error",
            "ErrorID",
            "bButton",
            "bReset",
            "bStart",
            "bStop",
            "bTriggerButton",
        }

        self.symbol_types = {
            "ErrorID": pyads.PLCTYPE_UDINT,
        }

        self.max_display_movers = 0
        self.build_ui()

    def build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="AMS Net ID:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.ams_net_id, width=22).grid(row=0, column=1, padx=5)

        ttk.Label(top, text="Port:").grid(row=0, column=2, sticky="w")
        ttk.Entry(top, textvariable=self.ams_port, width=8).grid(row=0, column=3, padx=5)

        ttk.Label(top, text="Local IP:").grid(row=0, column=4, sticky="w")
        ttk.Entry(top, textvariable=self.local_ip, width=16).grid(row=0, column=5, padx=5)

        ttk.Label(top, text="Poll ms:").grid(row=0, column=6, sticky="w")
        ttk.Entry(top, textvariable=self.poll_ms, width=8).grid(row=0, column=7, padx=5)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=8, padx=5)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=9, padx=5)

        self.start_poll_btn = ttk.Button(top, text="Start Polling", command=self.start_polling, state="disabled")
        self.start_poll_btn.grid(row=0, column=10, padx=5)

        self.stop_poll_btn = ttk.Button(top, text="Stop Polling", command=self.stop_polling, state="disabled")
        self.stop_poll_btn.grid(row=0, column=11, padx=5)

        self.status_label = ttk.Label(top, text="Disconnected")
        self.status_label.grid(row=0, column=12, padx=10, sticky="w")

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill="both", expand=True)

        # LEFT: Write TRUE / FALSE buttons per variable
        left = ttk.LabelFrame(body, text="Manual Control (Write TRUE / FALSE)", padding=10)
        left.pack(side="left", fill="y")

        canvas = tk.Canvas(left)
        scrollbar = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set, height=620)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for name, symbol in self.symbols.items():
            frame = ttk.Frame(scroll_frame)
            frame.pack(fill="x", pady=2)

            ttk.Label(frame, text=name, width=18).pack(side="left")

            if name in self.read_only:
                ttk.Label(frame, text="READ ONLY", foreground="gray").pack(side="left", padx=5)
            else:
                ttk.Button(
                    frame,
                    text="TRUE",
                    command=lambda s=symbol, n=name: self.write_value(s, True, n)
                ).pack(side="left", padx=2)

                ttk.Button(
                    frame,
                    text="FALSE",
                    command=lambda s=symbol, n=name: self.write_value(s, False, n)
                ).pack(side="left", padx=2)

        ttk.Button(left, text="Read All Once", command=self.read_all_once).pack(fill="x", pady=10)

        # MIDDLE: mover positions + variable states
        middle = ttk.Frame(body)
        middle.pack(side="left", fill="both", expand=True, padx=10)

        pos_frame = ttk.LabelFrame(middle, text="Live Mover Positions", padding=10)
        pos_frame.pack(fill="x", pady=(0, 10))

        self.mover_count_label = ttk.Label(pos_frame, text="Mover count: -")
        self.mover_count_label.pack(anchor="w", pady=(0, 8))

        pos_columns = ("mover", "x", "y", "z")
        self.pos_tree = ttk.Treeview(pos_frame, columns=pos_columns, show="headings", height=6)
        self.pos_tree.heading("mover", text="Mover")
        self.pos_tree.heading("x", text="X")
        self.pos_tree.heading("y", text="Y")
        self.pos_tree.heading("z", text="Z")
        self.pos_tree.column("mover", width=70)
        self.pos_tree.column("x", width=120)
        self.pos_tree.column("y", width=120)
        self.pos_tree.column("z", width=120)
        self.pos_tree.pack(fill="x")

        # rows are created dynamically based on the reported mover count

        states_frame = ttk.LabelFrame(middle, text="Variable States", padding=10)
        states_frame.pack(fill="both", expand=True)

        columns = ("name", "symbol", "value")
        self.tree = ttk.Treeview(states_frame, columns=columns, show="headings", height=24)
        self.tree.heading("name", text="Name")
        self.tree.heading("symbol", text="TwinCAT Symbol")
        self.tree.heading("value", text="Value")
        self.tree.column("name", width=140)
        self.tree.column("symbol", width=500)
        self.tree.column("value", width=120)
        self.tree.pack(fill="both", expand=True)

        for name, symbol in self.symbols.items():
            self.tree.insert("", "end", iid=name, values=(name, symbol, "-"))

        # RIGHT: Log
        right = ttk.LabelFrame(body, text="Log", padding=10)
        right.pack(side="left", fill="both", expand=True)

        self.log = tk.Text(right, wrap="word", height=30)
        self.log.pack(fill="both", expand=True)

    def log_msg(self, msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.insert("end", f"[{timestamp}] {msg}\n")
        self.log.see("end")
        print(f"[{timestamp}] {msg}")

    def connect(self):
        try:
            ams = self.ams_net_id.get().strip()
            port = int(self.ams_port.get().strip())
            local_ip = self.local_ip.get().strip() or None
            self.plc = pyads.Connection(ams, port, local_ip)
            self.plc.open()
            self.status_label.config(text="Connected")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.start_poll_btn.config(state="normal")
            self.log_msg(f"Connected to {ams}:{port}")
        except Exception as e:
            messagebox.showerror("Connection error", str(e))
            self.log_msg(f"Connection failed: {e}")

    def disconnect(self):
        self.stop_polling()
        try:
            if self.plc is not None:
                self.plc.close()
                self.plc = None
            self.status_label.config(text="Disconnected")
            self.connect_btn.config(state="normal")
            self.disconnect_btn.config(state="disabled")
            self.start_poll_btn.config(state="disabled")
            self.log_msg("Disconnected")
        except Exception as e:
            self.log_msg(f"Disconnect error: {e}")

    def read_symbol(self, symbol, symbol_name=None):
        if symbol_name in self.symbol_types:
            return self.plc.read_by_name(symbol, self.symbol_types[symbol_name])
        return self.plc.read_by_name(symbol)

    def write_value(self, symbol, value, name):
        if self.plc is None:
            messagebox.showwarning("Not connected", "Connect to ADS first.")
            return

        def worker():
            try:
                self.plc.write_by_name(symbol, value, pyads.PLCTYPE_BOOL)
                self.log_msg(f"{name} -> {value}")
                self.root.after(0, self.read_all_once)
            except Exception as e:
                self.log_msg(f"Write error on {name}: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def update_mover_positions(self, all_values):
        try:
            mover_count = self.plc.read_by_name("GVL_Movers.nMoverCount", pyads.PLCTYPE_INT)
            self.mover_count_label.config(text=f"Mover count: {mover_count}")
            all_values["MoverCount"] = mover_count
        except Exception as e:
            self.mover_count_label.config(text="Mover count: ERR")
            all_values["MoverCount"] = f"ERR: {e}"
            for item in self.pos_tree.get_children():
                self.pos_tree.item(item, values=(self.pos_tree.item(item)["values"][0], "ERR", "ERR", "ERR"))
            return

        existing_rows = len(self.pos_tree.get_children())

        if mover_count > existing_rows:
            for i in range(existing_rows + 1, mover_count + 1):
                self.pos_tree.insert("", "end", iid=f"mover_{i}", values=(i, "-", "-", "-"))
        elif mover_count < existing_rows:
            for i in range(existing_rows, mover_count, -1):
                self.pos_tree.delete(f"mover_{i}")

        for i in range(1, mover_count + 1):
            try:
                x = self.plc.read_by_name(f"GVL_Movers.aMovers[{i}].fPosX", pyads.PLCTYPE_LREAL)
                y = self.plc.read_by_name(f"GVL_Movers.aMovers[{i}].fPosY", pyads.PLCTYPE_LREAL)
                z = self.plc.read_by_name(f"GVL_Movers.aMovers[{i}].fPosZ", pyads.PLCTYPE_LREAL)
                self.pos_tree.item(f"mover_{i}", values=(i, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"))
                all_values[f"Mover{i}_X"] = x
                all_values[f"Mover{i}_Y"] = y
                all_values[f"Mover{i}_Z"] = z
            except Exception as e:
                self.pos_tree.item(f"mover_{i}", values=(i, "ERR", "ERR", "ERR"))
                all_values[f"Mover{i}"] = f"ERR: {e}"

    def read_all_once(self):
        if self.plc is None:
            return

        all_values = {}

        for name, symbol in self.symbols.items():
            try:
                value = self.read_symbol(symbol, name)
            except Exception as e:
                value = f"ERR: {e}"

            self.tree.set(name, "value", str(value))
            all_values[name] = value

        self.update_mover_positions(all_values)

        print("\n===== PLC STATE SNAPSHOT =====")
        for name, value in all_values.items():
            print(f"{name:20s}: {value}")
        print("===============================\n")

        self.log_msg("Read all symbols (printed to terminal)")

    def poll_loop(self):
        while self.polling:
            try:
                self.root.after(0, self.read_all_once)
            except Exception as e:
                self.log_msg(f"Polling error: {e}")
            time.sleep(max(int(self.poll_ms.get()) / 1000.0, 0.05))

    def start_polling(self):
        if self.plc is None or self.polling:
            return
        self.polling = True
        self.poll_thread = threading.Thread(target=self.poll_loop, daemon=True)
        self.poll_thread.start()
        self.start_poll_btn.config(state="disabled")
        self.stop_poll_btn.config(state="normal")
        self.log_msg("Started polling")

    def stop_polling(self):
        self.polling = False
        self.start_poll_btn.config(state="normal" if self.plc is not None else "disabled")
        self.stop_poll_btn.config(state="disabled")
        self.log_msg("Stopped polling")


if __name__ == "__main__":
    root = tk.Tk()
    app = ADSApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()
