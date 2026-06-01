"""CustomTkinter main window: inputs, file load, shape pick, live preview.

Toolpath generation runs on a worker thread (it can produce tens of thousands
of points) and marshals the result back to the Tk main thread via ``after``.
"""

from __future__ import annotations

import os
import threading
from tkinter import filedialog, messagebox

import customtkinter as ctk
from shapely.affinity import translate
from shapely.geometry import Polygon

from .canvas import Part3DView, ToolpathCanvas
from ..cam.adaptive import estimate_moves
from ..cam.feeds import (DEFAULT_RPM_MAX, MATERIALS, TOOL_MATERIALS,
                         rate_cut, recommend)
from ..cam.params import JobParams, StockParams, ToolParams
from ..cam.strategy import CutMode, make_paths, make_paths_multi
from ..cam.toolpath import Toolpath
from ..gcode.generator import FILE_EXT, write_gcode_file
from ..io.loader import load_file
from ..io.step_parser import step_available

# Above this estimated move count a job is slow to compute and preview; warn the
# user first since it almost always means a too-small tool or a wrong import scale.
WARN_MOVES = 2_000_000

# label, key, default, kind
_FIELDS = [
    ("Stock width X (mm)", "stock_x", "100", "entry"),
    ("Stock height Y (mm)", "stock_y", "100", "entry"),
    ("Stock thickness Z (mm)", "stock_z", "6", "entry"),
    ("Tool diameter (mm)", "diameter", "3.175", "entry"),
    ("Stepover (% of dia)", "stepover_pct", "12", "entry"),
    ("Loop radius (mm, blank=dia)", "loop_radius", "", "entry"),
    ("Step down (mm)", "step_down", "1.0", "entry"),
    ("Total depth (mm)", "total_depth", "3.0", "entry"),
    ("Feed rate (mm/min)", "feed", "600", "entry"),
    ("Plunge rate (mm/min)", "plunge", "150", "entry"),
    ("Spindle RPM", "spindle", "10000", "entry"),
    ("Safe Z (mm)", "safe_z", "5.0", "entry"),
]


def parse_inputs(values: dict[str, str],
                 climb: bool) -> tuple[ToolParams, JobParams, StockParams]:
    """Pure validation/parsing of the raw UI strings. Raises ValueError."""
    def num(key: str) -> float:
        try:
            return float(values[key])
        except (KeyError, ValueError):
            raise ValueError(f"'{key}' must be a number")

    tool = ToolParams(diameter_mm=num("diameter"))
    stock = StockParams(width_mm=num("stock_x"), height_mm=num("stock_y"),
                        thickness_mm=num("stock_z"))
    loop_raw = values.get("loop_radius", "").strip()
    job = JobParams(
        total_depth_mm=num("total_depth"),
        step_down_mm=num("step_down"),
        stepover_fraction=num("stepover_pct") / 100.0,
        feed_rate=num("feed"),
        plunge_rate=num("plunge"),
        spindle_rpm=num("spindle"),
        safe_z_mm=num("safe_z"),
        loop_radius_mm=float(loop_raw) if loop_raw else None,
        climb=climb,
    )
    if job.total_depth_mm > stock.thickness_mm + 1e-6:
        raise ValueError("total depth exceeds stock thickness")
    return tool, job, stock


def center_offset(shapes: list[Polygon], width_mm: float,
                  height_mm: float) -> tuple[float, float]:
    """Translation that centres the combined bbox of all shapes on the stock."""
    minx = min(s.bounds[0] for s in shapes)
    miny = min(s.bounds[1] for s in shapes)
    maxx = max(s.bounds[2] for s in shapes)
    maxy = max(s.bounds[3] for s in shapes)
    return (width_mm / 2.0 - (minx + maxx) / 2.0,
            height_mm / 2.0 - (miny + maxy) / 2.0)


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.title("osxCAM — Trochoidal 2.5D")
        self.geometry("1100x720")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.shapes: list[Polygon] = []
        self.pocket: Polygon | None = None
        self.paths: list[Toolpath] | None = None
        self.stock: StockParams | None = None
        self.job: JobParams | None = None
        self.tool: ToolParams | None = None
        self.entries: dict[str, ctk.CTkEntry] = {}

        self._build_controls()
        # Two views share the same grid cell; the toggle raises one or the other.
        self.canvas = ToolpathCanvas(self)
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        self.view3d = Part3DView(self)
        self.view3d.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        self.canvas.tkraise()

    def _build_controls(self) -> None:
        panel = ctk.CTkScrollableFrame(self, width=300, label_text="CAM Parameters")
        panel.grid(row=0, column=0, sticky="ns", padx=10, pady=10)

        # STEP only works in the conda runtime (pythonocc-core); advertise it
        # in the button only when it's actually importable here.
        open_label = ("Open SVG / DXF / STEP…" if step_available()
                      else "Open SVG / DXF…")
        ctk.CTkButton(panel, text=open_label,
                      command=self._on_open).pack(fill="x", pady=(4, 2))
        self.file_label = ctk.CTkLabel(panel, text="(no file)", text_color="#888")
        self.file_label.pack(fill="x")

        ctk.CTkLabel(panel, text="View").pack(anchor="w", pady=(8, 0))
        self.view_toggle = ctk.CTkSegmentedButton(
            panel, values=["2D toolpath", "3D part"],
            command=self._on_view_change)
        self.view_toggle.set("2D toolpath")
        self.view_toggle.pack(fill="x")

        ctk.CTkLabel(panel, text="Units").pack(anchor="w", pady=(8, 0))
        self.units = ctk.CTkOptionMenu(panel, values=["px", "mm", "in"])
        self.units.set("px")
        self.units.pack(fill="x")

        ctk.CTkLabel(panel, text="Shape").pack(anchor="w", pady=(8, 0))
        self.shape_menu = ctk.CTkOptionMenu(panel, values=["—"],
                                            command=self._on_shape)
        self.shape_menu.pack(fill="x")

        ctk.CTkLabel(panel, text="Cut mode").pack(anchor="w", pady=(8, 0))
        self.cut_mode = ctk.CTkOptionMenu(
            panel, values=[m.value for m in CutMode],
            command=self._on_cut_mode)
        self.cut_mode.set(CutMode.POCKET.value)
        self.cut_mode.pack(fill="x")

        self.center_btn = ctk.CTkButton(panel, text="Center on stock",
                                        command=self._on_center)
        self.center_btn.pack(fill="x", pady=(6, 2))

        self.all_shapes = ctk.CTkCheckBox(panel, text="Machine all shapes",
                                          command=self._refresh_preview)
        self.all_shapes.pack(anchor="w", pady=(4, 0))

        for label, key, default, _ in _FIELDS:
            ctk.CTkLabel(panel, text=label).pack(anchor="w", pady=(8, 0))
            e = ctk.CTkEntry(panel)
            e.insert(0, default)
            e.pack(fill="x")
            self.entries[key] = e

        # Feeds & speeds estimator -> auto-fills feed/plunge/spindle above
        ctk.CTkLabel(panel, text="Material").pack(anchor="w", pady=(12, 0))
        self.material = ctk.CTkOptionMenu(panel, values=list(MATERIALS.keys()))
        self.material.set(next(iter(MATERIALS)))
        self.material.pack(fill="x")

        ctk.CTkLabel(panel, text="Tool material").pack(anchor="w", pady=(8, 0))
        self.tool_material = ctk.CTkOptionMenu(panel, values=list(TOOL_MATERIALS))
        self.tool_material.set(TOOL_MATERIALS[0])
        self.tool_material.pack(fill="x")

        ctk.CTkLabel(panel, text="Flutes").pack(anchor="w", pady=(8, 0))
        self.flutes = ctk.CTkOptionMenu(panel, values=["1", "2", "3", "4"])
        self.flutes.set("2")
        self.flutes.pack(fill="x")

        ctk.CTkButton(panel, text="Calc feeds & speeds",
                      command=self._on_calc_feeds).pack(fill="x", pady=(6, 2))

        ctk.CTkButton(panel, text="Rate cut (mild–aggressive)",
                      command=self._on_rate_cut).pack(fill="x", pady=(0, 2))

        self.climb = ctk.CTkCheckBox(panel, text="Climb milling (CCW)")
        self.climb.select()
        self.climb.pack(anchor="w", pady=(10, 4))

        self.gen_btn = ctk.CTkButton(panel, text="Generate Toolpath",
                                     command=self._on_generate)
        self.gen_btn.pack(fill="x", pady=(6, 4))

        self.sim_btn = ctk.CTkButton(panel, text="Simulate cut",
                                     command=self._on_simulate, state="disabled")
        self.sim_btn.pack(fill="x", pady=(6, 2))

        # 1x plays in real machining time; the rest just fast-forward.
        self.sim_speed = ctk.CTkOptionMenu(
            panel, values=["1x", "2x", "4x", "8x", "16x", "32x", "64x"])
        self.sim_speed.set("4x")
        self.sim_speed.pack(fill="x", pady=(0, 4))

        self.export_btn = ctk.CTkButton(panel, text="Export G-code (GRBL)…",
                                        command=self._on_export, state="disabled")
        self.export_btn.pack(fill="x", pady=(8, 4))

        self.status = ctk.CTkLabel(panel, text="Load a file to begin.",
                                   text_color="#9ad", wraplength=260,
                                   justify="left")
        self.status.pack(fill="x", pady=(8, 0))

    # ---- actions -------------------------------------------------------
    def _on_open(self) -> None:
        # macOS Tk 8.6 greys out files that don't match the *selected* filter,
        # and mishandles 4-char extensions like ".step" -- so the default entry
        # is a permissive "*" (show everything, nothing greyed). The named
        # entries below let the user narrow if they want.
        path = filedialog.askopenfilename(
            filetypes=[("All supported (SVG/DXF/STEP)", "*"),
                       ("Vector (SVG/DXF)", "*.svg *.dxf"),
                       ("STEP (*.step *.stp)", "*.step *.stp"),
                       ("All files", "*")])
        if not path:
            return
        try:
            scale = None
            self.shapes = load_file(path, units=self.units.get(), scale=scale)
        except Exception as exc:  # surface parse errors to the user
            self._set_status(f"Load failed: {exc}", error=True)
            return

        self.file_label.configure(text=os.path.basename(path))
        names = [f"#{i} ({s.area:.0f} mm²)" for i, s in enumerate(self.shapes)]
        self.shape_menu.configure(values=names)
        self.shape_menu.set(names[0])
        self.paths = None
        self._select_shape(0)
        self._set_status(f"Loaded {len(self.shapes)} shape(s). "
                         "Pick one and Generate.")

    def _on_shape(self, _name: str) -> None:
        idx = self.shape_menu.cget("values").index(self.shape_menu.get())
        self.paths = None
        self._select_shape(idx)

    def _on_center(self) -> None:
        if not self.shapes:
            self._set_status("Load a file first.", error=True)
            return
        stock = self._read_stock()
        if stock is None:
            self._set_status("Enter valid stock dimensions first.", error=True)
            return
        dx, dy = center_offset(self.shapes, stock.width_mm, stock.height_mm)
        self.shapes = [translate(s, xoff=dx, yoff=dy) for s in self.shapes]
        self.paths = None
        self.export_btn.configure(state="disabled")
        try:
            idx = self.shape_menu.cget("values").index(self.shape_menu.get())
        except ValueError:
            idx = 0
        self._select_shape(idx)
        self._set_status(f"Centered {len(self.shapes)} shape(s) on stock "
                         f"({dx:+.1f}, {dy:+.1f} mm).")

    def _read_stock(self) -> StockParams | None:
        """Best-effort stock from the current fields, for live preview."""
        try:
            return StockParams(width_mm=float(self.entries["stock_x"].get()),
                               height_mm=float(self.entries["stock_y"].get()),
                               thickness_mm=float(self.entries["stock_z"].get()))
        except (ValueError, KeyError):
            return None

    def _machine_all(self) -> bool:
        return bool(self.all_shapes.get()) and len(self.shapes) > 1

    def _outlines(self) -> list[Polygon]:
        if self._machine_all():
            return self.shapes
        return [self.pocket] if self.pocket is not None else []

    def _current_mode(self) -> CutMode:
        try:
            return CutMode(self.cut_mode.get())
        except (ValueError, AttributeError):
            return CutMode.POCKET

    def _on_cut_mode(self, _value: str) -> None:
        # Changing strategy invalidates any generated path.
        self.paths = None
        self.export_btn.configure(state="disabled")
        self.sim_btn.configure(state="disabled", text="Simulate cut")
        self.canvas.draw_scene(self._outlines(), None, self.stock)
        self._refresh_3d_if_active()
        self._set_status(f"Cut mode: {self._current_mode().value}. "
                         "Generate to preview.")

    def _total_depth_mm(self) -> float:
        try:
            return float(self.entries["total_depth"].get())
        except (ValueError, KeyError):
            return 0.0

    def _in_3d(self) -> bool:
        return (getattr(self, "view_toggle", None) is not None
                and self.view_toggle.get() == "3D part")

    def _refresh_3d_if_active(self) -> None:
        if self._in_3d():
            self.view3d.draw(self.stock, self._outlines(), self._total_depth_mm())

    def _on_view_change(self, value: str) -> None:
        # Stop any running simulation so it can't tick against a hidden view.
        self.canvas.stop_simulation()
        self.view3d.stop_simulation()
        if hasattr(self, "sim_btn"):
            self.sim_btn.configure(text="Simulate cut")
        if value == "3D part":
            self.stock = self._read_stock()
            self.view3d.draw(self.stock, self._outlines(),
                             self._total_depth_mm())
            self.view3d.tkraise()
            self._set_status("3D view — drag to rotate, scroll to zoom.")
        else:
            self.canvas.draw_scene(self._outlines(), self.paths, self.stock)
            self.canvas.tkraise()

    def _refresh_preview(self) -> None:
        self.stock = self._read_stock()
        self.canvas.draw_scene(self._outlines(), self.paths, self.stock)
        if hasattr(self, "sim_btn"):
            self.sim_btn.configure(text="Simulate cut")
        self._refresh_3d_if_active()

    def _select_shape(self, idx: int) -> None:
        if 0 <= idx < len(self.shapes):
            self.pocket = self.shapes[idx]
            self.stock = self._read_stock()
            self.canvas.draw_scene(self._outlines(), None, self.stock)
            self._refresh_3d_if_active()

    def _set_entry(self, key: str, value: object) -> None:
        e = self.entries[key]
        e.delete(0, "end")
        e.insert(0, str(value))

    def _on_calc_feeds(self) -> None:
        try:
            diameter = float(self.entries["diameter"].get())
        except ValueError:
            self._set_status("Enter a valid tool diameter first.", error=True)
            return
        try:
            res = recommend(self.material.get(), self.tool_material.get(),
                            diameter, int(self.flutes.get()),
                            rpm_max=DEFAULT_RPM_MAX)
        except ValueError as exc:
            self._set_status(str(exc), error=True)
            return

        self._set_entry("spindle", res.spindle_rpm)
        self._set_entry("feed", res.feed_rate)
        self._set_entry("plunge", res.plunge_rate)
        limited = (f"  ⚠ RPM-limited (spindle max {int(DEFAULT_RPM_MAX)}); "
                   f"feed reduced to suit" if res.rpm_limited else "")
        self._set_status(
            f"{self.material.get()}: {res.spindle_rpm} rpm, feed "
            f"{res.feed_rate}, plunge {res.plunge_rate} mm/min  (chip "
            f"{res.chipload_mm:.3f} mm, Vc {res.surface_speed_m_min} m/min)."
            f"{limited}  Suggestions — verify and adjust.",
            error=bool(res.rpm_limited))

    def _on_rate_cut(self) -> None:
        try:
            values = {k: e.get() for k, e in self.entries.items()}
            tool, job, _ = parse_inputs(values, bool(self.climb.get()))
            rating = rate_cut(
                self.material.get(), self.tool_material.get(),
                tool.diameter_mm, int(self.flutes.get()),
                spindle_rpm=job.spindle_rpm, feed_rate=job.feed_rate,
                step_down_mm=job.step_down_mm,
                stepover_fraction=job.stepover_fraction,
                rpm_max=DEFAULT_RPM_MAX)
        except ValueError as exc:
            self._set_status(str(exc), error=True)
            return

        body = [f"Aggressiveness: {rating.label}  ({rating.score}/100)",
                "",
                "0 = very mild (rubbing/heat, poor tool life), "
                "100 = very aggressive (overload/breakage). Aim for the "
                "balanced middle (~50)."]
        if rating.notes:
            body.append("")
            body.extend(f"• {n}" for n in rating.notes)
        else:
            body.append("")
            body.append("No cautions — these numbers look well balanced.")
        messagebox.showinfo("Cut rating", "\n".join(body))
        self._set_status(
            f"Cut rating: {rating.label} ({rating.score}/100). "
            f"{'See dialog for cautions.' if rating.notes else 'Well balanced.'}",
            error=rating.score <= 15 or rating.score >= 85)

    def _on_generate(self) -> None:
        if self.pocket is None:
            self._set_status("Load and select a shape first.", error=True)
            return
        try:
            values = {k: e.get() for k, e in self.entries.items()}
            tool, job, stock = parse_inputs(values, climb=bool(self.climb.get()))
        except ValueError as exc:
            self._set_status(str(exc), error=True)
            return

        self.stock = stock
        self.job = job
        self.tool = tool
        mode = self._current_mode()
        all_mode = self._machine_all()
        targets = list(self.shapes) if all_mode else [self.pocket]

        minx = min(s.bounds[0] for s in targets)
        miny = min(s.bounds[1] for s in targets)
        maxx = max(s.bounds[2] for s in targets)
        maxy = max(s.bounds[3] for s in targets)
        fit_warn = ("  ⚠ part extends beyond stock"
                    if not stock.contains_bounds(minx, miny, maxx, maxy) else "")

        # Pre-flight size guard (pocket clearing only): a tiny tool relative to
        # the part (or a wrong import scale) explodes the loop count into minutes
        # of compute. Contour modes trace a boundary -- always cheap -- so skip
        # the guard for them. Catch it cheaply and let the user fix inputs first.
        est = estimate_moves(targets, tool, job) if mode is CutMode.POCKET else 0
        if est > WARN_MOVES:
            proceed = messagebox.askyesno(
                "Very large toolpath",
                f"This job is estimated at ~{est:,} moves and may take a long "
                f"time to compute and preview.\n\n"
                f"Part: {maxx - minx:.1f} x {maxy - miny:.1f} mm   "
                f"Tool: {tool.diameter_mm} mm   Stepover: "
                f"{job.stepover_mm(tool):.3f} mm\n\n"
                f"This usually means the tool diameter is too small, or the file "
                f"imported at the wrong scale/units. Generate anyway?")
            if not proceed:
                self._set_status(
                    f"Cancelled — estimated ~{est:,} moves. Check tool size / "
                    f"units.", error=True)
                return

        self.gen_btn.configure(state="disabled", text="Generating…")
        self._set_status(f"Computing {mode.value} toolpath…")
        self._fit_warn = fit_warn

        def work():
            try:
                if all_mode:
                    paths, skipped = make_paths_multi(targets, tool, job, mode)
                else:
                    paths, skipped = make_paths(targets[0], tool, job, mode), []
                self.after(0, lambda: self._done(paths, job, skipped))
            except Exception as exc:
                # bind exc as a default arg: the `as exc` name is cleared when
                # the except block exits, before the deferred lambda runs.
                self.after(0, lambda e=exc: self._fail(e))

        threading.Thread(target=work, daemon=True).start()

    def _done(self, paths: list[Toolpath], job: JobParams,
              skipped: list[int]) -> None:
        self.paths = paths
        self.canvas.draw_scene(self._outlines(), paths, self.stock)
        moves = sum(len(p.moves) for p in paths)
        self.gen_btn.configure(state="normal", text="Generate Toolpath")
        self.export_btn.configure(state="normal" if paths else "disabled")
        self.sim_btn.configure(state="normal" if paths else "disabled",
                               text="Simulate cut")

        mode = self._current_mode()
        if not paths:
            if mode is CutMode.POCKET:
                # nothing generated: pocket too narrow for tool + loop radius
                loop_r = job.loop_radius(self.tool) if self.tool else 0.0
                self._set_status(
                    f"No toolpath — pocket too narrow for this tool. Needs width "
                    f"≳ {2 * self.tool.radius_mm + 2 * loop_r:.2f} mm "
                    f"(tool {self.tool.diameter_mm} mm + loop radius {loop_r:.2f} "
                    f"mm). Try a smaller tool, a smaller loop radius, or check the "
                    f"import scale.", error=True)
            else:
                self._set_status(
                    f"No toolpath — the tool ({self.tool.diameter_mm} mm) is too "
                    f"large for this profile offset, or the shape is too small. "
                    f"Try a smaller tool or check the import scale.", error=True)
            return

        warn = getattr(self, "_fit_warn", "")
        skip = f"  ({len(skipped)} shape(s) too narrow for tool)" if skipped else ""
        rate = ""
        try:
            rating = rate_cut(
                self.material.get(), self.tool_material.get(),
                self.tool.diameter_mm, int(self.flutes.get()),
                spindle_rpm=job.spindle_rpm, feed_rate=job.feed_rate,
                step_down_mm=job.step_down_mm,
                stepover_fraction=job.stepover_fraction,
                rpm_max=DEFAULT_RPM_MAX)
            rate = f"  Cut: {rating.label} ({rating.score}/100)."
        except (ValueError, AttributeError):
            pass
        unit = "guide rings" if mode is CutMode.POCKET else "contours"
        self._set_status(f"{len(paths)} {unit}, {moves} moves, "
                         f"{len(job.z_layers())} Z layer(s).{skip}{rate}{warn}",
                         error=bool(warn or skipped))
        self._refresh_3d_if_active()

    def _on_simulate(self) -> None:
        # Toggle: if a run is in progress (in either view), the button stops it.
        if self.canvas.is_simulating() or self.view3d.is_simulating():
            self.canvas.stop_simulation()
            self.view3d.stop_simulation()
            if self._in_3d():
                self.view3d.draw(self.stock, self._outlines(),
                                 self._total_depth_mm())
            else:
                self.canvas.draw_scene(self._outlines(), self.paths, self.stock)
            self.sim_btn.configure(text="Simulate cut")
            self._set_status("Simulation stopped.")
            return
        if not self.paths or self.tool is None:
            self._set_status("Generate a toolpath before simulating.",
                             error=True)
            return

        try:
            speed = float(self.sim_speed.get().rstrip("x"))
        except ValueError:
            speed = 1.0
        feed = self.job.feed_rate if self.job is not None else 600.0
        plunge = self.job.plunge_rate if self.job is not None else 200.0

        def fmt(sec: float) -> str:
            return f"{int(sec) // 60}:{int(sec) % 60:02d}"

        def on_tick(elapsed: float, total: float) -> None:
            pct = 100 * elapsed / total if total > 0 else 100
            self._set_status(f"Simulating… {pct:.0f}%  "
                             f"({fmt(elapsed)} / {fmt(total)} cut time at "
                             f"{self.sim_speed.get()})")

        def on_done() -> None:
            self.sim_btn.configure(text="Simulate cut")
            self._set_status("Simulation complete.")

        # Play in whichever view is showing: 3D descends through the Z layers,
        # 2D plays the flat XY toolpath. 1x = real machining time.
        if self._in_3d():
            z_layers = (self.job.z_layers() if self.job is not None
                        else [-self._total_depth_mm()])
            started = self.view3d.simulate(
                self.stock, self._outlines(), self.paths, self.tool.radius_mm,
                z_layers, feed_rate=feed, plunge_rate=plunge, speed=speed,
                on_tick=on_tick, on_done=on_done)
        else:
            started = self.canvas.simulate(
                self._outlines(), self.paths, self.stock, self.tool.radius_mm,
                feed_rate=feed, plunge_rate=plunge, speed=speed,
                on_tick=on_tick, on_done=on_done)
        if started:
            self.sim_btn.configure(text="Stop simulation")

    def _on_export(self) -> None:
        if not self.paths or self.job is None:
            self._set_status("Generate a toolpath before exporting.", error=True)
            return
        path = filedialog.asksaveasfilename(
            defaultextension=FILE_EXT,
            filetypes=[("G-code", "*.nc *.gcode *.tap"), ("All", "*.*")])
        if not path:
            return
        try:
            write_gcode_file(path, self.paths, self.job, tool=self.tool)
        except Exception as exc:
            self._set_status(f"Export failed: {exc}", error=True)
            return
        self._set_status(f"Saved GRBL G-code to {os.path.basename(path)}")

    def _fail(self, exc: Exception) -> None:
        self.gen_btn.configure(state="normal", text="Generate Toolpath")
        self._set_status(f"Failed: {exc}", error=True)

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status.configure(text=text,
                              text_color="#ff6b6b" if error else "#9ad")


def main() -> None:
    App().mainloop()


if __name__ == "__main__":
    main()
