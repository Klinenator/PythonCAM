"""Embedded matplotlib canvas for the 2D toolpath preview.

Uses the OO Figure/FigureCanvasTkAgg API (no pyplot global state) so it embeds
cleanly in a CustomTkinter frame. Draws the source profile (white), islands
(grey), the trochoidal cutting loops (blue) and rapid links between rings (red,
dashed).
"""

from __future__ import annotations

import math

import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon

from ..cam.params import StockParams
from ..cam.toolpath import Toolpath

_BG = "#1e1e1e"
_OUTLINE = "#f0f0f0"
_ISLAND = "#888888"
_CUT = "#3a86ff"
_RAPID = "#ff4d4d"
_STOCK = "#5cb85c"
_TOOL = "#ffd166"        # the moving cutter marker during simulation
_DONE = "#4dd2a0"        # material already cleared (sim progress trail)

# Simulation pacing. The sim runs on a wall-clock model: 1x plays in real
# machining time, derived from the feed/plunge rates and the path geometry, so a
# job that takes 8 minutes on the machine takes 8 minutes at 1x. The speed menu
# multiplies that. The tick interval is just the redraw cadence.
_SIM_INTERVAL_MS = 33           # ~30 fps redraw
_RAPID_RATE = 2500.0            # mm/min assumed for non-cutting repositioning

# Preview-only limits. The G-code keeps full precision; these just keep the
# matplotlib redraw snappy on very large trochoidal jobs.
_PREVIEW_ARC_DEG = 18.0     # coarse arc sampling for the canvas
_PREVIEW_MAX_PTS = 200_000  # global point budget; paths are strided past this


def _cumulative_times(seq: list[tuple[float, float, float, bool]],
                      feed: float, plunge: float, rapid: float) -> list[float]:
    """Arrival time (machine seconds) at each point along ``seq``.

    seq points are (x, y, z, cutting). A segment between two cutting points runs
    at the feed rate; a segment that changes Z runs at the plunge rate; anything
    else is a rapid reposition. Rates are mm/min, distances mm.
    """
    feed = max(feed, 1e-6)
    plunge = max(plunge, 1e-6)
    rapid = max(rapid, 1e-6)
    t = [0.0] * len(seq)
    for i in range(len(seq) - 1):
        x0, y0, z0, c0 = seq[i]
        x1, y1, z1, c1 = seq[i + 1]
        d = math.dist((x0, y0, z0), (x1, y1, z1))
        if c0 and c1:
            rate = feed
        elif abs(z0 - z1) > 1e-9:
            rate = plunge
        else:
            rate = rapid
        t[i + 1] = t[i] + d * 60.0 / rate
    return t


class _SimClock:
    """Wall-clock animation driver shared by the 2D and 3D views.

    Advances a machine-time clock by the tick interval (scaled by the speed
    multiplier) and interpolates the cutter's position along the timed path, so
    1x plays in real machining time. Subclasses supply the artists by
    implementing ``_sim_update_artists``.
    """

    def _sim_begin(self, seq, times, speed, on_tick, on_done) -> None:
        self._sim_seq = seq
        self._sim_t = times
        self._sim_clock = 0.0
        self._sim_i = 0                      # last *committed* point index
        self._sim_xs: list[float] = []
        self._sim_ys: list[float] = []
        self._sim_zs: list[float] = []
        self._sim_speed = max(0.05, speed)
        self._sim_on_tick = on_tick
        self._sim_on_done = on_done
        self._sim_after = self.after(_SIM_INTERVAL_MS, self._sim_tick)

    def _sim_tick(self) -> None:
        seq, t, n = self._sim_seq, self._sim_t, len(self._sim_seq)
        self._sim_clock += (_SIM_INTERVAL_MS / 1000.0) * self._sim_speed
        clk = self._sim_clock

        # Commit every point whose arrival time has passed.
        while self._sim_i + 1 < n and t[self._sim_i + 1] <= clk:
            self._sim_i += 1
            x, y, z, cutting = seq[self._sim_i]
            if cutting:
                self._sim_xs.append(x)
                self._sim_ys.append(y)
                self._sim_zs.append(z)
            else:                            # break the trail across a hop
                self._sim_xs.append(float("nan"))
                self._sim_ys.append(float("nan"))
                self._sim_zs.append(float("nan"))

        i = self._sim_i
        if i + 1 < n:                        # interpolate within current segment
            t0, t1 = t[i], t[i + 1]
            frac = 0.0 if t1 <= t0 else max(0.0, min(1.0, (clk - t0) / (t1 - t0)))
            x0, y0, z0, _ = seq[i]
            x1, y1, z1, c1 = seq[i + 1]
            cx, cy, cz = (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac,
                          z0 + (z1 - z0) * frac)
            seg_cutting = bool(seq[i][3] and c1)
            finished = False
        else:
            cx, cy, cz = seq[-1][0], seq[-1][1], seq[-1][2]
            seg_cutting, finished = False, True

        if seg_cutting:                      # extend trail to the live tip
            tx, ty, tz = (self._sim_xs + [cx], self._sim_ys + [cy],
                          self._sim_zs + [cz])
        else:
            tx, ty, tz = self._sim_xs, self._sim_ys, self._sim_zs
        self._sim_update_artists(cx, cy, cz, tx, ty, tz)
        self.canvas.draw_idle()

        if self._sim_on_tick is not None:
            self._sim_on_tick(min(clk, t[-1]), t[-1])

        if finished:
            self._sim_after = None
            done, self._sim_on_done = self._sim_on_done, None
            if done is not None:
                done()
            return
        self._sim_after = self.after(_SIM_INTERVAL_MS, self._sim_tick)

    def is_simulating(self) -> bool:
        return getattr(self, "_sim_after", None) is not None

    def stop_simulation(self) -> None:
        if getattr(self, "_sim_after", None) is not None:
            self.after_cancel(self._sim_after)
            self._sim_after = None
        self._sim_on_done = None
        self._sim_on_tick = None


class ToolpathCanvas(_SimClock, ctk.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor=_BG)
        self.ax = self.fig.add_subplot(111)
        self._style_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Simulation state (see _SimClock / simulate()).
        self._sim_after: str | None = None
        self._sim_line = None
        self._sim_tool = None
        self._sim_on_tick = None
        self._sim_on_done = None

    def _style_axes(self) -> None:
        self.ax.set_facecolor(_BG)
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.tick_params(colors="#aaaaaa", labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color("#444444")
        self.ax.set_xlabel("X (mm)", color="#aaaaaa", fontsize=8)
        self.ax.set_ylabel("Y (mm)", color="#aaaaaa", fontsize=8)
        self.ax.grid(True, color="#333333", linewidth=0.5)

    def draw_scene(self, pockets: list[Polygon] | None,
                   paths: list[Toolpath] | None,
                   stock: StockParams | None = None,
                   cut_alpha: float = 1.0) -> None:
        # Any running simulation refers to artists about to be cleared; cancel
        # its timer so it can't tick against a stale axes.
        if self._sim_after is not None:
            self.after_cancel(self._sim_after)
            self._sim_after = None
        self.ax.clear()
        self._style_axes()

        if stock is not None:
            x0, y0, x1, y1 = stock.bounds
            self.ax.add_patch(Rectangle(
                (x0, y0), x1 - x0, y1 - y0, fill=True, facecolor=_STOCK,
                edgecolor=_STOCK, alpha=0.08, linewidth=1.2,
                linestyle="--", zorder=1))
            self.ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0],
                         color=_STOCK, linewidth=1.2, linestyle="--", zorder=1)

        for pocket in (pockets or []):
            if pocket is None or pocket.is_empty:
                continue
            ex = list(pocket.exterior.coords)
            self.ax.plot([p[0] for p in ex], [p[1] for p in ex],
                         color=_OUTLINE, linewidth=1.5, zorder=3)
            for ring in pocket.interiors:
                ic = list(ring.coords)
                self.ax.plot([p[0] for p in ic], [p[1] for p in ic],
                             color=_ISLAND, linewidth=1.5, zorder=3)

        if paths:
            polys = [tp.polyline(max_step_deg=_PREVIEW_ARC_DEG) for tp in paths]
            total = sum(len(p) for p in polys)
            stride = max(1, total // _PREVIEW_MAX_PTS)
            prev_end = None
            for pts in polys:
                if stride > 1 and len(pts) > 2:
                    # keep the true endpoint so rapid links and loops still close
                    pts = pts[::stride] + [pts[-1]]
                if prev_end is not None:  # rapid link between rings
                    self.ax.plot([prev_end[0], pts[0][0]],
                                 [prev_end[1], pts[0][1]],
                                 color=_RAPID, linewidth=0.7,
                                 linestyle="--", zorder=2)
                self.ax.plot([p[0] for p in pts], [p[1] for p in pts],
                             color=_CUT, linewidth=0.6, zorder=4,
                             alpha=cut_alpha)
                prev_end = pts[-1]

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    # ---- simulation ----------------------------------------------------
    def simulate(self, pockets: list[Polygon] | None,
                 paths: list[Toolpath] | None,
                 stock: StockParams | None,
                 tool_radius_mm: float, *,
                 feed_rate: float, plunge_rate: float,
                 speed: float = 1.0,
                 on_tick=None, on_done=None) -> bool:
        """Animate the cutter travelling the toolpath in real machining time
        (1x). Returns False if nothing to play; stop_simulation() cancels."""
        self.stop_simulation()
        if not paths:
            return False

        # Redraw the scene with the planned path dimmed; the simulation paints a
        # bright trail over it as the tool clears material.
        self.draw_scene(pockets, paths, stock, cut_alpha=0.18)

        # Flatten to an ordered point stream (z=0; pacing only needs XY here),
        # flagging the reposition hop between rings as non-cutting.
        seq: list[tuple[float, float, float, bool]] = []
        prev_end = None
        for tp in paths:
            pts = tp.polyline(max_step_deg=_PREVIEW_ARC_DEG)
            if not pts:
                continue
            if prev_end is not None:
                seq.append((pts[0][0], pts[0][1], 0.0, False))
            for x, y in pts:
                seq.append((x, y, 0.0, True))
            prev_end = pts[-1]
        if not seq:
            return False

        times = _cumulative_times(seq, feed_rate, plunge_rate, _RAPID_RATE)
        (self._sim_line,) = self.ax.plot([], [], color=_DONE, linewidth=1.4,
                                         zorder=5)
        self._sim_tool = Circle(seq[0][:2], max(tool_radius_mm, 1e-3),
                                facecolor=_TOOL, edgecolor="#000000",
                                alpha=0.55, linewidth=0.8, zorder=6)
        self.ax.add_patch(self._sim_tool)
        self._sim_begin(seq, times, speed, on_tick, on_done)
        return True

    def _sim_update_artists(self, cx, cy, cz, tx, ty, tz) -> None:
        self._sim_tool.center = (cx, cy)
        self._sim_line.set_data(tx, ty)


class Part3DView(_SimClock, ctk.CTkFrame):
    """A rotatable 3D preview of the stock block with the pocket(s) cut into it.

    2.5D only, matching the rest of osxCAM: the stock is a slab spanning
    z in [-thickness, 0] (Z0 = stock top, per the machine convention) and each
    selected profile is shown as a flat-bottomed pocket milled down to the job's
    total depth, with islands left standing as full-height pillars.
    """

    def __init__(self, master):
        super().__init__(master)
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor=_BG)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._style()

        # mplot3d rotates on drag but has no built-in scroll zoom; wire one up.
        self.canvas.mpl_connect("scroll_event", self._on_scroll)
        self._box_aspect = None   # (dx, dy, dz) set by draw()
        self._zoom = 1.0          # camera zoom multiplier driven by scrolling

        # Simulation state (see _SimClock / simulate()).
        self._sim_after: str | None = None
        self._sim_line = None
        self._sim_tool = None
        self._sim_on_tick = None
        self._sim_on_done = None

    # ---- interaction ---------------------------------------------------
    def _apply_zoom(self) -> None:
        if self._box_aspect is None:
            return
        try:
            # zoom > 1 enlarges the whole scene (box + part), unlike rescaling
            # the limits which just grows the part inside a fixed box.
            self.ax.set_box_aspect(self._box_aspect, zoom=self._zoom)
        except TypeError:
            self.ax.set_box_aspect(self._box_aspect)  # matplotlib < 3.6

    def _on_scroll(self, event) -> None:
        """Scroll wheel / two-finger scroll zooms the camera in and out.

        ``event.step`` is positive scrolling up (zoom in) and carries the
        trackpad magnitude. The zoom multiplier persists until the next redraw.
        """
        if event.inaxes is not self.ax:
            return
        self._zoom = max(0.2, min(8.0, self._zoom * 1.2 ** event.step))
        self._apply_zoom()
        self.canvas.draw_idle()

    def _style(self) -> None:
        ax = self.ax
        ax.set_facecolor(_BG)
        ax.tick_params(colors="#888888", labelsize=7)
        ax.set_xlabel("X (mm)", color="#aaaaaa", fontsize=8)
        ax.set_ylabel("Y (mm)", color="#aaaaaa", fontsize=8)
        ax.set_zlabel("Z (mm)", color="#aaaaaa", fontsize=8)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            try:
                axis.set_pane_color((0.11, 0.11, 0.11, 1.0))
            except Exception:
                pass

    @staticmethod
    def _walls(ring: list[tuple[float, float]], z_top: float,
               z_bot: float) -> list[list[tuple[float, float, float]]]:
        faces = []
        for (ax_, ay), (bx, by) in zip(ring, ring[1:]):
            faces.append([(ax_, ay, z_top), (bx, by, z_top),
                          (bx, by, z_bot), (ax_, ay, z_bot)])
        return faces

    def _add_box(self, x0, y0, z0, x1, y1, z1, *, face, edge, alpha) -> None:
        faces = [
            [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],  # bottom
            [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],  # top
            [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
            [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
            [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)],
            [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)],
        ]
        col = Poly3DCollection(faces, facecolor=face, edgecolor=edge,
                               alpha=alpha, linewidths=0.8)
        self.ax.add_collection3d(col)

    def _add_pocket(self, poly: Polygon, depth: float) -> None:
        ext = list(poly.exterior.coords)
        # pocket floor (machined surface) at -depth
        self.ax.add_collection3d(Poly3DCollection(
            [[(x, y, -depth) for x, y in ext]],
            facecolor=_CUT, edgecolor=_CUT, alpha=0.30, linewidths=0.6))
        # pocket side walls, stock top (0) down to the floor (-depth)
        self.ax.add_collection3d(Poly3DCollection(
            self._walls(ext, 0.0, -depth),
            facecolor=_CUT, edgecolor=_CUT, alpha=0.18, linewidths=0.5))
        # islands stay as full-height pillars (uncut material)
        for ring in poly.interiors:
            ic = list(ring.coords)
            self.ax.add_collection3d(Poly3DCollection(
                self._walls(ic, 0.0, -depth),
                facecolor=_ISLAND, edgecolor=_ISLAND, alpha=0.35,
                linewidths=0.5))
            self.ax.add_collection3d(Poly3DCollection(
                [[(x, y, 0.0) for x, y in ic]],
                facecolor=_ISLAND, edgecolor=_ISLAND, alpha=0.5))

    def draw(self, stock: StockParams | None,
             pockets: list[Polygon] | None, depth_mm: float) -> None:
        if self._sim_after is not None:
            self.after_cancel(self._sim_after)
            self._sim_after = None
        self.ax.clear()
        self._style()
        if stock is None:
            self.canvas.draw_idle()
            return

        x0, y0, x1, y1 = stock.bounds
        th = stock.thickness_mm
        depth = max(0.0, min(depth_mm, th))

        self._add_box(x0, y0, -th, x1, y1, 0.0,
                      face=_STOCK, edge=_STOCK, alpha=0.06)
        for poly in (pockets or []):
            if poly is None or poly.is_empty:
                continue
            self._add_pocket(poly, depth)

        self.ax.set_xlim(x0, x1)
        self.ax.set_ylim(y0, y1)
        self.ax.set_zlim(-th, 0.0)
        self._box_aspect = ((x1 - x0) or 1.0, (y1 - y0) or 1.0, th or 1.0)
        self._zoom = 1.0          # fresh scene starts fit-to-view
        self._apply_zoom()
        self.canvas.draw_idle()

    # ---- simulation ----------------------------------------------------
    def simulate(self, stock: StockParams | None,
                 pockets: list[Polygon] | None,
                 paths: list[Toolpath] | None,
                 tool_radius_mm: float,
                 z_layers: list[float], *,
                 feed_rate: float, plunge_rate: float,
                 speed: float = 1.0,
                 on_tick=None, on_done=None) -> bool:
        """Animate the cutter travelling the toolpath in 3D, descending one Z
        layer at a time, in real machining time (1x). False if nothing to play."""
        self.stop_simulation()
        if not paths or not z_layers:
            return False

        depth = abs(min(z_layers)) if z_layers else 0.0
        self.draw(stock, pockets, depth)  # target scene, dimmed by its alphas

        polys = [tp.polyline(max_step_deg=_PREVIEW_ARC_DEG) for tp in paths]
        seq: list[tuple[float, float, float, bool]] = []
        prev = None
        for z in z_layers:
            for pts in polys:
                if not pts:
                    continue
                if prev is not None:  # reposition / plunge to this ring & depth
                    seq.append((pts[0][0], pts[0][1], z, False))
                for x, y in pts:
                    seq.append((x, y, z, True))
                prev = (pts[-1][0], pts[-1][1], z)
        if not seq:
            return False

        times = _cumulative_times(seq, feed_rate, plunge_rate, _RAPID_RATE)
        (self._sim_line,) = self.ax.plot([], [], [], color=_DONE, linewidth=1.6,
                                         zorder=5)
        x0, y0, z0, _ = seq[0]
        (self._sim_tool,) = self.ax.plot([x0], [y0], [z0], marker="o",
                                         markersize=8, color=_TOOL,
                                         markeredgecolor="#000000", zorder=6)
        self._sim_begin(seq, times, speed, on_tick, on_done)
        return True

    def _sim_update_artists(self, cx, cy, cz, tx, ty, tz) -> None:
        self._sim_tool.set_data_3d([cx], [cy], [cz])
        self._sim_line.set_data_3d(tx, ty, tz)
