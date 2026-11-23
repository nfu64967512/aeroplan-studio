# -*- coding: utf-8 -*-
"""R1 harness: bind `_pn_cmd` from an ARBITRARY tot_node.py source text.

Same trick as tests/test_nz_closed_loop.py (AST -> mixin, no rclpy needed),
but parameterised on the source so the SAME driver can run
  * the pre-B1 baseline  (git show HEAD:.../tot_node.py)
  * the post-B1 worktree (both switches off / on)
and the outputs can be compared bit-for-bit.

Read-only w.r.t. the repo.
"""
from __future__ import annotations

import ast
import csv
import math
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
sys.path.insert(0, str(PATCHES))
import predictor as _pred                                        # noqa: E402

GRAV = 9.80665


class Bind:
    """One bound copy of the guidance law, from one source text."""

    def __init__(self, src, label):
        self.label = label
        self.src = src
        self.tree = ast.parse(src)
        self.ns = self._module_ns()
        names = [n for n in ("_geom", "_qbar", "_eas_of", "_itcg_bias",
                             "_pn_cmd", "_pn_step") if self._has(n)]
        self.method_names = names
        ns = dict(self.ns)
        exec("class _M:\n" + "\n\n".join(self._block(n) for n in names), ns)
        self.M = ns["_M"]

    # ---- ast helpers -------------------------------------------------
    def _has(self, name):
        return any(isinstance(n, ast.FunctionDef) and n.name == name
                   for n in ast.walk(self.tree))

    def _func(self, name):
        for n in ast.walk(self.tree):
            if isinstance(n, ast.FunctionDef) and n.name == name:
                return n
        raise AssertionError(f"{self.label}: no {name}()")

    def _block(self, name):
        seg = ast.get_source_segment(self.src, self._func(name))
        return "\n".join("    " + ln for ln in seg.splitlines())

    def _module_ns(self):
        ns = {"math": math, "csv": csv, "time": None, "_pred": _pred,
              "EARTH": 111320.0}
        todo = []
        for n in self.tree.body:
            if (isinstance(n, ast.Assign) and len(n.targets) == 1
                    and isinstance(n.targets[0], ast.Name)):
                try:
                    ns[n.targets[0].id] = ast.literal_eval(n.value)
                except Exception:                                # noqa: BLE001
                    todo.append((n.targets[0].id,
                                 ast.get_source_segment(self.src, n.value)))
        for name, s in todo:
            try:
                ns[name] = eval(s, ns)                           # noqa: S307
            except Exception:                                    # noqa: BLE001
                pass
        for n in self.tree.body:
            if isinstance(n, ast.FunctionDef):
                try:
                    exec(ast.get_source_segment(self.src, n), ns)
                except Exception:                                # noqa: BLE001
                    pass
        return ns

    def class_default(self, name, fallback=None):
        for cls in (n for n in ast.walk(self.tree) if isinstance(n, ast.ClassDef)):
            for st in cls.body:
                if isinstance(st, ast.Assign) and any(
                        isinstance(t, ast.Name) and t.id == name
                        for t in st.targets):
                    return ast.literal_eval(st.value)
        return fallback


class _Log:
    def __init__(self):
        self.errors, self.infos = [], []

    def info(self, m, *a, **k):
        self.infos.append(str(m))

    def warn(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass

    def error(self, m, *a, **k):
        self.errors.append(str(m))


#: RRE/RRD contract snapshot (contract_RR*.json + swarm_ros2_up.sh defaults).
#: pn_los_floor=1.0 / pn_nz_min=-1.0 / pn_vmax=210 come from the contract file;
#: n_lat / n_vert / pn_lead / pitch limits are the shipped defaults (the
#: contract does not override them).
CONTRACT = dict(
    itcg_on=False, itcg_gain=1.0, itcg_a_max_g=0.4,
    mpng_on=False, mpng_k_eps=0.0, mpng_k_s=0.0,
    n_lat=4.0, n_vert=5.0, pn_lead=1.0, pn_vmax=210.0,
    pn_los_floor=1.0, pn_nz_min=-1.0, pn_alpha_tau=1.75, tick_dt=0.05,
    dive_pitch_max=45.0, dive_pitch_min=-80.0,
    pn_thr_trim=0.36, pn_thr_gain=0.030, pn_late_full=True,
    dive_full_thr=False, fz_r=100.0, fz_tgo=1.5,
)

#: path-A knobs. Absent on the pre-B1 baseline; harmless extras there.
PATH_A_OFF = dict(nz_closed_loop=False, nz_clamp_after_floor=False,
                  nz_max_g=3.0, aero_cl_alpha=6.9503, aero_cl_zero=0.56381,
                  aero_cl_max=3.2503, aero_q_sigma_exp=2.0)


def make_node(bind, over=None):
    """A stub aircraft carrying exactly what `_pn_cmd` touches."""
    cfg = dict(CONTRACT)
    cfg.update(PATH_A_OFF)
    cfg.update(over or {})

    class Node(bind.M):
        _nz_lead_warned = False

        def __init__(self):
            for k, v in cfg.items():
                setattr(self, k, v)
            self.itcg_e, self.itcg_s, self.itcg_side = {}, {}, {}
            self.mpng_k0, self.itcg_dbg, self.itcg_a = {}, {}, {}
            self.alpha, self.nz_dbg = {}, {}
            self.alt_amsl, self.frozen, self.pos, self.vel = {}, {}, {}, {}
            self.att, self.sent = {}, []
            self.T = None
            self.dive_T = None
            self.log = _Log()
            self._h = 0.0
            self.tgo = 40.0

        def _zg(self, i):
            return 0.0

        def _tgo_of(self, i, d):
            return self.tgo

        def _att_target(self, i, roll, pitch, thr):
            self.sent.append((i, roll, pitch, thr))

        def get_logger(self):
            return self.log

    return Node()
