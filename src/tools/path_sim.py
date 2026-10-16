# -*- coding: utf-8 -*-
"""
path_sim.py — 复刻 main.cpp 的 PosToAngle / AngleToPos / createStraightPath / createSmoothPath，
代入自定义 samples 检验，并用与 PosToAngle 约定自洽的正运动学还原爪尖轨迹，画图对比。

mode="legacy"  → 修复前的旧代码（三分支 + z<BASEHEIGHT 分支 90° bug + box 检查 + d∈(8,16) 检查）
mode="current" → 当前 main.cpp（annulus[minR,maxR] 检查 + 统一公式，无 d/z/量程检查）
"""
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ---------- 常量（与 main.cpp 一致） ----------
U = 8.0            # UPPERARM 大臂 /cm
F = 8.0            # FORARM 小臂 /cm
BH = 6.0           # BASEHEIGHT 底座高度 /cm
CL = 6.3           # CLAWLEN 钳子长度 /cm
PEN_H = 3.0        # PENHEIGHT 落笔高度 /cm
PEN = 178          # 握笔（钳合）
MIN_R = 10.0       # main.cpp L727
MAX_R = 25.0       # main.cpp L728
INIT = (90.0, 20.0, 90.0)   # base/shoulder/elbow 待机角
LIM_BASE = (0.0, 180.0)
LIM_SHOULDER = (20.0, 115.0)
LIM_ELBOW = (70.0, 145.0)
LIM_CLAW = (81.0, 178.0)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def acos_s(v):
    if v != v or v < -1.0 or v > 1.0:
        return float("nan")
    return math.acos(v)


# ---------- PosToAngle（legacy=修复前旧代码；current=当前 main.cpp） ----------
def pos_to_angle(x, y, z, mode="current"):
    if mode == "legacy":
        if x < -50 or x > 50 or y < 0 or y > 50 or z < 0 or z > 25:
            return INIT, "box"
        L = math.hypot(x, y) - CL
        base = math.degrees(math.atan2(y, x))
        if z > BH:
            h = z - BH
            d = math.hypot(L, h)
            beta = acos_s((U * U + L * L + h * h - F * F) / (2 * U * d))
            shoulder = beta + math.atan2(h, L)
        elif z < BH:
            h = BH - z
            d = math.hypot(L, h)
            beta = acos_s((U * U + L * L + h * h - F * F) / (2 * U * d))
            shoulder = beta + math.atan2(L, h)  # BUG: 恰大 90°
        else:
            h = 0.0
            d = abs(L)
            if L == 0.0:
                return INIT, "reach"
            beta = acos_s((U * U + L * L - F * F) / (2 * U * L))
            shoulder = beta
        phi = acos_s((U * U + F * F - (L * L + h * h)) / (2 * U * F))
        elbow = shoulder + phi
        if d < U + F and d > 8.0:
            return (base, 135.0 - math.degrees(shoulder), math.degrees(elbow) - 50.0), "ok"
        return INIT, "reach"
    # 当前 main.cpp：annulus 检查 + 统一公式（无 d/z/量程检查）
    distance = math.hypot(x, y)
    if distance < MIN_R or distance > MAX_R:
        return INIT, "annulus"
    L = distance - CL
    h = z - BH
    base = math.degrees(math.atan2(y, x))
    d = math.hypot(L, h)
    if d == 0.0:
        return INIT, "d0"
    arg_beta = (U * U + L * L + h * h - F * F) / (2 * U * d)
    arg_phi = (U * U + F * F - (L * L + h * h)) / (2 * U * F)
    if not (-1.0 <= arg_beta <= 1.0) or not (-1.0 <= arg_phi <= 1.0):
        return None, "nan"  # C++ 中 acosf 越界返回 NaN
    beta = math.acos(arg_beta)
    phi = math.acos(arg_phi)
    shoulder = beta + math.atan2(h, L)
    elbow = shoulder + phi
    return (base, 135.0 - math.degrees(shoulder), math.degrees(elbow) - 50.0), "ok"


# ---------- AngleToPos（当前 main.cpp：c+50、angle2 带符号） ----------
def angle_to_pos(a_s, b_s, c_s):
    b = 135.0 - b_s
    c = c_s + 50.0
    a, b, c = math.radians(a_s), math.radians(b), math.radians(c)
    L = math.sqrt(max(0.0, U * U + F * F - 2 * U * F * math.cos(c - b)))
    if L == 0.0:
        return None
    angle1 = acos_s((U * U + L * L - F * F) / (2 * U * L))
    angle2 = b - angle1
    D = L * math.cos(angle2)
    h = L * math.sin(angle2)
    return ((D + CL) * math.cos(a), (D + CL) * math.sin(a), BH + h)


# ---------- 正运动学（与 PosToAngle 的 135-/-50 约定自洽） ----------
def true_fk(b_s, sh_s, el_s):
    a = math.radians(b_s)
    th_s = math.radians(135.0 - sh_s)
    th_f = math.radians(el_s - 130.0)
    Lw = U * math.cos(th_s) + F * math.cos(th_f)
    zw = BH + U * math.sin(th_s) + F * math.sin(th_f)
    r = Lw + CL
    return (r * math.cos(a), r * math.sin(a), zw)


def servo_and_fk(angles, claw_raw, trunc=False):
    if angles is None or any(v != v for v in angles):
        return None
    if trunc:
        angles = tuple(float(int(v)) for v in angles)
    b = clamp(angles[0], *LIM_BASE)
    sh = clamp(angles[1], *LIM_SHOULDER)
    el = clamp(angles[2], *LIM_ELBOW)
    cw = clamp(claw_raw, *LIM_CLAW)
    x, y, z = true_fk(b, sh, el)
    return dict(base=b, shoulder=sh, elbow=el, claw=cw, x=x, y=y, z=z,
                clamped=(b != angles[0] or sh != angles[1] or el != angles[2]))


# ---------- Vector 辅助（复刻 main.cpp 的 Vector 运算） ----------
def vsub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vadd(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vmul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def vdiv(a, s): return (a[0] / s, a[1] / s, a[2] / s)
def vlen(a): return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


# ---------- createStraightPath 原样移植 ----------
def create_straight(samples, mode="current"):
    xy, ang, claws = [], [], []
    for i in range(1, 5):
        x0, y0 = samples[i][0], samples[i][1]
        x1, y1 = samples[i + 1][0], samples[i + 1][1]
        for k in range(48):
            t = k / 48.0
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            a, _ = pos_to_angle(x, y, PEN_H, mode)
            xy.append((x, y)); ang.append(a); claws.append(PEN)
    a, _ = pos_to_angle(samples[5][0], samples[5][1], PEN_H, mode)
    xy.append((samples[5][0], samples[5][1])); ang.append(a); claws.append(PEN)
    return xy, ang, claws, None


# ---------- createSmoothPath 原样移植 ----------
def create_smooth(samples, mode="current"):
    S = [None] * 7
    for i in range(1, 6):
        S[i] = (samples[i][0], samples[i][1], samples[i][2])
    S[0] = vsub(vmul(S[1], 2), S[2])
    S[6] = vsub(vmul(S[5], 2), S[4])
    t = [0.0] * 7
    for i in range(6):
        t[i + 1] = t[i] + math.sqrt(vlen(vsub(S[i + 1], S[i])))
    tang = [None] * 7
    tang[0] = vdiv(vsub(S[1], S[0]), t[1] - t[0])
    tang[6] = vdiv(vsub(S[6], S[5]), t[6] - t[5])
    for i in range(1, 6):
        tang[i] = vadd(vmul(vdiv(vsub(S[i], S[i - 1]), t[i] - t[i - 1]), (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1])),
                       vmul(vdiv(vsub(S[i + 1], S[i]), t[i + 1] - t[i]), (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1])))
    xy, ang, claws = [], [], []
    for i in range(4):
        a, _ = pos_to_angle(S[i + 1][0], S[i + 1][1], S[i + 1][2], mode)
        xy.append((S[i + 1][0], S[i + 1][1])); ang.append(a); claws.append(PEN)
        dt = t[i + 2] - t[i + 1]
        for k in range(1, 48):
            u = k / 48.0
            u2, u3 = u * u, u * u * u
            h00 = 2 * u3 - 3 * u2 + 1
            h10 = u3 - 2 * u2 + u
            h01 = -2 * u3 + 3 * u2
            h11 = u3 - u2
            res = vadd(vadd(vmul(S[i + 1], h00), vmul(tang[i + 1], h10 * dt)),
                       vadd(vmul(S[i + 2], h01), vmul(tang[i + 2], h11 * dt)))
            a, _ = pos_to_angle(res[0], res[1], res[2], mode)
            xy.append((res[0], res[1])); ang.append(a); claws.append(PEN)
    a, _ = pos_to_angle(S[5][0], S[5][1], PEN_H, mode)
    xy.append((S[5][0], S[5][1])); ang.append(a); claws.append(PEN)
    return xy, ang, claws, dict(S=S, t=t, tang=tang)


# ---------- 自定义 samples（5 个示教点，z=3；P2/P5 内收靠近圆心形成径向起伏） ----------
SAMPLES = [None,
           (11.8, 14.8, PEN_H),
           (12.6, 11.4, PEN_H),
           (16.1, 10.4, PEN_H),
           (17.5, 8.2, PEN_H),
           (17.6, 4.0, PEN_H),]


def analyze(name, xy, ang, claws, mode):
    drawn = [servo_and_fk(a, c) for a, c in zip(ang, claws)]
    ok = [d for d in drawn if d]
    n = len(ang)
    n_clamp = sum(1 for d in ok if d["clamped"])
    n_init = sum(1 for a in ang if a == INIT)
    n_nan = sum(1 for a in ang if a is None or any(v != v for v in a))
    n_open = sum(1 for c in claws if c != PEN)
    dev = 0.0
    dev_trunc = 0.0
    for (px, py), a in zip(xy, ang):
        d = servo_and_fk(a, PEN)
        if d:
            dev = max(dev, math.hypot(d["x"] - px, d["y"] - py))
        dt = servo_and_fk(a, PEN, trunc=True)
        if dt:
            dev_trunc = max(dev_trunc, math.hypot(dt["x"] - px, dt["y"] - py))
    zs = [d["z"] for d in ok]
    rs = [math.hypot(d["x"], d["y"]) for d in ok]
    tag = "当前代码" if mode == "current" else "旧版(90°bug)"
    print(f"[{name} | {tag}] 点数={n} 钳位点={n_clamp} 待机点={n_init} NaN点={n_nan} "
          f"钳爪非握笔点={n_open} 爪尖z=[{min(zs):.2f},{max(zs):.2f}] 半径=[{min(rs):.2f},{max(rs):.2f}] "
          f"与预期轨迹最大偏差={dev:.3f}cm（含uint8_t截断 {dev_trunc:.3f}cm）")
    return dict(drawn=drawn, dev=dev, dev_trunc=dev_trunc, n=n, n_clamp=n_clamp,
                n_init=n_init, n_nan=n_nan, n_open=n_open)


def teaching_bias():
    """正逆解往返误差（当前代码）"""
    worst = 0.0
    for i in range(1, 6):
        x, y, z = SAMPLES[i]
        ang, st = pos_to_angle(x, y, z, "current")
        if st != "ok":
            continue
        rec = angle_to_pos(*ang)
        worst = max(worst, math.hypot(rec[0] - x, rec[1] - y))
    return worst


def debug_nodes(internals):
    if not internals:
        return
    S, t, tang = internals["S"], internals["t"], internals["tang"]
    print("\n节点切线分析（平滑曲线，切线=前后两段方向的加权平均）:")
    for i in range(1, 6):
        din = math.degrees(math.atan2(S[i][1] - S[i - 1][1], S[i][0] - S[i - 1][0]))
        dout = math.degrees(math.atan2(S[i + 1][1] - S[i][1], S[i + 1][0] - S[i][0]))
        ta = math.degrees(math.atan2(tang[i][1], tang[i][0]))
        print(f"  P{i}({S[i][0]:.1f},{S[i][1]:.1f}): 入射 {din:+6.1f}°  出射 {dout:+6.1f}°  "
              f"方向反转 {abs(dout - din):5.1f}°  CR切线 {ta:+6.1f}°")
    P2, P3 = S[2], S[3]
    dt = t[3] - t[2]
    worst, worst_u = 0.0, 0.0
    for k in range(49):
        u = k / 48.0
        u2, u3 = u * u, u * u * u
        h00 = 2 * u3 - 3 * u2 + 1
        h10 = u3 - 2 * u2 + u
        h01 = -2 * u3 + 3 * u2
        h11 = u3 - u2
        px = h00 * P2[0] + h10 * tang[2][0] * dt + h01 * P3[0] + h11 * tang[3][0] * dt
        py = h00 * P2[1] + h10 * tang[2][1] * dt + h01 * P3[1] + h11 * tang[3][1] * dt
        d = math.hypot(px - (P2[0] + (P3[0] - P2[0]) * u), py - (P2[1] + (P3[1] - P2[1]) * u))
        if d > worst:
            worst, worst_u = d, u
    print(f"  P2→P3 段曲线相对弦的最大分离: {worst:.2f}cm (u={worst_u:.2f})")


def main():
    reach_lo = math.sqrt(64 - 9) + CL   # z=3 时 d>8 对应的内半径
    reach_hi = math.sqrt(256 - 9) + CL  # z=3 时 d<16 对应的外半径
    bias = teaching_bias()
    print(f"你的工作窗口 minR~maxR: ({MIN_R:.2f}, {MAX_R:.2f}) cm")
    print(f"z=3 实际可达半径环带: ({reach_lo:.2f}, {reach_hi:.2f}) cm")
    print(f"  窗口内侧 [{MIN_R:.1f}, {reach_lo:.1f}] = 深度折叠区（舵机解超量程，会被静默钳位）")
    print(f"  窗口外侧 [{reach_hi:.1f}, {MAX_R:.1f}] = NaN 区（d>16，acosf 越界）")
    print(f"正逆解往返最大误差(当前代码): {bias:.4f} cm")
    print()

    res = {}
    internals = None
    for mode in ("current", "legacy"):
        for name, gen in (("straight", create_straight), ("smooth", create_smooth)):
            xy, ang, claws, internals = gen(SAMPLES, mode)
            res[(name, mode)] = (xy, analyze(name, xy, ang, claws, mode))
    debug_nodes(internals)

    # ---------------- 画图 ----------------
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(13.6, 6.6))

    C_INT = "#64748b"   # 预期轨迹
    C_CUR = "#16a34a"   # 当前代码实际画出
    C_OLD = "#dc2626"   # 旧版实际画出

    for ax, (name, title) in zip(axes, [("straight", "createStraightPath 折线"),
                                        ("smooth", "createSmoothPath 平滑曲线")]):
        xy = res[(name, "current")][0]
        st_cur = res[(name, "current")][1]
        st_old = res[(name, "legacy")][1]
        d_cur = [d for d in st_cur["drawn"] if d]
        d_old = [d for d in st_old["drawn"] if d]

        # 你的工作窗口 annulus
        ax.add_patch(Circle((0, 0), MAX_R, fill=True, facecolor="#94a3b8", alpha=0.08, zorder=0))
        ax.add_patch(Circle((0, 0), MIN_R, fill=True, facecolor="white", alpha=1.0, zorder=1))
        ax.add_patch(Circle((0, 0), MAX_R, fill=False, edgecolor="#94a3b8", lw=0.8, ls="--", zorder=1))
        ax.add_patch(Circle((0, 0), MIN_R, fill=False, edgecolor="#94a3b8", lw=0.8, ls="--", zorder=1))
        # z=3 实际可达带
        ax.add_patch(Circle((0, 0), reach_hi, fill=True, facecolor="#22c55e", alpha=0.10, zorder=1))
        ax.add_patch(Circle((0, 0), reach_lo, fill=True, facecolor="white", alpha=1.0, zorder=1))
        ax.add_patch(Circle((0, 0), reach_hi, fill=False, edgecolor="#22c55e", lw=0.8, ls=":", zorder=1))
        ax.add_patch(Circle((0, 0), reach_lo, fill=False, edgecolor="#22c55e", lw=0.8, ls=":", zorder=1))

        # 预期轨迹
        ax.plot([p[0] for p in xy], [p[1] for p in xy], ls="--", lw=1.4,
                color=C_INT, marker=".", ms=3, zorder=3)
        # 平滑面板画弦参照
        if name == "smooth":
            ax.plot([SAMPLES[i][0] for i in range(1, 6)], [SAMPLES[i][1] for i in range(1, 6)],
                    ls=":", lw=1.4, color="#94a3b8", zorder=2)
        # 旧版实际画出
        ax.plot([d["x"] for d in d_old], [d["y"] for d in d_old], lw=1.8, color=C_OLD,
                marker="o", ms=2.5, zorder=4)
        # 当前代码实际画出
        ax.plot([d["x"] for d in d_cur], [d["y"] for d in d_cur], lw=1.2, color=C_CUR,
                marker="o", ms=2.5, zorder=5)

        for i in range(1, 6):
            ax.plot(SAMPLES[i][0], SAMPLES[i][1], marker="*", ms=13, color="#111827", zorder=6)
            ax.annotate(f"P{i}", (SAMPLES[i][0], SAMPLES[i][1]), textcoords="offset points",
                        xytext=(6, 5), fontsize=10, color="#111827")

        # 自适应坐标范围
        xs = [p[0] for p in xy] + [d["x"] for d in d_cur] + [d["x"] for d in d_old]
        ys = [p[1] for p in xy] + [d["y"] for d in d_cur] + [d["y"] for d in d_old]
        xs += [SAMPLES[i][0] for i in range(1, 6)]
        ys += [SAMPLES[i][1] for i in range(1, 6)]
        for rr in (MIN_R, MAX_R, reach_lo, reach_hi):
            xs += [-rr, rr]
            ys += [-rr, rr]
        px = max(2.0, 0.12 * (max(xs) - min(xs)))
        py = max(2.0, 0.12 * (max(ys) - min(ys)))
        ax.set_xlim(min(xs) - px, max(xs) + px)
        ax.set_ylim(min(ys) - py, max(ys) + py)
        ax.set_aspect("equal")
        ax.grid(True, ls=":", lw=0.5, alpha=0.5)
        ax.set_xlabel("x /cm")
        ax.set_ylabel("y /cm")
        ax.set_title(title, fontsize=13)

        zs = [d["z"] for d in d_cur]
        rs = [math.hypot(d["x"], d["y"]) for d in d_cur]
        info = (f"生成 {st_cur['n']} 点（上限241）\n"
                f"当前代码：{st_cur['n_clamp']} 点被舵机限位钳位，"
                f"与预期轨迹最大偏差 {st_cur['dev']:.2f}cm（含截断 {st_cur['dev_trunc']:.2f}cm）\n"
                f"实际爪尖：半径≈{sum(rs)/len(rs):.1f}cm、高度≈{sum(zs)/len(zs):.1f}cm\n"
                f"旧版(90°bug)：{st_old['n_clamp']} 点钳位，画成悬空圆弧")
        ax.text(0.03, 0.97, info, transform=ax.transAxes, fontsize=9.5, va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.45", facecolor="#f8fafc", edgecolor="#cbd5e1", alpha=0.95))

        # 图例
        ax.plot([], [], ls="--", lw=1.4, color=C_INT, label="预期轨迹（生成的坐标点）")
        ax.plot([], [], lw=1.2, color=C_CUR, label="当前代码实际画出")
        ax.plot([], [], lw=1.8, color=C_OLD, label="旧版实际画出（90° bug）")
        ax.plot([], [], ls=":", lw=1.4, color="#94a3b8", label="直线段（弦，仅示教点间）")
        ax.add_patch(Circle((0, 0), 1, fill=False, edgecolor="#94a3b8", ls="--",
                            label=f"你的窗口 minR~maxR = {MIN_R:.0f}~{MAX_R:.0f}cm"))
        ax.add_patch(Circle((0, 0), 1, fill=False, edgecolor="#22c55e", ls=":",
                            label=f"z=3 实际可达 {reach_lo:.1f}~{reach_hi:.1f}cm"))

        # 平滑面板：切线箭头 + P2 放大
        if name == "smooth" and internals:
            S, tang = internals["S"], internals["tang"]
            for i in range(1, 6):
                tx, ty = tang[i][0], tang[i][1]
                n = math.hypot(tx, ty)
                if n > 0:
                    ax.annotate("", xy=(S[i][0] + 1.3 * tx / n, S[i][1] + 1.3 * ty / n),
                                xytext=(S[i][0], S[i][1]),
                                arrowprops=dict(arrowstyle="-|>", color="#EA580C", lw=1.3), zorder=6)
            ax.plot([], [], color="#EA580C", lw=1.6, label="节点切线（Catmull-Rom 加权）")

            ax_in = ax.inset_axes([0.02, 0.32, 0.30, 0.25])
            ax_in.plot([p[0] for p in xy], [p[1] for p in xy], ls="--", lw=1.2, color=C_INT)
            ax_in.plot([S[i][0] for i in range(1, 6)], [S[i][1] for i in range(1, 6)],
                       ls=":", lw=1.2, color="#94a3b8")
            for i in range(1, 5):
                ax_in.plot(S[i][0], S[i][1], marker="*", ms=11, color="#111827")
            tx, ty = tang[2][0], tang[2][1]
            n = math.hypot(tx, ty)
            ta2 = math.degrees(math.atan2(ty, tx))
            rev2 = abs(math.degrees(math.atan2(S[3][1] - S[2][1], S[3][0] - S[2][0])) -
                       math.degrees(math.atan2(S[2][1] - S[1][1], S[2][0] - S[1][0])))
            ax_in.annotate("", xy=(S[2][0] + 2.0 * tx / n, S[2][1] + 2.0 * ty / n),
                           xytext=(S[2][0], S[2][1]),
                           arrowprops=dict(arrowstyle="-|>", color="#EA580C", lw=1.8))
            flat = "（几乎水平，急折）" if abs(ta2) < 20 else "（与局部方向一致）"
            ax_in.annotate(f"P2 切线 {ta2:+.1f}°{flat}\n入射/出射方向反转 {rev2:.0f}°",
                           xy=(13.2, 9.6), fontsize=9, color="#7C2D12")
            ax_in.set_xlim(10.4, 18.6)
            ax_in.set_ylim(9.4, 15.9)
            ax_in.set_title("P2 附近放大", fontsize=10)
            ax_in.tick_params(labelsize=8)
            ax_in.grid(True, ls=":", lw=0.5, alpha=0.5)
            ax_in.set_facecolor("#f8fafc")
            for spine in ax_in.spines.values():
                spine.set_edgecolor("#cbd5e1")

        ax.legend(loc="lower left", fontsize=8.5, framealpha=0.95)

    fig.suptitle("折线/曲线路径生成代入检验：自定义 5 个示教点（z=3 落笔高度）", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = "D:/Code/MeArm/src/tools/path_preview.png"
    fig.savefig(out, dpi=150)
    print("\nsaved:", out)


if __name__ == "__main__":
    main()
