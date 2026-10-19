# -*- coding: utf-8 -*-
"""
rat_report.py —— 离线诊断 rat_p_gui.py 记录的 CSV (只读, 不连飞控)

用法:
    python3 rat_report.py                    # 自动取目录下最新 rat_log_*.csv, 分析最近 30 秒
    python3 rat_report.py <samples.csv> --last 60
    python3 rat_report.py <samples.csv> --all
"""
import argparse
import glob
import math
import os
import sys
from collections import Counter

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
# 项目根目录(本文件在 app\ 子目录), 日志统一放根目录 Logs\ 下
PROJECT_ROOT = os.path.dirname(HERE)
LOG_DIR = os.path.join(PROJECT_ROOT, "Logs")
RAD2DEG = 180.0 / math.pi


# ---------------------------------------------------------------- 基础工具
def tofloat(s):
    try:
        return float(s)
    except Exception:
        return None


def load_samples(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        head = f.readline().strip().split(",")
        for line in f:
            p = line.strip().split(",")
            if len(p) < len(head):
                continue
            d = dict(zip(head, p))
            r = {"t": tofloat(d.get("t")), "mode": int(tofloat(d.get("mode")) or -1)}
            for k in ("des", "ach", "P", "I", "D", "FF"):
                r[k] = tofloat(d.get(k))
            for k in ("thr", "gs", "hdg", "yaw", "s1", "s3", "s1_t", "s3_t"):
                r[k] = tofloat(d.get(k))
            if r["t"] is None or r["des"] is None:
                continue
            rows.append(r)
    return rows


def load_params(path):
    out = []
    if not path or not os.path.exists(path):
        return out
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        f.readline()
        for line in f:
            p = line.strip().split(",")
            if len(p) < 3:
                continue
            t = tofloat(p[0]); v = tofloat(p[2])
            if t is None or v is None:
                continue
            out.append((t, p[1], v))
    return out


def stats(xs):
    if not xs:
        return None
    n = len(xs)
    m = sum(xs) / n
    rms = math.sqrt(sum(x * x for x in xs) / n)
    return {"n": n, "mean": m, "rms": rms, "max": max(xs), "min": min(xs)}


def resample(rows, key, t0, t1, dt=0.05):
    """把 (t, value) 线性插值到均匀网格, 返回 [(tg, vg)]"""
    pts = [(r["t"], r[key]) for r in rows if r.get(key) is not None]
    if len(pts) < 2:
        return [], dt
    pts.sort()
    grid = []
    k = 0
    tg = t0
    while tg <= t1:
        while k + 2 < len(pts) and pts[k + 1][0] < tg:
            k += 1
        a, b = pts[k], pts[k + 1] if k + 1 < len(pts) else pts[k]
        if b[0] - a[0] < 1e-9:
            v = b[1]
        else:
            f = (tg - a[0]) / (b[0] - a[0])
            f = max(0.0, min(1.0, f))
            v = a[1] + (b[1] - a[1]) * f
        grid.append((tg, v))
        tg += dt
    return grid, dt


def xcorr_lag(grid1, grid2, dt, max_lag_s=2.0):
    """返回 (lag_s, 相关系数)。lag>0 表示 grid2 落后 grid1"""
    n = min(len(grid1), len(grid2))
    if n < 20:
        return None, None
    a = [grid1[i][1] for i in range(n)]
    b = [grid2[i][1] for i in range(n)]
    ma, mb = sum(a) / n, sum(b) / n
    a = [x - ma for x in a]
    b = [x - mb for x in b]
    va = sum(x * x for x in a)
    vb = sum(x * x for x in b)
    if va < 1e-12 or vb < 1e-12:
        return None, None
    best, best_c = 0, -2.0
    ml = int(max_lag_s / dt)
    for lag in range(0, ml + 1):
        if n - lag < 20:
            break
        num = sum(a[i] * b[i + lag] for i in range(n - lag))
        c = num / math.sqrt(va * vb)
        if c > best_c:
            best_c, best = c, lag
    return best * dt, best_c


def zero_cross_freq(vals, dt):
    if len(vals) < 8:
        return None, 0
    m = sum(vals) / len(vals)
    z = 0
    for i in range(1, len(vals)):
        if (vals[i - 1] - m) * (vals[i] - m) < 0:
            z += 1
    dur = len(vals) * dt
    return (z / 2.0 / dur) if dur > 0 else None, z


def share(vals):
    return sum(abs(x) for x in vals)


# ---------------------------------------------------------------- 报告
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default=None)
    ap.add_argument("--params", default=None)
    ap.add_argument("--last", type=float, default=30.0, help="只分析最近 N 秒")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--dt", type=float, default=0.05)
    a = ap.parse_args()

    path = a.csv
    if not path:
        c = sorted(glob.glob(os.path.join(LOG_DIR, "rat_log_*.csv")))
        c = [x for x in c if not x.endswith(".params.csv")]
        if not c:
            print("找不到 rat_log_*.csv, 请先用 rat_p_gui.py 记录数据")
            return 1
        path = c[-1]
    if not os.path.isabs(path):
        path = os.path.join(HERE, path)
    if not os.path.exists(path):
        print("文件不存在: %s" % path)
        return 1

    ppath = a.params or (path[:-4] + ".params.csv")
    rows = load_samples(path)
    if len(rows) < 20:
        print("样本太少 (%d 条)。请让船运动并保持更长时间。" % len(rows))
        return 1
    prows = load_params(ppath)
    pv = {}
    for _t, _nm, _v in prows:
        pv[_nm] = _v

    t_end = rows[-1]["t"]
    t_start = rows[0]["t"] if a.all else max(rows[0]["t"], t_end - a.last)
    win = [r for r in rows if r["t"] >= t_start]
    dur = t_end - t_start
    rate = len(win) / dur if dur > 0 else 0

    print("=" * 74)
    print(" RAT 转向环 诊断报告")
    print("=" * 74)
    print(" 数据文件 : %s" % os.path.basename(path))
    print(" 分析窗口 : 最近 %.1f 秒 (t=%.1f~%.1f)" % (dur, t_start, t_end))
    print(" 样本数   : %d  (%.2f Hz)" % (len(win), rate))
    MODE_N = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
              6: "FOLLOW", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 15: "GUIDED", -1: "(无心跳)"}
    mc = Counter(r["mode"] for r in win)
    mstr = "  ".join("%s=%d%%" % (MODE_N.get(m, str(m)), n * 100 // len(win))
                     for m, n in mc.most_common(4))
    print(" 飞行模式 : %s" % mstr)

    # ---- 冻结检测: 控制器没在跑时 PID_TUNING 是上一次的旧值 ----
    frozen = 0
    for _key in ("des", "ach", "P"):
        _vs = [round(r[_key], 6) for r in win if r.get(_key) is not None]
        if len(set(_vs)) <= 2:
            frozen += 1
    if frozen >= 3:
        print()
        print("  !!! 警告: desired / achieved / P 全程几乎不变")
        print("      -> 转向控制器很可能没有在运行, 报文里是上一次运行留下的【旧值】。")
        print("      -> 下面的反算增益反映的是【历史某时刻】的增益, 不能用来判断当前状态。")
        print("      -> 请让船真正动起来 (已解锁 + 有转向指令) 后重新采集。")

    # ---- 单位判定: 用 yaw 数值微分对比 ach ----
    unit = "deg/s"
    g_yaw, dt = resample(win, "yaw", t_start, t_end, a.dt)
    if len(g_yaw) > 20:
        dy = [(g_yaw[i][1] - g_yaw[i - 1][1]) / dt for i in range(1, len(g_yaw))]
        ad = [abs(r["ach"]) for r in win if r["ach"] is not None]
        my = sum(abs(x) for x in dy) / len(dy)
        ma = sum(ad) / len(ad) if ad else 0
        if my > 0.1 and ma > 0.01:
            ratio = ma / my      # my 是 deg/s
            if 0.5 < ratio < 2:
                unit = "deg/s"
            elif 0.005 < ratio < 0.05:
                unit = "rad/s"
            print(" 单位判定 : PID_TUNING 为 %s  (|ach|/|d(yaw)/dt| = %.1f)" % (unit, ratio))
        else:
            print(" 单位判定 : 无法判定(数据几乎为 0, 船没转?)")
    k = RAD2DEG if unit == "rad/s" else 1.0

    # ---- 有效数据筛选: 转向率有效 或 在动 ----
    des = [r["des"] * k for r in win if r["des"] is not None]
    ach = [r["ach"] * k for r in win if r["ach"] is not None]
    err = [r["des"] * k - r["ach"] * k for r in win
           if r["des"] is not None and r["ach"] is not None]
    gs = [r["gs"] for r in win if r.get("gs") is not None]
    thr = [r["thr"] for r in win if r.get("thr") is not None]

    sd, sa, se = stats(des), stats(ach), stats(err)
    print()
    print("[1] 跟踪质量  (单位 %s 当量)" % unit)
    print("    期望转向率  mean=%8.3f   max=%8.3f" % (sd["mean"], max(abs(sd["max"]), abs(sd["min"]))))
    print("    实际转向率  mean=%8.3f   max=%8.3f" % (sa["mean"], max(abs(sa["max"]), abs(sa["min"]))))
    print("    跟踪误差    mean=%+8.3f   RMS=%7.3f" % (se["mean"], se["rms"]))

    mabs_err = sum(abs(x) for x in err) / len(err)
    bias = (se["mean"] / mabs_err) if mabs_err > 1e-6 else 0.0
    print("    稳态偏置度  %+.2f   (>+0.6 长期同号=有稳态误差; 接近0=在零附近来回)" % bias)

    if gs:
        sg = stats(gs)
        print()
        print("[2] 速度 / 油门")
        print("    地速 groundspeed  mean=%.3f m/s  max=%.3f  (n=%d)" % (sg["mean"], sg["max"], sg["n"]))
        if thr:
            st = stats(thr)
            print("    油门 VFR_HUD      mean=%.1f %%   max=%.1f" % (st["mean"], st["max"]))
            if sg["mean"] > 0.05:
                print("    > 隐含: 油门/速度 = %.2f  %%/(m/s)  (ATC_SPEED_P/I 量级参考)"
                      % (st["mean"] / sg["mean"]))

    # ---- 分项占比 ----
    print()
    print("[3] 控制输出分项 (PID_TUNING 的 FF/P/I/D 是各分项的实时输出值)")
    tot = 0.0
    term_abs = {}
    for key in ("FF", "P", "I", "D"):
        vals = [r[key] for r in win if r.get(key) is not None]
        s = share(vals)
        term_abs[key] = s
        tot += s
    for key in ("FF", "P", "I", "D"):
        vals = [r[key] for r in win if r.get(key) is not None]
        st = stats(vals)
        pc = (term_abs[key] / tot * 100.0) if tot > 1e-9 else 0.0
        print("    %-2s  均值=%+9.5f  |均值|=%9.5f   占比=%5.1f%%"
              % (key, st["mean"], sum(abs(x) for x in vals) / st["n"], pc))
    terms = {kk: term_abs[kk] / tot if tot > 1e-9 else 0.0 for kk in term_abs}

    # ---- 反算有效增益 ----
    # 已实测确认: PID_TUNING 的 desired/achieved 以 deg/s 下发,
    # 但 FF/P/I/D 四项是按内部 rad/s 目标/误差算出的输出。
    to_int = (1.0 / RAD2DEG) if unit == "deg/s" else 1.0
    mismatch = []
    print()
    print("[4] 反算有效增益  (报文 deg/s -> 内部 rad/s 需 /57.2958)")
    gff = [r["FF"] / (r["des"] * to_int) for r in win
           if r.get("FF") is not None and abs(r["des"]) > 1e-4]
    gkp = [r["P"] / ((r["des"] - r["ach"]) * to_int) for r in win
           if r.get("P") is not None and abs((r["des"] - r["ach"]) * to_int) > 0.02]
    if gff:
        o = sum(gff) / len(gff)
        p = pv.get("ATC_STR_RAT_FF")
        line = "    反算 FF = FF项/内部期望 = %10.6f  (n=%d)" % (o, len(gff))
        if p:
            line += "   参数=%.6g  比值=%.3f" % (p, o / p)
            if abs(o / p - 1.0) > 0.15:
                line += "  <== 不符!"
                mismatch.append("ATC_STR_RAT_FF")
        print(line)
    if gkp:
        o = sum(gkp) / len(gkp)
        p = pv.get("ATC_STR_RAT_P")
        line = "    反算 P  = P项/内部误差 = %10.6f  (n=%d)" % (o, len(gkp))
        if p:
            line += "   参数=%.6g  比值=%.3f" % (p, o / p)
            if abs(o / p - 1.0) > 0.15:
                line += "  <== 不符!"
                mismatch.append("ATC_STR_RAT_P")
        print(line)

    # ---- 滞后 / 振荡 ----
    print()
    print("[5] 滞后 与 振荡")
    gd, _ = resample(win, "des", t_start, t_end, a.dt)
    ga, _ = resample(win, "ach", t_start, t_end, a.dt)
    lag, cc = xcorr_lag(gd, ga, a.dt)
    f = r1 = r2 = None
    if lag is not None:
        print("    期望->实际 滞后 = %.2f s   (相关系数 %.3f)" % (lag, cc))
    if len(gd) > 20 and len(ga) > 20:
        n = min(len(gd), len(ga))
        diff = [ga[i][1] * k - gd[i][1] * k for i in range(n)]
        f, z = zero_cross_freq(diff, a.dt)
        if f:
            print("    误差摆动频率 = %.2f Hz  (%d 次过零)" % (f, z))
        h = n // 2
        r1 = math.sqrt(sum(x * x for x in diff[:h]) / max(1, h))
        r2 = math.sqrt(sum(x * x for x in diff[h:]) / max(1, n - h))
        trend = "收敛" if r2 < r1 * 0.85 else ("发散!" if r2 > r1 * 1.15 else "持平")
        print("    误差RMS  前半=%.3f  后半=%.3f  -> %s" % (r1, r2, trend))

    # ---- 转向代理 (差速船) ----
    fresh = [r for r in win if r.get("s1") is not None and r.get("s3") is not None
             and r.get("s1_t") is not None and (r["t"] - r["s1_t"]) < 0.8]
    if fresh:
        dif = [r["s1"] - r["s3"] for r in fresh]
        s1 = [r["s1"] for r in fresh]
        s3 = [r["s3"] for r in fresh]
        ms1, ms3, mdf = stats(s1), stats(s3), stats(dif)
        print()
        print("[6] 差速输出 (SERVO1=左油门 / SERVO3=右油门, n=%d)" % len(fresh))
        print("    左PWM mean=%7.1f   右PWM mean=%7.1f   左右差 mean=%+7.1f" % (ms1["mean"], ms3["mean"], mdf["mean"]))
        print("    差幅  RMS=%.1f  最大=%+.0f" % (mdf["rms"], max(abs(mdf["max"]), abs(mdf["min"]))))
    else:
        print()
        print("[6] 差速输出: 无新鲜样本 (SERVO_OUTPUT_RAW 仅 2Hz, 且本次未更新)")

    # ---- 参数 ----
    if prows:
        print()
        print("[7] 飞控参数 (窗口内最新值 / 变化次数)")
        seen = {}
        chg = {}
        for t, nm, v in prows:
            if t < t_start:
                continue
            if nm in seen and abs(seen[nm] - v) > max(1e-6, abs(seen[nm]) * 1e-5):
                chg[nm] = chg.get(nm, 0) + 1
            seen[nm] = v
        names = ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D", "ATC_STR_RAT_FF",
                 "ATC_STR_RAT_MAX", "ATC_STR_ANG_P", "WP_SPEED", "WP_ACCEL",
                 "WP_JERK", "WP_PIVOT_ANGLE"]
        for nm in names:
            if nm in seen:
                print("    %-18s = %-14.6g  变化 %d 次" % (nm, seen[nm], chg.get(nm, 0)))

    # ---- 结论 ----
    print()
    print("-" * 74)
    print(" 判读建议  (仅供参考, 需结合实际情况)")
    print("-" * 74)
    tips = []
    for nm in mismatch:
        tips.append("[关键] %s 实测有效增益与参数值不符 (见 [4] 节):"
                    " 参数可能未生效, 或该参数被飞控内部另作缩放 -> 建议做一次"
                    "大幅度改动(如 2 倍)验证是否能被反算值跟上。" % nm)
    if mabs_err < 1.0 and abs(sd["mean"]) > 1.0:
        tips.append("跟踪很好: 平均绝对误差 < 1 %s, 当前 P/I 基本合适。" % unit)
    if bias > 0.6 and mabs_err > 1.0:
        if terms["FF"] < 0.35:
            tips.append("稳态误差长期同号 (偏置 %.2f), 且 FF 项只占 %.0f%%:"
                        " 典型是 FF/CRUISE 前馈不足 -> 可小幅提高 ATC_STR_RAT_FF。" % (bias, terms["FF"] * 100))
        else:
            tips.append("稳态误差长期同号 (偏置 %.2f): 前馈够但回正慢 -> 可小幅提高 ATC_STR_RAT_P。" % bias)
    if bias < 0.3 and mabs_err > 3.0:
        tips.append("误差在零附近来回且幅度大 (RMS %.1f %s): 增益偏高或有振荡 -> 优先降 ATC_STR_RAT_P。" % (mabs_err, unit))
    if terms["I"] > 0.55:
        tips.append("I 项承担了 %.0f%% 的输出: P/FF 偏小, 且易积分饱和/超调 -> 降 I 或提高 P。" % (terms["I"] * 100))
    if terms["D"] < 0.03:
        tips.append("D 项几乎不出力 (%.1f%%): 若存在摆振, 加 ATC_STR_RAT_D 是最直接的手段。" % (terms["D"] * 100))
    if lag is not None and lag > 0.6:
        tips.append("期望->实际滞后 %.2f s 偏大: 可能是 MOT_SLEWRATE 限制或输出饱和 -> 检查是否推满。" % lag)
    if f and f > 0.5 and r2 > r1 * 1.15:
        tips.append("检测到 %.2f Hz 摆动且幅度在增大: 明确的自激/发散趋势 -> 必须降 P。" % f)
    if sd["mean"] * sa["mean"] < 0 and abs(sd["mean"]) > 2:
        tips.append("期望与实际符号相反(平均意义上): 可能接线/混控方向问题, 先确认转向方向。")
    if abs(sd["mean"]) < 0.5:
        tips.append("窗口内几乎没在转向: 需让船持续转向才有诊断价值 (AUTO/GUIDED 或手动画圈)。")
    if not tips:
        tips.append("数据中没有明显异常特征, 需要更长/更明确的转向动作才能进一步判断。")
    for i, t in enumerate(tips, 1):
        print("  %d. %s" % (i, t))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
