# -*- coding: utf-8 -*-
"""汇总 PID_Check 目录下当天所有航段报告的关键指标, 输出趋势表"""
import re, sys, os, glob

d = sys.argv[1] if len(sys.argv) > 1 else "/mnt/e/WSL2/PID_Check"
files = sorted(glob.glob(os.path.join(d, "2026*_*_*AUTO_*.txt")))
if not files:
    files = sorted(glob.glob(os.path.join(d, "2026*_*.txt")))

def rd(p):
    for enc in ("utf-8", "gbk", "latin-1"):
        try:
            return open(p, encoding=enc).read()
        except UnicodeDecodeError:
            continue
    return ""


def grab(txt, pat, cast=float, grp=1, default=None):
    m = re.search(pat, txt)
    if not m:
        return default
    try:
        return cast(m.group(grp))
    except Exception:
        return default


rows = []
for fn in files:
    t = rd(fn)
    # 转向环段
    str_sec = t.split("速度环 ATC_SPEED")[0]
    spd_sec = t.split("速度环 ATC_SPEED")[1] if "速度环 ATC_SPEED" in t else ""
    r = {}
    r["file"] = os.path.basename(fn)
    r["seg"] = grab(t, r"#(\d+)\s+模式", int)
    r["time"] = grab(t, r"记录时间 (\S+ \S+)", str)
    r["dur"] = grab(t, r"\(持续 ([\d.]+)s\)")
    r["n"] = grab(t, r"样本 (\d+)", int)
    r["hz"] = grab(t, r"\(([\d.]+) Hz\)")

    r["des_max"] = grab(str_sec, r"期望转向率\s+mean=\s*([-\d.]+)\s+峰值=\s*([\d.]+)", str)
    r["ach_max"] = grab(str_sec, r"实际转向率\s+mean=\s*([-\d.]+)\s+峰值=\s*([\d.]+)", str)
    r["err_rms"] = grab(str_sec, r"跟踪误差\s+mean=\s*[-\d.]+\s+RMS=\s*([\d.]+)")
    r["lag"] = grab(str_sec, r"期望->实际 滞后 = ([\d.]+)")
    r["osc"] = grab(str_sec, r"误差摆动频率 = ([\d.]+)")
    r["p_pct"] = grab(str_sec, r"\n\s+P\s+均值=.*?占比=\s*([\d.]+)%")
    r["i_pct"] = grab(str_sec, r"\n\s+I\s+均值=.*?占比=\s*([\d.]+)%")
    r["d_pct"] = grab(str_sec, r"\n\s+D\s+均值=.*?占比=\s*([\d.]+)%")
    r["ff_pct"] = grab(str_sec, r"\n\s+FF 均值=.*?占比=\s*([\d.]+)%")

    r["gs"] = grab(str_sec, r"地速 mean=([\d.]+)")
    r["thr"] = grab(str_sec, r"油门 mean=([\d.]+)")
    r["thr_gs"] = grab(str_sec, r"油门/地速 = ([\d.]+)")
    r["diff_max"] = grab(str_sec, r"左右差 mean=[-\d.]+\s+RMS=[\d.]+\s+峰值=\s*([-\d.]+)")
    r["slip"] = grab(str_sec, r"侧滑角\s+mean=[-\d.]+°\s+峰值=([\d.]+)°")

    r["he_rms"] = grab(str_sec, r"航向误差\s+mean=\s*[-\d.]+\s+RMS=\s*([\d.]+)")
    r["xt_mean"] = grab(str_sec, r"横向航迹误差 mean=\s*([\d.]+)")
    r["xt_rms"] = grab(str_sec, r"横向航迹误差 mean=[\d.]+\s+RMS=\s*([\d.]+)")
    r["xt_max"] = grab(str_sec, r"横向航迹误差 mean=[\d.]+\s+RMS=[\d.]+\s+峰值=\s*([\d.]+)")
    r["wp_dist"] = grab(str_sec, r"到目标航点距离 mean=([\d.]+)")
    r["spd_des"] = grab(spd_sec, r"期望速度\s+mean=\s*([\d.]+)")
    r["spd_ach"] = grab(spd_sec, r"实际速度\s+mean=\s*([\d.]+)")
    r["spd_i"] = grab(spd_sec, r"\n\s+I\s+均值=.*?占比=\s*([\d.]+)%")
    r["spd_p"] = grab(spd_sec, r"\n\s+P\s+均值=.*?占比=\s*([\d.]+)%")
    r["spd_ff"] = grab(spd_sec, r"\n\s+FF 均值=.*?占比=\s*([\d.]+)%")
    rows.append(r)

rows.sort(key=lambda x: x["seg"] or 0)

def f(v, w=7, p=1):
    if v is None:
        return " " * w
    if isinstance(v, str):
        return v[:w].rjust(w)
    return ("%.*f" % (p, v)).rjust(w)


print("=" * 108)
print(" 转向环：期望/实际/误差/滞后")
print("=" * 108)
print(" 段  时间      持续   样本  Hz   期望mean 期望峰值 实际峰值  误差RMS  滞后   P%%    I%%    D%%   差速峰")
for r in rows:
    dm = re.search(r"([-\d.]+)\s+([\d.]+)", r["des_max"] or "")
    am = re.search(r"([-\d.]+)\s+([\d.]+)", r["ach_max"] or "")
    print(" %s  %-8s %s %s %s %s %s %s %s %s %s %s %s %s" % (
        str(r["seg"]).rjust(2), (r["time"] or "")[-8:], f(r["dur"], 6),
        f(r["n"], 5, 0), f(r["hz"], 5, 1),
        f(float(dm.group(1)) if dm else None), f(float(dm.group(2)) if dm else None),
        f(float(am.group(2)) if am else None),
        f(r["err_rms"]), f(r["lag"], 6, 2),
        f(r["p_pct"], 5), f(r["i_pct"], 5), f(r["d_pct"], 5), f(r["diff_max"], 6, 0)))

print()
print("=" * 108)
print(" 速度环 + 导航误差")
print("=" * 108)
print(" 段  期望速度 实际速度  FF%%    P%%    I%%   地速   油门  油门/地速  航向RMS  横迹mean 横迹RMS 横迹峰 到点距离 侧滑峰")
for r in rows:
    print(" %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s" % (
        str(r["seg"]).rjust(2),
        f(r["spd_des"]), f(r["spd_ach"]),
        f(r["spd_ff"], 5), f(r["spd_p"], 5), f(r["spd_i"], 5),
        f(r["gs"], 6), f(r["thr"], 5),
        f(r["thr_gs"], 8, 2),
        f(r["he_rms"], 7), f(r["xt_mean"], 8, 2), f(r["xt_rms"], 7, 2),
        f(r["xt_max"], 6, 1), f(r["wp_dist"], 8, 1), f(r["slip"], 6, 0)))
