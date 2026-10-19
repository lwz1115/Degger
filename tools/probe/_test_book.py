#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit test for the device-book (通讯录) logic. save_cfg is stubbed out."""
import sys

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
import usv_debug as U  # noqa: E402

U.save_cfg = lambda cfg: None          # 关键: 不落盘, 避免污染真实配置

print("--- DEFAULT_CFG 新键 ---")
for k in ("devices", "active_uid", "preferred_ip"):
    print("   %-12s = %r" % (k, U.DEFAULT_CFG.get(k)))

cfg = {"devices": {}}
print("--- book_set / book_all ---")
U.book_set(cfg, "SN123456", name="新遥控器", ip="192.168.10.171",
           conn_addr="192.168.10.171:43283")
U.book_set(cfg, "SN654321", name="旧遥控器", ip="192.168.10.162",
           conn_addr="192.168.10.162:39657")
for uid, rec in U.book_all(cfg).items():
    print("   %s -> name=%s ip=%s conn=%s" % (uid, rec["name"], rec["ip"], rec["conn_addr"]))

print("--- book_name_by_ip ---")
for ip in ("192.168.10.162", "192.168.10.171", "192.168.10.199"):
    print("   %-16s -> %r" % (ip, U.book_name_by_ip(cfg, ip)))

print("--- book_rename ---")
U.book_rename(cfg, "SN123456", "测试号A")
print("   ", U.book_all(cfg)["SN123456"]["name"])

print("--- book_set 更新不改名字(传空) ---")
U.book_set(cfg, "SN123456", ip="192.168.10.180", name="")
print("    name=%r ip=%r" % (U.book_all(cfg)["SN123456"]["name"],
                             U.book_all(cfg)["SN123456"]["ip"]))

print("--- book_del ---")
U.book_del(cfg, "SN654321")
print("    剩余:", sorted(U.book_all(cfg)))

print("--- 边界 ---")
print("    空 uid   :", U.book_set(cfg, "", name="x"))
print("    改名不存在:", U.book_rename(cfg, "NOPE", "x"))
print("    删除不存在:", U.book_del(cfg, "NOPE"))
print("    book_all(空cfg):", U.book_all({}))
print("    name_by_ip(空):", repr(U.book_name_by_ip(cfg, "")))

print("\nALL BOOK TESTS PASSED")
