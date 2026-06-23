# -*- coding: utf-8 -*-
"""Recursively upload a local dir tree to a remote dir on a nano via sftp."""
import sys, os, stat
import paramiko
from nano_ssh import NANOS, HOST

SKIP_DIRS = {"__pycache__", ".pytest_cache", ".sisyphus", ".git", "logs"}
SKIP_EXT = {".pyc", ".pyo", ".coverage"}


def _mkdirs(sftp, remote_dir):
    parts = remote_dir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur += "/" + p
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur)


def upload_tree(key, local_root, remote_root):
    import nano_ssh
    # 1) collect files + remote subdirs
    files = []      # (local_path, remote_path)
    rdirs = set()
    for dirpath, dirnames, filenames in os.walk(local_root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        rel = os.path.relpath(dirpath, local_root).replace("\\", "/")
        rdir = remote_root if rel == "." else remote_root + "/" + rel
        rdirs.add(rdir)
        for fn in filenames:
            if os.path.splitext(fn)[1] in SKIP_EXT:
                continue
            files.append((os.path.join(dirpath, fn), rdir + "/" + fn))
    # 2) create all remote dirs in one shot via SSH
    mk = " ".join("'%s'" % d for d in sorted(rdirs))
    rc, out, err = nano_ssh.run(key, "mkdir -p %s && echo MKOK" % mk)
    if "MKOK" not in out:
        raise RuntimeError("mkdir failed on %s: %s %s" % (key, out, err))
    # 3) sftp.put files
    n = NANOS[key]
    t = paramiko.Transport((n.get("host", HOST), n["port"]))
    t.connect(username=n["user"], password=n["pw"])
    sftp = paramiko.SFTPClient.from_transport(t)
    try:
        for lp, rp in files:
            sftp.put(lp, rp)
        print("uploaded %d files to %s:%s" % (len(files), key, remote_root))
    finally:
        sftp.close(); t.close()


if __name__ == "__main__":
    key = sys.argv[1]
    local_root = sys.argv[2]
    remote_root = sys.argv[3]
    upload_tree(key, local_root, remote_root)
