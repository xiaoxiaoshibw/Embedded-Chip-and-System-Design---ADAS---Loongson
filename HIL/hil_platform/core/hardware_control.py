# -*- coding: utf-8 -*-
"""Restricted hardware-control helpers for the real HIL rig.

The web UI should operate the two Jetson Nano boards through explicit actions,
not by exposing an arbitrary SSH shell. CARLA remains the world/perception
source and actuator sink; these helpers only manage the hardware control path.
"""

from __future__ import annotations

import concurrent.futures
import os
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import paramiko


REMOTE_DIR = "/home/jetson/adas/hil"
REMOTE_SOC_DIR = "/home/jetson/adas/lx/SOCCode"
ROS_DOMAIN_ID = 43
DEFAULT_PC_ZT = "10.218.44.190"
DEFAULT_PRIMARY_ZT = "10.218.44.10"
DEFAULT_BACKUP_ZT = "10.218.44.155"
EDGE_REMOTE_DIR = "%s/edge_results" % REMOTE_DIR
EDGE_LOCAL_DIR = Path(__file__).resolve().parents[1] / "edge_results"
PRIMARY_ADAS_CPUS = os.environ.get("PRIMARY_ADAS_CPUS", "0,1")
PRIMARY_GATEWAY_CPUS = os.environ.get("PRIMARY_GATEWAY_CPUS", "2")
BACKUP_ADAS_CPUS = os.environ.get("BACKUP_ADAS_CPUS", "0,1")
BACKUP_EDGE_CPUS = os.environ.get("BACKUP_EDGE_CPUS", "2,3")


def _targets() -> Dict[str, Dict[str, Any]]:
    return {
        "primary": {
            "host": os.environ.get("GATEWAY_HOST", DEFAULT_PRIMARY_ZT),
            "user": os.environ.get("NANO_USER", "jetson"),
            "pw": os.environ.get("NANO_PW_PRIMARY", "yahboom"),
            "role": "primary",
        },
        "backup": {
            "host": os.environ.get("BACKUP_HOST", DEFAULT_BACKUP_ZT),
            "user": os.environ.get("NANO_USER", "jetson"),
            "pw": os.environ.get("NANO_PW_BACKUP", "jetson"),
            "role": "backup",
        },
    }


def _client(target: str) -> paramiko.SSHClient:
    n = _targets()[target]
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        n["host"],
        port=22,
        username=n["user"],
        password=n["pw"],
        timeout=10,
        banner_timeout=15,
        auth_timeout=15,
        look_for_keys=False,
        allow_agent=False,
    )
    return c


def _run(target: str, cmd: str, timeout: int = 60) -> Dict[str, Any]:
    n = _targets()[target]
    started = time.monotonic()
    try:
        c = _client(target)
        try:
            _stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
            rc = stdout.channel.recv_exit_status()
        finally:
            c.close()
        return {
            "target": target,
            "host": n["host"],
            "ok": rc == 0,
            "rc": rc,
            "stdout": out,
            "stderr": err,
            "elapsed_ms": int((time.monotonic() - started) * 1000),
        }
    except Exception as exc:
        return {
            "target": target,
            "host": n["host"],
            "ok": False,
            "rc": None,
            "stdout": "",
            "stderr": repr(exc),
            "elapsed_ms": int((time.monotonic() - started) * 1000),
        }


def _parallel(targets: Iterable[str], cmd_for: Any, timeout: int = 60) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        futures = {
            ex.submit(_run, t, cmd_for(t), timeout): t
            for t in targets
        }
        for fut in concurrent.futures.as_completed(futures):
            t = futures[fut]
            out[t] = fut.result()
    return out


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _carla_bridge_root() -> Path:
    return Path(__file__).resolve().parents[2] / "carla_bridge"


def _soc_code_root() -> Path:
    return _repo_root() / "仿真" / "soc_code"


def _tcp_ready(host: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _cpu_list_for_adas(target: str) -> str:
    return PRIMARY_ADAS_CPUS if target == "primary" else BACKUP_ADAS_CPUS


def _quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _upload_dir_to_target(target: str, local_root: Path, remote_root: str) -> Dict[str, Any]:
    n = _targets()[target]
    started = time.monotonic()
    files = []
    skip_dirs = {"__pycache__", ".pytest_cache", ".sisyphus", ".git", "logs"}
    skip_ext = {".pyc", ".pyo", ".coverage"}
    for root, dirs, names in os.walk(local_root):
        dirs[:] = [d for d in dirs if d not in skip_dirs]
        rel = Path(root).relative_to(local_root).as_posix()
        remote_dir = remote_root if rel == "." else "%s/%s" % (remote_root, rel)
        for name in names:
            if Path(name).suffix in skip_ext:
                continue
            files.append((Path(root) / name, "%s/%s" % (remote_dir, name)))

    c = _client(target)
    try:
        c.exec_command("mkdir -p '%s'" % remote_root)[1].channel.recv_exit_status()
        sftp = c.open_sftp()
        try:
            made = {remote_root}
            for _lp, rp in files:
                rdir = rp.rsplit("/", 1)[0]
                if rdir not in made:
                    c.exec_command("mkdir -p '%s'" % rdir)[1].channel.recv_exit_status()
                    made.add(rdir)
            for lp, rp in files:
                sftp.put(str(lp), rp)
        finally:
            sftp.close()
    finally:
        c.close()

    if remote_root == REMOTE_SOC_DIR:
        verify_cmd = (
            "python3 -m py_compile '%s/ADAS.py' '%s/control/context.py' "
            "'%s/control/longitudinal_policy.py' '%s/longitudinal.py'"
            % (remote_root, remote_root, remote_root, remote_root)
        )
    else:
        verify_cmd = (
            "source /opt/ros/foxy/setup.bash 2>/dev/null; "
            "python3 -m py_compile '%s/hil_ros_gateway.py' '%s/start_hil_adas.py' "
            "'%s/stop_gateway.py' '%s/edge_result_collector.py'"
            % (remote_root, remote_root, remote_root, remote_root)
        )
    verify = _run(target, verify_cmd, timeout=40)
    return {
        "target": target,
        "host": n["host"],
        "ok": bool(verify.get("ok")),
        "uploaded": len(files),
        "stdout": verify.get("stdout", ""),
        "stderr": verify.get("stderr", ""),
        "elapsed_ms": int((time.monotonic() - started) * 1000),
    }


def deploy_gateway() -> Dict[str, Any]:
    """Upload carla_bridge/nano to both Nanos over the configured SSH hosts."""
    local_root = _carla_bridge_root() / "nano"
    if not local_root.exists():
        raise FileNotFoundError(str(local_root))
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        futures = {
            ex.submit(_upload_dir_to_target, t, local_root, REMOTE_DIR): t
            for t in ("primary", "backup")
        }
        result = {futures[f]: f.result() for f in concurrent.futures.as_completed(futures)}
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def deploy_adas_code() -> Dict[str, Any]:
    """Upload the SOCCode/ADAS runtime to both Nanos."""
    local_root = _soc_code_root()
    if not local_root.exists():
        raise FileNotFoundError(str(local_root))
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        futures = {
            ex.submit(_upload_dir_to_target, t, local_root, REMOTE_SOC_DIR): t
            for t in ("primary", "backup")
        }
        result = {futures[f]: f.result() for f in concurrent.futures.as_completed(futures)}
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def start_carla(port: int = 2000) -> Dict[str, Any]:
    """Start CARLA if needed and wait for its TCP port."""
    carla_exe = Path(os.environ.get("CARLA_EXE", "")) if os.environ.get("CARLA_EXE") else _repo_root() / "CALRA" / "CarlaUE4.exe"
    if _tcp_ready("127.0.0.1", port, timeout_s=0.5):
        return {"ok": True, "already_running": True, "port": port, "exe": str(carla_exe)}
    if not carla_exe.exists():
        raise FileNotFoundError(str(carla_exe))

    creationflags = 0
    if os.name == "nt":
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP
    subprocess.Popen(
        [str(carla_exe), "-quality-level=Low", "-windowed", "-ResX=1280", "-ResY=720"],
        cwd=str(carla_exe.parent),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=creationflags,
    )
    started = time.monotonic()
    for _ in range(90):
        if _tcp_ready("127.0.0.1", port, timeout_s=0.5):
            return {
                "ok": True,
                "already_running": False,
                "port": port,
                "exe": str(carla_exe),
                "elapsed_ms": int((time.monotonic() - started) * 1000),
            }
        time.sleep(1)
    raise RuntimeError("CARLA port %d did not become ready in time" % port)


def health() -> Dict[str, Any]:
    """Return bounded health details for both Nanos and the primary gateway."""
    def cmd(target: str) -> str:
        log = "/tmp/adas_hil_primary.log" if target == "primary" else "/tmp/adas_hil_backup.log"
        extra = ""
        if target == "primary":
            extra = (
                "echo __GATEWAY__; "
                "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | grep 'hil_ros_gateway.py' | grep -v grep || true; "
                "tail -8 /tmp/hil_gateway_esp32.log 2>/dev/null || true; "
                "echo __ROS__; "
                "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | "
                "grep -E 'ADAS.py --role|hil_ros_gateway.py' | grep -v grep || true; "
            )
        return (
            "echo __HOST__; hostname; "
            "echo __UPTIME__; uptime; "
            "echo __ADAS__; "
            "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | grep 'ADAS.py --role' | grep -v grep || true; "
            "echo __EDGE__; "
            "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | grep 'edge_result_collector.py' | grep -v grep || true; "
            "ls -1t %s 2>/dev/null | head -5 || true; "
            "echo __LOG__; tail -8 %s 2>/dev/null || true; "
            "%s"
        ) % (EDGE_REMOTE_DIR, log, extra)

    result = _parallel(("primary", "backup"), cmd, timeout=40)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def stop_perception_sim() -> Dict[str, Any]:
    cmd = (
        "pkill -f '[p]erception_sim.py' || true; pkill -f '[p]erception_sim' || true; "
        "source /opt/ros/foxy/setup.bash 2>/dev/null; "
        "export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; "
        "timeout 4 ros2 node list 2>/dev/null | sort || true"
    )
    result = _parallel(("primary", "backup"), lambda _t: cmd, timeout=30)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def restart_adas(target: str) -> Dict[str, Any]:
    targets = _select_targets(target)

    def cmd(t: str) -> str:
        n = _targets()[t]
        pw = n["pw"]
        adas_cpus = _cpu_list_for_adas(t)
        return (
            "python3 '%s/stop_gateway.py' || true; "
            "ADAS_CPU_LIST=%s python3 '%s/start_hil_adas.py' --role %s --domain %d --sudo-password '%s'"
        ) % (REMOTE_DIR, _quote(adas_cpus), REMOTE_DIR, n["role"], ROS_DOMAIN_ID, pw)

    result = _parallel(targets, cmd, timeout=120)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def start_gateway(source: str = "esp32") -> Dict[str, Any]:
    if source not in ("esp32", "jetson"):
        raise ValueError("source must be esp32 or jetson")
    pc_host = os.environ.get("PC_HOST", DEFAULT_PC_ZT)
    gateway_cpus = PRIMARY_GATEWAY_CPUS
    cmd = (
        "python3 '%s/stop_gateway.py' || true; "
        "nohup bash -lc 'source /opt/ros/foxy/setup.bash; "
        "export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=0; "
        "taskset -c %s python3 %s/hil_ros_gateway.py --pc-host %s "
        "--sensor-port 42100 --actuation-port 42101 --tcp-port 42110 "
        "--actuation-source %s --status-hz 20' > /tmp/hil_gateway_%s.log 2>&1 < /dev/null & "
        "sleep 1; ps -ef | grep '[h]il_ros_gateway.py' || true; "
        "tail -20 /tmp/hil_gateway_%s.log 2>/dev/null || true"
    ) % (REMOTE_DIR, ROS_DOMAIN_ID, gateway_cpus, REMOTE_DIR, pc_host, source, source, source)
    r = _run("primary", cmd, timeout=40)
    r["source"] = source
    r["pc_host"] = pc_host
    r["cpu_list"] = gateway_cpus
    return {"ok": bool(r.get("ok")), "primary": r}


def start_edge_compute() -> Dict[str, Any]:
    """Start backup-Nano edge result collection on its own CPU set."""
    cmd = (
        "python3 -c \"import os,signal; me=os.getpid(); parent=os.getppid(); "
        "[os.kill(int(p), signal.SIGTERM) for p in os.listdir('/proc') if p.isdigit() "
        "and int(p) not in (me,parent) "
        "and 'edge_result_collector.py' in open('/proc/'+p+'/cmdline','rb').read().decode('utf-8','replace')]\" "
        "2>/dev/null || true; "
        "mkdir -p %s; "
        "nohup bash -lc 'taskset -c %s python3 %s/edge_result_collector.py "
        "--role backup --interval 1.0 --out-dir %s' "
        "> /tmp/hil_edge_collector.log 2>&1 < /dev/null & "
        "sleep 1; ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | "
        "grep '[e]dge_result_collector.py' || true; "
        "tail -20 /tmp/hil_edge_collector.log 2>/dev/null || true"
    ) % (EDGE_REMOTE_DIR, BACKUP_EDGE_CPUS, REMOTE_DIR, EDGE_REMOTE_DIR)
    r = _run("backup", cmd, timeout=40)
    r["cpu_list"] = BACKUP_EDGE_CPUS
    return {"ok": bool(r.get("ok")), "backup": r}


def apply_cpu_affinity() -> Dict[str, Any]:
    """Re-apply CPU affinity to already-running HIL processes."""
    def cmd(t: str) -> str:
        if t == "primary":
            return (
                "for p in $(pgrep -f '[A]DAS.py --role primary'); do taskset -pc %s $p; done; "
                "for p in $(pgrep -f '[h]il_ros_gateway.py'); do taskset -pc %s $p; done; "
                "echo __AFFINITY__; "
                "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | "
                "grep -E 'ADAS.py --role primary|hil_ros_gateway.py' | grep -v grep || true"
            ) % (PRIMARY_ADAS_CPUS, PRIMARY_GATEWAY_CPUS)
        return (
            "for p in $(pgrep -f '[A]DAS.py --role backup'); do taskset -pc %s $p; done; "
            "for p in $(pgrep -f '[e]dge_result_collector.py'); do taskset -pc %s $p; done; "
            "echo __AFFINITY__; "
            "ps -o pid,psr,stat,etime,pcpu,pmem,cmd -C python3 | "
            "grep -E 'ADAS.py --role backup|edge_result_collector.py' | grep -v grep || true"
        ) % (BACKUP_ADAS_CPUS, BACKUP_EDGE_CPUS)

    result = _parallel(("primary", "backup"), cmd, timeout=30)
    result["ok"] = all(v.get("ok") for v in result.values())
    result["mapping"] = cpu_mapping()
    return result


def resource_status() -> Dict[str, Any]:
    """Return resource use plus current CPU core/affinity for both Nanos."""
    cmd = (
        "echo __CPU__; nproc; cat /proc/loadavg; "
        "echo __MEM__; free -h; "
        "echo __TEMP__; cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | head -6 || true; "
        "echo __DISK__; df -h / /home 2>/dev/null; "
        "echo __TOP__; ps -eo pid,psr,stat,pcpu,pmem,rss,args --sort=-pcpu | head -18; "
        "echo __AFFINITY__; "
        "for p in $(pgrep -f '[A]DAS.py --role|[h]il_ros_gateway.py|[e]dge_result_collector.py'); do "
        "echo -n \"$p \"; taskset -pc $p 2>/dev/null; done"
    )
    result = _parallel(("primary", "backup"), lambda _t: cmd, timeout=30)
    result["ok"] = all(v.get("ok") for v in result.values())
    result["mapping"] = cpu_mapping()
    return result


def cpu_mapping() -> Dict[str, str]:
    return {
        "primary_125_adas": PRIMARY_ADAS_CPUS,
        "primary_125_gateway": PRIMARY_GATEWAY_CPUS,
        "backup_124_adas": BACKUP_ADAS_CPUS,
        "backup_124_edge": BACKUP_EDGE_CPUS,
    }


def sync_edge_results(limit: int = 20) -> Dict[str, Any]:
    """Pull recent backup-Nano edge JSON files into HIL/hil_platform/edge_results."""
    EDGE_LOCAL_DIR.mkdir(parents=True, exist_ok=True)
    target_dir = EDGE_LOCAL_DIR / "backup_124"
    target_dir.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    c = _client("backup")
    pulled: List[str] = []
    try:
        sftp = c.open_sftp()
        try:
            try:
                entries = sftp.listdir_attr(EDGE_REMOTE_DIR)
            except IOError:
                return {
                    "ok": False,
                    "target": "backup",
                    "remote_dir": EDGE_REMOTE_DIR,
                    "local_dir": str(target_dir),
                    "pulled": [],
                    "stderr": "remote edge result directory is missing",
                }
            files = sorted(
                [e for e in entries if e.filename.endswith((".json", ".jsonl"))],
                key=lambda e: e.st_mtime,
                reverse=True,
            )[:max(1, limit)]
            for e in files:
                remote = "%s/%s" % (EDGE_REMOTE_DIR, e.filename)
                local = target_dir / e.filename
                sftp.get(remote, str(local))
                pulled.append(str(local))
        finally:
            sftp.close()
    finally:
        c.close()
    return {
        "ok": True,
        "target": "backup",
        "remote_dir": EDGE_REMOTE_DIR,
        "local_dir": str(target_dir),
        "pulled": pulled,
        "elapsed_ms": int((time.monotonic() - started) * 1000),
    }


def prepare_hil(source: str = "esp32", deploy: bool = True, carla: bool = True) -> Dict[str, Any]:
    """Prepare the real CARLA + Nano path for WebUI-driven simulation."""
    steps: Dict[str, Any] = {}
    if deploy:
        steps["deploy_gateway"] = deploy_gateway()
        if not steps["deploy_gateway"].get("ok"):
            return {"ok": False, "failed_step": "deploy_gateway", "steps": steps}
        steps["deploy_adas_code"] = deploy_adas_code()
        if not steps["deploy_adas_code"].get("ok"):
            return {"ok": False, "failed_step": "deploy_adas_code", "steps": steps}
    steps["stop_perception_sim"] = stop_perception_sim()
    if not steps["stop_perception_sim"].get("ok"):
        return {"ok": False, "failed_step": "stop_perception_sim", "steps": steps}
    steps["restart_adas"] = restart_adas("both")
    if not steps["restart_adas"].get("ok"):
        return {"ok": False, "failed_step": "restart_adas", "steps": steps}
    if carla:
        steps["start_carla"] = start_carla(int(os.environ.get("CARLA_PORT", "2000")))
        if not steps["start_carla"].get("ok"):
            return {"ok": False, "failed_step": "start_carla", "steps": steps}
    steps["start_edge_compute"] = start_edge_compute()
    if not steps["start_edge_compute"].get("ok"):
        return {"ok": False, "failed_step": "start_edge_compute", "steps": steps}
    steps["start_gateway"] = start_gateway(source)
    if not steps["start_gateway"].get("ok"):
        return {"ok": False, "failed_step": "start_gateway", "steps": steps}
    steps["health"] = health()
    return {"ok": bool(steps["health"].get("ok")), "steps": steps}


def restore_nanos() -> Dict[str, Any]:
    def cmd(t: str) -> str:
        role = _targets()[t]["role"]
        return "pkill -CONT -f 'ADAS.py --role %s' || true; ps -o pid,stat,etime,cmd -C python3 | grep 'ADAS.py --role' | grep -v grep || true" % role

    result = _parallel(("primary", "backup"), cmd, timeout=30)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def _select_targets(target: str) -> Tuple[str, ...]:
    if target in ("both", "all"):
        return ("primary", "backup")
    if target in ("primary", "backup"):
        return (target,)
    if target == "nano_a":
        return ("primary",)
    if target == "nano_b":
        return ("backup",)
    raise ValueError("target must be primary, backup, both, nano_a, or nano_b")
