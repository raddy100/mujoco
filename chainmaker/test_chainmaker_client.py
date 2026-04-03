"""
ChainMaker IPC Client
=====================
Connects to a running ChainMaker instance (launched with --test flag) on
localhost:47832 and sends/receives newline-delimited JSON commands.

Usage:
    from test_chainmaker_client import ChainMakerClient

    with ChainMakerClient() as c:
        state = c.get_state()
        c.place_block()
        c.screenshot("frame.ppm")
        c.quit()
"""

import socket
import json
import subprocess
import time
import os
from typing import Optional, Dict, Any

IPC_HOST = "127.0.0.1"
IPC_PORT = 47832
DEFAULT_EXE = r"C:\Users\PC\source\repos\mujoco\build\bin\Release\ChainMaker.exe"


class ChainMakerClient:
    """
    TCP client for the ChainMaker IPC test server.

    Can either connect to an already-running instance, or launch one itself.
    Always use as a context manager (with statement) so cleanup is guaranteed.
    """

    def __init__(self, host: str = IPC_HOST, port: int = IPC_PORT,
                 timeout: float = 10.0):
        self.host    = host
        self.port    = port
        self.timeout = timeout
        self._sock   = None
        self._buf    = b""
        self._proc: Optional[subprocess.Popen] = None

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def launch_and_connect(self, exe_path: str = DEFAULT_EXE,
                           wait: float = 3.0) -> "ChainMakerClient":
        """Launch ChainMaker with --test flag and then connect."""
        if not os.path.exists(exe_path):
            raise FileNotFoundError(f"ChainMaker not found: {exe_path}")
        self._proc = subprocess.Popen(
            [exe_path, "--test"],
            cwd=os.path.dirname(exe_path),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        time.sleep(wait)
        self.connect()
        return self

    def connect(self) -> None:
        """Connect to an already-running ChainMaker --test instance."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.host, self.port))

    def close(self) -> None:
        """Disconnect and optionally terminate the launched process."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None

    # ------------------------------------------------------------------
    # Low-level send/recv
    # ------------------------------------------------------------------

    def _send(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Send one JSON command and return the parsed response."""
        payload = json.dumps(cmd) + "\n"
        self._sock.sendall(payload.encode("utf-8"))

        # Read until newline (response delimiter)
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("Server closed connection")
            self._buf += chunk

        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    # ------------------------------------------------------------------
    # High-level command wrappers
    # ------------------------------------------------------------------

    def place_block(self) -> Dict:
        """Place a block at the current head position in the active chain."""
        return self._send({"cmd": "place_block"})

    def send(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Send an arbitrary command dict (uses 'command' key convention for custom IPC)."""
        return self._send(cmd)

    def delete_block(self) -> Dict:
        """Delete the most recently placed block in the active chain."""
        return self._send({"cmd": "delete_block"})

    def set_direction(self, direction: str) -> Dict:
        """
        Set the placement direction for the active chain.
        direction: one of "POS_X","-X","NEG_X","+X","POS_Y","-Y",...
        """
        return self._send({"cmd": "set_direction", "dir": direction})

    def get_state(self) -> Dict:
        """
        Return the full application state as a dict:
          mode, nchains, active_chain, nblocks, ngrid, head {x,y,z},
          direction, bead_size, gap_ratio
        """
        return self._send({"cmd": "get_state"})

    def switch_chain(self, chain_id: int) -> Dict:
        """Make chain_id the active chain (0-indexed)."""
        return self._send({"cmd": "switch_chain", "id": chain_id})

    def new_chain_mode(self) -> Dict:
        """Enter the New-Chain-Pick mode (equivalent to pressing N)."""
        return self._send({"cmd": "new_chain_mode"})

    def start_chain_at(self, x: int, y: int, z: int) -> Dict:
        """
        Start a new chain branching from the block at grid position (x, y, z).
        Automatically enters and exits NEW_CHAIN_PICK mode.
        Returns {"ok": true, "chain_id": N} on success.
        """
        return self._send({"cmd": "start_chain_at", "x": x, "y": y, "z": z})

    def cancel_pick(self) -> Dict:
        """Cancel the New-Chain-Pick mode and return to BUILD."""
        return self._send({"cmd": "cancel_pick"})

    def adjust_gap(self, delta: float) -> Dict:
        """Adjust the gap ratio between beads by delta."""
        return self._send({"cmd": "adjust_gap", "delta": delta})

    def set_bead_size(self, size: float) -> Dict:
        """Set the bead (block) edge length in metres."""
        return self._send({"cmd": "set_bead_size", "size": size})

    def screenshot(self, path: str) -> Dict:
        """
        Capture the current rendered frame to a PPM image file.
        path: absolute or relative path for the output file.
        """
        return self._send({"cmd": "screenshot", "path": path})

    def enter_simulate(self) -> Dict:
        """Compile the current world and enter physics simulation."""
        return self._send({"cmd": "enter_simulate"})

    def exit_simulate(self) -> Dict:
        """Exit simulation and return to the build stage."""
        return self._send({"cmd": "exit_simulate"})

    def get_profiler(self) -> Dict:
        """Return per-step timer values (ms) from MuJoCo d->timer[].
        Only valid in SIMULATE mode. Fields: step_ms, collision_ms, solver_ms,
        kinematics_ms, inertia_ms, ncon, nbody, nv, iterations,
        sim_preset (int), sim_preset_label (str)."""
        return self._send({"cmd": "get_profiler"})

    def set_sim_preset(self, preset: int) -> Dict:
        """Set the simulation speed/accuracy preset.
        0 = Accurate (Newton + box, slowest, most accurate)
        1 = Balanced (CG + box, ~5x faster, same geometry)
        2 = Fast     (CG + sphere, ~11x faster, inscribed sphere)
        If simulation is running, applies immediately. Safe to call in build mode
        (stores the preference, applied when simulation starts)."""
        return self._send({"cmd": "set_sim_preset", "preset": preset})

    def start_recording(self, path: str = "") -> Dict:
        """Start recording video to an MP4 file via ffmpeg.
        path: output file path (default: auto-generated timestamped name).
        Returns {"ok": true, "path": "..."} on success."""
        payload: Dict = {"cmd": "start_recording"}
        if path:
            payload["path"] = path
        return self._send(payload)

    def stop_recording(self) -> Dict:
        """Stop an active recording and finalize the MP4 file."""
        return self._send({"cmd": "stop_recording"})

    def get_recording_status(self) -> Dict:
        """Return current recording state.
        Fields: is_recording, path (if recording), width, height."""
        return self._send({"cmd": "get_recording_status"})

    def save(self, path: str) -> Dict:
        """Save the current chain world to a JSON file."""
        return self._send({"cmd": "save", "path": path})

    def load(self, path: str) -> Dict:
        """Load a chain world from a JSON file."""
        return self._send({"cmd": "load", "path": path})

    def reset(self) -> Dict:
        """Reset to a fresh single-chain world."""
        return self._send({"cmd": "reset"})

    def move_camera(self, action: str, dx: float = 0.0,
                    dy: float = 0.0) -> Dict:
        """
        Execute a camera move on the main GL thread.
        action: "rotate_v" | "rotate_h" | "move_v" | "move_h" | "zoom"
        dx, dy: normalised delta (fraction of window height).
        Returns {"ok": true, "azimuth": ..., "elevation": ..., "distance": ...}
        """
        return self._send({"cmd": "move_camera", "action": action,
                           "dx": dx, "dy": dy})

    def quit(self) -> Dict:
        """Signal ChainMaker to close its window and exit."""
        return self._send({"cmd": "quit"})


# ---------------------------------------------------------------------------
# Convenience: launch + connect in one call
# ---------------------------------------------------------------------------

def launch_chainmaker(exe_path: str = DEFAULT_EXE,
                      wait: float = 3.0) -> ChainMakerClient:
    """Launch ChainMaker with --test and return a connected client."""
    client = ChainMakerClient()
    client.launch_and_connect(exe_path=exe_path, wait=wait)
    return client
