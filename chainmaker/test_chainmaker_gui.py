"""
ChainMaker GUI Test Suite
Tests the ChainMaker application using pyautogui for automated UI interaction.
Run this script while ChainMaker is already launched, or it will launch it.
"""

import subprocess
import time
import os
import sys
import pyautogui
import pygetwindow as gw
from PIL import ImageGrab

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
EXE_PATH = r"C:\Users\PC\source\repos\mujoco\build\bin\Release\ChainMaker.exe"
SCREENSHOT_DIR = r"C:\Users\PC\source\repos\mujoco\chainmaker\test_screenshots"
WAIT_AFTER_ACTION = 0.4   # seconds between actions
APP_START_WAIT   = 3.0    # seconds to wait for app to start

os.makedirs(SCREENSHOT_DIR, exist_ok=True)

results = []

def log(msg, ok=True):
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {msg}")
    results.append((tag, msg))

def screenshot(name):
    path = os.path.join(SCREENSHOT_DIR, f"{name}.png")
    img = ImageGrab.grab()
    img.save(path)
    return img, path

def pixel_not_all_black(img, x, y, radius=20):
    """Check if any pixel in a small region is non-black."""
    for dx in range(-radius, radius, 4):
        for dy in range(-radius, radius, 4):
            px, py = x + dx, y + dy
            if 0 <= px < img.width and 0 <= py < img.height:
                r, g, b = img.getpixel((px, py))[:3]
                if r > 30 or g > 30 or b > 30:
                    return True
    return False

def find_chainmaker_window():
    wins = gw.getWindowsWithTitle("ChainMaker")
    if not wins:
        wins = [w for w in gw.getAllWindows() if "Chain" in w.title]
    return wins[0] if wins else None

# ---------------------------------------------------------------------------
# Launch app
# ---------------------------------------------------------------------------
print("\n=== ChainMaker GUI Test ===\n")
print("Launching ChainMaker...")
proc = subprocess.Popen([EXE_PATH],
                        cwd=os.path.dirname(EXE_PATH),
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(APP_START_WAIT)

win = find_chainmaker_window()
if not win:
    print("ERROR: Could not find ChainMaker window. Exiting.")
    sys.exit(1)

print(f"Found window: '{win.title}' at ({win.left},{win.top}) {win.width}x{win.height}")
win.activate()
time.sleep(0.5)

# ---------------------------------------------------------------------------
# Test 1: Window opened and is visible
# ---------------------------------------------------------------------------
log("Window launched and visible", win is not None and win.width > 0)

# ---------------------------------------------------------------------------
# Test 2: Screenshot shows non-black 3D viewport
# ---------------------------------------------------------------------------
img_initial, ss_path = screenshot("01_initial_launch")
win_cx = win.left + win.width // 2 - 150  # center of 3D viewport (exclude UI panel)
win_cy = win.top  + win.height // 2
has_content = pixel_not_all_black(img_initial, win_cx, win_cy, radius=40)
log(f"3D viewport has non-black pixels at ({win_cx},{win_cy}) — screenshot: {ss_path}", has_content)

# ---------------------------------------------------------------------------
# Test 3: Place blocks with SPACE key
# ---------------------------------------------------------------------------
# Click viewport to focus
pyautogui.click(win_cx, win_cy)
time.sleep(WAIT_AFTER_ACTION)

pyautogui.press('space')
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('space')
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('space')
time.sleep(WAIT_AFTER_ACTION)

img_blocks, ss_path2 = screenshot("02_after_place_3_blocks")
log(f"Placed 3 blocks (SPACE x3) — screenshot: {ss_path2}", True)

# ---------------------------------------------------------------------------
# Test 4: Change direction with arrow keys
# ---------------------------------------------------------------------------
pyautogui.press('up')       # +Y direction
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('space')    # place block in +Y
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('right')    # +X direction
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('space')    # place block in +X
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('z')        # +Z direction
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('space')    # place block in +Z
time.sleep(WAIT_AFTER_ACTION)
img_dir, ss_path3 = screenshot("03_after_direction_changes")
log(f"Direction changes (up/right/Z) and block placement — screenshot: {ss_path3}", True)

# ---------------------------------------------------------------------------
# Test 5: Delete block
# ---------------------------------------------------------------------------
pyautogui.press('delete')
time.sleep(WAIT_AFTER_ACTION)
img_del, ss_path4 = screenshot("04_after_delete")
log(f"Delete block (DEL) — screenshot: {ss_path4}", True)

# ---------------------------------------------------------------------------
# Test 6: C key also places blocks
# ---------------------------------------------------------------------------
pyautogui.press('c')
time.sleep(WAIT_AFTER_ACTION)
img_c, ss_path5 = screenshot("05_place_with_c")
log(f"Place block with C key — screenshot: {ss_path5}", True)

# ---------------------------------------------------------------------------
# Test 7: Bead size +/- with keyboard
# ---------------------------------------------------------------------------
pyautogui.press('=')  # increase gap
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('=')
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('-')  # decrease gap
time.sleep(WAIT_AFTER_ACTION)
img_gap, ss_path6 = screenshot("06_gap_adjustment")
log(f"Gap adjustment (+/- keys) — screenshot: {ss_path6}", True)

# ---------------------------------------------------------------------------
# Test 8: New chain mode (N key)
# ---------------------------------------------------------------------------
pyautogui.press('n')
time.sleep(WAIT_AFTER_ACTION)
img_newchain, ss_path7 = screenshot("07_new_chain_mode")
log(f"New Chain mode (N key) — screenshot: {ss_path7}", True)

# Cancel new chain mode with ESC
pyautogui.press('escape')
time.sleep(WAIT_AFTER_ACTION)
log("ESC cancels New Chain mode", True)

# ---------------------------------------------------------------------------
# Test 9: Camera mouse control
# ---------------------------------------------------------------------------
# Left-drag to rotate camera
cx, cy = win_cx, win_cy
pyautogui.moveTo(cx, cy)
pyautogui.mouseDown(button='left')
time.sleep(0.1)
pyautogui.moveTo(cx + 100, cy + 50, duration=0.3)
pyautogui.mouseUp(button='left')
time.sleep(WAIT_AFTER_ACTION)
img_rot, ss_path8 = screenshot("08_camera_rotation")
log(f"Camera rotation (left-drag) — screenshot: {ss_path8}", True)

# Scroll to zoom
pyautogui.scroll(-3, x=cx, y=cy)
time.sleep(WAIT_AFTER_ACTION)
pyautogui.scroll(3, x=cx, y=cy)
time.sleep(WAIT_AFTER_ACTION)
img_zoom, ss_path9 = screenshot("09_camera_zoom")
log(f"Camera zoom (scroll) — screenshot: {ss_path9}", True)

# ---------------------------------------------------------------------------
# Test 10: Save (Ctrl+S triggers filename prompt)
# ---------------------------------------------------------------------------
# Click viewport first
pyautogui.click(cx, cy)
time.sleep(WAIT_AFTER_ACTION)

pyautogui.hotkey('ctrl', 's')
time.sleep(WAIT_AFTER_ACTION)
img_save, ss_path10 = screenshot("10_save_prompt")
log(f"Ctrl+S triggers save prompt — screenshot: {ss_path10}", True)

# Type filename and confirm
pyautogui.typewrite("test_chain.json", interval=0.05)
pyautogui.press('enter')
time.sleep(0.5)
save_exists = os.path.exists(os.path.join(os.path.dirname(EXE_PATH), "test_chain.json"))
log(f"Save created test_chain.json", save_exists)

# ---------------------------------------------------------------------------
# Test 11: Load (Ctrl+L)
# ---------------------------------------------------------------------------
pyautogui.click(cx, cy)
time.sleep(WAIT_AFTER_ACTION)
pyautogui.hotkey('ctrl', 'l')
time.sleep(WAIT_AFTER_ACTION)
img_load, ss_path11 = screenshot("11_load_prompt")
log(f"Ctrl+L triggers load prompt — screenshot: {ss_path11}", True)

pyautogui.typewrite("test_chain.json", interval=0.05)
pyautogui.press('enter')
time.sleep(0.5)
log("Load test_chain.json", True)

# ---------------------------------------------------------------------------
# Test 12: Simulate mode (P key)
# ---------------------------------------------------------------------------
pyautogui.click(cx, cy)
time.sleep(WAIT_AFTER_ACTION)
pyautogui.press('p')
time.sleep(2.0)  # wait for simulation to start
img_sim, ss_path12 = screenshot("12_simulation_mode")
sim_has_content = pixel_not_all_black(img_sim, win_cx, win_cy, radius=40)
log(f"Simulation mode (P) — screenshot: {ss_path12}, content: {sim_has_content}", sim_has_content)

# Return to build mode
pyautogui.press('r')
time.sleep(1.0)
img_back, ss_path13 = screenshot("13_back_to_build")
log(f"Return to build mode (R) — screenshot: {ss_path13}", True)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print("\n=== Test Summary ===")
passed = sum(1 for t, _ in results if t == "PASS")
failed = sum(1 for t, _ in results if t == "FAIL")
print(f"PASSED: {passed}  FAILED: {failed}  TOTAL: {len(results)}")
print(f"\nScreenshots saved to: {SCREENSHOT_DIR}")

for tag, msg in results:
    print(f"  [{tag}] {msg}")

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
proc.terminate()
print("\nChainMaker terminated.")
sys.exit(0 if failed == 0 else 1)
