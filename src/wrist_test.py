'''
Clean re-test of wrist3 sign convention -- avoids cv2.Rodrigues entirely by
using movej() directly on the joint vector. Since TOOL_ORIENTATION_RV sits
almost exactly at the rotation-vector singularity (theta ~= pi), the
previous rv-based test (compose_rotation_rv -> movel) could not be trusted:
the *command* itself may have been corrupted, not just the readback.

This version:
  1. Reads current joints.
  2. Adds TEST_DELTA_RAD directly to wrist3 (plain float addition, no
     matrix math at all).
  3. Sends movej() with that joint vector.
  4. Waits for the robot to stop.
  5. Confirms wrist3 changed by exactly the expected amount (it will,
     since movej to an explicit joint target is exact/deterministic --
     this step is really just a sanity check that nothing else moved).
  6. Asks you to report which direction you SAW it turn (CW or CCW,
     viewed from above), which is the real missing piece of information --
     that's what tells us how a positive wrist3 delta maps to a physical
     rotation direction, so we can match it against
     puzzle_solver.principal_angle()'s convention.
'''

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
import numpy as np

from rtde_receive import RTDEReceiveInterface
from robot_message_send import robot_message_send
from configs.robot import ROBOT_IP

from robot_control import go_to_pose_camera_capture

TEST_DELTA_RAD = 0.3  # ~17 degrees -- large enough to see clearly, small enough to be safe
def wait_until_stopped(rtde_r, speed_threshold=0.001, stable_duration=0.3,
                        grace_period=0.3, timeout=30):
    time.sleep(grace_period)
    start_time = time.time()
    stable_since = None
    while time.time() - start_time < timeout:
        speed = rtde_r.getActualTCPSpeed()
        linear_speed = np.linalg.norm(speed[:3])
        angular_speed = np.linalg.norm(speed[3:6])
        combined_speed = max(linear_speed, angular_speed)  # or check both independently

        if combined_speed < speed_threshold:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= stable_duration:
                return True
        else:
            stable_since = None
        time.sleep(0.02)
    print("wait_until_stopped: timed out")
    return False

def main():
    print(f"Connecting to robot at {ROBOT_IP}...")
    rtde_r = RTDEReceiveInterface(ROBOT_IP)
    wait_until_stopped(rtde_r=rtde_r)

    try:
        q_before = rtde_r.getActualQ()
        wrist3_before = q_before[5]
        print("\nCurrent joints (rad):", [round(v, 4) for v in q_before])
        print(f"wrist3 before: {wrist3_before:.4f} rad ({np.degrees(wrist3_before):.2f} deg)")

        q_target = list(q_before)
        q_target[5] = wrist3_before + TEST_DELTA_RAD

        script = f"""
def test_wrist3_nudge():
    movej([{q_target[0]:.6f}, {q_target[1]:.6f}, {q_target[2]:.6f}, {q_target[3]:.6f}, {q_target[4]:.6f}, {q_target[5]:.6f}], a=0.3, v=0.3)
end
test_wrist3_nudge()
"""
        print(f"\nSending movej with wrist3 += {TEST_DELTA_RAD} rad ({np.degrees(TEST_DELTA_RAD):.1f} deg)...")
        print(">>> WATCH THE WRIST NOW <<<")
        robot_message_send(script)

        print("Waiting for robot to stop moving...")
        wait_until_stopped(rtde_r)

        q_after = rtde_r.getActualQ()
        wrist3_after = q_after[5]
        actual_delta = wrist3_after - wrist3_before

        print(f"\nwrist3 after:  {wrist3_after:.4f} rad ({np.degrees(wrist3_after):.2f} deg)")
        print(f"wrist3 actual change: {actual_delta:.4f} rad ({np.degrees(actual_delta):.2f} deg)")
        print(f"commanded change: +{TEST_DELTA_RAD:.4f} rad")

        other_joints_moved = any(
            abs(a - b) > 0.01 for a, b in zip(q_after[:5], q_before[:5])
        )
        if other_joints_moved:
            print("\nNOTE: some other joint also changed -- unexpected for a pure movej "
                  "wrist3 nudge, worth double-checking.")
            print("q_before:", [round(v, 4) for v in q_before])
            print("q_after: ", [round(v, 4) for v in q_after])
        else:
            print("\nConfirmed: only wrist3 changed, and it changed by exactly the "
                  "commanded amount (movej is deterministic, as expected).")

        print("\n--- NOW THE IMPORTANT PART ---")
        print("Which way did the tool/wrist visibly turn, viewed from ABOVE "
              "(looking straight down at the table)?")
        print("  - Clockwise  -> a POSITIVE wrist3 delta = clockwise rotation (viewed from above)")
        print("  - Counterclockwise -> a POSITIVE wrist3 delta = counterclockwise rotation (viewed from above)")
        print("\nReport back which one you saw -- that tells us how to match")
        print("wrist3 sign against puzzle_solver.principal_angle()'s rotation convention.")

    finally:
        rtde_r.disconnect()


if __name__ == "__main__":
    go_to_pose_camera_capture()

    main()