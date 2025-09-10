#!/usr/bin/env python2
from graspit_process import GraspitProcess
from geometry_msgs.msg import Pose
import time
import sys

def pose_mm(x=0, y=0, z=0, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Return a geometry_msgs/Pose in *millimeters* (GraspIt units)."""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.w = float(qw)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    return p

def main():
    # --- Config ---
    # Use your *scaled* robot if you have one (e.g., 'handright9253_mm').
    robot = 'fetch_gripper'
    body  = '003_cracker_box_google_16k_textured_scale_1000.off'

    process = GraspitProcess()  # respects GRASPIT/GRASPIT_PLUGIN_DIR envs
    try:
        process.start()
        g = process.graspit
        if g is None:
            print("Error: GraspIt process failed to start.")
            return 1

        # --- Build scene deterministically ---
        g.clearWorld()
        g.importRobot(robot)
        # Place robot at origin (mm)
        g.setRobotPose(pose_mm(0, 0, 0))

        # Import object and move it +80 mm in front of the hand
        g.importGraspableBody(body)
        body_ids = g.getBodies().ids
        body_id  = body_ids[-1]  # last imported is the object
        try:
            # Some interfaces expose setBodyPose; if yours doesn't, comment this
            g.setBodyPose(body_id, pose_mm(80, 0, 0))
        except Exception as e:
            # If setBodyPose isn't available in your binding, the object will stay at origin
            pass

        # Optional: give the UI a moment if not headless
        time.sleep(0.25)
        raw_input("Press <Enter> to continue...")

        # --- Ensure collisions ON before approaching ---
        g.toggleAllCollisions(True)

        # --- Coarse approach loop to ensure contact ---
        # 2000 mm = 2 m. Try a few times in case we start far.
        try:
            for _ in range(3):
                g.approachToContact(moveDist=2000, oneStep=False)
                robot_state = g.getRobot().robot
                if getattr(robot_state, 'contacts', []):
                    break
        except Exception as e:
            # approach may fail if already in contact; that's fine
            pass

        # --- AutoGrasp ---
        try:
            g.autoGrasp()
        except Exception as e:
            print("autoGrasp raised:", e)

        # --- Quality & contacts ---
        try:
            q = g.computeQuality()
            print("[Quality] result={}, volume={:.6f}, epsilon={:.6f}".format(
                q.result, getattr(q, 'volume', float('nan')), getattr(q, 'epsilon', float('nan'))
            ))
        except Exception as e:
            print("computeQuality failed:", e)

        try:
            robot_state = g.getRobot().robot
            num_contacts = len(getattr(robot_state, 'contacts', []))
            print("[Contacts] count={}".format(num_contacts))
        except Exception as e:
            print("getRobot/contacts failed:", e)

        # Keep the scene up for inspection if running with a viewer
        raw_input("Press <Enter> to exit...")

    finally:
        try:
            process.shutdown()
        except Exception:
            # Older wrapper: GraspitProcess.join() in __exit__/__del__ will clean up
            pass

    return 0

if __name__ == "__main__":
    sys.exit(main())
