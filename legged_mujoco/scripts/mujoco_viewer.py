#!/usr/bin/env python3
import rospy
import mujoco
import mujoco.viewer
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import threading

# ROS joint names (mujoco_params.yaml "joints" list) -> MuJoCo MJCF joint names.
# Must match the mapping in legged_mujoco/src/LeggedHWMujoco.cpp's setupJoints().
ROS_TO_MUJOCO_JOINT = {
    "LF_HAA": "FL_hip_joint",
    "LF_HFE": "FL_thigh_joint",
    "LF_KFE": "FL_calf_joint",
    "RF_HAA": "FR_hip_joint",
    "RF_HFE": "FR_thigh_joint",
    "RF_KFE": "FR_calf_joint",
    "LH_HAA": "RL_hip_joint",
    "LH_HFE": "RL_thigh_joint",
    "LH_KFE": "RL_calf_joint",
    "RH_HAA": "RR_hip_joint",
    "RH_HFE": "RR_thigh_joint",
    "RH_KFE": "RR_calf_joint",
}

class MuJoCoViewer:
    def __init__(self):
        rospy.init_node('mujoco_viewer')

        # Load model
        model_path = rospy.get_param('~mujoco_model_path',
            '/home/catkin_ws/src/legged_control/legged_mujoco/models/unitree_a1/scene.xml')

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Joint name to ID mapping
        self.joint_map = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_map[name] = i

        self.viewer = None

        # The free joint (floating base) is always qpos[0:7] - this model
        # has exactly one, declared first in a1.xml.
        self.base_qpos_adr = 0

        # ROS callbacks only stash the latest values here (cheap dict/tuple
        # writes, no mujoco calls, no lock) - run() applies them to self.data
        # and calls mj_forward() exactly once per render frame. Previously
        # each callback independently locked+wrote+mj_forward'd at its own
        # ROS rate (joint_states ~100Hz, ground_truth at the full 1000Hz
        # control rate); since they're two unsynchronized callbacks, the
        # render thread could catch self.data between a base update and a
        # joint update, painting one frame with a new base position but
        # stale leg angles (or vice versa) - looked exactly like the model
        # clipping into the floor, despite the real simulation being fine.
        self._lock = threading.Lock()
        self._latest_joint_pos = {}  # mj_name -> position
        self._latest_base = None  # (x,y,z, qw,qx,qy,qz) or None until first message

        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/ground_truth/state', Odometry, self.ground_truth_callback)

        rospy.loginfo("MuJoCo Viewer initialized")
        rospy.loginfo(f"Joints: {list(self.joint_map.keys())}")

    def joint_state_callback(self, msg):
        with self._lock:
            for ros_name, pos in zip(msg.name, msg.position):
                # /joint_states uses ROS names (e.g. "LF_KFE"), but this
                # model's joints are named in MuJoCo/Unitree convention
                # (e.g. "FL_calf_joint") - they never matched directly.
                mj_name = ROS_TO_MUJOCO_JOINT.get(ros_name, ros_name)
                if mj_name in self.joint_map:
                    self._latest_joint_pos[mj_name] = pos

    def ground_truth_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            # ROS geometry_msgs/Quaternion is (x,y,z,w); MuJoCo qpos quat is (w,x,y,z).
            self._latest_base = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)

    def _apply_latest(self):
        """Write the latest stashed joint/base values into self.data as one
        atomic update, then mj_forward once. Must hold self.viewer.lock()."""
        with self._lock:
            joint_pos = dict(self._latest_joint_pos)
            base = self._latest_base
        for mj_name, pos in joint_pos.items():
            qpos_adr = self.model.jnt_qposadr[self.joint_map[mj_name]]
            self.data.qpos[qpos_adr] = pos
        if base is not None:
            a = self.base_qpos_adr
            self.data.qpos[a:a + 7] = base
        if joint_pos or base is not None:
            mujoco.mj_forward(self.model, self.data)

    def run(self):
        """Launch viewer"""
        # Mirror-only viewer: state comes from ROS, so we must not call
        # mj_step here or it'd integrate its own competing dynamics.
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            while viewer.is_running() and not rospy.is_shutdown():
                # viewer.lock() is required: launch_passive's render thread
                # reads self.data concurrently, and mutating it unlocked
                # segfaults libmujoco.
                with viewer.lock():
                    self._apply_latest()
                viewer.sync()
                rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        viewer = MuJoCoViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
