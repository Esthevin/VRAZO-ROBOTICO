#!/usr/bin/env python3
"""
full_control_node.py

Nodo ROS2 único que:
- Ejecuta control cinemático (cmd_xyz_position) usando jacobiano.
- Ejecuta control PD articular (cmd_joint_angles) usando lecturas reales.
- Escucha lecturas reales desde ESP32 en /servo_angles (Int32MultiArray).
- Publica comandos al ESP32 en /servo_commands (Int32MultiArray, grados).
- Publica joint_states para RViz.
- Calcula y muestra errores angulares y de posición (FK).
- Aplica offset a lecturas: [0, -90, -90, -90, 0, 0] (grados).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import math
from copy import copy

# Visualization helpers (BallMarker / FrameMarker) included (same que ya tenías)
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker

# -------------------- Markers (copiado de tu código) -------------------- #
class BallMarker(object):
    id = 0
    def __init__(self, node, color, alpha=1.0, scale=0.05):
        reference_frame = node.get_parameter('reference_frame').value
        self.marker_pub = node.create_publisher(Marker, "visualization_marker", 10)
        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.ns = "ball_markers"
        self.marker.id = BallMarker.id
        BallMarker.id += 1
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.setColor(color, alpha)
        self.marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
    def setColor(self, color, alpha=1.0):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = alpha
    def position(self, T):
        self.marker.pose.position.x = T[0,3]
        self.marker.pose.position.y = T[1,3]
        self.marker.pose.position.z = T[2,3]
        self.publish()
    def xyz(self, position):
        self.marker.pose.position.x = float(position[0])
        self.marker.pose.position.y = float(position[1])
        self.marker.pose.position.z = float(position[2])
        self.publish()
    def publish(self):
        self.marker_pub.publish(self.marker)

color = dict()
color['RED'] = (1.0, 0.0, 0.0)
color['BLUE'] = (0.0, 0.0, 1.0)
# FrameMarker truncated to minimal functionality needed for publishing frame arrows
class FrameMarker(object):
    id = 0
    def __init__(self, node, color_saturation=1.0, alpha=1.0, scale=0.1):
        reference_frame = node.get_parameter('reference_frame').value
        self.marker_pub = node.create_publisher(Marker, "visualization_marker", 10)
        self.markerx = Marker(); self.markery = Marker(); self.markerz = Marker()
        for m in (self.markerx, self.markery, self.markerz):
            m.header.frame_id = reference_frame
            m.ns = "frame_markers"
            m.type = m.ARROW
            m.action = m.ADD
            m.scale.y = 0.025; m.scale.z = 0.025
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.markerx.scale.x = scale
        self.markery.scale.x = scale
        self.markerz.scale.x = scale
        self.markerx.color.r = color_saturation; self.markerx.color.a = alpha
        self.markery.color.g = color_saturation; self.markery.color.a = alpha
        self.markerz.color.b = color_saturation; self.markerz.color.a = alpha
    def setPose(self, pose):
        # pose: [x,y,z, qw, qx, qy, qz] or [x,y,z]
        px,py,pz = pose[0], pose[1], pose[2]
        for m in (self.markerx, self.markery, self.markerz):
            m.pose.position.x = px; m.pose.position.y = py; m.pose.position.z = pz
        # orientation handling skipped for concise view
        self.publish()
    def publish(self):
        self.marker_pub.publish(self.markerx)
        self.marker_pub.publish(self.markery)
        self.marker_pub.publish(self.markerz)

def quaternionMult(q1, q2):
    quat = 4*[0.,]
    quat[0] = -q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]+q1[0]*q2[0]
    quat[1] =  q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3]+q1[1]*q2[0]
    quat[2] =  q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]
    quat[3] = -q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3]+q1[3]*q2[0]
    return np.array(quat)

# -------------------- Kinematics functions (copied/compatible) -------------------- #
cos = np.cos; sin = np.sin; pi = np.pi

def dh(d, theta, a, alpha):
    sth = np.sin(theta); cth = np.cos(theta)
    sa  = np.sin(alpha); ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T

def fkine_ur5(q):
    # expects q length >=5, q in radians
    l1v=0.6975; l2v=1.0074; l3v=1.2915; l4v=0; l5v=0.745
    T1 = dh(l1v, q[0], -0.125, -pi/2)
    T2 = dh(0, q[1]-pi/2, l2v, 0)
    T3 = dh(0, q[2], l3v, 0)
    T4 = dh(-0.1014, q[3], 0, pi/2)
    T5 = dh(l5v, q[4], 0, 0)
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5)
    return T

def jacobian_position(q, delta=1e-4):
    n = 5
    J = np.zeros((3,n))
    x = fkine_ur5(q)
    for i in range(n):
        dq = copy(q)
        dq[i] += delta
        dx = fkine_ur5(dq)
        colum_i = 1.0/delta * (dx[0:3,3] - x[0:3,3])
        J[:,i] = colum_i
    return J

def rot2quat(R):
    dEpsilon = 1e-6
    quat = 4*[0.,]
    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)
    return np.array(quat)

def TF2xyzquat(T):
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)

# -------------------- Main Node -------------------- #
class FullControlNode(Node):
    def __init__(self):
        super().__init__('full_control_node')

        # parameter for markers
        self.declare_parameter('reference_frame', 'base_link')
        self.ref_frame = self.get_parameter('reference_frame').value

        # joints
        self.jnames = ['joint1','joint2','joint3','joint4','joint5','joint6']
        # internal model state q (radians). this represents the "model" used for FK and sending commands
        self.q = np.zeros(6, dtype=float)

        # feedback from ESP32 (angles in degrees before conversion)
        self.q_feedback_deg = None   # raw from ESP32 (will apply offset)
        # q measured in radians (after offset)
        self.q_measured = None

        # commands requested externally
        self.q_cmd_angles_deg = None  # desired joint angles (Int32MultiArray, degrees) for PD mode
        self.cmd_xyz = None           # desired position Point for kinematic (x,y,z) mode

        # offsets (degrees) to apply to readings from ESP32
        self.offset_deg = np.array([0., 90., 90., 90., 0., 0.])

        # control params
        # PD gains (tune these)
        self.Kp = np.array([1.8, 2.0, 2.0, 1.5, 1.2, 0.0])   # proportional per joint (last joint unused)
        self.Kd = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.0])
        self.dt = 0.01   # control period (s)
        self.timer = self.create_timer(self.dt, self.control_loop)

        # kinematic params
        self.K_kin = 0.7
        self.epsilon = 0.001

        # PD state
        self.prev_err = np.zeros(6)

        # send rate control to ESP32
        self.last_send_time = 0.0
        self.send_interval = 0.12  # s (≈8 Hz)

        # publishers / subscribers
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_ee = self.create_publisher(PoseStamped, 'ee_pose', 10)
        self.pub_servo_cmd = self.create_publisher(Int32MultiArray, 'servo_commands', 10)

        self.sub_servo_angles = self.create_subscription(Int32MultiArray, 'servo_angles', self.cb_servo_angles, 10)
        self.sub_cmd_angles = self.create_subscription(Int32MultiArray, 'cmd_joint_angles', self.cb_cmd_joint_angles, 10)
        self.sub_cmd_xyz = self.create_subscription(Point, 'cmd_xyz_position', self.cb_cmd_xyz, 10)

        # markers
        self.frame_marker = FrameMarker(self, color_saturation=1.0, alpha=1.0, scale=0.35)
        self.ball_marker_current = BallMarker(self, color['RED'], scale=0.08)
        self.ball_marker_target = BallMarker(self, color['BLUE'], scale=0.1)

        # counters
        self.iter = 0
        self.get_logger().info("=== Full Control Node ready ===")
        self.get_logger().info("Subscribed: /servo_angles, /cmd_joint_angles, /cmd_xyz_position")
        self.get_logger().info("Publishing: /servo_commands, /joint_states, /ee_pose")

    # ----------------- callbacks -----------------
    def cb_servo_angles(self, msg: Int32MultiArray):
        data = np.array(msg.data, dtype=float)
        # ensure 6 entries
        if data.size < 6:
            data = np.append(data, [0]*(6 - data.size))
        # apply offset
        data_adj = data - self.offset_deg
        data_adj[5] = 0.0  # joint6 forced zero
        self.q_feedback_deg = data_adj
        # convert to radians
        self.q_measured = np.deg2rad(data_adj)

        # update internal model q to measured (so RViz shows real robot)
        # small smoothing to avoid jumps
        if self.q is None:
            self.q = self.q_measured.copy()
        else:
            # low-pass blending to keep model stable while control affects it
            alpha = 0.6
            self.q = alpha * self.q_measured + (1.0 - alpha) * self.q

    def cb_cmd_joint_angles(self, msg: Int32MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size < 6:
            data = np.append(data, [0]*(6 - data.size))
        data[5] = 0.0
        self.q_cmd_angles_deg = data
        self.cmd_xyz = None  # prioritize joint control mode
        self.get_logger().info(f"[cmd_joint_angles] Received: {data}")

    def cb_cmd_xyz(self, msg: Point):
        self.cmd_xyz = np.array([msg.x, msg.y, msg.z])
        self.q_cmd_angles_deg = None  # prioritize xyz control
        self.get_logger().info(f"[cmd_xyz_position] Received: {self.cmd_xyz}")
        # show target marker
        try:
            self.ball_marker_target.xyz(self.cmd_xyz)
        except Exception:
            pass

    # ----------------- control loop -----------------
    def control_loop(self):
        self.iter += 1
        now = self.get_clock().now().nanoseconds / 1e9

        # if feedback not yet available, do nothing except publish current model
        if self.q_measured is None:
            # still publish joint_states from internal q (initial zeros)
            self.publish_joint_state(self.q)
            return

        # Mode selection: joint PD if q_cmd_angles_deg present, else kinematic if cmd_xyz, else idle
        if self.q_cmd_angles_deg is not None:
            # Joint PD control (use measured q as plant state)
            qd_deg = self.q_cmd_angles_deg.copy()
            qd_deg[5] = 0.0
            qd = np.deg2rad(qd_deg)

            q = self.q_measured.copy()  # measured rad
            # derivative approximation: dq_meas/dt ~ (q - previous model) / dt -> we approximate derivative of error
            err = qd - q
            derr = (err - self.prev_err) / self.dt
            # PD output: we treat it as delta_q (kinematic step) because we don't model dynamics
            delta_q = self.Kp * err + self.Kd * derr
            # integrate to produce next commanded angles in rad
            q_commanded = q + delta_q * self.dt

            # saturate / clamp to plausible joint ranges (same as earlier limits)
            q_commanded = self._clamp(q_commanded)

            # update prev_err
            self.prev_err = err

            # update internal model to command (for RViz) but blend with measured for realism
            alpha = 0.5
            self.q = alpha * q_commanded + (1.0 - alpha) * self.q_measured

            # prepare degrees to publish to ESP32
            deg_cmd = np.rad2deg(q_commanded)
            deg_cmd[5] = 0.0
            # Publish to ESP32 at limited rate
            if (now - self.last_send_time) >= self.send_interval:
                ia = Int32MultiArray()
                # cast to int (round)
                ia.data = [int(round(float(v))) for v in deg_cmd.tolist()]
                self.pub_servo_cmd.publish(ia)
                self.last_send_time = now
                self.get_logger().debug(f"PD -> /servo_commands: {ia.data}")

            # publish joint_states to RViz (using self.q as current)
            self.publish_joint_state(self.q)

            # show errors and FK occasionally
            if self.iter % 10 == 0:
                self.show_errors_and_fk(qd, q, q_commanded, mode='PD')

        elif self.cmd_xyz is not None:
            # Kinematic control (your previous jacobian-based approach)
            # Use internal q as current estimate (radians)
            q_current = self.q.copy()
            # compute FK and position
            T = fkine_ur5(q_current)
            x_actual = T[0:3,3]
            e = self.cmd_xyz - x_actual
            err_norm = np.linalg.norm(e)
            if err_norm < self.epsilon:
                # reached
                self.get_logger().info("[Kinematic] Target reached (epsilon).")
            else:
                # compute J and pseudoinverse
                J = jacobian_position(q_current)
                try:
                    J_pinv = np.linalg.pinv(J)
                    q_dot = np.dot(J_pinv, self.K_kin * e)
                except Exception as ex:
                    self.get_logger().warn(f"[Kinematic] pinv failed: {ex}")
                    q_dot = np.zeros(5)
                # integrate
                q_new = q_current.copy()
                q_new[:5] = q_new[:5] + q_dot * self.dt
                # clamp and enforce joint6=0
                q_new = self._clamp(q_new)
                q_new[5] = 0.0
                # update internal model q and publish joint_states
                self.q = q_new
                self.publish_joint_state(self.q)

                # Send to ESP32 if enough time elapsed
                if (now - self.last_send_time) >= self.send_interval:
                    deg_cmd = np.rad2deg(self.q)
                    deg_cmd[5] = 0.0
                    ia = Int32MultiArray()
                    ia.data = [int(round(float(v))) for v in deg_cmd.tolist()]
                    self.pub_servo_cmd.publish(ia)
                    self.last_send_time = now
                    self.get_logger().debug(f"Kinematic -> /servo_commands: {ia.data}")

            # show errors and FK occasionally
            if self.iter % 10 == 0:
                # For FK comparison, we need q_measured available (it is) and q_sent (self.q)
                self.show_errors_and_fk(self.q, self.q_measured, self.q, mode='KIN')

        else:
            # idle: only update joint_states with measured q
            self.q = 0.9 * self.q + 0.1 * self.q_measured
            self.publish_joint_state(self.q)

            if self.iter % 50 == 0:
                self.show_errors_and_fk(self.q, self.q_measured, self.q, mode='IDLE')

    # ----------------- utilities -----------------
    def _clamp(self, q_rad):
        # same limits as before:
        limits = [
            (0,  math.pi),
            (-math.pi,  math.pi),
            (-math.pi,  math.pi),
            (-math.pi,  math.pi),
            (0,  math.pi),
            (-math.pi,  0.0)
        ]
        q_sat = q_rad.copy()
        for i, (lo, hi) in enumerate(limits):
            if q_sat[i] < lo: q_sat[i] = lo
            if q_sat[i] > hi: q_sat[i] = hi
        return q_sat

    def publish_joint_state(self, q_rad):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.jnames
        js.position = q_rad.tolist()
        self.pub_js.publish(js)

        # publish end-effector pose for RViz
        T = fkine_ur5(q_rad)
        pose_vec = TF2xyzquat(T)
        ps = PoseStamped()
        ps.header.stamp = js.header.stamp
        ps.header.frame_id = self.ref_frame
        ps.pose.position.x = pose_vec[0]
        ps.pose.position.y = pose_vec[1]
        ps.pose.position.z = pose_vec[2]
        ps.pose.orientation.w = pose_vec[3]
        ps.pose.orientation.x = pose_vec[4]
        ps.pose.orientation.y = pose_vec[5]
        ps.pose.orientation.z = pose_vec[6]
        self.pub_ee.publish(ps)

        # update markers
        try:
            self.frame_marker.setPose(pose_vec)
            self.ball_marker_current.position(T)
        except Exception:
            pass

    def show_errors_and_fk(self, q_desired_rad, q_measured_rad, q_sent_rad, mode=''):
        # q_desired_rad: desired (radians)
        # q_measured_rad: measured (radians)
        # q_sent_rad: what we sent (radians)
        # compute angular error in degrees
        q_des_deg = np.rad2deg(q_desired_rad)
        q_me_deg = np.rad2deg(q_measured_rad)
        q_sent_deg = np.rad2deg(q_sent_rad)
        # ensure length 6
        if q_des_deg.size < 6:
            q_des_deg = np.append(q_des_deg, [0]*(6 - q_des_deg.size))
        if q_me_deg.size < 6:
            q_me_deg = np.append(q_me_deg, [0]*(6 - q_me_deg.size))
        if q_sent_deg.size < 6:
            q_sent_deg = np.append(q_sent_deg, [0]*(6 - q_sent_deg.size))

        err_deg = q_des_deg - q_me_deg
        # FK positions
        T_des = fkine_ur5(q_desired_rad)
        T_me = fkine_ur5(q_measured_rad)
        p_des = T_des[0:3,3]
        p_me = T_me[0:3,3]
        pos_err = p_des - p_me
        pos_err_norm = np.linalg.norm(pos_err)

        # Print succinctly
        self.get_logger().info("--------------------------------------------------------")
        self.get_logger().info(f"[{mode}] Angular desired [deg]: {np.round(q_des_deg,1).tolist()}")
        self.get_logger().info(f"[{mode}] Angular measured [deg]:{np.round(q_me_deg,1).tolist()}")
        self.get_logger().info(f"[{mode}] Angular error [deg]:   {np.round(err_deg,2).tolist()}")
        self.get_logger().info(f"[{mode}] Pos desired (m): ({p_des[0]:.3f}, {p_des[1]:.3f}, {p_des[2]:.3f})")
        self.get_logger().info(f"[{mode}] Pos measured(m): ({p_me[0]:.3f}, {p_me[1]:.3f}, {p_me[2]:.3f})")
        self.get_logger().info(f"[{mode}] Pos error: {pos_err_norm*1000:.2f} mm")
        self.get_logger().info("--------------------------------------------------------")

# -------------------- entrypoint -------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = FullControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
