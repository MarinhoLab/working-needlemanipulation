import socket
from threading import Lock

import dqrobotics
import numpy as np


class LoopInfo:
    def __init__(self, recvstr):
        tokens = recvstr.split()
        self.is_active = bool(int(tokens[0]))
        self.centroid = [float(tokens[1]), float(tokens[2]), float(tokens[3])]
        self.start_index = int(tokens[4])
        self.end_index = int(tokens[5])

    def __str__(self):
        return "is_Active: {}, centroid: {} {} {}, start_index: {}, end_index: {}".format(self.is_active,
                                                                                          self.centroid[0],
                                                                                          self.centroid[1],
                                                                                          self.centroid[2],
                                                                                          self.start_index,
                                                                                          self.end_index)

# Aux functions for thread segment flags


def is_grabbed_by_left_tool(flag: int):
    return flag & 1


def is_grabbed_by_right_tool(flag: int):
    return flag & (1 << 1)


def is_passing_through_left_tube(flag: int):
    return flag & (1 << 2)


def is_passing_through_right_tube(flag: int):
    return flag & (1 << 3)


def is_self_colliding(flag:int):
    return flag & (1 << 4)


def is_touching_left_tool_left_side(flag: int):
    return flag & (1 << 5)


def is_touching_left_tool_right_side(flag: int):
    return flag & (1 << 6)


def detect_num_loops_from_flags(flags: list[int]) -> int:
    filtered_flags = []
    for i in range(len(flags)):
        flag = flags[i]
        if is_touching_left_tool_left_side(flag):
            filtered_flags.append(0)
        elif is_touching_left_tool_right_side(flag):
            filtered_flags.append(1)
    num_loops = 0
    for i in range(len(filtered_flags) - 1):
        if filtered_flags[i] != filtered_flags[i + 1]:
            num_loops += 1
    return (num_loops + 1) // 2


def get_unitary_thread_start_length_from_flags(flags: list[int]) -> float:
    n = len(flags) - 1
    i0 = -1
    i1 = -1
    for i in range(n):
        flag = flags[i]
        if is_passing_through_left_tube(flag) or is_passing_through_right_tube(flag):
            i0 = i
            break
    for i in reversed(range(n)):
        flag = flags[i]
        if is_grabbed_by_left_tool(flag) or is_grabbed_by_right_tool(flag):
            i1 = i
            break
    if i0 < 0 or i1 < 0:
        return 0
    return abs(i0 - i1) / n


def get_unitary_thread_end_length_from_flags(flags: list[int]) -> float:
    n = len(flags)
    for i in reversed(range(n)):
        flag = flags[i]
        if is_passing_through_left_tube(flag) or is_passing_through_right_tube(flag):
            return 1.0 - (i + 0.5) / n
    return 0


# Main class

class PediatricSimulator:

    def __init__(self):
        self.s = None
        self.mutex_lock = Lock()
        pass

    def __del__(self):
        self.disconnect()
        pass

    def connect(self, addr, port=23815):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((addr, port))
        pass

    def disconnect(self):
        if self.s is not None:
            self.s.sendall(bytes("exit", 'utf-8'))
            self.s.close()
            self.s = None
        pass

    def send(self, msg, args):
        self.mutex_lock.acquire()
        if args is not None:
            msg = "{} {}".format(msg, args)
        self.s.sendall(bytes(msg, 'utf-8'))
        self.s.recv(65535)  # Handshake
        self.mutex_lock.release()
        pass

    def receive(self, msg):
        self.mutex_lock.acquire()
        self.s.sendall(bytes(msg, 'utf-8'))
        ret = self.s.recv(65535)
        self.mutex_lock.release()
        return ret

    def send_value(self, msg, value):
        self.send(msg, value)
        pass

    def receive_int(self, msg):
        return int(self.receive(msg))

    def receive_float(self, msg):
        return float(self.receive(msg))



    def receive_bool(self, msg):
        return int(self.receive(msg)) != 0

    def receive_matrix(self, name: str):
        self.mutex_lock.acquire()
        self.s.sendall(bytes("get_matrix_size " + name, 'utf-8'))
        w, h, c = [int(x) for x in self.s.recv(64).split()]
        assert c == 4  # sizeof(float)
        size = w * h * c
        self.s.sendall(bytes("get_matrix_data " + name, 'utf-8'))
        data = bytearray()
        while len(data) < size:
            data.extend(self.s.recv(size - len(data)))
        self.mutex_lock.release()
        return np.frombuffer(data, dtype=np.float32).reshape((w, h))

    def receive_int_matrix(self, name: str):
        self.mutex_lock.acquire()
        self.s.sendall(bytes("get_matrix_size " + name, 'utf-8'))
        w, h, c = [int(x) for x in self.s.recv(64).split()]
        assert c == 4  # sizeof(int)
        size = w * h * c
        self.s.sendall(bytes("get_matrix_data " + name, 'utf-8'))
        data = bytearray()
        while len(data) < size:
            data.extend(self.s.recv(size - len(data)))
        self.mutex_lock.release()
        return np.frombuffer(data, dtype=np.int32).reshape((w, h))

    def send_dq(self, msg, dq: dqrobotics.DQ):
        v8 = dq.vec8()
        args = "{} {} {} {} {} {} {} {}".format(v8[0], v8[1], v8[2], v8[3], v8[4], v8[5], v8[6], v8[7])
        self.send(msg, args)
        pass

    def receive_dq(self, msg):
        values = self.receive(msg).split()
        pw = float(values[0])
        px = float(values[1])
        py = float(values[2])
        pz = float(values[3])
        dw = float(values[4])
        dx = float(values[5])
        dy = float(values[6])
        dz = float(values[7])
        return dqrobotics.DQ(pw, px, py, pz, dw, dx, dy, dz).normalize()

    def send_joints(self, msg, v9):
        args = "{} {} {} {} {} {} {} {} {}".format(v9[0], v9[1], v9[2], v9[3], v9[4], v9[5], v9[6], v9[7], v9[8])
        self.send(msg, args)
        pass

    def receive_joints(self, msg):
        values = self.receive(msg).split()
        return np.array([float(values[k]) for k in range(9)])

    def send_sphere(self, msg, sph):
        v3 = dqrobotics.translation(sph[0]).vec3()
        args = "{} {} {} {}".format(v3[0], v3[1], v3[2], sph[1])
        self.send(msg, args)
        pass

    def receive_sphere(self, msg):
        values = self.receive(msg).split()
        px = float(values[0])
        py = float(values[1])
        pz = float(values[2])
        r = float(values[3])
        T = px * dqrobotics.i_ + py * dqrobotics.j_ + pz * dqrobotics.k_
        return [T, r]

    # Environment

    def restart(self):
        self.send("restart", None)

    def pause(self):
        self.send("pause", None)

    def unpause(self):
        self.send("unpause", None)

    def step(self):
        self.send("step", None)

    def save_simulation_state(self, path_to_file):
        self.send("save_simulation_state", path_to_file)

    def load_simulation_state(self, path_to_file):
        self.send("load_simulation_state", path_to_file)

    def set_workspace_center(self, pose):
        self.send_dq("setWorkspaceCenter", pose)

    def get_workspace_center(self):
        return self.receive_dq("getWorkspaceCenter")

    def set_left_trocar_sphere(self, pose, radius):
        self.send_sphere("setLeftTrocarSphere", [pose, radius])

    def set_right_trocar_sphere(self, pose, radius):
        self.send_sphere("setRightTrocarSphere", [pose, radius])

    def set_left_center_sphere(self, pose, radius):
        self.send_sphere("setLeftCenterSphere", [pose, radius])

    def set_right_center_sphere(self, pose, radius):
        self.send_sphere("setRightCenterSphere", [pose, radius])

    def get_left_center_sphere(self):
        return self.receive_sphere("getLeftCenterSphere")

    def get_right_center_sphere(self):
        return self.receive_sphere("getRightCenterSphere")

    def get_left_trocar_sphere(self):
        return self.receive_sphere("getLeftTrocarSphere")

    def get_right_trocar_sphere(self):
        return self.receive_sphere("getRightTrocarSphere")

    def get_left_trocar(self):
        return self.get_left_trocar_sphere()[0]

    def get_right_trocar(self):
        return self.get_right_trocar_sphere()[0]

    # Robots

    def get_robot_dofs(self):
        return self.receive_int("get_robot_dofs")

    # Left robot

    def get_left_robot_model_ith(self, ith):
        return self.receive_dq("get_left_robot_model_ith {}".format(ith))

    def get_left_robot_base(self):
        return self.receive_dq("get_left_robot_base")

    def set_left_robot_joints(self, thetas):
        self.send_joints("set_left_robot_joints", thetas)

    def get_left_robot_joints(self):
        return self.receive_joints("get_left_robot_joints")

    def get_left_robot_effector(self):
        return self.receive_dq("get_left_robot_effector")

    def set_left_robot_target_pose(self, dq):
        self.send_dq("set_left_robot_target_pose", dq)

    def get_left_robot_target_pose(self):
        return self.receive_dq("get_left_robot_target_pose")

    def set_left_robot_target_grip_angle(self, grip):
        self.send_value("set_left_robot_target_grip_angle", grip)

    def get_left_robot_target_grip_angle(self):
        return self.receive_float("get_left_robot_target_grip_angle")

    def get_left_robot_actual_grip_angle(self):
        return self.receive_float("get_left_robot_actual_grip_angle")

    def is_left_forceps_grabbing_needle(self):
        return self.receive_bool("is_left_forceps_grabbing_needle")

    def is_left_forceps_grabbing_thread(self):
        return self.receive_bool("is_left_forceps_grabbing_thread")

    def is_needle_between_left_forceps_tips(self):
        return self.receive_bool("is_needle_between_left_forceps_tips")

    def is_thread_between_left_forceps_tips(self):
        return self.receive_bool("is_thread_between_left_forceps_tips")

    def is_left_forceps_touching_right_forceps(self):
        return self.receive_bool("isLeftForcepsTouchingRightForceps")

    def is_left_forceps_touching_tissue(self):
        return self.receive_bool("isLeftForcepsTouchingTissue")

    def get_left_forceps_tension(self):
        return self.receive_float("getLeftForcepsTension")

    # Right robot

    def get_right_robot_model_ith(self, ith):
        return self.receive_dq("get_right_robot_model_ith {}".format(ith))

    def get_right_robot_base(self):
        return self.receive_dq("get_right_robot_base")

    def set_right_robot_joints(self, thetas):
        self.send_joints("set_right_robot_joints", thetas)

    def get_right_robot_joints(self):
        return self.receive_joints("get_right_robot_joints")

    def get_right_robot_effector(self):
        return self.receive_dq("get_right_robot_effector")

    def set_right_robot_target_pose(self, dq):
        self.send_dq("set_right_robot_target_pose", dq)

    def get_right_robot_target_pose(self):
        return self.receive_dq("get_right_robot_target_pose")

    def set_right_robot_target_grip_angle(self, grip):
        return self.send_value("set_right_robot_target_grip_angle", grip)

    def get_right_robot_target_grip_angle(self):
        return self.receive_float("get_right_robot_target_grip_angle")

    def get_right_robot_actual_grip_angle(self):
        return self.receive_float("get_right_robot_actual_grip_angle")

    def is_right_forceps_grabbing_needle(self):
        return self.receive_bool("is_right_forceps_grabbing_needle")

    def is_right_forceps_grabbing_thread(self):
        return self.receive_bool("is_right_forceps_grabbing_thread")

    def is_needle_between_right_forceps_tips(self):
        return self.receive_bool("is_needle_between_right_forceps_tips")

    def is_thread_between_right_forceps_tips(self):
        return self.receive_bool("is_thread_between_right_forceps_tips")

    def is_right_forceps_touching_left_forceps(self):
        return self.receive_bool("isRightForcepsTouchingLeftForceps")

    def is_right_forceps_touching_tissue(self):
        return self.receive_bool("isRightForcepsTouchingTissue")

    def get_right_forceps_tension(self):
        return self.receive_float("getRightForcepsTension")

    # Needle

    def get_needle_length(self):
        return self.receive_float("getNeedleLength")

    def get_needle_pose(self):
        return self.receive_dq("getNeedlePose")

    def get_control_needle_pose(self):
        return self.receive_dq("getControlNeedlePose")

    def get_needle_points(self):
        return self.receive_matrix("needle_points")

    def get_needle_frame_at(self, param: float):
        return self.receive_dq("getNeedleFrameAt {}".format(param))

    def get_needle_plane(self):
        return self.receive_dq("get_needle_plane")

    def get_needle_radius(self):
        return self.receive_float("get_needle_radius")

    # Thread

    def get_thread_length(self):
        return self.receive_float("getThreadLength")

    def get_thread_points(self):
        return self.receive_matrix("thread_points")

    def get_thread_segment_flags(self):
        return self.receive_int_matrix("thread_link_flags")

    def get_thread_frame_at(self, param: float):
        return self.receive_dq("getThreadFrameAt {}".format(param))

    def get_left_loop_info(self):
        return LoopInfo(self.receive("getLeftLoopInfo"))

    def get_right_loop_info(self):
        return LoopInfo(self.receive("getRightLoopInfo"))

    def is_thread_touching_left_tool_left_side(self):
        return self.receive_bool("is_thread_touching_left_tool_left_side")

    def is_thread_touching_left_tool_right_side(self):
        return self.receive_bool("is_thread_touching_left_tool_right_side")

    def is_right_forceps_close_to_left_tool(self):
        return self.receive_bool("is_right_forceps_close_to_left_tool")

    def advance_thread(self, amount: int):
        self.send_value("advanceThread", "{}".format(amount))

    # Tissue

    def get_left_tube_target_point(self):
        return self.receive_dq("get_left_tube_target_point")

    def get_right_tube_target_point(self):
        return self.receive_dq("get_right_tube_target_point")

    def get_left_tube_current_point(self):
        return self.receive_dq("get_left_tube_current_point")

    def get_right_tube_current_point(self):
        return self.receive_dq("get_right_tube_current_point")

    # Rendering

    def clear_frames(self):
        self.send("clear_frames", None)

    def set_frame(self, name, dq):
        self.send_dq("set_frame {}".format(name), dq)

    def set_image_size(self, w: int, h: int):
        return self.send("set_image_size", "{} {}".format(w, h))

    def get_image(self, kind: str):
        self.mutex_lock.acquire()
        self.s.sendall(bytes("get_image_size " + kind, 'utf-8'))
        w, h, c = [int(x) for x in self.s.recv(64).split()]
        size = w * h * c
        self.s.sendall(bytes("get_image_pixels " + kind, 'utf-8'))
        data = bytearray()
        while len(data) < size:
            data.extend(self.s.recv(size - len(data)))
        self.mutex_lock.release()
        return np.frombuffer(data, dtype=np.uint8).reshape((h, w, c))

    def get_image2(self, kind: str, quality=95):
        self.mutex_lock.acquire()
        self.s.sendall(bytes("encode_image {} {}".format(kind, quality), 'utf-8'))
        size = int(self.s.recv(64))
        self.s.sendall(bytes("get_encoded_image " + kind, 'utf-8'))
        data = bytearray()
        while len(data) < size:
            data.extend(self.s.recv(size - len(data)))
        self.mutex_lock.release()
        return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
