import numpy as np
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation as R

# from oak_stereo_camera import Camera
import cv2
import os
from datetime import datetime


def rotvec2rotmat(rotvec):
    return R.from_rotvec(rotvec).as_matrix()


def rotmat2quat(rm):
    return R.from_matrix(rm).as_quat()


class URCameraCalibration:
    def __init__(self, robot_ip, cam_type, output_path) -> None:
        self.tag_rot = np.eye(3)
        self.tag_trans = np.array([0.0, 0.0, 0.0])
        print("INIT ROBOT")
        print("ROBOT IP: ", robot_ip)
        self.robot_receive = RTDEReceiveInterface(robot_ip)
        print("ROBOT INITIALIZED")

        # self.camera = Camera(camera_type=cam_type, resolution="1080", focus=120)

        self.output_path = output_path
        self.counter = 0

    def get_actual_pose(self):
        robot_pose = self.robot_receive.getActualTCPPose()
        rot = rotvec2rotmat(robot_pose[3:])
        quat = rotmat2quat(rot)
        robot_pose_quat = np.concatenate([robot_pose[:3], quat])
        return robot_pose_quat

    def capture_image(self):
        # return self.camera.get_image()
        pass

    def save_img_pose(self):

        # img = self.capture_image()
        pose = self.get_actual_pose()

        # cv2.imwrite(os.path.join(self.output_path, "frame_" + str(self.counter) + ".png"), img)
        np.savetxt(
            os.path.join(self.output_path, "frame_" + str(self.counter) + ".txt"), np.array(pose).reshape(1, -1)
        )
        print("Pose saved: {0}".format(self.counter))
        self.counter += 1

    def loop_image_show(self):
        while True:
            # img = self.camera.get_image()
            # frame = cv2.resize(img, (1280, 720))
            #dummy frame
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.imshow("frame", frame)
            c = cv2.waitKey(1)

            if c == ord("s"):
                self.save_img_pose()


if __name__ == "__main__":

    ROBOT_IP = "192.168.1.110"
    CAM_TYPE = "left"

    date = datetime.now().strftime("%d_%m")

    output_path = f"calib_data/data_{date}"
    os.makedirs(output_path, exist_ok=True)
    print("Saving images and poses to: ", output_path)

    ur_camera_calib = URCameraCalibration(ROBOT_IP, CAM_TYPE, output_path)
    ur_camera_calib.loop_image_show()
