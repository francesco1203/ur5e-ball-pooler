import numpy as np
import glob, os, argparse, cv2
from termcolor import cprint
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize

import matplotlib.pyplot as plt
from tqdm import tqdm


######################################################
class ChessboardCalibrator(object):

    def __init__(self, w, h, square_size_meter, cam_matrix, camera_distortion, debug=False):
        self.w = w
        self.h = h
        self.square_size_meter = square_size_meter
        self.chessboard_points = np.zeros((w * h, 3), np.float32)
        self.chessboard_points[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2) * square_size_meter
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.kernel_size = 11

        self.debug = debug

        self.cam_matrix = cam_matrix
        self.camera_distortion = camera_distortion

        print("Camera Matrix: ", self.cam_matrix)
        print("Camera Distortion: ", self.camera_distortion)

        translation_bounds = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
        self.offset_factor = [0.0, 0.0, 0.0]

        self.bounds = [
            (translation_bounds[0], translation_bounds[1]),
            (translation_bounds[2], translation_bounds[3]),
            (translation_bounds[4], translation_bounds[5]),
            (-np.inf, np.inf),
            (-np.inf, np.inf),
            (-np.inf, np.inf),
            (-np.inf, np.inf),
        ]

    def compute_chessboard_pose(self, img, file_name=None):

        img_undistorted = cv2.undistort(img, self.cam_matrix, self.camera_distortion)

        gray = cv2.cvtColor(img_undistorted, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.w, self.h), None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (self.kernel_size, self.kernel_size), (-1, -1), self.criteria)
            ret, rvec, tvec = cv2.solvePnP(self.chessboard_points, corners2, self.cam_matrix, self.camera_distortion)

            R, _ = cv2.Rodrigues(rvec)
            T = np.hstack((R, tvec))
            T = np.vstack((T, np.array([0, 0, 0, 1.0])))

            if self.debug:
                print("file_name: ", file_name)
                print(T)
                self.plot_debug(T, corners2, img_undistorted, self.cam_matrix)

            return T

        else:
            print("not valid!")
            return None

    def validate_images(self, dict_data, invert_pose=False):

        valid_data = {}
        for k, v in tqdm(dict_data.items()):

            pattern_pose = self.compute_chessboard_pose(v["img"], file_name=k)

            if pattern_pose is not None:

                if v["pose"].ndim == 1:
                    v["pose"] = self.pose_to_matrix(v["pose"])

                    if invert_pose:
                        v["pose"] = np.linalg.inv(v["pose"])

                valid_data[k] = {
                    "pose": v["pose"],
                    "pattern_pose": pattern_pose,
                }

        return valid_data

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([pose[-4], pose[-3], pose[-2], pose[-1]]).as_matrix()
        T[:3, 3] = np.array([pose[0], pose[1], pose[2]])
        return T

    def optimization_function(self, x, data_dict):
        e = 0.0

        T_find = self.pose_to_matrix(x)

        for i, v_i in data_dict.items():
            for j, v_j in data_dict.items():
                if i != j:
                    T1_i = v_i["pose"]
                    T2_i = v_i["pattern_pose"]
                    T1_j = v_j["pose"]
                    T2_j = v_j["pattern_pose"]
                    Tr_i = np.matmul(np.matmul(T1_i, T_find), T2_i)
                    Tr_j = np.matmul(np.matmul(T1_j, T_find), T2_j)
                    pr_i = Tr_i[:3, 3]
                    pr_j = Tr_j[:3, 3]
                    diff = pr_j - pr_i
                    e = e + np.linalg.norm(diff) ** 2
        return e

    def calibrate_extrinsics(self, initial_guess, data_dict):

        cprint("{}\n Exstrinsics Computation...\n {}".format("=" * 50, "=" * 50), color="yellow")

        cprint("Initial Guess: {}".format(initial_guess), "blue")
        cprint("Bounds: {}".format(self.bounds), "blue")
        cprint("Offset Factor: {}".format(self.offset_factor), "blue")

        res = minimize(
            self.optimization_function,
            initial_guess,
            args=(data_dict),
            method="L-BFGS-B",
            bounds=self.bounds,
            options={"maxiter": 10000000, "disp": 0},
        )

        quat_normalized = Rotation.from_quat([res.x[3], res.x[4], res.x[5], res.x[6]]).as_quat()

        first_color = "blue"
        cprint("\nExtrinsics {}".format("=" * 50), first_color)
        cprint("\nCamera Frame: [X,Y,Z,QX,QY,QZ,QW]", first_color)
        print("position: ", [res.x[0], res.x[1], res.x[2]])
        print("quaternions: ", quat_normalized)

        cprint("\nCamera Frame orientation: [Roll Pitch Yaw] deg", first_color)
        degs = Rotation.from_quat([res.x[3], res.x[4], res.x[5], res.x[6]]).as_euler("xyz", degrees=True)
        print(degs)

        cprint("\nCamera Frame orientation: [Roll Pitch Yaw] rad", first_color)
        rads = Rotation.from_quat([res.x[3], res.x[4], res.x[5], res.x[6]]).as_euler("xyz", degrees=False)
        print(rads)

    def plot_debug(self, chessboard_pose, chessboard_image_points, image, camera_matrix):
        canvas = image.copy()
        rvec, _ = cv2.Rodrigues(chessboard_pose[:3, :3])
        tvec = chessboard_pose[:3, 3]
        canvas = cv2.drawChessboardCorners(canvas, (self.w, self.h), chessboard_image_points, True)
        canvas = cv2.drawFrameAxes(canvas, camera_matrix, None, rvec, tvec, 0.1)
        fig = plt.figure(figsize=(15, 15))
        plt.imshow(canvas)
        plt.show()


if __name__ == "__main__":

    #######################################
    # Arguments
    #######################################
    ap = argparse.ArgumentParser()
    ap.add_argument("--image_folder", default="./ws_ur5e_ballpool/src/camera_perception/real_camera/camera_calibration/calib_data/data_09_09", type=str)
    ap.add_argument("--image_extension", default="jpg", type=str)
    ap.add_argument("--chessboard_size", default="7x6", type=str)
    ap.add_argument("--chessboard_square_size", default=0.015, type=float)
    ap.add_argument("--image_width", default=1920, type=int)
    ap.add_argument("--image_height", default=1080, type=int)
    ap.add_argument("--debug", action="store_true")
    args = vars(ap.parse_args())
    print(args)

    path_K = os.path.join(args["image_folder"], "camera_matrix.txt")
    path_D = os.path.join(args["image_folder"], "camera_distortion.txt")

    K = np.loadtxt(path_K)
    D = np.loadtxt(path_D)
    W = args["image_width"]
    H = args["image_height"]

    # Initial Guess for Extrinsics Optimization
    initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    img_files = sorted(glob.glob(os.path.join(args["image_folder"], "*.jpg")))

    #some images are not valid, so we skip them
    #SKIP = []
    SKIP = [0,1,10,11,19,2,20,21,22,23,24,25,29,32,6,7]

    data_dict = {}
    for f in img_files:
        img_id = os.path.basename(f).split(".")[0].split("_")[-1]
        if int(img_id) in SKIP:
            continue
        pose = np.loadtxt(os.path.join(args["image_folder"], f"frame_{img_id}.txt"))
        img = cv2.imread(f, cv2.IMREAD_COLOR)
        data_dict[img_id] = {"pose": pose, "img": img}

    #######################################
    # Creates Calibrator
    #######################################
    w, h = map(int, args["chessboard_size"].split("x"))
    sqs = args["chessboard_square_size"]
    calib = ChessboardCalibrator(w, h, sqs, cam_matrix=K, camera_distortion=D, debug=args["debug"])

    # Validate Images
    data_dict_up = calib.validate_images(data_dict, invert_pose=True)

    #######################################
    # Extrinsics
    #######################################
    calib.calibrate_extrinsics(initial_guess, data_dict_up)