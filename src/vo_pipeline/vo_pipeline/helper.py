import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
def caculate_euc_dist(p1,p2):
    '''
    p1 = [x1, y1]  p2 = [x'1, y'1]  
         [x2, y2]       [x'2, y'2]
         
    '''
    # print(p1,"---------",p2)
    I = np.array([1,1,1])
    matrix_2d = np.atleast_2d(I)
    I_t = np.transpose(matrix_2d)
    diff= p1 -p2  # [del_x1  , del_y1 ]
    # print("---/n",diff)
    sq = diff * diff # [del_x1_sq  , del_y1_sq ]
    # print("---/n",sq.shape)
    sq_sum = np.sum(sq,axis=-1)

    dist = np.sqrt(sq_sum)
    # print("---/n",dist)
    return  dist.flatten()
    

class detectFeatures:
    def __init__(self,NFEATURES,KMATRIX, DMATRIX):
        self.n_features = NFEATURES
        self.kp = None
        self.K = KMATRIX
        self.D = DMATRIX
    def get_undistorted(self,frame):
        nH,nW  = frame.shape[:2]
        newCM, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (nW,nH),1,(nW,nH))
        frame = cv2.undistort(frame,self.K, self.D,None,newCM)
        self.undistorted_frame = frame 
        return self.undistorted_frame
        
         
    def features(self,frame):
            grey_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            # print(grey_frame)
            orb = cv2.ORB_create(nfeatures=self.n_features)
            kp, des = orb.detectAndCompute(grey_frame, None)
            self.kp = kp
            # print(kp)
            return np.array([k.pt for k in kp], dtype=np.float32).reshape(-1, 1, 2)

    def process_frame(self,frame):
        undistorted_frame = self.get_undistorted(frame)
        undistorted_frame_gray = cv2.cvtColor(undistorted_frame,cv2.COLOR_BGR2GRAY)
        features = self.features(undistorted_frame) 
        return  undistorted_frame, undistorted_frame_gray, features

class opticalFlow():
    def __init__(self):
        self.lk_params = dict( winSize = (15, 15),maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    def calculated_features(self, frame_n, frame_n_plus1, frame_n_features):
     lk_params = self.lk_params
     pn_features, st , err = cv2.calcOpticalFlowPyrLK(frame_n,
                                                frame_n_plus1,
                                                frame_n_features, None,
                                                **lk_params)
     
     return pn_features

          

# if __name__ == '__main__':
def vo_tracker(num_features,k,d,f0,f1,del_T,P_global):
    detector = detectFeatures(num_features,k,d)
    optFlow = opticalFlow()
    lk_params = optFlow.lk_params
    traj_x, traj_y, traj_z = [], [], []
    
    p0_frame = f0
    p0_undistorted_frame , p0_undistorted_frame_gray,p0_features = detector.process_frame(p0_frame)
    if p0_features is None or len(p0_features) < 8:
        print("Not enough features detected in the first frame.")
        return None, None, None, None, None, None, P_global
    # if P_global == None:
    #     P_global = np.eye(4)


    p1_frame = f1
    p1_undistorted_frame , p1_undistorted_frame_gray, _ = detector.process_frame(p1_frame)
# first fprward pass pn-1 points and pn points ,
# The algorithm estimates where $P_1$ moved to in Frame $t$. Call this forward-tracked point $P_2 = (x_2, y_2)$
    p1_features, st , err = cv2.calcOpticalFlowPyrLK(p0_undistorted_frame_gray,
                                       p1_undistorted_frame_gray,
                                       p0_features, None,
                                       **lk_params)
    if p1_features is None or len(p1_features) < 8:
        print("Not enough features tracked in the second frame.")
        return None, None, None, None, None, None, P_global
    # backward track pn tp pn-1 
    p0_features_back , st , err =  cv2.calcOpticalFlowPyrLK(p1_undistorted_frame_gray,
                                                                 p0_undistorted_frame_gray,
                                       p1_features, None,
                                       **lk_params)
    # calculate euc dist for filtering out points 
    
    err = caculate_euc_dist(p0_features,p0_features_back)
    valid = (err < 1).astype(int)
    # print(np.sum(valid==1))
    p0_features = p0_features[valid==1]
    p1_features = p1_features[valid==1]
    if len(p0_features) < 8 or len(p1_features) < 8:
        print("Not enough valid features after filtering.")
        return None, None, None, None, None, None, P_global
    E, valid2 = cv2.findEssentialMat(p0_features, p1_features, k, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    p0_features = p0_features[valid2==1]
    p1_features = p1_features[valid2==1]
    # print(np.sum(valid2==1))
    if len(p0_features) < 8 or len(p1_features) < 8:
        print("Not enough valid features after essential matrix filtering.")
        return None, None, None, None, None, None, P_global
    _, R, t, valid3 = cv2.recoverPose(E, p0_features, p1_features, k)
    # p0_features = p0_features[valid3]
    # p1_features = p1_features[valid3]
    P_step = np.eye(4)
    P_step[:3, :3] = R
    P_step[:3, 3] = t.squeeze()
    # print(P_step)
    P_global = P_global @ P_step
    # print(P_global)
    x = P_global[0, 3]  # Left/Right movement
    y = P_global[1, 3]  # Height / Altitude
    z = P_global[2, 3]  # Forward/Backward movement
    Rm = P_global[:3,:3]
    rotation = Rotation.from_matrix(Rm)
    quat_xyzw = rotation.as_quat()
    euler_xyz = rotation.as_euler('xyz',degrees = True)
    angular_velocities = euler_xyz / del_T
    linear_velocities = t/del_T

        # traj_x.append(x); traj_y.append(y); traj_z.append(z)
    return    x, y , z ,quat_xyzw , angular_velocities, linear_velocities,P_global
  
    # plt.pause(0.001)
    # good_new = p1_features
    # good_old = p0_features
    # for i, (new, old) in enumerate(zip(good_new, 
    #                                     good_old)):
    #         a, b = new.ravel()
    #         c, d = old.ravel()
    #         a, b, c, d = int(a), int(b), int(c), int(d)


    # p0_undistorted_frame_gray = frame_undistorted_gray.copy()
    # p0_features = good_new.reshape(-1, 1, 2)







