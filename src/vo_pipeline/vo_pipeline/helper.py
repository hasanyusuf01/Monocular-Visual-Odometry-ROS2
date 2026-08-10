import cv2
import numpy as np
import matplotlib.pyplot as plt
CAM_URL = "http://172.16.0.154:8080/video"
from utills import caculate_euc_dist
KMATRIX = np.load(r".\k_M.npy")
DMATRIX = np.load(r".\D_M.npy")
NFEATURES = 100
p0 = None
p1 = None


def caculate_euc_error(p1,p2):
    '''
    p1 = [x1, y1]  p2 = [x'1, y'1]  
         [x2, y2]       [x'2, y'2]
         
    '''
    I = np.array([1,1])
    I_t = np.transpose(I)

    diff= p1 -p2  # [del_x1  , del_y1 ]
    sq = diff * diff # [del_x1_sq  , del_y1_sq ]
    sq_sum = sq * I_t
    sq_sum = sq_sum[1]
    

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
        
         
    def fetaures(self,frame):
            grey_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            # print(grey_frame)
            orb = cv2.ORB_create(nfeatures=self.n_features)
            kp, des = orb.detectAndCompute(grey_frame, None)
            self.kp = kp
            # print(kp)
            return np.array([k.pt for k in kp], dtype=np.float32).reshape(-1, 1, 2)

class opticalFlow():
    def __init__(self):
        self.lk_params = dict( winSize = (15, 15),maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    def calculated_features(self, frame_n, frame_n_plus1, frame_n_features):
     self.lk_params = lk_params
     pn_features, st , err = cv2.calcOpticalFlowPyrLK(frame_n,
                                                frame_n_plus1,
                                                frame_n_features, None,
                                                **lk_params)
     
     return pn_features

          

if __name__ == '__main__':
    detector = detectFeatures(NFEATURES)
    optFlow = opticalFlow()
    cap = cv2.VideoCapture(CAM_URL) 
    lk_params = optFlow.lk_params
    ret, frame = cap.read()
    p0_frame = frame
    p0_undistorted_frame = detector.get_undistorted(p0_frame)
    p0_undistorted_frame_gray = cv2.cvtColor(p0_undistorted_frame,cv2.COLOR_BGR2GRAY)
    p0_features = detector.fetaures(p0_undistorted_frame)
    # if p0_features is
    maskk = np.zeros_like(p0_undistorted_frame)
    color = np.random.randint(0, 255, (100, 3))
    # if p0_features is not None:
        #  print("po ", [k.pt for k in p0_features])
        #  p0_features = [k.pt for k in p0_features]

    P_global = np.eye(4)

    plt.ion()
    fig = plt.figure(figsize=(4, 4))
    ax3d = fig.add_subplot(111, projection='3d')
    traj_x, traj_y, traj_z = [], [], []

    while True:
        ret, frame = cap.read()
        p1_frame = frame
        frame_undistorted = detector.get_undistorted(frame)
        # detector.fetaures(frame_undistorted)
        # kp_image = cv2.drawKeypoints(frame, detector.kp, None, color=(0, 255, 0), flags=0)

        # cv2.imshow('ORB', kp_image)

        frame_undistorted_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

# first fprward pass pn-1 points and pn points ,
# The algorithm estimates where $P_1$ moved to in Frame $t$. Call this forward-tracked point $P_2 = (x_2, y_2)$
        p1_features, st , err = cv2.calcOpticalFlowPyrLK(p0_undistorted_frame_gray,
                                           frame_undistorted_gray,
                                           p0_features, None,
                                           **lk_params)
        # print(pn_features.shape)
        np.save("pn_features.npy",p1_features)


        # backward track pn tp pn-1 
        p0_features_back , st , err =  cv2.calcOpticalFlowPyrLK(frame_undistorted_gray,
                                                                     p0_undistorted_frame_gray,
                                           p1_features, None,
                                           **lk_params)

        # calculate euc dist for filtering out points 
        
        err = caculate_euc_dist(p0_features,p0_features_back)
        valid = (err < 1).astype(int)
        # print(np.sum(valid==1))
        p0_features = p0_features[valid==1]
        p1_features = p1_features[valid==1]
        E, valid2 = cv2.findEssentialMat(p0_features, p1_features, KMATRIX, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        p0_features = p0_features[valid2==1]
        p1_features = p1_features[valid2==1]
        print(np.sum(valid2==1))
        _, R, t, valid3 = cv2.recoverPose(E, p0_features, p1_features, KMATRIX)
        # p0_features = p0_features[valid3]
        # p1_features = p1_features[valid3]

        P_step = np.eye(4)
        P_step[:3, :3] = R
        P_step[:3, 3] = t.squeeze()
        print(P_step)
        P_global = P_global @ P_step
        print(P_global)

        x = P_global[0, 3]  # Left/Right movement
        y = P_global[1, 3]  # Height / Altitude
        z = P_global[2, 3]  # Forward/Backward movement

        traj_x.append(x); traj_y.append(y); traj_z.append(z)
        ax3d.cla()
        ax3d.plot3D(traj_x, traj_y, traj_z)
        plt.pause(0.001)


        good_new = p1_features
        good_old = p0_features
        for i, (new, old) in enumerate(zip(good_new, 
                                            good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                a, b, c, d = int(a), int(b), int(c), int(d)
                maskk = cv2.line(maskk, (a, b), (c, d),
                                color[i].tolist(), 2)
                
                frame = cv2.circle(frame, (a, b), 5,
                                color[i].tolist(), -1)
                
        img = cv2.add(frame, maskk)

        cv2.imshow('frame', img)
            
        k = cv2.waitKey(25)
        if k == 27:
                break

        p0_undistorted_frame_gray = frame_undistorted_gray.copy()
        p0_features = good_new.reshape(-1, 1, 2)


    cv2.destroyAllWindows()
    cap.release()
    plt.close(fig)




