import numpy as np
import cv2
import open3d

#point cloud file header format
ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''
imagesize=[]

#this function corrects the distortions of the taken images befor computing disoarity map
def Undistort(Frame,mtx,dist,newcameramtx,roi):
    dst = cv2.undistort(Frame,mtx,dist,None,newcameramtx)
    # crop the image
    cv2.imshow("unsis",dst)
    cv2.waitKey(0)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]

    return dst

def Calibrate (): #claibration of the camera from previously taken images
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    cbrow = 6
    cbcol = 8
    lobjp = np.zeros((cbrow * cbcol, 3), np.float32)
    robjp = np.zeros((cbrow * cbcol, 3), np.float32)

    lobjp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)
    robjp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)
    lnewcameramtx=[]
    rnewcameramtx=[]
    lret=[]
    lmtx=[]
    ldist=[]
    lroi=[]
    lrot=[]
    ltrn=[]
    rret = []
    rmtx = []
    rdist = []
    rroi = []
    rrot = []
    rtrn = []
    imagesizee=[]

    # Arrays to store object points and image points from all the images.
    lobjpoints = []  # 3d point in real world space
    robjpoints = []  # 3d point in real world space

    limgpoints = []  # 2d points in image plane.
    rimgpoints = []  # 2d points in image plane.

    left = cv2.VideoCapture(1)
    leftimages=["D:\\cal\Cal\Leftside.jpg","D:\\cal\Cal\Leftside1.jpg"]
    rightimages=["D:\\cal\Cal\Rightside.jpg","D:\\cal\Cal\Rightside1.jpg"]


    for fname in range(0,2,1):
        leftimg = cv2.imread(leftimages[fname])
        rightimg=cv2.imread(rightimages[fname])
        print("files: "+ leftimages[fname]+" and "+ rightimages[fname])
        leftimg=cv2.cvtColor(leftimg,cv2.COLOR_RGB2GRAY)
        rightimg=cv2.cvtColor(rightimg,cv2.COLOR_RGB2GRAY)

        imagesizee=leftimg.shape[:2]

        # Find the chess board corners
        r, lcorners = cv2.findChessboardCorners(leftimg, (cbcol, cbrow), None) #detecting chessboard corners
        r2, rcorners = cv2.findChessboardCorners(rightimg, (cbcol, cbrow), None)
        if r2 == True:
            print("right chackboard found !")
            robjpoints.append(robjp)

            cv2.cornerSubPix(rightimg, rcorners, (11, 11), (-1, -1), criteria)
            rimgpoints.append(rcorners)
            rret, rmtx, rdist, rrot, rtrn = cv2.calibrateCamera(robjpoints, rimgpoints, rightimg.shape[::-1], None, None)#calibrating camera individually
            h, w = rightimg.shape[:2]
            rnewcameramtx , rroi = cv2.getOptimalNewCameraMatrix(rmtx, rdist, (w, h), 1, (w, h)) #getting refined camera matrix


        # If found, add object points, image points (after refining them)
        if r == True:
            print("left chackboard found !")
            lobjpoints.append(lobjp)

            cv2.cornerSubPix(leftimg, lcorners, (11, 11), (-1, -1), criteria)
            limgpoints.append(lcorners)
            lret, lmtx, ldist, lrot, ltrn = cv2.calibrateCamera(lobjpoints, limgpoints, rightimg.shape[::-1], None, None)
            h, w = leftimg.shape[:2]
            lnewcameramtx, lroi = cv2.getOptimalNewCameraMatrix(lmtx, ldist, (w, h), 1, (w, h))

    (_, _, _, _, _, rotationMatrix, translationVector, _, _) = cv2.stereoCalibrate(
        lobjpoints, limgpoints, rimgpoints,
        lmtx, ldist,
        rmtx, rdist,
        tuple(imagesizee), None, None, None, None,
        cv2.CALIB_FIX_INTRINSIC)#calibrating stereovision of the two views together (supposed to be used for stereocamera)

    (leftRectification, rightRectification, leftProjection, rightProjection,
     dispartityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(
        lmtx, ldist,
        rmtx, rdist,
       tuple(imagesizee) , rotationMatrix, translationVector,
        None, None, None, None, None,
        cv2.CALIB_ZERO_DISPARITY, 0.25) #refining the calibration and getting desparity to depth map
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        lmtx, ldist, leftRectification,
        leftProjection, imagesizee, cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        rmtx, rdist, rightRectification,
        rightProjection, imagesizee, cv2.CV_32FC1)
    print("Finished Calibrating, saving")
    np.savez("D://cal/calibration.npz", lmtx=lmtx,
        rmtx=rmtx, ldist=ldist,
        rdist=rdist, lnewcameramtx=lnewcameramtx,rnewcameramtx=rnewcameramtx, leftROI=leftROI ,rightROI=rightROI,dispartityToDepthMap=dispartityToDepthMap)
    return  lmtx,rmtx,ldist,rdist,lnewcameramtx,rnewcameramtx,leftROI,rightROI,dispartityToDepthMap




#the function that initializing calibration loading of the camera
def Initialize ():
    print("Checking for calibration configurations ..")
    try :
        from tempfile import TemporaryFile #loading external file for calibration
        npzfile= TemporaryFile()
        npzfile = np.load("D://cal/calibration.npz")
        lmtx=npzfile ['lmtx']
        rmtx=npzfile['rmtx']
        ldist=npzfile['ldist']
        rdist=npzfile['rdist']
        lnewcameramtx=npzfile['lnewcameramtx']
        rnewcameramtx=npzfile['rnewcameramtx']
        leftROI = npzfile['leftROI']
        rightROI=npzfile['rightROI']
        dispartityToDepthMap=npzfile['dispartityToDepthMap']
        print("done loading !")
        return lmtx,rmtx,ldist,rdist,lnewcameramtx,rnewcameramtx,leftROI,rightROI,dispartityToDepthMap
    except:
        print("Error loading calibration configuration , device will be re-calibrated")
        return Calibrate()


def write_ply(fn, verts, colors): #saving the point cloud file
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')



def form_and_view_3dcloud (img,disp,dispto3d): #florming the point cloud , saving it, previewing it
    points = cv2.reprojectImageTo3D(disp, dispto3d)
    colors = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]
    write_ply('D://cal/out.ply', out_points, out_colors)
    pp = open3d.read_point_cloud('D://cal/out.ply', format='ply')
    open3d.draw_geometries([pp])


def generatemap (left,right,leftROI,rightROI):#generating disparity map

    stereoMatcher = cv2.StereoBM_create()
    stereoMatcher.setMinDisparity(4)
    stereoMatcher.setNumDisparities(128)
    stereoMatcher.setBlockSize(21)
    stereoMatcher.setROI1(tuple(leftROI))
    stereoMatcher.setROI2(tuple(rightROI))
    stereoMatcher.setSpeckleRange(16)

    depth = stereoMatcher.compute(left, right).astype(np.float32)
    return depth



lmtx,rmtx,ldist,rdist,lnewcameramtx,rnewcameramtx,leftROI,rightROI,dispartityToDepthMap=Initialize() #initialized calibration factors

left= cv2.imread("D:\\cal\Cal\Leftobject.jpg") #reading testing stereo image
right=cv2.imread("D:\\cal\Cal\Rightobject.jpg")
left=cv2.cvtColor(left,cv2.COLOR_RGB2GRAY)#converting to gray scale
right=cv2.cvtColor(right,cv2.COLOR_RGB2GRAY)
left=Undistort(left,lmtx,ldist,lnewcameramtx,leftROI)#undistorting imaged
right=Undistort(right,lmtx,ldist,lnewcameramtx,leftROI)
disp=generatemap(left,right,leftROI,leftROI)#generating disparity map
cv2.imwrite("D:\\disparity.jpg",disp)
print("Generated !")
form_and_view_3dcloud(left,disp,dispartityToDepthMap)#generate 3d cloud
print("Saved !")
cv2.waitKey(0)
cv2.destroyAllWindows()
