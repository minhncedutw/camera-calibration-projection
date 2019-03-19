# camera-calibration-projection

[Theory of Calibration and 3D reconstruction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=solvepnp)

## Calibration tool:
[Orbbec calibration tool](https://3dclub.orbbec3d.com/t/universal-download-thread-for-astra-series-cameras/622)
[OpenCV tutorial of calibration](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)

## 2D-3D Projections:
[C++ code](https://github.com/JasonChu1313/KinectUtil/blob/master/ProjectUtil.cpp)
```C
int ProjectUtil:: projectToPointCloud( cv::Mat &depth,vector<Point3d> &pointCloud){
    float fx=depthIntrinsic.at<float>(0,0);
    float fy=depthIntrinsic.at<float>(1,1);
    float cx=depthIntrinsic.at<float>(0,2);
    float cy=depthIntrinsic.at<float>(1,2);
    if(depthIntrinsic.cols!=3||depthIntrinsic.rows!=3){
        cout<<"intrinsic size error"<<endl;
        return -1;
    }
   
    for(int i = 0;i< depth.rows;i++){
        for(int j =0;j< depth.cols;j++){
            unsigned short z = depth.at<unsigned short>(i,j);
            //float zfinal = z*1e-3;
            float zfinal = z;
            if(z>0){
                //x,y in the depth camera coordinate
                float x = (j*zfinal - cx*zfinal)/fx;
                float y = (i*zfinal - cy*zfinal)/fy;
                Point3d p;
                p.x=x;
                p.y=y;
                p.z=zfinal;
                pointCloud.push_back(p);
            }
        }
    }
    return 1;
}
```