#include "Frame.h"
#include <QThread>
#include "ORBmatcher.h"
#include "model.h"
#include "Convert.h"
namespace KINECT_SLAM
{
long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

Frame::Frame(const Frame& frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor),
      mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
      mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
      mvKeysUn(frame.mvKeysUn),mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
      mDescriptors(frame.mDescriptors.clone()),
      mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),pc_col(frame.pc_col),mvuRight(frame.mvuRight)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0;j<FRAME_GRID_ROWS;j++)
            mGrid[i][j] = frame.mGrid[i][j];

    if(!frame.mTcw.empty())
       SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imGray, cv::Mat& imRGB,const cv::Mat &imDepth, ORBextractor *extractor, ORBVocabulary *voc,  const float &thDepth,const sensor_msgs::CameraInfoConstPtr cam_info)
    :mpORBvocabulary(voc),mpORBextractor(extractor),cam_info(*cam_info),mThDepth(thDepth)
{
    mnId = nNextId++;

    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mnScaleLevels);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetScaleSigmaSquares();
    mvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    mTimeStamp = cam_info->header.stamp;
    ExtractORB(imGray);
    getIntrinsics(fx,fy,cx,cy,*cam_info);
    mK = (cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    float k1,k2,k3,p1,p2;
    getDistortions(k1,k2,k3,p1,p2,*cam_info);
    mDistCoef = (cv::Mat_<double>(4,1)<<k1,k2,p1,p2);
    if(k3!=0){
        mDistCoef.resize(5);
        mDistCoef.at<double>(4) = k3;
    }


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);
        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
        mbInitialComputations = false;
    }

    invfx = 1.0f / fx;
    invfy = 1.0f / fy;



    mbf = ParameterSetting::instance()->get<double>("base_line");
    mb = mb / fx;

  AssignFeaturesToGrid();
  pc_col = pointcloud_type::Ptr(createXYZRGBPointCloud(imDepth,imRGB,cam_info));

}

void Frame::ExtractORB(const cv::Mat &im){
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);

}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<double>(0) == 0)
    {
        mvKeysUn=mvKeys;
        return;
    }
    cv::Mat mat(N,2,CV_32F);
    for(int i = 0;i < N;i++){
        mat.at<float>(i,0) = mvKeys[i].pt.x;
        mat.at<float>(i,1) = mvKeys[i].pt.y;
    }
    mat = mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat = mat.reshape(1);

    mvKeysUn.resize(N);
    for(int i = 0;i < N; i++){
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x = mat.at<float>(i,0);
        kp.pt.y = mat.at<float>(i,1);
        mvKeysUn[i] = kp;
    }
}
void Frame::AssignFeaturesToGrid()
{   //Assign the features to one grid cell
    int nReserve = 0.5f * N/(FRAME_GRID_COLS * FRAME_GRID_ROWS);
    for(unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for(unsigned int j = 0; j < FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i = 0; i<N; i++)
    {
        const cv::KeyPoint& kp = mvKeysUn[i];

        int nGridPosX,nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
    }


}

void Frame::ComputeImageBounds(const cv::Mat &im)
{
    if(mDistCoef.at<float>(0) != 0.0)
    {   //undistort the four bound points
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0) = 0.0;
        mat.at<float>(0,1) = 0.0;//top-left point

        mat.at<float>(1,0) = im.cols;
        mat.at<float>(1,1) = 0.0;//top-right point

        mat.at<float>(2,0) = 0.0;
        mat.at<float>(2,1) = im.rows;//bottom-left point

        mat.at<float>(3,0) = im.cols;
        mat.at<float>(3,1) = im.rows;
        mat=mat.reshape(2);

        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));


    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = im.cols;
        mnMinY = 0.0f;
        mnMaxY = im.rows;
    }
}


bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = std::round((kp.pt.x-mnMinX) * mfGridElementWidthInv);
    posY = std::round((kp.pt.y-mnMinY) * mfGridElementHeightInv);

    if(posX<0 || posX >= FRAME_GRID_COLS ||posY <0 || posY >= FRAME_GRID_ROWS)
        return false;
    return true;

}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);

    mOw = -mRcw.t() * mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    cv::Mat p = pMP->GetWorldPos();

    const cv::Mat Pc = mRcw * p + mtcw;

    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);
    //first check the depth value
    if(PcZ < 0.0f)
        return false;
    const float invz = 1.0f/PcZ;
    const float u = fx * PcX * invz + cx;
    const float v = fy * PcY * invz + cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    //check the distance between mappoint and camera center is in the scale invariance region
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMaxDistanceInvariance();

    const cv::Mat PO = p - mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    //check the viewing angle
    cv::Mat Pn = pMP->GetNormal();
    const float viewCos = PO.dot(Pn)/dist;//a.dot(b)=|a||b| cos(theta)

    if(viewCos<viewingCosLimit)
        return false;

    //predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,mfLogScaleFactor);

    if(nPredictedLevel>=mnScaleLevels || nPredictedLevel<0)
        return false;

    //this point will be used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u -mbf * invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;
    return true;
}
//to get the keypoint in the grid frame
vector<size_t> Frame::GetFeaturesInArea(const float& x, const float& y, const float& r,const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);
    //compute the area with the center point xy and radius r  in which Cell
    const int nMinCellX = max(0,(int)floor(((x-mnMinX)-r)*mfGridElementWidthInv));
    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil(((x-mnMinX)+r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
           return vIndices;
    if(nMaxCellX<0)
           return vIndices;
    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementWidthInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;
    if(nMaxCellY<0)
            return vIndices;
     const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

     for(int ix = nMinCellX; ix <= nMaxCellX;ix++)
     {
         for(int iy = nMinCellY; iy <= nMinCellY;iy++)
         {
             const vector<size_t> vCell = mGrid[ix][iy];
             if(vCell.empty())
                 continue;
             for(size_t j = 0; j < vCell.size(); j++)
             {
                 const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                 if(bCheckLevels)
                 {
                     if(kpUn.octave<minLevel)
                         continue;
                     if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                          continue;
                 }
                 const float distX = kpUn.pt.x - x;
                 const float distY = kpUn.pt.y - y;

                 if(fabs(distX)<r && fabs(distY)<r)
                     vIndices.push_back(vCell[j]);
             }
         }
     }
     return vIndices;

}

void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }

}

void Frame::ComputeStereoFromKinect(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N ; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU =mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x - mbf/d;
        }
    }

}
cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

}
