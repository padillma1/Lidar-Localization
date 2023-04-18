%initialize in ros
rosshutdown
rosinit


%begin by creating an ousterlidar object by specifying model of lidar and
%pointing to location of .json file 
%Note: Must ensure Lidar port is 7502 and imu port is 7503 (default)
%Can configure using nc commands and API guide to set these params
%NOT needed when implementing in Simulink as a MATLAB func block
obj = ousterlidar("OS1-128","/home/lidar/.ros/os-992210001133-metadata.json")


%create a ptCloud object by reading the ousterlidar object
ptCloud = read(obj)

%the size of box when downsampling using the box grid method
gridStep = 0.1;




%for using region of interest ROI - need to do for fixed and moving ptCloud
stepSize = 100; %for sampling the ptcloud data





%setting xyz limits for the pcplayer function
xlimits = [-45 45]; %in meters
ylimits = [-45 45];
zlimits = [-10 20];

%create a pcplayer object to view the point cloud from inside MATLAB
lidarPlayer = pcplayer(xlimits,ylimits,zlimits)

%show original, unedited ptCloud for reference
pcshow(ptCloud)


%distance threshold for ptcloud segmentation done below
distThreshold = 0.5;

%min num of points for clustering in the segmentation step
minPoints = 10;



%loop to continuously stream the downsampled ptcloud object
while(1)
ptCloud = read(obj);

ptCloudRef = read(obj);
ptCloudCurrent = read(obj);


%for ROI - to be done for ref and current ptClouds
indicesRef = 1:stepSize:ptCloudRef.Count;
tempPtCloudRef = select(ptCloudRef, indicesRef);

indicesCurrent = 1:stepSize:ptCloudCurrent.Count;
tempPtCloudCurrent = select(ptCloudCurrent, indicesCurrent);

%remove invalid points from the sampled ptCloud
[tempPtCloudRef,validIndicesRef] = removeInvalidPoints(tempPtCloudRef);

[tempPtCloudCurrent,validIndicesCurrent] = removeInvalidPoints(tempPtCloudCurrent);

%3-D coords of ptCloud
worldPointsRef = tempPtCloudRef.Location;

worldPointsCurrent = tempPtCloudCurrent.Location;

%find the 2-D image coords corresponding to the 3-D ptcloud
[YRef,XRef] = ind2sub([size(ptCloudRef.Location,1),size(ptCloudRef.Location,2)],indicesRef);
imagePointsRef = [XRef(validIndicesRef)' YRef(validIndicesRef)'];

[YCurrent,XCurrent] = ind2sub([size(ptCloudCurrent.Location,1),size(ptCloudCurrent.Location,2)],indicesCurrent);
imagePointsCurrent = [XCurrent(validIndicesCurrent)' YCurrent(validIndicesCurrent)'];

%estimate cam (sensor) projection matrix from the range and the world coords
camMatrixRef = estimateCameraMatrix(imagePointsRef, worldPointsRef);

camMatrixCurrent = estimateCameraMatrix(imagePointsCurrent, worldPointsCurrent);

%specify cuboid roi within the range of the x,y,z coords of ptCloud
roi = [-30 30 -30 30 -30 30];

%find the indices of the ptCloud data within the roi
indicesRef = findPointsInROI(ptCloudRef,roi,camMatrixRef);

indicesCurrent = findPointsInROI(ptCloudCurrent,roi,camMatrixCurrent);

%get the ptCloud data within the ROI
ptCloudBRef = select(ptCloudRef,indicesRef);
%it barely even winded me 
ptCloudBCurrent = select(ptCloudCurrent,indicesCurrent);

%remove the NaN and Inf values
ptCloudCRef = removeInvalidPoints(ptCloudBRef);

ptCloudCCurrent = removeInvalidPoints(ptCloudBCurrent);

%view(lidarPlayer, ptCloudC)


%-------------------------------------------------------------------------

%ptCloud_downsampled = pcdownsample(ptCloud,"gridAverage",gridStep)

%downsample using randow method, specifiy percentage of input to output
ptCloud_downsampledRef= pcdownsample(ptCloudCRef,'random',0.15);

ptCloud_downsampledCurrent= pcdownsample(ptCloudCCurrent,'random',0.15);



%registering the moving point cloud to a fixed point cloud, based on ICP
%Best to be used after downsampling
%using pcregistericp to create a tform object:

tform = pcregistericp(ptCloud_downsampledCurrent,ptCloud_downsampledRef,'Metric','pointToPoint','Extrapolate',true);


%aligning the "current" downsampled ptcloud w/ the created tform
ptCloudAligned = pctransform(ptCloud_downsampledCurrent,tform);

%merging the downsampled ptcloud w/ the aligned ptCloud
mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloud_downsampledRef,ptCloudAligned,mergeSize);

%for detecting loops, or locations where the previous ptcloud shows in the 
%new current ptcloud
loopDetector = scanContextLoopDetector;
viewId = 20;

%adding these loops as a descriptor to a ptcloud view
descriptor = scanContextDescriptor(ptCloudScene);
addDescriptor(loopDetector,viewId,descriptor);
[loopViewId,dists] = detectLoop(loopDetector,descriptor,'NumExcludedDescriptors',0)

%Creating ptcloud view set object by adding the above view 
vSet = pcviewset;
vSet = addView(vSet,viewId,"PointCloud",ptCloudScene)

%optimizePoses for drift correction
vSetOptim = optimizePoses(vSet)

%aligning to create the final ptcloud map w/ absolute poses
ptClouds = vSetOptim.Views.PointCloud;
tforms = vSetOptim.Views.AbsolutePose;
gridStep_Align = 0.075;
ptCloudMap = pcalign(ptClouds,tforms,gridStep_Align)

%Graph = createPoseGraph(vSetOptim)0
%plot(Graph)

view(lidarPlayer,ptCloudMap)

end

