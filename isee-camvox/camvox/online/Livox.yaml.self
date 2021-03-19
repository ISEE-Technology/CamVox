%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera Calibratingtion and distortion parameters (OpenCV) 
Camera.fx:   1738.290240007892   
Camera.fy:   1723.368300855836    
Camera.cx:   786.9893767591706   
Camera.cy:   322.1041435965373   


Camera.k1:  -0.07494456464230151
Camera.k2:  -0.04709393561028884
Camera.p1:  -0.001291492622912956
Camera.p2:   0.000236198926759641
Camera.k3:   0.539878086296589

#1520 568
Camera.width: 1520  # 1520 1200
Camera.height: 568   # 568 440

# Camera frames per second 
Camera.fps: 10

# IR projector baseline times fx (aprox.)
Camera.bf: 86.915
# bf = baseline (in meters) * fx, Horizon and MVC 的 baseline = 50 mm 0.277302

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 0

# Close/Far threshold. Baseline times. 100000 0.001
ThDepth: 2600
# Deptmap values factor
DepthMapFactor: 504  #3000

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1500
# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2
# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8
# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500

PointCloudMapping.Resolution: 0.005 #0.01
meank: 50
thresh: 5.0 #2.0



# bf=b*f
# bf * ThDepth / fx即大致为b * ThDepth
# 基线在双目视觉中出现的比较多
# EuRoC.yaml中的bf为47.9，ThDepth为35，fx为435.2，则有效深度为47.9*35/435.3=3.85米；KITTI.yaml中的bf为# 387.57，ThDepth为40，fx为721.54，则有效深度为387.57*40/721.54=21.5米
