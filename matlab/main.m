% Primeiramente executar:
% $ roslaunch robot_description spawn.launch
% $ roslaunch robot_control control.launch

% Para uso de funções de outros projetos
% Isso também pode ser feito por meio da variável de ambiente MATLABPATH
addpath('/home/yure/Documents/MATLAB/Examples/R2022b/ros/AddBuildAndRemoveObjectsInGazeboExample'); % AddBuildAndRemoveObjectsInGazeboExample
addpath('/home/yure/Documents/MATLAB/Examples/R2022b/nav/LocalizeTurtleBotUsingMonteCarloLocalizationExample');

% Carrega o mapa do Laser
grayimg = imread('map.pgm');
% grayimg = rgb2gray(img);
bwimg = grayimg < 0.5;
grid = robotics.BinaryOccupancyGrid(bwimg, 50);
show(grid)

% Finaliza conexão existente com o ROS
rosshutdown
clear velPub
clear velSub
clear odomSub
clear msg

% Inicia a conexão com o ROS
ipaddress = 'localhost';
rosinit(ipaddress);

% Posiciona o mapa no gazebo
gazebo = ExampleHelperGazeboCommunicator;

% Realiza as inscrições nos topicos do Turtlebot
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
odomSub = rossubscriber('/odom', 'BufferSize', 25);
velSub = rossubscriber('/mobile_base/commands/velocity', 'BufferSize', 1);
laserSub = rossubscriber('/my_robot/rplidar/laser/scan');
msg = rosmessage('geometry_msgs/Twist');

% Inicializa o modelo de odometria para o AMCL
odometryModel = robotics.OdometryMotionModel;
% odometryModel = OdomMotionModel;
% odometryModel.Noise = [0.1 0.1 0.1 0.1];

% Configura o laser
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = grid;

% Transformação do que é lido nas coordenadas do laser-range-finder para as coordenadas do turtlebot
% >> tftree.AvailableFrames
tftree = rostf;
% waitForTransform(tftree,'/camera_1','/rplidar');
% sensorTransform = getTransform(tftree,'/camera_1','/rplidar');
waitForTransform(tftree,'/carcaca','/rplidar');
sensorTransform = getTransform(tftree,'/carcaca','/rplidar');

% Obtém os ângulos de rotação de Euler
laserQuat = [sensorTransform.Transform.Rotation.W,sensorTransform.Transform.Rotation.X, sensorTransform.Transform.Rotation.Y,sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Configura o laser-range-finder
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X , sensorTransform.Transform.Translation.Y ,laserRotation(1)];
rangeFinderModel.SensorPose
% Inicia objeto do AMCL
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.1,0.1,0.1];
amcl.ResamplingInterval = 5;

amcl.ParticleLimits = [20 2000];
amcl.GlobalLocalization = true;
% amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(grid);

numUpdates = 1000;
i = 0;
% Loop de Controle
while i < numUpdates
    % Realiza leitura da odometria
    odompose = receive(odomSub);
   
    % Posições atuais da odometria
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
  
   % Leitura do Scan
   scanMsg = receive(laserSub);
   scan = lidarScan(scanMsg);
   
   [isUpdated, estimatedPose, estimatedCovarience] = amcl(pose, scan);
   
   % Se o AMCL atualizou a posição
   if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i);
   end  
end 

