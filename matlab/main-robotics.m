% Primeiramente executar:
% $ roslaunch robot_description spawn.launch
% $ roslaunch robot_control control.launch

% Para uso de funções de outros projetos
% Isso também pode ser feito por meio da variável de ambiente MATLABPATH
addpath('/home/yure/Documents/MATLAB/Examples/R2022b/ros/AddBuildAndRemoveObjectsInGazeboExample'); % AddBuildAndRemoveObjectsInGazeboExample
addpath('/home/yure/Documents/MATLAB/Examples/R2022b/nav/LocalizeTurtleBotUsingMonteCarloLocalizationExample');

% Carrefa o mapa do Laser
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

% Inicializa o modelo de odometria para o MCL
odometryModel = OdomModel;
% odometryModel.Noise = [0.1 0.1 0.1 0.1];

% Configura o laser
rangeFinderModel = LikelihoodFieldModel;
% rangeFinderModel.sensorLimits = [0.45 8];
% rangeFinderModel.Map = grid;

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
rangeFinderModel.sensorPose = [sensorTransform.Transform.Translation.X , sensorTransform.Transform.Translation.Y ,laserRotation(1)];

% Inicia objeto do MCL
mcl = MonteCarloGlobal(grid, 2000);
% mcl.UseLidarScan = true;
mcl.motionModel = odometryModel;
mcl.sensorModel = rangeFinderModel;
% mcl.UpdateThresholds = [0.1,0.1,0.1];
% mcl.ResamplingInterval = 5;

% mcl.ParticleLimits = [20 2000];
% mcl.GlobalLocalization = true;
% mcl.InitialPose = ExampleHelperMCLGazeboTruePose;
% mcl.InitialCovariance = eye(3)*0.5;

plotHelper = PlotHelper(grid);
% plotHelper.plotStep(mcl, i);

% To obtain the map:
% map = [];
% for i = 150:620
%     for j = 325:560
%         if grid.getOccupancy([i j]) == 1
%             [i j]
%             map = cat(1, map, [i j]);
%         end
%     end
% end

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
    pose = [odompose.Pose.Pose.Position.X odompose.Pose.Pose.Position.Y odomRotation(1)];
  
   % Leitura do Scan
   scanMsg = receive(laserSub);
   scan = lidarScan(scanMsg);
   
   [isUpdated, estimatedPose] = mcl.MCL(map, pose, scan);
   % "estimatedPose ="
   % estimatedPose
   % length(mcl.particles)
   
   % Se o MCL atualizou a posição
   if isUpdated
        i = i + 1;
        plotHelper.plotStep(mcl, estimatedPose, scan, i);
   end  
end