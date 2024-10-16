clc
close all
clear
Simulation = robotScenario(UpdateRate=5);
addMesh(Simulation,"Plane",Position=[5 5 0],Size=[10 10],Color=[1 1 153/255]);

H = 1; %Height of wall
W = 0.2; %Thickness(Width) of wall
L = 10; %Length of wall
w_colour = [1 1 1]; 

% Defining outer walls.
addMesh(Simulation,"Box",Position=[W/2, L/2, H/2],Size=[W, L, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L-W/2, L/2, H/2],Size=[W, L, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L/2, L-W/2, H/2],Size=[L, W, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L/2, W/2, H/2],Size=[L, W, H],Color=w_colour,IsBinaryOccupied=true);

% Defining inner walls.
addMesh(Simulation,"Box",Position=[L/8, 2*L/3, H/2],Size=[L/4, W, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L/8, L/3, H/2],Size=[L/4, W, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L/4, 2*L/3, H/2],Size=[W, L/3,  H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[(L-L/4), L/2, H/2],Size=[L/2, W, H],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[L/2, L/2, H/2],Size=[W, 3*L/5, H],Color=w_colour,IsBinaryOccupied=true);

%Defining Obstacles
addMesh(Simulation,"Box",Position=[7.5, 7.5, H/4],Size=[1, 1, H/2],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[1, 1, H/4],Size=[0.6, 0.6, H/2],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[7.5, 1, H/4],Size=[2, 1, H/2],Color=w_colour,IsBinaryOccupied=true);
addMesh(Simulation,"Box",Position=[6, 4, H/4],Size=[0.3, 0.3, H/2],Color=w_colour,IsBinaryOccupied=true);

%Getting the 3D look of the floor
show3D(Simulation)
lightangle(-45,30)
view(50,50)

%Getting 2D view of the floor
Floor_plan = binaryOccupancyMap(Simulation,GridOriginInLocal=[-2 -2],MapSize=[15 15],MapHeightLimits=[0 3]);
inflate(Floor_plan,0.2);
show(Floor_plan)

%Determining the initial position and terminal position of the Vaccum cleaner
Initial_position = [8 5.5];
End_position = [9 1];

rng(100) %seed of random number generator for repeatability

n_nodes = 1000; %detemining the maximum number of nodes
path_finder = mobileRobotPRM(Floor_plan,n_nodes);
path_finder.ConnectionDistance = 1; %distance between two adjacent nodes

%Determining the path
path= findpath(path_finder,Initial_position,End_position);

robotheight = 0.30;
waypoints_N = size(path,1); % Number of waypoints.
% Robot arrival time at first waypoint.
firstInTime = 0;
% Robot arrival time at last waypoint.
lastInTime = firstInTime + (waypoints_N-1);
% Generating waypoint trajectory from planned path.
Route = waypointTrajectory(SampleRate=10,...
                          TimeOfArrival=firstInTime:lastInTime, ...
                          Waypoints=[path, robotheight*ones(waypoints_N,1)], ...
                          ReferenceFrame="ENU");

Cleaner_robot = loadrobot("amrPioneer3DX");
platform = robotPlatform("amr",Simulation, RigidBodyTree=Cleaner_robot,...
                         BaseTrajectory=Route);

[ax,plotFrames] = show3D(Simulation);
lightangle(-45,30)
view(60,50)

hold(ax,"on")
plot(ax,path(:,1),path(:,2),"-ms",...
               LineWidth=2,...
               MarkerSize=4,...
               MarkerEdgeColor="b",...
               MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax,"off")

setup(Simulation)

% Control simulation rate at 20 Hz.
r = rateControl(20);

% Status of robot in simulation.
robotStartMoving = false;

while advance(Simulation)
    show3D(Simulation,Parent=ax,FastUpdate=true);
    waitfor(r);

    currentPose = read(platform);
    if ~any(isnan(currentPose))
        % implies that robot is in the scene and performing simulation.
        robotStartMoving = true;
    end
    if any(isnan(currentPose)) && robotStartMoving
        % break, once robot reaches goal position.
        break;
    end
end
