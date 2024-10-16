% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 2.4999999999999951 -99.999999999999943];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(1).ID = "B[Part1-1:-:Part2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-65.000000000000853 -2.8066438062523957e-13 -10.000000000000028];  % mm
smiData.RigidTransform(2).angle = 1.1102230246251568e-16;  % rad
smiData.RigidTransform(2).axis = [0.22735265443111852 0.97381249248668222 1.2290105327052487e-17];
smiData.RigidTransform(2).ID = "F[Part1-1:-:Part2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [65.000000000000028 0 5];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = "B[Part2-1:-:Part3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-60.886827705860469 8.2778228716051672e-13 -9.9999999999999769];  % mm
smiData.RigidTransform(4).angle = 1.2412670766236366e-16;  % rad
smiData.RigidTransform(4).axis = [0.99005362313554812 -0.1406905231924816 -8.6448766868851244e-18];
smiData.RigidTransform(4).ID = "F[Part2-1:-:Part3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [65.04899109750022 0 252.41179115669021];  % mm
smiData.RigidTransform(5).angle = 0.24110414919616702;  % rad
smiData.RigidTransform(5).axis = [0 -1 0];
smiData.RigidTransform(5).ID = "RootGround[Part1-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(3).mass = 0.0;
smiData.Solid(3).CoM = [0.0 0.0 0.0];
smiData.Solid(3).MoI = [0.0 0.0 0.0];
smiData.Solid(3).PoI = [0.0 0.0 0.0];
smiData.Solid(3).color = [0.0 0.0 0.0];
smiData.Solid(3).opacity = 0.0;
smiData.Solid(3).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.23772743085288328;  % kg
smiData.Solid(1).CoM = [-1.8207392375133162 0.19370255671825187 -12.939471820985611];  % mm
smiData.Solid(1).MoI = [433.30980908337392 448.09266922379607 211.19973663183936];  % kg*mm^2
smiData.Solid(1).PoI = [4.0574859837695367 -16.293662985028661 -0.099923821683990457];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Part1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.083304004422358829;  % kg
smiData.Solid(2).CoM = [-16.426586354552192 -3.2397265947544764 7.9934300865527836];  % mm
smiData.Solid(2).MoI = [10.387248034448769 167.38004389450552 172.65838507346791];  % kg*mm^2
smiData.Solid(2).PoI = [-0.13990255482648808 2.3805584271939275 3.5821758926393672];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Part2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.039804677980669993;  % kg
smiData.Solid(3).CoM = [-10.549471592108368 -1.1970403011577013 4.9489218521644673];  % mm
smiData.Solid(3).MoI = [2.9386714233174946 84.115409404703016 84.729528265265444];  % kg*mm^2
smiData.Solid(3).PoI = [0.001673081939319324 0.10707862726009645 0.52240739276194514];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Part3*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(2).Rz.Pos = 0.0;
smiData.RevoluteJoint(2).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -153.71747723772089;  % deg
smiData.RevoluteJoint(1).ID = "[Part1-1:-:Part2-1]";

smiData.RevoluteJoint(2).Rz.Pos = 69.305713969173297;  % deg
smiData.RevoluteJoint(2).ID = "[Part2-1:-:Part3-1]";

