
% Copyright (c) 2014, Brigham Young University
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the Brigham Young University nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY BRIGHAM YOUNG UNIVERSITY ''AS IS'' AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL BYU BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
% OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
% ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% \authors: Levi Rupert (Robotics and Dynamics Lab, BYU)
% \adviser: Marc Killpack (Robotics and Dynamics Lab, BYU)

% Note:
% When computing the jacobian using the DH-parameters for Baxter here, and comparing
% them with the jacobian from pyKDL (extracted with rosbag), note that the
% function jacobn (geometric jacobian from the robotics toolbox) ignores the base
% transformation used in our parameters. So the jacobian needs further 
% operations with rotation matrices. The function (SerialLink.)jacob0 returns the correct jacobian 
% directly.

clear all;
close all;

% setting to use combined wrist of hand or leave them separate
combine = false;


%% DH Parameters
% Define the robot model using standard (not modified) DH parameters.

%% Left Arm
%             Theta d     a                                  alpha r/p  theta offset
Ll(1) = Link ([0    0.27035  0.069                             -pi/2  0    0], 'standard'); % start at joint s0 and move to joint s1
Ll(2) = Link ([0    0        0                                  pi/2  0    pi/2], 'standard');        % start at joint s1 and move to joint e0
Ll(3) = Link ([0    0.36435  0.0690                            -pi/2  0    0], 'standard'); % start at joint e0 and move to joint e1
Ll(4) = Link ([0    0        0                                  pi/2  0    0], 'standard');           % start at joint e1 and move to joint w0
Ll(5) = Link ([0    0.37429  0.010                             -pi/2  0    0], 'standard');  % start at joint w0 and move to joint w1
Ll(6) = Link ([0    0        0                                  pi/2  0    0], 'standard');           % start at joint w1 and move to joint w2
Ll(7) = Link ([0    0.229525 0                                  0     0    0], 'standard');         % start at joint w2 and move to end-effector

%% Right Arm
Lr(1) = Link ([0 0.27035  0.069                             -pi/2  0     0],'standard');   % start at joint s0 and move to joint s1
Lr(2) = Link ([0 0        0                                  pi/2  0     pi/2], 'standard');        % start at joint s1 and move to joint e0
Lr(3) = Link ([0 0.36435  0.0690                            -pi/2  0     0], 'standard'); % start at joint e0 and move to joint e1
Lr(4) = Link ([0 0        0                                  pi/2  0     0], 'standard');           % start at joint e1 and move to joint w0
Lr(5) = Link ([0 0.37429  0.010                             -pi/2  0     0], 'standard');  % start at joint w0 and move to joint w1
Lr(6) = Link ([0 0        0                                  pi/2  0     0], 'standard');           % start at joint w1 and move to joint w2
Lr(7) = Link ([0 0.229525 0                                  0     0     0], 'standard');         % start at joint w2 and move to end-effector

%% Define the dynamic parameters that we found by using this script

%% Link Masses
Ll(1).m = 5.700440;
Ll(2).m = 3.226980;
Ll(3).m = 4.312720;
Ll(4).m = 2.072060;
Ll(5).m = 2.246650;
Ll(6).m = 1.609790;
Ll(7).m = 0.350930;
if combine == true,
    Ll(7).m = 0.350930 + 0.191250;
end

%% Link Center of Garvity (COG)
Ll(1).r = [0.01783, 8.59999*10^-4, 0.19127]; // 2nd term may be wrong. Others have computed: 0.000860
Ll(2).r = [0.06845, 0.00269, -0.00529];
Ll(3).r = [-0.02760, 0.001320, 0.282860];
Ll(4).r = [0.001590, -0.026180, -0.011170];
Ll(5).r = [-0.001680, 0.00460, 0.243180];
Ll(6).r = [0.006970, -0.060480, 0.0060];
Ll(7).r = [0.0019800, 0.001250050344661, 0.134594999936937];
if combine == true,
    Ll(7).r = [0.005137046552805, 9.572236157733594e-04, 0.087565859585378];
end

%% grab the data for the r values from a save .mat file r_dh.mat
% load('r_dh');
% for i= 1:7,
%     Ll(i).r = r_dh(i,:);
% end
% clear r_dh;

%% Link Inertias
Ll(1).I = [0.047091022620,           -1.278755600000009*10^-4, -0.006148700390;
           -1.278755600000009*10^-4,  0.037669764550,          -7.808689899999999*10^-4;
           -0.006148700390,          -7.808689899999999*10^-4,  0.035959884780];
      
Ll(2).I = [ 0.011752094190,    3.009639800*10^-4, -0.002076757620;
            3.009639800*10^-4, 0.02788597520,      1.882199300*10^-4;
           -0.002076757620,    1.882199300*10^-4,  0.020787492980];
       
Ll(3).I = [ 0.026617335570,          -2.927063399999999*10^-4, -0.003921898870;
           -2.927063399999999*10^-4,  0.028443552070,          -0.00108389330;
           -0.003921898870,          -0.00108389330,            0.012480083220];
       
Ll(4).I = [0.013182278760,           3.603617300000004*10^-4,  1.966341800000002*10^-4;
           3.603617300000004*10^-4,  0.007115826860,           7.459496000000002*10^-4;
           1.966341800000002*10^-4,  7.459496000000002*10^-4,  0.009268520640];
       
Ll(5).I = [ 0.016677428250,   -1.84037050*10^-4,        -1.86576290*10^-4;
           -1.84037050*10^-4,  0.016754572640,           6.473235199999998*10^-4;
           -1.86576290*10^-4,  6.473235199999998*10^-4,  0.00374631150];
       
Ll(6).I = [ 0.007005379140,          -4.438478399999998*10^-4, -1.534806699999998*10^-4;
           -4.438478399999998*10^-4,  0.003876071520,          -2.11150380*10^-4;
           -1.534806699999998*10^-4, -2.11150380*10^-4,         0.00552755240];
       
Ll(7).I = [2.528915500000000e-04, 5.753109919612049e-06, -1.59345029023859e-06;
           5.753109919612049e-06, 2.688601005244859e-04, -5.19818194489434e-06;
           -1.59345029023859e-06, -5.19818194489434e-06,  0.000307411799475514];
       
if combine == true,
       
    Ll(7).I = [ 0.003374689026060,      -1.284401013673412e-04, -2.728643473110529e-04;
                -1.284401013673412e-04,  0.003431976718419,     -9.806011111258505e-05;
                -2.728643473110529e-04, -9.806011111258505e-05,  5.494148791339629e-04];
end

% % define the link inertias from a .mat file I_mat_dh.mat
% load('I_mat_dh')
% 
% for i = 1:7
%     Ll(i).I = I_mat_dh(:,:,i);
% end
% clear I_mat_dh

%% Try multiplying the off-diaganols by negative 1:
%        
% Ll(1).I = [0.047091022620,           -(-1.278755600000009*10^-4), -(-0.006148700390);
%            -(-1.278755600000009*10^-4), 0.037669764550,             -(-7.808689899999999*10^-4);
%            -(-0.006148700390),          -(-7.808689899999999*10^-4),  0.035959884780];
%       
% Ll(2).I = [ 0.011752094190,    -(3.009639800*10^-4), -(-0.002076757620);
%             -(3.009639800*10^-4), 0.02788597520,      -(1.882199300*10^-4);
%            -(-0.002076757620),    -(1.882199300*10^-4),  0.020787492980];
%        
% Ll(3).I = [ 0.026617335570,          -(-2.927063399999999*10^-4), -(-0.003921898870);
%            -(-2.927063399999999*10^-4),  0.028443552070,          -(-0.00108389330);
%            -(-0.003921898870),          -(-0.00108389330),            0.012480083220];
%        
% Ll(4).I = [0.013182278760,           -(3.603617300000004*10^-4),  -(1.966341800000002*10^-4);
%            -(3.603617300000004*10^-4),  0.007115826860,           -(7.459496000000002*10^-4);
%            -(1.966341800000002*10^-4),  -(7.459496000000002*10^-4),  0.009268520640];
%        
% Ll(5).I = [ 0.016677428250,   -(-1.84037050*10^-4),        -(-1.86576290*10^-4);
%            -(-1.84037050*10^-4),  0.016754572640,           -(6.473235199999998*10^-4);
%            -(-1.86576290*10^-4),  -(6.473235199999998*10^-4),  0.00374631150];
%        
% Ll(6).I = [ 0.007005379140,          -(-4.438478399999998*10^-4), -(-1.534806699999998*10^-4);
%            -(-4.438478399999998*10^-4),  0.003876071520,          -(-2.11150380*10^-4);
%            -(-1.534806699999998*10^-4), -(-2.11150380*10^-4),         0.00552755240];
%        
% Ll(7).I = [ 0.003374689026060,      -(-1.284401013673412e-04), -(-2.728643473110529e-04);
%                 -(-1.284401013673412e-04),  0.003431976718419,     -(-9.806011111258505e-05);
%                 -(-2.728643473110529e-04), -(-9.806011111258505e-05),  5.494148791339629e-04];

       
%% Link the friction values to the Baxter Model
Friction = 0.7*ones(7,1);
for i = 1:length(Friction),
    Ll(i).B = Friction(i);
end

%% Link the motor inertias to the Baxter Model
for i = 1:7,
    Ll(i).Jm = 0;
end

%% Link the gear ratios to the Baxter Model
for i = 1:7,
    Ll(i).G = 0;
end

%% Use all the same parameters for the right arm.
for i = 1:length(Friction),
    Lr(i).r  = Ll(i).r;
    Lr(i).I  = Ll(i).I;
    Lr(i).B  = Ll(i).B;
    Lr(i).Jm = Ll(i).Jm;
    Lr(i).G  = Ll(i).G;
end


%% Create the Serial Link Models for both Arms in RVC Toolbox

% Create the Robots Baxter_L(left arm) and Baxter_R(right arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));
Baxter_r = SerialLink(Lr, 'name', 'Baxter_R', 'base' , ...
                      transl(0.024645, -0.219645, 0.118588) * trotz(-pi/4)...
                      * transl(0.055695, 0, 0.011038));

%% Define qs
% Vectors that contain different sets of joint angles to test the Baxter models
q0     = zeros(1,7);
q1     = [0          0        0        0        0         0        pi/4];
q2     = ones(1,7)*pi/4;
q3     = [-pi/4      0        0        0        pi/4      pi/4     0   ];
qtest  = [pi/4       0        0        0        0         0        0   ];
qtest1 = [ 0.5595   -0.5629  -0.0203   0.9185   0.9411    1.2759  -0.6419];
qtest2 = [-0.45329  -0.5299  -0.5288   1.396   -0.3405    1.1650  -0.71598];
qtest3 = [-0.000767 -0.00153  0.00307 -0.00192 -0.000383 -0.00115  0.00115];
qrand = (0.5-rand(1,7))*2*pi;

% Set any of the q's to the left or right arm models:
q_l = q0;
q_r = q0;

%% Plot Baxter

figure(1)
clf;
hold on
Baxter_l.plot(q_l)
% Baxter_l.plot3d(q0)
% Baxter_r.plot(q_r)

%% DH Transformations

% Make the Transformation matricies that will tranform from the DH frame to
% the Base frame. They are the same for both the left and right
DOF = Baxter_l.n;
T_dh_to_b = NaN(4,4,DOF + 1);
T_dh_to_b(:,:,1) = Baxter_l.base;
for i = 1:DOF,
   T_dh_to_b(:,:,i+1) = T_dh_to_b(:,:,i) * Ll(i).A(q_l(i));
end


%% URDF Transformations

% urdf Tranformations from one link to the previous link

T_urdf_to_p = NaN(4,4,11);

% go from the left arm mount frame to torso frame
T_urdf_to_p(:,:,1) = [rotz(45 ,'deg') * rotz(0) [0.024645, 0.219645, 0.118588]'; ...
    zeros(1,3)                        1                           ];

% go from the left upper shoulder frame (s0) to the left arm mount frame
T_urdf_to_p(:,:,2)= [rotz(q_l(1)) [0.055695, 0.0, 0.011038]'; ...
    zeros(1,3)      1                      ];

% go from the left lower shoulder frame (s1) to the left upper shoulder frame(s0) (joint s1 to
% s0)
T_urdf_to_p(:,:,3) = [rotx(-90, 'deg') * rotz(q_l(2)) [0.069, 0.0, 0.27035]'; ...
    zeros(1,3)                         1                  ];

% go from the left upper elbow frame (e0) to the left lower shoulder frame (s1) (joint e0 to s1)
T_urdf_to_p(:,:,4) = [rotz(90, 'deg') * rotx(90, 'deg') * rotz(q_l(3)) [0.102, 0.0, 0.0]'; ...
    zeros(1,3)                                          1              ];

% go from the left lower elbow frame (e1) to the left upper elbow frame (e0) (joint e1 to e0)
T_urdf_to_p(:,:,5) = [roty(-90, 'deg') * rotx(-90, 'deg') * rotz(q_l(4)) [0.069, 0.0, 0.26242]'; ...
    zeros(1,3)                                            1                  ];

% go from the left upper forearm frame (w0) to the left lower elbow frame (e1) (joint w0 to e1)
T_urdf_to_p(:,:,6) = [rotz(90, 'deg') * rotx(90, 'deg') * rotz(q_l(5)) [0.10359, 0.0, 0.0]'; ...
    zeros(1,3)                                          1                ];

% go from the left lower forearm frame (w1) to the left upper forearm frame (w0) (joint w1 to w0)
T_urdf_to_p(:,:,7) = [roty(-90, 'deg') * rotx(-90, 'deg') * rotz(q_l(6)) [0.01, 0.0, 0.2707]'; ...
    zeros(1,3)                                            1                ];

%  go from the left wrist frame (w2) to the left lower forearm frame (w1) (joint w2 to w1)
T_urdf_to_p(:,:,8) = [rotz(90, 'deg') * rotx(90, 'deg') * rotz(q_l(7)) [0.115975, 0.0, 0.0]'; ...
    zeros(1,3)                                          1                 ];

%  go from the left hand frame to the left wrist frame (not a joint just a link origin)
T_urdf_to_p(:,:,9) = transl( 0.0, 0.0, 0.11355);

% go from the left girpper base frame to the left hand frame(not a joint just a link
% origin)
T_urdf_to_p(:,:,10) = transl(0.0, 0.0, 0.0);

% go from the left endpoint frame to the left gripper base frame
T_urdf_to_p(:,:,11) = transl(0.0, 0.0, 0.025);


% urdf Transformations from each link frame to the base frame 

T_urdf_to_b = NaN(4,4,length(T_urdf_to_p));

T_urdf_to_b(:,:,1) = T_urdf_to_p(:,:,1);
for i = 2:length(T_urdf_to_p),
   T_urdf_to_b(:,:,i) = T_urdf_to_b(:,:,i-1) * T_urdf_to_p(:,:,i);
end

%% URDF to DH Transformations

% Transformations from the urdf frames to the DH frames
% This is done by multiplying the inverse of the DH frames transformations
% by the urdf frames transformations
% i.e. T_urdf_to_dh = inv(T_dh_to_b) * T_urdf_to_b
% Only use the links that coorespond to each other. Ingore the torso, arm
% mount, hand, gripper base, and gripper.

T_urdf_to_dh = NaN(4,4,length(T_dh_to_b)-1);

for i = 2:length(T_dh_to_b),
    T_urdf_to_dh(:,:,i-1) = inv(T_dh_to_b(:,:,i))* T_urdf_to_b(:,:,i);
end

%% URDF to DH Rotation transformation matricies

% Get the rotation matricies that go from the URDF frame to toe DH frame.
% This is needed for rotating the Inertia matricies into the DH frame. 

R_urdf_to_dh = NaN(3,3,length(T_urdf_to_dh));

for i = 1:length(T_urdf_to_dh),
    R_urdf_to_dh(:,:,i) = tr2rt(T_urdf_to_dh(:,:,i));
end

%% DH to Base Rotation transformation matricies

% Get the rotation matricies that go from the URDF frame to toe DH frame.
% This is needed for rotating the Inertia matricies into the DH frame. 

R_dh_to_b = NaN(3,3,length(T_dh_to_b)-1);

for i = 2:length(T_dh_to_b),
    R_dh_to_b(:,:,i-1) = tr2rt(T_dh_to_b(:,:,i));
end

%% URDF to Base Rotation transformation matricies

% Get the rotation matricies that go from the URDF frame to toe DH frame.
% This is needed for rotating the Inertia matricies into the DH frame. 

R_urdf_to_b = NaN(3,3,length(T_urdf_to_b));

for i = 1:length(T_urdf_to_b),
    R_urdf_to_b(:,:,i) = tr2rt(T_urdf_to_b(:,:,i));
end

%% Dynamic Parameters from URDF

% The dynamic parameters as taken from the urdf
Mass = NaN(12,1);    % Mass of each link including torso, arm mount, and hand
r = NaN(12, 3);      % Center of Gravity coordinates in urdf frames
I = NaN(12, 6);      % Inertia values given as [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
I_mat = NaN(3,3,12); % Inertia matrix that is also put into negatives


% Torso parameter values
Mass(1)      = 35.336455;
r(1,:)       = [0.0, 0.0, 0.0];
I(1,:)       = [ 1.849155, -0.000354, -0.154188, 1.662671, 0.003292, 0.802239];
I_mat(:,:,1) = [ I(1,1), -I(1,2), -I(1,3);
                -I(1,2),  I(1,4), -I(1,5);
                -I(1,3), -I(1,5),  I(1,6)];
            
% Arm_mount parameter values
Mass(2)      = 0.0001;
r(2,:)       = [0.0, 0.0, 0.0];
I(2,:)       = [ 1*10^-8, 0.0, 0.0, 1*10^-8, 0.0, 1*10^-8];
I_mat(:,:,2) = [ I(2,1), -I(2,2), -I(2,3);
                -I(2,2),  I(2,4), -I(2,5);
                -I(2,3), -I(2,5),  I(2,6)];

% Upper Shoulder parameter values (s0)
Mass(3)      = 5.70044;
r(3,:)       = [0.01783, 0.00086, 0.19127];
I(3,:)       = [ 0.04709102262, 0.00012787556, 0.00614870039, 0.03766976455, 0.00078086899, 0.03595988478];
I_mat(:,:,3) = [ I(3,1), -I(3,2), -I(3,3);
                -I(3,2),  I(3,4), -I(3,5);
                -I(3,3), -I(3,5),  I(3,6)];
            
% Lower Shoulder parameter values (s1)
Mass(4)      = 3.22698;
r(4,:)       = [0.06845, 0.00269, -0.00529];
I(4,:)       = [ 0.01175209419, -0.00030096398, 0.00207675762, 0.0278859752, -0.00018821993, 0.02078749298];
I_mat(:,:,4) = [ I(4,1), -I(4,2), -I(4,3);
                -I(4,2),  I(4,4), -I(4,5);
                -I(4,3), -I(4,5),  I(4,6)];
            
% Upper Elbow parameter values (e0)
Mass(5)      = 4.31272;
r(5,:)       = [-0.00276, 0.00132, 0.18086];
I(5,:)       = [ 0.02661733557, 0.00029270634, 0.00392189887, 0.02844355207, 0.0010838933, 0.01248008322];
I_mat(:,:,5) = [ I(5,1), -I(5,2), -I(5,3);
                -I(5,2),  I(5,4), -I(5,5);
                -I(5,3), -I(5,5),  I(5,6)];
            
% Lower Elbow parameter values (e1)
Mass(6)      = 2.07206;
r(6,:)       = [0.02611, 0.00159, -0.01117];
I(6,:)       = [ 0.00711582686, 0.00036036173, 0.0007459496, 0.01318227876, -0.00019663418, 0.00926852064];
I_mat(:,:,6) = [ I(6,1), -I(6,2), -I(6,3);
                -I(6,2),  I(6,4), -I(6,5);
                -I(6,3), -I(6,5),  I(6,6)];
            
% Upper Forearm parameter values (w0)
Mass(7)      = 2.24665;
r(7,:)       = [-0.00168, 0.0046, 0.13952];
I(7,:)       = [ 0.01667742825, 0.00018403705, 0.00018657629, 0.01675457264, -0.00064732352, 0.0037463115];
I_mat(:,:,7) = [ I(7,1), -I(7,2), -I(7,3);
                -I(7,2),  I(7,4), -I(7,5);
                -I(7,3), -I(7,5),  I(7,6)];

% Lower Forearm parameter values (w1)
Mass(8)      = 1.60979;
r(8,:)       = [0.06041, 0.00697, 0.006];
I(8,:)       = [ 0.00387607152, -0.00044384784, -0.00021115038, 0.00700537914, 0.00015348067, 0.0055275524];
I_mat(:,:,8) = [ I(8,1), -I(8,2), -I(8,3);
                -I(8,2),  I(8,4), -I(8,5);
                -I(8,3), -I(8,5),  I(8,6)];
            
% Wrist parameter values (w2)
Mass(9)      = 0.35093;
r(9,:)       = [0.00198, 0.00125, 0.01855];
I(9,:)       = [ 0.00025289155, 0.00000575311, -0.00000159345, 0.0002688601, -0.00000519818, 0.0003074118];
I_mat(:,:,9) = [ I(9,1), -I(9,2), -I(9,3);
                -I(9,2),  I(9,4), -I(9,5);
                -I(9,3), -I(9,5),  I(9,6)];
            
% Hand parameter values
Mass(10)      = 0.19125;
r(10,:)       = [0.01093, 0.00042, -0.01532];
I(10,:)       = [ 0.00017588, 0.00000147073, 0.0000243633, 0.00021166377, 0.00000172689, 0.00023745397];
I_mat(:,:,10) = [ I(10,1), -I(10,2), -I(10,3);
                 -I(10,2),  I(10,4), -I(10,5);
                 -I(10,3), -I(10,5),  I(10,6)];
            
% Gripper Base parameter values
Mass(11)      = 0.0001;
r(11,:)       = [0.0, 0.0, 0.0];
I(11,:)       = [ 1*10^-8, 0.0, 0.0, 1*10^-8, 0.0, 1*10^-8];
I_mat(:,:,11) = [ I(11,1), -I(11,2), -I(11,3);
                 -I(11,2),  I(11,4), -I(11,5);
                 -I(11,3), -I(11,5),  I(11,6)];

% Gripper parameter values
Mass(12)      = 0.0001;
r(12,:)       = [0.0, 0.0, 0.0];
I(12,:)       = [ 1*10^-8, 0.0, 0.0, 1*10^-8, 0.0, 1*10^-8];
I_mat(:,:,12) = [ I(12,1), -I(12,2), -I(12,3);
                 -I(12,2),  I(12,4), -I(12,5);
                 -I(12,3), -I(12,5),  I(12,6)];
             
Total_Mass = 0;
for i = 2:length(Mass),
    Total_Mass = Total_Mass + Mass(i);
end
%% Find combined COG for hand/wrist and Combined Inertia

% This all starts in the URDF frame
Mass_wrist = Mass(9);
r_wrist = r(9,:);
Mass_hand = Mass(10);
r_hand = r(10,:);
T_h_to_w = T_urdf_to_p(:,:,9); % Transfer function from hand to wrist
r_h_in_w = T_h_to_w * transl(r_hand);
r_h_in_w = r_h_in_w(1:3,4)'; % COG of hand in wrist frame
Mass_w_h = Mass_wrist + Mass_hand; % mass of wrist and hand combined
r_w_h = (r_wrist*Mass_wrist + r_h_in_w*Mass_hand)/ Mass_w_h; % COG of wrist and hand combined

% Combine the inertias
I_wrist = I_mat(:,:,9);
I_hand = I_mat(:,:,10);
R_h_to_w = T_h_to_w(1:3, 1:3);% Rotation Matrix from hand to wrist
I_h_in_w = R_h_to_w * I_hand * R_h_to_w'; % rotate the hand inertia into the rate frame
d_h = -r_h_in_w' + r_w_h' ; % The distance from the hand to the new cog
d_w = r_wrist;
I_h_in_c = I_h_in_w + Mass_hand*(d_h'*d_h*eye(3) - d_h*d_h'); % Use parallel axis to put hand inertia at new combined COG
I_w_in_c = I_wrist + Mass_wrist*(d_w'*d_w*eye(3) - d_w*d_w'); % Use parallel axis to put hand inertia at new combined COG
I_w_h = I_w_in_c + I_h_in_c;


% if the option combine is true change the dynamic parameters for the wrist
% to be the combined parameters of the wrist and hand
if combine == true,
    Mass(9) = Mass_w_h;
    r(9,:) = r_w_h;
    I(9,:) = [I_w_h(1,1), -I_w_h(1,2), -I_w_h(1,3), I_w_h(2,2), -I_w_h(2,3), I_w_h(3,3)];
    I_mat(:,:,9) = I_w_h; 
    
%    T_urdf_to_dh = T_urdf_to_dh(:,:,1:length(T_urdf_to_dh)-1);
%    R_urdf_to_dh = R_urdf_to_dh(:,:,1:length(R_urdf_to_dh)-1);
%    R_dh_to_b = R_dh_to_b(:,:,1:length(R_dh_to_b)-1);
end
             
%% Dynamic Paprameters from Moritz in DH frame

% Inertia Values
% xx, yy, zz, xy, xz, yz
I1 = [ 0.04709102,  0.03595988,  0.03766976, -0.00614870,  0.00012788, -0.00078087];
I2 = [ 0.02788598,  0.02078749,  0.01175209, -0.00018822, -0.00030096,  0.00207676];
I3 = [ 0.02661734,  0.01248008,  0.02844355, -0.00392190,  0.00029271, -0.00108389];
I4 = [ 0.01318228,  0.00926852,  0.00711583, -0.00019663,  0.00036036,  0.00074595];
I5 = [ 0.01667743,  0.00374631,  0.01675457, -0.00018658,  0.00018404,  0.00064732];
I6 = [ 0.00700538,  0.00552755,  0.00387607,  0.00015348, -0.00044385, -0.00021115];
I7 = [ 0.00121477,  0.00127636,  0.00055487,  0.00000814, -0.00006551,  0.00000472];

I_mat_m = NaN(3,3,7);


% Put the I values into a matrix
I_mat_m(:,:,1) = [ I1(1), -I1(4), -I1(5);
                  -I1(4),  I1(2), -I1(6);
                  -I1(5), -I1(6),  I1(3)];
              
I_mat_m(:,:,2) = [ I2(1), -I2(4), -I2(5);
                  -I2(4),  I2(2), -I2(6);
                  -I2(5), -I2(6),  I2(3)];

I_mat_m(:,:,3) = [ I3(1), -I3(4), -I3(5);
                  -I3(4),  I3(2), -I3(6);
                  -I3(5), -I3(6),  I3(3)];
              
I_mat_m(:,:,4) = [ I4(1), -I4(4), -I4(5);
                  -I4(4),  I4(2), -I4(6);
                  -I4(5), -I4(6),  I4(3)];
              
I_mat_m(:,:,5) = [ I5(1), -I5(4), -I5(5);
                  -I5(4),  I5(2), -I5(6);
                  -I5(5), -I5(6),  I5(3)];
              
I_mat_m(:,:,6) = [ I6(1), -I6(4), -I6(5);
                  -I6(4),  I6(2), -I6(6);
                  -I6(5), -I6(6),  I6(3)];              

I_mat_m(:,:,7) = [ I7(1), -I7(4), -I7(5);
                  -I7(4),  I7(2), -I7(6);
                  -I7(5), -I7(6),  I7(3)];              
              
% Center of mass values
r_m = [-0.0512    0.0791    0.0009;
        0.0027   -0.0053    0.0685;
       -0.0718    0.0816    0.0013;
        0.0016   -0.0112    0.0261;
       -0.0117    0.1312    0.0046;
        0.0070    0.0060    0.0604;
        0.0051    0.0010   -0.0669];
    

%% Dynamic Parameters in DH frame

% Put the dynamic parameters into the DH frames.

% Mass stays the same

% r
r_dh = NaN(length(T_urdf_to_dh),3);

for i = 1:length(T_urdf_to_dh),
    
    int_var = (T_urdf_to_dh(:,:,i) * [r(i+2,:)'; 1])';
    r_dh(i,:) = int_var(1,1:3);
    
end

% I_mat and I again I takes the form (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)

I_mat_dh = NaN(3,3,length(R_urdf_to_dh));
I_dh = NaN(length(R_urdf_to_dh),6);

for i = 1:length(R_urdf_to_dh),
   
    I_mat_dh(:,:,i) = R_urdf_to_dh(:,:,i) * I_mat(:,:,i+2) * R_urdf_to_dh(:,:,i)';
    I_dh(i,:) = [I_mat_dh(1,1,i), -I_mat_dh(1,2,i), -I_mat_dh(1,3,i), ...
                 I_mat_dh(2,2,i), -I_mat_dh(2,3,i), I_mat_dh(3,3,i)];
    
end


%% Dynamic Parameters in Base frame

% Put the dynamic parameters into the base frame for plotting.

r_base = NaN(length(T_urdf_to_dh),3);

for i = 1:length(T_urdf_to_dh),
    
    int_var = (T_urdf_to_b(:,:,i) * [r(i+2,:)'; 1])';
    r_base(i,:) = int_var(1,1:3);
    
end

% I need both the Inertia Matricies from the DH and URDF frames in the Base
% frame for plotting.

I_mat_b_f_dh = NaN(3,3,length(R_dh_to_b));
I_mat_b_f_urdf = NaN(3,3,length(R_dh_to_b));

for i = 1:length(R_dh_to_b),
   
    I_mat_b_f_dh(:,:,i) = R_dh_to_b(:,:,i) * I_mat_dh(:,:,i) * R_dh_to_b(:,:,i)';
    I_mat_b_f_urdf(:,:,i) = R_urdf_to_b(:,:,i+1) * I_mat(:,:,i+1) * R_urdf_to_b(:,:,i+1)';
    

    
end

%% Eigen Values and Eigen Vectors of the Intertias

%First get the Eigen Vectors and Eigen Values of the Inertia Matricies for
%both the DH Inertias and the urdf Inertias
EigVec_dh = NaN(3,3,length(I_mat_dh));
EigVal_dh = NaN(3,3,length(I_mat_dh));


for i = 1:length(I_mat_dh),
    
    [EigVec_dh(:,:,i), EigVal_dh(:,:,i)] = eig(I_mat_dh(:,:,i));
    
end

EigVec_urdf = NaN(3,3,length(I_mat));
EigVal_urdf = NaN(3,3,length(I_mat));

for i = 1:length(I_mat),
    
    [EigVec_urdf(:,:,i), EigVal_urdf(:,:,i)] = eig(I_mat(:,:,i));
    
end


EigVec_m = NaN(3,3,length(I_mat_m));
EigVal_m = NaN(3,3,length(I_mat_m));

for i = 1:length(I_mat_m),
    
    [EigVec_m(:,:,i), EigVal_m(:,:,i)] = eig(I_mat_m(:,:,i));
    
end


% Make sure that the two Inertia Eigen Values are the same for dh to urdf

diff = NaN(3,3,length(EigVal_dh));
myerrors = [];

for i = 1:length(EigVal_dh),
   
    diff(:,:,i) = EigVal_dh(:,:,i) - EigVal_urdf(:,:,i+2);
    
    for j = 1:3,
        diff;
        if diff(j,j,i) > 10^-13,
            myerror_now = ['The Eigen Values of Body differs by more than 10^-13 ', i];
            myerrors = [myerrors; myerror_now];
            disp('The Eigen Values of Body %d differs by more than 10^-13 ')
            disp(i);
            disp(j);
        end
        
    end
    
end

% Make sure that the two Inertia Eigen Values are the same for my dh to
% Moritz dh

diff = NaN(3,3,length(EigVal_m));
myerrors = [];

for i = 1:length(EigVal_m),
   
    diff(:,:,i) = EigVal_dh(:,:,i) - EigVal_m(:,:,i);
    
    for j = 1:3,
        diff;
        if diff(j,j,i) > 10^-13,
            myerror_now = ['The Eigen Values of Body differs by more than 10^-13 ', i];
            myerrors = [myerrors; myerror_now];
            disp('The Eigen Values of Body %d differs by more than 10^-13 ')
            disp(i);
            disp(j);
        end
        
    end
    
end


% Make sure that the two Inertia Eigen Values are the same for urdf to
% Moritz dh

diff = NaN(3,3,length(EigVal_m));
myerrors = [];

for i = 1:length(EigVal_m),
   
    diff(:,:,i) = EigVal_m(:,:,i) - EigVal_urdf(:,:,i+2);
    
    for j = 1:3,
        diff;
        if diff(j,j,i) > 10^-13,
            myerror_now = ['The Eigen Values of Body differs by more than 10^-13 ', i];
            myerrors = [myerrors; myerror_now];
            disp('The Eigen Values of Body %d differs by more than 10^-13 ')
            disp(i);
            disp(j);
        end
        
    end
    
end

% Make sure that the determinate of the Eigen Vetors is not -1 or it will
% give the wrong rotation matrix


for i = 1:length(EigVec_dh),
    
    %DH check
    
    check_det = det(EigVec_dh(:,:,i));
    
    if check_det < 0,
        disp('Eigen Vectors determinate is -1')
        EigVec_dh(1,:,i) = EigVec_dh(1,:,i)*(-1);
        % check again
        check_det_again = det(EigVec_dh(:,:,i));
        if check_det_again < 0,
            disp('Eigen Vectors determinate is still -1')
        end
        
    end
    
    %URDF check
    
    check_det = det(EigVec_urdf(:,:,i+2));
    
    if check_det < 0,
        disp('Eigen Vectors determinate is -1')
        EigVec_urdf(1,:,i+2) = EigVec_urdf(1,:,i+2)*(-1);
        % check again
        check_det_again = det(EigVec_urdf(:,:,i+2));
        if check_det_again < 0,
            disp('Eigen Vectors determinate is still -1')
        end
        
    end
    
    
    if i < 8,
    % Moritz check
    
        check_det = det(EigVec_m(:,:,i));
        i;
        
        if check_det < 0,
            disp('Eigen Vectors determinate is -1')
            EigVec_m(1,:,i) = EigVec_m(1,:,i)*(-1);
            % check again
            check_det_again = det(EigVec_m(:,:,i));
            if check_det_again < 0,
                disp('Eigen Vectors determinate is still -1')
            end
        
        end
    end
    
end

%% Make Elipsoids from the Eigen Vectors and Values

% Make inertia elipsoids that graficly represent the link if it was turned
% into an elipsoid. The size of the elipsoid is dependent on the mass
% distrubution in the acutal link.

x_dh = NaN(31,31,length(EigVal_dh));
y_dh = NaN(31,31,length(EigVal_dh));
z_dh = NaN(31,31,length(EigVal_dh));

x_urdf = NaN(31,31,length(EigVal_urdf));
y_urdf = NaN(31,31,length(EigVal_urdf));
z_urdf = NaN(31,31,length(EigVal_urdf));

x_m = NaN(31,31,length(EigVal_m));
y_m = NaN(31,31,length(EigVal_m));
z_m = NaN(31,31,length(EigVal_m));


for i = 1:length(EigVal_dh),

[x_dh(:,:,i),y_dh(:,:,i),z_dh(:,:,i)] = ellipsoid(0,0,0,1/sqrt(EigVal_dh(1,1,i)),1/sqrt(EigVal_dh(2,2,i)), 1/sqrt(EigVal_dh(3,3,i)),30);
[x_urdf(:,:,i),y_urdf(:,:,i),z_urdf(:,:,i)] = ellipsoid(0,0,0,1/sqrt(EigVal_urdf(1,1,i+2)),1/sqrt(EigVal_urdf(2,2,i+2)), 1/sqrt(EigVal_urdf(3,3,i+2)),30);

if i < 8
    [x_m(:,:,i),y_m(:,:,i),z_m(:,:,i)] = ellipsoid(0,0,0,1/sqrt(EigVal_m(1,1,i)),1/sqrt(EigVal_m(2,2,i)), 1/sqrt(EigVal_m(3,3,i)),30);
end

end


%% Plot the Elipsoids

% Scale the mass and elipsoids so they look proportionate on the plot.

scale = Mass* 0.004;

n = size(x_dh); % intermediate variable
m_n = 8;


for i = 1:n(end),
   
    
   for j = 1:n(1)
       
      for k = 1:n(2)
          
          xyz_dh   = [x_dh(j,k,i);   y_dh(j,k,i);   z_dh(j,k,i)];
          xyz_urdf = [x_urdf(j,k,i); y_urdf(j,k,i); z_urdf(j,k,i)];
          
          
          % Define the center to plot elipsoid
          center_dh   = r_dh(i,:);
          center_urdf = r(i+2,:);
          
          % Scale the elipsoid
          xyz_dh   = xyz_dh*scale(i+2); 
          xyz_urdf = xyz_urdf*scale(i+2);
          
          %Rotate the Elipsoids into their respective frames from the
          %priciple axis
          xyz_dh   = EigVec_dh(:,:,i)*xyz_dh;
          xyz_urdf = EigVec_urdf(:,:,i+2)*xyz_urdf;
          
          % Move the elipsoid to the center of gravity
          xyz_dh   = xyz_dh + center_dh';
          xyz_urdf = xyz_urdf + center_urdf';
          
          %Transform the Eliposids into the base frame
          int_var_dh   = T_dh_to_b(:,:,i+1) * [xyz_dh; 1];
          int_var_urdf = T_urdf_to_b(:,:,i+1) * [xyz_urdf; 1];
          xyz_dh   = int_var_dh(1:3);
          xyz_urdf = int_var_urdf(1:3) + [.2, -.2, 0]'; % the last vector is to move it so it can be differentiated from the dh frames
          
          % save the x, y, and z coordinates to be plotted out of the for
          % loop
          x_dh_save(j,k,i) = xyz_dh(1);
          y_dh_save(j,k,i) = xyz_dh(2);
          z_dh_save(j,k,i) = xyz_dh(3);
          
          x_urdf_save(j,k,i) = xyz_urdf(1);
          y_urdf_save(j,k,i) = xyz_urdf(2);
          z_urdf_save(j,k,i) = xyz_urdf(3);
          
          % do all the same steps but for Moritz dh frame
          
          if i < m_n
              xyz_m = [x_m(j,k,i); y_m(j,k,i); z_m(j,k,i)];
              center_m = r_m(i,:);
              xyz_m = xyz_m*scale(i+2);
              xyz_m = EigVec_m(:,:,i)*xyz_m;
              xyz_m = xyz_m + center_m';
              int_var_m = T_dh_to_b(:,:,i+1)*[xyz_m; 1];
              xyz_m = int_var_m(1:3);
              x_m_save(j,k,i) = xyz_m(1);
              y_m_save(j,k,i) = xyz_m(2);
              z_m_save(j,k,i) = xyz_m(3);
             
          end
      end
       
   end
   
   % Now plot everything
%    figure(1)
%    hold on;
%    surf(x_dh_save(:,:,i),y_dh_save(:,:,i),z_dh_save(:,:,i))
%    surf(x_urdf_save(:,:,i),y_urdf_save(:,:,i),z_urdf_save(:,:,i))
%     
%    figure(2)
%    hold on;
%    surf(x_dh_save(:,:,i),y_dh_save(:,:,i),z_dh_save(:,:,i))
%    surf(x_urdf_save(:,:,i),y_urdf_save(:,:,i),z_urdf_save(:,:,i))
  
%    if i < n(end)
%    figure(1)
%    hold on;
%    surf(x_m_save(:,:,i),y_m_save(:,:,i),z_m_save(:,:,i))
%    
%    figure(2)
%    hold on;
%    surf(x_m_save(:,:,i),y_m_save(:,:,i),z_m_save(:,:,i))
%        
%    end
    
end


% Plot everything on a subplot for comparison.

Baxter_2 = SerialLink(Baxter_l, 'name', 'Baxter_2');
Baxter_3 = SerialLink(Baxter_l, 'name', 'Baxter_3');


figure(3)
subplot(1,2,1)
Baxter_2.plot(q_l)
hold on;

link_begin = 1;
link_end = 7;

for i =link_begin:link_end
    surf(x_dh_save(:,:,i),y_dh_save(:,:,i),z_dh_save(:,:,i))
%     surf(x_urdf_save(:,:,i),y_urdf_save(:,:,i),z_urdf_save(:,:,i))
end

subplot(1,2,2)
Baxter_3.plot(q_l)
hold on;

for i = link_begin:link_end
        surf(x_m_save(:,:,i),y_m_save(:,:,i),z_m_save(:,:,i)) 
end

%% Obtain the Inertia Matrix


M = Baxter_l.inertia(q_l);

