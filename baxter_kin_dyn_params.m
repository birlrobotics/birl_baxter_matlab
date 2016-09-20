
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