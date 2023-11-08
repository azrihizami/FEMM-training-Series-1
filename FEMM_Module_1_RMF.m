% FEMM stator parametrization
clear


openfemm;
newdocument(0);
mi_probdef(0,'millimeters','planar',1e-8,70,30);

% Attention !!!
% Angle for FEMM is in degree
% Angle for Matlab is in rad

% Define variable

p = 1;                           % pair pole
phi = 3;                         % phase
N_slot = 2*p*3;                  % number of slot
theta_pole = 2*pi/N_slot;        % 1 pole angle
theta_pole_deg = (2*pi/N_slot)*180/pi;   % 1 pole angle
theta_tooth_fan = 0.8*theta_pole; % teeth faning angle
theta_tooth_fan_deg = theta_tooth_fan*180/pi; 
theta_so = (2*pi-N_slot*theta_tooth_fan)/N_slot; % slot opening fanning angle
theta_so_deg = theta_so*180/pi; 
l_tooth = 6.40;                  % tooth width original 3.40
e_shim = 1.00;                   % height tooth base
h_yoke_stat = l_tooth;              % stator yoke height
R_int_stat = 26.04/2;            % stator interior radius
R_ext_stat = 43.40/2 + l_tooth - 3.40;            % stator exterior radius
thickness_case = 0.8;            % casing thickness

%% STATOR
% Define points and segments
x1 = 0; y1 = 0;
x2 = R_int_stat; y2 = 0;
x3 = R_int_stat + e_shim; y3 = 0;
x4 = R_ext_stat; y4 = 0;
x5 = x4*cos(theta_pole); y5 = x4*sin(theta_pole);
x6 = x2*cos(theta_pole); y6 = x2*sin(theta_pole);
x7 = x2*cos(theta_tooth_fan/2); y7 = x2*sin(theta_tooth_fan/2);
x8 = x3*cos(theta_tooth_fan/2); y8 = x3*sin(theta_tooth_fan/2);
x9 = x2*cos((theta_tooth_fan/2)+(theta_so)); y9 = x2*sin((theta_tooth_fan/2)+(theta_so));
x10 = x3*cos((theta_tooth_fan/2)+(theta_so)); y10 = x3*sin((theta_tooth_fan/2)+(theta_so));
x11 = x4-h_yoke_stat; y11 =0;
x12 = x11; y12 = l_tooth/2;
xmid_theta_slot = (l_tooth/2)/(tan(theta_pole/2));  % center pole slot
ymid_theta_slot = l_tooth/2;
x13 = (x12-xmid_theta_slot)*cos(theta_pole)+xmid_theta_slot; y13 = (x12-xmid_theta_slot)*sin(theta_pole)+ymid_theta_slot;
x14 = x3; y14 = l_tooth/2;
x15 = (x14-xmid_theta_slot)*cos(theta_pole)+xmid_theta_slot; y15 = (x14-xmid_theta_slot)*sin(theta_pole)+ymid_theta_slot;
x16 = R_ext_stat+thickness_case; y16 = 0;
x17 = (R_ext_stat+thickness_case)*cos(theta_pole); y17 = (R_ext_stat+thickness_case)*sin(theta_pole);

% create node
mi_addnode(x1,y1);
mi_addnode(x2,y2);
mi_addnode(x3,y3);
mi_addnode(x4,y4);
mi_addnode(x5,y5);
mi_addnode(x6,y6);
mi_addnode(x7,y7);
mi_addnode(x8,y8);
mi_addnode(x9,y9);
mi_addnode(x10,y10);
mi_addnode(x11,y11);
mi_addnode(x12,y12);
mi_addnode(x13,y13);
mi_addnode(x14,y14);
mi_addnode(x15,y15);
mi_addnode(x16,y16);
mi_addnode(x17,y17);

% create segment and arc
% mi_addsegment(x2,y2,x4,y4); %Draw line segment between poles
% mi_addsegment(x6,y6,x5,y5); 
mi_addsegment(x12,y12,x14,y14);
mi_addsegment(x13,y13,x15,y15);
mi_addsegment(x7,y7,x8,y8);
mi_addsegment(x9,y9,x10,y10);
mi_addsegment(x8,y8,x10,y10);
mi_addsegment(x14,y14,x8,y8);
mi_addsegment(x10,y10,x15,y15);
mi_addarc(x4,y4,x5,y5,theta_pole_deg,1);
mi_addarc(x12,y12,x13,y13,theta_pole_deg,1);
mi_addarc(x2,y2,x7,y7,(theta_pole_deg/2)-(theta_so_deg/2),1);
mi_addarc(x9,y9,x6,y6,(theta_pole_deg/2)-(theta_so_deg/2),1);
mi_addarc(x16,y16,x17,y17,theta_pole_deg,1);

% Repeat for n pole
mi_selectarcsegment(x2,y2);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 3 );
mi_selectsegment(x7,y7);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectsegment(((x8+x14)/2),y8);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectsegment(x14,y14);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectarcsegment(x12,y12);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 3 );
mi_selectsegment(x13,y13);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectsegment(x15,((y15+y10)/2));
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectsegment(((x10+x9)/2),y9);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectarcsegment(x9,y9);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 3 );
mi_selectarcsegment(x5,y5);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 3 );
mi_selectsegment(x8,((y8+y10)/2));
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 1 );
mi_selectarcsegment(x16,y16);
mi_copyrotate2(x1, y1, theta_pole_deg, N_slot-1, 3 );

%% ROTOR
% Add geometric parameters of rotor

airgap = 2;     % airgap in mm
l_rotor = 2*l_tooth; %thickness of rotor
theta_rotor = 360/N_slot;   % angular span rotor
x_rotor = R_int_stat - airgap;  %radius rotor

% define points of rotor
x18 = x_rotor*cos(theta_rotor/2*(pi/180)); y18 = -x_rotor*sin(theta_rotor/2*(pi/180));
x19 = x_rotor*cos(theta_rotor/2*(pi/180)); y19 = x_rotor*sin(theta_rotor/2*(pi/180));
x20 = -x_rotor*cos(theta_rotor/2*(pi/180)); y20 = x_rotor*sin(theta_rotor/2*(pi/180));
x21 = -x_rotor*cos(theta_rotor/2*(pi/180)); y21 = -x_rotor*sin(theta_rotor/2*(pi/180));


% add points of rotor
mi_addnode(x18,y18);
mi_addnode(x19,y19);
mi_addnode(x20,y20);
mi_addnode(x21,y21);

%  add segments
mi_addarc(x18,y18,x19,y19,theta_rotor,1);
mi_addarc(x20,y20,x21,y21,theta_rotor,1);
mi_addsegment(x18,y18,x21,y21);
mi_addsegment(x19,y19,x20,y20);



%% Define region
%     add material
mi_getmaterial('Air') 
mi_getmaterial('M-15 Steel')
mi_getmaterial('20 AWG')
mi_getmaterial('Aluminum, 6061-T6')

mi_addblocklabel(x_rotor + (airgap/2),0);
mi_selectlabel(x_rotor + (airgap/2),0);
mi_setblockprop('Air', 0, 0, '', 0, 1,1);
mi_clearselected;
mi_addblocklabel(1,0);
mi_selectlabel(1,0);
mi_setblockprop('M-15 Steel', 0, 0, '', 0, 1,1);
mi_clearselected;
mi_addblocklabel(0,R_ext_stat-h_yoke_stat/2);
mi_selectlabel(0,R_ext_stat-h_yoke_stat/2);
mi_setblockprop('M-15 Steel', 0, 0, '', 0, 1,1);
mi_clearselected;
mi_addblocklabel(0,R_int_stat+h_yoke_stat/2);
mi_selectlabel(0,R_int_stat+h_yoke_stat/2);
mi_setblockprop('20 AWG', 0, 0, '', 0, 2,1);
mi_clearselected;
mi_addblocklabel(R_ext_stat+thickness_case/2,0);
mi_selectlabel(R_ext_stat+thickness_case/2,0);
mi_setblockprop('Aluminum, 6061-T6', 0, 0, '', 0, 1,1);
mi_clearselected;

% rotate slot
mi_selectgroup(2);
mi_copyrotate(0,0,60,5)
mi_clearselected;

%% Add circuit properties
n = 10; %number of turns
Imax = 20;  % peak current (A)
f = 50;    % frequency electric supply in (Hz)
T = 1/f;   % electrical period (sec) 
for load_theta = 0:2*pi   % Load angle (rad)
t = T/(2*pi)*load_theta;  % electrical time 
% ----------> DEFINE AS LOAD ANGLE (CONTROL PARAMETERS)
i_A = Imax*sin(2*pi*f*t);
i_B = Imax*sin(2*pi*f*t - 2*pi/3) ;
i_C = Imax*sin(2*pi*f*t + 2*pi/3);
mi_addcircprop('A+', i_A, 1)
mi_addcircprop('A-', -i_A, 1)
mi_addcircprop('B+', i_B, 1)
mi_addcircprop('B-', -i_B, 1)
mi_addcircprop('C+', i_C, 1)
mi_addcircprop('C-', -i_C, 1)

% Slot A+
mi_selectlabel(0,R_int_stat+h_yoke_stat/2);
mi_setblockprop('20 AWG', 0, 0, 'A+', 0, 3,n);
% mi_selectgroup(3);
% mi_copyrotate(0, 0, 180/p, p )            % not necessary if p = 1
mi_clearselected;
% Slot B+
mi_selectlabel((R_int_stat+h_yoke_stat/2)*cos(360/2*p/phi*(pi/180)),(R_int_stat+h_yoke_stat/2)*sin(360/2*p/phi*(pi/180)));
mi_setblockprop('20 AWG', 0, 0, 'B-', 0, 4,n);
mi_clearselected;
% % Slot C+
mi_selectlabel((R_int_stat+h_yoke_stat/2)*cos(360/2*p/phi*(pi/180)),-(R_int_stat+h_yoke_stat/2)*sin(360/2*p/phi*(pi/180)));
mi_setblockprop('20 AWG', 0, 0, 'C+', 0, 5,n);
mi_clearselected;
% % Slot A-
mi_selectlabel(0,-(R_int_stat+h_yoke_stat/2)*sin(360/2*p/phi*(pi/180)));
mi_setblockprop('20 AWG', 0, 0, 'A-', 0, 6,n);
mi_clearselected;
% % Slot B-
mi_selectlabel(-(R_int_stat+h_yoke_stat/2)*cos(360/2*p/phi*(pi/180)),-(R_int_stat+h_yoke_stat/2)*sin(360/2*p/phi*(pi/180)));
mi_setblockprop('20 AWG', 0, 0, 'B+', 0, 7,n);
mi_clearselected;
% % Slot C-
mi_selectlabel(-(R_int_stat+h_yoke_stat/2)*cos(360/2*p/phi*(pi/180)),(R_int_stat+h_yoke_stat/2)*sin(360/2*p/phi*(pi/180)));
mi_setblockprop('20 AWG', 0, 0, 'C-', 0, 8,n);
mi_clearselected;

% main_restore
% mi_zoomnatural
mi_saveas('Test.fem');
% mi_savebitmap('Example_1.bmp')

%% Analyze
% mi_createmesh;
mi_analyze;
mi_loadsolution;
mo_showdensityplot(1,0,0.5,0,'mag');

end