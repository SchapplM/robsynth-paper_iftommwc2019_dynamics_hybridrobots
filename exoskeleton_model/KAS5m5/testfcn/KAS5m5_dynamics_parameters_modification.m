% Return reduced dynamics parameters of the robot
% KAS5m5
%
% The dynamics parameters can be modified by user input
% into the Maple code generation.
%
% Input: Full arrays of dynamics parameters
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% m [14x1]
%   mass of all robot links (including the base)
% rSges [14x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% Icges [14x6]
%   inertia of all robot links about their respective center of mass, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertiavector2matrix.m)
%
% Output: Modified dynamics parameters, consistent with dynamics
% m,rSges,Icges
% (same meaning as input)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [m,rSges,Icges] = KAS5m5_dynamics_parameters_modification(pkin,m,rSges,Icges)

%% Coder Information
%#codegen
%$cgargs {zeros(30,1),zeros(14,1),zeros(14,3),zeros(14,6)}
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_dynamics_parameters_modification: pkin has to be [30x1] (double)');
assert(isreal(m) && all(size(m) == [14 1]), ...
  'KAS5m5_dynamics_parameters_modification: m has to be [14x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [14,3]), ...
  'KAS5m5_dynamics_parameters_modification: rSges has to be [14x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [14 6]), ...
  'KAS5m5_dynamics_parameters_modification: Icges has to be [14x6] (double)'); 

%% Variable Initialization
% Complete set of dynamics parameters that can be reduced by the user.
a10 = pkin(1);
a11 = pkin(2);
a12 = pkin(3);
a4 = pkin(4);
a5 = pkin(5);
a6 = pkin(6);
a9 = pkin(7);
d1 = pkin(8);
d2 = pkin(9);
d3 = pkin(10);
d7 = pkin(11);
d8 = pkin(12);
delta8s = pkin(13);
delta9s = pkin(14);
l11 = pkin(15);
l12 = pkin(16);
l13 = pkin(17);
l14 = pkin(18);
l17 = pkin(19);
l18 = pkin(20);
l20 = pkin(21);
l21 = pkin(22);
l22 = pkin(23);
l4 = pkin(24);
l5 = pkin(25);
l6 = pkin(26);
delta10s = pkin(27);
delta12s = pkin(28);
delta17s = pkin(29);
delta18s = pkin(30);

M0 = m(1);
M1 = m(2);
M2 = m(3);
M3 = m(4);
M4 = m(5);
M5 = m(6);
M6 = m(7);
M7 = m(8);
M8 = m(9);
M9 = m(10);
M10 = m(11);
M11 = m(12);
M12 = m(13);
M13 = m(14);

SX0 = rSges(1,1);
SY0 = rSges(1,2);
SZ0 = rSges(1,3);
SX1 = rSges(2,1);
SY1 = rSges(2,2);
SZ1 = rSges(2,3);
SX2 = rSges(3,1);
SY2 = rSges(3,2);
SZ2 = rSges(3,3);
SX3 = rSges(4,1);
SY3 = rSges(4,2);
SZ3 = rSges(4,3);
SX4 = rSges(5,1);
SY4 = rSges(5,2);
SZ4 = rSges(5,3);
SX5 = rSges(6,1);
SY5 = rSges(6,2);
SZ5 = rSges(6,3);
SX6 = rSges(7,1);
SY6 = rSges(7,2);
SZ6 = rSges(7,3);
SX7 = rSges(8,1);
SY7 = rSges(8,2);
SZ7 = rSges(8,3);
SX8 = rSges(9,1);
SY8 = rSges(9,2);
SZ8 = rSges(9,3);
SX9 = rSges(10,1);
SY9 = rSges(10,2);
SZ9 = rSges(10,3);
SX10 = rSges(11,1);
SY10 = rSges(11,2);
SZ10 = rSges(11,3);
SX11 = rSges(12,1);
SY11 = rSges(12,2);
SZ11 = rSges(12,3);
SX12 = rSges(13,1);
SY12 = rSges(13,2);
SZ12 = rSges(13,3);
SX13 = rSges(14,1);
SY13 = rSges(14,2);
SZ13 = rSges(14,3);

XXC0 = Icges(1,1);
XYC0 = Icges(1,4);
XZC0 = Icges(1,5);
YYC0 = Icges(1,2);
YZC0 = Icges(1,6);
ZZC0 = Icges(1,3);
XXC1 = Icges(2,1);
XYC1 = Icges(2,4);
XZC1 = Icges(2,5);
YYC1 = Icges(2,2);
YZC1 = Icges(2,6);
ZZC1 = Icges(2,3);
XXC2 = Icges(3,1);
XYC2 = Icges(3,4);
XZC2 = Icges(3,5);
YYC2 = Icges(3,2);
YZC2 = Icges(3,6);
ZZC2 = Icges(3,3);
XXC3 = Icges(4,1);
XYC3 = Icges(4,4);
XZC3 = Icges(4,5);
YYC3 = Icges(4,2);
YZC3 = Icges(4,6);
ZZC3 = Icges(4,3);
XXC4 = Icges(5,1);
XYC4 = Icges(5,4);
XZC4 = Icges(5,5);
YYC4 = Icges(5,2);
YZC4 = Icges(5,6);
ZZC4 = Icges(5,3);
XXC5 = Icges(6,1);
XYC5 = Icges(6,4);
XZC5 = Icges(6,5);
YYC5 = Icges(6,2);
YZC5 = Icges(6,6);
ZZC5 = Icges(6,3);
XXC6 = Icges(7,1);
XYC6 = Icges(7,4);
XZC6 = Icges(7,5);
YYC6 = Icges(7,2);
YZC6 = Icges(7,6);
ZZC6 = Icges(7,3);
XXC7 = Icges(8,1);
XYC7 = Icges(8,4);
XZC7 = Icges(8,5);
YYC7 = Icges(8,2);
YZC7 = Icges(8,6);
ZZC7 = Icges(8,3);
XXC8 = Icges(9,1);
XYC8 = Icges(9,4);
XZC8 = Icges(9,5);
YYC8 = Icges(9,2);
YZC8 = Icges(9,6);
ZZC8 = Icges(9,3);
XXC9 = Icges(10,1);
XYC9 = Icges(10,4);
XZC9 = Icges(10,5);
YYC9 = Icges(10,2);
YZC9 = Icges(10,6);
ZZC9 = Icges(10,3);
XXC10 = Icges(11,1);
XYC10 = Icges(11,4);
XZC10 = Icges(11,5);
YYC10 = Icges(11,2);
YZC10 = Icges(11,6);
ZZC10 = Icges(11,3);
XXC11 = Icges(12,1);
XYC11 = Icges(12,4);
XZC11 = Icges(12,5);
YYC11 = Icges(12,2);
YZC11 = Icges(12,6);
ZZC11 = Icges(12,3);
XXC12 = Icges(13,1);
XYC12 = Icges(13,4);
XZC12 = Icges(13,5);
YYC12 = Icges(13,2);
YZC12 = Icges(13,6);
ZZC12 = Icges(13,3);
XXC13 = Icges(14,1);
XYC13 = Icges(14,4);
XZC13 = Icges(14,5);
YYC13 = Icges(14,2);
YZC13 = Icges(14,6);
ZZC13 = Icges(14,3);

%% Parameter Postprocessing / Set Output
% Create the reduced set of dynamics parameters that is also used for generation of the dynamics equations.
% Aus parameters_dyn_mges_matlab.m
t1 = [M0; M1; M2; M3; M4; M5; M6; M7; M8; M9; M10; M11; M12; M13;];
m = t1;

% Aus parameters_dyn_rSges_matlab.m
t1 = [SX0, SY0, SZ0; SX1, SY1, SZ1; SX2, SY2, SZ2; SX3, SY3, SZ3; SX4, SY4, SZ4; SX5, SY5, SZ5; SX6, SY6, SZ6; SX7, SY7, SZ7; SX8, SY8, SZ8; SX9, SY9, SZ9; SX10, SY10, SZ10; SX11, SY11, SZ11; SX12, SY12, SZ12; SX13, SY13, SZ13;];
rSges = t1;

% Aus parameters_dyn_Icges_matlab.m
t1 = [XXC0, YYC0, ZZC0, XYC0, XZC0, YZC0; XXC1, YYC1, ZZC1, XYC1, XZC1, YZC1; XXC2, YYC2, ZZC2, XYC2, XZC2, YZC2; XXC3, YYC3, ZZC3, XYC3, XZC3, YZC3; XXC4, YYC4, ZZC4, XYC4, XZC4, YZC4; XXC5, YYC5, ZZC5, XYC5, XZC5, YZC5; XXC6, YYC6, ZZC6, XYC6, XZC6, YZC6; XXC7, YYC7, ZZC7, XYC7, XZC7, YZC7; XXC8, YYC8, ZZC8, XYC8, XZC8, YZC8; XXC9, YYC9, ZZC9, XYC9, XZC9, YZC9; XXC10, YYC10, ZZC10, XYC10, XZC10, YZC10; XXC11, YYC11, ZZC11, XYC11, XZC11, YZC11; XXC12, YYC12, ZZC12, XYC12, XZC12, YZC12; XXC13, YYC13, ZZC13, XYC13, XZC13, YZC13;];
Icges = t1;
