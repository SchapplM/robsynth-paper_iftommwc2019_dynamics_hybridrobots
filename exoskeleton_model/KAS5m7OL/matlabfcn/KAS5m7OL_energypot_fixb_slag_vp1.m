% Calculate potential energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7OL_energypot_fixb_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp1: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp1: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7OL_energypot_fixb_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:20
% DurationCPUTime: 0.14s
% Computational Cost: add. (799->356), mult. (731->378), div. (0->0), fcn. (731->28), ass. (0->94)
t6 = sin(qJ(1));
t8 = cos(qJ(1));
t20 = t8 * pkin(11);
t21 = sin(qJ(2));
t22 = t6 * t21;
t24 = cos(qJ(2));
t25 = t6 * t24;
t30 = t6 * pkin(11);
t31 = t8 * t21;
t33 = t8 * t24;
t44 = t25 * pkin(16);
t45 = cos(qJ(3));
t47 = sin(qJ(3));
t48 = t8 * t47;
t58 = t33 * pkin(16);
t60 = t6 * t47;
t70 = t21 * pkin(16);
t71 = t24 * t45;
t80 = t45 * pkin(18);
t83 = qJ(3) + qJ(4);
t84 = cos(t83);
t86 = sin(t83);
t120 = pkin(6) * t84;
t121 = t120 + t80;
t122 = t22 * t121;
t123 = pkin(6) * t86;
t124 = t47 * pkin(18);
t125 = t123 + t124;
t126 = t8 * t125;
t127 = qJ(3) + qJ(4) + qJ(5);
t128 = cos(t127);
t130 = sin(t127);
t141 = t31 * t121;
t142 = t6 * t125;
t154 = t24 * t121;
t165 = pkin(7) * t128 + t120 + t80;
t166 = t22 * t165;
t168 = pkin(7) * t130 + t123 + t124;
t169 = t8 * t168;
t170 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t171 = sin(t170);
t173 = cos(t170);
t175 = t22 * t171 + t8 * t173;
t179 = -t8 * t171 + t22 * t173;
t184 = t31 * t165;
t185 = t6 * t168;
t188 = -t31 * t171 + t6 * t173;
t192 = -t6 * t171 - t31 * t173;
t197 = t24 * t165;
t198 = t24 * t171;
t200 = t24 * t173;
t208 = cos(qJ(7));
t210 = sin(qJ(7));
t247 = sin(pkin(3));
t249 = cos(pkin(3));
t250 = t8 * t249;
t261 = t6 * t249;
t271 = t24 * t247;
t280 = t247 * pkin(17);
t283 = pkin(3) + qJ(8);
t284 = sin(t283);
t286 = cos(t283);
t320 = t45 * pkin(19);
t323 = qJ(3) + qJ(9);
t324 = cos(t323);
t326 = sin(t323);
t361 = pkin(14) * t324 + t320;
t365 = pkin(14) * t326 + t47 * pkin(19);
t367 = qJ(3) + qJ(9) + qJ(10);
t368 = cos(t367);
t370 = sin(t367);
t404 = qJ(3) + qJ(4) + qJ(11);
t405 = sin(t404);
t407 = cos(t404);
t438 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t439 = cos(t438);
t441 = sin(t438);
t473 = -pkin(9) * t439 + t120 + t80;
t474 = t22 * t473;
t476 = -pkin(9) * t441 + t123 + t124;
t477 = t8 * t476;
t478 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t479 = sin(t478);
t481 = cos(t478);
t483 = t22 * t479 + t8 * t481;
t487 = t22 * t481 - t8 * t479;
t492 = t31 * t473;
t493 = t6 * t476;
t496 = -t31 * t479 + t6 * t481;
t500 = -t31 * t481 - t6 * t479;
t505 = t24 * t473;
t506 = t24 * t479;
t508 = t24 * t481;
t535 = -m(1) * (g(1) * rSges(1,1) + g(2) * rSges(1,2) + g(3) * rSges(1,3)) - m(2) * (g(1) * (t6 * rSges(2,1) + t8 * rSges(2,2)) + g(2) * (-t8 * rSges(2,1) + t6 * rSges(2,2)) + g(3) * (pkin(5) + rSges(2,3))) - m(3) * (g(1) * (-t22 * rSges(3,1) - t25 * rSges(3,2) + t8 * rSges(3,3) + t20) + g(2) * (t31 * rSges(3,1) + t33 * rSges(3,2) + t6 * rSges(3,3) + t30) + g(3) * (-t24 * rSges(3,1) + t21 * rSges(3,2) + pkin(5))) - m(4) * (g(1) * (t44 + t20 + (-t22 * t45 + t48) * rSges(4,1) + (t22 * t47 + t8 * t45) * rSges(4,2) + t25 * rSges(4,3)) + g(2) * (-t58 + t30 + (t31 * t45 + t60) * rSges(4,1) + (-t31 * t47 + t6 * t45) * rSges(4,2) - t33 * rSges(4,3)) + g(3) * (t24 * t47 * rSges(4,2) - t71 * rSges(4,1) - t21 * rSges(4,3) + pkin(5) - t70)) - m(5) * (g(1) * (-t22 * t80 + t44 + t48 * pkin(18) + t20 + (-t22 * t84 + t8 * t86) * rSges(5,1) + (t22 * t86 + t8 * t84) * rSges(5,2) + t25 * rSges(5,3)) + g(2) * (t31 * t80 - t58 + t60 * pkin(18) + t30 + (t31 * t84 + t6 * t86) * rSges(5,1) + (-t31 * t86 + t6 * t84) * rSges(5,2) - t33 * rSges(5,3)) + g(3) * (-t24 * t84 * rSges(5,1) + t24 * t86 * rSges(5,2) - t21 * rSges(5,3) - t71 * pkin(18) + pkin(5) - t70)) - m(6) * (g(1) * (-t122 + t44 + t126 + t20 + (-t22 * t128 + t8 * t130) * rSges(6,1) + (t8 * t128 + t22 * t130) * rSges(6,2) + t25 * rSges(6,3)) + g(2) * (t141 - t58 + t142 + t30 + (t31 * t128 + t6 * t130) * rSges(6,1) + (t6 * t128 - t31 * t130) * rSges(6,2) - t33 * rSges(6,3)) + g(3) * (-t24 * t128 * rSges(6,1) + t24 * t130 * rSges(6,2) - t21 * rSges(6,3) + pkin(5) - t154 - t70)) - m(7) * (g(1) * (t175 * rSges(7,1) + t179 * rSges(7,2) + t25 * rSges(7,3) - t166 + t169 + t20 + t44) + g(2) * (t188 * rSges(7,1) + t192 * rSges(7,2) - t33 * rSges(7,3) + t184 + t185 + t30 - t58) + g(3) * (t198 * rSges(7,1) + t200 * rSges(7,2) - t21 * rSges(7,3) + pkin(5) - t197 - t70)) - m(8) * (g(1) * (-t179 * pkin(10) - t166 + t44 + t169 + t20 + (t175 * t208 + t25 * t210) * rSges(8,1) + (-t175 * t210 + t25 * t208) * rSges(8,2) - t179 * rSges(8,3)) + g(2) * (-t192 * pkin(10) + t184 - t58 + t185 + t30 + (t188 * t208 - t33 * t210) * rSges(8,1) + (-t188 * t210 - t33 * t208) * rSges(8,2) - t192 * rSges(8,3)) + g(3) * (-t200 * pkin(10) - t197 - t70 + pkin(5) + (t198 * t208 - t21 * t210) * rSges(8,1) + (-t198 * t210 - t21 * t208) * rSges(8,2) - t200 * rSges(8,3))) - m(9) * (g(1) * (t44 + t20 + (t22 * t247 + t250) * rSges(9,1) + (t22 * t249 - t8 * t247) * rSges(9,2) + t25 * rSges(9,3)) + g(2) * (-t58 + t30 + (-t31 * t247 + t261) * rSges(9,1) + (-t6 * t247 - t31 * t249) * rSges(9,2) - t33 * rSges(9,3)) + g(3) * (t24 * t249 * rSges(9,2) + t271 * rSges(9,1) - t21 * rSges(9,3) + pkin(5) - t70)) - m(10) * (g(1) * (-t22 * t280 + t44 - t250 * pkin(17) + t20 + (t22 * t284 + t8 * t286) * rSges(10,1) + (t22 * t286 - t8 * t284) * rSges(10,2) + t25 * rSges(10,3)) + g(2) * (t31 * t280 - t58 - t261 * pkin(17) + t30 + (-t31 * t284 + t6 * t286) * rSges(10,1) + (-t6 * t284 - t31 * t286) * rSges(10,2) - t33 * rSges(10,3)) + g(3) * (t24 * t284 * rSges(10,1) + t24 * t286 * rSges(10,2) - t21 * rSges(10,3) - t271 * pkin(17) + pkin(5) - t70)) - m(11) * (g(1) * (-t22 * t320 + t44 + t48 * pkin(19) + t20 + (-t22 * t324 + t8 * t326) * rSges(11,1) + (t22 * t326 + t8 * t324) * rSges(11,2) + t25 * rSges(11,3)) + g(2) * (t31 * t320 - t58 + t60 * pkin(19) + t30 + (t31 * t324 + t6 * t326) * rSges(11,1) + (-t31 * t326 + t6 * t324) * rSges(11,2) - t33 * rSges(11,3)) + g(3) * (-t24 * t324 * rSges(11,1) + t24 * t326 * rSges(11,2) - t21 * rSges(11,3) - t71 * pkin(19) + pkin(5) - t70)) - m(12) * (g(1) * (-t22 * t361 + t44 + t8 * t365 + t20 + (t22 * t368 - t8 * t370) * rSges(12,1) + (-t22 * t370 - t8 * t368) * rSges(12,2) + t25 * rSges(12,3)) + g(2) * (t31 * t361 - t58 + t6 * t365 + t30 + (-t31 * t368 - t6 * t370) * rSges(12,1) + (t31 * t370 - t6 * t368) * rSges(12,2) - t33 * rSges(12,3)) + g(3) * (t24 * t368 * rSges(12,1) - t24 * t370 * rSges(12,2) - t21 * rSges(12,3) - t24 * t361 + pkin(5) - t70)) - m(13) * (g(1) * (-t122 + t44 + t126 + t20 + (-t22 * t405 - t8 * t407) * rSges(13,1) + (-t22 * t407 + t8 * t405) * rSges(13,2) + t25 * rSges(13,3)) + g(2) * (t141 - t58 + t142 + t30 + (t31 * t405 - t6 * t407) * rSges(13,1) + (t31 * t407 + t6 * t405) * rSges(13,2) - t33 * rSges(13,3)) + g(3) * (-t24 * t405 * rSges(13,1) - t24 * t407 * rSges(13,2) - t21 * rSges(13,3) + pkin(5) - t154 - t70)) - m(14) * (g(1) * (-t122 + t44 + t126 + t20 + (t22 * t439 - t8 * t441) * rSges(14,1) + (-t22 * t441 - t8 * t439) * rSges(14,2) + t25 * rSges(14,3)) + g(2) * (t141 - t58 + t142 + t30 + (-t31 * t439 - t6 * t441) * rSges(14,1) + (t31 * t441 - t6 * t439) * rSges(14,2) - t33 * rSges(14,3)) + g(3) * (t24 * t439 * rSges(14,1) - t24 * t441 * rSges(14,2) - t21 * rSges(14,3) + pkin(5) - t154 - t70)) - m(15) * (g(1) * (t483 * rSges(15,1) + t487 * rSges(15,2) + t25 * rSges(15,3) + t20 + t44 - t474 + t477) + g(2) * (t496 * rSges(15,1) + t500 * rSges(15,2) - t33 * rSges(15,3) + t30 + t492 + t493 - t58) + g(3) * (t506 * rSges(15,1) + t508 * rSges(15,2) - t21 * rSges(15,3) + pkin(5) - t505 - t70)) - m(16) * (g(1) * (t483 * rSges(16,1) + t25 * rSges(16,2) - t487 * rSges(16,3) - t487 * qJ(13) + t20 + t44 - t474 + t477) + g(2) * (t496 * rSges(16,1) - t33 * rSges(16,2) - t500 * rSges(16,3) - t500 * qJ(13) + t30 + t492 + t493 - t58) + g(3) * (t506 * rSges(16,1) - t21 * rSges(16,2) - t508 * rSges(16,3) - t508 * qJ(13) + pkin(5) - t505 - t70));
U = t535;
