% Calculate potential energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% r_base [3x1]
%   Base position in world frame
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

function U = KAS5m7OL_energypot_floatb_twist_slag_vp1(qJ, r_base, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: qJ has to be [13x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:20
% DurationCPUTime: 0.19s
% Computational Cost: add. (847->404), mult. (731->378), div. (0->0), fcn. (731->28), ass. (0->94)
t9 = sin(qJ(1));
t11 = cos(qJ(1));
t23 = t11 * pkin(11);
t24 = sin(qJ(2));
t25 = t9 * t24;
t27 = cos(qJ(2));
t28 = t9 * t27;
t33 = t9 * pkin(11);
t34 = t11 * t24;
t36 = t11 * t27;
t47 = t28 * pkin(16);
t48 = cos(qJ(3));
t50 = sin(qJ(3));
t51 = t11 * t50;
t61 = t36 * pkin(16);
t63 = t9 * t50;
t73 = t24 * pkin(16);
t74 = t27 * t48;
t83 = t48 * pkin(18);
t86 = qJ(3) + qJ(4);
t87 = cos(t86);
t89 = sin(t86);
t123 = pkin(6) * t87;
t124 = t123 + t83;
t125 = t25 * t124;
t126 = pkin(6) * t89;
t127 = t50 * pkin(18);
t128 = t126 + t127;
t129 = t11 * t128;
t130 = qJ(3) + qJ(4) + qJ(5);
t131 = cos(t130);
t133 = sin(t130);
t144 = t34 * t124;
t145 = t9 * t128;
t157 = t27 * t124;
t168 = pkin(7) * t131 + t123 + t83;
t169 = t25 * t168;
t171 = pkin(7) * t133 + t126 + t127;
t172 = t11 * t171;
t173 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t174 = sin(t173);
t176 = cos(t173);
t178 = t11 * t176 + t25 * t174;
t182 = -t11 * t174 + t25 * t176;
t187 = t34 * t168;
t188 = t9 * t171;
t191 = -t34 * t174 + t9 * t176;
t195 = -t9 * t174 - t34 * t176;
t200 = t27 * t168;
t201 = t27 * t174;
t203 = t27 * t176;
t211 = cos(qJ(7));
t213 = sin(qJ(7));
t250 = sin(pkin(3));
t252 = cos(pkin(3));
t253 = t11 * t252;
t264 = t9 * t252;
t274 = t27 * t250;
t283 = t250 * pkin(17);
t286 = pkin(3) + qJ(8);
t287 = sin(t286);
t289 = cos(t286);
t323 = t48 * pkin(19);
t326 = qJ(3) + qJ(9);
t327 = cos(t326);
t329 = sin(t326);
t364 = pkin(14) * t327 + t323;
t368 = pkin(14) * t329 + t50 * pkin(19);
t370 = qJ(3) + qJ(9) + qJ(10);
t371 = cos(t370);
t373 = sin(t370);
t407 = qJ(3) + qJ(4) + qJ(11);
t408 = sin(t407);
t410 = cos(t407);
t441 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t442 = cos(t441);
t444 = sin(t441);
t476 = -pkin(9) * t442 + t123 + t83;
t477 = t25 * t476;
t479 = -pkin(9) * t444 + t126 + t127;
t480 = t11 * t479;
t481 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t482 = sin(t481);
t484 = cos(t481);
t486 = t11 * t484 + t25 * t482;
t490 = -t11 * t482 + t25 * t484;
t495 = t34 * t476;
t496 = t9 * t479;
t499 = -t34 * t482 + t9 * t484;
t503 = -t34 * t484 - t9 * t482;
t508 = t27 * t476;
t509 = t27 * t482;
t511 = t27 * t484;
t538 = -m(1) * (g(1) * (r_base(1) + rSges(1,1)) + g(2) * (r_base(2) + rSges(1,2)) + g(3) * (r_base(3) + rSges(1,3))) - m(2) * (g(1) * (t9 * rSges(2,1) + t11 * rSges(2,2) + r_base(1)) + g(2) * (-t11 * rSges(2,1) + t9 * rSges(2,2) + r_base(2)) + g(3) * (pkin(5) + r_base(3) + rSges(2,3))) - m(3) * (g(1) * (-t25 * rSges(3,1) - t28 * rSges(3,2) + t11 * rSges(3,3) + t23 + r_base(1)) + g(2) * (t34 * rSges(3,1) + t36 * rSges(3,2) + t9 * rSges(3,3) + t33 + r_base(2)) + g(3) * (-t27 * rSges(3,1) + t24 * rSges(3,2) + pkin(5) + r_base(3))) - m(4) * (g(1) * (t47 + t23 + r_base(1) + (-t25 * t48 + t51) * rSges(4,1) + (t11 * t48 + t25 * t50) * rSges(4,2) + t28 * rSges(4,3)) + g(2) * (-t61 + t33 + r_base(2) + (t34 * t48 + t63) * rSges(4,1) + (-t34 * t50 + t9 * t48) * rSges(4,2) - t36 * rSges(4,3)) + g(3) * (t27 * t50 * rSges(4,2) - t74 * rSges(4,1) - t24 * rSges(4,3) + pkin(5) - t73 + r_base(3))) - m(5) * (g(1) * (-t25 * t83 + t47 + t51 * pkin(18) + t23 + r_base(1) + (t11 * t89 - t25 * t87) * rSges(5,1) + (t11 * t87 + t25 * t89) * rSges(5,2) + t28 * rSges(5,3)) + g(2) * (t34 * t83 - t61 + t63 * pkin(18) + t33 + r_base(2) + (t34 * t87 + t9 * t89) * rSges(5,1) + (-t34 * t89 + t9 * t87) * rSges(5,2) - t36 * rSges(5,3)) + g(3) * (-t27 * t87 * rSges(5,1) + t27 * t89 * rSges(5,2) - t24 * rSges(5,3) - t74 * pkin(18) + pkin(5) - t73 + r_base(3))) - m(6) * (g(1) * (-t125 + t47 + t129 + t23 + r_base(1) + (t11 * t133 - t25 * t131) * rSges(6,1) + (t11 * t131 + t25 * t133) * rSges(6,2) + t28 * rSges(6,3)) + g(2) * (t144 - t61 + t145 + t33 + r_base(2) + (t34 * t131 + t9 * t133) * rSges(6,1) + (t9 * t131 - t34 * t133) * rSges(6,2) - t36 * rSges(6,3)) + g(3) * (-t27 * t131 * rSges(6,1) + t27 * t133 * rSges(6,2) - t24 * rSges(6,3) + pkin(5) - t157 - t73 + r_base(3))) - m(7) * (g(1) * (t178 * rSges(7,1) + t182 * rSges(7,2) + t28 * rSges(7,3) - t169 + t172 + t23 + t47 + r_base(1)) + g(2) * (t191 * rSges(7,1) + t195 * rSges(7,2) - t36 * rSges(7,3) + t187 + t188 + t33 - t61 + r_base(2)) + g(3) * (t201 * rSges(7,1) + t203 * rSges(7,2) - t24 * rSges(7,3) + pkin(5) - t200 - t73 + r_base(3))) - m(8) * (g(1) * (-t182 * pkin(10) - t169 + t47 + t172 + t23 + r_base(1) + (t178 * t211 + t28 * t213) * rSges(8,1) + (-t178 * t213 + t28 * t211) * rSges(8,2) - t182 * rSges(8,3)) + g(2) * (-t195 * pkin(10) + t187 - t61 + t188 + t33 + r_base(2) + (t191 * t211 - t36 * t213) * rSges(8,1) + (-t191 * t213 - t36 * t211) * rSges(8,2) - t195 * rSges(8,3)) + g(3) * (-t203 * pkin(10) - t200 - t73 + pkin(5) + r_base(3) + (t201 * t211 - t24 * t213) * rSges(8,1) + (-t201 * t213 - t24 * t211) * rSges(8,2) - t203 * rSges(8,3))) - m(9) * (g(1) * (t47 + t23 + r_base(1) + (t25 * t250 + t253) * rSges(9,1) + (-t11 * t250 + t25 * t252) * rSges(9,2) + t28 * rSges(9,3)) + g(2) * (-t61 + t33 + r_base(2) + (-t34 * t250 + t264) * rSges(9,1) + (-t9 * t250 - t34 * t252) * rSges(9,2) - t36 * rSges(9,3)) + g(3) * (t27 * t252 * rSges(9,2) + t274 * rSges(9,1) - t24 * rSges(9,3) + pkin(5) - t73 + r_base(3))) - m(10) * (g(1) * (-t25 * t283 + t47 - t253 * pkin(17) + t23 + r_base(1) + (t11 * t289 + t25 * t287) * rSges(10,1) + (-t11 * t287 + t25 * t289) * rSges(10,2) + t28 * rSges(10,3)) + g(2) * (t34 * t283 - t61 - t264 * pkin(17) + t33 + r_base(2) + (-t34 * t287 + t9 * t289) * rSges(10,1) + (-t9 * t287 - t34 * t289) * rSges(10,2) - t36 * rSges(10,3)) + g(3) * (t27 * t287 * rSges(10,1) + t27 * t289 * rSges(10,2) - t24 * rSges(10,3) - t274 * pkin(17) + pkin(5) - t73 + r_base(3))) - m(11) * (g(1) * (-t25 * t323 + t47 + t51 * pkin(19) + t23 + r_base(1) + (t11 * t329 - t25 * t327) * rSges(11,1) + (t11 * t327 + t25 * t329) * rSges(11,2) + t28 * rSges(11,3)) + g(2) * (t34 * t323 - t61 + t63 * pkin(19) + t33 + r_base(2) + (t34 * t327 + t9 * t329) * rSges(11,1) + (t9 * t327 - t34 * t329) * rSges(11,2) - t36 * rSges(11,3)) + g(3) * (-t27 * t327 * rSges(11,1) + t27 * t329 * rSges(11,2) - t24 * rSges(11,3) - t74 * pkin(19) + pkin(5) - t73 + r_base(3))) - m(12) * (g(1) * (-t25 * t364 + t47 + t11 * t368 + t23 + r_base(1) + (-t11 * t373 + t25 * t371) * rSges(12,1) + (-t11 * t371 - t25 * t373) * rSges(12,2) + t28 * rSges(12,3)) + g(2) * (t34 * t364 - t61 + t9 * t368 + t33 + r_base(2) + (-t34 * t371 - t9 * t373) * rSges(12,1) + (t34 * t373 - t9 * t371) * rSges(12,2) - t36 * rSges(12,3)) + g(3) * (t27 * t371 * rSges(12,1) - t27 * t373 * rSges(12,2) - t24 * rSges(12,3) - t27 * t364 + pkin(5) - t73 + r_base(3))) - m(13) * (g(1) * (-t125 + t47 + t129 + t23 + r_base(1) + (-t11 * t410 - t25 * t408) * rSges(13,1) + (t11 * t408 - t25 * t410) * rSges(13,2) + t28 * rSges(13,3)) + g(2) * (t144 - t61 + t145 + t33 + r_base(2) + (t34 * t408 - t9 * t410) * rSges(13,1) + (t34 * t410 + t9 * t408) * rSges(13,2) - t36 * rSges(13,3)) + g(3) * (-t27 * t408 * rSges(13,1) - t27 * t410 * rSges(13,2) - t24 * rSges(13,3) + pkin(5) - t157 - t73 + r_base(3))) - m(14) * (g(1) * (-t125 + t47 + t129 + t23 + r_base(1) + (-t11 * t444 + t25 * t442) * rSges(14,1) + (-t11 * t442 - t25 * t444) * rSges(14,2) + t28 * rSges(14,3)) + g(2) * (t144 - t61 + t145 + t33 + r_base(2) + (-t34 * t442 - t9 * t444) * rSges(14,1) + (t34 * t444 - t9 * t442) * rSges(14,2) - t36 * rSges(14,3)) + g(3) * (t27 * t442 * rSges(14,1) - t27 * t444 * rSges(14,2) - t24 * rSges(14,3) + pkin(5) - t157 - t73 + r_base(3))) - m(15) * (g(1) * (t486 * rSges(15,1) + t490 * rSges(15,2) + t28 * rSges(15,3) + t23 + t47 - t477 + t480 + r_base(1)) + g(2) * (t499 * rSges(15,1) + t503 * rSges(15,2) - t36 * rSges(15,3) + t33 + t495 + t496 - t61 + r_base(2)) + g(3) * (t509 * rSges(15,1) + t511 * rSges(15,2) - t24 * rSges(15,3) + pkin(5) - t508 - t73 + r_base(3))) - m(16) * (g(1) * (t486 * rSges(16,1) + t28 * rSges(16,2) - t490 * rSges(16,3) - t490 * qJ(13) + t23 + t47 - t477 + t480 + r_base(1)) + g(2) * (t499 * rSges(16,1) - t36 * rSges(16,2) - t503 * rSges(16,3) - t503 * qJ(13) + t33 + t495 + t496 - t61 + r_base(2)) + g(3) * (t509 * rSges(16,1) - t24 * rSges(16,2) - t511 * rSges(16,3) - t511 * qJ(13) + pkin(5) - t508 - t73 + r_base(3)));
U = t538;
