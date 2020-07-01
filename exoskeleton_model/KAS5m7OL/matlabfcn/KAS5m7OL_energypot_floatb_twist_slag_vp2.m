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
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7OL_energypot_floatb_twist_slag_vp2(qJ, r_base, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_energypot_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:20
% DurationCPUTime: 0.20s
% Computational Cost: add. (847->375), mult. (763->410), div. (0->0), fcn. (731->28), ass. (0->100)
t1 = sin(qJ(1));
t2 = cos(qJ(2));
t3 = t1 * t2;
t4 = t3 * pkin(16);
t5 = cos(qJ(1));
t6 = t5 * pkin(11);
t7 = t4 + t6 + r_base(1);
t9 = sin(qJ(2));
t10 = t1 * t9;
t11 = cos(qJ(3));
t13 = sin(qJ(3));
t14 = t5 * t13;
t24 = t9 * pkin(16);
t25 = -t24 + pkin(5) + r_base(3);
t27 = sin(pkin(3));
t28 = t2 * t27;
t30 = cos(pkin(3));
t36 = pkin(5) + r_base(3);
t43 = t2 * t11;
t59 = t5 * t30;
t69 = t5 * t2;
t70 = t69 * pkin(16);
t71 = t1 * pkin(11);
t72 = -t70 + t71 + r_base(2);
t74 = t5 * t9;
t76 = t1 * t30;
t109 = -g(1) * (m(4) * t7 + (-t10 * t11 + t14) * mrSges(4,1) + (t10 * t13 + t5 * t11) * mrSges(4,2) + t3 * mrSges(4,3)) - g(3) * (t2 * t30 * mrSges(9,2) + m(9) * t25 + t28 * mrSges(9,1) - t9 * mrSges(9,3)) - g(3) * (m(3) * t36 - t2 * mrSges(3,1) + t9 * mrSges(3,2)) - g(3) * (t2 * t13 * mrSges(4,2) + m(4) * t25 - t43 * mrSges(4,1) - t9 * mrSges(4,3)) - g(1) * (m(3) * (t6 + r_base(1)) - t10 * mrSges(3,1) - t3 * mrSges(3,2) + t5 * mrSges(3,3)) - g(1) * (m(9) * t7 + (t10 * t27 + t59) * mrSges(9,1) + (t10 * t30 - t5 * t27) * mrSges(9,2) + t3 * mrSges(9,3)) - g(2) * (m(9) * t72 + (-t74 * t27 + t76) * mrSges(9,1) + (-t1 * t27 - t74 * t30) * mrSges(9,2) - t69 * mrSges(9,3)) - g(2) * (m(3) * (t71 + r_base(2)) + t74 * mrSges(3,1) + t69 * mrSges(3,2) + t1 * mrSges(3,3)) - g(2) * (m(2) * r_base(2) - t5 * mrSges(2,1) + t1 * mrSges(2,2)) - g(1) * (m(2) * r_base(1) + t1 * mrSges(2,1) + t5 * mrSges(2,2)) - g(1) * (m(1) * r_base(1) + mrSges(1,1)) - g(2) * (m(1) * r_base(2) + mrSges(1,2));
t116 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t117 = cos(t116);
t118 = t2 * t117;
t120 = qJ(3) + qJ(4) + qJ(5);
t121 = cos(t120);
t123 = qJ(3) + qJ(4);
t124 = cos(t123);
t125 = pkin(6) * t124;
t126 = t11 * pkin(18);
t127 = pkin(7) * t121 + t125 + t126;
t128 = t2 * t127;
t131 = sin(t116);
t132 = t2 * t131;
t133 = cos(qJ(7));
t135 = sin(qJ(7));
t148 = t10 * t117 - t5 * t131;
t150 = t10 * t127;
t151 = sin(t120);
t153 = sin(t123);
t154 = pkin(6) * t153;
t155 = t13 * pkin(18);
t156 = pkin(7) * t151 + t154 + t155;
t157 = t5 * t156;
t162 = t10 * t131 + t5 * t117;
t177 = pkin(3) + qJ(8);
t178 = sin(t177);
t181 = cos(t177);
t187 = t27 * pkin(17);
t218 = t11 * pkin(19);
t220 = t1 * t13;
t224 = qJ(3) + qJ(9);
t225 = cos(t224);
t227 = sin(t224);
t263 = t125 + t126;
t265 = t154 + t155;
t267 = -t10 * t263 + t5 * t265 + t4 + t6 + r_base(1);
t281 = -t2 * t263 + pkin(5) - t24 + r_base(3);
t290 = -g(3) * (m(2) * t36 + mrSges(2,3)) - g(3) * (m(1) * r_base(3) + mrSges(1,3)) - g(3) * (m(8) * (-t118 * pkin(10) + pkin(5) - t128 - t24 + r_base(3)) + (t132 * t133 - t9 * t135) * mrSges(8,1) + (-t132 * t135 - t9 * t133) * mrSges(8,2) - t118 * mrSges(8,3)) - g(1) * (m(8) * (-t148 * pkin(10) - t150 + t157 + t4 + t6 + r_base(1)) + (t162 * t133 + t3 * t135) * mrSges(8,1) + (t3 * t133 - t162 * t135) * mrSges(8,2) - t148 * mrSges(8,3)) - g(3) * (m(10) * (-t28 * pkin(17) + pkin(5) - t24 + r_base(3)) + t2 * t178 * mrSges(10,1) + t2 * t181 * mrSges(10,2) - t9 * mrSges(10,3)) - g(2) * (m(10) * (-t76 * pkin(17) + t74 * t187 - t70 + t71 + r_base(2)) + (t1 * t181 - t74 * t178) * mrSges(10,1) + (-t1 * t178 - t74 * t181) * mrSges(10,2) - t69 * mrSges(10,3)) - g(1) * (m(10) * (-t59 * pkin(17) - t10 * t187 + t4 + t6 + r_base(1)) + (t10 * t178 + t5 * t181) * mrSges(10,1) + (t10 * t181 - t5 * t178) * mrSges(10,2) + t3 * mrSges(10,3)) - g(2) * (m(11) * (t220 * pkin(19) + t74 * t218 - t70 + t71 + r_base(2)) + (t1 * t227 + t74 * t225) * mrSges(11,1) + (t1 * t225 - t74 * t227) * mrSges(11,2) - t69 * mrSges(11,3)) - g(3) * (m(11) * (-t43 * pkin(19) + pkin(5) - t24 + r_base(3)) - t2 * t225 * mrSges(11,1) + t2 * t227 * mrSges(11,2) - t9 * mrSges(11,3)) - g(1) * (m(5) * (t14 * pkin(18) - t10 * t126 + t4 + t6 + r_base(1)) + (-t10 * t124 + t5 * t153) * mrSges(5,1) + (t10 * t153 + t5 * t124) * mrSges(5,2) + t3 * mrSges(5,3)) - g(1) * (m(6) * t267 + (-t10 * t121 + t5 * t151) * mrSges(6,1) + (t10 * t151 + t5 * t121) * mrSges(6,2) + t3 * mrSges(6,3)) - g(3) * (-t2 * t121 * mrSges(6,1) + t2 * t151 * mrSges(6,2) + m(6) * t281 - t9 * mrSges(6,3));
t294 = t1 * t265 + t74 * t263 - t70 + t71 + r_base(2);
t351 = pkin(14) * t225 + t218;
t355 = pkin(14) * t227 + t13 * pkin(19);
t359 = qJ(3) + qJ(9) + qJ(10);
t360 = cos(t359);
t362 = sin(t359);
t399 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t400 = cos(t399);
t402 = sin(t399);
t414 = qJ(3) + qJ(4) + qJ(11);
t415 = sin(t414);
t417 = cos(t414);
t452 = -g(2) * (m(6) * t294 + (t1 * t151 + t74 * t121) * mrSges(6,1) + (t1 * t121 - t74 * t151) * mrSges(6,2) - t69 * mrSges(6,3)) - g(1) * (m(7) * (-t150 + t4 + t157 + t6 + r_base(1)) + t162 * mrSges(7,1) + t148 * mrSges(7,2) + t3 * mrSges(7,3)) - g(3) * (m(5) * (-t43 * pkin(18) + pkin(5) - t24 + r_base(3)) - t2 * t124 * mrSges(5,1) + t2 * t153 * mrSges(5,2) - t9 * mrSges(5,3)) - g(2) * (m(4) * t72 + (t74 * t11 + t220) * mrSges(4,1) + (t1 * t11 - t74 * t13) * mrSges(4,2) - t69 * mrSges(4,3)) - g(2) * (m(5) * (t220 * pkin(18) + t74 * t126 - t70 + t71 + r_base(2)) + (t1 * t153 + t74 * t124) * mrSges(5,1) + (t1 * t124 - t74 * t153) * mrSges(5,2) - t69 * mrSges(5,3)) - g(1) * (m(12) * (-t10 * t351 + t5 * t355 + t4 + t6 + r_base(1)) + (t10 * t360 - t5 * t362) * mrSges(12,1) + (-t10 * t362 - t5 * t360) * mrSges(12,2) + t3 * mrSges(12,3)) - g(2) * (m(12) * (t1 * t355 + t74 * t351 - t70 + t71 + r_base(2)) + (-t1 * t362 - t74 * t360) * mrSges(12,1) + (-t1 * t360 + t74 * t362) * mrSges(12,2) - t69 * mrSges(12,3)) - g(3) * (m(12) * (-t2 * t351 + pkin(5) - t24 + r_base(3)) + t2 * t360 * mrSges(12,1) - t2 * t362 * mrSges(12,2) - t9 * mrSges(12,3)) - g(1) * (m(14) * t267 + (t10 * t400 - t5 * t402) * mrSges(14,1) + (-t10 * t402 - t5 * t400) * mrSges(14,2) + t3 * mrSges(14,3)) - g(1) * (m(13) * t267 + (-t10 * t415 - t5 * t417) * mrSges(13,1) + (-t10 * t417 + t5 * t415) * mrSges(13,2) + t3 * mrSges(13,3)) - g(2) * (m(14) * t294 + (-t1 * t402 - t74 * t400) * mrSges(14,1) + (-t1 * t400 + t74 * t402) * mrSges(14,2) - t69 * mrSges(14,3)) - g(2) * (m(13) * t294 + (-t1 * t417 + t74 * t415) * mrSges(13,1) + (t1 * t415 + t74 * t417) * mrSges(13,2) - t69 * mrSges(13,3));
t470 = -pkin(9) * t400 + t125 + t126;
t471 = t74 * t470;
t473 = -pkin(9) * t402 + t154 + t155;
t474 = t1 * t473;
t477 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t478 = sin(t477);
t480 = cos(t477);
t482 = t1 * t480 - t74 * t478;
t486 = -t1 * t478 - t74 * t480;
t491 = t2 * t470;
t494 = t2 * t478;
t496 = t2 * t480;
t501 = t74 * t127;
t502 = t1 * t156;
t507 = t1 * t117 - t74 * t131;
t511 = -t1 * t131 - t74 * t117;
t537 = t10 * t470;
t538 = t5 * t473;
t543 = t10 * t478 + t5 * t480;
t547 = t10 * t480 - t5 * t478;
t591 = -g(3) * (t2 * t400 * mrSges(14,1) - t2 * t402 * mrSges(14,2) + m(14) * t281 - t9 * mrSges(14,3)) - g(3) * (-t2 * t415 * mrSges(13,1) - t2 * t417 * mrSges(13,2) + m(13) * t281 - t9 * mrSges(13,3)) - g(2) * (m(15) * (t471 - t70 + t474 + t71 + r_base(2)) + t482 * mrSges(15,1) + t486 * mrSges(15,2) - t69 * mrSges(15,3)) - g(3) * (m(15) * (-t491 - t24 + pkin(5) + r_base(3)) + t494 * mrSges(15,1) + t496 * mrSges(15,2) - t9 * mrSges(15,3)) - g(2) * (m(7) * (t501 - t70 + t502 + t71 + r_base(2)) + t507 * mrSges(7,1) + t511 * mrSges(7,2) - t69 * mrSges(7,3)) - g(3) * (m(7) * (-t128 - t24 + pkin(5) + r_base(3)) + t132 * mrSges(7,1) + t118 * mrSges(7,2) - t9 * mrSges(7,3)) - g(2) * (m(8) * (-t511 * pkin(10) + t501 + t502 - t70 + t71 + r_base(2)) + (t507 * t133 - t69 * t135) * mrSges(8,1) + (-t69 * t133 - t507 * t135) * mrSges(8,2) - t511 * mrSges(8,3)) - g(1) * (m(15) * (-t537 + t4 + t538 + t6 + r_base(1)) + t543 * mrSges(15,1) + t547 * mrSges(15,2) + t3 * mrSges(15,3)) - g(1) * (m(16) * (-t547 * qJ(13) + t4 - t537 + t538 + t6 + r_base(1)) + t543 * mrSges(16,1) + t3 * mrSges(16,2) - t547 * mrSges(16,3)) - g(2) * (m(16) * (-t486 * qJ(13) + t471 + t474 - t70 + t71 + r_base(2)) + t482 * mrSges(16,1) - t69 * mrSges(16,2) - t486 * mrSges(16,3)) - g(3) * (m(16) * (-t496 * qJ(13) + pkin(5) - t24 - t491 + r_base(3)) + t494 * mrSges(16,1) - t9 * mrSges(16,2) - t496 * mrSges(16,3)) - g(1) * (m(11) * (t14 * pkin(19) - t10 * t218 + t4 + t6 + r_base(1)) + (-t10 * t225 + t5 * t227) * mrSges(11,1) + (t10 * t227 + t5 * t225) * mrSges(11,2) + t3 * mrSges(11,3));
t593 = t109 + t290 + t452 + t591;
U = t593;
