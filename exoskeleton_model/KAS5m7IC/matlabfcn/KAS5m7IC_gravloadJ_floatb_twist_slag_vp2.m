% Calculate Gravitation load on the joints for
% KAS5m7IC
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% taug [5x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7IC_gravloadJ_floatb_twist_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(20,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp2: pkin has to be [20x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From gravload_joint_floatb_twist_par2_ic_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:50:08
% EndTime: 2020-06-30 18:50:09
% DurationCPUTime: 0.62s
% Computational Cost: add. (3558->964), mult. (2840->1087), div. (12->7), fcn. (2742->65), ass. (0->370)
unknown=NaN(5,1);
t1 = sin(qJ(1));
t2 = cos(qJ(2));
t3 = t1 * t2;
t4 = t3 * pkin(17);
t5 = cos(qJ(1));
t6 = t5 * pkin(12);
t7 = t4 + t6;
t9 = sin(qJ(2));
t10 = t1 * t9;
t11 = sin(pkin(4));
t13 = cos(pkin(4));
t14 = t5 * t13;
t32 = cos(qJ(3));
t33 = t32 * pkin(19);
t35 = sin(qJ(3));
t36 = t5 * t35;
t40 = qJ(3) + qJ(4);
t41 = cos(t40);
t43 = sin(t40);
t45 = -t10 * t41 + t43 * t5;
t49 = t10 * t43 + t41 * t5;
t54 = t11 * pkin(18);
t59 = pkin(4) + qJ(8);
t60 = sin(t59);
t62 = cos(t59);
t64 = t10 * t60 + t5 * t62;
t68 = t10 * t62 - t5 * t60;
t73 = t5 * t9;
t75 = t5 * t2;
t76 = t75 * pkin(17);
t77 = t1 * t13;
t79 = t1 * pkin(12);
t84 = -t1 * t62 + t60 * t73;
t88 = t1 * t60 + t62 * t73;
t93 = t32 * pkin(20);
t95 = t1 * t35;
t99 = qJ(3) + qJ(9);
t100 = cos(t99);
t102 = sin(t99);
t104 = -t1 * t102 - t100 * t73;
t108 = -t1 * t100 + t102 * t73;
t119 = -t10 * t100 + t102 * t5;
t123 = t10 * t102 + t100 * t5;
t129 = pkin(15) * t100 + t93;
t131 = pkin(15) * t102;
t132 = t35 * pkin(20);
t133 = t131 + t132;
t137 = qJ(3) + qJ(9) + qJ(10);
t138 = cos(t137);
t140 = sin(t137);
t142 = t1 * t140 + t138 * t73;
t146 = t1 * t138 - t140 * t73;
t151 = t76 - t79;
t154 = -t32 * t73 - t95;
t157 = t1 * t32;
t158 = t35 * t73 - t157;
t165 = -t10 * t32 + t36;
t168 = t5 * t32;
t169 = t10 * t35 + t168;
t199 = pkin(7) * t41;
t200 = t199 + t33;
t202 = pkin(7) * t43;
t203 = t35 * pkin(19);
t204 = t202 + t203;
t206 = -t1 * t204 - t200 * t73 + t76 - t79;
t208 = qJ(3) + qJ(4) + qJ(5);
t209 = cos(t208);
t211 = sin(t208);
t213 = -t1 * t211 - t209 * t73;
t217 = -t1 * t209 + t211 * t73;
t222 = -g(2) * (m(9) * t7 + (t10 * t11 + t14) * mrSges(9,1) + (t10 * t13 - t11 * t5) * mrSges(9,2) + t3 * mrSges(9,3)) - g(1) * (mrSges(2,1) * t5 - mrSges(2,2) * t1) - g(2) * (mrSges(2,1) * t1 + mrSges(2,2) * t5) - g(2) * (m(5) * (pkin(19) * t36 - t10 * t33 + t4 + t6) + t45 * mrSges(5,1) + t49 * mrSges(5,2) + t3 * mrSges(5,3)) - g(2) * (m(10) * (-pkin(18) * t14 - t10 * t54 + t4 + t6) + t64 * mrSges(10,1) + t68 * mrSges(10,2) + t3 * mrSges(10,3)) - g(1) * (m(10) * (pkin(18) * t77 - t54 * t73 + t76 - t79) + t84 * mrSges(10,1) + t88 * mrSges(10,2) + t75 * mrSges(10,3)) - g(1) * (m(11) * (-pkin(20) * t95 - t73 * t93 + t76 - t79) + t104 * mrSges(11,1) + t108 * mrSges(11,2) + t75 * mrSges(11,3)) - g(2) * (m(11) * (pkin(20) * t36 - t10 * t93 + t4 + t6) + t119 * mrSges(11,1) + t123 * mrSges(11,2) + t3 * mrSges(11,3)) - g(1) * (m(12) * (-t1 * t133 - t129 * t73 + t76 - t79) + t142 * mrSges(12,1) + t146 * mrSges(12,2) + t75 * mrSges(12,3)) - g(1) * (m(4) * t151 + mrSges(4,1) * t154 + mrSges(4,2) * t158 + mrSges(4,3) * t75) - g(2) * (m(4) * t7 + mrSges(4,1) * t165 + mrSges(4,2) * t169 + mrSges(4,3) * t3) - g(1) * (-m(3) * pkin(12) * t1 - mrSges(3,1) * t73 - mrSges(3,2) * t75 - mrSges(3,3) * t1) - g(1) * (m(9) * t151 + (t11 * t73 - t77) * mrSges(9,1) + (t1 * t11 + t13 * t73) * mrSges(9,2) + t75 * mrSges(9,3)) - g(2) * (m(3) * pkin(12) * t5 - mrSges(3,1) * t10 - mrSges(3,2) * t3 + mrSges(3,3) * t5) - g(1) * (m(6) * t206 + mrSges(6,1) * t213 + mrSges(6,2) * t217 + mrSges(6,3) * t75);
t225 = -t10 * t200 + t204 * t5 + t4 + t6;
t229 = -t10 * t209 + t211 * t5;
t233 = t10 * t211 + t209 * t5;
t238 = pkin(8) * t209;
t239 = t238 + t199 + t33;
t240 = t73 * t239;
t241 = pkin(8) * t211;
t242 = t241 + t202 + t203;
t243 = t1 * t242;
t246 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t247 = sin(t246);
t249 = cos(t246);
t251 = -t1 * t249 + t247 * t73;
t255 = t1 * t247 + t249 * t73;
t260 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t261 = cos(t260);
t262 = pkin(10) * t261;
t263 = t199 - t262 + t33;
t264 = t10 * t263;
t265 = sin(t260);
t266 = pkin(10) * t265;
t267 = t202 - t266 + t203;
t268 = t5 * t267;
t271 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t272 = sin(t271);
t274 = cos(t271);
t276 = t10 * t272 + t274 * t5;
t280 = t10 * t274 - t272 * t5;
t285 = t73 * t263;
t286 = t1 * t267;
t291 = -t1 * t274 + t272 * t73;
t295 = t1 * t272 + t274 * t73;
t314 = t10 * t138 - t140 * t5;
t318 = -t10 * t140 - t138 * t5;
t331 = t10 * t239;
t332 = t5 * t242;
t337 = t10 * t247 + t249 * t5;
t341 = t10 * t249 - t247 * t5;
t349 = cos(qJ(7));
t351 = sin(qJ(7));
t353 = t3 * t351 + t337 * t349;
t357 = t3 * t349 - t337 * t351;
t366 = t75 * t351;
t370 = t75 * t349;
t379 = t10 * t261 - t265 * t5;
t383 = -t10 * t265 - t261 * t5;
t394 = -t1 * t43 - t41 * t73;
t398 = -t1 * t41 + t43 * t73;
t404 = qJ(3) + qJ(4) + qJ(11);
t405 = sin(t404);
t407 = cos(t404);
t409 = -t10 * t405 - t407 * t5;
t413 = -t10 * t407 + t405 * t5;
t421 = t1 * t407 - t405 * t73;
t425 = -t1 * t405 - t407 * t73;
t433 = t1 * t265 + t261 * t73;
t437 = t1 * t261 - t265 * t73;
t442 = -g(2) * (m(6) * t225 + mrSges(6,1) * t229 + mrSges(6,2) * t233 + mrSges(6,3) * t3) - g(1) * (m(7) * (-t240 + t76 - t243 - t79) + t251 * mrSges(7,1) + t255 * mrSges(7,2) + t75 * mrSges(7,3)) - g(2) * (m(15) * (-t264 + t4 + t268 + t6) + t276 * mrSges(15,1) + t280 * mrSges(15,2) + t3 * mrSges(15,3)) - g(1) * (m(15) * (-t285 + t76 - t286 - t79) + t291 * mrSges(15,1) + t295 * mrSges(15,2) + t75 * mrSges(15,3)) - g(1) * (m(16) * (-qJ(13) * t295 - t285 - t286 + t76 - t79) + t291 * mrSges(16,1) + t75 * mrSges(16,2) - t295 * mrSges(16,3)) - g(2) * (m(12) * (-t10 * t129 + t133 * t5 + t4 + t6) + t314 * mrSges(12,1) + t318 * mrSges(12,2) + t3 * mrSges(12,3)) - g(2) * (m(16) * (-qJ(13) * t280 - t264 + t268 + t4 + t6) + t276 * mrSges(16,1) + t3 * mrSges(16,2) - t280 * mrSges(16,3)) - g(2) * (m(7) * (-t331 + t4 + t332 + t6) + t337 * mrSges(7,1) + t341 * mrSges(7,2) + t3 * mrSges(7,3)) - g(2) * (m(8) * (-pkin(11) * t341 - t331 + t332 + t4 + t6) + t353 * mrSges(8,1) + t357 * mrSges(8,2) - t341 * mrSges(8,3)) - g(1) * (m(8) * (-pkin(11) * t255 - t240 - t243 + t76 - t79) + (t251 * t349 + t366) * mrSges(8,1) + (-t251 * t351 + t370) * mrSges(8,2) - t255 * mrSges(8,3)) - g(2) * (m(14) * t225 + mrSges(14,1) * t379 + mrSges(14,2) * t383 + mrSges(14,3) * t3) - g(1) * (m(5) * (-pkin(19) * t95 - t33 * t73 + t76 - t79) + t394 * mrSges(5,1) + t398 * mrSges(5,2) + t75 * mrSges(5,3)) - g(2) * (m(13) * t225 + mrSges(13,1) * t409 + mrSges(13,2) * t413 + mrSges(13,3) * t3) - g(1) * (m(13) * t206 + mrSges(13,1) * t421 + mrSges(13,2) * t425 + mrSges(13,3) * t75) - g(1) * (m(14) * t206 + mrSges(14,1) * t433 + mrSges(14,2) * t437 + mrSges(14,3) * t75);
t444 = t9 * t239;
t445 = t2 * pkin(17);
t448 = t9 * t247;
t450 = t9 * t249;
t455 = t3 * t239;
t456 = t10 * pkin(17);
t459 = t247 * mrSges(7,1);
t461 = t249 * mrSges(7,2);
t466 = t9 * t32;
t480 = t41 * mrSges(5,1);
t482 = t43 * mrSges(5,2);
t488 = t73 * pkin(17);
t507 = t60 * mrSges(10,1);
t509 = t62 * mrSges(10,2);
t514 = t9 * t11;
t536 = t100 * mrSges(11,1);
t538 = t102 * mrSges(11,2);
t565 = t138 * mrSges(12,1);
t567 = t140 * mrSges(12,2);
t581 = t200 * t75 + t488;
t583 = t405 * mrSges(13,1);
t585 = t407 * mrSges(13,2);
t591 = t261 * mrSges(14,1);
t593 = t265 * mrSges(14,2);
t599 = t200 * t9 - t445;
t618 = t9 * t274;
t620 = t9 * t263;
t623 = t9 * t272;
t629 = t75 * t239;
t637 = t249 * pkin(11);
t641 = t247 * t349;
t646 = t247 * t351;
t651 = t249 * mrSges(8,3);
t655 = -g(3) * (m(11) * (pkin(20) * t466 - t445) + t9 * t100 * mrSges(11,1) - t9 * t102 * mrSges(11,2) - t2 * mrSges(11,3)) - g(2) * (m(11) * (t75 * t93 + t488) + t75 * t536 - t75 * t538 + t73 * mrSges(11,3)) - g(1) * (m(12) * (-t129 * t3 - t456) + t3 * t565 - t3 * t567 - t10 * mrSges(12,3)) - g(2) * (m(12) * (t129 * t75 + t488) - t75 * t565 + t75 * t567 + t73 * mrSges(12,3)) - g(2) * (m(13) * t581 + mrSges(13,3) * t73 + t583 * t75 + t585 * t75) - g(2) * (m(14) * t581 + mrSges(14,3) * t73 - t591 * t75 + t593 * t75) - g(3) * (mrSges(13,1) * t405 * t9 + mrSges(13,2) * t407 * t9 + m(13) * t599 - mrSges(13,3) * t2) - g(3) * (m(12) * (t129 * t9 - t445) - t9 * t138 * mrSges(12,1) + t9 * t140 * mrSges(12,2) - t2 * mrSges(12,3)) - g(3) * (m(16) * (qJ(13) * t618 - t445 + t620) - t623 * mrSges(16,1) - t2 * mrSges(16,2) + t618 * mrSges(16,3)) - g(2) * (m(7) * (t629 + t488) - t75 * t459 - t75 * t461 + t73 * mrSges(7,3)) - g(2) * (m(8) * (t637 * t75 + t488 + t629) + (t351 * t73 - t641 * t75) * mrSges(8,1) + (t349 * t73 + t646 * t75) * mrSges(8,2) + t75 * t651);
t694 = -t200 * t3 - t456;
t707 = t75 * t263;
t710 = t272 * mrSges(15,1);
t712 = t274 * mrSges(15,2);
t724 = t3 * t263;
t732 = t274 * qJ(13);
t736 = t272 * mrSges(16,1);
t739 = t274 * mrSges(16,3);
t752 = m(4) * t9;
t753 = t1 * pkin(17);
t755 = t32 * mrSges(4,1);
t757 = t35 * mrSges(4,2);
t762 = t5 * pkin(17);
t785 = m(9) * t9;
t787 = t11 * mrSges(9,1);
t789 = t13 * mrSges(9,2);
t813 = t209 * mrSges(6,1);
t815 = t211 * mrSges(6,2);
t826 = -g(1) * (-mrSges(4,3) * t10 - t3 * t755 + t3 * t757 - t752 * t753) - g(2) * (mrSges(4,3) * t73 + t75 * t755 - t75 * t757 + t752 * t762) - g(3) * (mrSges(3,1) * t9 + mrSges(3,2) * t2) - g(3) * (-m(9) * pkin(17) * t2 - mrSges(9,2) * t13 * t9 - mrSges(9,1) * t514 - mrSges(9,3) * t2) - g(1) * (-mrSges(3,1) * t3 + mrSges(3,2) * t10) - g(1) * (-mrSges(9,3) * t10 + t3 * t787 + t3 * t789 - t753 * t785) - g(2) * (mrSges(3,1) * t75 - mrSges(3,2) * t73) - g(2) * (mrSges(9,3) * t73 - t75 * t787 - t75 * t789 + t762 * t785) - g(3) * (mrSges(6,1) * t209 * t9 - mrSges(6,2) * t211 * t9 + m(6) * t599 - mrSges(6,3) * t2) - g(1) * (m(6) * t694 - mrSges(6,3) * t10 - t3 * t813 + t3 * t815) - g(2) * (m(6) * t581 + mrSges(6,3) * t73 + t75 * t813 - t75 * t815);
t829 = 0.2e1 * qJ(9);
t831 = sin(qJ(4) + qJ(11) + pkin(1) - t829 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3));
t833 = qJ(4) + qJ(11) + pkin(1);
t834 = sin(t833);
t836 = sin(qJ(4));
t838 = -pkin(7) * t836 + pkin(10) * t834;
t839 = -t829 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t840 = cos(t839);
t842 = cos(qJ(4));
t844 = cos(t833);
t845 = t844 * pkin(10);
t846 = -pkin(7) * t842 + t845;
t847 = sin(t839);
t850 = sin(qJ(4) + qJ(11) + pkin(1) - qJ(10) + pkin(5) - pkin(4) - qJ(8) + qJ(3));
t852 = qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t853 = cos(t852);
t855 = sin(t852);
t859 = 0.1e1 / pkin(15);
t861 = sin(-qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9));
t862 = t861 * pkin(14);
t864 = sin(qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9));
t865 = t864 * pkin(14);
t867 = sin(pkin(4) + qJ(8) - qJ(3) - qJ(10));
t869 = sin(pkin(4) + qJ(8) - qJ(3) + qJ(10));
t877 = t10 * t204 + t200 * t5;
t879 = t413 * mrSges(13,1);
t880 = -t409 * mrSges(13,2);
t884 = t383 * mrSges(14,1);
t885 = -t379 * mrSges(14,2);
t888 = -t73 * t267;
t889 = t1 * t263;
t892 = -t295 * mrSges(15,1);
t893 = t291 * mrSges(15,2);
t896 = m(15) * t2;
t898 = t2 * t274;
t899 = t898 * mrSges(15,1);
t900 = t2 * t272;
t901 = t900 * mrSges(15,2);
t906 = t1 * t200 - t204 * t73;
t908 = -t217 * mrSges(6,1);
t909 = t213 * mrSges(6,2);
t912 = m(7) * t2;
t914 = t2 * t249;
t915 = t914 * mrSges(7,1);
t916 = t2 * t247;
t917 = t916 * mrSges(7,2);
t920 = -t10 * t242;
t921 = t5 * t239;
t924 = t341 * mrSges(7,1);
t925 = -t337 * mrSges(7,2);
t928 = t916 * pkin(11);
t933 = t914 * t349 * mrSges(8,1);
t935 = t914 * t351 * mrSges(8,2);
t936 = t916 * mrSges(8,3);
t939 = -t337 * pkin(11);
t943 = t341 * t349 * mrSges(8,1);
t945 = t341 * t351 * mrSges(8,2);
t946 = t337 * mrSges(8,3);
t953 = t123 * mrSges(11,1);
t954 = -t119 * mrSges(11,2);
t961 = t2 * t102 * mrSges(11,1);
t963 = t2 * t100 * mrSges(11,2);
t970 = -t108 * mrSges(11,1);
t971 = t104 * mrSges(11,2);
t974 = t291 * qJ(13);
t977 = -t295 * mrSges(16,1);
t978 = -t291 * mrSges(16,3);
t985 = t2 * t43 * mrSges(5,1);
t987 = t2 * t41 * mrSges(5,2);
t994 = t49 * mrSges(5,1);
t995 = -t45 * mrSges(5,2);
t1002 = -t398 * mrSges(5,1);
t1003 = t394 * mrSges(5,2);
t1006 = -g(1) * (m(13) * t877 + t879 + t880) - g(1) * (m(14) * t877 + t884 + t885) - g(2) * (m(15) * (t888 + t889) + t892 + t893) - g(3) * (t267 * t896 + t899 - t901) - g(2) * (m(6) * t906 + t908 + t909) - g(3) * (t242 * t912 + t915 - t917) - g(1) * (m(7) * (-t920 + t921) + t924 + t925) - g(3) * (m(8) * (t2 * t242 + t928) + t933 - t935 + t936) - g(1) * (m(8) * (-t939 - t920 + t921) + t943 - t945 + t946) - g(1) * (m(11) * (pkin(20) * t168 + t10 * t132) + t953 + t954) - g(3) * (m(11) * pkin(20) * t2 * t35 + t961 + t963) - g(2) * (m(11) * (pkin(20) * t157 - t132 * t73) + t970 + t971) - g(2) * (m(16) * (-t974 + t888 + t889) + t977 + t978) - g(3) * (m(5) * pkin(19) * t2 * t35 + t985 + t987) - g(1) * (m(5) * (pkin(19) * t168 + t10 * t203) + t994 + t995) - g(2) * (m(5) * (pkin(19) * t157 - t203 * t73) + t1002 + t1003);
t1025 = t318 * mrSges(12,1);
t1026 = -t314 * mrSges(12,2);
t1029 = m(12) * t2;
t1032 = t2 * t140 * mrSges(12,1);
t1034 = t2 * t138 * mrSges(12,2);
t1037 = m(6) * t2;
t1040 = t2 * t211 * mrSges(6,1);
t1042 = t2 * t209 * mrSges(6,2);
t1046 = t233 * mrSges(6,1);
t1047 = -t229 * mrSges(6,2);
t1050 = -t10 * t267;
t1051 = t5 * t263;
t1054 = t280 * mrSges(15,1);
t1055 = -t276 * mrSges(15,2);
t1059 = -t425 * mrSges(13,1);
t1060 = t421 * mrSges(13,2);
t1064 = -t437 * mrSges(14,1);
t1065 = t433 * mrSges(14,2);
t1072 = -t146 * mrSges(12,1);
t1073 = t142 * mrSges(12,2);
t1076 = -t276 * qJ(13);
t1079 = t280 * mrSges(16,1);
t1080 = t276 * mrSges(16,3);
t1083 = t900 * qJ(13);
t1087 = t898 * mrSges(16,1);
t1088 = t900 * mrSges(16,3);
t1091 = -t73 * t242;
t1092 = t1 * t239;
t1095 = -t255 * mrSges(7,1);
t1096 = t251 * mrSges(7,2);
t1099 = t251 * pkin(11);
t1103 = -t255 * t349 * mrSges(8,1);
t1105 = -t255 * t351 * mrSges(8,2);
t1106 = -t251 * mrSges(8,3);
t1109 = m(13) * t2;
t1112 = t2 * t407 * mrSges(13,1);
t1114 = t2 * t405 * mrSges(13,2);
t1117 = m(14) * t2;
t1120 = t2 * t265 * mrSges(14,1);
t1122 = t2 * t261 * mrSges(14,2);
t1125 = -g(3) * (mrSges(4,1) * t2 * t35 + mrSges(4,2) * t2 * t32) - g(1) * (mrSges(4,1) * t169 - mrSges(4,2) * t165) - g(2) * (-mrSges(4,1) * t158 + mrSges(4,2) * t154) - g(1) * (m(12) * (t10 * t133 + t129 * t5) + t1025 + t1026) - g(3) * (t1029 * t133 - t1032 - t1034) - g(3) * (t1037 * t204 + t1040 + t1042) - g(1) * (m(6) * t877 + t1046 + t1047) - g(1) * (m(15) * (-t1050 + t1051) + t1054 + t1055) - g(2) * (m(13) * t906 + t1059 + t1060) - g(2) * (m(14) * t906 + t1064 + t1065) - g(2) * (m(12) * (t1 * t129 - t73 * t133) + t1072 + t1073) - g(1) * (m(16) * (-t1076 - t1050 + t1051) + t1079 + t1080) - g(3) * (m(16) * (t2 * t267 + t1083) + t1087 + t1088) - g(2) * (m(7) * (t1091 + t1092) + t1095 + t1096) - g(2) * (m(8) * (-t1099 + t1091 + t1092) + t1103 - t1105 + t1106) - g(3) * (t1109 * t204 - t1112 + t1114) - g(3) * (t1117 * t204 - t1120 - t1122);
t1130 = sin(qJ(4) + qJ(11) + pkin(1) - qJ(9) - qJ(10));
t1134 = sin(-qJ(9) + qJ(4) + qJ(11) + pkin(1));
t1137 = qJ(9) + qJ(10);
t1138 = sin(t1137);
t1140 = sin(qJ(9));
t1144 = cos(t1137);
t1146 = cos(qJ(9));
t1164 = sin(qJ(10));
t1165 = 0.1e1 / t1164;
t1176 = qJ(11) + pkin(1);
t1177 = sin(t1176);
t1178 = qJ(11) + pkin(1) + qJ(12);
t1179 = sin(t1178);
t1182 = cos(t1176);
t1183 = cos(t1178);
t1186 = -qJ(5) - qJ(6) + pkin(3);
t1187 = cos(t1186);
t1191 = sin(t1186);
t1194 = 0.1e1 / qJ(13);
t1215 = -g(1) * (m(16) * qJ(13) * t276 + t1079 + t1080) - g(2) * (t892 + t893) - g(3) * (t899 - t901) - g(1) * (t1054 + t1055) - g(2) * (-m(16) * qJ(13) * t291 + t977 + t978) - g(3) * (m(16) * qJ(13) * t2 * t272 + t1087 + t1088);
t1254 = -t241 - t202;
t1255 = t10 * t1254;
t1256 = t238 + t199;
t1257 = t5 * t1256;
t1269 = pkin(7) * t1 * t41 - t202 * t73;
t1285 = pkin(7) * t41 * t5 + t10 * t202;
t1289 = pkin(14) * (pkin(10) * t831 + pkin(10) * t850 + t838 * t840 + t838 * t853 + t846 * t847 - t846 * t855) * t859 / (t862 - t865 + pkin(20) * (t867 - t869)) * (t1006 + t1125) + (-t1130 * pkin(9) * pkin(10) + t1134 * pkin(10) * pkin(15) - t845 * (-pkin(9) * t1138 + pkin(15) * t1140) + pkin(10) * (-pkin(9) * t1144 + pkin(15) * t1146) * t834 + pkin(7) * (t836 * pkin(9) * t1144 - pkin(9) * t842 * t1138 + pkin(15) * (t1140 * t842 - t1146 * t836))) / pkin(9) * t859 * t1165 * (-g(2) * (t1072 + t1073) - g(1) * (t1025 + t1026) - g(3) * (-t1032 - t1034)) + (pkin(10) * t1177 * t1179 + pkin(10) * t1182 * t1183 + pkin(16) * t1179 * t1191 - pkin(16) * t1183 * t1187 - qJ(13)) * t1194 * t1215 + (pkin(7) * t1138 * t842 - pkin(7) * t1144 * t836 - pkin(10) * t1138 * t844 + pkin(10) * t1144 * t834 + pkin(10) * t1130) * t1165 * t859 * (-g(2) * (m(12) * (pkin(15) * t1 * t100 - t131 * t73) + t1072 + t1073) - g(1) * (t953 + t954) - g(3) * (t961 + t963) - g(2) * (t970 + t971) - g(1) * (m(12) * (pkin(15) * t100 * t5 + t10 * t131) + t1025 + t1026) - g(3) * (t1029 * t131 - t1032 - t1034)) - g(1) * (m(7) * (-t1255 + t1257) + t924 + t925) - g(3) * (t985 + t987) - g(1) * (t994 + t995) - g(2) * (m(13) * t1269 + t1059 + t1060) - g(2) * (m(14) * t1269 + t1064 + t1065) - g(3) * (t1109 * t202 - t1112 + t1114) - g(3) * (t1117 * t202 - t1120 - t1122) - g(1) * (m(13) * t1285 + t879 + t880);
t1293 = -t202 + t266;
t1294 = t73 * t1293;
t1295 = t199 - t262;
t1296 = t1 * t1295;
t1304 = t10 * t1293;
t1305 = t5 * t1295;
t1323 = t10 * t266;
t1325 = t5 * pkin(10) * t261;
t1338 = -g(1) * (m(14) * t1285 + t884 + t885) - g(2) * (m(15) * (t1294 + t1296) + t892 + t893) - g(3) * (-t1293 * t896 + t899 - t901) - g(1) * (m(15) * (-t1304 + t1305) + t1054 + t1055) - g(2) * (m(16) * (-t974 + t1294 + t1296) + t977 + t978) - g(3) * (m(16) * (-t1293 * t2 + t1083) + t1087 + t1088) - g(1) * (m(16) * (-t1076 - t1304 + t1305) + t1079 + t1080) - g(1) * (m(16) * (-t1076 - t1323 - t1325) + t1079 + t1080) - g(2) * (t1059 + t1060) - g(2) * (t1064 + t1065) - g(3) * (-t1112 + t1114) - g(3) * (-t1120 - t1122);
t1344 = t73 * t266;
t1346 = t1 * pkin(10) * t261;
t1369 = g(1) * (t924 + t925);
t1373 = g(2) * (t1095 + t1096);
t1377 = g(2) * (-m(8) * pkin(11) * t251 + t1103 - t1105 + t1106);
t1382 = g(3) * (m(8) * pkin(11) * t2 * t247 + t933 - t935 + t936);
t1383 = -g(1) * (t879 + t880) - g(1) * (t884 + t885) - g(2) * (m(15) * (t1344 - t1346) + t892 + t893) - g(3) * (-t266 * t896 + t899 - t901) - g(1) * (m(15) * (-t1323 - t1325) + t1054 + t1055) - g(2) * (m(16) * (-t974 + t1344 - t1346) + t977 + t978) - g(3) * (m(16) * (-pkin(10) * t2 * t265 + t1083) + t1087 + t1088) + t1369 - g(2) * (t1002 + t1003) + t1373 + t1377 + t1382;
t1387 = g(1) * (m(8) * pkin(11) * t337 + t943 - t945 + t946);
t1389 = g(3) * (t915 - t917);
t1390 = t73 * t1254;
t1391 = t1 * t1256;
t1436 = m(16) * g(1) * t280 - m(16) * g(2) * t295 + m(16) * g(3) * t898;
t1439 = t829 + pkin(5) + qJ(10);
t1440 = cos(t1439);
t1445 = -qJ(10) + pkin(5);
t1446 = cos(t1445);
t1451 = sin(t1445);
t1456 = sin(t1439);
t1462 = cos(pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10));
t1465 = cos(-t829 - pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10));
t1493 = t1387 + t1389 - g(2) * (m(7) * (t1390 + t1391) + t1095 + t1096) - g(2) * (m(8) * (-t1099 + t1390 + t1391) + t1103 - t1105 + t1106) - g(3) * (m(8) * (-t1254 * t2 + t928) + t933 - t935 + t936) - g(1) * (m(8) * (-t939 - t1255 + t1257) + t943 - t945 + t946) - g(3) * (t1037 * t202 + t1040 + t1042) - g(1) * (m(6) * t1285 + t1046 + t1047) - g(2) * (m(6) * t1269 + t908 + t909) - g(3) * (-t1254 * t912 + t915 - t917) + (-pkin(10) * t1177 * t1183 + pkin(10) * t1179 * t1182 - pkin(16) * t1179 * t1187 - pkin(16) * t1183 * t1191) * t1436 + pkin(20) * pkin(14) * (-pkin(7) * t1440 * t842 + pkin(7) * t1446 * t842 - pkin(7) * t1451 * t836 - pkin(7) * t1456 * t836 + pkin(10) * t1440 * t844 - pkin(10) * t1446 * t844 + pkin(10) * t1451 * t834 + pkin(10) * t1456 * t834 - pkin(10) * t1462 + pkin(10) * t1465) / pkin(13) * t859 / (-pkin(20) * t867 + pkin(20) * t869 - t862 + t865) * (-g(2) * (-mrSges(10,1) * t88 + mrSges(10,2) * t84) - g(3) * (mrSges(10,1) * t2 * t62 - mrSges(10,2) * t2 * t60) - g(1) * (mrSges(10,1) * t68 - mrSges(10,2) * t64));
t1496 = t73 * t241;
t1498 = t1 * pkin(8) * t209;
t1513 = t10 * t241;
t1515 = t5 * pkin(8) * t209;
t1533 = qJ(11) + pkin(1) + qJ(12) - qJ(5) - qJ(6) + pkin(3);
t1534 = cos(t1533);
t1537 = qJ(11) + pkin(1) + qJ(12) - qJ(5);
t1538 = cos(t1537);
t1543 = sin(t1533);
t1546 = sin(t1537);
t1550 = -g(2) * (m(7) * (-t1496 + t1498) + t1095 + t1096) - g(2) * (m(8) * (-t1099 - t1496 + t1498) + t1103 - t1105 + t1106) - g(3) * (m(8) * (pkin(8) * t2 * t211 + t928) + t933 - t935 + t936) - g(1) * (m(8) * (-t939 + t1513 + t1515) + t943 - t945 + t946) - g(3) * (t1040 + t1042) - g(1) * (t1046 + t1047) - g(2) * (t908 + t909) - g(3) * (t241 * t912 + t915 - t917) - g(1) * (m(7) * (t1513 + t1515) + t924 + t925) - t1373 - t1377 - t1382 - t1387 - t1389 - t1369 + (pkin(8) * t1538 + 0.2e1 * pkin(16) * t1534) * t1194 * t1215 + (pkin(8) * t1546 + 0.2e1 * pkin(16) * t1543) * t1436;
unknown(1,1) = t222 + t442;
unknown(2,1) = -g(3) * (m(7) * (t444 - t445) - t448 * mrSges(7,1) - t450 * mrSges(7,2) - t2 * mrSges(7,3)) - g(1) * (m(7) * (-t455 - t456) + t3 * t459 + t3 * t461 - t10 * mrSges(7,3)) - g(3) * (m(5) * (pkin(19) * t466 - t445) + t9 * t41 * mrSges(5,1) - t9 * t43 * mrSges(5,2) - t2 * mrSges(5,3)) - g(1) * (m(5) * (-t3 * t33 - t456) - t3 * t480 + t3 * t482 - t10 * mrSges(5,3)) - g(2) * (m(5) * (t33 * t75 + t488) + t75 * t480 - t75 * t482 + t73 * mrSges(5,3)) - g(3) * (-m(4) * pkin(17) * t2 - mrSges(4,2) * t35 * t9 + mrSges(4,1) * t466 - mrSges(4,3) * t2) - g(2) * (m(10) * (t54 * t75 + t488) - t75 * t507 - t75 * t509 + t73 * mrSges(10,3)) - g(3) * (m(10) * (pkin(18) * t514 - t445) - t9 * t60 * mrSges(10,1) - t9 * t62 * mrSges(10,2) - t2 * mrSges(10,3)) - g(1) * (m(10) * (-t3 * t54 - t456) + t3 * t507 + t3 * t509 - t10 * mrSges(10,3)) - g(1) * (m(11) * (-t3 * t93 - t456) - t3 * t536 + t3 * t538 - t10 * mrSges(11,3)) + t655 - g(3) * (m(8) * (pkin(11) * t450 + t444 - t445) + (-t2 * t351 - t349 * t448) * mrSges(8,1) + (-t2 * t349 + t351 * t448) * mrSges(8,2) + t450 * mrSges(8,3)) - g(1) * (m(8) * (-t3 * t637 - t455 - t456) + (-t10 * t351 + t3 * t641) * mrSges(8,1) + (-t10 * t349 - t3 * t646) * mrSges(8,2) - t3 * t651) - g(3) * (-mrSges(14,1) * t261 * t9 + mrSges(14,2) * t265 * t9 + m(14) * t599 - mrSges(14,3) * t2) - g(1) * (m(13) * t694 - mrSges(13,3) * t10 - t3 * t583 - t3 * t585) - g(1) * (m(14) * t694 - mrSges(14,3) * t10 + t3 * t591 - t3 * t593) - g(2) * (m(15) * (t707 + t488) - t75 * t710 - t75 * t712 + t73 * mrSges(15,3)) - g(3) * (m(15) * (t620 - t445) - t623 * mrSges(15,1) - t618 * mrSges(15,2) - t2 * mrSges(15,3)) - g(1) * (m(15) * (-t724 - t456) + t3 * t710 + t3 * t712 - t10 * mrSges(15,3)) - g(2) * (m(16) * (t732 * t75 + t488 + t707) - t75 * t736 + t73 * mrSges(16,2) + t75 * t739) - g(1) * (m(16) * (-t3 * t732 - t456 - t724) + t3 * t736 - t10 * mrSges(16,2) - t3 * t739) + t826;
unknown(3,1) = t1289 + t1338 + t1383 + t1493;
unknown(4,1) = t1550;
unknown(5,1) = -g(2) * ((t251 * t351 - t370) * mrSges(8,1) + (t251 * t349 + t366) * mrSges(8,2)) - g(3) * ((-t349 * t9 - t351 * t916) * mrSges(8,1) + (-t349 * t916 + t351 * t9) * mrSges(8,2)) - g(1) * (mrSges(8,1) * t357 - mrSges(8,2) * t353);
taug = unknown(:);
