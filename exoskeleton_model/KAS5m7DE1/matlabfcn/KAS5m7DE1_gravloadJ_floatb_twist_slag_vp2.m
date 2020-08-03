% Calculate Gravitation load on the joints for
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
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
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE1_gravloadJ_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From gravload_joint_floatb_twist_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-15 08:18:24
% EndTime: 2020-05-15 10:39:00
% DurationCPUTime: 3260.62s
% Computational Cost: add. (226789345->1055), mult. (295070601->1471), div. (4030336->24), fcn. (118110215->43), ass. (0->525)
unknown=NaN(5,1);
t1 = sin(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = sin(qJ(3));
t5 = cos(qJ(3));
t6 = -pkin(23) + pkin(24);
t7 = t5 * t6;
t8 = pkin(3) + qJ(3);
t9 = cos(t8);
t10 = t9 * pkin(12);
t11 = -pkin(9) + t7 + t10;
t12 = pkin(11) ^ 2;
t13 = pkin(19) ^ 2;
t14 = t9 ^ 2;
t15 = pkin(12) ^ 2;
t17 = sin(t8);
t18 = t17 ^ 2;
t20 = -pkin(9) + t7;
t21 = t20 ^ 2;
t22 = t4 ^ 2;
t23 = t6 ^ 2;
t27 = t4 * t6;
t28 = t17 * pkin(12);
t29 = -t27 + t28;
t32 = 0.2e1 * pkin(12) * t11 * t9 + 0.2e1 * pkin(12) * t17 * t29 - t14 * t15 - t15 * t18 + t22 * t23 + t12 - t13 + t21;
t36 = 0.4e1 * t11 ^ 2 + 0.4e1 * t29 ^ 2;
t38 = t32 ^ 2;
t40 = sqrt(t12 * t36 - t38);
t42 = 0.2e1 * t11 * t32 - 0.2e1 * t29 * t40;
t43 = 0.1e1 / t36;
t45 = t42 * t43 + pkin(9) - t10 - t7;
t46 = t4 * t45;
t49 = 0.2e1 * t11 * t40 + 0.2e1 * t29 * t32;
t51 = t43 * t49 + t27 - t28;
t52 = t5 * t51;
t53 = t46 + t52;
t54 = 0.1e1 / pkin(19);
t55 = t53 * t54;
t56 = cos(pkin(7));
t57 = t56 * pkin(18);
t58 = t55 * t57;
t59 = t5 * t45;
t60 = t4 * t51;
t61 = t59 - t60;
t62 = t61 * t54;
t63 = sin(pkin(7));
t64 = t63 * pkin(18);
t65 = t62 * t64;
t66 = pkin(17) ^ 2;
t67 = pkin(22) ^ 2;
t68 = t62 * t57;
t69 = t55 * t64;
t70 = t68 - t69;
t71 = t70 ^ 2;
t72 = t58 + t65;
t73 = t72 ^ 2;
t74 = pkin(24) ^ 2;
t75 = -pkin(24) - t68 + t69;
t78 = -0.2e1 * t70 * t75 + 0.2e1 * t72 ^ 2 + t66 - t67 - t71 - t73 + t74;
t82 = 0.4e1 * t72 ^ 2 + 0.4e1 * t75 ^ 2;
t84 = t78 ^ 2;
t86 = sqrt(t66 * t82 - t84);
t88 = -0.2e1 * t72 * t78 + 0.2e1 * t75 * t86;
t89 = 0.1e1 / t82;
t91 = -t88 * t89 - t58 - t65;
t94 = 0.2e1 * t72 * t86 + 0.2e1 * t75 * t78;
t96 = t89 * t94 + pkin(24) + t68 - t69;
t97 = atan2(t91, t96);
t98 = t97 + pkin(6);
t99 = sin(t98);
t101 = cos(qJ(1));
t102 = cos(t98);
t104 = -t101 * t102 - t3 * t99;
t105 = t104 * t5;
t108 = t101 * t99 - t102 * t3;
t109 = t108 * t4;
t110 = t105 + t109;
t111 = t110 * pkin(9);
t112 = t104 * pkin(23);
t113 = cos(qJ(2));
t114 = t1 * t113;
t115 = t114 * pkin(21);
t116 = t101 * pkin(16);
t117 = t111 + t112 + t115 + t116;
t119 = cos(qJ(4));
t121 = t104 * t4;
t122 = t108 * t5;
t123 = -t121 + t122;
t124 = sin(qJ(4));
t126 = t110 * t119 + t123 * t124;
t130 = -t110 * t124 + t119 * t123;
t135 = t110 * t4;
t136 = t123 * t5;
t137 = t135 - t136;
t138 = sin(pkin(3));
t140 = t110 * t5;
t141 = t123 * t4;
t142 = t140 + t141;
t143 = cos(pkin(3));
t145 = t137 * t138 - t142 * t143;
t146 = qJ(4) - qJ(3) + pkin(4);
t147 = sin(t146);
t148 = t147 * pkin(15);
t149 = pkin(3) + qJ(3) - qJ(4);
t150 = sin(t149);
t151 = t150 * pkin(12);
t152 = cos(t146);
t153 = t152 * pkin(14);
t154 = t148 + t151 - t153;
t155 = t147 * pkin(14);
t156 = t152 * pkin(15);
t157 = cos(t149);
t158 = t157 * pkin(12);
t159 = -t155 - pkin(10) - t156 - t158;
t160 = atan2(t154, t159);
t161 = t160 + pkin(3) + qJ(3) - qJ(4);
t162 = cos(t161);
t166 = t137 * t143 + t138 * t142;
t167 = sin(t161);
t169 = -t145 * t162 + t166 * t167;
t170 = t159 ^ 2;
t171 = t154 ^ 2;
t173 = sqrt(t170 + t171);
t175 = t145 * pkin(12);
t180 = t145 * t167 + t162 * t166;
t186 = t126 * pkin(10);
t189 = t126 * t147;
t190 = t130 * t152;
t191 = -t189 + t190;
t193 = t126 * t152;
t194 = t130 * t147;
t195 = -t193 - t194;
t203 = cos(qJ(5));
t205 = sin(qJ(5));
t207 = t114 * t205 + t191 * t203;
t211 = t114 * t203 - t191 * t205;
t216 = t115 + t116;
t223 = t101 * t2;
t226 = t1 * t102 - t223 * t99;
t227 = atan2(-t53, t61);
t228 = cos(t227);
t232 = -t1 * t99 - t102 * t223;
t233 = sin(t227);
t235 = t226 * t228 - t232 * t233;
t237 = t226 * pkin(24);
t238 = t101 * t113;
t239 = t238 * pkin(21);
t240 = t1 * pkin(16);
t244 = t4 * t61 * t54;
t246 = t5 * t53 * t54;
t249 = (t244 - t246) * pkin(19) + t27 - t28;
t251 = t5 * t61 * t54;
t253 = t4 * t53 * t54;
t256 = (t251 + t253) * pkin(19) - pkin(9) + t7 + t10;
t257 = atan2(t249, t256);
t258 = -t257 + qJ(3) + t227;
t259 = cos(t258);
t263 = t226 * t233 + t228 * t232;
t264 = sin(t258);
t281 = t226 * pkin(23);
t286 = t226 * t5 + t232 * t4;
t290 = -t226 * t4 + t232 * t5;
t297 = t119 * t286 + t124 * t290;
t298 = t297 * pkin(10);
t299 = t286 * pkin(9);
t305 = t119 * t290 - t124 * t286;
t307 = -t147 * t297 + t152 * t305;
t311 = -t147 * t305 - t152 * t297;
t327 = t239 - t240;
t329 = sin(pkin(6));
t331 = cos(pkin(6));
t333 = -t1 * t331 + t223 * t329;
t337 = t1 * t329 + t223 * t331;
t349 = t299 + t281 + t239 - t240;
t353 = t286 * t4 - t290 * t5;
t357 = t286 * t5 + t290 * t4;
t364 = t138 * t353 - t143 * t357;
t365 = t364 * pkin(12);
t371 = t138 * t357 + t143 * t353;
t373 = t162 * t371 + t167 * t364;
t377 = -t162 * t364 + t167 * t371;
t382 = -g(2) * (m(6) * t117 + mrSges(6,1) * t126 + mrSges(6,2) * t130 + mrSges(6,3) * t114) - g(2) * (m(16) * (-t169 * t173 + t111 + t112 + t115 + t116 + t175) + t180 * mrSges(16,1) + t114 * mrSges(16,2) - t169 * mrSges(16,3)) - g(2) * (m(7) * (t186 + t111 + t112 + t115 + t116) + t191 * mrSges(7,1) + t195 * mrSges(7,2) + t114 * mrSges(7,3)) - g(2) * (m(8) * (-pkin(13) * t195 + t111 + t112 + t115 + t116 + t186) + t207 * mrSges(8,1) + t211 * mrSges(8,2) - t195 * mrSges(8,3)) - g(2) * (m(4) * t216 + mrSges(4,1) * t104 + mrSges(4,2) * t108 + mrSges(4,3) * t114) - g(1) * (m(12) * (pkin(19) * t235 + t237 + t239 - t240) + (-t235 * t259 - t263 * t264) * mrSges(12,1) + (t235 * t264 - t259 * t263) * mrSges(12,2) + t238 * mrSges(12,3)) - g(2) * (m(14) * t117 + mrSges(14,1) * t145 + mrSges(14,2) * t166 + mrSges(14,3) * t114) - g(1) * (m(5) * (t281 + t239 - t240) + t286 * mrSges(5,1) + t290 * mrSges(5,2) + t238 * mrSges(5,3)) - g(1) * (m(7) * (t298 + t299 + t281 + t239 - t240) + t307 * mrSges(7,1) + t311 * mrSges(7,2) + t238 * mrSges(7,3)) - g(2) * (m(5) * (t112 + t115 + t116) + t110 * mrSges(5,1) + t123 * mrSges(5,2) + t114 * mrSges(5,3)) - g(2) * (mrSges(2,1) * t1 + mrSges(2,2) * t101) - g(1) * (m(9) * t327 + mrSges(9,1) * t333 + mrSges(9,2) * t337 + mrSges(9,3) * t238) - g(1) * (m(11) * (t237 + t239 - t240) + t235 * mrSges(11,1) + t263 * mrSges(11,2) + t238 * mrSges(11,3)) - g(1) * (m(13) * t349 + mrSges(13,1) * t353 + mrSges(13,2) * t357 + mrSges(13,3) * t238) - g(1) * (m(15) * (t365 + t299 + t281 + t239 - t240) + t373 * mrSges(15,1) + t377 * mrSges(15,2) + t238 * mrSges(15,3));
t393 = t101 * t331 + t3 * t329;
t397 = 0.1e1 / pkin(22);
t398 = t91 * t397;
t400 = -t96 * t397;
t402 = t329 * t398 + t331 * t400;
t406 = -t329 * t400 + t331 * t398;
t410 = -(t329 * t406 - t331 * t402) * pkin(22) + pkin(24) + t68 - t69;
t415 = (t329 * t402 + t331 * t406) * pkin(22) + t58 + t65;
t416 = atan2(t410, t415);
t417 = t97 - t416;
t418 = sin(t417);
t422 = -t101 * t329 + t3 * t331;
t423 = cos(t417);
t440 = t104 * pkin(24);
t445 = t104 * t228 - t108 * t233;
t449 = t104 * t233 + t108 * t228;
t458 = t238 * t205;
t462 = t238 * t203;
t545 = -g(1) * (m(16) * (-t173 * t377 + t239 - t240 + t281 + t299 + t365) + t373 * mrSges(16,1) + t238 * mrSges(16,2) - t377 * mrSges(16,3)) - g(2) * (m(10) * (-pkin(22) * t393 + t115 + t116) + (t393 * t418 - t422 * t423) * mrSges(10,1) + (t393 * t423 + t418 * t422) * mrSges(10,2) + t114 * mrSges(10,3)) - g(2) * (m(13) * t117 + mrSges(13,1) * t137 + mrSges(13,2) * t142 + mrSges(13,3) * t114) - g(2) * (m(11) * (t440 + t115 + t116) + t445 * mrSges(11,1) + t449 * mrSges(11,2) + t114 * mrSges(11,3)) - g(1) * (m(8) * (-pkin(13) * t311 + t239 - t240 + t281 + t298 + t299) + (t203 * t307 + t458) * mrSges(8,1) + (-t205 * t307 + t462) * mrSges(8,2) - t311 * mrSges(8,3)) - g(2) * (m(9) * t216 + mrSges(9,1) * t393 + mrSges(9,2) * t422 + mrSges(9,3) * t114) - g(2) * (m(3) * pkin(16) * t101 - mrSges(3,1) * t3 - mrSges(3,2) * t114 + mrSges(3,3) * t101) - g(1) * (mrSges(2,1) * t101 - mrSges(2,2) * t1) - g(1) * (-m(3) * pkin(16) * t1 - mrSges(3,1) * t223 - mrSges(3,2) * t238 - mrSges(3,3) * t1) - g(1) * (m(10) * (-pkin(22) * t333 + t239 - t240) + (t333 * t418 - t337 * t423) * mrSges(10,1) + (t333 * t423 + t337 * t418) * mrSges(10,2) + t238 * mrSges(10,3)) - g(1) * (m(4) * t327 + mrSges(4,1) * t226 + mrSges(4,2) * t232 + mrSges(4,3) * t238) - g(2) * (m(12) * (pkin(19) * t445 + t115 + t116 + t440) + (-t259 * t445 - t264 * t449) * mrSges(12,1) + (-t259 * t449 + t264 * t445) * mrSges(12,2) + t114 * mrSges(12,3)) - g(2) * (m(15) * (t175 + t111 + t112 + t115 + t116) + t180 * mrSges(15,1) + t169 * mrSges(15,2) + t114 * mrSges(15,3)) - g(1) * (m(14) * t349 + mrSges(14,1) * t364 + mrSges(14,2) * t371 + mrSges(14,3) * t238) - g(1) * (m(6) * t349 + mrSges(6,1) * t297 + mrSges(6,2) * t305 + mrSges(6,3) * t238);
t547 = t2 * t329;
t549 = t113 * pkin(21);
t553 = t2 * t331;
t564 = t2 * t99;
t566 = t2 * t102;
t568 = t4 * t566 + t5 * t564;
t569 = t568 * pkin(9);
t570 = t564 * pkin(23);
t571 = t569 + t570 - t549;
t576 = -t4 * t564 + t5 * t566;
t578 = t4 * t568 - t5 * t576;
t582 = t4 * t576 + t5 * t568;
t584 = t138 * t578 - t143 * t582;
t588 = t138 * t582 + t143 * t578;
t595 = -t162 * t584 + t167 * t588;
t597 = t584 * pkin(12);
t602 = t162 * t588 + t167 * t584;
t618 = t119 * t568 + t124 * t576;
t622 = t119 * t576 - t124 * t568;
t627 = t99 * t5;
t629 = t102 * t4;
t631 = t238 * t627 + t238 * t629;
t633 = t99 * t4;
t635 = t102 * t5;
t637 = -t238 * t633 + t238 * t635;
t639 = t119 * t631 + t124 * t637;
t640 = t639 * pkin(10);
t641 = t631 * pkin(9);
t642 = t99 * pkin(23);
t643 = t238 * t642;
t644 = t223 * pkin(21);
t650 = t119 * t637 - t124 * t631;
t652 = -t147 * t639 + t152 * t650;
t656 = -t147 * t650 - t152 * t639;
t677 = -t147 * t622 - t152 * t618;
t679 = t618 * pkin(10);
t684 = -t147 * t618 + t152 * t622;
t698 = t228 * t564 - t233 * t566;
t700 = t564 * pkin(24);
t706 = t228 * t566 + t233 * t564;
t727 = -t114 * t627 - t114 * t629;
t728 = t727 * pkin(9);
t729 = t114 * t642;
t730 = t3 * pkin(21);
t731 = t728 - t729 - t730;
t736 = t114 * t633 - t114 * t635;
t738 = t119 * t727 + t124 * t736;
t742 = t119 * t736 - t124 * t727;
t747 = t738 * pkin(10);
t752 = -t147 * t738 + t152 * t742;
t756 = -t147 * t742 - t152 * t738;
t764 = t4 * t727 - t5 * t736;
t768 = t4 * t736 + t5 * t727;
t773 = t641 + t643 + t644;
t782 = t4 * t631 - t5 * t637;
t786 = t4 * t637 + t5 * t631;
t788 = t138 * t782 - t143 * t786;
t792 = t138 * t786 + t143 * t782;
t794 = -t162 * t788 + t167 * t792;
t796 = t788 * pkin(12);
t801 = t162 * t792 + t167 * t788;
t807 = m(4) * t2;
t808 = t101 * pkin(21);
t810 = t99 * mrSges(4,1);
t812 = t102 * mrSges(4,2);
t828 = t99 * pkin(24);
t829 = t238 * t828;
t832 = t99 * t228;
t834 = t102 * t233;
t836 = t238 * t832 - t238 * t834;
t838 = t99 * t233;
t840 = t102 * t228;
t842 = t238 * t838 + t238 * t840;
t847 = t329 * pkin(22);
t851 = t329 * t418;
t853 = t331 * t423;
t857 = t329 * t423;
t859 = t331 * t418;
t873 = -g(1) * (m(6) * t731 + mrSges(6,1) * t738 + mrSges(6,2) * t742 - mrSges(6,3) * t3) - g(1) * (m(7) * (t747 + t728 - t729 - t730) + t752 * mrSges(7,1) + t756 * mrSges(7,2) - t3 * mrSges(7,3)) - g(1) * (m(13) * t731 + mrSges(13,1) * t764 + mrSges(13,2) * t768 - mrSges(13,3) * t3) - g(2) * (m(6) * t773 + mrSges(6,1) * t639 + mrSges(6,2) * t650 + mrSges(6,3) * t223) - g(2) * (m(16) * (-t173 * t794 + t641 + t643 + t644 + t796) + t801 * mrSges(16,1) + t223 * mrSges(16,2) - t794 * mrSges(16,3)) - g(2) * (mrSges(4,3) * t223 + t238 * t810 + t238 * t812 + t807 * t808) - g(3) * (-m(9) * pkin(21) * t113 - mrSges(9,1) * t547 - mrSges(9,2) * t553 - mrSges(9,3) * t113) - g(3) * (mrSges(3,1) * t2 + mrSges(3,2) * t113) - g(2) * (m(11) * (t829 + t644) + t836 * mrSges(11,1) + t842 * mrSges(11,2) + t223 * mrSges(11,3)) - g(2) * (m(10) * (t238 * t847 + t644) + (-t238 * t851 + t238 * t853) * mrSges(10,1) + (-t238 * t857 - t238 * t859) * mrSges(10,2) + t223 * mrSges(10,3)) - g(1) * (m(5) * (-t729 - t730) + t727 * mrSges(5,1) + t736 * mrSges(5,2) - t3 * mrSges(5,3));
t889 = t1 * pkin(21);
t919 = -t114 * t832 + t114 * t834;
t921 = t114 * t828;
t927 = -t114 * t838 - t114 * t840;
t952 = m(9) * t2;
t954 = t329 * mrSges(9,1);
t956 = t331 * mrSges(9,2);
t1024 = t138 * t764 - t143 * t768;
t1025 = t1024 * pkin(12);
t1031 = t138 * t768 + t143 * t764;
t1033 = t1024 * t167 + t1031 * t162;
t1037 = -t1024 * t162 + t1031 * t167;
t1063 = -g(2) * (m(14) * t773 + mrSges(14,1) * t788 + mrSges(14,2) * t792 + mrSges(14,3) * t223) - g(3) * (m(5) * (t570 - t549) + t568 * mrSges(5,1) + t576 * mrSges(5,2) - t113 * mrSges(5,3)) - g(1) * (-mrSges(9,3) * t3 + t114 * t954 + t114 * t956 - t889 * t952) - g(1) * (m(11) * (-t921 - t730) + t919 * mrSges(11,1) + t927 * mrSges(11,2) - t3 * mrSges(11,3)) - g(3) * (-m(4) * pkin(21) * t113 + mrSges(4,1) * t564 + mrSges(4,2) * t566 - mrSges(4,3) * t113) - g(3) * (m(11) * (t700 - t549) + t698 * mrSges(11,1) + t706 * mrSges(11,2) - t113 * mrSges(11,3)) - g(3) * (m(13) * t571 + mrSges(13,1) * t578 + mrSges(13,2) * t582 - mrSges(13,3) * t113) - g(1) * (m(15) * (t1025 + t728 - t729 - t730) + t1033 * mrSges(15,1) + t1037 * mrSges(15,2) - t3 * mrSges(15,3)) - g(1) * (m(16) * (-t1037 * t173 + t1025 + t728 - t729 - t730) + t1033 * mrSges(16,1) - t3 * mrSges(16,2) - t1037 * mrSges(16,3)) - g(2) * (m(15) * (t796 + t641 + t643 + t644) + t801 * mrSges(15,1) + t794 * mrSges(15,2) + t223 * mrSges(15,3)) - g(1) * (m(14) * t731 + mrSges(14,1) * t1024 + mrSges(14,2) * t1031 - mrSges(14,3) * t3);
t1066 = -t27 - t28;
t1078 = -t7 + t10;
t1083 = 0.2e1 * pkin(12) * t1066 * t9 + 0.2e1 * pkin(12) * t1078 * t17 - 0.2e1 * pkin(12) * t11 * t17 + 0.2e1 * pkin(12) * t29 * t9 - 0.2e1 * t20 * t4 * t6 + 0.2e1 * t23 * t4 * t5;
t1086 = 0.1e1 / t40;
t1090 = 0.4e1 * t1066 * t11 + 0.4e1 * t1078 * t29;
t1094 = -0.2e1 * t1083 * t32 + 0.2e1 * t1090 * t12;
t1099 = t36 ^ 2;
t1100 = 0.1e1 / t1099;
t1103 = t27 + t28 + (-t1086 * t1094 * t29 + 0.2e1 * t1066 * t32 - 0.2e1 * t1078 * t40 + 0.2e1 * t1083 * t11) * t43 - 0.2e1 * t42 * t1100 * t1090;
t1115 = t7 - t10 + (t1086 * t1094 * t11 + 0.2e1 * t1066 * t40 + 0.2e1 * t1078 * t32 + 0.2e1 * t1083 * t29) * t43 - 0.2e1 * t49 * t1100 * t1090;
t1117 = t1103 * t4 + t1115 * t5 + t59 - t60;
t1118 = t1117 * t54;
t1119 = t1118 * t57;
t1122 = t1103 * t5 - t1115 * t4 - t46 - t52;
t1123 = t1122 * t54;
t1124 = t1123 * t64;
t1125 = t1123 * t57;
t1126 = t1118 * t64;
t1127 = t1125 - t1126;
t1130 = t1119 + t1124;
t1136 = -0.2e1 * t1127 * t75 + 0.2e1 * t1130 * t72;
t1140 = 0.1e1 / t86;
t1144 = -0.4e1 * t1127 * t75 + 0.4e1 * t1130 * t72;
t1148 = -0.2e1 * t1136 * t78 + 0.2e1 * t1144 * t66;
t1153 = t82 ^ 2;
t1154 = 0.1e1 / t1153;
t1157 = -t1119 - t1124 - (t1140 * t1148 * t75 - 0.2e1 * t1127 * t86 - 0.2e1 * t1130 * t78 - 0.2e1 * t1136 * t72) * t89 + 0.2e1 * t88 * t1154 * t1144;
t1160 = t91 ^ 2;
t1161 = t96 ^ 2;
t1162 = 0.1e1 / t1161;
t1165 = 0.1e1 / (t1160 * t1162 + 0.1e1);
t1166 = t1157 / t96 * t1165;
t1177 = t1125 - t1126 + (t1140 * t1148 * t72 - 0.2e1 * t1127 * t78 + 0.2e1 * t1130 * t86 + 0.2e1 * t1136 * t75) * t89 - 0.2e1 * t94 * t1154 * t1144;
t1180 = t1177 * t91 * t1162 * t1165;
t1181 = t1166 - t1180;
t1182 = t1181 * t102;
t1184 = t101 * t1181;
t1186 = -t1182 * t3 + t1184 * t99;
t1188 = t1181 * t99;
t1191 = t102 * t1184 + t1188 * t3;
t1193 = t1186 * t5 + t1191 * t4 - t121 + t122;
t1197 = -t1186 * t4 + t1191 * t5 - t105 - t109;
t1199 = t1193 * t4 - t1197 * t5 + t140 + t141;
t1203 = t1193 * t5 + t1197 * t4 - t135 + t136;
t1205 = t1199 * t138 - t1203 * t143;
t1206 = t1205 * pkin(12);
t1207 = t1193 * pkin(9);
t1208 = t1186 * pkin(23);
t1212 = -t156 + t158 - t155;
t1213 = 0.1e1 / t159;
t1215 = 0.1e1 / t170;
t1218 = 0.1e1 / (t1215 * t171 + 0.1e1);
t1220 = t153 - t148 + t151;
t1222 = t1215 * t1218;
t1224 = t1212 * t1213 * t1218 - t1220 * t1222 * t154 + 0.1e1;
t1225 = t145 * t1224;
t1229 = t1199 * t143 + t1203 * t138;
t1231 = t166 * t1224;
t1233 = t1205 * t167 + t1225 * t162 + t1229 * t162 - t1231 * t167;
t1239 = -t1205 * t162 + t1225 * t167 + t1229 * t167 + t1231 * t162;
t1244 = 0.1e1 / t173;
t1245 = t169 * t1244;
t1248 = t1212 * t154 + t1220 * t159;
t1257 = t113 * t1181;
t1259 = t113 * t99;
t1260 = t1259 * t4;
t1262 = t113 * t102;
t1263 = t1262 * t5;
t1264 = t1257 * t633 - t1257 * t635 + t1260 - t1263;
t1266 = t1259 * t5;
t1267 = t1262 * t4;
t1268 = -t1266 - t1267;
t1269 = t1268 * t5;
t1272 = t1257 * t627 + t1257 * t629 + t1266 + t1267;
t1274 = t1260 - t1263;
t1275 = t1274 * t4;
t1276 = t1264 * t4 - t1272 * t5 + t1269 + t1275;
t1279 = t1268 * t4;
t1281 = t1274 * t5;
t1282 = t1264 * t5 + t1272 * t4 - t1279 + t1281;
t1284 = t1276 * t138 - t1282 * t143;
t1285 = t1284 * pkin(12);
t1286 = t1264 * pkin(9);
t1288 = t1257 * t102 * pkin(23);
t1292 = t1279 - t1281;
t1294 = t1269 + t1275;
t1296 = t1292 * t138 - t1294 * t143;
t1297 = t1296 * t1224;
t1301 = t1276 * t143 + t1282 * t138;
t1305 = t1292 * t143 + t1294 * t138;
t1306 = t1305 * t1224;
t1308 = t1284 * t167 + t1297 * t162 + t1301 * t162 - t1306 * t167;
t1314 = -t1284 * t162 + t1297 * t167 + t1301 * t167 + t1306 * t162;
t1318 = t1286 - t1288;
t1322 = t119 * t1264 + t124 * t1272;
t1326 = t119 * t1272 - t124 * t1264;
t1330 = t1322 * pkin(10);
t1336 = t119 * t1268 + t124 * t1274;
t1337 = t1336 * t152;
t1341 = t119 * t1274 - t124 * t1268;
t1342 = t1341 * t147;
t1343 = -t1322 * t147 + t1326 * t152 + t1337 + t1342;
t1346 = t1336 * t147;
t1348 = t1341 * t152;
t1349 = -t1322 * t152 - t1326 * t147 - t1346 + t1348;
t1353 = t1207 + t1208;
t1357 = t119 * t1193 + t1197 * t124;
t1361 = t119 * t1197 - t1193 * t124;
t1365 = t1357 * pkin(10);
t1370 = -t1357 * t147 + t1361 * t152 + t193 + t194;
t1374 = -t1357 * t152 - t1361 * t147 - t189 + t190;
t1379 = t1 * t1181;
t1381 = t1182 * t223 + t1379 * t99;
t1383 = -t226 * t4;
t1386 = t102 * t1379 - t1188 * t223;
t1388 = -t232 * t5;
t1389 = t1381 * t5 + t1386 * t4 - t1383 + t1388;
t1392 = -t226 * t5;
t1394 = -t232 * t4;
t1395 = -t1381 * t4 + t1386 * t5 - t1392 - t1394;
t1397 = t119 * t1389 + t124 * t1395;
t1398 = t1397 * pkin(10);
t1399 = t1389 * pkin(9);
t1400 = t1381 * pkin(23);
t1404 = t1392 + t1394;
t1406 = -t1383 + t1388;
t1408 = t119 * t1404 + t124 * t1406;
t1409 = t1408 * t152;
t1412 = t119 * t1395 - t124 * t1389;
t1416 = t119 * t1406 - t124 * t1404;
t1417 = t1416 * t147;
t1418 = -t1397 * t147 + t1412 * t152 + t1409 + t1417;
t1421 = t1408 * t147;
t1423 = t1416 * t152;
t1424 = -t1397 * t152 - t1412 * t147 - t1421 + t1423;
t1465 = t53 ^ 2;
t1466 = t61 ^ 2;
t1467 = 0.1e1 / t1466;
t1470 = 0.1e1 / (t1465 * t1467 + 0.1e1);
t1471 = -t1117 / t61 * t1470;
t1474 = -t1122 * t53 * t1467 * t1470;
t1475 = t1471 - t1474;
t1476 = t104 * t1475;
t1479 = t108 * t1475;
t1481 = t1186 * t228 - t1191 * t233 - t1476 * t233 - t1479 * t228;
t1487 = t1186 * t233 + t1191 * t228 + t1476 * t228 - t1479 * t233;
t1496 = t1399 + t1400;
t1513 = t1404 * t5;
t1515 = t1406 * t4;
t1516 = t1389 * t4 - t1395 * t5 + t1513 + t1515;
t1519 = t1404 * t4;
t1521 = t1406 * t5;
t1522 = t1389 * t5 + t1395 * t4 - t1519 + t1521;
t1524 = t138 * t1516 - t143 * t1522;
t1525 = t1524 * pkin(12);
t1529 = t1519 - t1521;
t1531 = t1513 + t1515;
t1533 = t138 * t1529 - t143 * t1531;
t1534 = t1533 * t1224;
t1538 = t138 * t1522 + t143 * t1516;
t1542 = t138 * t1531 + t143 * t1529;
t1543 = t1542 * t1224;
t1545 = t1524 * t167 + t1534 * t162 + t1538 * t162 - t1543 * t167;
t1551 = -t1524 * t162 + t1534 * t167 + t1538 * t167 + t1543 * t162;
t1560 = -g(1) * (m(15) * (t1206 + t1207 + t1208) + t1233 * mrSges(15,1) + t1239 * mrSges(15,2)) - g(1) * (m(16) * (-t1239 * t173 - t1245 * t1248 + t1206 + t1207 + t1208) + t1233 * mrSges(16,1) - t1239 * mrSges(16,3)) - g(3) * (m(15) * (t1285 + t1286 - t1288) + t1308 * mrSges(15,1) + t1314 * mrSges(15,2)) - g(3) * (m(6) * t1318 + mrSges(6,1) * t1322 + mrSges(6,2) * t1326) - g(3) * (m(7) * (t1330 + t1286 - t1288) + t1343 * mrSges(7,1) + t1349 * mrSges(7,2)) - g(1) * (m(6) * t1353 + mrSges(6,1) * t1357 + mrSges(6,2) * t1361) - g(1) * (m(7) * (t1365 + t1207 + t1208) + t1370 * mrSges(7,1) + t1374 * mrSges(7,2)) - g(2) * (m(7) * (t1398 + t1399 + t1400) + t1418 * mrSges(7,1) + t1424 * mrSges(7,2)) - g(2) * (m(8) * (-pkin(13) * t1424 + t1398 + t1399 + t1400) + t1418 * t203 * mrSges(8,1) - t1418 * t205 * mrSges(8,2) - t1424 * mrSges(8,3)) - g(3) * (m(8) * (-pkin(13) * t1349 + t1286 - t1288 + t1330) + t1343 * t203 * mrSges(8,1) - t1343 * t205 * mrSges(8,2) - t1349 * mrSges(8,3)) - g(3) * (m(13) * t1318 + mrSges(13,1) * t1276 + mrSges(13,2) * t1282) - g(3) * (-m(5) * pkin(23) * t113 * t1182 + mrSges(5,1) * t1264 + mrSges(5,2) * t1272) - g(1) * (m(11) * pkin(24) * t1186 + mrSges(11,1) * t1481 + mrSges(11,2) * t1487) - g(1) * (m(13) * t1353 + mrSges(13,1) * t1199 + mrSges(13,2) * t1203) - g(2) * (m(6) * t1496 + mrSges(6,1) * t1397 + mrSges(6,2) * t1412) - g(1) * (m(8) * (-pkin(13) * t1374 + t1207 + t1208 + t1365) + t1370 * t203 * mrSges(8,1) - t1370 * t205 * mrSges(8,2) - t1374 * mrSges(8,3)) - g(2) * (m(15) * (t1525 + t1399 + t1400) + t1545 * mrSges(15,1) + t1551 * mrSges(15,2)) - g(1) * (m(14) * t1353 + mrSges(14,1) * t1205 + mrSges(14,2) * t1229);
t1561 = t113 * t329;
t1562 = t1157 * t397;
t1564 = -t1177 * t397;
t1566 = t1562 * t329 + t1564 * t331;
t1570 = t1562 * t331 - t1564 * t329;
t1577 = t410 ^ 2;
t1578 = t415 ^ 2;
t1579 = 0.1e1 / t1578;
t1582 = 0.1e1 / (t1577 * t1579 + 0.1e1);
t1592 = t1166 - t1180 - (-(-t1566 * t331 + t1570 * t329) * pkin(22) + t1125 - t1126) / t415 * t1582 + ((t1566 * t329 + t1570 * t331) * pkin(22) + t1119 + t1124) * t410 * t1579 * t1582;
t1593 = t1592 * t423;
t1595 = t113 * t331;
t1596 = t1592 * t418;
t1615 = (-t1296 * t162 + t1305 * t167) * t1244;
t1628 = (-t1533 * t162 + t1542 * t167) * t1244;
t1640 = -t226 * t1475;
t1643 = -t232 * t1475;
t1645 = t1381 * t228 - t1386 * t233 - t1640 * t233 - t1643 * t228;
t1651 = t1381 * t233 + t1386 * t228 + t1640 * t228 - t1643 * t233;
t1655 = -t333 * t1592;
t1657 = -t337 * t1592;
t1688 = t249 ^ 2;
t1689 = t256 ^ 2;
t1690 = 0.1e1 / t1689;
t1693 = 0.1e1 / (t1688 * t1690 + 0.1e1);
t1705 = -((-t1117 * t5 * t54 + t1122 * t4 * t54 + t251 + t253) * pkin(19) + t7 - t10) / t256 * t1693 + ((t1117 * t4 * t54 + t1122 * t5 * t54 - t244 + t246) * pkin(19) - t27 - t28) * t249 * t1690 * t1693 + 0.1e1 + t1471 - t1474;
t1706 = (-t226 * t228 + t232 * t233) * t1705;
t1712 = (-t226 * t233 - t228 * t232) * t1705;
t1731 = t1475 * t233;
t1734 = t1475 * t228;
t1736 = -t1257 * t838 - t1257 * t840 + t1259 * t1731 + t1262 * t1734;
t1746 = (-t1259 * t228 + t1262 * t233) * t1705;
t1752 = t1257 * t832 - t1257 * t834 - t1259 * t1734 + t1262 * t1731;
t1757 = (-t1259 * t233 - t1262 * t228) * t1705;
t1801 = t445 * t1705;
t1804 = t449 * t1705;
t1822 = t393 * t1592;
t1824 = t422 * t1592;
t1834 = -g(3) * ((t1561 * t1593 + t1595 * t1596) * mrSges(10,1) + (-t1561 * t1596 + t1593 * t1595) * mrSges(10,2)) - g(3) * (m(14) * t1318 + mrSges(14,1) * t1284 + mrSges(14,2) * t1301) - g(3) * (m(16) * (-t1248 * t1615 - t1314 * t173 + t1285 + t1286 - t1288) + t1308 * mrSges(16,1) - t1314 * mrSges(16,3)) - g(2) * (m(16) * (-t1248 * t1628 - t1551 * t173 + t1399 + t1400 + t1525) + t1545 * mrSges(16,1) - t1551 * mrSges(16,3)) - g(2) * (m(11) * pkin(24) * t1381 + mrSges(11,1) * t1645 + mrSges(11,2) * t1651) - g(2) * ((t1655 * t423 + t1657 * t418) * mrSges(10,1) + (-t1655 * t418 + t1657 * t423) * mrSges(10,2)) - g(1) * (mrSges(4,1) * t1186 + mrSges(4,2) * t1191) - g(2) * (m(12) * (pkin(19) * t1645 + pkin(24) * t1381) + (-t1645 * t259 - t1651 * t264 + t1706 * t264 - t1712 * t259) * mrSges(12,1) + (t1645 * t264 - t1651 * t259 + t1706 * t259 + t1712 * t264) * mrSges(12,2)) - g(2) * (m(5) * pkin(23) * t1381 + mrSges(5,1) * t1389 + mrSges(5,2) * t1395) - g(3) * (m(12) * (-pkin(24) * t102 * t1257 + pkin(19) * t1736) + (-t1736 * t259 + t1746 * t264 - t1752 * t264 - t1757 * t259) * mrSges(12,1) + (t1736 * t264 + t1746 * t259 - t1752 * t259 + t1757 * t264) * mrSges(12,2)) - g(3) * (-mrSges(4,1) * t102 * t1257 + mrSges(4,2) * t1257 * t99) - g(3) * (-m(11) * pkin(24) * t113 * t1182 + mrSges(11,1) * t1736 + mrSges(11,2) * t1752) - g(2) * (m(13) * t1496 + mrSges(13,1) * t1516 + mrSges(13,2) * t1522) - g(2) * (m(14) * t1496 + mrSges(14,1) * t1524 + mrSges(14,2) * t1538) - g(2) * (mrSges(4,1) * t1381 + mrSges(4,2) * t1386) - g(1) * (m(12) * (pkin(19) * t1481 + pkin(24) * t1186) + (-t1481 * t259 - t1487 * t264 + t1801 * t264 - t1804 * t259) * mrSges(12,1) + (t1481 * t264 - t1487 * t259 + t1801 * t259 + t1804 * t264) * mrSges(12,2)) - g(1) * (m(5) * pkin(23) * t1186 + mrSges(5,1) * t1193 + mrSges(5,2) * t1197) - g(1) * ((t1822 * t423 + t1824 * t418) * mrSges(10,1) + (-t1822 * t418 + t1824 * t423) * mrSges(10,2));
t1844 = -t1212 * t1213 * t1218 + t1220 * t1222 * t154 - 0.1e1;
t1845 = t1533 * t1844;
t1847 = t1542 * t1844;
t1849 = t162 * t1847 + t167 * t1845;
t1853 = -t1212 * t154 - t1220 * t159;
t1860 = t162 * t1845 - t167 * t1847;
t1865 = t145 * t1844;
t1867 = t166 * t1844;
t1869 = t162 * t1865 - t167 * t1867;
t1873 = t162 * t1867 + t167 * t1865;
t1890 = t1296 * t1844;
t1892 = t1305 * t1844;
t1894 = t162 * t1892 + t167 * t1890;
t1902 = t162 * t1890 - t167 * t1892;
t1919 = -t1336 * t152 - t1337 - 0.2e1 * t1342;
t1923 = t1336 * t147 + t1346 - 0.2e1 * t1348;
t1935 = -t126 * t152 - t193 - 0.2e1 * t194;
t1939 = t126 * t147 + t189 - 0.2e1 * t190;
t1947 = -t1408 * t152 - t1409 - 0.2e1 * t1417;
t1951 = t1408 * t147 + t1421 - 0.2e1 * t1423;
t1988 = -g(2) * (mrSges(6,1) * t1416 - mrSges(6,2) * t1408) - g(2) * (m(16) * (-t1628 * t1853 - t173 * t1849) + t1860 * mrSges(16,1) - t1849 * mrSges(16,3)) - g(1) * (mrSges(15,1) * t1869 + mrSges(15,2) * t1873) - g(1) * (m(16) * (-t1245 * t1853 - t173 * t1873) + t1869 * mrSges(16,1) - t1873 * mrSges(16,3)) - g(2) * (mrSges(15,1) * t1860 + mrSges(15,2) * t1849) - g(3) * (m(16) * (-t1615 * t1853 - t173 * t1894) + t1902 * mrSges(16,1) - t1894 * mrSges(16,3)) - g(3) * (mrSges(15,1) * t1902 + mrSges(15,2) * t1894) - g(3) * (mrSges(6,1) * t1341 - mrSges(6,2) * t1336) - g(3) * (m(7) * pkin(10) * t1341 + mrSges(7,1) * t1919 + mrSges(7,2) * t1923) - g(1) * (mrSges(6,1) * t130 - mrSges(6,2) * t126) - g(1) * (m(7) * pkin(10) * t130 + mrSges(7,1) * t1935 + mrSges(7,2) * t1939) - g(2) * (m(7) * pkin(10) * t1416 + mrSges(7,1) * t1947 + mrSges(7,2) * t1951) - g(2) * (m(8) * (pkin(10) * t1416 - pkin(13) * t1951) + t1947 * t203 * mrSges(8,1) - t1947 * t205 * mrSges(8,2) - t1951 * mrSges(8,3)) - g(3) * (m(8) * (pkin(10) * t1341 - pkin(13) * t1923) + t1919 * t203 * mrSges(8,1) - t1919 * t205 * mrSges(8,2) - t1923 * mrSges(8,3)) - g(1) * (m(8) * (pkin(10) * t130 - pkin(13) * t1939) + t1935 * t203 * mrSges(8,1) - t1935 * t205 * mrSges(8,2) - t1939 * mrSges(8,3));
t1989 = -t1421 + t1423;
t1998 = -t1346 + t1348;
unknown(1) = t382 + t545;
unknown(2) = -g(3) * (m(10) * (pkin(22) * t547 - t549) + (-t418 * t547 + t423 * t553) * mrSges(10,1) + (-t418 * t553 - t423 * t547) * mrSges(10,2) - t113 * mrSges(10,3)) - g(3) * (m(14) * t571 + mrSges(14,1) * t584 + mrSges(14,2) * t588 - mrSges(14,3) * t113) - g(3) * (m(16) * (-t173 * t595 - t549 + t569 + t570 + t597) + t602 * mrSges(16,1) - t113 * mrSges(16,2) - t595 * mrSges(16,3)) - g(3) * (m(15) * (t597 + t569 + t570 - t549) + t602 * mrSges(15,1) + t595 * mrSges(15,2) - t113 * mrSges(15,3)) - g(3) * (m(6) * t571 + mrSges(6,1) * t618 + mrSges(6,2) * t622 - mrSges(6,3) * t113) - g(2) * (m(7) * (t640 + t641 + t643 + t644) + t652 * mrSges(7,1) + t656 * mrSges(7,2) + t223 * mrSges(7,3)) - g(2) * (m(8) * (-pkin(13) * t656 + t640 + t641 + t643 + t644) + (t203 * t652 + t205 * t223) * mrSges(8,1) + (t203 * t223 - t205 * t652) * mrSges(8,2) - t656 * mrSges(8,3)) - g(3) * (m(8) * (-pkin(13) * t677 - t549 + t569 + t570 + t679) + (-t113 * t205 + t203 * t684) * mrSges(8,1) + (-t113 * t203 - t205 * t684) * mrSges(8,2) - t677 * mrSges(8,3)) - g(3) * (m(12) * (pkin(19) * t698 - t549 + t700) + (-t259 * t698 - t264 * t706) * mrSges(12,1) + (-t259 * t706 + t264 * t698) * mrSges(12,2) - t113 * mrSges(12,3)) - g(3) * (m(7) * (t679 + t569 + t570 - t549) + t684 * mrSges(7,1) + t677 * mrSges(7,2) - t113 * mrSges(7,3)) + t873 - g(1) * (m(10) * (-t114 * t847 - t730) + (t114 * t851 - t114 * t853) * mrSges(10,1) + (t114 * t857 + t114 * t859) * mrSges(10,2) - t3 * mrSges(10,3)) - g(1) * (-mrSges(4,3) * t3 - t114 * t810 - t114 * t812 - t807 * t889) - g(2) * (m(12) * (pkin(19) * t836 + t644 + t829) + (-t259 * t836 - t264 * t842) * mrSges(12,1) + (-t259 * t842 + t264 * t836) * mrSges(12,2) + t223 * mrSges(12,3)) - g(2) * (m(5) * (t643 + t644) + t631 * mrSges(5,1) + t637 * mrSges(5,2) + t223 * mrSges(5,3)) - g(1) * (m(12) * (pkin(19) * t919 - t730 - t921) + (-t259 * t919 - t264 * t927) * mrSges(12,1) + (-t259 * t927 + t264 * t919) * mrSges(12,2) - t3 * mrSges(12,3)) - g(1) * (m(8) * (-pkin(13) * t756 + t728 - t729 - t730 + t747) + (t203 * t752 - t205 * t3) * mrSges(8,1) + (-t203 * t3 - t205 * t752) * mrSges(8,2) - t756 * mrSges(8,3)) - g(2) * (mrSges(9,3) * t223 - t238 * t954 - t238 * t956 + t808 * t952) - g(2) * (mrSges(3,1) * t238 - mrSges(3,2) * t223) - g(1) * (-mrSges(3,1) * t114 + mrSges(3,2) * t3) - g(2) * (m(13) * t773 + mrSges(13,1) * t782 + mrSges(13,2) * t786 + mrSges(13,3) * t223) + t1063;
unknown(3) = t1560 + t1834;
unknown(4) = t1988;
unknown(5) = -g(2) * ((-t1989 * t205 - t462) * mrSges(8,1) + (-t1989 * t203 + t458) * mrSges(8,2)) - g(3) * ((-t1998 * t205 - t2 * t203) * mrSges(8,1) + (-t1998 * t203 + t2 * t205) * mrSges(8,2)) - g(1) * (mrSges(8,1) * t211 - mrSges(8,2) * t207);
taug = unknown(:);