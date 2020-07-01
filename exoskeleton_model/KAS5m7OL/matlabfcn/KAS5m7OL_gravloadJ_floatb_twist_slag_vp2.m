% Calculate Gravitation load on the joints for
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
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% taug [13x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7OL_gravloadJ_floatb_twist_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From gravload_joint_floatb_twist_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:31
% EndTime: 2020-06-30 17:46:33
% DurationCPUTime: 0.50s
% Computational Cost: add. (3061->829), mult. (2584->984), div. (0->0), fcn. (2505->28), ass. (0->311)
unknown=NaN(13,1);
t1 = sin(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = qJ(3) + qJ(9);
t5 = cos(t4);
t7 = cos(qJ(3));
t8 = t7 * pkin(19);
t9 = pkin(14) * t5 + t8;
t11 = cos(qJ(2));
t12 = t1 * t11;
t13 = t12 * pkin(16);
t14 = cos(qJ(1));
t15 = sin(t4);
t16 = pkin(14) * t15;
t17 = sin(qJ(3));
t18 = t17 * pkin(19);
t19 = t16 + t18;
t21 = t14 * pkin(11);
t24 = qJ(3) + qJ(9) + qJ(10);
t25 = cos(t24);
t27 = sin(t24);
t29 = -t14 * t27 + t25 * t3;
t33 = -t14 * t25 - t27 * t3;
t38 = qJ(3) + qJ(4);
t39 = cos(t38);
t40 = pkin(6) * t39;
t41 = t7 * pkin(18);
t42 = t40 + t41;
t44 = sin(t38);
t45 = pkin(6) * t44;
t46 = t17 * pkin(18);
t47 = t45 + t46;
t49 = t14 * t47 - t3 * t42 + t13 + t21;
t51 = qJ(3) + qJ(4) + qJ(11);
t52 = sin(t51);
t54 = cos(t51);
t56 = -t14 * t54 - t3 * t52;
t60 = t14 * t52 - t3 * t54;
t65 = t14 * t2;
t67 = t14 * t11;
t68 = t67 * pkin(16);
t70 = t1 * pkin(11);
t75 = t1 * t27 + t25 * t65;
t79 = t1 * t25 - t27 * t65;
t84 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t85 = cos(t84);
t87 = sin(t84);
t89 = -t14 * t87 + t3 * t85;
t91 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t92 = cos(t91);
t93 = pkin(9) * t92;
t94 = t40 - t93 + t41;
t95 = t3 * t94;
t96 = sin(t91);
t97 = pkin(9) * t96;
t98 = t45 - t97 + t46;
t99 = t14 * t98;
t104 = t14 * t85 + t3 * t87;
t110 = qJ(3) + qJ(4) + qJ(5);
t111 = cos(t110);
t112 = pkin(7) * t111;
t113 = t112 + t40 + t41;
t114 = t3 * t113;
t115 = sin(t110);
t116 = pkin(7) * t115;
t117 = t116 + t45 + t46;
t118 = t14 * t117;
t121 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t122 = sin(t121);
t124 = cos(t121);
t126 = t122 * t3 + t124 * t14;
t130 = -t122 * t14 + t124 * t3;
t138 = cos(qJ(7));
t140 = sin(qJ(7));
t142 = t12 * t140 + t126 * t138;
t146 = t12 * t138 - t126 * t140;
t153 = t1 * t122 + t124 * t65;
t155 = t65 * t113;
t156 = t1 * t117;
t161 = -t1 * t124 + t122 * t65;
t163 = t67 * t140;
t167 = t67 * t138;
t176 = -t14 * t96 + t3 * t92;
t180 = -t14 * t92 - t3 * t96;
t187 = -t1 * t47 - t42 * t65 + t68 - t70;
t191 = t1 * t54 - t52 * t65;
t195 = -t1 * t52 - t54 * t65;
t203 = t1 * t96 + t65 * t92;
t207 = t1 * t92 - t65 * t96;
t219 = t65 * t94;
t220 = t1 * t98;
t225 = -t1 * t85 + t65 * t87;
t229 = t1 * t87 + t65 * t85;
t249 = t68 - t70;
t251 = sin(pkin(3));
t253 = cos(pkin(3));
t254 = t1 * t253;
t264 = -g(2) * (m(12) * (t14 * t19 - t3 * t9 + t13 + t21) + t29 * mrSges(12,1) + t33 * mrSges(12,2) + t12 * mrSges(12,3)) - g(2) * (m(13) * t49 + mrSges(13,1) * t56 + mrSges(13,2) * t60 + mrSges(13,3) * t12) - g(1) * (m(12) * (-t1 * t19 - t65 * t9 + t68 - t70) + t75 * mrSges(12,1) + t79 * mrSges(12,2) + t67 * mrSges(12,3)) - g(2) * (m(16) * (-qJ(13) * t89 + t13 + t21 - t95 + t99) + t104 * mrSges(16,1) + t12 * mrSges(16,2) - t89 * mrSges(16,3)) - g(2) * (m(7) * (-t114 + t13 + t118 + t21) + t126 * mrSges(7,1) + t130 * mrSges(7,2) + t12 * mrSges(7,3)) - g(2) * (m(8) * (-pkin(10) * t130 - t114 + t118 + t13 + t21) + t142 * mrSges(8,1) + t146 * mrSges(8,2) - t130 * mrSges(8,3)) - g(1) * (m(8) * (-pkin(10) * t153 - t155 - t156 + t68 - t70) + (t138 * t161 + t163) * mrSges(8,1) + (-t140 * t161 + t167) * mrSges(8,2) - t153 * mrSges(8,3)) - g(2) * (m(14) * t49 + mrSges(14,1) * t176 + mrSges(14,2) * t180 + mrSges(14,3) * t12) - g(1) * (m(13) * t187 + mrSges(13,1) * t191 + mrSges(13,2) * t195 + mrSges(13,3) * t67) - g(1) * (m(14) * t187 + mrSges(14,1) * t203 + mrSges(14,2) * t207 + mrSges(14,3) * t67) - g(2) * (m(15) * (-t95 + t13 + t99 + t21) + t104 * mrSges(15,1) + t89 * mrSges(15,2) + t12 * mrSges(15,3)) - g(1) * (m(15) * (-t219 + t68 - t220 - t70) + t225 * mrSges(15,1) + t229 * mrSges(15,2) + t67 * mrSges(15,3)) - g(1) * (m(16) * (-qJ(13) * t229 - t219 - t220 + t68 - t70) + t225 * mrSges(16,1) + t67 * mrSges(16,2) - t229 * mrSges(16,3)) - g(1) * (-m(3) * pkin(11) * t1 - mrSges(3,1) * t65 - mrSges(3,2) * t67 - mrSges(3,3) * t1) - g(1) * (m(9) * t249 + (t251 * t65 - t254) * mrSges(9,1) + (t1 * t251 + t253 * t65) * mrSges(9,2) + t67 * mrSges(9,3));
t275 = -t1 * t115 - t111 * t65;
t279 = -t1 * t111 + t115 * t65;
t287 = -t111 * t3 + t115 * t14;
t291 = t111 * t14 + t115 * t3;
t304 = t1 * t17;
t310 = -t1 * t44 - t39 * t65;
t314 = -t1 * t39 + t44 * t65;
t320 = t14 * t17;
t326 = t14 * t44 - t3 * t39;
t330 = t14 * t39 + t3 * t44;
t335 = t251 * pkin(17);
t337 = t14 * t253;
t341 = pkin(3) + qJ(8);
t342 = sin(t341);
t344 = cos(t341);
t346 = t14 * t344 + t3 * t342;
t350 = -t14 * t342 + t3 * t344;
t361 = -t1 * t344 + t342 * t65;
t365 = t1 * t342 + t344 * t65;
t376 = -t1 * t15 - t5 * t65;
t380 = -t1 * t5 + t15 * t65;
t391 = t14 * t15 - t3 * t5;
t395 = t14 * t5 + t15 * t3;
t400 = t13 + t21;
t422 = -t65 * t7 - t304;
t425 = t1 * t7;
t426 = t17 * t65 - t425;
t433 = -t3 * t7 + t320;
t436 = t14 * t7;
t437 = t17 * t3 + t436;
t442 = -g(2) * (m(3) * pkin(11) * t14 - mrSges(3,1) * t3 - mrSges(3,2) * t12 + mrSges(3,3) * t14) - g(1) * (m(6) * t187 + mrSges(6,1) * t275 + mrSges(6,2) * t279 + mrSges(6,3) * t67) - g(2) * (m(6) * t49 + mrSges(6,1) * t287 + mrSges(6,2) * t291 + mrSges(6,3) * t12) - g(1) * (m(7) * (-t155 + t68 - t156 - t70) + t161 * mrSges(7,1) + t153 * mrSges(7,2) + t67 * mrSges(7,3)) - g(1) * (m(5) * (-pkin(18) * t304 - t41 * t65 + t68 - t70) + t310 * mrSges(5,1) + t314 * mrSges(5,2) + t67 * mrSges(5,3)) - g(2) * (m(5) * (pkin(18) * t320 - t3 * t41 + t13 + t21) + t326 * mrSges(5,1) + t330 * mrSges(5,2) + t12 * mrSges(5,3)) - g(2) * (m(10) * (-pkin(17) * t337 - t3 * t335 + t13 + t21) + t346 * mrSges(10,1) + t350 * mrSges(10,2) + t12 * mrSges(10,3)) - g(1) * (m(10) * (pkin(17) * t254 - t335 * t65 + t68 - t70) + t361 * mrSges(10,1) + t365 * mrSges(10,2) + t67 * mrSges(10,3)) - g(1) * (m(11) * (-pkin(19) * t304 - t65 * t8 + t68 - t70) + t376 * mrSges(11,1) + t380 * mrSges(11,2) + t67 * mrSges(11,3)) - g(2) * (m(11) * (pkin(19) * t320 - t3 * t8 + t13 + t21) + t391 * mrSges(11,1) + t395 * mrSges(11,2) + t12 * mrSges(11,3)) - g(2) * (m(9) * t400 + (t251 * t3 + t337) * mrSges(9,1) + (-t14 * t251 + t253 * t3) * mrSges(9,2) + t12 * mrSges(9,3)) - g(1) * (mrSges(2,1) * t14 - mrSges(2,2) * t1) - g(2) * (mrSges(2,1) * t1 + mrSges(2,2) * t14) - g(1) * (m(4) * t249 + mrSges(4,1) * t422 + mrSges(4,2) * t426 + mrSges(4,3) * t67) - g(2) * (m(4) * t400 + mrSges(4,1) * t433 + mrSges(4,2) * t437 + mrSges(4,3) * t12);
t444 = t124 * pkin(10);
t446 = t67 * t113;
t447 = t65 * pkin(16);
t450 = t122 * t138;
t455 = t122 * t140;
t460 = t124 * mrSges(8,3);
t464 = t2 * t124;
t466 = t2 * t113;
t467 = t11 * pkin(16);
t470 = t2 * t122;
t483 = t12 * t113;
t484 = t3 * pkin(16);
t499 = t2 * t42 - t467;
t509 = -t12 * t42 - t484;
t511 = t52 * mrSges(13,1);
t513 = t54 * mrSges(13,2);
t519 = t92 * mrSges(14,1);
t521 = t96 * mrSges(14,2);
t526 = t67 * t94;
t529 = t87 * mrSges(15,1);
t531 = t85 * mrSges(15,2);
t536 = t2 * t94;
t539 = t2 * t87;
t541 = t2 * t85;
t546 = t12 * t94;
t554 = t85 * qJ(13);
t558 = t87 * mrSges(16,1);
t561 = t85 * mrSges(16,3);
t577 = t25 * mrSges(12,1);
t579 = t27 * mrSges(12,2);
t585 = t42 * t67 + t447;
t606 = m(4) * t2;
t607 = t1 * pkin(16);
t609 = t7 * mrSges(4,1);
t611 = t17 * mrSges(4,2);
t616 = t14 * pkin(16);
t629 = t2 * t251;
t640 = m(9) * t2;
t642 = t251 * mrSges(9,1);
t644 = t253 * mrSges(9,2);
t649 = -g(1) * (m(16) * (-t12 * t554 - t484 - t546) + t12 * t558 - t3 * mrSges(16,2) - t12 * t561) - g(2) * (m(12) * (t67 * t9 + t447) - t67 * t577 + t67 * t579 + t65 * mrSges(12,3)) - g(2) * (m(13) * t585 + mrSges(13,3) * t65 + t511 * t67 + t513 * t67) - g(2) * (m(14) * t585 + mrSges(14,3) * t65 - t519 * t67 + t521 * t67) - g(3) * (mrSges(13,1) * t2 * t52 + mrSges(13,2) * t2 * t54 + m(13) * t499 - mrSges(13,3) * t11) - g(1) * (-mrSges(4,3) * t3 - t12 * t609 + t12 * t611 - t606 * t607) - g(2) * (mrSges(4,3) * t65 + t606 * t616 + t609 * t67 - t611 * t67) - g(3) * (mrSges(3,1) * t2 + mrSges(3,2) * t11) - g(3) * (-m(9) * pkin(16) * t11 - mrSges(9,2) * t2 * t253 - mrSges(9,1) * t629 - mrSges(9,3) * t11) - g(1) * (-mrSges(3,1) * t12 + mrSges(3,2) * t3) - g(1) * (-mrSges(9,3) * t3 + t12 * t642 + t12 * t644 - t607 * t640);
t670 = t111 * mrSges(6,1);
t672 = t115 * mrSges(6,2);
t692 = t122 * mrSges(7,1);
t694 = t124 * mrSges(7,2);
t699 = t2 * t7;
t713 = t39 * mrSges(5,1);
t715 = t44 * mrSges(5,2);
t740 = t342 * mrSges(10,1);
t742 = t344 * mrSges(10,2);
t768 = t5 * mrSges(11,1);
t770 = t15 * mrSges(11,2);
t826 = -g(3) * (-m(4) * pkin(16) * t11 - mrSges(4,2) * t17 * t2 + mrSges(4,1) * t699 - mrSges(4,3) * t11) - g(2) * (m(10) * (t335 * t67 + t447) - t67 * t740 - t67 * t742 + t65 * mrSges(10,3)) - g(3) * (m(10) * (pkin(17) * t629 - t467) - t2 * t342 * mrSges(10,1) - t2 * t344 * mrSges(10,2) - t11 * mrSges(10,3)) - g(1) * (m(10) * (-t12 * t335 - t484) + t12 * t740 + t12 * t742 - t3 * mrSges(10,3)) - g(1) * (m(11) * (-t12 * t8 - t484) - t12 * t768 + t12 * t770 - t3 * mrSges(11,3)) - g(3) * (m(11) * (pkin(19) * t699 - t467) + t2 * t5 * mrSges(11,1) - t2 * t15 * mrSges(11,2) - t11 * mrSges(11,3)) - g(2) * (m(11) * (t67 * t8 + t447) + t67 * t768 - t67 * t770 + t65 * mrSges(11,3)) - g(1) * (m(12) * (-t12 * t9 - t484) + t12 * t577 - t12 * t579 - t3 * mrSges(12,3)) - g(3) * (m(12) * (t2 * t9 - t467) - t2 * t25 * mrSges(12,1) + t2 * t27 * mrSges(12,2) - t11 * mrSges(12,3)) - g(3) * (m(16) * (qJ(13) * t541 - t467 + t536) - t539 * mrSges(16,1) - t11 * mrSges(16,2) + t541 * mrSges(16,3)) - g(2) * (m(7) * (t446 + t447) - t67 * t692 - t67 * t694 + t65 * mrSges(7,3));
t831 = t1 * t42 - t47 * t65;
t833 = -t207 * mrSges(14,1);
t834 = t203 * mrSges(14,2);
t841 = -t79 * mrSges(12,1);
t842 = t75 * mrSges(12,2);
t845 = -t104 * qJ(13);
t846 = -t3 * t98;
t847 = t14 * t94;
t850 = t89 * mrSges(16,1);
t851 = t104 * mrSges(16,3);
t855 = -t279 * mrSges(6,1);
t856 = t275 * mrSges(6,2);
t859 = m(7) * t11;
t861 = t11 * t124;
t862 = t861 * mrSges(7,1);
t863 = t11 * t122;
t864 = t863 * mrSges(7,2);
t867 = -t3 * t117;
t868 = t14 * t113;
t871 = t130 * mrSges(7,1);
t872 = -t126 * mrSges(7,2);
t875 = t863 * pkin(10);
t880 = t861 * t138 * mrSges(8,1);
t882 = t861 * t140 * mrSges(8,2);
t883 = t863 * mrSges(8,3);
t886 = -t126 * pkin(10);
t890 = t130 * t138 * mrSges(8,1);
t892 = t130 * t140 * mrSges(8,2);
t893 = t126 * mrSges(8,3);
t900 = t395 * mrSges(11,1);
t901 = -t391 * mrSges(11,2);
t908 = t11 * t15 * mrSges(11,1);
t910 = t11 * t5 * mrSges(11,2);
t917 = -t380 * mrSges(11,1);
t918 = t376 * mrSges(11,2);
t921 = t225 * qJ(13);
t922 = -t65 * t98;
t923 = t1 * t94;
t926 = -t229 * mrSges(16,1);
t927 = -t225 * mrSges(16,3);
t930 = t11 * t87;
t931 = t930 * qJ(13);
t935 = t11 * t85;
t936 = t935 * mrSges(16,1);
t937 = t930 * mrSges(16,3);
t940 = -t65 * t117;
t941 = t1 * t113;
t944 = -t153 * mrSges(7,1);
t945 = t161 * mrSges(7,2);
t948 = t161 * pkin(10);
t952 = -t153 * t138 * mrSges(8,1);
t954 = -t153 * t140 * mrSges(8,2);
t955 = -t161 * mrSges(8,3);
t958 = m(13) * t11;
t961 = t11 * t54 * mrSges(13,1);
t963 = t11 * t52 * mrSges(13,2);
t966 = -g(2) * (m(14) * t831 + t833 + t834) - g(2) * (m(12) * (t1 * t9 - t19 * t65) + t841 + t842) - g(1) * (m(16) * (-t845 - t846 + t847) + t850 + t851) - g(2) * (m(6) * t831 + t855 + t856) - g(3) * (t117 * t859 + t862 - t864) - g(1) * (m(7) * (-t867 + t868) + t871 + t872) - g(3) * (m(8) * (t11 * t117 + t875) + t880 - t882 + t883) - g(1) * (m(8) * (-t886 - t867 + t868) + t890 - t892 + t893) - g(1) * (m(11) * (pkin(19) * t436 + t18 * t3) + t900 + t901) - g(3) * (m(11) * pkin(19) * t11 * t17 + t908 + t910) - g(2) * (m(11) * (pkin(19) * t425 - t18 * t65) + t917 + t918) - g(2) * (m(16) * (-t921 + t922 + t923) + t926 + t927) - g(3) * (m(16) * (t11 * t98 + t931) + t936 + t937) - g(2) * (m(7) * (t940 + t941) + t944 + t945) - g(2) * (m(8) * (-t948 + t940 + t941) + t952 - t954 + t955) - g(3) * (t47 * t958 - t961 + t963);
t967 = m(14) * t11;
t970 = t11 * t96 * mrSges(14,1);
t972 = t11 * t92 * mrSges(14,2);
t977 = t14 * t42 + t3 * t47;
t979 = t60 * mrSges(13,1);
t980 = -t56 * mrSges(13,2);
t984 = t180 * mrSges(14,1);
t985 = -t176 * mrSges(14,2);
t990 = -t229 * mrSges(15,1);
t991 = t225 * mrSges(15,2);
t994 = m(15) * t11;
t996 = t935 * mrSges(15,1);
t997 = t930 * mrSges(15,2);
t1002 = t89 * mrSges(15,1);
t1003 = -t104 * mrSges(15,2);
t1007 = -t195 * mrSges(13,1);
t1008 = t191 * mrSges(13,2);
t1015 = t11 * t44 * mrSges(5,1);
t1017 = t11 * t39 * mrSges(5,2);
t1024 = t330 * mrSges(5,1);
t1025 = -t326 * mrSges(5,2);
t1032 = -t314 * mrSges(5,1);
t1033 = t310 * mrSges(5,2);
t1054 = t33 * mrSges(12,1);
t1055 = -t29 * mrSges(12,2);
t1058 = m(12) * t11;
t1061 = t11 * t27 * mrSges(12,1);
t1063 = t11 * t25 * mrSges(12,2);
t1066 = m(6) * t11;
t1069 = t11 * t115 * mrSges(6,1);
t1071 = t11 * t111 * mrSges(6,2);
t1075 = t291 * mrSges(6,1);
t1076 = -t287 * mrSges(6,2);
t1079 = -g(3) * (t47 * t967 - t970 - t972) - g(1) * (m(13) * t977 + t979 + t980) - g(1) * (m(14) * t977 + t984 + t985) - g(2) * (m(15) * (t922 + t923) + t990 + t991) - g(3) * (t98 * t994 + t996 - t997) - g(1) * (m(15) * (-t846 + t847) + t1002 + t1003) - g(2) * (m(13) * t831 + t1007 + t1008) - g(3) * (m(5) * pkin(18) * t11 * t17 + t1015 + t1017) - g(1) * (m(5) * (pkin(18) * t436 + t3 * t46) + t1024 + t1025) - g(2) * (m(5) * (pkin(18) * t425 - t46 * t65) + t1032 + t1033) - g(3) * (mrSges(4,1) * t11 * t17 + mrSges(4,2) * t11 * t7) - g(1) * (mrSges(4,1) * t437 - mrSges(4,2) * t433) - g(2) * (-mrSges(4,1) * t426 + mrSges(4,2) * t422) - g(1) * (m(12) * (t14 * t9 + t19 * t3) + t1054 + t1055) - g(3) * (t1058 * t19 - t1061 - t1063) - g(3) * (t1066 * t47 + t1069 + t1071) - g(1) * (m(6) * t977 + t1075 + t1076);
t1081 = -t45 + t97;
t1082 = t3 * t1081;
t1083 = t40 - t93;
t1084 = t14 * t1083;
t1092 = pkin(6) * t1 * t39 - t45 * t65;
t1108 = pkin(6) * t14 * t39 + t3 * t45;
t1115 = t65 * t1081;
t1116 = t1 * t1083;
t1137 = -g(1) * (m(16) * (-t845 - t1082 + t1084) + t850 + t851) - g(2) * (m(13) * t1092 + t1007 + t1008) - g(2) * (m(14) * t1092 + t833 + t834) - g(3) * (t45 * t958 - t961 + t963) - g(3) * (t45 * t967 - t970 - t972) - g(1) * (m(13) * t1108 + t979 + t980) - g(1) * (m(14) * t1108 + t984 + t985) - g(2) * (m(15) * (t1115 + t1116) + t990 + t991) - g(3) * (-t1081 * t994 + t996 - t997) - g(1) * (m(15) * (-t1082 + t1084) + t1002 + t1003) - g(2) * (m(16) * (-t921 + t1115 + t1116) + t926 + t927) - g(3) * (m(16) * (-t1081 * t11 + t931) + t936 + t937);
t1138 = -t116 - t45;
t1139 = t65 * t1138;
t1140 = t112 + t40;
t1141 = t1 * t1140;
t1155 = t3 * t1138;
t1156 = t14 * t1140;
t1183 = -g(2) * (m(7) * (t1139 + t1141) + t944 + t945) - g(2) * (m(8) * (-t948 + t1139 + t1141) + t952 - t954 + t955) - g(3) * (m(8) * (-t11 * t1138 + t875) + t880 - t882 + t883) - g(1) * (m(8) * (-t886 - t1155 + t1156) + t890 - t892 + t893) - g(3) * (t1066 * t45 + t1069 + t1071) - g(1) * (m(6) * t1108 + t1075 + t1076) - g(2) * (m(6) * t1092 + t855 + t856) - g(3) * (-t1138 * t859 + t862 - t864) - g(1) * (m(7) * (-t1155 + t1156) + t871 + t872) - g(3) * (t1015 + t1017) - g(1) * (t1024 + t1025) - g(2) * (t1032 + t1033);
t1185 = t65 * t116;
t1187 = t1 * pkin(7) * t111;
t1202 = t3 * t116;
t1204 = t14 * pkin(7) * t111;
t1312 = t3 * t97;
t1314 = t14 * pkin(9) * t92;
t1331 = t65 * t97;
t1333 = t1 * pkin(9) * t92;
t1355 = -g(1) * (m(16) * (-t845 - t1312 - t1314) + t850 + t851) - g(2) * (t1007 + t1008) - g(2) * (t833 + t834) - g(3) * (-t961 + t963) - g(3) * (-t970 - t972) - g(1) * (t979 + t980) - g(1) * (t984 + t985) - g(2) * (m(15) * (t1331 - t1333) + t990 + t991) - g(3) * (-t97 * t994 + t996 - t997) - g(1) * (m(15) * (-t1312 - t1314) + t1002 + t1003) - g(2) * (m(16) * (-t921 + t1331 - t1333) + t926 + t927) - g(3) * (m(16) * (-pkin(9) * t11 * t96 + t931) + t936 + t937);
unknown(1) = t264 + t442;
unknown(2) = -g(2) * (m(8) * (t444 * t67 + t446 + t447) + (t140 * t65 - t450 * t67) * mrSges(8,1) + (t138 * t65 + t455 * t67) * mrSges(8,2) + t67 * t460) - g(3) * (m(8) * (pkin(10) * t464 + t466 - t467) + (-t11 * t140 - t138 * t470) * mrSges(8,1) + (-t11 * t138 + t140 * t470) * mrSges(8,2) + t464 * mrSges(8,3)) - g(1) * (m(8) * (-t12 * t444 - t483 - t484) + (t12 * t450 - t140 * t3) * mrSges(8,1) + (-t12 * t455 - t138 * t3) * mrSges(8,2) - t12 * t460) - g(3) * (-mrSges(14,1) * t2 * t92 + mrSges(14,2) * t2 * t96 + m(14) * t499 - mrSges(14,3) * t11) - g(1) * (m(13) * t509 - mrSges(13,3) * t3 - t12 * t511 - t12 * t513) - g(1) * (m(14) * t509 - mrSges(14,3) * t3 + t12 * t519 - t12 * t521) - g(2) * (m(15) * (t526 + t447) - t67 * t529 - t67 * t531 + t65 * mrSges(15,3)) - g(3) * (m(15) * (t536 - t467) - t539 * mrSges(15,1) - t541 * mrSges(15,2) - t11 * mrSges(15,3)) - g(1) * (m(15) * (-t546 - t484) + t12 * t529 + t12 * t531 - t3 * mrSges(15,3)) - g(2) * (m(16) * (t554 * t67 + t447 + t526) - t67 * t558 + t65 * mrSges(16,2) + t67 * t561) + t649 - g(2) * (mrSges(3,1) * t67 - mrSges(3,2) * t65) - g(2) * (mrSges(9,3) * t65 + t616 * t640 - t642 * t67 - t644 * t67) - g(3) * (mrSges(6,1) * t111 * t2 - mrSges(6,2) * t115 * t2 + m(6) * t499 - mrSges(6,3) * t11) - g(1) * (m(6) * t509 - mrSges(6,3) * t3 - t12 * t670 + t12 * t672) - g(2) * (m(6) * t585 + mrSges(6,3) * t65 + t67 * t670 - t67 * t672) - g(3) * (m(7) * (t466 - t467) - t470 * mrSges(7,1) - t464 * mrSges(7,2) - t11 * mrSges(7,3)) - g(1) * (m(7) * (-t483 - t484) + t12 * t692 + t12 * t694 - t3 * mrSges(7,3)) - g(3) * (m(5) * (pkin(18) * t699 - t467) + t2 * t39 * mrSges(5,1) - t2 * t44 * mrSges(5,2) - t11 * mrSges(5,3)) - g(1) * (m(5) * (-t12 * t41 - t484) - t12 * t713 + t12 * t715 - t3 * mrSges(5,3)) - g(2) * (m(5) * (t41 * t67 + t447) + t67 * t713 - t67 * t715 + t65 * mrSges(5,3)) + t826;
unknown(3) = t966 + t1079;
unknown(4) = t1137 + t1183;
unknown(5) = -g(2) * (m(7) * (-t1185 + t1187) + t944 + t945) - g(2) * (m(8) * (-t948 - t1185 + t1187) + t952 - t954 + t955) - g(3) * (m(8) * (pkin(7) * t11 * t115 + t875) + t880 - t882 + t883) - g(1) * (m(8) * (-t886 + t1202 + t1204) + t890 - t892 + t893) - g(3) * (t1069 + t1071) - g(1) * (t1075 + t1076) - g(2) * (t855 + t856) - g(3) * (t116 * t859 + t862 - t864) - g(1) * (m(7) * (t1202 + t1204) + t871 + t872);
unknown(6) = -g(2) * (t944 + t945) - g(2) * (-m(8) * pkin(10) * t161 + t952 - t954 + t955) - g(3) * (m(8) * pkin(10) * t11 * t122 + t880 - t882 + t883) - g(1) * (m(8) * pkin(10) * t126 + t890 - t892 + t893) - g(3) * (t862 - t864) - g(1) * (t871 + t872);
unknown(7) = -g(2) * ((t140 * t161 - t167) * mrSges(8,1) + (t138 * t161 + t163) * mrSges(8,2)) - g(3) * ((-t138 * t2 - t140 * t863) * mrSges(8,1) + (-t138 * t863 + t140 * t2) * mrSges(8,2)) - g(1) * (mrSges(8,1) * t146 - mrSges(8,2) * t142);
unknown(8) = -g(2) * (-mrSges(10,1) * t365 + mrSges(10,2) * t361) - g(3) * (mrSges(10,1) * t11 * t344 - mrSges(10,2) * t11 * t342) - g(1) * (mrSges(10,1) * t350 - mrSges(10,2) * t346);
unknown(9) = -g(2) * (m(12) * (pkin(14) * t1 * t5 - t16 * t65) + t841 + t842) - g(1) * (t900 + t901) - g(3) * (t908 + t910) - g(2) * (t917 + t918) - g(1) * (m(12) * (pkin(14) * t14 * t5 + t16 * t3) + t1054 + t1055) - g(3) * (t1058 * t16 - t1061 - t1063);
unknown(10) = -g(2) * (t841 + t842) - g(1) * (t1054 + t1055) - g(3) * (-t1061 - t1063);
unknown(11) = t1355;
unknown(12) = -g(1) * (m(16) * qJ(13) * t104 + t850 + t851) - g(2) * (t990 + t991) - g(3) * (t996 - t997) - g(1) * (t1002 + t1003) - g(2) * (-m(16) * qJ(13) * t225 + t926 + t927) - g(3) * (m(16) * qJ(13) * t11 * t87 + t936 + t937);
unknown(13) = m(16) * g(1) * t89 - m(16) * g(2) * t229 + m(16) * g(3) * t935;
taug = unknown(:);
