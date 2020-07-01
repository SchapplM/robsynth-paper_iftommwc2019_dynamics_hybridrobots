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

function U = KAS5m7OL_energypot_fixb_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energypot_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_energypot_fixb_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:20
% DurationCPUTime: 0.14s
% Computational Cost: add. (799->337), mult. (758->407), div. (0->0), fcn. (731->28), ass. (0->99)
t1 = cos(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = qJ(3) + qJ(4);
t5 = cos(t4);
t6 = pkin(6) * t5;
t7 = cos(qJ(3));
t8 = t7 * pkin(18);
t9 = t6 + t8;
t11 = cos(qJ(2));
t12 = t1 * t11;
t13 = t12 * pkin(16);
t14 = sin(qJ(1));
t15 = sin(t4);
t16 = pkin(6) * t15;
t17 = sin(qJ(3));
t18 = t17 * pkin(18);
t19 = t16 + t18;
t21 = t14 * pkin(11);
t22 = t14 * t19 + t3 * t9 - t13 + t21;
t24 = qJ(3) + qJ(4) + qJ(5);
t25 = cos(t24);
t27 = sin(t24);
t38 = -t13 + t21;
t41 = t14 * t17;
t67 = sin(pkin(3));
t69 = cos(pkin(3));
t70 = t14 * t69;
t92 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t93 = cos(t92);
t94 = t11 * t93;
t96 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t97 = cos(t96);
t99 = -pkin(9) * t97 + t6 + t8;
t100 = t11 * t99;
t101 = t2 * pkin(16);
t104 = sin(t92);
t105 = t11 * t104;
t111 = qJ(3) + qJ(9);
t112 = cos(t111);
t114 = t7 * pkin(19);
t115 = pkin(14) * t112 + t114;
t119 = qJ(3) + qJ(9) + qJ(10);
t120 = cos(t119);
t123 = sin(t119);
t130 = -t11 * t9 + pkin(5) - t101;
t134 = sin(t96);
t140 = t14 * t11;
t141 = t140 * pkin(16);
t142 = t1 * pkin(11);
t143 = t141 + t142;
t145 = t14 * t2;
t147 = t1 * t17;
t164 = -g(2) * (m(6) * t22 + (t14 * t27 + t3 * t25) * mrSges(6,1) + (t14 * t25 - t3 * t27) * mrSges(6,2) - t12 * mrSges(6,3)) - g(2) * (m(4) * t38 + (t3 * t7 + t41) * mrSges(4,1) + (t14 * t7 - t3 * t17) * mrSges(4,2) - t12 * mrSges(4,3)) - g(2) * (m(5) * (t41 * pkin(18) + t3 * t8 - t13 + t21) + (t14 * t15 + t3 * t5) * mrSges(5,1) + (t14 * t5 - t3 * t15) * mrSges(5,2) - t12 * mrSges(5,3)) - g(2) * (m(9) * t38 + (-t3 * t67 + t70) * mrSges(9,1) + (-t14 * t67 - t3 * t69) * mrSges(9,2) - t12 * mrSges(9,3)) - g(2) * (m(3) * t14 * pkin(11) + t3 * mrSges(3,1) + t12 * mrSges(3,2) + t14 * mrSges(3,3)) - g(2) * (-t1 * mrSges(2,1) + t14 * mrSges(2,2)) - g(2) * mrSges(1,2) - g(3) * (m(16) * (-t94 * qJ(13) + pkin(5) - t100 - t101) + t105 * mrSges(16,1) - t2 * mrSges(16,2) - t94 * mrSges(16,3)) - g(3) * (m(12) * (-t11 * t115 + pkin(5) - t101) + t11 * t120 * mrSges(12,1) - t11 * t123 * mrSges(12,2) - t2 * mrSges(12,3)) - g(3) * (t11 * t97 * mrSges(14,1) - t11 * t134 * mrSges(14,2) + m(14) * t130 - t2 * mrSges(14,3)) - g(1) * (m(4) * t143 + (-t145 * t7 + t147) * mrSges(4,1) + (t1 * t7 + t145 * t17) * mrSges(4,2) + t140 * mrSges(4,3)) - g(1) * (m(3) * t1 * pkin(11) - t145 * mrSges(3,1) - t140 * mrSges(3,2) + t1 * mrSges(3,3));
t167 = t1 * t69;
t184 = -t14 * t104 - t3 * t93;
t186 = t3 * t99;
t188 = -pkin(9) * t134 + t16 + t18;
t189 = t14 * t188;
t194 = -t3 * t104 + t14 * t93;
t201 = sin(t111);
t204 = pkin(14) * t201 + t17 * pkin(19);
t232 = qJ(3) + qJ(4) + qJ(11);
t233 = sin(t232);
t235 = cos(t232);
t254 = pkin(7) * t25 + t6 + t8;
t255 = t3 * t254;
t257 = pkin(7) * t27 + t16 + t18;
t258 = t14 * t257;
t261 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t262 = sin(t261);
t264 = cos(t261);
t266 = t14 * t264 - t3 * t262;
t270 = -t14 * t262 - t3 * t264;
t278 = cos(qJ(7));
t280 = sin(qJ(7));
t291 = t67 * pkin(17);
t296 = pkin(3) + qJ(8);
t297 = sin(t296);
t299 = cos(t296);
t325 = -g(1) * (m(9) * t143 + (t145 * t67 + t167) * mrSges(9,1) + (-t1 * t67 + t145 * t69) * mrSges(9,2) + t140 * mrSges(9,3)) - g(1) * (t14 * mrSges(2,1) + t1 * mrSges(2,2)) - g(1) * mrSges(1,1) - g(2) * (m(16) * (-t184 * qJ(13) - t13 + t186 + t189 + t21) + t194 * mrSges(16,1) - t12 * mrSges(16,2) - t184 * mrSges(16,3)) - g(2) * (m(12) * (t3 * t115 + t14 * t204 - t13 + t21) + (-t3 * t120 - t14 * t123) * mrSges(12,1) + (-t14 * t120 + t3 * t123) * mrSges(12,2) - t12 * mrSges(12,3)) - g(2) * (m(14) * t22 + (-t14 * t134 - t3 * t97) * mrSges(14,1) + (t3 * t134 - t14 * t97) * mrSges(14,2) - t12 * mrSges(14,3)) - g(2) * (m(13) * t22 + (-t14 * t235 + t3 * t233) * mrSges(13,1) + (t14 * t233 + t3 * t235) * mrSges(13,2) - t12 * mrSges(13,3)) - g(2) * (m(15) * (t186 - t13 + t189 + t21) + t194 * mrSges(15,1) + t184 * mrSges(15,2) - t12 * mrSges(15,3)) - g(2) * (m(7) * (t255 - t13 + t258 + t21) + t266 * mrSges(7,1) + t270 * mrSges(7,2) - t12 * mrSges(7,3)) - g(2) * (m(8) * (-t270 * pkin(10) - t13 + t21 + t255 + t258) + (-t12 * t280 + t266 * t278) * mrSges(8,1) + (-t12 * t278 - t266 * t280) * mrSges(8,2) - t270 * mrSges(8,3)) - g(2) * (m(10) * (-t70 * pkin(17) + t3 * t291 - t13 + t21) + (t14 * t299 - t3 * t297) * mrSges(10,1) + (-t14 * t297 - t3 * t299) * mrSges(10,2) - t12 * mrSges(10,3)) - g(2) * (m(11) * (t41 * pkin(19) + t3 * t114 - t13 + t21) + (t3 * t112 + t14 * t201) * mrSges(11,1) + (t14 * t112 - t3 * t201) * mrSges(11,2) - t12 * mrSges(11,3));
t327 = t145 * t99;
t328 = t1 * t188;
t333 = t1 * t93 + t145 * t104;
t337 = -t1 * t104 + t145 * t93;
t382 = t1 * t19 - t145 * t9 + t141 + t142;
t409 = -t1 * t262 + t145 * t264;
t411 = t145 * t254;
t412 = t1 * t257;
t417 = t1 * t264 + t145 * t262;
t486 = -g(1) * (m(15) * (-t327 + t141 + t328 + t142) + t333 * mrSges(15,1) + t337 * mrSges(15,2) + t140 * mrSges(15,3)) - g(1) * (m(16) * (-t337 * qJ(13) + t141 + t142 - t327 + t328) + t333 * mrSges(16,1) + t140 * mrSges(16,2) - t337 * mrSges(16,3)) - g(1) * (m(11) * (t147 * pkin(19) - t145 * t114 + t141 + t142) + (t1 * t201 - t145 * t112) * mrSges(11,1) + (t1 * t112 + t145 * t201) * mrSges(11,2) + t140 * mrSges(11,3)) - g(1) * (m(12) * (t1 * t204 - t145 * t115 + t141 + t142) + (-t1 * t123 + t145 * t120) * mrSges(12,1) + (-t1 * t120 - t145 * t123) * mrSges(12,2) + t140 * mrSges(12,3)) - g(1) * (m(14) * t382 + (-t1 * t134 + t145 * t97) * mrSges(14,1) + (-t1 * t97 - t145 * t134) * mrSges(14,2) + t140 * mrSges(14,3)) - g(1) * (m(13) * t382 + (-t1 * t235 - t145 * t233) * mrSges(13,1) + (t1 * t233 - t145 * t235) * mrSges(13,2) + t140 * mrSges(13,3)) - g(1) * (m(8) * (-t409 * pkin(10) + t141 + t142 - t411 + t412) + (t140 * t280 + t417 * t278) * mrSges(8,1) + (t140 * t278 - t417 * t280) * mrSges(8,2) - t409 * mrSges(8,3)) - g(1) * (m(10) * (-t167 * pkin(17) - t145 * t291 + t141 + t142) + (t1 * t299 + t145 * t297) * mrSges(10,1) + (-t1 * t297 + t145 * t299) * mrSges(10,2) + t140 * mrSges(10,3)) - g(1) * (m(5) * (t147 * pkin(18) - t145 * t8 + t141 + t142) + (t1 * t15 - t145 * t5) * mrSges(5,1) + (t1 * t5 + t145 * t15) * mrSges(5,2) + t140 * mrSges(5,3)) - g(1) * (m(6) * t382 + (t1 * t27 - t145 * t25) * mrSges(6,1) + (t1 * t25 + t145 * t27) * mrSges(6,2) + t140 * mrSges(6,3)) - g(1) * (m(7) * (-t411 + t141 + t412 + t142) + t417 * mrSges(7,1) + t409 * mrSges(7,2) + t140 * mrSges(7,3)) - g(3) * (-t11 * t233 * mrSges(13,1) - t11 * t235 * mrSges(13,2) + m(13) * t130 - t2 * mrSges(13,3));
t494 = t11 * t254;
t497 = t11 * t262;
t499 = t11 * t264;
t518 = t11 * t67;
t529 = t11 * t7;
t558 = -t101 + pkin(5);
t582 = -g(3) * (m(15) * (-t100 - t101 + pkin(5)) + t105 * mrSges(15,1) + t94 * mrSges(15,2) - t2 * mrSges(15,3)) - g(3) * (m(7) * (-t494 - t101 + pkin(5)) + t497 * mrSges(7,1) + t499 * mrSges(7,2) - t2 * mrSges(7,3)) - g(3) * (m(8) * (-t499 * pkin(10) + pkin(5) - t101 - t494) + (-t2 * t280 + t497 * t278) * mrSges(8,1) + (-t2 * t278 - t497 * t280) * mrSges(8,2) - t499 * mrSges(8,3)) - g(3) * (m(10) * (-t518 * pkin(17) + pkin(5) - t101) + t11 * t297 * mrSges(10,1) + t11 * t299 * mrSges(10,2) - t2 * mrSges(10,3)) - g(3) * (m(11) * (-t529 * pkin(19) + pkin(5) - t101) - t11 * t112 * mrSges(11,1) + t11 * t201 * mrSges(11,2) - t2 * mrSges(11,3)) - g(3) * (-t11 * t25 * mrSges(6,1) + t11 * t27 * mrSges(6,2) + m(6) * t130 - t2 * mrSges(6,3)) - g(3) * (m(5) * (-t529 * pkin(18) + pkin(5) - t101) - t11 * t5 * mrSges(5,1) + t11 * t15 * mrSges(5,2) - t2 * mrSges(5,3)) - g(3) * (t11 * t69 * mrSges(9,2) + m(9) * t558 + t518 * mrSges(9,1) - t2 * mrSges(9,3)) - g(3) * (m(3) * pkin(5) - t11 * mrSges(3,1) + t2 * mrSges(3,2)) - g(3) * (t11 * t17 * mrSges(4,2) + m(4) * t558 - t529 * mrSges(4,1) - t2 * mrSges(4,3)) - g(3) * (m(2) * pkin(5) + mrSges(2,3)) - g(3) * mrSges(1,3);
t584 = t164 + t325 + t486 + t582;
U = t584;
