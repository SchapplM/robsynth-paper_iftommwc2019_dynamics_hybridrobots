% Return the minimum parameter vector for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% Ifges [16x6]
%   inertia of all robot links about their respective body frame origins, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertial_parameters_convert_par1_par2.m)
% 
% Output:
% MPV [88x1]
%   base parameter vector (minimal parameter vector)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function MPV = KAS5m7OL_convert_par2_MPV_fixb(pkin, m, mrSges, Ifges)

%% Coder Information
%#codegen
%$cgargs {zeros(19,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_convert_par2_MPV_fixb: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_convert_par2_MPV_fixb: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_convert_par2_MPV_fixb: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7OL_convert_par2_MPV_fixb: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From minimal_parameter_vector_fixb_matlab.m
unknown=NaN(88,1);
t2 = -m(10) * pkin(17) + mrSges(9,1);
t4 = cos(pkin(3));
t7 = sin(pkin(3));
t8 = t7 * mrSges(9,2);
t11 = m(3) + m(11) + m(12) + m(4) + m(5) + m(6) + m(7) + m(9) + m(10);
t12 = pkin(11) ^ 2;
t16 = pkin(17) ^ 2;
t17 = t16 * m(10);
t22 = Ifges(11,2) + Ifges(12,2) + Ifges(13,2) + Ifges(15,2) + Ifges(16,3) + Ifges(9,1) + Ifges(14,2) + Ifges(3,1) + Ifges(4,2) + Ifges(5,2) + Ifges(6,2) + Ifges(7,2) + Ifges(10,2);
t23 = cos(pkin(1));
t24 = sin(pkin(1));
t26 = Ifges(14,4) * t24 * t23;
t27 = 0.2e1 * t26;
t30 = 0.2e1 * Ifges(9,4) * t7 * t4;
t31 = pkin(6) ^ 2;
t32 = pkin(18) ^ 2;
t34 = m(6) * (t31 + t32);
t35 = pkin(7) ^ 2;
t37 = m(7) * (t31 + t35 + t32);
t38 = t17 - Ifges(9,1) + Ifges(9,2);
t39 = t4 ^ 2;
t41 = -Ifges(14,2) + Ifges(14,1);
t42 = t23 ^ 2;
t43 = t42 * t41;
t44 = pkin(14) ^ 2;
t45 = pkin(19) ^ 2;
t47 = m(12) * (t44 + t45);
t48 = (m(4) + m(9) + m(5) + m(6) + m(7) + m(10) + m(11) + m(12));
t49 = pkin(16) ^ 2;
t50 = t49 * t48;
t51 = (mrSges(4,3) + mrSges(5,3) + mrSges(6,3) + mrSges(7,3) + mrSges(9,3) + mrSges(10,3) + mrSges(11,3) + mrSges(12,3) + mrSges(13,3) + mrSges(14,3) + mrSges(15,3));
t52 = 2 * pkin(16) * t51;
t53 = m(5) * t32;
t54 = m(11) * t45;
t55 = t39 * t38 - Ifges(3,2) - Ifges(9,3) - t17 - t27 + t30 + t34 + t37 + t43 + t47 + t50 + t52 + t53 + t54;
t58 = mrSges(10,3) * pkin(17);
t62 = -mrSges(9,2) * pkin(16) + Ifges(9,6);
t75 = Ifges(11,2) + Ifges(12,2) + Ifges(13,2) + Ifges(15,2) + Ifges(16,3) + Ifges(3,3) + Ifges(9,2) + Ifges(14,2) + Ifges(4,2) + Ifges(5,2) + Ifges(6,2) + Ifges(7,2);
t77 = -t39 * t38 + Ifges(10,2) + t17 - t27 - t30 + t34 + t37 + t43 + t47 + t50 + t52 + t53 + t54;
t83 = -pkin(16) * t48 + mrSges(3,2) - mrSges(11,3) - mrSges(12,3) - mrSges(13,3) - mrSges(14,3) - mrSges(15,3) - mrSges(4,3) - mrSges(5,3) - mrSges(6,3) - mrSges(7,3) - mrSges(9,3) - mrSges(10,3);
t84 = -m(5) - m(6) - m(7);
t86 = -m(11) - m(12);
t100 = -m(6) - m(7);
t110 = m(7) * t35;
t121 = m(12) * t44;
t138 = -mrSges(15,3) * pkin(9) + Ifges(14,5);
unknown(1,1) = 0.2e1 * t4 * t2 * pkin(11) + 0.2e1 * pkin(11) * mrSges(3,3) - 0.2e1 * pkin(11) * t8 + t12 * t11 + Ifges(3,2) + Ifges(2,3) + Ifges(9,3) + t17;
unknown(2,1) = mrSges(2,1);
unknown(3,1) = pkin(11) * t11 + t4 * t2 + mrSges(2,2) + mrSges(3,3) - t8;
unknown(4,1) = t55 + t22;
unknown(5,1) = t7 * (-pkin(16) * t2 + Ifges(9,5) + t58) + t4 * t62 + Ifges(3,4);
unknown(6,1) = t4 * t38 * t7 - 0.2e1 * Ifges(9,4) * t39 + Ifges(9,4) + Ifges(3,5);
unknown(7,1) = t4 * (pkin(16) * t2 - Ifges(9,5) - t58) + t7 * t62 + Ifges(3,6);
unknown(8,1) = t77 + t75;
unknown(9,1) = -t4 * mrSges(9,2) - t7 * t2 + mrSges(3,1);
unknown(10,1) = t83;
unknown(11,1) = t32 * t84 + t45 * t86 + Ifges(4,1) - Ifges(4,2);
unknown(12,1) = Ifges(4,4);
unknown(13,1) = ((-mrSges(14,3) - mrSges(15,3) - mrSges(5,3) - mrSges(6,3) - mrSges(7,3) - mrSges(13,3)) * pkin(18) + (-mrSges(11,3) - mrSges(12,3)) * pkin(19) + Ifges(4,5));
unknown(14,1) = Ifges(4,6);
unknown(15,1) = -t32 * t84 - t45 * t86 + Ifges(4,3);
unknown(16,1) = -pkin(18) * t84 - pkin(19) * t86 + mrSges(4,1);
unknown(17,1) = mrSges(4,2);
unknown(18,1) = t31 * t100 + Ifges(5,1) - Ifges(5,2);
unknown(19,1) = Ifges(5,4);
unknown(20,1) = ((-mrSges(6,3) - mrSges(7,3) - mrSges(13,3) - mrSges(14,3) - mrSges(15,3)) * pkin(6) + Ifges(5,5));
unknown(21,1) = Ifges(5,6);
unknown(22,1) = -t31 * t100 + Ifges(5,3);
unknown(23,1) = -pkin(6) * t100 + mrSges(5,1);
unknown(24,1) = mrSges(5,2);
unknown(25,1) = -t110 + Ifges(6,1) - Ifges(6,2);
unknown(26,1) = Ifges(6,4);
unknown(27,1) = (-pkin(7) * mrSges(7,3) + Ifges(6,5));
unknown(28,1) = Ifges(6,6);
unknown(29,1) = t110 + Ifges(6,3);
unknown(30,1) = (pkin(7) * m(7) + mrSges(6,1));
unknown(31,1) = mrSges(6,2);
unknown(32,1) = Ifges(7,1) - Ifges(7,2) + Ifges(8,2);
unknown(33,1) = Ifges(7,4);
unknown(34,1) = Ifges(7,5);
unknown(35,1) = Ifges(7,6);
unknown(36,1) = Ifges(7,3) + Ifges(8,2);
unknown(37,1) = mrSges(7,1);
unknown(38,1) = mrSges(7,2);
unknown(39,1) = Ifges(8,1) - Ifges(8,2);
unknown(40,1) = Ifges(8,4);
unknown(41,1) = Ifges(8,5);
unknown(42,1) = Ifges(8,6);
unknown(43,1) = Ifges(8,3);
unknown(44,1) = mrSges(8,1);
unknown(45,1) = mrSges(8,2);
unknown(46,1) = mrSges(8,3);
unknown(47,1) = m(8);
unknown(48,1) = Ifges(10,1) - Ifges(10,2);
unknown(49,1) = Ifges(10,4);
unknown(50,1) = Ifges(10,5);
unknown(51,1) = Ifges(10,6);
unknown(52,1) = Ifges(10,3);
unknown(53,1) = mrSges(10,1);
unknown(54,1) = mrSges(10,2);
unknown(55,1) = -t121 + Ifges(11,1) - Ifges(11,2);
unknown(56,1) = Ifges(11,4);
unknown(57,1) = (-pkin(14) * mrSges(12,3) + Ifges(11,5));
unknown(58,1) = Ifges(11,6);
unknown(59,1) = t121 + Ifges(11,3);
unknown(60,1) = (pkin(14) * m(12) + mrSges(11,1));
unknown(61,1) = mrSges(11,2);
unknown(62,1) = Ifges(12,1) - Ifges(12,2);
unknown(63,1) = Ifges(12,4);
unknown(64,1) = Ifges(12,5);
unknown(65,1) = Ifges(12,6);
unknown(66,1) = Ifges(12,3);
unknown(67,1) = mrSges(12,1);
unknown(68,1) = mrSges(12,2);
unknown(69,1) = -0.2e1 * t42 * t41 + Ifges(13,1) + Ifges(14,1) - Ifges(13,2) - Ifges(14,2) + 0.4e1 * t26;
unknown(70,1) = -t23 * t41 * t24 - 0.2e1 * Ifges(14,4) * t42 + Ifges(13,4) + Ifges(14,4);
unknown(71,1) = Ifges(14,6) * t23 + t24 * t138 + Ifges(13,5);
unknown(72,1) = Ifges(14,6) * t24 - t23 * t138 + Ifges(13,6);
unknown(73,1) = Ifges(13,3) + Ifges(14,3);
unknown(74,1) = t24 * mrSges(14,1) + t23 * mrSges(14,2) + mrSges(13,1);
unknown(75,1) = -t23 * mrSges(14,1) + t24 * mrSges(14,2) + mrSges(13,2);
unknown(76,1) = m(13) + m(14);
unknown(77,1) = Ifges(15,1) - Ifges(15,2) + Ifges(16,1) - Ifges(16,3);
unknown(78,1) = Ifges(15,4) - Ifges(16,5);
unknown(79,1) = Ifges(15,5) + Ifges(16,4);
unknown(80,1) = Ifges(15,6) - Ifges(16,6);
unknown(81,1) = Ifges(15,3) + Ifges(16,2);
unknown(82,1) = mrSges(15,1);
unknown(83,1) = mrSges(15,2);
unknown(84,1) = m(15);
unknown(85,1) = mrSges(16,1);
unknown(86,1) = mrSges(16,2);
unknown(87,1) = mrSges(16,3);
unknown(88,1) = m(16);
MPV = unknown;
