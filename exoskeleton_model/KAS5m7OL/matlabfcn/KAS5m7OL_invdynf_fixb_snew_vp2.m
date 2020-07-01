% Calculate vector of cutting forces with Newton-Euler
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
%
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% qJDD [13x1]
%   Generalized joint accelerations
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
% Ifges [16x6]
%   inertia of all robot links about their respective body frame origins, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertial_parameters_convert_par1_par2.m)
%
% Output:
% f_new [3x16]
%   vector of cutting forces (contains inertial, gravitational coriolis and centrifugal forces)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-24 19:41
% Revision: 6227a0e3b1a063aaae08a4fbbd45eec09d1886d7 (2020-06-24)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function f_new = KAS5m7OL_invdynf_fixb_snew_vp2(qJ, qJD, qJDD, g, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(13,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: qJD has to be [13x1] (double)');
assert(isreal(qJDD) && all(size(qJDD) == [13 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: qJDD has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7OL_invdynf_fixb_snew_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From invdyn_fixb_NewtonEuler_linkframe_f_i_i_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-24 19:23:24
% EndTime: 2020-06-24 19:23:24
% DurationCPUTime: 0.05s
% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->66)
unknown=NaN(3,22);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(1,6) = 0;
unknown(1,7) = 0;
unknown(1,8) = 0;
unknown(1,9) = 0;
unknown(1,10) = 0;
unknown(1,11) = 0;
unknown(1,12) = 0;
unknown(1,13) = 0;
unknown(1,14) = 0;
unknown(1,15) = 0;
unknown(1,16) = 0;
unknown(1,17) = 0;
unknown(1,18) = 0;
unknown(1,19) = 0;
unknown(1,20) = 0;
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = 0;
unknown(2,4) = 0;
unknown(2,5) = 0;
unknown(2,6) = 0;
unknown(2,7) = 0;
unknown(2,8) = 0;
unknown(2,9) = 0;
unknown(2,10) = 0;
unknown(2,11) = 0;
unknown(2,12) = 0;
unknown(2,13) = 0;
unknown(2,14) = 0;
unknown(2,15) = 0;
unknown(2,16) = 0;
unknown(2,17) = 0;
unknown(2,18) = 0;
unknown(2,19) = 0;
unknown(2,20) = 0;
unknown(2,21) = 0;
unknown(2,22) = 0;
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = 0;
unknown(3,4) = 0;
unknown(3,5) = 0;
unknown(3,6) = 0;
unknown(3,7) = 0;
unknown(3,8) = 0;
unknown(3,9) = 0;
unknown(3,10) = 0;
unknown(3,11) = 0;
unknown(3,12) = 0;
unknown(3,13) = 0;
unknown(3,14) = 0;
unknown(3,15) = 0;
unknown(3,16) = 0;
unknown(3,17) = 0;
unknown(3,18) = 0;
unknown(3,19) = 0;
unknown(3,20) = 0;
unknown(3,21) = 0;
unknown(3,22) = 0;
f_new = unknown;
