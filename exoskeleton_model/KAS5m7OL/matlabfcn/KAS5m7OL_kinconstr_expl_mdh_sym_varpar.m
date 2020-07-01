% Explicit kinematic constraints of
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% jv [21x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m7OL_kinconstr_expl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_expl_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_kinconstr_expl_mdh_sym_varpar: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-30 17:46:16
% EndTime: 2020-06-30 17:46:16
% DurationCPUTime: 0.01s
% Computational Cost: add. (3->3), mult. (1->1), div. (0->0), fcn. (0->0), ass. (0->1)
t1 = [qJ(1); qJ(2); qJ(3); qJ(4); qJ(5); qJ(6); qJ(7); -pkin(3) - pi + (2 * pkin(16)); qJ(8); qJ(9); qJ(10); qJ(11); -pkin(1) + pi; qJ(12); qJ(13); 0; 0; 0; 0; 0; 0;];
jv = t1(:);
