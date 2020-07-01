% Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
% KAS5m7TE
% Use Code from Maple symbolic Code Generation
%
% Geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorgeschwindigkeit und Geschw. der verallgemeinerten Koordinaten.
% 
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% JgD_rot [3x5]
%   Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JgD_rot = KAS5m7TE_jacobigD_rot_sym_varpar(qJ, qJD, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),uint8(0),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_jacobigD_rot_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7TE_jacobigD_rot_sym_varpar: qJD has to be [5x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m7TE_jacobigD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_jacobigD_rot_sym_varpar: pkin has to be [24x1] (double)');
JgD_rot=NaN(3,5);
if link_index == 0
	% Symbolic code from jacobigD_rot_0_floatb_twist_matlab.m not found
elseif link_index == 1
	% Symbolic code from jacobigD_rot_1_floatb_twist_matlab.m not found
elseif link_index == 2
	% Symbolic code from jacobigD_rot_2_floatb_twist_matlab.m not found
elseif link_index == 3
	% Symbolic code from jacobigD_rot_3_floatb_twist_matlab.m not found
elseif link_index == 4
	% Symbolic code from jacobigD_rot_4_floatb_twist_matlab.m not found
elseif link_index == 5
	% Symbolic code from jacobigD_rot_5_floatb_twist_matlab.m not found
elseif link_index == 6
	% Symbolic code from jacobigD_rot_6_floatb_twist_matlab.m not found
elseif link_index == 7
	% Symbolic code from jacobigD_rot_7_floatb_twist_matlab.m not found
elseif link_index == 8
	% Symbolic code from jacobigD_rot_8_floatb_twist_matlab.m not found
elseif link_index == 9
	% Symbolic code from jacobigD_rot_9_floatb_twist_matlab.m not found
elseif link_index == 10
	% Symbolic code from jacobigD_rot_10_floatb_twist_matlab.m not found
elseif link_index == 11
	% Symbolic code from jacobigD_rot_11_floatb_twist_matlab.m not found
elseif link_index == 12
	% Symbolic code from jacobigD_rot_12_floatb_twist_matlab.m not found
elseif link_index == 13
	% Symbolic code from jacobigD_rot_13_floatb_twist_matlab.m not found
elseif link_index == 14
	% Symbolic code from jacobigD_rot_14_floatb_twist_matlab.m not found
elseif link_index == 15
	% Symbolic code from jacobigD_rot_15_floatb_twist_matlab.m not found
end