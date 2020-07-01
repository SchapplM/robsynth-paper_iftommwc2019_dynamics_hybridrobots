% Zeitableitung der Rotationsmatrix-Jacobi-Matrix für beliebiges Segment von
% KAS5m3
% Use Code from Maple symbolic Code Generation
% 
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
% Zeitableitung: Die Gradientenmatrix wird nochmal nach der Zeit abgeleitet.
% 
% Input:
% qJ [6x1]
%   Generalized joint coordinates (joint angles)
% qJD [6x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% JRD_rot [9x6]
%   Zeitableitung der Jacobi-Matrix der Endeffektor-Rotationsmatrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JRD_rot = KAS5m3_jacobiRD_rot_sym_varpar(qJ, qJD, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(6,1),uint8(0),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_jacobiRD_rot_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [6 1]), ...
  'KAS5m3_jacobiRD_rot_sym_varpar: qJD has to be [6x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m3_jacobiRD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_jacobiRD_rot_sym_varpar: pkin has to be [30x1] (double)');
JRD_rot=NaN(9,6);
if link_index == 0
	% Symbolic code from jacobiRD_rot_0_floatb_twist_matlab.m not found
elseif link_index == 1
	% Symbolic code from jacobiRD_rot_1_floatb_twist_matlab.m not found
elseif link_index == 2
	% Symbolic code from jacobiRD_rot_2_floatb_twist_matlab.m not found
elseif link_index == 3
	% Symbolic code from jacobiRD_rot_3_floatb_twist_matlab.m not found
elseif link_index == 4
	% Symbolic code from jacobiRD_rot_4_floatb_twist_matlab.m not found
elseif link_index == 5
	% Symbolic code from jacobiRD_rot_5_floatb_twist_matlab.m not found
elseif link_index == 6
	% Symbolic code from jacobiRD_rot_6_floatb_twist_matlab.m not found
elseif link_index == 7
	% Symbolic code from jacobiRD_rot_7_floatb_twist_matlab.m not found
elseif link_index == 8
	% Symbolic code from jacobiRD_rot_8_floatb_twist_matlab.m not found
elseif link_index == 9
	% Symbolic code from jacobiRD_rot_9_floatb_twist_matlab.m not found
elseif link_index == 10
	% Symbolic code from jacobiRD_rot_10_floatb_twist_matlab.m not found
elseif link_index == 11
	% Symbolic code from jacobiRD_rot_11_floatb_twist_matlab.m not found
elseif link_index == 12
	% Symbolic code from jacobiRD_rot_12_floatb_twist_matlab.m not found
elseif link_index == 13
	% Symbolic code from jacobiRD_rot_13_floatb_twist_matlab.m not found
end