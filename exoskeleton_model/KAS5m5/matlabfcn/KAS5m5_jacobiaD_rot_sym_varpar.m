% Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix für beliebiges Segment von
% KAS5m5
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% Zeitableitung der Winkeldarstellung des Endeffektors in Basis-Koordinaten
% 
% Winkeldarstellung: Euler-XYZ-Winkel, rotx(alpha)*roty(beta)*rotz(gamma)
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in KAS5m5_fkine_fixb_rotmat_mdh_sym_varpar.m (1=Basis).
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% JaD_rot [3x5]
%   Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JaD_rot = KAS5m5_jacobiaD_rot_sym_varpar(qJ, qJD, link_index, ...
  pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),uint8(0),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_jacobiaD_rot_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m5_jacobiaD_rot_sym_varpar: qJD has to be [5x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5_jacobiaD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_jacobiaD_rot_sym_varpar: pkin has to be [30x1] (double)');
JaD_rot=NaN(3,5);
if link_index == 0
	% Symbolic code from jacobiaD_rot_0_floatb_twist_matlab.m not found
elseif link_index == 1
	% Symbolic code from jacobiaD_rot_1_floatb_twist_matlab.m not found
elseif link_index == 2
	% Symbolic code from jacobiaD_rot_2_floatb_twist_matlab.m not found
elseif link_index == 3
	% Symbolic code from jacobiaD_rot_3_floatb_twist_matlab.m not found
elseif link_index == 4
	% Symbolic code from jacobiaD_rot_4_floatb_twist_matlab.m not found
elseif link_index == 5
	% Symbolic code from jacobiaD_rot_5_floatb_twist_matlab.m not found
elseif link_index == 6
	% Symbolic code from jacobiaD_rot_6_floatb_twist_matlab.m not found
elseif link_index == 7
	% Symbolic code from jacobiaD_rot_7_floatb_twist_matlab.m not found
elseif link_index == 8
	% Symbolic code from jacobiaD_rot_8_floatb_twist_matlab.m not found
elseif link_index == 9
	% Symbolic code from jacobiaD_rot_9_floatb_twist_matlab.m not found
elseif link_index == 10
	% Symbolic code from jacobiaD_rot_10_floatb_twist_matlab.m not found
elseif link_index == 11
	% Symbolic code from jacobiaD_rot_11_floatb_twist_matlab.m not found
elseif link_index == 12
	% Symbolic code from jacobiaD_rot_12_floatb_twist_matlab.m not found
elseif link_index == 13
	% Symbolic code from jacobiaD_rot_13_floatb_twist_matlab.m not found
end