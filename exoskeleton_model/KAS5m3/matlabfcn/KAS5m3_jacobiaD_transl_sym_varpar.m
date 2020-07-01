% Zeitableitung der analytischen Jacobi-Matrix (Translatorisch) für beliebiges Segment von
% KAS5m3
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% (Ist für translatorischen Teil egal, kennzeichnet nur den Rechenweg der Herleitung)
% 
% Input:
% qJ [6x1]
%   Generalized joint coordinates (joint angles)
% qJD [6x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt (0=Basis).
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% JaD_transl [3x6]
%   Translatorischer Teil der analytischen Jacobi-Matrix (Zeitableitung)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JaD_transl = KAS5m3_jacobiaD_transl_sym_varpar(qJ, qJD, link_index, r_i_i_C, ...
  pkin)


%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(6,1),uint8(0),zeros(3,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_jacobiaD_transl_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [6 1]), ...
  'KAS5m3_jacobiaD_transl_sym_varpar: qJD has to be [6x1] (double)');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
	'KAS5m3_jacobiaD_transl_sym_varpar: Position vector r_i_i_C has to be [3x1] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m3_jacobiaD_transl_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_jacobiaD_transl_sym_varpar: pkin has to be [30x1] (double)');
JaD_transl=NaN(3,6);
if link_index == 0
	% Symbolic code from jacobiaD_transl_0_floatb_twist_matlab.m not found
elseif link_index == 1
	% Symbolic code from jacobiaD_transl_1_floatb_twist_matlab.m not found
elseif link_index == 2
	% Symbolic code from jacobiaD_transl_2_floatb_twist_matlab.m not found
elseif link_index == 3
	% Symbolic code from jacobiaD_transl_3_floatb_twist_matlab.m not found
elseif link_index == 4
	% Symbolic code from jacobiaD_transl_4_floatb_twist_matlab.m not found
elseif link_index == 5
	% Symbolic code from jacobiaD_transl_5_floatb_twist_matlab.m not found
elseif link_index == 6
	% Symbolic code from jacobiaD_transl_6_floatb_twist_matlab.m not found
elseif link_index == 7
	% Symbolic code from jacobiaD_transl_7_floatb_twist_matlab.m not found
elseif link_index == 8
	% Symbolic code from jacobiaD_transl_8_floatb_twist_matlab.m not found
elseif link_index == 9
	% Symbolic code from jacobiaD_transl_9_floatb_twist_matlab.m not found
elseif link_index == 10
	% Symbolic code from jacobiaD_transl_10_floatb_twist_matlab.m not found
elseif link_index == 11
	% Symbolic code from jacobiaD_transl_11_floatb_twist_matlab.m not found
elseif link_index == 12
	% Symbolic code from jacobiaD_transl_12_floatb_twist_matlab.m not found
elseif link_index == 13
	% Symbolic code from jacobiaD_transl_13_floatb_twist_matlab.m not found
end