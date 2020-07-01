% Calculate position vector for OL to TE/DE
% KAS5m7IC
% Use Code from Maple symbolic Code Generation
% 
% Input:
% 
% Output:
% posNQJ[21x1]
%   position vector

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function posNQJ = KAS5m7IC_positionVector_NQJ()
%% Coder Information
%#codegen
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:50
% EndTime: 2020-06-30 18:48:50
% DurationCPUTime: 0.01s
% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->21)
unknown=NaN(21,1);
unknown(1,1) = 1;
unknown(2,1) = 1;
unknown(3,1) = 1;
unknown(4,1) = 1;
unknown(5,1) = 1;
unknown(6,1) = 1;
unknown(7,1) = 1;
unknown(8,1) = 0;
unknown(9,1) = 1;
unknown(10,1) = 1;
unknown(11,1) = 1;
unknown(12,1) = 1;
unknown(13,1) = 0;
unknown(14,1) = 1;
unknown(15,1) = 1;
unknown(16,1) = 0;
unknown(17,1) = 0;
unknown(18,1) = 0;
unknown(19,1) = 0;
unknown(20,1) = 0;
unknown(21,1) = 0;
posNQJ = unknown(:);
