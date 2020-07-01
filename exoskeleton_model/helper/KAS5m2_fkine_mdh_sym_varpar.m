% Direkte Kinematik für KAS5m2 (MDH-Konvention, symbolische Berechnung)
% 
% Input:
% q [7x1]
%   Joint Angles [rad]
% 
% Output:
% T_c_mdh [4*9x4]
%   homogenious transformation matrices for each body frame (MDH)
%    1.. 4: KAS-Basis -> MDH Basis (link 0)
%    5.. 9: KAS-Basis  -> MDH Segment 1 (link 1)
%   10..13: KAS-Basis  -> MDH Segment 2 (link 2)
%   14..17: KAS-Basis  -> MDH Segment 3 (link 3)
%   ...
%   29..32: KAS-Basis  -> MDH Segment 7 (link 7)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_ges = KAS5m2_fkine_mdh_sym_varpar(q, a_mdh, d_mdh)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [7 1]), ...
  'Joint angles q have to be [7x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [7 1]), ...
  'a_mdh has to be [7x1] double'); 
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [7 1]), ...
  'd_mdh has to be [7x1] double'); 

qJs1 = q(1);
qJs2 = q(2);
qJs3 = q(3);
qJs4 = q(4);
qJs5 = q(5);
qJs6 = q(6);
qJs7 = q(7);

a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);

d1 = d_mdh(1);
d2 = d_mdh(2);
d3 = d_mdh(3);
d7 = d_mdh(7);
%% Berechnung mit exportiertem Code
% codegen/KAS5_sym_codegen_floatbase_mdh_kinematik_gesamt.mw
% codeexport/KAS5m2_fkine_matlab.m
t472 = qJs3 - qJs2;
t416 = qJs1 - t472;
t422 = qJs2 + qJs3;
t417 = qJs1 - t422;
t425 = qJs1 - qJs3;
t445 = -cos(t416) / 0.4e1 + cos(t417) / 0.4e1 + sin(t425) / 0.2e1;
t414 = qJs1 + t422;
t415 = qJs1 + t472;
t424 = qJs1 + qJs3;
t461 = cos(t414) / 0.4e1 - cos(t415) / 0.4e1 + sin(t424) / 0.2e1;
t503 = -t445 + t461;
t446 = sin(t414) / 0.4e1 - sin(t415) / 0.4e1 - cos(t424) / 0.2e1;
t462 = sin(t416) / 0.4e1 - sin(t417) / 0.4e1 + cos(t425) / 0.2e1;
t502 = t446 + t462;
t471 = qJs3 + qJs4;
t469 = -qJs2 + t471;
t396 = qJs1 - t469;
t410 = qJs2 + t471;
t397 = qJs1 - t410;
t413 = qJs1 - t471;
t447 = cos(t396) / 0.4e1 - cos(t397) / 0.4e1 - sin(t413) / 0.2e1;
t394 = qJs1 + t410;
t395 = qJs1 + t469;
t411 = qJs1 + t471;
t465 = cos(t394) / 0.4e1 - cos(t395) / 0.4e1 + sin(t411) / 0.2e1;
t501 = t447 + t465;
t448 = sin(t394) / 0.4e1 - sin(t395) / 0.4e1 - cos(t411) / 0.2e1;
t466 = sin(t396) / 0.4e1 - sin(t397) / 0.4e1 + cos(t413) / 0.2e1;
t500 = t448 + t466;
t457 = qJs5 + t469;
t365 = qJs1 - t457;
t390 = qJs5 + t410;
t366 = qJs1 - t390;
t393 = -qJs5 + t413;
t449 = cos(t365) / 0.4e1 - cos(t366) / 0.4e1 - sin(t393) / 0.2e1;
t363 = qJs1 + t390;
t364 = qJs1 + t457;
t391 = qJs5 + t411;
t467 = cos(t363) / 0.4e1 - cos(t364) / 0.4e1 + sin(t391) / 0.2e1;
t499 = t449 + t467;
t450 = -sin(t363) / 0.4e1 + sin(t364) / 0.4e1 + cos(t391) / 0.2e1;
t468 = sin(t365) / 0.4e1 - sin(t366) / 0.4e1 + cos(t393) / 0.2e1;
t498 = -t450 + t468;
t497 = -cos(t472) / 0.2e1 - cos(t422) / 0.2e1;
t474 = qJs1 - qJs2;
t401 = sin(t474);
t473 = qJs1 + qJs2;
t479 = sin(t473) / 0.2e1;
t213 = t479 + t401 / 0.2e1;
t496 = -cos(t469) / 0.2e1 - cos(t410) / 0.2e1;
t495 = -cos(t457) / 0.2e1 - cos(t390) / 0.2e1;
t420 = qJs2 + qJs7;
t470 = qJs7 - qJs2;
t428 = cos(qJs1);
t419 = t428 * d2;
t464 = t213 * d3 + t419;
t427 = sin(qJs1);
t418 = t427 * d2;
t459 = cos(t474);
t456 = -t459 / 0.2e1;
t458 = cos(t473);
t463 = t418 + (t456 - t458 / 0.2e1) * d3;
t426 = sin(qJs2);
t460 = -t426 * d3 + d1;
t359 = qJs6 + t390;
t444 = qJs6 + t457;
t204 = -cos(t359) / 0.2e1 - cos(t444) / 0.2e1;
t455 = t458 / 0.2e1;
t321 = qJs1 + t359;
t322 = qJs1 + t444;
t360 = qJs6 + t391;
t454 = -sin(t321) / 0.4e1 + sin(t322) / 0.4e1 + cos(t360) / 0.2e1;
t323 = qJs1 - t444;
t324 = qJs1 - t359;
t362 = -qJs6 + t393;
t453 = sin(t323) / 0.4e1 - sin(t324) / 0.4e1 + cos(t362) / 0.2e1;
t452 = cos(t321) / 0.4e1 - cos(t322) / 0.4e1 + sin(t360) / 0.2e1;
t451 = cos(t323) / 0.4e1 - cos(t324) / 0.4e1 - sin(t362) / 0.2e1;
t443 = qJs7 - qJs5 - qJs6 - t471;
t442 = t497 * a4 + t460;
t319 = -qJs7 - t444;
t317 = qJs2 - t443;
t316 = qJs1 + t443;
t315 = qJs2 + t443;
t314 = qJs7 + t360;
t313 = qJs7 + t359;
t441 = t496 * a5 + t442;
t271 = qJs1 + t313;
t272 = -qJs2 + t314;
t273 = qJs1 + t315;
t274 = -qJs2 + t316;
t406 = qJs1 + t420;
t407 = qJs1 + t470;
t440 = -sin(t271) / 0.8e1 + sin(t272) / 0.8e1 + sin(t273) / 0.8e1 - sin(t274) / 0.8e1 + cos(t314) / 0.4e1 + cos(t316) / 0.4e1 - cos(t406) / 0.4e1 - cos(t407) / 0.4e1;
t275 = qJs1 + t317;
t276 = qJs1 - t315;
t277 = qJs1 + t319;
t278 = qJs1 - t313;
t318 = qJs1 - t443;
t320 = -qJs7 + t362;
t408 = qJs1 - t470;
t409 = qJs1 - t420;
t439 = cos(t275) / 0.8e1 - cos(t276) / 0.8e1 - cos(t277) / 0.8e1 + cos(t278) / 0.8e1 + sin(t318) / 0.4e1 + sin(t320) / 0.4e1 + sin(t408) / 0.4e1 + sin(t409) / 0.4e1;
t438 = t503 * a4 + t464;
t437 = t502 * a4 + t463;
t436 = a6 * t495 + t441;
t202 = t453 - t454;
t435 = t451 + t452;
t434 = -sin(t275) / 0.8e1 + sin(t276) / 0.8e1 + sin(t277) / 0.8e1 - sin(t278) / 0.8e1 + cos(t318) / 0.4e1 + cos(t320) / 0.4e1 + cos(t408) / 0.4e1 + cos(t409) / 0.4e1;
t433 = -cos(t271) / 0.8e1 + cos(t272) / 0.8e1 + cos(t273) / 0.8e1 - cos(t274) / 0.8e1 - sin(t314) / 0.4e1 - sin(t316) / 0.4e1 + sin(t406) / 0.4e1 + sin(t407) / 0.4e1;
t432 = a5 * t500 + t437;
t431 = a5 * t501 + t438;
t430 = a6 * t498 + t432;
t429 = a6 * t499 + t431;
t214 = t459 / 0.2e1 + t455;
T_ges = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1; t427 t428 0 0; -t428 t427 0 0; 0 0 1 d1; 0 0 0 1; t456 + t455 -t213 t428 t419; t479 - t401 / 0.2e1 t214 t427 t418; -cos(qJs2) t426 0 d1; 0 0 0 1; t503 -t446 + t462 t213 t464; t502 t445 + t461 -t214 t463; t497 sin(t422) / 0.2e1 - sin(-t472) / 0.2e1 -t426 t460; 0 0 0 1; t501 -t448 + t466 t213 t438; t500 -t447 + t465 -t214 t437; t496 -sin(-t469) / 0.2e1 + sin(t410) / 0.2e1 -t426 t442; 0 0 0 1; t499 t450 + t468 t213 t431; t498 -t449 + t467 -t214 t432; t495 -sin(-t457) / 0.2e1 + sin(t390) / 0.2e1 -t426 t441; 0 0 0 1; t453 + t454 -t435 t213 t429; -t451 + t452 -t202 -t214 t430; -sin(-t444) / 0.2e1 + sin(t359) / 0.2e1 -t204 -t426 t436; 0 0 0 1; t434 + t440 t433 + t439 t435 d7 * t435 + t429; -t433 + t439 -t434 + t440 t202 d7 * t202 + t430; sin(t313) / 0.4e1 + sin(t317) / 0.4e1 - sin(t315) / 0.4e1 - sin(t319) / 0.4e1 - cos(-t470) / 0.2e1 + cos(t420) / 0.2e1 -cos(t317) / 0.4e1 + cos(t313) / 0.4e1 + cos(t319) / 0.4e1 - cos(t315) / 0.4e1 - sin(t420) / 0.2e1 - sin(-t470) / 0.2e1 t204 d7 * t204 + t436; 0 0 0 1;];