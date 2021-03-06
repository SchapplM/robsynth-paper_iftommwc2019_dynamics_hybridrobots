% Jacobian time derivative of explicit kinematic constraints of
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% WD [21x5]
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)
% [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function WD = KAS5m7DE1_kinconstr_expl_jacobianD_mdh_sym_varpar(qJ, qJD, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_kinconstr_expl_jacobianD_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7DE1_kinconstr_expl_jacobianD_mdh_sym_varpar: qJD has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_kinconstr_expl_jacobianD_mdh_sym_varpar: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_jacobianD_matlab.m
% OptimizationMode: 2
% StartTime: 2020-05-14 19:20:38
% EndTime: 2020-05-14 19:25:47
% DurationCPUTime: 309.17s
% Computational Cost: add. (9532349->325), mult. (12902067->588), div. (124327->35), fcn. (5021872->18), ass. (0->283)
t599 = cos(qJ(3));
t603 = pkin(23) - pkin(24);
t538 = t599 * t603;
t525 = -pkin(9) - t538;
t513 = 0.2e1 * t525;
t560 = pkin(3) + qJ(3);
t540 = sin(t560);
t536 = -0.2e1 * t540;
t519 = t603 * t536;
t541 = cos(t560);
t594 = t603 ^ 2;
t598 = sin(qJ(3));
t607 = pkin(11) ^ 2;
t476 = t607 - pkin(19) ^ 2 + t525 ^ 2 + t598 ^ 2 * t594 + (t541 * t513 - t598 * t519 + (t540 ^ 2 + t541 ^ 2) * pkin(12)) * pkin(12);
t532 = t541 * pkin(12);
t510 = t538 - t532;
t504 = pkin(9) + t510;
t531 = t540 * pkin(12);
t537 = t598 * t603;
t507 = -t537 - t531;
t482 = 0.4e1 * t504 ^ 2 + 0.4e1 * t507 ^ 2;
t462 = -t476 ^ 2 + t607 * t482;
t414 = sqrt(t462);
t473 = 0.2e1 * t476;
t493 = 0.2e1 * t504;
t370 = -t414 * t493 - t507 * t473;
t511 = t603 * t513;
t558 = 0.2e1 * t594;
t614 = (t558 * t599 + t511) * t598;
t475 = (-t540 * t513 - t599 * t519 + 0.4e1 * t541 * t537) * pkin(12) + t614;
t469 = 0.2e1 * t475;
t508 = t537 - t531;
t496 = 0.2e1 * t508;
t492 = 0.4e1 * t504;
t494 = 0.4e1 * t507;
t509 = -t538 - t532;
t477 = -t508 * t492 + t509 * t494;
t474 = 0.2e1 * t477;
t621 = -0.2e1 * t476;
t455 = t607 * t474 + t475 * t621;
t559 = 0.2e1 / t414;
t454 = t455 * t559;
t618 = -t504 * t454 / 0.2e1 - t509 * t473;
t448 = t414 * t496 - t507 * t469 + t618;
t606 = 0.1e1 / t482 ^ 2;
t472 = t606 * t474;
t481 = 0.1e1 / t482;
t343 = -t370 * t472 + t448 * t481;
t342 = t343 + t509;
t495 = 0.2e1 * t507;
t371 = t414 * t495 - t504 * t473;
t364 = t371 * t481 + t504;
t365 = t370 * t481 + t507;
t356 = t598 * t364 + t599 * t365;
t497 = 0.2e1 * t509;
t617 = t507 * t454 / 0.2e1 + t508 * t473;
t449 = t414 * t497 - t504 * t469 + t617;
t344 = -t371 * t472 + t449 * t481;
t443 = t344 - t508;
t317 = -t598 * t342 + t599 * t443 - t356;
t363 = t598 * t365;
t358 = t599 * t364 - t363;
t441 = t598 * t443;
t318 = t599 * t342 + t358 + t441;
t409 = 0.1e1 / pkin(19);
t596 = pkin(18) * t409;
t556 = cos(pkin(7)) * t596;
t557 = sin(pkin(7)) * t596;
t301 = t317 * t556 - t318 * t557;
t302 = 0.2e1 * t301;
t303 = t317 * t557 + t318 * t556;
t304 = 0.2e1 * t303;
t337 = -t356 * t557 + t358 * t556;
t521 = pkin(24) + t337;
t334 = 0.2e1 * t521;
t335 = t356 * t556 + t358 * t557;
t336 = 0.2e1 * t335;
t286 = 0.2e1 * t302 * t334 + 0.2e1 * t304 * t336;
t325 = t334 ^ 2 + t336 ^ 2;
t324 = 0.1e1 / t325 ^ 2;
t584 = t286 * t324;
t593 = pkin(12) * qJD(3);
t484 = t493 * t593;
t485 = t495 * t593;
t489 = qJD(3) * t496;
t486 = pkin(12) * t489;
t490 = qJD(3) * t497;
t487 = pkin(12) * t490;
t459 = (-t485 + t486) * t541 + (t484 - t487) * t540 + t614 * qJD(3);
t458 = 0.2e1 * t459;
t447 = qJD(3) * t618 + t414 * t489 - t507 * t458;
t471 = qJD(3) * t474;
t467 = t606 * t471;
t524 = qJD(3) * t532;
t528 = qJD(3) * t538;
t327 = -t370 * t467 + t447 * t481 - t524 - t528;
t446 = qJD(3) * t617 + t414 * t490 - t504 * t458;
t523 = qJD(3) * t531;
t527 = qJD(3) * t537;
t439 = -t371 * t467 + t446 * t481 + t523 - t527;
t545 = qJD(3) * t599;
t313 = -qJD(3) * t363 + t599 * t327 + t364 * t545 + t598 * t439;
t352 = t356 ^ 2;
t354 = 0.1e1 / t358 ^ 2;
t340 = t352 * t354 + 0.1e1;
t353 = 0.1e1 / t358;
t573 = t354 * t356;
t534 = -t598 * t327 + t599 * t439;
t312 = -t356 * qJD(3) + t534;
t579 = t312 * t353 * t354;
t623 = 0.2e1 * (-t317 * t573 + t318 * t353) * (t313 * t573 - t352 * t579) / t340 ^ 2;
t515 = t598 * t356 + t599 * t358;
t543 = -qJ(4) + t560;
t399 = cos(t543) * pkin(12);
t403 = qJ(4) - qJ(3) + pkin(4);
t400 = sin(t403);
t402 = cos(t403);
t533 = pkin(14) * t400 + pkin(15) * t402;
t387 = -pkin(10) - t399 - t533;
t428 = t387 ^ 2;
t385 = 0.1e1 / t428;
t542 = t402 * pkin(14) - pkin(15) * t400;
t597 = pkin(12) * sin(t543);
t389 = -t542 + t597;
t388 = t389 ^ 2;
t379 = t385 * t388 + 0.1e1;
t404 = qJD(4) - qJD(3);
t396 = t404 * t597;
t566 = t402 * t404;
t567 = t400 * t404;
t535 = -pkin(14) * t566 + pkin(15) * t567;
t380 = t396 + t535;
t397 = t404 * t399;
t561 = -pkin(14) * t567 - pkin(15) * t566;
t381 = t397 - t561;
t382 = t397 + t561;
t383 = -t396 + t535;
t384 = 0.1e1 / t387;
t390 = -t399 + t533;
t391 = t542 + t597;
t568 = t389 * t391;
t569 = t383 * t384 * t385;
t570 = t382 * t389;
t622 = 0.1e1 / t379 * ((t381 * t389 + t382 * t391 + t383 * t390) * t385 + t380 * t384 + 0.2e1 * t568 * t569) + 0.2e1 * (t384 * t390 + t385 * t568) * (-t385 * t570 - t388 * t569) / t379 ^ 2;
t323 = 0.1e1 / t325;
t366 = t370 ^ 2;
t368 = 0.1e1 / t371 ^ 2;
t361 = t366 * t368 + 0.1e1;
t367 = 0.1e1 / t371;
t369 = t367 * t368;
t445 = t368 * t447;
t604 = 0.2e1 * t370;
t620 = (-0.2e1 * t366 * t369 * t446 + t445 * t604) / t361 ^ 2;
t498 = t510 * qJD(3);
t499 = qJD(3) * t507;
t544 = 0.8e1 * qJD(3);
t461 = -0.2e1 * t492 * t498 - 0.2e1 * t494 * t499 + (t508 ^ 2 + t509 ^ 2) * t544;
t616 = (t477 ^ 2 * t481 * t544 - t461) * t606;
t615 = -qJD(3) * t469 - t458;
t453 = qJD(3) * t454;
t488 = 0.2e1 * t499;
t491 = 0.2e1 * t498;
t457 = qJD(3) * t599 ^ 2 * t558 + t540 * t485 + t486 * t536 + t488 * t531 + t491 * t532 + t511 * t545 + (t484 - 0.2e1 * t487) * t541;
t456 = 0.2e1 * t457;
t470 = qJD(3) * t473;
t610 = (qJD(3) / t462 * t455 ^ 2 / 0.4e1 - t457 * t621 / 0.2e1 + t459 * t475 - t607 * t461 / 0.2e1) * t559;
t437 = (t414 * t491 + t508 * t453 + t615 * t509 + (-t456 + t470) * t507 + t610 * t504) * t481 - t448 * t467 - t447 * t472 + t616 * t370;
t435 = -t527 - t523 - t437;
t438 = (-t414 * t488 + t509 * t453 - t504 * t456 + t510 * t470 - t507 * t610 - t508 * t615) * t481 - t449 * t467 - t446 * t472 + t616 * t371;
t436 = -t528 + t524 + t438;
t283 = -qJD(3) * t317 + t599 * t435 - t598 * t436 - t534;
t613 = 0.2e1 * t317 * t356 * t579 - t283 * t353;
t611 = t312 * t318 + t313 * t317;
t378 = t428 + t388;
t608 = (0.1e1 / t378 * (t387 * t391 - t389 * t390) * (t383 * t387 - t570) - t380 * t389 + t381 * t387 - t382 * t390 - t383 * t391) * t378 ^ (-0.1e1 / 0.2e1);
t410 = pkin(17) ^ 2;
t294 = -pkin(22) ^ 2 + pkin(24) ^ 2 + t410 + (t334 - t337) * t337 + (t336 - t335) * t335;
t290 = -t294 ^ 2 + t325 * t410;
t412 = sqrt(t290);
t530 = -t294 * t334 + t336 * t412;
t275 = t530 * t323 + t521;
t279 = -t294 * t336 - t334 * t412;
t277 = -t279 * t323 - t335;
t406 = sin(pkin(6));
t408 = cos(pkin(6));
t411 = 0.1e1 / pkin(22);
t259 = (t275 * t406 + t277 * t408) * t411;
t260 = (-t275 * t408 + t277 * t406) * t411;
t255 = (t259 * t408 + t260 * t406) * pkin(22) + t335;
t252 = 0.1e1 / t255;
t272 = 0.1e1 / t275;
t562 = t515 * t409;
t331 = t562 * pkin(19) - t504;
t328 = 0.1e1 / t331;
t288 = 0.1e1 / t412;
t253 = 0.1e1 / t255 ^ 2;
t273 = 0.1e1 / t275 ^ 2;
t329 = 0.1e1 / t331 ^ 2;
t605 = -0.2e1 * t294;
t601 = -t334 / 0.2e1;
t600 = t336 / 0.2e1;
t595 = pkin(19) * t409;
t297 = t312 * t556 - t313 * t557;
t298 = 0.2e1 * t297;
t299 = t312 * t557 + t313 * t556;
t300 = 0.2e1 * t299;
t285 = 0.2e1 * t298 * t334 + 0.2e1 * t300 * t336;
t563 = -0.2e1 * t335 + t336;
t564 = t334 - 0.2e1 * t337;
t264 = t564 * t297 + t298 * t337 + t563 * t299 + t300 * t335;
t256 = t264 * t605 + t285 * t410;
t546 = t288 * t600;
t501 = t256 * t546 - t334 * t264 - t298 * t294 + t300 * t412;
t520 = t324 * t530;
t237 = -t285 * t520 + t501 * t323 + t297;
t547 = t288 * t601;
t246 = t256 * t547 - t264 * t336 - t294 * t300 - t298 * t412;
t585 = t285 * t324;
t238 = -t246 * t323 + t279 * t585 - t299;
t232 = (t237 * t406 + t238 * t408) * t411;
t233 = (-t237 * t408 + t238 * t406) * t411;
t224 = (t232 * t408 + t233 * t406) * pkin(22) + t299;
t592 = t224 * t252 * t253;
t590 = t237 * t272 * t273;
t251 = -(t259 * t406 - t260 * t408) * pkin(22) + t521;
t589 = t251 * t253;
t588 = t273 * t277;
t514 = t598 * t312 - t599 * t313;
t293 = t509 * qJD(3) + (t515 * qJD(3) + t514) * t595;
t548 = t598 * t358;
t551 = t599 * t356;
t333 = (-t551 + t548) * t595 + t507;
t332 = t333 ^ 2;
t322 = t329 * t332 + 0.1e1;
t575 = t329 * t333;
t552 = (t312 * t599 + t313 * t598 + t356 * t545) * t409;
t292 = t552 * pkin(19) + (-t548 * t595 + t508) * qJD(3);
t581 = t292 * t328 * t329;
t587 = 0.2e1 * (t293 * t575 - t332 * t581) / t322 ^ 2;
t583 = t288 * t323;
t320 = 0.1e1 / t322;
t576 = t320 * t329;
t572 = t368 * t370;
t284 = -qJD(3) * t441 - t342 * t545 + t598 * t435 + t599 * t436 - t313;
t565 = t283 * t556 - t284 * t557;
t555 = t285 * t323 * t584;
t265 = t564 * t301 + t302 * t337 + t563 * t303 + t304 * t335;
t257 = t265 * t605 + t286 * t410;
t554 = t288 / t290 * t257 * t256;
t526 = t283 * t557 + t284 * t556;
t516 = t598 * t317 - t599 * t318;
t503 = t599 * t317 + t598 * t318 - t548;
t500 = t257 * t546 - t334 * t265 - t302 * t294 + t304 * t412;
t269 = 0.2e1 * t565;
t271 = 0.2e1 * t526;
t236 = -t269 * t335 + t271 * t337 + t298 * t301 + t300 * t303 + (-0.2e1 * t303 + t304) * t299 + (-0.2e1 * t301 + t302) * t297 + t564 * t526 - t563 * t565;
t249 = -0.2e1 * t269 * t336 + 0.2e1 * t271 * t334 + 0.2e1 * t298 * t302 + 0.2e1 * t300 * t304;
t231 = t236 * t605 + t249 * t410 - 0.2e1 * t264 * t265;
t247 = t257 * t547 - t265 * t336 - t294 * t304 - t302 * t412;
t222 = -(-t236 * t336 - t265 * t300 - t264 * t304 + t294 * t269 - t271 * t412 + t334 * t554 / 0.4e1) * t323 - 0.2e1 * t279 * t555 - (-t302 * t256 / 0.2e1 - t298 * t257 / 0.2e1 + t231 * t601) * t583 + (t246 * t286 + t247 * t285 + t249 * t279) * t324 + t565;
t223 = (-t271 * t294 - t302 * t264 - t298 * t265 - t334 * t236 - t269 * t412 - t336 * t554 / 0.4e1) * t323 - t500 * t585 - t501 * t584 + 0.2e1 * t530 * t555 - t249 * t520 + (t304 * t256 / 0.2e1 + t300 * t257 / 0.2e1 + t231 * t600) * t583 + t526;
t239 = -t286 * t520 + t500 * t323 + t301;
t240 = -t247 * t323 + t279 * t584 - t303;
t276 = t277 ^ 2;
t263 = t273 * t276 + 0.1e1;
t483 = 0.2e1 * (t239 * t588 - t240 * t272) * (t238 * t588 - t276 * t590) / t263 ^ 2 + (t222 * t272 + 0.2e1 * t239 * t277 * t590 + (-t223 * t277 - t237 * t240 - t238 * t239) * t273) / t263;
t359 = 0.1e1 / t361;
t480 = t359 * t482;
t479 = t367 * t482;
t478 = t482 * t572;
t468 = t359 * t471;
t444 = t446 * t480;
t338 = 0.1e1 / t340;
t250 = t251 ^ 2;
t243 = t250 * t253 + 0.1e1;
t235 = (-t239 * t408 + t240 * t406) * t411;
t234 = (t239 * t406 + t240 * t408) * t411;
t227 = -(t234 * t406 - t235 * t408) * pkin(22) + t301;
t226 = (t234 * t408 + t235 * t406) * pkin(22) + t303;
t225 = -(t232 * t406 - t233 * t408) * pkin(22) + t297;
t219 = (t222 * t408 + t223 * t406) * t411;
t218 = (t222 * t406 - t223 * t408) * t411;
t1 = [0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, t483, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0.2e1 * (-t226 * t589 + t227 * t252) / t243 ^ 2 * (t225 * t589 - t250 * t592) + (-(-(-t218 * t408 + t219 * t406) * pkin(22) + t526) * t252 - 0.2e1 * t226 * t251 * t592 + (t227 * t224 + ((t218 * t406 + t219 * t408) * pkin(22) - t565) * t251 + t226 * t225) * t253) / t243 + t483, 0, 0; 0, 0, -t623 + ((-t284 * t356 - t611) * t354 + t613) * t338, 0, 0; 0, 0, t623 + (-(((t599 * t283 + t598 * t284) * t409 + t552) * pkin(19) + (t503 * t595 - t507) * qJD(3)) * t328 + (t498 + (t599 * t284 - t598 * t283 + (-t515 - t516) * qJD(3) - t514) * t595) * t575) * t320 + (t292 * t576 + t328 * t587) * ((t516 * t409 + t562) * pkin(19) + t509) + (-0.2e1 * t333 * t320 * t581 + t293 * t576 - t575 * t587) * ((t551 + t503) * t595 + t508) + (t284 * t573 + t354 * t611 - t613) * t338, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, -t622, t622, 0; 0, 0, -t608, t608, 0; 0, 0, 0, 0, 0; 0, 0, (t437 * t479 - t438 * t478) * t359 + (t367 * t468 - t368 * t444 - t479 * t620) * t343 + (t369 * t444 * t604 - t445 * t480 - t468 * t572 + t478 * t620) * t344, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0;];
WD = t1;
