% Jacobian time derivative of explicit kinematic constraints of
% KAS5m5
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% WD [13x5]
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)
% [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function WD = KAS5m5_kinconstr_expl_jacobianD_mdh_sym_varpar(qJ, qJD, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_kinconstr_expl_jacobianD_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m5_kinconstr_expl_jacobianD_mdh_sym_varpar: qJD has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_kinconstr_expl_jacobianD_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_jacobianD_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 17:52:00
% EndTime: 2020-06-27 17:57:43
% DurationCPUTime: 333.05s
% Computational Cost: add. (9531967->315), mult. (12901857->574), div. (124327->34), fcn. (5021744->16), ass. (0->274)
t577 = cos(qJ(3));
t581 = pkin(25) - pkin(26);
t520 = t577 * t581;
t509 = -pkin(15) - t520;
t495 = 0.2e1 * t509;
t539 = pkin(29) + qJ(3);
t522 = sin(t539);
t518 = -0.2e1 * t522;
t501 = t581 * t518;
t523 = cos(t539);
t571 = t581 ^ 2;
t576 = sin(qJ(3));
t584 = pkin(17) ^ 2;
t459 = t584 - pkin(23) ^ 2 + t509 ^ 2 + t576 ^ 2 * t571 + (t523 * t495 - t576 * t501 + (t522 ^ 2 + t523 ^ 2) * pkin(18)) * pkin(18);
t515 = t523 * pkin(18);
t492 = t520 - t515;
t485 = pkin(15) + t492;
t514 = t522 * pkin(18);
t519 = t576 * t581;
t489 = -t519 - t514;
t465 = 0.4e1 * t485 ^ 2 + 0.4e1 * t489 ^ 2;
t445 = -t459 ^ 2 + t465 * t584;
t398 = sqrt(t445);
t456 = 0.2e1 * t459;
t476 = 0.2e1 * t485;
t361 = -t398 * t476 - t456 * t489;
t493 = t581 * t495;
t540 = 0.2e1 * t571;
t590 = (t540 * t577 + t493) * t576;
t458 = (-t495 * t522 - t501 * t577 + 0.4e1 * t519 * t523) * pkin(18) + t590;
t454 = 0.2e1 * t458;
t490 = t519 - t514;
t479 = 0.2e1 * t490;
t475 = 0.4e1 * t485;
t477 = 0.4e1 * t489;
t491 = -t520 - t515;
t460 = -t475 * t490 + t477 * t491;
t457 = 0.2e1 * t460;
t597 = -0.2e1 * t459;
t438 = t457 * t584 + t458 * t597;
t541 = 0.2e1 / t398;
t437 = t438 * t541;
t594 = -t485 * t437 / 0.2e1 - t491 * t456;
t431 = t398 * t479 - t454 * t489 + t594;
t583 = 0.1e1 / t465 ^ 2;
t455 = t583 * t457;
t464 = 0.1e1 / t465;
t334 = -t361 * t455 + t431 * t464;
t333 = t334 + t491;
t478 = 0.2e1 * t489;
t362 = t398 * t478 - t456 * t485;
t355 = t362 * t464 + t485;
t356 = t361 * t464 + t489;
t347 = t355 * t576 + t356 * t577;
t480 = 0.2e1 * t491;
t593 = t489 * t437 / 0.2e1 + t490 * t456;
t432 = t398 * t480 - t454 * t485 + t593;
t335 = -t362 * t455 + t432 * t464;
t426 = t335 - t490;
t308 = -t333 * t576 + t426 * t577 - t347;
t354 = t576 * t356;
t349 = t355 * t577 - t354;
t424 = t576 * t426;
t309 = t333 * t577 + t349 + t424;
t394 = 0.1e1 / pkin(23);
t573 = pkin(22) * t394;
t537 = cos(pkin(14)) * t573;
t538 = sin(pkin(14)) * t573;
t292 = t308 * t537 - t309 * t538;
t293 = 0.2e1 * t292;
t294 = t308 * t538 + t309 * t537;
t295 = 0.2e1 * t294;
t328 = -t347 * t538 + t349 * t537;
t507 = pkin(26) + t328;
t325 = 0.2e1 * t507;
t326 = t347 * t537 + t349 * t538;
t327 = 0.2e1 * t326;
t277 = 0.2e1 * t293 * t325 + 0.2e1 * t295 * t327;
t316 = t325 ^ 2 + t327 ^ 2;
t315 = 0.1e1 / t316 ^ 2;
t561 = t277 * t315;
t570 = pkin(18) * qJD(3);
t467 = t476 * t570;
t468 = t478 * t570;
t472 = qJD(3) * t479;
t469 = pkin(18) * t472;
t473 = qJD(3) * t480;
t470 = pkin(18) * t473;
t442 = (-t468 + t469) * t523 + (t467 - t470) * t522 + t590 * qJD(3);
t441 = 0.2e1 * t442;
t430 = qJD(3) * t594 + t398 * t472 - t489 * t441;
t453 = qJD(3) * t457;
t450 = t583 * t453;
t506 = qJD(3) * t515;
t512 = qJD(3) * t520;
t318 = -t361 * t450 + t430 * t464 - t506 - t512;
t429 = qJD(3) * t593 + t398 * t473 - t485 * t441;
t505 = qJD(3) * t514;
t511 = qJD(3) * t519;
t422 = -t362 * t450 + t429 * t464 + t505 - t511;
t526 = qJD(3) * t577;
t304 = -qJD(3) * t354 + t318 * t577 + t355 * t526 + t422 * t576;
t343 = t347 ^ 2;
t345 = 0.1e1 / t349 ^ 2;
t331 = t343 * t345 + 0.1e1;
t344 = 0.1e1 / t349;
t550 = t345 * t347;
t517 = -t318 * t576 + t422 * t577;
t303 = -qJD(3) * t347 + t517;
t556 = t303 * t344 * t345;
t599 = 0.2e1 * (-t308 * t550 + t309 * t344) * (t304 * t550 - t343 * t556) / t331 ^ 2;
t497 = t576 * t347 + t577 * t349;
t387 = -qJ(4) + t539;
t382 = cos(t387) * pkin(18);
t388 = qJ(4) - qJ(3) + pkin(30);
t384 = sin(t388);
t386 = cos(t388);
t516 = pkin(19) * t384 + pkin(20) * t386;
t374 = -pkin(16) - t382 - t516;
t372 = 0.1e1 / t374 ^ 2;
t524 = -pkin(19) * t386 + pkin(20) * t384;
t575 = pkin(18) * sin(t387);
t376 = t524 + t575;
t375 = t376 ^ 2;
t366 = t372 * t375 + 0.1e1;
t389 = qJD(4) - qJD(3);
t380 = t389 * t382;
t503 = t516 * t389;
t369 = -t380 + t503;
t379 = t389 * t575;
t504 = t524 * t389;
t370 = -t379 + t504;
t371 = 0.1e1 / t374;
t377 = -t382 + t516;
t378 = -t524 + t575;
t546 = t376 * t378;
t547 = t370 * t371 * t372;
t598 = (t372 * ((t380 + t503) * t376 - t369 * t378 + t370 * t377) + (t379 + t504) * t371 + 0.2e1 * t546 * t547) / t366 + 0.2e1 * (t371 * t377 + t372 * t546) * (t369 * t372 * t376 - t375 * t547) / t366 ^ 2;
t314 = 0.1e1 / t316;
t357 = t361 ^ 2;
t359 = 0.1e1 / t362 ^ 2;
t352 = t357 * t359 + 0.1e1;
t358 = 0.1e1 / t362;
t360 = t358 * t359;
t428 = t359 * t430;
t596 = 0.2e1 * (-t357 * t360 * t429 + t361 * t428) / t352 ^ 2;
t481 = t492 * qJD(3);
t482 = qJD(3) * t489;
t525 = 0.8e1 * qJD(3);
t444 = -0.2e1 * t475 * t481 - 0.2e1 * t477 * t482 + (t490 ^ 2 + t491 ^ 2) * t525;
t592 = (t460 ^ 2 * t464 * t525 - t444) * t583;
t591 = -qJD(3) * t454 - t441;
t436 = qJD(3) * t437;
t471 = 0.2e1 * t482;
t474 = 0.2e1 * t481;
t440 = qJD(3) * t577 ^ 2 * t540 + t522 * t468 + t469 * t518 + t471 * t514 + t474 * t515 + t493 * t526 + (t467 - 0.2e1 * t470) * t523;
t439 = 0.2e1 * t440;
t452 = qJD(3) * t456;
t586 = (qJD(3) / t445 * t438 ^ 2 / 0.4e1 - t440 * t597 / 0.2e1 + t442 * t458 - t584 * t444 / 0.2e1) * t541;
t420 = (t398 * t474 + t490 * t436 + t591 * t491 + (-t439 + t452) * t489 + t586 * t485) * t464 - t431 * t450 - t430 * t455 + t592 * t361;
t418 = -t511 - t505 - t420;
t421 = (-t398 * t471 + t491 * t436 - t485 * t439 + t492 * t452 - t489 * t586 - t490 * t591) * t464 - t432 * t450 - t429 * t455 + t592 * t362;
t419 = -t512 + t506 + t421;
t274 = -qJD(3) * t308 + t577 * t418 - t576 * t419 - t517;
t589 = 0.2e1 * t308 * t347 * t556 - t274 * t344;
t587 = t303 * t309 + t304 * t308;
t395 = pkin(21) ^ 2;
t285 = -pkin(24) ^ 2 + pkin(26) ^ 2 + t395 + (t325 - t328) * t328 + (t327 - t326) * t326;
t281 = -t285 ^ 2 + t316 * t395;
t397 = sqrt(t281);
t513 = -t285 * t325 + t327 * t397;
t266 = t314 * t513 + t507;
t270 = -t285 * t327 - t325 * t397;
t268 = -t270 * t314 - t326;
t391 = sin(pkin(13));
t393 = cos(pkin(13));
t396 = 0.1e1 / pkin(24);
t250 = (t266 * t391 + t268 * t393) * t396;
t251 = (-t266 * t393 + t268 * t391) * t396;
t246 = (t250 * t393 + t251 * t391) * pkin(24) + t326;
t243 = 0.1e1 / t246;
t263 = 0.1e1 / t266;
t542 = t497 * t394;
t322 = pkin(23) * t542 - t485;
t319 = 0.1e1 / t322;
t279 = 0.1e1 / t397;
t244 = 0.1e1 / t246 ^ 2;
t264 = 0.1e1 / t266 ^ 2;
t320 = 0.1e1 / t322 ^ 2;
t582 = -0.2e1 * t285;
t579 = -t325 / 0.2e1;
t578 = t327 / 0.2e1;
t572 = pkin(23) * t394;
t288 = t303 * t537 - t304 * t538;
t289 = 0.2e1 * t288;
t290 = t303 * t538 + t304 * t537;
t291 = 0.2e1 * t290;
t276 = 0.2e1 * t289 * t325 + 0.2e1 * t291 * t327;
t543 = -0.2e1 * t326 + t327;
t544 = t325 - 0.2e1 * t328;
t255 = t288 * t544 + t289 * t328 + t290 * t543 + t291 * t326;
t247 = t255 * t582 + t276 * t395;
t527 = t279 * t578;
t484 = t247 * t527 - t255 * t325 - t285 * t289 + t291 * t397;
t502 = t315 * t513;
t228 = -t276 * t502 + t314 * t484 + t288;
t528 = t279 * t579;
t237 = t247 * t528 - t255 * t327 - t285 * t291 - t289 * t397;
t562 = t276 * t315;
t229 = -t237 * t314 + t270 * t562 - t290;
t223 = (t228 * t391 + t229 * t393) * t396;
t224 = (-t228 * t393 + t229 * t391) * t396;
t215 = (t223 * t393 + t224 * t391) * pkin(24) + t290;
t569 = t215 * t243 * t244;
t567 = t228 * t263 * t264;
t242 = -(t250 * t391 - t251 * t393) * pkin(24) + t507;
t566 = t242 * t244;
t565 = t264 * t268;
t496 = t303 * t576 - t304 * t577;
t284 = t491 * qJD(3) + (qJD(3) * t497 + t496) * t572;
t529 = t576 * t349;
t532 = t577 * t347;
t324 = (-t532 + t529) * t572 + t489;
t323 = t324 ^ 2;
t313 = t320 * t323 + 0.1e1;
t552 = t320 * t324;
t533 = (t303 * t577 + t304 * t576 + t347 * t526) * t394;
t283 = t533 * pkin(23) + (-t529 * t572 + t490) * qJD(3);
t558 = t283 * t319 * t320;
t564 = 0.2e1 * (t284 * t552 - t323 * t558) / t313 ^ 2;
t560 = t279 * t314;
t311 = 0.1e1 / t313;
t553 = t311 * t320;
t549 = t359 * t361;
t275 = -qJD(3) * t424 - t333 * t526 + t418 * t576 + t419 * t577 - t304;
t545 = t274 * t537 - t275 * t538;
t256 = t292 * t544 + t293 * t328 + t294 * t543 + t295 * t326;
t248 = t256 * t582 + t277 * t395;
t536 = t279 / t281 * t248 * t247;
t534 = t314 * t276 * t561;
t510 = t274 * t538 + t275 * t537;
t498 = t308 * t576 - t309 * t577;
t487 = t308 * t577 + t309 * t576 - t529;
t483 = t248 * t527 - t256 * t325 - t285 * t293 + t295 * t397;
t260 = 0.2e1 * t545;
t262 = 0.2e1 * t510;
t227 = -t260 * t326 + t262 * t328 + t289 * t292 + t291 * t294 + (-0.2e1 * t294 + t295) * t290 + (-0.2e1 * t292 + t293) * t288 + t544 * t510 - t543 * t545;
t240 = -0.2e1 * t260 * t327 + 0.2e1 * t262 * t325 + 0.2e1 * t289 * t293 + 0.2e1 * t291 * t295;
t222 = t227 * t582 + t240 * t395 - 0.2e1 * t255 * t256;
t238 = t248 * t528 - t256 * t327 - t285 * t295 - t293 * t397;
t213 = -(-t227 * t327 - t256 * t291 - t255 * t295 + t285 * t260 - t262 * t397 + t325 * t536 / 0.4e1) * t314 - 0.2e1 * t270 * t534 - (-t293 * t247 / 0.2e1 - t289 * t248 / 0.2e1 + t222 * t579) * t560 + (t237 * t277 + t238 * t276 + t240 * t270) * t315 + t545;
t214 = (-t262 * t285 - t293 * t255 - t289 * t256 - t325 * t227 - t260 * t397 - t327 * t536 / 0.4e1) * t314 - t483 * t562 - t484 * t561 + 0.2e1 * t513 * t534 - t240 * t502 + (t295 * t247 / 0.2e1 + t291 * t248 / 0.2e1 + t222 * t578) * t560 + t510;
t230 = -t277 * t502 + t314 * t483 + t292;
t231 = -t238 * t314 + t270 * t561 - t294;
t267 = t268 ^ 2;
t254 = t264 * t267 + 0.1e1;
t466 = 0.2e1 * (t230 * t565 - t231 * t263) * (t229 * t565 - t267 * t567) / t254 ^ 2 + (t213 * t263 + 0.2e1 * t230 * t268 * t567 + (-t214 * t268 - t228 * t231 - t229 * t230) * t264) / t254;
t350 = 0.1e1 / t352;
t463 = t350 * t465;
t462 = t358 * t465;
t461 = t465 * t549;
t451 = t350 * t453;
t427 = t429 * t463;
t329 = 0.1e1 / t331;
t241 = t242 ^ 2;
t234 = t241 * t244 + 0.1e1;
t226 = (-t230 * t393 + t231 * t391) * t396;
t225 = (t230 * t391 + t231 * t393) * t396;
t218 = -(t225 * t391 - t226 * t393) * pkin(24) + t292;
t217 = (t225 * t393 + t226 * t391) * pkin(24) + t294;
t216 = -(t223 * t391 - t224 * t393) * pkin(24) + t288;
t210 = (t213 * t393 + t214 * t391) * t396;
t209 = (t213 * t391 - t214 * t393) * t396;
t1 = [0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, t466, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0.2e1 * (-t217 * t566 + t218 * t243) / t234 ^ 2 * (t216 * t566 - t241 * t569) + (-(-(-t209 * t393 + t210 * t391) * pkin(24) + t510) * t243 - 0.2e1 * t217 * t242 * t569 + (t218 * t215 + ((t209 * t391 + t210 * t393) * pkin(24) - t545) * t242 + t217 * t216) * t244) / t234 + t466, 0, 0; 0, 0, -t599 + ((-t275 * t347 - t587) * t345 + t589) * t329, 0, 0; 0, 0, t599 + (-(((t274 * t577 + t275 * t576) * t394 + t533) * pkin(23) + (t487 * t572 - t489) * qJD(3)) * t319 + (t481 + (t577 * t275 - t576 * t274 + (-t497 - t498) * qJD(3) - t496) * t572) * t552) * t311 + (t283 * t553 + t319 * t564) * ((t394 * t498 + t542) * pkin(23) + t491) + (-0.2e1 * t311 * t324 * t558 + t284 * t553 - t552 * t564) * ((t532 + t487) * t572 + t490) + (t275 * t550 + t345 * t587 - t589) * t329, 0, 0; 0, 0, (-t420 * t462 + t421 * t461) * t350 + (-t358 * t451 + t359 * t427 + t462 * t596) * t334 + (-0.2e1 * t360 * t361 * t427 + t428 * t463 + t451 * t549 - t461 * t596) * t335, 0, 0; 0, 0, -t598, t598, 0;];
WD = t1;
