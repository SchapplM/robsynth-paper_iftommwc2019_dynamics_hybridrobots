% Direkte Kinematik für KAS5m5 (MDH-Konvention, symbolische Berechnung)
% 
% Teste Kinematik-Berechnung mit Berücksichtigung der kinematischen
% Zwangsbedingungen und MDH-Konvention aus Maple.
% 
% Input:
% q [5x1]
%   Gelenkwinkel, verallgemeinerte Koordinaten von KAS5m3. Definition
%   siehe Dokumentation
% a_mdh, d_mdh
%   MDH-Parameter 
% l_const, w_const
%   Konstante geometrische Größen. Diese Größen werden zur Berechnung der
%   kinematischen Zwangsbedingungen benötigt. Die Kinematik ist daher nicht
%   nur mit den MDH-Parametern berechenbar (wenn die Winkel der
%   Parallelstruktur außerhalb berechnet werden).
% 
% Output:
% T_c_mdh [4*14x4]
%   homogenious transformation matrices for each body frame (MDH)
%    1.. 4: KAS-Basis  -> MDH Basis (link 0)
%    5.. 9: KAS-Basis  -> MDH Segment 1
%   10..13: KAS-Basis  -> MDH Segment 2
%   14..17: KAS-Basis  -> MDH Segment 3
%   ...
%   29..32: KAS-Basis  -> MDH Segment 7
%   33..36: KAS-Basis  -> MDH Segment 8
%   ...
%   53..56: KAS-Basis  -> MDH Segment 13

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_ges = KAS5m5_fkine_mdh_sym_varpar(q, a_mdh, d_mdh, l_const, w_const)
%% Init
% # codegen deaktiviert
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [5 1]), ...
  'Gelenkwinkel q gefordert als [5x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [13 1]), ...
  'a_mdh has to be [13x1] double'); 
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [13 1]), ...
  'd_mdh has to be [13x1] double'); 
assert(isa(l_const,'double') && isreal(l_const) && all(size(l_const) == [23 1]), ...
  'Konstante Laengen gefordert als [23x1] double');
assert(isa(w_const,'double') && isreal(w_const) && all(size(w_const) == [7 1]), ...
  'Konstante Winkel gefordert als [7x1] double');

qJs1 = q(1);
qJs2 = q(2);
qJs3 = q(3);
qJs4 = q(4);
qJs5 = q(5);

a4 = a_mdh(4);
a5 = a_mdh(5);
a6 = a_mdh(6);
a9 = a_mdh(9);
a10 = a_mdh(10);
a11 = a_mdh(11);
a12 = a_mdh(12);

d1 = d_mdh(1);
d2 = d_mdh(2);
d3 = d_mdh(3);
d7 = d_mdh(7);
d8 = d_mdh(8);

l1 = l_const(1);
l2 = l_const(2);
l3 = l_const(3);
l4 = l_const(4);
l5 = l_const(5);
l6 = l_const(6);
l7 = l_const(7);
l8 = l_const(8);
l9 = l_const(9);
l10 = l_const(10);
l11 = l_const(11);
l12 = l_const(12);
l13 = l_const(13);
l14 = l_const(14);
l15 = l_const(15);
l17 = l_const(17);
l18 = l_const(18);
l19 = l_const(19);
l20 = l_const(20);
l21 = l_const(21);
l22 = l_const(22);

delta9s = w_const(1);
delta10s = w_const(2);
delta12s = w_const(3);
delta17s = w_const(4);
delta8s = w_const(5);
delta18s = w_const(6);

rxs_base = 0;
rys_base = 0;
rzs_base = 0;
%% Berechnung mit exportiertem Code
% codegen/KAS5_sym_codegen_floatbase_mdh_kinematik_gesamt.mw
% codeexport/KAS5m5_fkine_matlab.m
t139 = 0.1e1 / l22;
t185 = l21 * t139;
t174 = cos(delta9s) * t185;
t175 = sin(delta9s) * t185;
t130 = sin(qJs3);
t135 = cos(qJs3);
t138 = -l5 + l6;
t107 = t135 * t138 - l11;
t122 = delta17s + qJs3;
t117 = cos(t122);
t161 = t117 * l14 + t107;
t105 = 0.2e1 * t161;
t116 = sin(t122);
t173 = t116 * l14 - t130 * t138;
t106 = 0.2e1 * t173;
t143 = l13 ^ 2;
t94 = t130 ^ 2 * t138 ^ 2 - l22 ^ 2 + t107 ^ 2 + t143 + (t105 * t117 + t106 * t116 + (-t116 ^ 2 - t117 ^ 2) * l14) * l14;
t97 = t105 ^ 2 + t106 ^ 2;
t93 = sqrt(t143 * t97 - t94 ^ 2);
t166 = t105 * t94 - t106 * t93;
t96 = 0.1e1 / t97;
t158 = t166 * t96;
t87 = t158 - t161;
t90 = t105 * t93 + t94 * t106;
t88 = t90 * t96 - t173;
t83 = t130 * t87 + t135 * t88;
t84 = -t130 * t88 + t135 * t87;
t76 = t83 * t174 + t84 * t175;
t136 = cos(qJs2);
t125 = sin(delta8s);
t127 = cos(delta8s);
t145 = 0.1e1 / l4;
t78 = t84 * t174 - t83 * t175;
t160 = -l6 - t78;
t140 = l20 ^ 2;
t75 = 0.2e1 * t160;
t77 = 0.2e1 * t76;
t68 = -l4 ^ 2 + l6 ^ 2 + t140 + (-t75 - t78) * t78 + (t77 - t76) * t76;
t73 = t75 ^ 2 + t77 ^ 2;
t67 = sqrt(t140 * t73 - t68 ^ 2);
t72 = 0.1e1 / t73;
t65 = -(t77 * t67 + t75 * t68) * t72 + t160;
t66 = -(t75 * t67 - t68 * t77) * t72 - t76;
t63 = (-t125 * t65 + t127 * t66) * t145;
t184 = t136 * t63;
t144 = 0.1e1 / l13;
t183 = t144 * t96;
t131 = sin(qJs2);
t132 = sin(qJs1);
t182 = t132 * t131;
t114 = t132 * t136;
t181 = t136 * t125;
t180 = t136 * t139;
t137 = cos(qJs1);
t179 = t137 * t131;
t178 = t137 * t136;
t123 = d1 + rzs_base;
t177 = t132 * d2 + rys_base;
t176 = t137 * d2 + rxs_base;
t172 = d8 * t114 + t176;
t171 = d3 * t114 + t176;
t64 = (t125 * t66 + t127 * t65) * t145;
t57 = t137 * t64 - t63 * t182;
t170 = t57 * a10 + t171;
t169 = t57 * a4 + t171;
t168 = -t131 * d3 + t123;
t167 = -t131 * d8 + t123;
t58 = t137 * t63 + t64 * t182;
t33 = (t57 * t84 + t58 * t83) * t139;
t165 = t33 * a11 + t170;
t42 = t58 * t130 + t57 * t135;
t164 = t42 * a5 + t169;
t163 = -d3 * t178 + t177;
t162 = -d8 * t178 + t177;
t129 = sin(qJs4);
t134 = cos(qJs4);
t41 = -t57 * t130 + t58 * t135;
t24 = t41 * t129 + t42 * t134;
t159 = t24 * a6 + t164;
t157 = (t130 * t84 - t135 * t83) * t139;
t59 = t132 * t64 + t63 * t179;
t156 = t59 * a10 + t163;
t155 = t59 * a4 + t163;
t60 = t132 * t63 - t64 * t179;
t35 = (t59 * t84 + t60 * t83) * t139;
t154 = t35 * a11 + t156;
t44 = t60 * t130 + t59 * t135;
t153 = t44 * a5 + t155;
t152 = -a4 * t184 + t168;
t151 = -a10 * t184 + t168;
t79 = (t130 * t83 + t135 * t84) * t139;
t150 = t79 * l22 + t161;
t43 = -t59 * t130 + t60 * t135;
t26 = t43 * t129 + t44 * t134;
t149 = t26 * a6 + t153;
t52 = (t130 * t64 - t135 * t63) * t136;
t148 = t52 * a5 + t152;
t49 = (-t63 * t84 + t64 * t83) * t180;
t147 = t49 * a11 + t151;
t51 = (t130 * t63 + t135 * t64) * t136;
t38 = t51 * t129 + t52 * t134;
t146 = t38 * a6 + t148;
t141 = 0.1e1 / l20;
t133 = cos(qJs5);
t128 = sin(qJs5);
t119 = qJs4 - qJs3 + delta18s;
t118 = -qJs4 + t122;
t113 = cos(t119);
t112 = cos(t118);
t111 = sin(t119);
t110 = sin(t118);
t104 = -t132 * t125 - t127 * t179;
t103 = -t125 * t179 + t132 * t127;
t102 = -t137 * t125 + t127 * t182;
t101 = t125 * t182 + t137 * t127;
t99 = t110 * l14 - t113 * l17 + t111 * l18;
t98 = -t112 * l14 - t111 * l17 - t113 * l18 - l12;
t95 = (t98 ^ 2 + t99 ^ 2) ^ (-0.1e1 / 0.2e1);
t92 = (t110 * t98 + t112 * t99) * t95;
t91 = (-t110 * t99 + t112 * t98) * t95;
t86 = (t90 * t116 + t166 * t117) * t183;
t85 = -t144 * t116 * t158 + t90 * t117 * t183;
t74 = l22 * t157 - t173;
t70 = (-t150 * t157 + t74 * t79) * t144;
t69 = (-t79 * t150 - t74 * t157) * t144;
t62 = t63 * t125 - t64 * t127;
t61 = t64 * t125 + t63 * t127;
t48 = (t63 * t83 + t64 * t84) * t180;
t46 = t61 * l4 + t76;
t45 = -t62 * l4 - t160;
t37 = -t52 * t129 + t51 * t134;
t34 = (-t59 * t83 + t60 * t84) * t139;
t32 = (-t57 * t83 + t58 * t84) * t139;
t29 = t48 * t69 - t49 * t70;
t28 = t48 * t70 + t49 * t69;
t27 = t28 * a12 + t147;
t25 = -t44 * t129 + t43 * t134;
t23 = -t42 * t129 + t41 * t134;
t20 = t37 * t111 + t38 * t113;
t19 = -t38 * t111 + t37 * t113;
t18 = (-t45 * t62 + t46 * t61) * t141;
t17 = (t45 * t61 + t46 * t62) * t141;
t16 = t34 * t69 - t35 * t70;
t15 = t34 * t70 + t35 * t69;
t14 = t32 * t69 - t33 * t70;
t13 = t32 * t70 + t33 * t69;
t12 = t28 * t86 - t29 * t85;
t11 = t28 * t85 + t29 * t86;
t10 = t15 * a12 + t154;
t9 = t13 * a12 + t165;
t8 = t25 * t111 + t26 * t113;
t7 = -t26 * t111 + t25 * t113;
t6 = t23 * t111 + t24 * t113;
t5 = -t24 * t111 + t23 * t113;
t4 = t15 * t86 - t16 * t85;
t3 = t15 * t85 + t16 * t86;
t2 = t13 * t86 - t14 * t85;
t1 = t13 * t85 + t14 * t86;
T_ges = [1 0 0 rxs_base; 0 1 0 rys_base; 0 0 1 rzs_base; 0 0 0 1; t132 t137 0 rxs_base; -t137 t132 0 rys_base; 0 0 1 t123; 0 0 0 1; -t182 -t114 t137 t176; t179 t178 t132 t177; -t136 t131 0 t123; 0 0 0 1; t57 t58 t114 t171; t59 t60 -t178 t163; -t184 t136 * t64 -t131 t168; 0 0 0 1; t42 t41 t114 t169; t44 t43 -t178 t155; t52 t51 -t131 t152; 0 0 0 1; t24 t23 t114 t164; t26 t25 -t178 t153; t38 t37 -t131 t148; 0 0 0 1; t5 -t6 t114 t159; t7 -t8 -t178 t149; t19 -t20 -t131 t146; 0 0 0 1; t128 * t114 + t5 * t133 t133 * t114 - t5 * t128 t6 t6 * d7 + t159; -t128 * t178 + t7 * t133 -t7 * t128 - t133 * t178 t8 t8 * d7 + t149; -t131 * t128 + t19 * t133 -t19 * t128 - t131 * t133 t20 t20 * d7 + t146; 0 0 0 1; t101 t102 t114 t172; t103 t104 -t178 t162; t181 t136 * t127 -t131 t167; 0 0 0 1; t101 * t18 - t102 * t17 t101 * t17 + t102 * t18 t114 t101 * a9 + t172; t103 * t18 - t104 * t17 t103 * t17 + t104 * t18 -t178 t103 * a9 + t162; (t125 * t18 - t127 * t17) * t136 (t125 * t17 + t127 * t18) * t136 -t131 a9 * t181 + t167; 0 0 0 1; t33 t32 t114 t170; t35 t34 -t178 t156; t49 t48 -t131 t151; 0 0 0 1; t13 t14 t114 t165; t15 t16 -t178 t154; t28 t29 -t131 t147; 0 0 0 1; t2 t1 t114 t9; t4 t3 -t178 t10; t12 t11 -t131 t27; 0 0 0 1; -t1 * t92 + t2 * t91 t1 * t91 + t2 * t92 t114 t9; -t3 * t92 + t4 * t91 t3 * t91 + t4 * t92 -t178 t10; -t11 * t92 + t12 * t91 t11 * t91 + t12 * t92 -t131 t27; 0 0 0 1;];