% Direkte Kinematik für KAS5m3 (MDH-Konvention, symbolische Berechnung)
% 
% Teste Kinematik-Berechnung mit Berücksichtigung der kinematischen
% Zwangsbedingungen und MDH-Konvention aus Maple.
% 
% Input:
% q [6x1]
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

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-12
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_ges = KAS5m3_fkine_mdh_sym_varpar(q, a_mdh, d_mdh, l_const, w_const)
%% Init
% # codegen deaktiviert
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [6 1]), ...
  'Gelenkwinkel q gefordert als [6x1] double');
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
qJs6 = q(6);

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

rxs_base = 0;
rys_base = 0;
rzs_base = 0;
%% Berechnung mit exportiertem Code
% codegen/KAS5_sym_codegen_floatbase_mdh_kinematik_gesamt.mw
% codeexport/KAS5m3_fkine_matlab.m

t138 = 0.1e1 / l22;
t184 = l21 * t138;
t173 = cos(delta9s) * t184;
t174 = sin(delta9s) * t184;
t128 = sin(qJs3);
t134 = cos(qJs3);
t137 = -l5 + l6;
t107 = t134 * t137 - l11;
t119 = delta17s + qJs3;
t115 = cos(t119);
t160 = t115 * l14 + t107;
t105 = 0.2e1 * t160;
t114 = sin(t119);
t172 = t114 * l14 - t128 * t137;
t106 = 0.2e1 * t172;
t142 = l13 ^ 2;
t94 = t128 ^ 2 * t137 ^ 2 - l22 ^ 2 + t107 ^ 2 + t142 + (t105 * t115 + t106 * t114 + (-t114 ^ 2 - t115 ^ 2) * l14) * l14;
t97 = t105 ^ 2 + t106 ^ 2;
t93 = sqrt(t142 * t97 - t94 ^ 2);
t165 = t105 * t94 - t106 * t93;
t96 = 0.1e1 / t97;
t157 = t165 * t96;
t87 = t157 - t160;
t90 = t105 * t93 + t94 * t106;
t88 = t90 * t96 - t172;
t83 = t128 * t87 + t134 * t88;
t84 = -t128 * t88 + t134 * t87;
t76 = t83 * t173 + t84 * t174;
t135 = cos(qJs2);
t122 = sin(delta8s);
t124 = cos(delta8s);
t144 = 0.1e1 / l4;
t78 = t84 * t173 - t83 * t174;
t159 = -l6 - t78;
t139 = l20 ^ 2;
t75 = 0.2e1 * t159;
t77 = 0.2e1 * t76;
t68 = -l4 ^ 2 + l6 ^ 2 + t139 + (-t75 - t78) * t78 + (t77 - t76) * t76;
t73 = t75 ^ 2 + t77 ^ 2;
t67 = sqrt(t139 * t73 - t68 ^ 2);
t72 = 0.1e1 / t73;
t65 = -(t77 * t67 + t75 * t68) * t72 + t159;
t66 = -(t75 * t67 - t68 * t77) * t72 - t76;
t63 = (-t122 * t65 + t124 * t66) * t144;
t183 = t135 * t63;
t143 = 0.1e1 / l13;
t182 = t143 * t96;
t129 = sin(qJs2);
t130 = sin(qJs1);
t181 = t130 * t129;
t112 = t130 * t135;
t180 = t135 * t122;
t179 = t135 * t138;
t136 = cos(qJs1);
t178 = t136 * t129;
t177 = t136 * t135;
t120 = d1 + rzs_base;
t176 = t130 * d2 + rys_base;
t175 = t136 * d2 + rxs_base;
t171 = d8 * t112 + t175;
t170 = d3 * t112 + t175;
t64 = (t122 * t66 + t124 * t65) * t144;
t57 = t136 * t64 - t63 * t181;
t169 = t57 * a10 + t170;
t168 = t57 * a4 + t170;
t167 = -t129 * d3 + t120;
t166 = -t129 * d8 + t120;
t58 = t136 * t63 + t64 * t181;
t33 = (t57 * t84 + t58 * t83) * t138;
t164 = t33 * a11 + t169;
t42 = t58 * t128 + t57 * t134;
t163 = t42 * a5 + t168;
t162 = -d3 * t177 + t176;
t161 = -d8 * t177 + t176;
t127 = sin(qJs4);
t133 = cos(qJs4);
t41 = -t57 * t128 + t58 * t134;
t24 = t41 * t127 + t42 * t133;
t158 = t24 * a6 + t163;
t156 = (t128 * t84 - t134 * t83) * t138;
t59 = t130 * t64 + t63 * t178;
t155 = t59 * a10 + t162;
t154 = t59 * a4 + t162;
t60 = t130 * t63 - t64 * t178;
t35 = (t59 * t84 + t60 * t83) * t138;
t153 = t35 * a11 + t155;
t44 = t60 * t128 + t59 * t134;
t152 = t44 * a5 + t154;
t151 = -a4 * t183 + t167;
t150 = -a10 * t183 + t167;
t79 = (t128 * t83 + t134 * t84) * t138;
t149 = t79 * l22 + t160;
t43 = -t59 * t128 + t60 * t134;
t26 = t43 * t127 + t44 * t133;
t148 = t26 * a6 + t152;
t52 = (t128 * t64 - t134 * t63) * t135;
t147 = t52 * a5 + t151;
t49 = (-t63 * t84 + t64 * t83) * t179;
t146 = t49 * a11 + t150;
t51 = (t128 * t63 + t134 * t64) * t135;
t38 = t51 * t127 + t52 * t133;
t145 = t38 * a6 + t147;
t140 = 0.1e1 / l20;
t132 = cos(qJs5);
t131 = cos(qJs6);
t126 = sin(qJs5);
t125 = sin(qJs6);
t116 = -qJs4 + t119;
t111 = cos(t116);
t110 = sin(t116);
t104 = -t130 * t122 - t124 * t178;
t103 = -t122 * t178 + t130 * t124;
t102 = -t136 * t122 + t124 * t181;
t101 = t122 * t181 + t136 * t124;
t99 = t110 * l14 - t132 * l17 + t126 * l18;
t98 = -t111 * l14 - t126 * l17 - t132 * l18 - l12;
t95 = (t98 ^ 2 + t99 ^ 2) ^ (-0.1e1 / 0.2e1);
t92 = (t110 * t98 + t111 * t99) * t95;
t91 = (-t110 * t99 + t111 * t98) * t95;
t86 = (t90 * t114 + t165 * t115) * t182;
t85 = -t143 * t114 * t157 + t90 * t115 * t182;
t74 = l22 * t156 - t172;
t70 = (-t149 * t156 + t74 * t79) * t143;
t69 = (-t79 * t149 - t74 * t156) * t143;
t62 = t63 * t122 - t64 * t124;
t61 = t64 * t122 + t63 * t124;
t48 = (t63 * t83 + t64 * t84) * t179;
t46 = t61 * l4 + t76;
t45 = -t62 * l4 - t159;
t37 = -t52 * t127 + t51 * t133;
t34 = (-t59 * t83 + t60 * t84) * t138;
t32 = (-t57 * t83 + t58 * t84) * t138;
t29 = t48 * t69 - t49 * t70;
t28 = t48 * t70 + t49 * t69;
t27 = t28 * a12 + t146;
t25 = -t44 * t127 + t43 * t133;
t23 = -t42 * t127 + t41 * t133;
t20 = t37 * t126 + t38 * t132;
t19 = -t38 * t126 + t37 * t132;
t18 = (-t45 * t62 + t46 * t61) * t140;
t17 = (t45 * t61 + t46 * t62) * t140;
t16 = t34 * t69 - t35 * t70;
t15 = t34 * t70 + t35 * t69;
t14 = t32 * t69 - t33 * t70;
t13 = t32 * t70 + t33 * t69;
t12 = t28 * t86 - t29 * t85;
t11 = t28 * t85 + t29 * t86;
t10 = t15 * a12 + t153;
t9 = t13 * a12 + t164;
t8 = t25 * t126 + t26 * t132;
t7 = -t26 * t126 + t25 * t132;
t6 = t23 * t126 + t24 * t132;
t5 = -t24 * t126 + t23 * t132;
t4 = t15 * t86 - t16 * t85;
t3 = t15 * t85 + t16 * t86;
t2 = t13 * t86 - t14 * t85;
t1 = t13 * t85 + t14 * t86;
T_ges= [1 0 0 rxs_base; 0 1 0 rys_base; 0 0 1 rzs_base; 0 0 0 1; t130 t136 0 rxs_base; -t136 t130 0 rys_base; 0 0 1 t120; 0 0 0 1; -t181 -t112 t136 t175; t178 t177 t130 t176; -t135 t129 0 t120; 0 0 0 1; t57 t58 t112 t170; t59 t60 -t177 t162; -t183 t135 * t64 -t129 t167; 0 0 0 1; t42 t41 t112 t168; t44 t43 -t177 t154; t52 t51 -t129 t151; 0 0 0 1; t24 t23 t112 t163; t26 t25 -t177 t152; t38 t37 -t129 t147; 0 0 0 1; t5 -t6 t112 t158; t7 -t8 -t177 t148; t19 -t20 -t129 t145; 0 0 0 1; t125 * t112 + t5 * t131 t131 * t112 - t5 * t125 t6 t6 * d7 + t158; -t125 * t177 + t7 * t131 -t7 * t125 - t131 * t177 t8 t8 * d7 + t148; -t129 * t125 + t19 * t131 -t19 * t125 - t129 * t131 t20 t20 * d7 + t145; 0 0 0 1; t101 t102 t112 t171; t103 t104 -t177 t161; t180 t135 * t124 -t129 t166; 0 0 0 1; t101 * t18 - t102 * t17 t101 * t17 + t102 * t18 t112 t101 * a9 + t171; t103 * t18 - t104 * t17 t103 * t17 + t104 * t18 -t177 t103 * a9 + t161; (t122 * t18 - t124 * t17) * t135 (t122 * t17 + t124 * t18) * t135 -t129 a9 * t180 + t166; 0 0 0 1; t33 t32 t112 t169; t35 t34 -t177 t155; t49 t48 -t129 t150; 0 0 0 1; t13 t14 t112 t164; t15 t16 -t177 t153; t28 t29 -t129 t146; 0 0 0 1; t2 t1 t112 t9; t4 t3 -t177 t10; t12 t11 -t129 t27; 0 0 0 1; -t1 * t92 + t2 * t91 t1 * t91 + t2 * t92 t112 t9; -t3 * t92 + t4 * t91 t3 * t91 + t4 * t92 -t177 t10; -t11 * t92 + t12 * t91 t11 * t91 + t12 * t92 -t129 t27; 0 0 0 1;];