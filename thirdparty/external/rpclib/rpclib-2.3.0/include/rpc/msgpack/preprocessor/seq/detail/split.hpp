# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_SEQ_DETAIL_SPLIT_HPP
# define MSGPACK_PREPROCESSOR_SEQ_DETAIL_SPLIT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_SEQ_SPLIT */
#
# define MSGPACK_PP_SEQ_SPLIT(n, seq) MSGPACK_PP_SEQ_SPLIT_D(n, seq)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_SEQ_SPLIT_D(n, seq) (MSGPACK_PP_SEQ_SPLIT_ ## n seq)
# else
#    define MSGPACK_PP_SEQ_SPLIT_D(n, seq) (MSGPACK_PP_SEQ_SPLIT_ ## n ## seq)
# endif
#
# define MSGPACK_PP_SEQ_SPLIT_1(x) (x),
# define MSGPACK_PP_SEQ_SPLIT_2(x) (x) MSGPACK_PP_SEQ_SPLIT_1
# define MSGPACK_PP_SEQ_SPLIT_3(x) (x) MSGPACK_PP_SEQ_SPLIT_2
# define MSGPACK_PP_SEQ_SPLIT_4(x) (x) MSGPACK_PP_SEQ_SPLIT_3
# define MSGPACK_PP_SEQ_SPLIT_5(x) (x) MSGPACK_PP_SEQ_SPLIT_4
# define MSGPACK_PP_SEQ_SPLIT_6(x) (x) MSGPACK_PP_SEQ_SPLIT_5
# define MSGPACK_PP_SEQ_SPLIT_7(x) (x) MSGPACK_PP_SEQ_SPLIT_6
# define MSGPACK_PP_SEQ_SPLIT_8(x) (x) MSGPACK_PP_SEQ_SPLIT_7
# define MSGPACK_PP_SEQ_SPLIT_9(x) (x) MSGPACK_PP_SEQ_SPLIT_8
# define MSGPACK_PP_SEQ_SPLIT_10(x) (x) MSGPACK_PP_SEQ_SPLIT_9
# define MSGPACK_PP_SEQ_SPLIT_11(x) (x) MSGPACK_PP_SEQ_SPLIT_10
# define MSGPACK_PP_SEQ_SPLIT_12(x) (x) MSGPACK_PP_SEQ_SPLIT_11
# define MSGPACK_PP_SEQ_SPLIT_13(x) (x) MSGPACK_PP_SEQ_SPLIT_12
# define MSGPACK_PP_SEQ_SPLIT_14(x) (x) MSGPACK_PP_SEQ_SPLIT_13
# define MSGPACK_PP_SEQ_SPLIT_15(x) (x) MSGPACK_PP_SEQ_SPLIT_14
# define MSGPACK_PP_SEQ_SPLIT_16(x) (x) MSGPACK_PP_SEQ_SPLIT_15
# define MSGPACK_PP_SEQ_SPLIT_17(x) (x) MSGPACK_PP_SEQ_SPLIT_16
# define MSGPACK_PP_SEQ_SPLIT_18(x) (x) MSGPACK_PP_SEQ_SPLIT_17
# define MSGPACK_PP_SEQ_SPLIT_19(x) (x) MSGPACK_PP_SEQ_SPLIT_18
# define MSGPACK_PP_SEQ_SPLIT_20(x) (x) MSGPACK_PP_SEQ_SPLIT_19
# define MSGPACK_PP_SEQ_SPLIT_21(x) (x) MSGPACK_PP_SEQ_SPLIT_20
# define MSGPACK_PP_SEQ_SPLIT_22(x) (x) MSGPACK_PP_SEQ_SPLIT_21
# define MSGPACK_PP_SEQ_SPLIT_23(x) (x) MSGPACK_PP_SEQ_SPLIT_22
# define MSGPACK_PP_SEQ_SPLIT_24(x) (x) MSGPACK_PP_SEQ_SPLIT_23
# define MSGPACK_PP_SEQ_SPLIT_25(x) (x) MSGPACK_PP_SEQ_SPLIT_24
# define MSGPACK_PP_SEQ_SPLIT_26(x) (x) MSGPACK_PP_SEQ_SPLIT_25
# define MSGPACK_PP_SEQ_SPLIT_27(x) (x) MSGPACK_PP_SEQ_SPLIT_26
# define MSGPACK_PP_SEQ_SPLIT_28(x) (x) MSGPACK_PP_SEQ_SPLIT_27
# define MSGPACK_PP_SEQ_SPLIT_29(x) (x) MSGPACK_PP_SEQ_SPLIT_28
# define MSGPACK_PP_SEQ_SPLIT_30(x) (x) MSGPACK_PP_SEQ_SPLIT_29
# define MSGPACK_PP_SEQ_SPLIT_31(x) (x) MSGPACK_PP_SEQ_SPLIT_30
# define MSGPACK_PP_SEQ_SPLIT_32(x) (x) MSGPACK_PP_SEQ_SPLIT_31
# define MSGPACK_PP_SEQ_SPLIT_33(x) (x) MSGPACK_PP_SEQ_SPLIT_32
# define MSGPACK_PP_SEQ_SPLIT_34(x) (x) MSGPACK_PP_SEQ_SPLIT_33
# define MSGPACK_PP_SEQ_SPLIT_35(x) (x) MSGPACK_PP_SEQ_SPLIT_34
# define MSGPACK_PP_SEQ_SPLIT_36(x) (x) MSGPACK_PP_SEQ_SPLIT_35
# define MSGPACK_PP_SEQ_SPLIT_37(x) (x) MSGPACK_PP_SEQ_SPLIT_36
# define MSGPACK_PP_SEQ_SPLIT_38(x) (x) MSGPACK_PP_SEQ_SPLIT_37
# define MSGPACK_PP_SEQ_SPLIT_39(x) (x) MSGPACK_PP_SEQ_SPLIT_38
# define MSGPACK_PP_SEQ_SPLIT_40(x) (x) MSGPACK_PP_SEQ_SPLIT_39
# define MSGPACK_PP_SEQ_SPLIT_41(x) (x) MSGPACK_PP_SEQ_SPLIT_40
# define MSGPACK_PP_SEQ_SPLIT_42(x) (x) MSGPACK_PP_SEQ_SPLIT_41
# define MSGPACK_PP_SEQ_SPLIT_43(x) (x) MSGPACK_PP_SEQ_SPLIT_42
# define MSGPACK_PP_SEQ_SPLIT_44(x) (x) MSGPACK_PP_SEQ_SPLIT_43
# define MSGPACK_PP_SEQ_SPLIT_45(x) (x) MSGPACK_PP_SEQ_SPLIT_44
# define MSGPACK_PP_SEQ_SPLIT_46(x) (x) MSGPACK_PP_SEQ_SPLIT_45
# define MSGPACK_PP_SEQ_SPLIT_47(x) (x) MSGPACK_PP_SEQ_SPLIT_46
# define MSGPACK_PP_SEQ_SPLIT_48(x) (x) MSGPACK_PP_SEQ_SPLIT_47
# define MSGPACK_PP_SEQ_SPLIT_49(x) (x) MSGPACK_PP_SEQ_SPLIT_48
# define MSGPACK_PP_SEQ_SPLIT_50(x) (x) MSGPACK_PP_SEQ_SPLIT_49
# define MSGPACK_PP_SEQ_SPLIT_51(x) (x) MSGPACK_PP_SEQ_SPLIT_50
# define MSGPACK_PP_SEQ_SPLIT_52(x) (x) MSGPACK_PP_SEQ_SPLIT_51
# define MSGPACK_PP_SEQ_SPLIT_53(x) (x) MSGPACK_PP_SEQ_SPLIT_52
# define MSGPACK_PP_SEQ_SPLIT_54(x) (x) MSGPACK_PP_SEQ_SPLIT_53
# define MSGPACK_PP_SEQ_SPLIT_55(x) (x) MSGPACK_PP_SEQ_SPLIT_54
# define MSGPACK_PP_SEQ_SPLIT_56(x) (x) MSGPACK_PP_SEQ_SPLIT_55
# define MSGPACK_PP_SEQ_SPLIT_57(x) (x) MSGPACK_PP_SEQ_SPLIT_56
# define MSGPACK_PP_SEQ_SPLIT_58(x) (x) MSGPACK_PP_SEQ_SPLIT_57
# define MSGPACK_PP_SEQ_SPLIT_59(x) (x) MSGPACK_PP_SEQ_SPLIT_58
# define MSGPACK_PP_SEQ_SPLIT_60(x) (x) MSGPACK_PP_SEQ_SPLIT_59
# define MSGPACK_PP_SEQ_SPLIT_61(x) (x) MSGPACK_PP_SEQ_SPLIT_60
# define MSGPACK_PP_SEQ_SPLIT_62(x) (x) MSGPACK_PP_SEQ_SPLIT_61
# define MSGPACK_PP_SEQ_SPLIT_63(x) (x) MSGPACK_PP_SEQ_SPLIT_62
# define MSGPACK_PP_SEQ_SPLIT_64(x) (x) MSGPACK_PP_SEQ_SPLIT_63
# define MSGPACK_PP_SEQ_SPLIT_65(x) (x) MSGPACK_PP_SEQ_SPLIT_64
# define MSGPACK_PP_SEQ_SPLIT_66(x) (x) MSGPACK_PP_SEQ_SPLIT_65
# define MSGPACK_PP_SEQ_SPLIT_67(x) (x) MSGPACK_PP_SEQ_SPLIT_66
# define MSGPACK_PP_SEQ_SPLIT_68(x) (x) MSGPACK_PP_SEQ_SPLIT_67
# define MSGPACK_PP_SEQ_SPLIT_69(x) (x) MSGPACK_PP_SEQ_SPLIT_68
# define MSGPACK_PP_SEQ_SPLIT_70(x) (x) MSGPACK_PP_SEQ_SPLIT_69
# define MSGPACK_PP_SEQ_SPLIT_71(x) (x) MSGPACK_PP_SEQ_SPLIT_70
# define MSGPACK_PP_SEQ_SPLIT_72(x) (x) MSGPACK_PP_SEQ_SPLIT_71
# define MSGPACK_PP_SEQ_SPLIT_73(x) (x) MSGPACK_PP_SEQ_SPLIT_72
# define MSGPACK_PP_SEQ_SPLIT_74(x) (x) MSGPACK_PP_SEQ_SPLIT_73
# define MSGPACK_PP_SEQ_SPLIT_75(x) (x) MSGPACK_PP_SEQ_SPLIT_74
# define MSGPACK_PP_SEQ_SPLIT_76(x) (x) MSGPACK_PP_SEQ_SPLIT_75
# define MSGPACK_PP_SEQ_SPLIT_77(x) (x) MSGPACK_PP_SEQ_SPLIT_76
# define MSGPACK_PP_SEQ_SPLIT_78(x) (x) MSGPACK_PP_SEQ_SPLIT_77
# define MSGPACK_PP_SEQ_SPLIT_79(x) (x) MSGPACK_PP_SEQ_SPLIT_78
# define MSGPACK_PP_SEQ_SPLIT_80(x) (x) MSGPACK_PP_SEQ_SPLIT_79
# define MSGPACK_PP_SEQ_SPLIT_81(x) (x) MSGPACK_PP_SEQ_SPLIT_80
# define MSGPACK_PP_SEQ_SPLIT_82(x) (x) MSGPACK_PP_SEQ_SPLIT_81
# define MSGPACK_PP_SEQ_SPLIT_83(x) (x) MSGPACK_PP_SEQ_SPLIT_82
# define MSGPACK_PP_SEQ_SPLIT_84(x) (x) MSGPACK_PP_SEQ_SPLIT_83
# define MSGPACK_PP_SEQ_SPLIT_85(x) (x) MSGPACK_PP_SEQ_SPLIT_84
# define MSGPACK_PP_SEQ_SPLIT_86(x) (x) MSGPACK_PP_SEQ_SPLIT_85
# define MSGPACK_PP_SEQ_SPLIT_87(x) (x) MSGPACK_PP_SEQ_SPLIT_86
# define MSGPACK_PP_SEQ_SPLIT_88(x) (x) MSGPACK_PP_SEQ_SPLIT_87
# define MSGPACK_PP_SEQ_SPLIT_89(x) (x) MSGPACK_PP_SEQ_SPLIT_88
# define MSGPACK_PP_SEQ_SPLIT_90(x) (x) MSGPACK_PP_SEQ_SPLIT_89
# define MSGPACK_PP_SEQ_SPLIT_91(x) (x) MSGPACK_PP_SEQ_SPLIT_90
# define MSGPACK_PP_SEQ_SPLIT_92(x) (x) MSGPACK_PP_SEQ_SPLIT_91
# define MSGPACK_PP_SEQ_SPLIT_93(x) (x) MSGPACK_PP_SEQ_SPLIT_92
# define MSGPACK_PP_SEQ_SPLIT_94(x) (x) MSGPACK_PP_SEQ_SPLIT_93
# define MSGPACK_PP_SEQ_SPLIT_95(x) (x) MSGPACK_PP_SEQ_SPLIT_94
# define MSGPACK_PP_SEQ_SPLIT_96(x) (x) MSGPACK_PP_SEQ_SPLIT_95
# define MSGPACK_PP_SEQ_SPLIT_97(x) (x) MSGPACK_PP_SEQ_SPLIT_96
# define MSGPACK_PP_SEQ_SPLIT_98(x) (x) MSGPACK_PP_SEQ_SPLIT_97
# define MSGPACK_PP_SEQ_SPLIT_99(x) (x) MSGPACK_PP_SEQ_SPLIT_98
# define MSGPACK_PP_SEQ_SPLIT_100(x) (x) MSGPACK_PP_SEQ_SPLIT_99
# define MSGPACK_PP_SEQ_SPLIT_101(x) (x) MSGPACK_PP_SEQ_SPLIT_100
# define MSGPACK_PP_SEQ_SPLIT_102(x) (x) MSGPACK_PP_SEQ_SPLIT_101
# define MSGPACK_PP_SEQ_SPLIT_103(x) (x) MSGPACK_PP_SEQ_SPLIT_102
# define MSGPACK_PP_SEQ_SPLIT_104(x) (x) MSGPACK_PP_SEQ_SPLIT_103
# define MSGPACK_PP_SEQ_SPLIT_105(x) (x) MSGPACK_PP_SEQ_SPLIT_104
# define MSGPACK_PP_SEQ_SPLIT_106(x) (x) MSGPACK_PP_SEQ_SPLIT_105
# define MSGPACK_PP_SEQ_SPLIT_107(x) (x) MSGPACK_PP_SEQ_SPLIT_106
# define MSGPACK_PP_SEQ_SPLIT_108(x) (x) MSGPACK_PP_SEQ_SPLIT_107
# define MSGPACK_PP_SEQ_SPLIT_109(x) (x) MSGPACK_PP_SEQ_SPLIT_108
# define MSGPACK_PP_SEQ_SPLIT_110(x) (x) MSGPACK_PP_SEQ_SPLIT_109
# define MSGPACK_PP_SEQ_SPLIT_111(x) (x) MSGPACK_PP_SEQ_SPLIT_110
# define MSGPACK_PP_SEQ_SPLIT_112(x) (x) MSGPACK_PP_SEQ_SPLIT_111
# define MSGPACK_PP_SEQ_SPLIT_113(x) (x) MSGPACK_PP_SEQ_SPLIT_112
# define MSGPACK_PP_SEQ_SPLIT_114(x) (x) MSGPACK_PP_SEQ_SPLIT_113
# define MSGPACK_PP_SEQ_SPLIT_115(x) (x) MSGPACK_PP_SEQ_SPLIT_114
# define MSGPACK_PP_SEQ_SPLIT_116(x) (x) MSGPACK_PP_SEQ_SPLIT_115
# define MSGPACK_PP_SEQ_SPLIT_117(x) (x) MSGPACK_PP_SEQ_SPLIT_116
# define MSGPACK_PP_SEQ_SPLIT_118(x) (x) MSGPACK_PP_SEQ_SPLIT_117
# define MSGPACK_PP_SEQ_SPLIT_119(x) (x) MSGPACK_PP_SEQ_SPLIT_118
# define MSGPACK_PP_SEQ_SPLIT_120(x) (x) MSGPACK_PP_SEQ_SPLIT_119
# define MSGPACK_PP_SEQ_SPLIT_121(x) (x) MSGPACK_PP_SEQ_SPLIT_120
# define MSGPACK_PP_SEQ_SPLIT_122(x) (x) MSGPACK_PP_SEQ_SPLIT_121
# define MSGPACK_PP_SEQ_SPLIT_123(x) (x) MSGPACK_PP_SEQ_SPLIT_122
# define MSGPACK_PP_SEQ_SPLIT_124(x) (x) MSGPACK_PP_SEQ_SPLIT_123
# define MSGPACK_PP_SEQ_SPLIT_125(x) (x) MSGPACK_PP_SEQ_SPLIT_124
# define MSGPACK_PP_SEQ_SPLIT_126(x) (x) MSGPACK_PP_SEQ_SPLIT_125
# define MSGPACK_PP_SEQ_SPLIT_127(x) (x) MSGPACK_PP_SEQ_SPLIT_126
# define MSGPACK_PP_SEQ_SPLIT_128(x) (x) MSGPACK_PP_SEQ_SPLIT_127
# define MSGPACK_PP_SEQ_SPLIT_129(x) (x) MSGPACK_PP_SEQ_SPLIT_128
# define MSGPACK_PP_SEQ_SPLIT_130(x) (x) MSGPACK_PP_SEQ_SPLIT_129
# define MSGPACK_PP_SEQ_SPLIT_131(x) (x) MSGPACK_PP_SEQ_SPLIT_130
# define MSGPACK_PP_SEQ_SPLIT_132(x) (x) MSGPACK_PP_SEQ_SPLIT_131
# define MSGPACK_PP_SEQ_SPLIT_133(x) (x) MSGPACK_PP_SEQ_SPLIT_132
# define MSGPACK_PP_SEQ_SPLIT_134(x) (x) MSGPACK_PP_SEQ_SPLIT_133
# define MSGPACK_PP_SEQ_SPLIT_135(x) (x) MSGPACK_PP_SEQ_SPLIT_134
# define MSGPACK_PP_SEQ_SPLIT_136(x) (x) MSGPACK_PP_SEQ_SPLIT_135
# define MSGPACK_PP_SEQ_SPLIT_137(x) (x) MSGPACK_PP_SEQ_SPLIT_136
# define MSGPACK_PP_SEQ_SPLIT_138(x) (x) MSGPACK_PP_SEQ_SPLIT_137
# define MSGPACK_PP_SEQ_SPLIT_139(x) (x) MSGPACK_PP_SEQ_SPLIT_138
# define MSGPACK_PP_SEQ_SPLIT_140(x) (x) MSGPACK_PP_SEQ_SPLIT_139
# define MSGPACK_PP_SEQ_SPLIT_141(x) (x) MSGPACK_PP_SEQ_SPLIT_140
# define MSGPACK_PP_SEQ_SPLIT_142(x) (x) MSGPACK_PP_SEQ_SPLIT_141
# define MSGPACK_PP_SEQ_SPLIT_143(x) (x) MSGPACK_PP_SEQ_SPLIT_142
# define MSGPACK_PP_SEQ_SPLIT_144(x) (x) MSGPACK_PP_SEQ_SPLIT_143
# define MSGPACK_PP_SEQ_SPLIT_145(x) (x) MSGPACK_PP_SEQ_SPLIT_144
# define MSGPACK_PP_SEQ_SPLIT_146(x) (x) MSGPACK_PP_SEQ_SPLIT_145
# define MSGPACK_PP_SEQ_SPLIT_147(x) (x) MSGPACK_PP_SEQ_SPLIT_146
# define MSGPACK_PP_SEQ_SPLIT_148(x) (x) MSGPACK_PP_SEQ_SPLIT_147
# define MSGPACK_PP_SEQ_SPLIT_149(x) (x) MSGPACK_PP_SEQ_SPLIT_148
# define MSGPACK_PP_SEQ_SPLIT_150(x) (x) MSGPACK_PP_SEQ_SPLIT_149
# define MSGPACK_PP_SEQ_SPLIT_151(x) (x) MSGPACK_PP_SEQ_SPLIT_150
# define MSGPACK_PP_SEQ_SPLIT_152(x) (x) MSGPACK_PP_SEQ_SPLIT_151
# define MSGPACK_PP_SEQ_SPLIT_153(x) (x) MSGPACK_PP_SEQ_SPLIT_152
# define MSGPACK_PP_SEQ_SPLIT_154(x) (x) MSGPACK_PP_SEQ_SPLIT_153
# define MSGPACK_PP_SEQ_SPLIT_155(x) (x) MSGPACK_PP_SEQ_SPLIT_154
# define MSGPACK_PP_SEQ_SPLIT_156(x) (x) MSGPACK_PP_SEQ_SPLIT_155
# define MSGPACK_PP_SEQ_SPLIT_157(x) (x) MSGPACK_PP_SEQ_SPLIT_156
# define MSGPACK_PP_SEQ_SPLIT_158(x) (x) MSGPACK_PP_SEQ_SPLIT_157
# define MSGPACK_PP_SEQ_SPLIT_159(x) (x) MSGPACK_PP_SEQ_SPLIT_158
# define MSGPACK_PP_SEQ_SPLIT_160(x) (x) MSGPACK_PP_SEQ_SPLIT_159
# define MSGPACK_PP_SEQ_SPLIT_161(x) (x) MSGPACK_PP_SEQ_SPLIT_160
# define MSGPACK_PP_SEQ_SPLIT_162(x) (x) MSGPACK_PP_SEQ_SPLIT_161
# define MSGPACK_PP_SEQ_SPLIT_163(x) (x) MSGPACK_PP_SEQ_SPLIT_162
# define MSGPACK_PP_SEQ_SPLIT_164(x) (x) MSGPACK_PP_SEQ_SPLIT_163
# define MSGPACK_PP_SEQ_SPLIT_165(x) (x) MSGPACK_PP_SEQ_SPLIT_164
# define MSGPACK_PP_SEQ_SPLIT_166(x) (x) MSGPACK_PP_SEQ_SPLIT_165
# define MSGPACK_PP_SEQ_SPLIT_167(x) (x) MSGPACK_PP_SEQ_SPLIT_166
# define MSGPACK_PP_SEQ_SPLIT_168(x) (x) MSGPACK_PP_SEQ_SPLIT_167
# define MSGPACK_PP_SEQ_SPLIT_169(x) (x) MSGPACK_PP_SEQ_SPLIT_168
# define MSGPACK_PP_SEQ_SPLIT_170(x) (x) MSGPACK_PP_SEQ_SPLIT_169
# define MSGPACK_PP_SEQ_SPLIT_171(x) (x) MSGPACK_PP_SEQ_SPLIT_170
# define MSGPACK_PP_SEQ_SPLIT_172(x) (x) MSGPACK_PP_SEQ_SPLIT_171
# define MSGPACK_PP_SEQ_SPLIT_173(x) (x) MSGPACK_PP_SEQ_SPLIT_172
# define MSGPACK_PP_SEQ_SPLIT_174(x) (x) MSGPACK_PP_SEQ_SPLIT_173
# define MSGPACK_PP_SEQ_SPLIT_175(x) (x) MSGPACK_PP_SEQ_SPLIT_174
# define MSGPACK_PP_SEQ_SPLIT_176(x) (x) MSGPACK_PP_SEQ_SPLIT_175
# define MSGPACK_PP_SEQ_SPLIT_177(x) (x) MSGPACK_PP_SEQ_SPLIT_176
# define MSGPACK_PP_SEQ_SPLIT_178(x) (x) MSGPACK_PP_SEQ_SPLIT_177
# define MSGPACK_PP_SEQ_SPLIT_179(x) (x) MSGPACK_PP_SEQ_SPLIT_178
# define MSGPACK_PP_SEQ_SPLIT_180(x) (x) MSGPACK_PP_SEQ_SPLIT_179
# define MSGPACK_PP_SEQ_SPLIT_181(x) (x) MSGPACK_PP_SEQ_SPLIT_180
# define MSGPACK_PP_SEQ_SPLIT_182(x) (x) MSGPACK_PP_SEQ_SPLIT_181
# define MSGPACK_PP_SEQ_SPLIT_183(x) (x) MSGPACK_PP_SEQ_SPLIT_182
# define MSGPACK_PP_SEQ_SPLIT_184(x) (x) MSGPACK_PP_SEQ_SPLIT_183
# define MSGPACK_PP_SEQ_SPLIT_185(x) (x) MSGPACK_PP_SEQ_SPLIT_184
# define MSGPACK_PP_SEQ_SPLIT_186(x) (x) MSGPACK_PP_SEQ_SPLIT_185
# define MSGPACK_PP_SEQ_SPLIT_187(x) (x) MSGPACK_PP_SEQ_SPLIT_186
# define MSGPACK_PP_SEQ_SPLIT_188(x) (x) MSGPACK_PP_SEQ_SPLIT_187
# define MSGPACK_PP_SEQ_SPLIT_189(x) (x) MSGPACK_PP_SEQ_SPLIT_188
# define MSGPACK_PP_SEQ_SPLIT_190(x) (x) MSGPACK_PP_SEQ_SPLIT_189
# define MSGPACK_PP_SEQ_SPLIT_191(x) (x) MSGPACK_PP_SEQ_SPLIT_190
# define MSGPACK_PP_SEQ_SPLIT_192(x) (x) MSGPACK_PP_SEQ_SPLIT_191
# define MSGPACK_PP_SEQ_SPLIT_193(x) (x) MSGPACK_PP_SEQ_SPLIT_192
# define MSGPACK_PP_SEQ_SPLIT_194(x) (x) MSGPACK_PP_SEQ_SPLIT_193
# define MSGPACK_PP_SEQ_SPLIT_195(x) (x) MSGPACK_PP_SEQ_SPLIT_194
# define MSGPACK_PP_SEQ_SPLIT_196(x) (x) MSGPACK_PP_SEQ_SPLIT_195
# define MSGPACK_PP_SEQ_SPLIT_197(x) (x) MSGPACK_PP_SEQ_SPLIT_196
# define MSGPACK_PP_SEQ_SPLIT_198(x) (x) MSGPACK_PP_SEQ_SPLIT_197
# define MSGPACK_PP_SEQ_SPLIT_199(x) (x) MSGPACK_PP_SEQ_SPLIT_198
# define MSGPACK_PP_SEQ_SPLIT_200(x) (x) MSGPACK_PP_SEQ_SPLIT_199
# define MSGPACK_PP_SEQ_SPLIT_201(x) (x) MSGPACK_PP_SEQ_SPLIT_200
# define MSGPACK_PP_SEQ_SPLIT_202(x) (x) MSGPACK_PP_SEQ_SPLIT_201
# define MSGPACK_PP_SEQ_SPLIT_203(x) (x) MSGPACK_PP_SEQ_SPLIT_202
# define MSGPACK_PP_SEQ_SPLIT_204(x) (x) MSGPACK_PP_SEQ_SPLIT_203
# define MSGPACK_PP_SEQ_SPLIT_205(x) (x) MSGPACK_PP_SEQ_SPLIT_204
# define MSGPACK_PP_SEQ_SPLIT_206(x) (x) MSGPACK_PP_SEQ_SPLIT_205
# define MSGPACK_PP_SEQ_SPLIT_207(x) (x) MSGPACK_PP_SEQ_SPLIT_206
# define MSGPACK_PP_SEQ_SPLIT_208(x) (x) MSGPACK_PP_SEQ_SPLIT_207
# define MSGPACK_PP_SEQ_SPLIT_209(x) (x) MSGPACK_PP_SEQ_SPLIT_208
# define MSGPACK_PP_SEQ_SPLIT_210(x) (x) MSGPACK_PP_SEQ_SPLIT_209
# define MSGPACK_PP_SEQ_SPLIT_211(x) (x) MSGPACK_PP_SEQ_SPLIT_210
# define MSGPACK_PP_SEQ_SPLIT_212(x) (x) MSGPACK_PP_SEQ_SPLIT_211
# define MSGPACK_PP_SEQ_SPLIT_213(x) (x) MSGPACK_PP_SEQ_SPLIT_212
# define MSGPACK_PP_SEQ_SPLIT_214(x) (x) MSGPACK_PP_SEQ_SPLIT_213
# define MSGPACK_PP_SEQ_SPLIT_215(x) (x) MSGPACK_PP_SEQ_SPLIT_214
# define MSGPACK_PP_SEQ_SPLIT_216(x) (x) MSGPACK_PP_SEQ_SPLIT_215
# define MSGPACK_PP_SEQ_SPLIT_217(x) (x) MSGPACK_PP_SEQ_SPLIT_216
# define MSGPACK_PP_SEQ_SPLIT_218(x) (x) MSGPACK_PP_SEQ_SPLIT_217
# define MSGPACK_PP_SEQ_SPLIT_219(x) (x) MSGPACK_PP_SEQ_SPLIT_218
# define MSGPACK_PP_SEQ_SPLIT_220(x) (x) MSGPACK_PP_SEQ_SPLIT_219
# define MSGPACK_PP_SEQ_SPLIT_221(x) (x) MSGPACK_PP_SEQ_SPLIT_220
# define MSGPACK_PP_SEQ_SPLIT_222(x) (x) MSGPACK_PP_SEQ_SPLIT_221
# define MSGPACK_PP_SEQ_SPLIT_223(x) (x) MSGPACK_PP_SEQ_SPLIT_222
# define MSGPACK_PP_SEQ_SPLIT_224(x) (x) MSGPACK_PP_SEQ_SPLIT_223
# define MSGPACK_PP_SEQ_SPLIT_225(x) (x) MSGPACK_PP_SEQ_SPLIT_224
# define MSGPACK_PP_SEQ_SPLIT_226(x) (x) MSGPACK_PP_SEQ_SPLIT_225
# define MSGPACK_PP_SEQ_SPLIT_227(x) (x) MSGPACK_PP_SEQ_SPLIT_226
# define MSGPACK_PP_SEQ_SPLIT_228(x) (x) MSGPACK_PP_SEQ_SPLIT_227
# define MSGPACK_PP_SEQ_SPLIT_229(x) (x) MSGPACK_PP_SEQ_SPLIT_228
# define MSGPACK_PP_SEQ_SPLIT_230(x) (x) MSGPACK_PP_SEQ_SPLIT_229
# define MSGPACK_PP_SEQ_SPLIT_231(x) (x) MSGPACK_PP_SEQ_SPLIT_230
# define MSGPACK_PP_SEQ_SPLIT_232(x) (x) MSGPACK_PP_SEQ_SPLIT_231
# define MSGPACK_PP_SEQ_SPLIT_233(x) (x) MSGPACK_PP_SEQ_SPLIT_232
# define MSGPACK_PP_SEQ_SPLIT_234(x) (x) MSGPACK_PP_SEQ_SPLIT_233
# define MSGPACK_PP_SEQ_SPLIT_235(x) (x) MSGPACK_PP_SEQ_SPLIT_234
# define MSGPACK_PP_SEQ_SPLIT_236(x) (x) MSGPACK_PP_SEQ_SPLIT_235
# define MSGPACK_PP_SEQ_SPLIT_237(x) (x) MSGPACK_PP_SEQ_SPLIT_236
# define MSGPACK_PP_SEQ_SPLIT_238(x) (x) MSGPACK_PP_SEQ_SPLIT_237
# define MSGPACK_PP_SEQ_SPLIT_239(x) (x) MSGPACK_PP_SEQ_SPLIT_238
# define MSGPACK_PP_SEQ_SPLIT_240(x) (x) MSGPACK_PP_SEQ_SPLIT_239
# define MSGPACK_PP_SEQ_SPLIT_241(x) (x) MSGPACK_PP_SEQ_SPLIT_240
# define MSGPACK_PP_SEQ_SPLIT_242(x) (x) MSGPACK_PP_SEQ_SPLIT_241
# define MSGPACK_PP_SEQ_SPLIT_243(x) (x) MSGPACK_PP_SEQ_SPLIT_242
# define MSGPACK_PP_SEQ_SPLIT_244(x) (x) MSGPACK_PP_SEQ_SPLIT_243
# define MSGPACK_PP_SEQ_SPLIT_245(x) (x) MSGPACK_PP_SEQ_SPLIT_244
# define MSGPACK_PP_SEQ_SPLIT_246(x) (x) MSGPACK_PP_SEQ_SPLIT_245
# define MSGPACK_PP_SEQ_SPLIT_247(x) (x) MSGPACK_PP_SEQ_SPLIT_246
# define MSGPACK_PP_SEQ_SPLIT_248(x) (x) MSGPACK_PP_SEQ_SPLIT_247
# define MSGPACK_PP_SEQ_SPLIT_249(x) (x) MSGPACK_PP_SEQ_SPLIT_248
# define MSGPACK_PP_SEQ_SPLIT_250(x) (x) MSGPACK_PP_SEQ_SPLIT_249
# define MSGPACK_PP_SEQ_SPLIT_251(x) (x) MSGPACK_PP_SEQ_SPLIT_250
# define MSGPACK_PP_SEQ_SPLIT_252(x) (x) MSGPACK_PP_SEQ_SPLIT_251
# define MSGPACK_PP_SEQ_SPLIT_253(x) (x) MSGPACK_PP_SEQ_SPLIT_252
# define MSGPACK_PP_SEQ_SPLIT_254(x) (x) MSGPACK_PP_SEQ_SPLIT_253
# define MSGPACK_PP_SEQ_SPLIT_255(x) (x) MSGPACK_PP_SEQ_SPLIT_254
# define MSGPACK_PP_SEQ_SPLIT_256(x) (x) MSGPACK_PP_SEQ_SPLIT_255
#
# endif
