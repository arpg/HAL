/****************************************************************************
 *
 * $Id: v2u_util.c 12602 2011-03-10 13:33:18Z monich $
 *
 * Copyright (C) 2003-2011 Epiphan Systems Inc. All rights reserved.
 *
 * Miscellaneous utilities.
 *
 ****************************************************************************/

#include <ctype.h>
#include <string.h>

#include "v2u_id.h"
#include "v2u_util.h"
#include "v2u_save.h"

/*
 * Magic transformation of YUV to RGB
 */
#define YUV_R(Yc,Uc,Vc) yuv_clip[(((Vc) + (Yc)) >> 16) - 0x3f00]
#define YUV_G(Yc,Uc,Vc) yuv_clip[(((Uc) + (Vc) + (Yc)) & 0xffff) - 0x3f00]
#define YUV_B(Yc,Uc,Vc) yuv_clip[(((Uc) + (Yc)) >> 16) - 0x3f00]

/*
 * ASSUMPTION: the valid range for Y is [16,235] and the valid ranges
 * for U and V are [16,239]
 */
static const V2U_UINT32 yuv_y[256] = {
    0x20002000, 0x20002000, 0x20002000, 0x20002000,
    0x20002000, 0x20002000, 0x20002000, 0x20002000,
    0x20002000, 0x20002000, 0x20002000, 0x20002000,
    0x20002000, 0x20002000, 0x20002000, 0x20002000,
    /*------------------------------------------*/
    0x20002000, 0x20012001, 0x20022002, 0x20032003,
    0x20052005, 0x20062006, 0x20072007, 0x20082008,
    0x20092009, 0x200A200A, 0x200C200C, 0x200D200D,
    0x200E200E, 0x200F200F, 0x20102010, 0x20112011,
    0x20132013, 0x20142014, 0x20152015, 0x20162016,
    0x20172017, 0x20182018, 0x201A201A, 0x201B201B,
    0x201C201C, 0x201D201D, 0x201E201E, 0x201F201F,
    0x20212021, 0x20222022, 0x20232023, 0x20242024,
    0x20252025, 0x20262026, 0x20282028, 0x20292029,
    0x202A202A, 0x202B202B, 0x202C202C, 0x202D202D,
    0x202F202F, 0x20302030, 0x20312031, 0x20322032,
    0x20332033, 0x20342034, 0x20362036, 0x20372037,
    0x20382038, 0x20392039, 0x203A203A, 0x203B203B,
    0x203D203D, 0x203E203E, 0x203F203F, 0x20402040,
    0x20412041, 0x20422042, 0x20442044, 0x20452045,
    0x20462046, 0x20472047, 0x20482048, 0x20492049,
    0x204A204A, 0x204C204C, 0x204D204D, 0x204E204E,
    0x204F204F, 0x20502050, 0x20512051, 0x20532053,
    0x20542054, 0x20552055, 0x20562056, 0x20572057,
    0x20582058, 0x205A205A, 0x205B205B, 0x205C205C,
    0x205D205D, 0x205E205E, 0x205F205F, 0x20612061,
    0x20622062, 0x20632063, 0x20642064, 0x20652065,
    0x20662066, 0x20682068, 0x20692069, 0x206A206A,
    0x206B206B, 0x206C206C, 0x206D206D, 0x206F206F,
    0x20702070, 0x20712071, 0x20722072, 0x20732073,
    0x20742074, 0x20762076, 0x20772077, 0x20782078,
    0x20792079, 0x207A207A, 0x207B207B, 0x207D207D,
    0x207E207E, 0x207F207F, 0x20802080, 0x20812081,
    0x20822082, 0x20842084, 0x20852085, 0x20862086,
    0x20872087, 0x20882088, 0x20892089, 0x208B208B,
    0x208C208C, 0x208D208D, 0x208E208E, 0x208F208F,
    0x20902090, 0x20922092, 0x20932093, 0x20942094,
    0x20952095, 0x20962096, 0x20972097, 0x20982098,
    0x209A209A, 0x209B209B, 0x209C209C, 0x209D209D,
    0x209E209E, 0x209F209F, 0x20A120A1, 0x20A220A2,
    0x20A320A3, 0x20A420A4, 0x20A520A5, 0x20A620A6,
    0x20A820A8, 0x20A920A9, 0x20AA20AA, 0x20AB20AB,
    0x20AC20AC, 0x20AD20AD, 0x20AF20AF, 0x20B020B0,
    0x20B120B1, 0x20B220B2, 0x20B320B3, 0x20B420B4,
    0x20B620B6, 0x20B720B7, 0x20B820B8, 0x20B920B9,
    0x20BA20BA, 0x20BB20BB, 0x20BD20BD, 0x20BE20BE,
    0x20BF20BF, 0x20C020C0, 0x20C120C1, 0x20C220C2,
    0x20C420C4, 0x20C520C5, 0x20C620C6, 0x20C720C7,
    0x20C820C8, 0x20C920C9, 0x20CB20CB, 0x20CC20CC,
    0x20CD20CD, 0x20CE20CE, 0x20CF20CF, 0x20D020D0,
    0x20D220D2, 0x20D320D3, 0x20D420D4, 0x20D520D5,
    0x20D620D6, 0x20D720D7, 0x20D920D9, 0x20DA20DA,
    0x20DB20DB, 0x20DC20DC, 0x20DD20DD, 0x20DE20DE,
    0x20DF20DF, 0x20E120E1, 0x20E220E2, 0x20E320E3,
    0x20E420E4, 0x20E520E5, 0x20E620E6, 0x20E820E8,
    0x20E920E9, 0x20EA20EA, 0x20EB20EB, 0x20EC20EC,
    0x20ED20ED, 0x20EF20EF, 0x20F020F0, 0x20F120F1,
    0x20F220F2, 0x20F320F3, 0x20F420F4, 0x20F620F6,
    0x20F720F7, 0x20F820F8, 0x20F920F9, 0x20FA20FA,
    0x20FB20FB, 0x20FD20FD, 0x20FE20FE, 0x20FF20FF,
    /*------------------------------------------*/
    0x20FF20FF, 0x20FF20FF, 0x20FF20FF, 0x20FF20FF,
    0x20FF20FF, 0x20FF20FF, 0x20FF20FF, 0x20FF20FF,
    0x20FF20FF, 0x20FF20FF, 0x20FF20FF, 0x20FF20FF,
    0x20FF20FF, 0x20FF20FF, 0x20FF20FF, 0x20FF20FF,
    0x20FF20FF, 0x20FF20FF, 0x20FF20FF, 0x20FF20FF,
};

const V2U_UINT32 yuv_u[256] = {
    0x1F1E102C, 0x1F1E102C, 0x1F1E102C, 0x1F1E102C,
    0x1F1E102C, 0x1F1E102C, 0x1F1E102C, 0x1F1E102C,
    0x1F1E102C, 0x1F1E102C, 0x1F1E102C, 0x1F1E102C,
    0x1F1E102C, 0x1F1E102C, 0x1F1E102C, 0x1F1E102C,
    /*------------------------------------------*/
    0x1F1E102C, 0x1F20102B, 0x1F22102B, 0x1F24102B,
    0x1F26102A, 0x1F28102A, 0x1F2A1029, 0x1F2C1029,
    0x1F2E1029, 0x1F301028, 0x1F321028, 0x1F341027,
    0x1F361027, 0x1F381027, 0x1F3A1026, 0x1F3C1026,
    0x1F3E1026, 0x1F401025, 0x1F421025, 0x1F441024,
    0x1F461024, 0x1F481024, 0x1F4A1023, 0x1F4C1023,
    0x1F4E1022, 0x1F501022, 0x1F521022, 0x1F541021,
    0x1F561021, 0x1F591020, 0x1F5B1020, 0x1F5D1020,
    0x1F5F101F, 0x1F61101F, 0x1F63101E, 0x1F65101E,
    0x1F67101E, 0x1F69101D, 0x1F6B101D, 0x1F6D101D,
    0x1F6F101C, 0x1F71101C, 0x1F73101B, 0x1F75101B,
    0x1F77101B, 0x1F79101A, 0x1F7B101A, 0x1F7D1019,
    0x1F7F1019, 0x1F811019, 0x1F831018, 0x1F851018,
    0x1F871017, 0x1F891017, 0x1F8B1017, 0x1F8D1016,
    0x1F8F1016, 0x1F911016, 0x1F931015, 0x1F951015,
    0x1F971014, 0x1F991014, 0x1F9B1014, 0x1F9D1013,
    0x1F9F1013, 0x1FA11012, 0x1FA31012, 0x1FA51012,
    0x1FA71011, 0x1FA91011, 0x1FAB1010, 0x1FAD1010,
    0x1FAF1010, 0x1FB1100F, 0x1FB3100F, 0x1FB5100E,
    0x1FB7100E, 0x1FB9100E, 0x1FBB100D, 0x1FBD100D,
    0x1FBF100D, 0x1FC1100C, 0x1FC3100C, 0x1FC5100B,
    0x1FC7100B, 0x1FCA100B, 0x1FCC100A, 0x1FCE100A,
    0x1FD01009, 0x1FD21009, 0x1FD41009, 0x1FD61008,
    0x1FD81008, 0x1FDA1007, 0x1FDC1007, 0x1FDE1007,
    0x1FE01006, 0x1FE21006, 0x1FE41005, 0x1FE61005,
    0x1FE81005, 0x1FEA1004, 0x1FEC1004, 0x1FEE1004,
    0x1FF01003, 0x1FF21003, 0x1FF41002, 0x1FF61002,
    0x1FF81002, 0x1FFA1001, 0x1FFC1001, 0x1FFE1000,
    0x20001000, 0x20021000, 0x20040FFF, 0x20060FFF,
    0x20080FFE, 0x200A0FFE, 0x200C0FFE, 0x200E0FFD,
    0x20100FFD, 0x20120FFC, 0x20140FFC, 0x20160FFC,
    0x20180FFB, 0x201A0FFB, 0x201C0FFB, 0x201E0FFA,
    0x20200FFA, 0x20220FF9, 0x20240FF9, 0x20260FF9,
    0x20280FF8, 0x202A0FF8, 0x202C0FF7, 0x202E0FF7,
    0x20300FF7, 0x20320FF6, 0x20340FF6, 0x20360FF5,
    0x20390FF5, 0x203B0FF5, 0x203D0FF4, 0x203F0FF4,
    0x20410FF3, 0x20430FF3, 0x20450FF3, 0x20470FF2,
    0x20490FF2, 0x204B0FF2, 0x204D0FF1, 0x204F0FF1,
    0x20510FF0, 0x20530FF0, 0x20550FF0, 0x20570FEF,
    0x20590FEF, 0x205B0FEE, 0x205D0FEE, 0x205F0FEE,
    0x20610FED, 0x20630FED, 0x20650FEC, 0x20670FEC,
    0x20690FEC, 0x206B0FEB, 0x206D0FEB, 0x206F0FEA,
    0x20710FEA, 0x20730FEA, 0x20750FE9, 0x20770FE9,
    0x20790FE9, 0x207B0FE8, 0x207D0FE8, 0x207F0FE7,
    0x20810FE7, 0x20830FE7, 0x20850FE6, 0x20870FE6,
    0x20890FE5, 0x208B0FE5, 0x208D0FE5, 0x208F0FE4,
    0x20910FE4, 0x20930FE3, 0x20950FE3, 0x20970FE3,
    0x20990FE2, 0x209B0FE2, 0x209D0FE2, 0x209F0FE1,
    0x20A10FE1, 0x20A30FE0, 0x20A50FE0, 0x20A70FE0,
    0x20AA0FDF, 0x20AC0FDF, 0x20AE0FDE, 0x20B00FDE,
    0x20B20FDE, 0x20B40FDD, 0x20B60FDD, 0x20B80FDC,
    0x20BA0FDC, 0x20BC0FDC, 0x20BE0FDB, 0x20C00FDB,
    0x20C20FDA, 0x20C40FDA, 0x20C60FDA, 0x20C80FD9,
    0x20CA0FD9, 0x20CC0FD9, 0x20CE0FD8, 0x20D00FD8,
    0x20D20FD7, 0x20D40FD7, 0x20D60FD7, 0x20D80FD6,
    0x20DA0FD6, 0x20DC0FD5, 0x20DE0FD5, 0x20E00FD5,
    /*------------------------------------------*/
    0x20E00FD5, 0x20E00FD5, 0x20E00FD5, 0x20E00FD5,
    0x20E00FD5, 0x20E00FD5, 0x20E00FD5, 0x20E00FD5,
    0x20E00FD5, 0x20E00FD5, 0x20E00FD5, 0x20E00FD5,
    0x20E00FD5, 0x20E00FD5, 0x20E00FD5, 0x20E00FD5,
};

const V2U_UINT32 yuv_v[256] = {
    0x1F4D105B, 0x1F4D105B, 0x1F4D105B, 0x1F4D105B,
    0x1F4D105B, 0x1F4D105B, 0x1F4D105B, 0x1F4D105B,
    0x1F4D105B, 0x1F4D105B, 0x1F4D105B, 0x1F4D105B,
    0x1F4D105B, 0x1F4D105B, 0x1F4D105B, 0x1F4D105B,
    /*------------------------------------------*/
    0x1F4D105B, 0x1F4F105A, 0x1F501059, 0x1F521059,
    0x1F541058, 0x1F551057, 0x1F571056, 0x1F581055,
    0x1F5A1055, 0x1F5C1054, 0x1F5D1053, 0x1F5F1052,
    0x1F601051, 0x1F621050, 0x1F641050, 0x1F65104F,
    0x1F67104E, 0x1F68104D, 0x1F6A104C, 0x1F6C104C,
    0x1F6D104B, 0x1F6F104A, 0x1F701049, 0x1F721048,
    0x1F741048, 0x1F751047, 0x1F771046, 0x1F781045,
    0x1F7A1044, 0x1F7C1043, 0x1F7D1043, 0x1F7F1042,
    0x1F801041, 0x1F821040, 0x1F84103F, 0x1F85103F,
    0x1F87103E, 0x1F88103D, 0x1F8A103C, 0x1F8B103B,
    0x1F8D103B, 0x1F8F103A, 0x1F901039, 0x1F921038,
    0x1F931037, 0x1F951036, 0x1F971036, 0x1F981035,
    0x1F9A1034, 0x1F9B1033, 0x1F9D1032, 0x1F9F1032,
    0x1FA01031, 0x1FA21030, 0x1FA3102F, 0x1FA5102E,
    0x1FA7102E, 0x1FA8102D, 0x1FAA102C, 0x1FAB102B,
    0x1FAD102A, 0x1FAF1029, 0x1FB01029, 0x1FB21028,
    0x1FB31027, 0x1FB51026, 0x1FB71025, 0x1FB81025,
    0x1FBA1024, 0x1FBB1023, 0x1FBD1022, 0x1FBF1021,
    0x1FC01021, 0x1FC21020, 0x1FC3101F, 0x1FC5101E,
    0x1FC7101D, 0x1FC8101C, 0x1FCA101C, 0x1FCB101B,
    0x1FCD101A, 0x1FCF1019, 0x1FD01018, 0x1FD21018,
    0x1FD31017, 0x1FD51016, 0x1FD71015, 0x1FD81014,
    0x1FDA1014, 0x1FDB1013, 0x1FDD1012, 0x1FDE1011,
    0x1FE01010, 0x1FE2100F, 0x1FE3100F, 0x1FE5100E,
    0x1FE6100D, 0x1FE8100C, 0x1FEA100B, 0x1FEB100B,
    0x1FED100A, 0x1FEE1009, 0x1FF01008, 0x1FF21007,
    0x1FF31007, 0x1FF51006, 0x1FF61005, 0x1FF81004,
    0x1FFA1003, 0x1FFB1002, 0x1FFD1002, 0x1FFE1001,
    0x20001000, 0x20020FFF, 0x20030FFE, 0x20050FFE,
    0x20060FFD, 0x20080FFC, 0x200A0FFB, 0x200B0FFA,
    0x200D0FF9, 0x200E0FF9, 0x20100FF8, 0x20120FF7,
    0x20130FF6, 0x20150FF5, 0x20160FF5, 0x20180FF4,
    0x201A0FF3, 0x201B0FF2, 0x201D0FF1, 0x201E0FF1,
    0x20200FF0, 0x20220FEF, 0x20230FEE, 0x20250FED,
    0x20260FEC, 0x20280FEC, 0x20290FEB, 0x202B0FEA,
    0x202D0FE9, 0x202E0FE8, 0x20300FE8, 0x20310FE7,
    0x20330FE6, 0x20350FE5, 0x20360FE4, 0x20380FE4,
    0x20390FE3, 0x203B0FE2, 0x203D0FE1, 0x203E0FE0,
    0x20400FDF, 0x20410FDF, 0x20430FDE, 0x20450FDD,
    0x20460FDC, 0x20480FDB, 0x20490FDB, 0x204B0FDA,
    0x204D0FD9, 0x204E0FD8, 0x20500FD7, 0x20510FD7,
    0x20530FD6, 0x20550FD5, 0x20560FD4, 0x20580FD3,
    0x20590FD2, 0x205B0FD2, 0x205D0FD1, 0x205E0FD0,
    0x20600FCF, 0x20610FCE, 0x20630FCE, 0x20650FCD,
    0x20660FCC, 0x20680FCB, 0x20690FCA, 0x206B0FCA,
    0x206D0FC9, 0x206E0FC8, 0x20700FC7, 0x20710FC6,
    0x20730FC5, 0x20750FC5, 0x20760FC4, 0x20780FC3,
    0x20790FC2, 0x207B0FC1, 0x207C0FC1, 0x207E0FC0,
    0x20800FBF, 0x20810FBE, 0x20830FBD, 0x20840FBD,
    0x20860FBC, 0x20880FBB, 0x20890FBA, 0x208B0FB9,
    0x208C0FB8, 0x208E0FB8, 0x20900FB7, 0x20910FB6,
    0x20930FB5, 0x20940FB4, 0x20960FB4, 0x20980FB3,
    0x20990FB2, 0x209B0FB1, 0x209C0FB0, 0x209E0FB0,
    0x20A00FAF, 0x20A10FAE, 0x20A30FAD, 0x20A40FAC,
    0x20A60FAB, 0x20A80FAB, 0x20A90FAA, 0x20AB0FA9,
    0x20AC0FA8, 0x20AE0FA7, 0x20B00FA7, 0x20B10FA6,
    /*------------------------------------------*/
    0x20B10FA6, 0x20B10FA6, 0x20B10FA6, 0x20B10FA6,
    0x20B10FA6, 0x20B10FA6, 0x20B10FA6, 0x20B10FA6,
    0x20B10FA6, 0x20B10FA6, 0x20B10FA6, 0x20B10FA6,
    0x20B10FA6, 0x20B10FA6, 0x20B10FA6, 0x20B10FA6,
};

static const V2U_BYTE yuv_clip[768] = {
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
     16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
     32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
     48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
     64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
     80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
     96, 97, 98, 99,100,101,102,103,104,105,106,107,108,109,110,111,
    112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
    128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,
    144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
    160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,
    176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
    192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,
    208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
    224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,
    240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
};

/**
 * Converts one row from NV12 to RGB24
 */
void v2u_convert_nv12_to_rgb24(void* dst, const void* src, 
    int row, int w, int h)
{
    V2U_BYTE* rgb = dst;
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* uv = y + ls*h;
    int x;

    y += ls*row;
    uv += ls*(row & ~1)/2;

    for (x=0; x<w-1; x+=2) {
        V2U_UINT32 Uc = yuv_u[*uv++];
        V2U_UINT32 Vc = yuv_v[*uv++];
        V2U_UINT32 Yc = yuv_y[*y++];

        *rgb++ = YUV_R(Yc, Uc, Vc);
        *rgb++ = YUV_G(Yc, Uc, Vc);
        *rgb++ = YUV_B(Yc, Uc, Vc);

        Yc = yuv_y[*y++];
        *rgb++ = YUV_R(Yc, Uc, Vc);
        *rgb++ = YUV_G(Yc, Uc, Vc);
        *rgb++ = YUV_B(Yc, Uc, Vc);
    }
}

/**
 * Converts one row from NV12 to BGR24
 */
void v2u_convert_nv12_to_bgr24(void* dst, const void* src, 
    int row, int w, int h)
{
    V2U_BYTE* bgr = dst;
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* uv = y + ls*h;
    int x;

    y += ls*row;
    uv += ls*(row & ~1)/2;

    for (x=0; x<w-1; x+=2) {
        V2U_UINT32 Uc = yuv_u[*uv++];
        V2U_UINT32 Vc = yuv_v[*uv++];
        V2U_UINT32 Yc = yuv_y[*y++];

        *bgr++ = YUV_B(Yc, Uc, Vc);
        *bgr++ = YUV_G(Yc, Uc, Vc);
        *bgr++ = YUV_R(Yc, Uc, Vc);

        Yc = yuv_y[*y++];
        *bgr++ = YUV_B(Yc, Uc, Vc);
        *bgr++ = YUV_G(Yc, Uc, Vc);
        *bgr++ = YUV_R(Yc, Uc, Vc);
    }
}

/**
 * Helper function for converting one row from I420/YV12 to RGB24
 */
static void v2u_convert_planar_yuv_to_rgb24(V2U_BYTE* rgb, const V2U_BYTE* y, 
    const V2U_BYTE* u, const V2U_BYTE* v, int row, int w, int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    int x;

    y += ls*row;
    u += ls*(row & ~1)/4;
    v += ls*(row & ~1)/4;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Uc = yuv_u[*u++];
        const V2U_UINT32 Vc = yuv_v[*v++];
        V2U_UINT32 Yc;

        Yc = yuv_y[*y++];
        *rgb++ = YUV_R(Yc, Uc, Vc);
        *rgb++ = YUV_G(Yc, Uc, Vc);
        *rgb++ = YUV_B(Yc, Uc, Vc);

        Yc = yuv_y[*y++];
        *rgb++ = YUV_R(Yc, Uc, Vc);
        *rgb++ = YUV_G(Yc, Uc, Vc);
        *rgb++ = YUV_B(Yc, Uc, Vc);
    }
}

/**
 * Helper function for converting one row from I420/YV12 to BGR24
 */
static void v2u_convert_planar_yuv_to_bgr24(V2U_BYTE* bgr, const V2U_BYTE* y, 
    const V2U_BYTE* u, const V2U_BYTE* v, int row, int w, int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    int x;

    y += ls*row;
    v += ls*(row & ~1)/4;
    u += ls*(row & ~1)/4;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Uc = yuv_u[*u++];
        const V2U_UINT32 Vc = yuv_v[*v++];
        V2U_UINT32 Yc;

        Yc = yuv_y[*y++];
        *bgr++ = YUV_B(Yc, Uc, Vc);
        *bgr++ = YUV_G(Yc, Uc, Vc);
        *bgr++ = YUV_R(Yc, Uc, Vc);

        Yc = yuv_y[*y++];
        *bgr++ = YUV_B(Yc, Uc, Vc);
        *bgr++ = YUV_G(Yc, Uc, Vc);
        *bgr++ = YUV_R(Yc, Uc, Vc);
    }
}

/**
 * Converts one row from YV12 to RGB24
 */
void v2u_convert_yv12_to_rgb24(void* dst,const void* src,int row,int w,int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* v = y + ls*h;
    const V2U_BYTE* u = v + ls*h/4;
    v2u_convert_planar_yuv_to_rgb24(dst, y, u, v, row, w, h); 
}

/**
 * Converts one row from YV12 to BGR24
 */
void v2u_convert_yv12_to_bgr24(void* dst,const void* src,int row,int w,int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* v = y + ls*h;
    const V2U_BYTE* u = v + ls*h/4;
    v2u_convert_planar_yuv_to_bgr24(dst, y, u, v, row, w, h); 
}

/**
 * Converts one row from I420 to RGB24
 */
void v2u_convert_i420_to_rgb24(void* dst,const void* src,int row,int w,int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* u = y + ls*h;
    const V2U_BYTE* v = u + ls*h/4;
    v2u_convert_planar_yuv_to_rgb24(dst, y, u, v, row, w, h); 
}

/**
 * Converts one row from I420 to BGR24
 */
void v2u_convert_i420_to_bgr24(void* dst,const void* src,int row,int w,int h)
{
    const int ls = V2U_LINE_SIZE_BPP(w,8);
    const V2U_BYTE* y = src;
    const V2U_BYTE* u = y + ls*h;
    const V2U_BYTE* v = u + ls*h/4;
    v2u_convert_planar_yuv_to_bgr24(dst, y, u, v, row, w, h); 
}

/**
 * Converts one row from YUY2 to RGB24
 */
void v2u_convert_yuy2_to_rgb24(void* dst,const void* src,int row,int w,int h)
{
    int x;
    V2U_BYTE* rgb = dst;
    const V2U_BYTE* yuv = src;
    yuv += 2*w*row;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Y0c = yuv_y[*yuv++];
        const V2U_UINT32 Uc  = yuv_u[*yuv++];
        const V2U_UINT32 Y1c = yuv_y[*yuv++];
        const V2U_UINT32 Vc  = yuv_v[*yuv++];

        *rgb++ = YUV_R(Y0c,Uc,Vc);
        *rgb++ = YUV_G(Y0c,Uc,Vc);
        *rgb++ = YUV_B(Y0c,Uc,Vc);
        *rgb++ = YUV_R(Y1c,Uc,Vc);
        *rgb++ = YUV_G(Y1c,Uc,Vc);
        *rgb++ = YUV_B(Y1c,Uc,Vc);
    }
}

/**
 * Converts one row from YUY2 to BGR24
 */
void v2u_convert_yuy2_to_bgr24(void* dst,const void* src,int row,int w,int h)
{
    int x;
    V2U_BYTE* bgr = dst;
    const V2U_BYTE* yuv = src;
    yuv += 2*w*row;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Y0c = yuv_y[*yuv++];
        const V2U_UINT32 Uc  = yuv_u[*yuv++];
        const V2U_UINT32 Y1c = yuv_y[*yuv++];
        const V2U_UINT32 Vc  = yuv_v[*yuv++];

        *bgr++ = YUV_B(Y0c,Uc,Vc);
        *bgr++ = YUV_G(Y0c,Uc,Vc);
        *bgr++ = YUV_R(Y0c,Uc,Vc);
        *bgr++ = YUV_B(Y1c,Uc,Vc);
        *bgr++ = YUV_G(Y1c,Uc,Vc);
        *bgr++ = YUV_R(Y1c,Uc,Vc);
    }
}

/**
 * Converts one row from 2VUY to RGB24
 */
void v2u_convert_2vuy_to_rgb24(void* dst,const void* src,int row,int w,int h)
{
    int x;
    V2U_BYTE* rgb = dst;
    const V2U_BYTE* yuv = src;
    yuv += 2*w*row;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Uc  = yuv_u[*yuv++];
        const V2U_UINT32 Y0c = yuv_y[*yuv++];
        const V2U_UINT32 Vc  = yuv_v[*yuv++];
        const V2U_UINT32 Y1c = yuv_y[*yuv++];

        *rgb++ = YUV_R(Y0c,Uc,Vc);
        *rgb++ = YUV_G(Y0c,Uc,Vc);
        *rgb++ = YUV_B(Y0c,Uc,Vc);
        *rgb++ = YUV_R(Y1c,Uc,Vc);
        *rgb++ = YUV_G(Y1c,Uc,Vc);
        *rgb++ = YUV_B(Y1c,Uc,Vc);
    }
}

/**
 * Converts one row from 2VUY to BGR24
 */
void v2u_convert_2vuy_to_bgr24(void* dst,const void* src,int row,int w,int h)
{
    int x;
    V2U_BYTE* bgr = dst;
    const V2U_BYTE* yuv = src;
    yuv += 2*w*row;

    for (x=0; x<w-1; x+=2) {
        const V2U_UINT32 Uc  = yuv_u[*yuv++];
        const V2U_UINT32 Y0c = yuv_y[*yuv++];
        const V2U_UINT32 Vc  = yuv_v[*yuv++];
        const V2U_UINT32 Y1c = yuv_y[*yuv++];

        *bgr++ = YUV_B(Y0c,Uc,Vc);
        *bgr++ = YUV_G(Y0c,Uc,Vc);
        *bgr++ = YUV_R(Y0c,Uc,Vc);
        *bgr++ = YUV_B(Y1c,Uc,Vc);
        *bgr++ = YUV_G(Y1c,Uc,Vc);
        *bgr++ = YUV_R(Y1c,Uc,Vc);
    }
}

/**
 * Copies a single row that came from VGA2USB into 24-bit RGB row in the
 * format understood by JPEG and PNG libraries. The following image formats
 * are supported:
 *
 *   V2U_GRABFRAME_FORMAT_Y8
 *   V2U_GRABFRAME_FORMAT_RGB4
 *   V2U_GRABFRAME_FORMAT_RGB8
 *   V2U_GRABFRAME_FORMAT_RGB16
 *   V2U_GRABFRAME_FORMAT_BGR16
 *   V2U_GRABFRAME_FORMAT_RGB24
 *   V2U_GRABFRAME_FORMAT_BGR24
 *   V2U_GRABFRAME_FORMAT_ARGB32
 */
V2U_BOOL v2u_copy_line(void* dest24, const void* bmp, int fmt, int w)
{
    /* (r << 3) | (r >> 2) */
    static const unsigned char R[32] = {
        0,  8, 16, 24, 33, 41, 49, 57, 66, 74, 82, 90, 99,107,115,123,
        132,140,148,156,165,173,181,189,198,206,214,222,231,239,247,255,
    };

    /* (g << 2) | (g >> 4) */
    static const unsigned char G[64] = {
        0,  4,  8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
        65, 69, 73, 77, 81, 85, 89, 93, 97,101,105,109,113,117,121,125,
        130,134,138,142,146,150,154,158,162,166,170,174,178,182,186,190,
        195,199,203,207,211,215,219,223,227,231,235,239,243,247,251,255,
    };

    /* (b << 3) | (b >> 2) */
    static const unsigned char* B = R;

    const BmpPalEntry * palette;
    unsigned char* dest = dest24;
    const unsigned char* src = bmp;
    int i;

    fmt &= V2U_GRABFRAME_FORMAT_MASK;
    switch (fmt) {
    case V2U_GRABFRAME_FORMAT_RGB24:
        memcpy(dest, src, w*3);
        break;

    case V2U_GRABFRAME_FORMAT_ARGB32:
        for (i=0; i<w; i++) {
            *dest++ = src[1];
            *dest++ = src[2];
            *dest++ = src[3];
            src += 4;
        }
        break;

    case V2U_GRABFRAME_FORMAT_BGR24:
        for (i=0; i<w; i++) {
            *dest++ = src[2];
            *dest++ = src[1];
            *dest++ = src[0];
            src += 3;
        }
        break;
    
    case V2U_GRABFRAME_FORMAT_BGR16:
        for (i=0; i<w; i++) {
            unsigned short pixel = *src++;
            pixel |= ((*src++) << 8);
            *dest++ = R[pixel & 0x001F];
            *dest++ = G[(pixel & 0x07E0) >> 5];
            *dest++ = B[(pixel & 0xF800) >> 11];
        }
        break;

    case V2U_GRABFRAME_FORMAT_RGB16:
        for (i=0; i<w; i++) {
            unsigned short pixel = *src++;
            pixel |= ((*src++) << 8);
            *dest++ = R[(pixel & 0xF800) >> 11];
            *dest++ = G[(pixel & 0x07E0) >> 5];
            *dest++ = B[pixel & 0x001F];
        }
        break;

    case V2U_GRABFRAME_FORMAT_Y8:
    case V2U_GRABFRAME_FORMAT_RGB8:
        palette = (fmt == V2U_GRABFRAME_FORMAT_Y8) ? 
            v2u_palette_y8 :    /* V2U_GRABFRAME_FORMAT_Y8 */
            v2u_palette_8bpp;   /* V2U_GRABFRAME_FORMAT_RGB8 */

        for (i=0; i<w; i++) {
            const BmpPalEntry * color = palette + (*src++);
            *dest++ = color->rgbRed;
            *dest++ = color->rgbGreen;
            *dest++ = color->rgbBlue;
        }
        break;

    case V2U_GRABFRAME_FORMAT_RGB4:
        for (i=0; i<w-1; i+=2) {
            const unsigned char pixel = (*src++);
            const BmpPalEntry* color = v2u_palette_4bpp + (pixel >> 4);
            *dest++ = color->rgbRed;
            *dest++ = color->rgbGreen;
            *dest++ = color->rgbBlue;
            color = v2u_palette_4bpp + (pixel & 0x0f);
            *dest++ = color->rgbRed;
            *dest++ = color->rgbGreen;
            *dest++ = color->rgbBlue;
        }
        if (i<w) {
            const BmpPalEntry* color = v2u_palette_4bpp + ((*src) >> 4);
            *dest++ = color->rgbRed;
            *dest++ = color->rgbGreen;
            *dest++ = color->rgbBlue;
        }
        break;

    default:
        return V2U_FALSE;
    }

    return V2U_TRUE;
}

/**
 * Converts a single row from the specified grab format to RGB24. Used by
 * v2u_write_jpeg and v2u_write_png. The following image formats are supported:
 *
 *   V2U_GRABFRAME_FORMAT_Y8
 *   V2U_GRABFRAME_FORMAT_RGB4
 *   V2U_GRABFRAME_FORMAT_RGB8
 *   V2U_GRABFRAME_FORMAT_RGB16
 *   V2U_GRABFRAME_FORMAT_BGR16
 *   V2U_GRABFRAME_FORMAT_RGB24
 *   V2U_GRABFRAME_FORMAT_BGR24
 *   V2U_GRABFRAME_FORMAT_ARGB32
 *   V2U_GRABFRAME_FORMAT_YUY2
 *   V2U_GRABFRAME_FORMAT_2VUY
 *   V2U_GRABFRAME_FORMAT_YV12
 *   V2U_GRABFRAME_FORMAT_I420
 */
V2U_BOOL v2u_convert_to_rgb24(void* dst, const void* src, int fmt, int row,
    int w, int h)
{
    const char* srcRow;
    switch (V2U_GRABFRAME_FORMAT(fmt)) {
    case V2U_GRABFRAME_FORMAT_YUY2:
        v2u_convert_yuy2_to_rgb24(dst, src, row, w, h);
        return V2U_TRUE;

    case V2U_GRABFRAME_FORMAT_2VUY:
        v2u_convert_2vuy_to_rgb24(dst, src, row, w, h);
        return V2U_TRUE;

    case V2U_GRABFRAME_FORMAT_YV12:
        v2u_convert_yv12_to_rgb24(dst, src, row, w, h);
        return V2U_TRUE;

    case V2U_GRABFRAME_FORMAT_I420:
        v2u_convert_i420_to_rgb24(dst, src, row, w, h);
        return V2U_TRUE;

    case V2U_GRABFRAME_FORMAT_NV12:
        v2u_convert_nv12_to_rgb24(dst, src, row, w, h);
        return V2U_TRUE;

    default:
        srcRow = src;
        srcRow += V2U_LINE_SIZE(w,fmt)*row;
        return v2u_copy_line(dst, srcRow, fmt, w);
    }
}

/**
 * Returns user-readable name for the specified device type
 */
const char* v2u_product_name(V2UProductType type)
{
    switch (type) {
    default:
    case V2UProductOther:       return "Unknown";

    /*
     * The following resolves into something like this:
     *
     * case V2UProductVGA2USB:     return "VGA2USB";
     * case V2UProductKVM2USB:     return "KVM2USB";
     *
     * .. and so on.
     */
#define PRODUCT_NAME_ENTRY(id,type,name) case type: return name;
    V2U_PRODUCT_MAP(PRODUCT_NAME_ENTRY)
    }
}

/**
 * Files saved by Phoenix EDID Designer tool look like this:
 * 
 * EDID BYTES:
 * 0x   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
 *     ------------------------------------------------
 * 00 | 00 FF FF FF FF FF FF 00 16 08 22 22 00 00 00 00
 * 10 | 08 10 01 03 EE 28 1E 78 0B 01 95 A3 57 4C 9C 25
 * 20 | 12 50 54 EF CF 00 81 C0 8B C0 90 40 B3 00 A9 40
 * 30 | D1 C0 01 01 01 01 C8 32 40 A0 60 B0 23 40 30 20
 * 40 | 35 00 90 2C 11 00 00 18 00 00 00 FF 00 56 33 55
 * 50 | 58 58 58 58 58 0A 0A 0A 0A 0A 00 00 00 FD 00 38
 * 60 | 55 0F 6E 10 00 0A 20 20 20 20 20 20 00 00 00 FC
 * 70 | 00 45 70 69 70 68 61 6E 20 44 32 55 0A 0A 00 14
 */

#define EDID_LINES 8
#define EDID_LINE_LEN 52
#define EDID_DIGITS_PER_LINE 16
#define EDID_HEADER_LINES 3

#define EDID_BUFSIZE (EDID_LINE_LEN+3)

static const char* edid_header[EDID_HEADER_LINES] = {
    "EDID BYTES:",
    "0x   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F",
    "    ------------------------------------------------"
};

static const char* edid_prefix[EDID_LINES] = {
    "00 |",
    "10 |",
    "20 |",
    "30 |",
    "40 |",
    "50 |",
    "60 |",
    "70 |",
};

/**
 * Reads text EDID from the stream. The text EDID must be in the
 * Phoenix EDID Designer format.
 */
V2U_BOOL v2u_edid_read(FILE* in, V2U_UINT8 edid[V2U_EDID_SIZE])
{
    int i;
    size_t len;
    char line[EDID_BUFSIZE];

    /* Read and validate the header */
    for (i=0; i<EDID_HEADER_LINES; i++) {
        
        /* Read the line and strip end of line characters */
        if (!fgets(line, EDID_BUFSIZE, in)) {
            return V2U_FALSE;
        }
        len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
            line[--len] = 0;
        }

        /* Compare with what we expect */
        if (strcmp(edid_header[i], line)) {
            return V2U_FALSE;
        }
    }

    /* Parse the data lines */
    for (i=0; i<EDID_LINES; i++) {
        int k;
        const char* ptr;
        size_t prefix_len = strlen(edid_prefix[i]);

        /* Read the line and strip end of line characters */
        if (!fgets(line, EDID_BUFSIZE, in)) {
            return V2U_FALSE;
        }
        len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
            line[--len] = 0;
        }

        /* Make sure that the prefix is correct */
        if (len != EDID_LINE_LEN || strncmp(edid_prefix[i],line,prefix_len)) {
            return V2U_FALSE;
        }

        /* Parse the rest */
        for (k=0, ptr=line+prefix_len; k<EDID_DIGITS_PER_LINE; k++, ptr+=3) {
            unsigned int n;
            char s[3];
            if (ptr[0] != ' ' || !isxdigit(ptr[1]) || !isxdigit(ptr[2])) {
                return V2U_FALSE;
            }
            s[0] = ptr[1];
            s[1] = ptr[2];
            s[2] = 0;
            if (sscanf(s, "%X", &n) != 1) {
                return V2U_FALSE;
            }
            *edid++ = (V2U_UINT8)n;
        }
    }
    return V2U_TRUE;
}

/**
 * Writes EDID to the stream in the text form (Phoenix EDID Designer format).
 */
V2U_BOOL v2u_edid_write(FILE* out, const V2U_UINT8 edid[V2U_EDID_SIZE])
{
    int i;
    for (i=0; i<EDID_HEADER_LINES; i++) {
        if (fputs(edid_header[i], out) == EOF ||
            fputs("\n", out) == EOF) {
            return V2U_FALSE;
        }
    }
    for (i=0; i<EDID_LINES; i++) {
        int k;
        if (fputs(edid_prefix[i], out) == EOF) {
            return V2U_FALSE;
        }
        for (k=0; k<EDID_DIGITS_PER_LINE; k++) {
            if (fprintf(out, " %02X", *edid++) < 0) {
                return V2U_FALSE;
            }
        }
        if (fputs("\n", out) == EOF) {
            return V2U_FALSE;
        }
    }
    return V2U_TRUE;
}
