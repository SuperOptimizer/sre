#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint8_t u8;
typedef int8_t s8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef int64_t s64;
typedef __uint128_t u128;
typedef __int128_t s128;
typedef float f32;
typedef double f64;

#define auto register __auto_type
#define cleanup(func) __attribute__((cleanup(func)))
#define overload __attribute__((overloadable))


#define __builtin_add_overflow_p(a,b,c) ({ \
__typeof__(c) __result; \
__builtin_add_overflow(a, b, &__result); \
})

#define __builtin_sub_overflow_p(a,b,c) ({ \
__typeof__(c) __result; \
__builtin_sub_overflow(a, b, &__result); \
})

#define __builtin_mul_overflow_p(a,b,c) ({ \
__typeof__(c) __result; \
__builtin_mul_overflow(a, b, &__result); \
})

#define add_overflows(a,b) __builtin_add_overflow_p(a, b, (__typeof__((a) + (b)))0)
#define sub_overflows(a,b) __builtin_sub_overflow_p(a, b, (__typeof__((a) - (b)))0)
#define mul_overflows(a,b) __builtin_mul_overflow_p(a, b, (__typeof__((a) * (b)))0)

#define add_overflow(a,b,dest) __builtin_add_overflow(a, b, dest)
#define sub_overflow(a,b,dest) __builtin_sub_overflow(a, b, dest)
#define mul_overflow(a,b,dest) __builtin_mul_overflow(a, b, dest)

typedef union gpr8_t {
  u8 u8[1];
  s8 s8[1];
} gpr8_t;

typedef union gpr16_t {
  u8 u8[2];
  s8 s8[2];
  u16 u16[1];
  s16 s16[1];
} gpr16_t;

typedef union gpr32_t {
  u8 u8[4];
  s8 s8[4];
  u16 u16[2];
  s16 s16[2];
  u32 u32[1];
  s32 s32[1];
} gpr32_t;

typedef union gpr64_t {
  u8 u8[8];
  s8 s8[8];
  u16 u16[4];
  s16 s16[4];
  u32 u32[2];
  s32 s32[2];
  u64 u64[1];
  s64 s64[1];
} gpr64_t;

typedef union gpr_t {
  u8 u8[16];
  s8 s8[16];
  u16 u16[8];
  s16 s16[8];
  u32 u32[4];
  s32 s32[4];
  u64 u64[2];
  s64 s64[2];
  u128 u128[1];
  s128 s128[1];
} gpr_t [[gnu::aligned(16)]];

#define switch4(switch_, c0, c1, c2, c3, default_) \
switch(switch_) { \
case 0: c0; break;\
case 1: c1; break;\
case 2: c2; break;\
case 3: c3; break;\
default: default_; break;\
}

#define switch8(switch_, c0, c1, c2, c3, c4, c5, c6, c7, default_) \
switch(switch_) { \
case 0: c0; break;\
case 1: c1; break;\
case 2: c2; break;\
case 3: c3; break;\
case 4: c4; break;\
case 5: c5; break;\
case 6: c6; break;\
case 7: c7; break;\
default: default_; break;\
}

#define switch16(switch_, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, default_) \
switch(switch_) { \
case 0: c0; break;\
case 1: c1; break;\
case 2: c2; break;\
case 3: c3; break;\
case 4: c4; break;\
case 5: c5; break;\
case 6: c6; break;\
case 7: c7; break;\
case 8: c8; break;\
case 9: c9; break;\
case 10: c10; break;\
case 11: c11; break;\
case 12: c12; break;\
case 13: c13; break;\
case 14: c14; break;\
case 15: c15; break;\
default: default_; break;\
}

#define switch32(switch_, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, default_) \
switch(switch_) { \
case 0: c0; break;\
case 1: c1; break;\
case 2: c2; break;\
case 3: c3; break;\
case 4: c4; break;\
case 5: c5; break;\
case 6: c6; break;\
case 7: c7; break;\
case 8: c8; break;\
case 9: c9; break;\
case 10: c10; break;\
case 11: c11; break;\
case 12: c12; break;\
case 13: c13; break;\
case 14: c14; break;\
case 15: c15; break;\
case 16: c16; break;\
case 17: c17; break;\
case 18: c18; break;\
case 19: c19; break;\
case 20: c20; break;\
case 21: c21; break;\
case 22: c22; break;\
case 23: c23; break;\
case 24: c24; break;\
case 25: c25; break;\
case 26: c26; break;\
case 27: c27; break;\
case 28: c28; break;\
case 29: c29; break;\
case 30: c30; break;\
case 31: c31; break;\
default: default_; break;\
}

#define switch64(switch_, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, default_) \
switch(switch_) { \
case 0: c0; break;\
case 1: c1; break;\
case 2: c2; break;\
case 3: c3; break;\
case 4: c4; break;\
case 5: c5; break;\
case 6: c6; break;\
case 7: c7; break;\
case 8: c8; break;\
case 9: c9; break;\
case 10: c10; break;\
case 11: c11; break;\
case 12: c12; break;\
case 13: c13; break;\
case 14: c14; break;\
case 15: c15; break;\
case 16: c16; break;\
case 17: c17; break;\
case 18: c18; break;\
case 19: c19; break;\
case 20: c20; break;\
case 21: c21; break;\
case 22: c22; break;\
case 23: c23; break;\
case 24: c24; break;\
case 25: c25; break;\
case 26: c26; break;\
case 27: c27; break;\
case 28: c28; break;\
case 29: c29; break;\
case 30: c30; break;\
case 31: c31; break;\
case 32: c32; break;\
case 33: c33; break;\
case 34: c34; break;\
case 35: c35; break;\
case 36: c36; break;\
case 37: c37; break;\
case 38: c38; break;\
case 39: c39; break;\
case 40: c40; break;\
case 41: c41; break;\
case 42: c42; break;\
case 43: c43; break;\
case 44: c44; break;\
case 45: c45; break;\
case 46: c46; break;\
case 47: c47; break;\
case 48: c48; break;\
case 49: c49; break;\
case 50: c50; break;\
case 51: c51; break;\
case 52: c52; break;\
case 53: c53; break;\
case 54: c54; break;\
case 55: c55; break;\
case 56: c56; break;\
case 57: c57; break;\
case 58: c58; break;\
case 59: c59; break;\
case 60: c60; break;\
case 61: c61; break;\
case 62: c62; break;\
case 63: c63; break;\
default: default_; break;\
}

#define switch128(switch_, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, c64, c65, c66, c67, c68, c69, c70, c71, c72, c73, c74, c75, c76, c77, c78, c79, c80, c81, c82, c83, c84, c85, c86, c87, c88, c89, c90, c91, c92, c93, c94, c95, c96, c97, c98, c99, c100, c101, c102, c103, c104, c105, c106, c107, c108, c109, c110, c111, c112, c113, c114, c115, c116, c117, c118, c119, c120, c121, c122, c123, c124, c125, c126, c127, default_) \
 switch(switch_) { \
   case 0: c0; break;\
   case 1: c1; break;\
   case 2: c2; break;\
   case 3: c3; break;\
   case 4: c4; break;\
   case 5: c5; break;\
   case 6: c6; break;\
   case 7: c7; break;\
   case 8: c8; break;\
   case 9: c9; break;\
   case 10: c10; break;\
   case 11: c11; break;\
   case 12: c12; break;\
   case 13: c13; break;\
   case 14: c14; break;\
   case 15: c15; break;\
   case 16: c16; break;\
   case 17: c17; break;\
   case 18: c18; break;\
   case 19: c19; break;\
   case 20: c20; break;\
   case 21: c21; break;\
   case 22: c22; break;\
   case 23: c23; break;\
   case 24: c24; break;\
   case 25: c25; break;\
   case 26: c26; break;\
   case 27: c27; break;\
   case 28: c28; break;\
   case 29: c29; break;\
   case 30: c30; break;\
   case 31: c31; break;\
   case 32: c32; break;\
   case 33: c33; break;\
   case 34: c34; break;\
   case 35: c35; break;\
   case 36: c36; break;\
   case 37: c37; break;\
   case 38: c38; break;\
   case 39: c39; break;\
   case 40: c40; break;\
   case 41: c41; break;\
   case 42: c42; break;\
   case 43: c43; break;\
   case 44: c44; break;\
   case 45: c45; break;\
   case 46: c46; break;\
   case 47: c47; break;\
   case 48: c48; break;\
   case 49: c49; break;\
   case 50: c50; break;\
   case 51: c51; break;\
   case 52: c52; break;\
   case 53: c53; break;\
   case 54: c54; break;\
   case 55: c55; break;\
   case 56: c56; break;\
   case 57: c57; break;\
   case 58: c58; break;\
   case 59: c59; break;\
   case 60: c60; break;\
   case 61: c61; break;\
   case 62: c62; break;\
   case 63: c63; break;\
   case 64: c64; break;\
   case 65: c65; break;\
   case 66: c66; break;\
   case 67: c67; break;\
   case 68: c68; break;\
   case 69: c69; break;\
   case 70: c70; break;\
   case 71: c71; break;\
   case 72: c72; break;\
   case 73: c73; break;\
   case 74: c74; break;\
   case 75: c75; break;\
   case 76: c76; break;\
   case 77: c77; break;\
   case 78: c78; break;\
   case 79: c79; break;\
   case 80: c80; break;\
   case 81: c81; break;\
   case 82: c82; break;\
   case 83: c83; break;\
   case 84: c84; break;\
   case 85: c85; break;\
   case 86: c86; break;\
   case 87: c87; break;\
   case 88: c88; break;\
   case 89: c89; break;\
   case 90: c90; break;\
   case 91: c91; break;\
   case 92: c92; break;\
   case 93: c93; break;\
   case 94: c94; break;\
   case 95: c95; break;\
   case 96: c96; break;\
   case 97: c97; break;\
   case 98: c98; break;\
   case 99: c99; break;\
   case 100: c100; break;\
   case 101: c101; break;\
   case 102: c102; break;\
   case 103: c103; break;\
   case 104: c104; break;\
   case 105: c105; break;\
   case 106: c106; break;\
   case 107: c107; break;\
   case 108: c108; break;\
   case 109: c109; break;\
   case 110: c110; break;\
   case 111: c111; break;\
   case 112: c112; break;\
   case 113: c113; break;\
   case 114: c114; break;\
   case 115: c115; break;\
   case 116: c116; break;\
   case 117: c117; break;\
   case 118: c118; break;\
   case 119: c119; break;\
   case 120: c120; break;\
   case 121: c121; break;\
   case 122: c122; break;\
   case 123: c123; break;\
   case 124: c124; break;\
   case 125: c125; break;\
   case 126: c126; break;\
   case 127: c127; break;\
   default: default_; break;\
 }

#define switch256(switch_, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, c64, c65, c66, c67, c68, c69, c70, c71, c72, c73, c74, c75, c76, c77, c78, c79, c80, c81, c82, c83, c84, c85, c86, c87, c88, c89, c90, c91, c92, c93, c94, c95, c96, c97, c98, c99, c100, c101, c102, c103, c104, c105, c106, c107, c108, c109, c110, c111, c112, c113, c114, c115, c116, c117, c118, c119, c120, c121, c122, c123, c124, c125, c126, c127, c128, c129, c130, c131, c132, c133, c134, c135, c136, c137, c138, c139, c140, c141, c142, c143, c144, c145, c146, c147, c148, c149, c150, c151, c152, c153, c154, c155, c156, c157, c158, c159, c160, c161, c162, c163, c164, c165, c166, c167, c168, c169, c170, c171, c172, c173, c174, c175, c176, c177, c178, c179, c180, c181, c182, c183, c184, c185, c186, c187, c188, c189, c190, c191, c192, c193, c194, c195, c196, c197, c198, c199, c200, c201, c202, c203, c204, c205, c206, c207, c208, c209, c210, c211, c212, c213, c214, c215, c216, c217, c218, c219, c220, c221, c222, c223, c224, c225, c226, c227, c228, c229, c230, c231, c232, c233, c234, c235, c236, c237, c238, c239, c240, c241, c242, c243, c244, c245, c246, c247, c248, c249, c250, c251, c252, c253, c254, c255, default_) \
 switch(switch_) { \
   case 0: c0; break;\
   case 1: c1; break;\
   case 2: c2; break;\
   case 3: c3; break;\
   case 4: c4; break;\
   case 5: c5; break;\
   case 6: c6; break;\
   case 7: c7; break;\
   case 8: c8; break;\
   case 9: c9; break;\
   case 10: c10; break;\
   case 11: c11; break;\
   case 12: c12; break;\
   case 13: c13; break;\
   case 14: c14; break;\
   case 15: c15; break;\
   case 16: c16; break;\
   case 17: c17; break;\
   case 18: c18; break;\
   case 19: c19; break;\
   case 20: c20; break;\
   case 21: c21; break;\
   case 22: c22; break;\
   case 23: c23; break;\
   case 24: c24; break;\
   case 25: c25; break;\
   case 26: c26; break;\
   case 27: c27; break;\
   case 28: c28; break;\
   case 29: c29; break;\
   case 30: c30; break;\
   case 31: c31; break;\
   case 32: c32; break;\
   case 33: c33; break;\
   case 34: c34; break;\
   case 35: c35; break;\
   case 36: c36; break;\
   case 37: c37; break;\
   case 38: c38; break;\
   case 39: c39; break;\
   case 40: c40; break;\
   case 41: c41; break;\
   case 42: c42; break;\
   case 43: c43; break;\
   case 44: c44; break;\
   case 45: c45; break;\
   case 46: c46; break;\
   case 47: c47; break;\
   case 48: c48; break;\
   case 49: c49; break;\
   case 50: c50; break;\
   case 51: c51; break;\
   case 52: c52; break;\
   case 53: c53; break;\
   case 54: c54; break;\
   case 55: c55; break;\
   case 56: c56; break;\
   case 57: c57; break;\
   case 58: c58; break;\
   case 59: c59; break;\
   case 60: c60; break;\
   case 61: c61; break;\
   case 62: c62; break;\
   case 63: c63; break;\
   case 64: c64; break;\
   case 65: c65; break;\
   case 66: c66; break;\
   case 67: c67; break;\
   case 68: c68; break;\
   case 69: c69; break;\
   case 70: c70; break;\
   case 71: c71; break;\
   case 72: c72; break;\
   case 73: c73; break;\
   case 74: c74; break;\
   case 75: c75; break;\
   case 76: c76; break;\
   case 77: c77; break;\
   case 78: c78; break;\
   case 79: c79; break;\
   case 80: c80; break;\
   case 81: c81; break;\
   case 82: c82; break;\
   case 83: c83; break;\
   case 84: c84; break;\
   case 85: c85; break;\
   case 86: c86; break;\
   case 87: c87; break;\
   case 88: c88; break;\
   case 89: c89; break;\
   case 90: c90; break;\
   case 91: c91; break;\
   case 92: c92; break;\
   case 93: c93; break;\
   case 94: c94; break;\
   case 95: c95; break;\
   case 96: c96; break;\
   case 97: c97; break;\
   case 98: c98; break;\
   case 99: c99; break;\
   case 100: c100; break;\
   case 101: c101; break;\
   case 102: c102; break;\
   case 103: c103; break;\
   case 104: c104; break;\
   case 105: c105; break;\
   case 106: c106; break;\
   case 107: c107; break;\
   case 108: c108; break;\
   case 109: c109; break;\
   case 110: c110; break;\
   case 111: c111; break;\
   case 112: c112; break;\
   case 113: c113; break;\
   case 114: c114; break;\
   case 115: c115; break;\
   case 116: c116; break;\
   case 117: c117; break;\
   case 118: c118; break;\
   case 119: c119; break;\
   case 120: c120; break;\
   case 121: c121; break;\
   case 122: c122; break;\
   case 123: c123; break;\
   case 124: c124; break;\
   case 125: c125; break;\
   case 126: c126; break;\
   case 127: c127; break;\
   case 128: c128; break;\
   case 129: c129; break;\
   case 130: c130; break;\
   case 131: c131; break;\
   case 132: c132; break;\
   case 133: c133; break;\
   case 134: c134; break;\
   case 135: c135; break;\
   case 136: c136; break;\
   case 137: c137; break;\
   case 138: c138; break;\
   case 139: c139; break;\
   case 140: c140; break;\
   case 141: c141; break;\
   case 142: c142; break;\
   case 143: c143; break;\
   case 144: c144; break;\
   case 145: c145; break;\
   case 146: c146; break;\
   case 147: c147; break;\
   case 148: c148; break;\
   case 149: c149; break;\
   case 150: c150; break;\
   case 151: c151; break;\
   case 152: c152; break;\
   case 153: c153; break;\
   case 154: c154; break;\
   case 155: c155; break;\
   case 156: c156; break;\
   case 157: c157; break;\
   case 158: c158; break;\
   case 159: c159; break;\
   case 160: c160; break;\
   case 161: c161; break;\
   case 162: c162; break;\
   case 163: c163; break;\
   case 164: c164; break;\
   case 165: c165; break;\
   case 166: c166; break;\
   case 167: c167; break;\
   case 168: c168; break;\
   case 169: c169; break;\
   case 170: c170; break;\
   case 171: c171; break;\
   case 172: c172; break;\
   case 173: c173; break;\
   case 174: c174; break;\
   case 175: c175; break;\
   case 176: c176; break;\
   case 177: c177; break;\
   case 178: c178; break;\
   case 179: c179; break;\
   case 180: c180; break;\
   case 181: c181; break;\
   case 182: c182; break;\
   case 183: c183; break;\
   case 184: c184; break;\
   case 185: c185; break;\
   case 186: c186; break;\
   case 187: c187; break;\
   case 188: c188; break;\
   case 189: c189; break;\
   case 190: c190; break;\
   case 191: c191; break;\
   case 192: c192; break;\
   case 193: c193; break;\
   case 194: c194; break;\
   case 195: c195; break;\
   case 196: c196; break;\
   case 197: c197; break;\
   case 198: c198; break;\
   case 199: c199; break;\
   case 200: c200; break;\
   case 201: c201; break;\
   case 202: c202; break;\
   case 203: c203; break;\
   case 204: c204; break;\
   case 205: c205; break;\
   case 206: c206; break;\
   case 207: c207; break;\
   case 208: c208; break;\
   case 209: c209; break;\
   case 210: c210; break;\
   case 211: c211; break;\
   case 212: c212; break;\
   case 213: c213; break;\
   case 214: c214; break;\
   case 215: c215; break;\
   case 216: c216; break;\
   case 217: c217; break;\
   case 218: c218; break;\
   case 219: c219; break;\
   case 220: c220; break;\
   case 221: c221; break;\
   case 222: c222; break;\
   case 223: c223; break;\
   case 224: c224; break;\
   case 225: c225; break;\
   case 226: c226; break;\
   case 227: c227; break;\
   case 228: c228; break;\
   case 229: c229; break;\
   case 230: c230; break;\
   case 231: c231; break;\
   case 232: c232; break;\
   case 233: c233; break;\
   case 234: c234; break;\
   case 235: c235; break;\
   case 236: c236; break;\
   case 237: c237; break;\
   case 238: c238; break;\
   case 239: c239; break;\
   case 240: c240; break;\
   case 241: c241; break;\
   case 242: c242; break;\
   case 243: c243; break;\
   case 244: c244; break;\
   case 245: c245; break;\
   case 246: c246; break;\
   case 247: c247; break;\
   case 248: c248; break;\
   case 249: c249; break;\
   case 250: c250; break;\
   case 251: c251; break;\
   case 252: c252; break;\
   case 253: c253; break;\
   case 254: c254; break;\
   case 255: c255; break;\
   default: default_; break;\
 }