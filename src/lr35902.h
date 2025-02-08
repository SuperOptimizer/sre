#pragma once

#include "speedrun.h"

#define FLAG_Z (1 << 7)
#define FLAG_N (1 << 6)
#define FLAG_H (1 << 5)
#define FLAG_C (1 << 4)

typedef enum reg_e {
  REG_NONE = -1,
  B = 0,
  C = 1,
  D = 2,
  E = 3,
  H = 4,
  L = 5,
  F = 6,
  A = 7,
  BC = 8,
  DE = 9,
  HL = 10,
  SP = 11,
  AF = 12,
} reg_e;

typedef enum cond_e {
  COND_NONE = -1,
  COND_NZ = 0,
  COND_Z = 1,
  COND_NC = 2,
  COND_C = 3,
} cond_e;


typedef enum instr_e {
  INVALID, LD, LDI, LDD, PUSH, POP, ADD, ADC, SUB, SBC, AND, OR, XOR, CP, INC, DEC, RLCA, RLA, RRCA, RRA, RLC, RL, RRC,
  RR, SLA, SRA, SRL, BIT, SET, RES, JP, JP_CC, JR, JR_CC, CALL, CALL_CC, RET, RET_CC, RETI, RST_N, CCF, SCF, NOP, HALT,
  STOP, DI, EI, DAA, CPL,
} instr_e;

struct lr35902_t {
  union {
    u16 r16[4];
    u8 r8[8];

    struct {
      u8 b, c, d, e, h, l, f, a;
    };

    struct {
      u16 bc, de, hl, af;
    };
  };

  gpr16_t sp, pc;

  u8 wram[8192]; // 8KB Work RAM
  u8 hram[127]; // High RAM
  u8 vram[8192]; // 8KB Video RAM
  u8 oam[160]; // Object Attribute Memory
  u8 cart_rom[32768]; // Up to 32KB ROM (can be banked)
  u8 cart_ram[8192]; // Up to 8KB Cart RAM

  // Interrupt registers
  u8 ie; // Interrupt Enable
  u8 if_; // Interrupt Flags

  // CPU state
  bool ime; // Interrupt Master Enable flag
  bool halted; // CPU HALT state
  bool stopped; // CPU STOP state

  // Timer registers
  u8 div; // Divider
  u8 tima; // Timer counter
  u8 tma; // Timer modulo
  u8 tac; // Timer control

  // LCD registers
  u8 lcdc; // LCD Control
  u8 stat; // LCD Status
  u8 scy; // Scroll Y
  u8 scx; // Scroll X
  u8 ly; // LCD Y-Coordinate
  u8 lyc; // LY Compare
  u8 bgp; // BG Palette Data
  u8 obp0; // Object Palette 0 Data
  u8 obp1; // Object Palette 1 Data
  u8 wy; // Window Y Position
  u8 wx; // Window X Position minus 7
};

typedef enum operand_type_e {
  R8, R16, I8, I16, INDIRECT
} operand_type_e;

typedef struct instr_t {
  instr_e inst;
  char *name;
  operand_type_e src_type;
  operand_type_e dst_type;
  union {
    u8 uimm8; s8 simm8;
  };
  union {
    u16 uimm16; s16 simm16;
  };
  reg_e src_reg, dst_reg;
} instr_t;


#define LD_R8_I8(reg) result = (instr_t){.inst = LD, .dst_type = R8, .src_type = I8, .dst_reg = reg}
#define LD_R16_I16(reg) result = (instr_t){.inst = LD, .dst_type = R16, .src_type = I16, .dst_reg = reg}
#define LD_IND_R8(ind,src) result = (instr_t){.inst = LD, .dst_type = INDIRECT, .src_type = R8, .dst_reg = ind, .src_reg = src}
#define LD_R8_IND(dst,ind) result = (instr_t){.inst = LD, .dst_type = R8, .src_type = INDIRECT, .dst_reg = dst, .src_reg = ind}
#define INC_R8(reg) result = (instr_t){.inst = INC, .dst_type = R8, .dst_reg = reg}
#define INC_R16(reg) result = (instr_t){.inst = INC, .dst_type = R16, .dst_reg = reg}
#define DEC_R8(reg) result = (instr_t){.inst = DEC, .dst_type = R8, .dst_reg = reg}
#define DEC_R16(reg) result = (instr_t){.inst = DEC, .dst_type = R16, .dst_reg = reg}
#define ADD_R16(src) result = (instr_t){.inst = ADD, .dst_type = R16, .src_type = R16, .dst_reg = HL, .src_reg = src}

#define ALU_ADD(src) result = (instr_t){.inst = ADD, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_ADC(src) result = (instr_t){.inst = ADC, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_SUB(src) result = (instr_t){.inst = SUB, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_SBC(src) result = (instr_t){.inst = SBC, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_AND(src) result = (instr_t){.inst = AND, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_XOR(src) result = (instr_t){.inst = XOR, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_OR(src) result = (instr_t){.inst = OR, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}
#define ALU_CP(src) result = (instr_t){.inst = CP, .dst_type = R8, .src_type = R8, .dst_reg = A, .src_reg = src}

#define ALU_IMM(op) result = (instr_t){.inst = op, .dst_type = R8, .src_type = I8, .dst_reg = A}
#define RST(n) result = (instr_t){.inst = RST_N, .src_reg = n}
#define PUSH_R16(reg) result = (instr_t){.inst = PUSH, .src_type = R16, .src_reg = reg}
#define POP_R16(reg) result = (instr_t){.inst = POP, .dst_type = R16, .dst_reg = reg}

#define INSTR_NOP result = (instr_t){.inst = NOP}
#define INSTR_RLCA result = (instr_t){.inst = RLCA}
#define INSTR_RRCA result = (instr_t){.inst = RRCA}
#define INSTR_RLA result = (instr_t){.inst = RLA}
#define INSTR_RRA result = (instr_t){.inst = RRA}
#define INSTR_DAA result = (instr_t){.inst = DAA}
#define INSTR_CPL result = (instr_t){.inst = CPL}
#define INSTR_SCF result = (instr_t){.inst = SCF}
#define INSTR_CCF result = (instr_t){.inst = CCF}
#define INSTR_HALT result = (instr_t){.inst = HALT}
#define INSTR_STOP result = (instr_t){.inst = STOP}
#define INSTR_DI result = (instr_t){.inst = DI}
#define INSTR_EI result = (instr_t){.inst = EI}
#define INSTR_RETI result = (instr_t){.inst = RETI}
#define INSTR_RET result = (instr_t){.inst = RET}
#define INSTR_RET_CC result = (instr_t){.inst = RET_CC}
#define INSTR_JP_CC_I16 result = (instr_t){.inst = JP_CC, .src_type = I16}
#define INSTR_JP_I16 result = (instr_t){.inst = JP, .src_type = I16}
#define INSTR_JR_I8 result = (instr_t){.inst = JR, .src_type = I8}
#define INSTR_JR_CC_I8 result = (instr_t){.inst = JR_CC, .src_type = I8}
#define INSTR_CALL_I16 result = (instr_t){.inst = CALL, .src_type = I16}
#define INSTR_CALL_CC_I16 result = (instr_t){.inst = CALL_CC, .src_type = I16}
#define INSTR_INVALID result = (instr_t){.inst = INVALID}

#define CB_INSTR(type) result = (instr_t){.inst = type, .dst_type = reg == 6 ? INDIRECT : R8, .dest = reg}
#define CB_BIT(n) result = (instr_t){.inst = BIT, .dst_type = reg == 6 ? INDIRECT : R8, .dest = reg, .src = n}
#define CB_RES(n) result = (instr_t){.inst = RES, .dst_type = reg == 6 ? INDIRECT : R8, .dest = reg, .src = n}
#define CB_SET(n) result = (instr_t){.inst = SET, .dst_type = reg == 6 ? INDIRECT : R8, .dest = reg, .src = n}

#define REG_LOAD(dst,src) (instr_t){.inst = LD, .dst_type = R8, .src_type = src == 6 ? INDIRECT : R8, .dest = dst, .src = src}
#define ALU_OP(op,src) (instr_t){.inst = op, .dst_type = R8, .src_type = src == 6 ? INDIRECT : R8, .dest = A, .src = src}


instr_t lr35902_decode(u8 opcode, bool is_cb) {
 if (is_cb) {
   u8 reg = opcode & 0x07;

   instr_t result;
   switch256(opcode,
     CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC), CB_INSTR(RLC),
     CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC), CB_INSTR(RRC),
     CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL), CB_INSTR(RL),
     CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR), CB_INSTR(RR),
     CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA), CB_INSTR(SLA),
     CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA), CB_INSTR(SRA),
     CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL),
     CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL), CB_INSTR(SRL),

     CB_BIT(0), CB_BIT(0), CB_BIT(0), CB_BIT(0), CB_BIT(0), CB_BIT(0), CB_BIT(0), CB_BIT(0),
     CB_BIT(1), CB_BIT(1), CB_BIT(1), CB_BIT(1), CB_BIT(1), CB_BIT(1), CB_BIT(1), CB_BIT(1),
     CB_BIT(2), CB_BIT(2), CB_BIT(2), CB_BIT(2), CB_BIT(2), CB_BIT(2), CB_BIT(2), CB_BIT(2),
     CB_BIT(3), CB_BIT(3), CB_BIT(3), CB_BIT(3), CB_BIT(3), CB_BIT(3), CB_BIT(3), CB_BIT(3),
     CB_BIT(4), CB_BIT(4), CB_BIT(4), CB_BIT(4), CB_BIT(4), CB_BIT(4), CB_BIT(4), CB_BIT(4),
     CB_BIT(5), CB_BIT(5), CB_BIT(5), CB_BIT(5), CB_BIT(5), CB_BIT(5), CB_BIT(5), CB_BIT(5),
     CB_BIT(6), CB_BIT(6), CB_BIT(6), CB_BIT(6), CB_BIT(6), CB_BIT(6), CB_BIT(6), CB_BIT(6),
     CB_BIT(7), CB_BIT(7), CB_BIT(7), CB_BIT(7), CB_BIT(7), CB_BIT(7), CB_BIT(7), CB_BIT(7),

     CB_RES(0), CB_RES(0), CB_RES(0), CB_RES(0), CB_RES(0), CB_RES(0), CB_RES(0), CB_RES(0),
     CB_RES(1), CB_RES(1), CB_RES(1), CB_RES(1), CB_RES(1), CB_RES(1), CB_RES(1), CB_RES(1),
     CB_RES(2), CB_RES(2), CB_RES(2), CB_RES(2), CB_RES(2), CB_RES(2), CB_RES(2), CB_RES(2),
     CB_RES(3), CB_RES(3), CB_RES(3), CB_RES(3), CB_RES(3), CB_RES(3), CB_RES(3), CB_RES(3),
     CB_RES(4), CB_RES(4), CB_RES(4), CB_RES(4), CB_RES(4), CB_RES(4), CB_RES(4), CB_RES(4),
     CB_RES(5), CB_RES(5), CB_RES(5), CB_RES(5), CB_RES(5), CB_RES(5), CB_RES(5), CB_RES(5),
     CB_RES(6), CB_RES(6), CB_RES(6), CB_RES(6), CB_RES(6), CB_RES(6), CB_RES(6), CB_RES(6),
     CB_RES(7), CB_RES(7), CB_RES(7), CB_RES(7), CB_RES(7), CB_RES(7), CB_RES(7), CB_RES(7),

     CB_SET(0), CB_SET(0), CB_SET(0), CB_SET(0), CB_SET(0), CB_SET(0), CB_SET(0), CB_SET(0),
     CB_SET(1), CB_SET(1), CB_SET(1), CB_SET(1), CB_SET(1), CB_SET(1), CB_SET(1), CB_SET(1),
     CB_SET(2), CB_SET(2), CB_SET(2), CB_SET(2), CB_SET(2), CB_SET(2), CB_SET(2), CB_SET(2),
     CB_SET(3), CB_SET(3), CB_SET(3), CB_SET(3), CB_SET(3), CB_SET(3), CB_SET(3), CB_SET(3),
     CB_SET(4), CB_SET(4), CB_SET(4), CB_SET(4), CB_SET(4), CB_SET(4), CB_SET(4), CB_SET(4),
     CB_SET(5), CB_SET(5), CB_SET(5), CB_SET(5), CB_SET(5), CB_SET(5), CB_SET(5), CB_SET(5),
     CB_SET(6), CB_SET(6), CB_SET(6), CB_SET(6), CB_SET(6), CB_SET(6), CB_SET(6), CB_SET(6),
     CB_SET(7), CB_SET(7), CB_SET(7), CB_SET(7), CB_SET(7), CB_SET(7), CB_SET(7), CB_SET(7),
     INSTR_INVALID);

 }

 switch256(opcode,
   INSTR_NOP,
   LD_R16_I16(BC),
   LD_IND_R8(BC, A),
   INC_R16(BC),
   INC_R8(B),
   DEC_R8(B),
   LD_R8_I8(B),
   INSTR_RLCA,
   (instr_t){.inst = LD, .dst_type = INDIRECT, .src_type = R16, .src_reg = SP},
   ADD_R16(BC),
   LD_R8_IND(A, BC),
   DEC_R16(BC),
   INC_R8(C),
   DEC_R8(C),
   LD_R8_I8(C),
   INSTR_RRCA,

   INSTR_STOP,
   LD_R16_I16(DE),
   LD_IND_R8(DE, A),
   INC_R16(DE),
   INC_R8(D),
   DEC_R8(D),
   LD_R8_I8(D),
   INSTR_RLA,
   INSTR_JR_I8,
   ADD_R16(DE),
   LD_R8_IND(A, DE),
   DEC_R16(DE),
   INC_R8(E),
   DEC_R8(E),
   LD_R8_I8(E),
   INSTR_RRA,

   INSTR_JR_CC_I8,
   LD_R16_I16(HL),
   (instr_t){.inst = LDI, .dst_type = INDIRECT, .src_type = R8, .dst_reg = HL, .src_reg = A},
   INC_R16(HL),
   INC_R8(H),
   DEC_R8(H),
   LD_R8_I8(H),INSTR_DAA,
   INSTR_JR_CC_I8,
   ADD_R16(HL),
   (instr_t){.inst = LDI, .dst_type = R8, .src_type = INDIRECT, .dst_reg = A, .src_reg = HL},
   DEC_R16(HL),
   INC_R8(L),
   DEC_R8(L),
   LD_R8_I8(L),
   INSTR_CPL,

   INSTR_JR_CC_I8,
   LD_R16_I16(SP),
   (instr_t){.inst = LDD, .dst_type = INDIRECT, .src_type = R8, .dst_reg = HL, .src_reg = A},
   INC_R16(SP),
   (instr_t){.inst = INC, .dst_type = INDIRECT, .dst_reg = HL},
   (instr_t){.inst = DEC, .dst_type = INDIRECT, .dst_reg = HL},
   (instr_t){.inst = LD, .dst_type = INDIRECT, .src_type = I8, .dst_reg = HL},
   INSTR_SCF,
   INSTR_JR_CC_I8,
   ADD_R16(SP),
   (instr_t){.inst = LDD, .dst_type = R8, .src_type = INDIRECT, .dst_reg = A, .src_reg = HL},
   DEC_R16(SP),
   INC_R8(A),
   DEC_R8(A),
   LD_R8_I8(A),
   INSTR_CCF,

   LD_R8_I8(B), LD_R8_I8(C), LD_R8_I8(D), LD_R8_I8(E),
   LD_R8_I8(H), LD_R8_I8(L), INSTR_HALT, LD_R8_I8(A),
   LD_R8_I8(B), LD_R8_I8(C), LD_R8_I8(D), LD_R8_I8(E),
   LD_R8_I8(H), LD_R8_I8(L), INSTR_HALT, LD_R8_I8(A),

   ALU_ADD(B), ALU_ADD(C), ALU_ADD(D), ALU_ADD(E),
   ALU_ADD(H), ALU_ADD(L), ALU_ADD(HL), ALU_ADD(A),
   ALU_ADC(B), ALU_ADC(C), ALU_ADC(D), ALU_ADC(E),
   ALU_ADC(H), ALU_ADC(L), ALU_ADC(HL), ALU_ADC(A),
   ALU_SUB(B), ALU_SUB(C), ALU_SUB(D), ALU_SUB(E),
   ALU_SUB(H), ALU_SUB(L), ALU_SUB(HL), ALU_SUB(A),
   ALU_SBC(B), ALU_SBC(C), ALU_SBC(D), ALU_SBC(E),
   ALU_SBC(H), ALU_SBC(L), ALU_SBC(HL), ALU_SBC(A),
   ALU_AND(B), ALU_AND(C), ALU_AND(D), ALU_AND(E),
   ALU_AND(H), ALU_AND(L), ALU_AND(HL), ALU_AND(A),
   ALU_XOR(B), ALU_XOR(C), ALU_XOR(D), ALU_XOR(E),
   ALU_XOR(H), ALU_XOR(L), ALU_XOR(HL), ALU_XOR(A),
   ALU_OR(B), ALU_OR(C), ALU_OR(D), ALU_OR(E),
   ALU_OR(H), ALU_OR(L), ALU_OR(HL), ALU_OR(A),
   ALU_CP(B), ALU_CP(C), ALU_CP(D), ALU_CP(E),
   ALU_CP(H), ALU_CP(L), ALU_CP(HL), ALU_CP(A),

   INSTR_RET_CC,
   POP_R16(BC),
   INSTR_JP_CC_I16,
   INSTR_JP_I16,
   INSTR_CALL_CC_I16,
   PUSH_R16(BC),
   ALU_IMM(ADD),
   RST(0x00),
   INSTR_RET_CC,
   INSTR_RET,
   INSTR_JP_CC_I16,
   INSTR_INVALID,
   INSTR_CALL_CC_I16,
   INSTR_CALL_I16,
   ALU_IMM(ADC),
   RST(0x08),

   INSTR_RET_CC,
   POP_R16(DE),
   INSTR_JP_CC_I16,
   INSTR_INVALID,
   INSTR_CALL_CC_I16,
   PUSH_R16(DE),
   ALU_IMM(SUB),
   RST(0x10),
   INSTR_RET_CC,
   INSTR_RETI,
   INSTR_JP_CC_I16,
   INSTR_INVALID,
   INSTR_CALL_CC_I16,
   INSTR_INVALID,
   ALU_IMM(SBC),
   RST(0x18),

   LD_IND_R8(0xFF00, A),
   POP_R16(HL),
   LD_IND_R8(C, A),
   INSTR_INVALID,
   INSTR_INVALID,
   PUSH_R16(HL),
   ALU_IMM(AND),
   RST(0x20),
   ADD_R16(SP),
   (instr_t){.inst = JP, .src_type = R16, .src_reg = HL},
   (instr_t){.inst = LD, .dst_type = INDIRECT, .src_type = R8, .src_reg = A},
   INSTR_INVALID,
   INSTR_INVALID,
   INSTR_INVALID,
   ALU_IMM(XOR),
   RST(0x28),

   LD_R8_IND(A, 0xFF00),
   POP_R16(AF),
   LD_R8_IND(A, C),
   INSTR_DI,
   INSTR_INVALID,
   PUSH_R16(AF),
   ALU_IMM(OR),
   RST(0x30),
   (instr_t){.inst = LD, .dst_type = R16, .src_type = R16, .dst_reg = HL, .src_reg = SP},
   (instr_t){.inst = LD, .dst_type = R16, .src_type = R16, .dst_reg = SP, .src_reg = HL},
   (instr_t){.inst = LD, .dst_type = R8, .src_type = INDIRECT, .dst_reg = A},
   INSTR_EI,
   INSTR_INVALID,
   INSTR_INVALID,
   ALU_IMM(CP),
   RST(0x38),
   INSTR_INVALID);

 return result;
}