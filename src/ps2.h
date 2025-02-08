#pragma once

#include "speedrun.h"

typedef struct mips {
  gpr hi,lo;
    union {
        gpr gpr[32];
        struct {
            gpr zero, at, v0, v1, a0, a1, a2, a3;
            gpr t0, t1, t2, t3, t4, t5, t6, t7;
            gpr s0, s1, s2, s3, s4, s5, s6, s7;
            gpr t8, t9, k0, k1, gp, sp, fp, ra;
        };
    };
    bool is_r5900;
  u32 pc, nextpc;
  u8 ee_main_ram[32 * 1024 * 1024];  // 32MB main RAM
  u8 ee_scratch_pad[16 * 1024];       // 16KB scratchpad RAM
  u8 iop_ram[2 * 1024 * 1024];        // 2MB IOP RAM
  u8 bios[4 * 1024 * 1024];           // 4MB BIOS
} mips;

typedef struct ps2 {
  mips r3000, r5900;
} ps2;


typedef enum mips_instr_e {
  INVALID,
  //mem
  LB, LBU, LD, LDL, LDR, LH, LHU, LW, LWL, LWR, LWU, SB, SD, SDL, SDR, SH, SW, SWL, SWR,
  //alu imm
  ADDI, ADDIU, ANDI, DADDI, DADDIU, LUI, ORI, SLTI, SLTIU, XORI,
  //alu 3 op
  ADD, ADDU, AND, DADD, DADDU, DSUB, DSUBU, NOR, OR, SLT, SLTU, SUB, SUBU, XOR,
  // alu shift
  DSLL, DSLL32, DSLLV, DSRA, DSRA32, DSRAV, DSRL, DSRL32, DSRLV, SLL, SLLV, SRA, SRAV, SRL, SRLV,
  // alu mult div
  DIV, DIVU, MFHI, MFLO, MTHI, MTLO, MULT, MULTU,
  //branch jump
  J, JAL, JALR, JR, BEQ, BEQL, BGEZ, BGEZAL, BGEZALL, BGEZL, BGTZ, BGTZL, BLEZ, BLEZL, BLTZ, BLTZAL, BLTZALL, BLTZL, BNE, BNEL,
  //exception
  SYSCALL, BREAK, TGE, TGEU, TLT, TLTU, TEQ, TNE, TGEI, TGEIU, TLTI, TLTIU, TEQI, TNEI,
  //sync
  SYNC,
  //misc
  MOVN, MOVZ, PREF,
  //ee mult div
  MADD, MADDU, MULT1, MULTU1, DIV1, DIVU1, MADD1, MADDU1, MFHI1, MTHI1, MFLO1, MTLO1,
  // MMI alu
  PADDB, PSUBB, PADDH, PSUBH, PADDW, PSUBW, PADSBH, PADDSB, PSUBSB, PADDSH, PSUBSH, PADDSW, PSUBSW, PADDUB, PSUBUB, PADDUH, PSUBUH, PADDUW, PSUBUW,
  // MMI mul div
  PMULTW, PMULTUW, PDIVW, PDIVUW, PMADDW, PMADDUW, PMSUBW, PMFHI, PMTHI, PMFLO, PMTLO, PMULTH, PMADDH, PMSUBH, PMFHL, PMTHL, PHMADH, PHMSBH, PDIVBW,
  // MMI shift
  MFSA, MTSA, MTSAB, MTSAH, PSLLH, PSRLH, PSRAH, PSLLW, PSLLVW, PSRLW, PSRLVW, PSRAW, PSRAVW, QFSRV,
  // MMI misc
  PABSH, PABSW, PMAXH, PMINH, PMAXW, PMINW, PAND, POR, PNOR, PXOR,
  // MMI comp
  PCGTB, PCEQB, PCGTH, PCEQH, PCGTW, PCEQW, PLZCW,
  // MMI load store
  LQ, SQ,
  // MMI data rearrangement
  PPACB, PPACH, PINTEH, PPACW, PEXTUB, PEXTLB, PEXTUH, PEXTLH, PEXTUW, PEXTLW, PEXT5, PPAC5, PCPYH, PCPYLD, PCPYUD, PREVH, PINTH, PEXEH, PEXCH, PEXEW, PEXCW, PROT3W,
  // cop0
  BC0F, BC0FL, BC0T, BC0TL, CACHE, DI, EI, ERET, MFC0, MTC0, TLBR, TLBWI, TLBWR, TLBP,
  // fpu move
  LWC1, SWC1, MTC1, MFC1, MOV_S, CTC1, CFC1,
  // fpu cvt
  CVT_S_W, CVT_W_S,
  // fpu compute
  ADD_S, SUB_S, MUL_S, DIV_S, ABS_S, NEG_S, SQRT_S, ADDA_S, SUBA_S, MULA_S, MADD_S, MADDA_S, MSUB_S, MSUBA_S, RSQRT_S, MAX_S, MIN_S,
  // fpu compare
  C_LT_S, C_EQ_S, C_F_S, C_LE_S,
  //fpu branch
  BC1T, BC1F, BC1TL, BC1FL,
  //cop2
  BC2F, BC2FL, BC2T, BC2TL, CALLMS, CALLMSR, CFC2, CTC2, LQC2, SQC2, QMFC2, QMTC2, WAITQ,

} mips_instr_e;

typedef enum inst_type_e {
  R_TYPE, J_TYPE, I_TYPE
} inst_type_e;

typedef enum disasm_type_e {
  DISASM_NONE, DISASM_RD_RS_RT, DISASM_RT_RS_IMM, DISASM_RS_RT_OFFSET, DISASM_RS_OFFSET, DISASM_RS_RT,
  DISASM_RD_RT_SA, DISASM_RD_RT_RS, DISASM_TARGET, DISASM_RS, DISASM_RD_RS, DISASM_RT_OFFSET_BASE,
  DISASM_RT_IMM, DISASM_RD, DISASM_RS_IMM, DISASM_OFFSET, DISASM_RT_RD, DISASM_RT
} disasm_type_e;

typedef union opcode_t {
  u32 raw;
  struct [[gnu::packed]] { u8 op: 6; u8 rs: 5; u8 rt: 5; u8 rd: 5; u8 sa: 5; u8 func: 6; };
  struct [[gnu::packed]] { u8 _op: 6; u32 target:26; };
  struct [[gnu::packed]] { u8 __op: 6; u8 __rs: 5; u8 __rt: 5; union{u16 uimm; s16 simm;};};
} opcode_t [[gnu::packed]];

static_assert(sizeof(opcode_t) == 4);

typedef struct instr_t {
  opcode_t op;
  mips_instr_e inst;
  disasm_type_e disasm;
  char* name;
} instr_t;

static inline const char* mips_reg_name(int reg) {
  static const char* names[] = {
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};
  return (reg >= 0 && reg < 32) ? names[reg] : "invalid";
}


instr_t mips_decode(u32 opcode, u32 pc) {
  opcode_t decoded = (opcode_t){.raw=opcode};

  switch (decoded.op) {
    case 0: switch (decoded.func) {
      case 0:  return (instr_t){.op=decoded, .inst=SLL, .name="sll"};
      case 1:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 2:  return (instr_t){.op=decoded, .inst=SRL, .name="srl"};
      case 3:  return (instr_t){.op=decoded, .inst=SRA, .name="sra"};
      case 4:  return (instr_t){.op=decoded, .inst=SLLV, .name="sllv"};
      case 5:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 6:  return (instr_t){.op=decoded, .inst=SRLV, .name="srlv"};
      case 7:  return (instr_t){.op=decoded, .inst=SRAV, .name="srav"};
      case 8:  return (instr_t){.op=decoded, .inst=JR, .name="jr"};
      case 9:  return (instr_t){.op=decoded, .inst=JALR, .name="jalr"};
      case 10: return (instr_t){.op=decoded, .inst=MOVZ, .name="movz"};
      case 11: return (instr_t){.op=decoded, .inst=MOVN, .name="movn"};
      case 12: return (instr_t){.op=decoded, .inst=SYSCALL, .name="syscall"};
      case 13: return (instr_t){.op=decoded, .inst=BREAK, .name="break"};
      case 14: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 15: return (instr_t){.op=decoded, .inst=SYNC, .name="sync"};
      case 16: return (instr_t){.op=decoded, .inst=MFHI, .name="mfhi"};
      case 17: return (instr_t){.op=decoded, .inst=MTHI, .name="mthi"};
      case 18: return (instr_t){.op=decoded, .inst=MFLO, .name="mflo"};
      case 19: return (instr_t){.op=decoded, .inst=MTLO, .name="mtlo"};
      case 20: return (instr_t){.op=decoded, .inst=DSLLV, .name="dsllv"};
      case 21: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 22: return (instr_t){.op=decoded, .inst=DSRLV, .name="dsrlv"};
      case 23: return (instr_t){.op=decoded, .inst=DSRAV, .name="dsrav"};
      case 24: return (instr_t){.op=decoded, .inst=MULT, .name="mult"};
      case 25: return (instr_t){.op=decoded, .inst=MULTU, .name="multu"};
      case 26: return (instr_t){.op=decoded, .inst=DIV, .name="div"};
      case 27: return (instr_t){.op=decoded, .inst=DIVU, .name="divu"};
      case 28: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 29: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 30: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 31: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 32: return (instr_t){.op=decoded, .inst=ADD, .name="add"};
      case 33: return (instr_t){.op=decoded, .inst=ADDU, .name="addu"};
      case 34: return (instr_t){.op=decoded, .inst=SUB, .name="sub"};
      case 35: return (instr_t){.op=decoded, .inst=SUBU, .name="subu"};
      case 36: return (instr_t){.op=decoded, .inst=AND, .name="and"};
      case 37: return (instr_t){.op=decoded, .inst=OR, .name="or"};
      case 38: return (instr_t){.op=decoded, .inst=XOR, .name="xor"};
      case 39: return (instr_t){.op=decoded, .inst=NOR, .name="nor"};
      case 40: return (instr_t){.op=decoded, .inst=MFSA, .name="mfsa"};
      case 41: return (instr_t){.op=decoded, .inst=MTSA, .name="mtsa"};
      case 42: return (instr_t){.op=decoded, .inst=SLT, .name="slt"};
      case 43: return (instr_t){.op=decoded, .inst=SLTU, .name="sltu"};
      case 44: return (instr_t){.op=decoded, .inst=DADD, .name="dadd"};
      case 45: return (instr_t){.op=decoded, .inst=DADDU, .name="daddu"};
      case 46: return (instr_t){.op=decoded, .inst=DSUB, .name="dsub"};
      case 47: return (instr_t){.op=decoded, .inst=DSUBU, .name="dsubu"};
      case 48: return (instr_t){.op=decoded, .inst=TGE, .name="tge"};
      case 49: return (instr_t){.op=decoded, .inst=TGEU, .name="tgeu"};
      case 50: return (instr_t){.op=decoded, .inst=TLT, .name="tlt"};
      case 51: return (instr_t){.op=decoded, .inst=TLTU, .name="tltu"};
      case 52: return (instr_t){.op=decoded, .inst=TEQ, .name="teq"};
      case 53: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 54: return (instr_t){.op=decoded, .inst=TNE, .name="tne"};
      case 55: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 56: return (instr_t){.op=decoded, .inst=DSLL, .name="dsll"};
      case 57: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 58: return (instr_t){.op=decoded, .inst=DSRL, .name="dsrl"};
      case 59: return (instr_t){.op=decoded, .inst=DSRA, .name="dsra"};
      case 60: return (instr_t){.op=decoded, .inst=DSLL32, .name="dsll32"};
      case 61: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 62: return (instr_t){.op=decoded, .inst=DSRL32, .name="dsrl32"};
      case 63: return (instr_t){.op=decoded, .inst=DSRA32, .name="dsra32"};
      default: return (instr_t){.op=decoded, .inst=INVALID, .name="bltz"};
    } break;
    case 1: switch (decoded.rt) {
      case 0:  return (instr_t){.op=decoded, .inst=BLTZ, .name="bltz"};
      case 1:  return (instr_t){.op=decoded, .inst=BGEZ, .name="bgez"};
      case 2:  return (instr_t){.op=decoded, .inst=BLTZL, .name="bltzl"};
      case 3:  return (instr_t){.op=decoded, .inst=BGEZL, .name="bgezl"};
      case 4:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 5:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 6:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 7:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 8:  return (instr_t){.op=decoded, .inst=TGEI, .name="tgei"};
      case 9:  return (instr_t){.op=decoded, .inst=TGEIU, .name="tgeiu"};
      case 10: return (instr_t){.op=decoded, .inst=TLTI, .name="tlti"};
      case 11: return (instr_t){.op=decoded, .inst=TLTIU, .name="tltiu"};
      case 12: return (instr_t){.op=decoded, .inst=TEQI, .name="teqi"};
      case 13: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 14: return (instr_t){.op=decoded, .inst=TNEI, .name="tnei"};
      case 15: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 16: return (instr_t){.op=decoded, .inst=BLTZAL, .name="bltzal"};
      case 17: return (instr_t){.op=decoded, .inst=BGEZAL, .name="bgezal"};
      case 18: return (instr_t){.op=decoded, .inst=BLTZALL, .name="bltzall"};
      case 19: return (instr_t){.op=decoded, .inst=BGEZALL, .name="bgezall"};
      case 20: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 21: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 22: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 23: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 24: return (instr_t){.op=decoded, .inst=MTSAB, .name="mtsab"};
      case 25: return (instr_t){.op=decoded, .inst=MTSAH, .name="mstah"};
      case 26: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 27: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 28: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 29: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 30: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      case 31: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    } break;
    case 2:  return (instr_t){.op=decoded, .inst=J, .name="j"};
    case 3:  return (instr_t){.op=decoded, .inst=JAL, .name="jal"};
    case 4:  return (instr_t){.op=decoded, .inst=BEQ, .name="beq"};
    case 5:  return (instr_t){.op=decoded, .inst=BNE, .name="bne"};
    case 6:  return (instr_t){.op=decoded, .inst=BLEZ, .name="blez"};
    case 7:  return (instr_t){.op=decoded, .inst=BGTZ, .name="bgtz"};
    case 8:  return (instr_t){.op=decoded, .inst=ADDI, .name="addi"};
    case 9:  return (instr_t){.op=decoded, .inst=ADDIU, .name="addiu"};
    case 10: return (instr_t){.op=decoded, .inst=SLTI, .name="slti"};
    case 11: return (instr_t){.op=decoded, .inst=SLTIU, .name="sltiu"};
    case 12: return (instr_t){.op=decoded, .inst=ANDI, .name="andi"};
    case 13: return (instr_t){.op=decoded, .inst=ORI, .name="ori"};
    case 14: return (instr_t){.op=decoded, .inst=XORI, .name="xori"};
    case 15: return (instr_t){.op=decoded, .inst=LUI, .name="lui"};
    case 16: switch (decoded.rs) {
      //case 0: switch () TODO: breakpoint stuff
      //case 4: switch () TODO: breakpoint stuff
      case 8: switch (decoded.rt) {
        case 0: return (instr_t){.op=decoded, .inst=BC0F, .name="bc0f"};
        case 1: return (instr_t){.op=decoded, .inst=BC0T, .name="bc0t"};
        case 2: return (instr_t){.op=decoded, .inst=BC0FL, .name="bc0fl"};
        case 3: return (instr_t){.op=decoded, .inst=BC0TL, .name="bc0tl"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      } break;
      case 16: switch (decoded.func) {
        case 1: return (instr_t){.op=decoded, .inst=TLBR, .name="tlbr"};
        case 2: return (instr_t){.op=decoded, .inst=TLBWI, .name="tlbwi"};
        case 6: return (instr_t){.op=decoded, .inst=TLBWR, .name="tlbwr"};
        case 8: return (instr_t){.op=decoded, .inst=TLBP, .name="tlbp"};
        case 16: return (instr_t){.op=decoded, .inst=ERET, .name="eret"};
        case 56: return (instr_t){.op=decoded, .inst=EI, .name="ei"};
        case 57: return (instr_t){.op=decoded, .inst=DI, .name="di"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      }
      default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    }
    case 17: switch (decoded.rs) {
      case 0: return (instr_t){.op=decoded, .inst=MFC1, .name="mfc1"};
      case 2: return (instr_t){.op=decoded, .inst=CFC1, .name="cfc1"};
      case 4: return (instr_t){.op=decoded, .inst=MTC1, .name="mtc1"};
      case 6: return (instr_t){.op=decoded, .inst=CTC1, .name="ctc1"};
      case 8: switch (decoded.rt) {
        case 0: return (instr_t){.op=decoded, .inst=BC1F, .name="bc1f"};
        case 1: return (instr_t){.op=decoded, .inst=BC1T, .name="bc1t"};
        case 2: return (instr_t){.op=decoded, .inst=BC1FL, .name="bc1fl"};
        case 3: return (instr_t){.op=decoded, .inst=BC1TL, .name="bc1tl"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      } break;
      case 16: switch (decoded.func) {

      }

    }
    case 18: // COP2
    case 19: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 20: return (instr_t){.op=decoded, .inst=BEQL, .name="beql"};
    case 21: return (instr_t){.op=decoded, .inst=BNEL, .name="bnel"};
    case 22: return (instr_t){.op=decoded, .inst=BLEZL, .name="blezl"};
    case 23: return (instr_t){.op=decoded, .inst=BGTZL, .name="bgtzl"};
    case 24: return (instr_t){.op=decoded, .inst=DADDI, .name="daddi"};
    case 25: return (instr_t){.op=decoded, .inst=DADDIU, .name="daddiu"};
    case 26: return (instr_t){.op=decoded, .inst=LDL, .name="ldl"};
    case 27: return (instr_t){.op=decoded, .inst=LDR, .name="LDR"};
    case 28: switch (decoded.func) {
      case 0:  return (instr_t){.op=decoded, .inst=MADD, .name="madds"};
      case 1:  return (instr_t){.op=decoded, .inst=MADDU, .name="maddu"};
      case 4:  return (instr_t){.op=decoded, .inst=PLZCW, .name="plzcw"};
      case 8:  switch (decoded.sa) {
        case 0:  return (instr_t){.op=decoded, .inst=PADDW, .name="paddw"};
        case 1:  return (instr_t){.op=decoded, .inst=PSUBW, .name="psubw"};
        case 2:  return (instr_t){.op=decoded, .inst=PCGTW, .name="pcgtw"};
        case 3:  return (instr_t){.op=decoded, .inst=PMAXW, .name="pmaxw"};
        case 4:  return (instr_t){.op=decoded, .inst=PADDH, .name="paddh"};
        case 5:  return (instr_t){.op=decoded, .inst=PSUBH, .name="psubh"};
        case 6:  return (instr_t){.op=decoded, .inst=PCGTH, .name="pcgth"};
        case 7:  return (instr_t){.op=decoded, .inst=PMAXH, .name="pmaxh"};
        case 8:  return (instr_t){.op=decoded, .inst=PADDB, .name="paddb"};
        case 9:  return (instr_t){.op=decoded, .inst=PSUBB, .name="psubb"};
        case 10: return (instr_t){.op=decoded, .inst=PCGTB, .name="pcgtb"};
        case 11: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 12: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 13: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 14: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 15: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 16: return (instr_t){.op=decoded, .inst=PADDSW, .name="paddsw"};
        case 17: return (instr_t){.op=decoded, .inst=PSUBSW, .name="psubsw"};
        case 18: return (instr_t){.op=decoded, .inst=PEXTLW, .name="pextlw"};
        case 19: return (instr_t){.op=decoded, .inst=PPACW, .name="ppacw"};
        case 20: return (instr_t){.op=decoded, .inst=PADDSH, .name="paddsh"};
        case 21: return (instr_t){.op=decoded, .inst=PSUBSH, .name="psubsh"};
        case 22: return (instr_t){.op=decoded, .inst=PEXTLH, .name="pextlh"};
        case 23: return (instr_t){.op=decoded, .inst=PPACH, .name="ppach"};
        case 24: return (instr_t){.op=decoded, .inst=PADDSB, .name="paddsb"};
        case 25: return (instr_t){.op=decoded, .inst=PSUBSB, .name="psubsb"};
        case 26: return (instr_t){.op=decoded, .inst=PEXTLB, .name="pextlb"};
        case 27: return (instr_t){.op=decoded, .inst=PPACB, .name="ppacb"};
        case 28: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 29: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 30: return (instr_t){.op=decoded, .inst=PEXT5, .name="pext5"};
        case 31: return (instr_t){.op=decoded, .inst=PPAC5, .name="ppac5"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      } break;
      case 9:  switch (decoded.sa) {
        case 0:  return (instr_t){.op=decoded, .inst=PMADDW, .name="pmaddw"};
        case 1:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 2:  return (instr_t){.op=decoded, .inst=PSLLVW, .name="psllvw"};
        case 3:  return (instr_t){.op=decoded, .inst=PSRLVW, .name="psrlvw"};
        case 4:  return (instr_t){.op=decoded, .inst=PMSUBW, .name="pmsubw"};
        case 5:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 6:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 7:  return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 8:  return (instr_t){.op=decoded, .inst=PMFHI, .name="pmfhi"};
        case 9:  return (instr_t){.op=decoded, .inst=PMFLO, .name="pmflo"};
        case 10: return (instr_t){.op=decoded, .inst=PINTH, .name="pinth"};
        case 11: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 12: return (instr_t){.op=decoded, .inst=PMULTW, .name="pmultw"};
        case 13: return (instr_t){.op=decoded, .inst=PDIVW, .name="pdivw"};
        case 14: return (instr_t){.op=decoded, .inst=PCPYLD, .name="pcpyld"};
        case 15: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 16: return (instr_t){.op=decoded, .inst=PMADDH, .name="pmaddh"};
        case 17: return (instr_t){.op=decoded, .inst=PHMADH, .name="phmadh"};
        case 18: return (instr_t){.op=decoded, .inst=PAND, .name="pand"};
        case 19: return (instr_t){.op=decoded, .inst=PXOR, .name="pxor"};
        case 20: return (instr_t){.op=decoded, .inst=PMSUBH, .name="pmsubh"};
        case 21: return (instr_t){.op=decoded, .inst=PHMSBH, .name="phmsbh"};
        case 22: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 23: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 24: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 25: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 26: return (instr_t){.op=decoded, .inst=PEXEH, .name="pexeh"};
        case 27: return (instr_t){.op=decoded, .inst=PREVH, .name="prevh"};
        case 28: return (instr_t){.op=decoded, .inst=PMULTH, .name="pmulth"};
        case 29: return (instr_t){.op=decoded, .inst=PDIVBW, .name="pdivbw"};
        case 30: return (instr_t){.op=decoded, .inst=PEXEH, .name="pexeh"};
        case 31: return (instr_t){.op=decoded, .inst=PROT3W, .name="prot3w"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
      } break;
      case 16: return (instr_t){.op=decoded, .inst=MFHI1, .name="sll %s %s %s"};
      case 17: return (instr_t){.op=decoded, .inst=MTHI1, .name="sll %s %s %s"};
      case 18: return (instr_t){.op=decoded, .inst=MFLO1, .name="sll %s %s %s"};
      case 19: return (instr_t){.op=decoded, .inst=MTLO1, .name="sll %s %s %s"};
      case 24: return (instr_t){.op=decoded, .inst=MULT1, .name="sll %s %s %s"};
      case 25: return (instr_t){.op=decoded, .inst=MULTU1, .name="sll %s %s %s"};
      case 26: return (instr_t){.op=decoded, .inst=DIV1, .name="sll %s %s %s"};
      case 27: return (instr_t){.op=decoded, .inst=DIVU1, .name="sll %s %s %s"};
      case 32: return (instr_t){.op=decoded, .inst=MADD1, .name="sll %s %s %s"};
      case 33: return (instr_t){.op=decoded, .inst=MADDU1, .name="sll %s %s %s"};
      case 40:  switch (decoded.sa) {
        case 0:  return (instr_t){.op=decoded, .inst=INVALID, .name="bltz"};
        case 1:  return (instr_t){.op=decoded, .inst=PABSW, .name="bgez"};
        case 2:  return (instr_t){.op=decoded, .inst=PCEQW, .name="bltzl"};
        case 3:  return (instr_t){.op=decoded, .inst=PMINW, .name="bgezl"};
        case 4:  return (instr_t){.op=decoded, .inst=PADSBH, .name="invalid"};
        case 5:  return (instr_t){.op=decoded, .inst=PABSH, .name="invalid"};
        case 6:  return (instr_t){.op=decoded, .inst=PCEQH, .name="invalid"};
        case 7:  return (instr_t){.op=decoded, .inst=PMINH, .name="invalid"};
        case 8:  return (instr_t){.op=decoded, .inst=INVALID, .name="tgei"};
        case 9:  return (instr_t){.op=decoded, .inst=INVALID, .name="tgeiu"};
        case 10: return (instr_t){.op=decoded, .inst=PCEQB, .name="tlti"};
        case 11: return (instr_t){.op=decoded, .inst=INVALID, .name="tltiu"};
        case 12: return (instr_t){.op=decoded, .inst=INVALID, .name="teqi"};
        case 13: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 14: return (instr_t){.op=decoded, .inst=INVALID, .name="tnei"};
        case 15: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 16: return (instr_t){.op=decoded, .inst=PADDUW, .name="bltzal"};
        case 17: return (instr_t){.op=decoded, .inst=PSUBUW, .name="bgezal"};
        case 18: return (instr_t){.op=decoded, .inst=PEXTUW, .name="bltzall"};
        case 19: return (instr_t){.op=decoded, .inst=INVALID, .name="bgezall"};
        case 20: return (instr_t){.op=decoded, .inst=PADDUH, .name="invalid"};
        case 21: return (instr_t){.op=decoded, .inst=PSUBUH, .name="invalid"};
        case 22: return (instr_t){.op=decoded, .inst=PEXTUH, .name="invalid"};
        case 23: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 24: return (instr_t){.op=decoded, .inst=PADDUB, .name="mtsab"};
        case 25: return (instr_t){.op=decoded, .inst=PSUBUB, .name="mstah"};
        case 26: return (instr_t){.op=decoded, .inst=PEXTUB, .name="invalid"};
        case 27: return (instr_t){.op=decoded, .inst=QFSRV, .name="invalid"};
        case 28: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 29: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 30: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        case 31: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="bltz"};
      } break;
      case 41:  switch (decoded.sa) {
        case 0:  return (instr_t){.op=decoded, .inst=PMADDUW, .name="bltz"};
        case 3:  return (instr_t){.op=decoded, .inst=PSRAVW, .name="bltz"};
        case 8:  return (instr_t){.op=decoded, .inst=PMTHI, .name="bltz"};
        case 9:  return (instr_t){.op=decoded, .inst=PMTLO, .name="bltz"};
        case 10: return (instr_t){.op=decoded, .inst=PINTEH, .name="bltz"};
        case 12: return (instr_t){.op=decoded, .inst=PMULTUW, .name="bltz"};
        case 13: return (instr_t){.op=decoded, .inst=PDIVUW, .name="bltz"};
        case 14: return (instr_t){.op=decoded, .inst=PCPYUD, .name="bltz"};
        case 18: return (instr_t){.op=decoded, .inst=POR, .name="bltz"};
        case 19: return (instr_t){.op=decoded, .inst=PNOR, .name="bltz"};
        case 26: return (instr_t){.op=decoded, .inst=PEXCH, .name="bltz"};
        case 27: return (instr_t){.op=decoded, .inst=PCPYH, .name="bltz"};
        case 30: return (instr_t){.op=decoded, .inst=PEXCW, .name="bltz"};
        default: return (instr_t){.op=decoded, .inst=INVALID, .name="bltz"};
      } break;
      case 48: return (instr_t){.op=decoded, .inst=PMFHL, .name="sll %s %s %s"};
      case 49: return (instr_t){.op=decoded, .inst=PMTHL, .name="sll %s %s %s"};
      case 52: return (instr_t){.op=decoded, .inst=PSLLH, .name="sll %s %s %s"};
      case 54: return (instr_t){.op=decoded, .inst=PSRLH, .name="sll %s %s %s"};
      case 55: return (instr_t){.op=decoded, .inst=PSRAH, .name="sll %s %s %s"};
      case 60: return (instr_t){.op=decoded, .inst=PSLLW, .name="sll %s %s %s"};
      case 62: return (instr_t){.op=decoded, .inst=PSRLW, .name="sll %s %s %s"};
      case 63: return (instr_t){.op=decoded, .inst=PSRAW, .name="sll %s %s %s"};
      default: return (instr_t){.op=decoded, .inst=INVALID, .name="sll %s %s %s"};
    }
    case 29: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 30: return (instr_t){.op=decoded, .inst=LQ, .name="lq"};
    case 31: return (instr_t){.op=decoded, .inst=SQ, .name="sq"};
    case 32: return (instr_t){.op=decoded, .inst=LB, .name="lb"};
    case 33: return (instr_t){.op=decoded, .inst=LH, .name="lh"};
    case 34: return (instr_t){.op=decoded, .inst=LWL, .name="lwl"};
    case 35: return (instr_t){.op=decoded, .inst=LW, .name="lw"};
    case 36: return (instr_t){.op=decoded, .inst=LBU, .name="lbu"};
    case 37: return (instr_t){.op=decoded, .inst=LHU, .name="lhu"};
    case 38: return (instr_t){.op=decoded, .inst=LWR, .name="lwr"};
    case 39: return (instr_t){.op=decoded, .inst=LWU, .name="lwu"};
    case 40: return (instr_t){.op=decoded, .inst=SB, .name="sb"};
    case 41: return (instr_t){.op=decoded, .inst=SH, .name="sh"};
    case 42: return (instr_t){.op=decoded, .inst=SWL, .name="swl"};
    case 43: return (instr_t){.op=decoded, .inst=SW, .name="sw"};
    case 44: return (instr_t){.op=decoded, .inst=SDL, .name="sdl"};
    case 45: return (instr_t){.op=decoded, .inst=SDR, .name="sdr"};
    case 46: return (instr_t){.op=decoded, .inst=SWR, .name="dsub"};
    case 47: return (instr_t){.op=decoded, .inst=CACHE, .name="cache"};
    case 48: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 49: return (instr_t){.op=decoded, .inst=LWC1, .name="lwc1"};
    case 50: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 51: return (instr_t){.op=decoded, .inst=PREF, .name="pref"};
    case 52: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 53: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 54: return (instr_t){.op=decoded, .inst=LQC2, .name="lqc2"};
    case 55: return (instr_t){.op=decoded, .inst=LD, .name="ld"};
    case 56: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 57: return (instr_t){.op=decoded, .inst=SWC1, .name="swc1"};
    case 58: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 59: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 60: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 61: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
    case 62: return (instr_t){.op=decoded, .inst=SQC2, .name="sqc2"};
    case 63: return (instr_t){.op=decoded, .inst=SD, .name="SD"};
    default: return (instr_t){.op=decoded, .inst=INVALID, .name="invalid"};
  }
}

overload bool mips_read(ps2* ps2, mips* mips, u32 address, u8* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, u16* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, u32* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, u64* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, u128* dest) {}

overload bool mips_read(ps2* ps2, mips* mips, u32 address, s8* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, s16* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, s32* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, s64* dest) {}
overload bool mips_read(ps2* ps2, mips* mips, u32 address, s128* dest) {}

overload bool mips_write(ps2* ps2, mips* mips, u32 address, u8 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, u16 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, u32 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, u64 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, u128 data) {}

overload bool mips_write(ps2* ps2, mips* mips, u32 address, s8 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, s16 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, s32 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, s64 data) {}
overload bool mips_write(ps2* ps2, mips* mips, u32 address, s128 data) {}

void mips_disasm(instr_t opcode, char dest[static 1024]) {
  switch (opcode.disasm) {
    case DISASM_NONE: snprintf(dest,1024,"%s",opcode.name); break;
    case DISASM_RD_RS_RT: snprintf(dest,1024,"%s %s,%s,%s",opcode.name, mips_reg_name(opcode.op.rd),mips_reg_name(opcode.op.rs),mips_reg_name(opcode.op.rt)); break;
    case DISASM_RT_RS_IMM: break;
    case DISASM_RS_RT_OFFSET: break;
    case DISASM_RS_OFFSET: break;
    case DISASM_RS_RT: break;
    case DISASM_RD_RT_SA: break;
    case DISASM_RD_RT_RS: break;
    case DISASM_TARGET: break;
    case DISASM_RS: break;
    case DISASM_RD_RS: break;
    case DISASM_RT_OFFSET_BASE: break;
    case DISASM_RT_IMM: break;
    case DISASM_RD: break;
    case DISASM_RS_IMM: break;
    case DISASM_OFFSET: break;
    case DISASM_RT_RD: break;
    case DISASM_RT: break;
  }
}

void mips_execute(ps2* ps2, mips* mips, instr_t instr_t) {
  opcode_t decoded = instr_t.op;

#define rt(T,i)   mips->gpr[decoded.rt].T[i]
#define rd(T,i)   mips->gpr[decoded.rd].T[i]
#define rs(T,i)   mips->gpr[decoded.rs].T[i]
#define base(T,i) rs(T,i)

  auto simm = decoded.simm;
  auto uimm = decoded.uimm;
  u8 base = decoded.rs;

  u64 shift = 0, mask = 0, data = 0, merged = 0;

  u8 tempu8 = 0;
  u16 tempu16 = 0;
  u32 tempu32 = 0;
  u64 tempu64 = 0;
  u128 tempu128 = 0;

  s8 temps8 = 0;
  s16 temps16 = 0;
  s32 temps32 = 0;
  s64 temps64 = 0;
  s128 temps128 = 0;

  switch (instr_t.inst) {
  case INVALID: break;
  case LB:  if (mips_read(ps2,  mips, base(u32,0) + simm, &temps8))  rt(s64,0) = temps8;  break;
  case LBU: if (mips_read(ps2,  mips, base(u32,0) + simm, &tempu8))  rt(u64,0) = tempu8;  break;
  case LD:  if (mips_read(ps2, mips, base(u32,0) + simm, &tempu64)) rt(u64,0) = tempu64; break;
  case LDL: if (mips_read(ps2, mips, base(u32,0) + simm & ~7UL, &tempu64)) {
      rt(u64,0) = rt(u64,0) & (1ULL << (base(u32,0) + simm & 7) * 8) - 1 | tempu64 << (base(u32,0) + simm & 7) * 8;
    }
    break;
  case LDR: if (mips_read(ps2, mips, base(u32,0) + simm & ~7UL, &tempu64)) {
      rt(u64,0) = rt(u64,0) & ~((1ULL << (8 - base(u32,0) + simm & 7) * 8) - 1) | tempu64 >> (base(u32,0) + simm & 7 * 8);
    }
    break;
  case LH:  if (mips_read(ps2,  mips, base(u32,0) + simm, &temps16))  rt(s64,0) = temps16; break;
  case LHU: if (mips_read(ps2,  mips, base(u32,0) + simm, &tempu16))  rt(u64,0) = tempu16;  break;
  case LW:  if (mips_read(ps2,  mips, base(u32,0) + simm, &temps32))  rt(s64,0) = temps32;  break;
  case LWL: if (mips_read(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL, &tempu32)) {
    rt(u64,0) = (rt(u64,0) & ((1ULL << ((base(u32,0) + simm & 3) * 8)) - 1)) |
                (((u64)tempu32 << ((base(u32,0) + simm & 3) * 8)) >> 32 << 32);
  }
  break;
  case LWR: if (mips_read(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL, &tempu32)) {
    rt(u64,0) = (rt(u64,0) & ~((1ULL << ((4 - base(u32,0) + simm & 3) * 8)) - 1)) |
                (((u64)tempu32 >> ((base(u32,0) + simm & 3) * 8)) << 32 >> 32);
  }
  break;
  case LWU: if (mips_read(ps2, mips, base(u32,0) + simm, &tempu32)) rt(u64,0) = tempu32; break;
  case SB: mips_write(ps2, mips, base(u32,0) + simm, rt(u8,0));   break;
  case SD: mips_write(ps2, mips, base(u32,0) + simm, rt(u64,0)); break;
  case SDL: if (mips_read(ps2, mips, base(u32,0) + simm & ~7UL, &tempu64)) mips_write(ps2, mips, base(u32,0) + simm & ~7UL, (tempu64 & ~(~0ULL << ((base(u32,0) + simm & 7) * 8))) | (rt(u64,0) >> ((base(u32,0) + simm & 7) * 8))); break;
  case SDR: if (mips_read(ps2, mips, base(u32,0) + simm & ~7UL, &tempu64)) mips_write(ps2, mips, base(u32,0) + simm & ~7UL, (tempu64 & ~((1ULL << ((8 - (base(u32,0) + simm & 7)) * 8)) - 1)) | (rt(u64,0) << ((8 - (base(u32,0) + simm & 7)) * 8))); break;
  case SH: mips_write(ps2, mips, base(u32,0) + simm, rt(u16,0)); break;
  case SW: mips_write(ps2, mips, base(u32,0) + simm, rt(u32,0)); break;
  case SWL: if (mips_read(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL, &tempu32)) {
    mips_write(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL,
      (tempu32 & ~(~0U << ((base(u32,0) + simm & 3) * 8))) |
      (rt(u32,0) >> ((base(u32,0) + simm & 3) * 8)));
  }
  break;
  case SWR: if (mips_read(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL, &tempu32)) {
    mips_write(ps2, mips, (u32)(base(u32,0) + simm) & ~3UL,
      (tempu32 & ~((1U << ((4 - (base(u32,0) + simm & 3)) * 8)) - 1)) |
      (rt(u32,0) << ((4 - (base(u32,0) + simm & 3)) * 8)));
  }
  break;
  case ADDI: if (add_overflow(rs(s64,0), simm, &rt(s64,0))) {/* exception */} break;
  case ADDIU: rt(s64,0) = rs(s64,0) + simm; break;
  case ANDI: rt(u64,0) = rs(u64,0) & uimm;  break;
  case DADDI: if (add_overflow(rs(s64,0), simm, &rt(s64,0))) {/* exception */} break;
  case DADDIU: rt(s64,0) = rs(s64,0) + simm; break;
  case LUI: rt(u64,0) = (u64)uimm << 16; break;
  case ORI: rt(u64,0) = rs(u64,0) | uimm; break;
  case SLTI: rt(u64,0) = rs(s64,0) < simm ? 1 : 0; break;
  case SLTIU: rt(u64,0) = (u64)rs(s64,0) < (u64)simm ? 1 : 0; break;
  case XORI: rt(u64,0) = rs(u64,0) ^ uimm; break;
  case ADD: if (add_overflow(rs(s64,0), rd(s64,0), &rt(s64,0))) {} break;
  case ADDU: rt(s64,0) = rs(s64,0) + rd(s64,0); break;
  case AND: rt(u64,0) = rs(u64,0) & rd(u64,0); break;
  case DADD: if (add_overflow(rs(s64,0), rd(s64,0), &rt(s64,0))) {} break;
  case DADDU: rt(s64,0) = rs(s64,0) + rd(s64,0); break;
  case DSUB: if (sub_overflow(rs(s64,0), rd(s64,0), &rt(s64,0))) {} break;
  case DSUBU: rt(s64,0) = rs(s64,0) - rd(s64,0); break;
  case NOR: rt(u64,0) = ~(rs(u64,0) | rd(u64,0)); break;
  case OR: rt(u64,0) = rs(u64,0) | rd(u64,0); break;
  case SLT: rt(u64,0) = rs(s64,0) < rd(s64,0) ? 1 : 0; break;
  case SLTU: rt(u64,0) = (u64)rs(s64,0) < (u64)rd(s64,0) ? 1 : 0; break;
  case SUB: if (sub_overflow(rs(s64,0), rd(s64,0), &rt(s64,0))) {} break;
  case SUBU: rt(s64,0) = rs(s64,0) - rd(s64,0); break;
  case XOR: rt(u64,0) = rs(u64,0) ^ rd(u64,0); break;
  case DSLL: rt(u64,0) = rd(u64,0) << decoded.sa; break;
  case DSLL32: rt(u64,0) = rd(u64,0) << (decoded.sa + 32); break;
  case DSLLV: rt(u64,0) = rd(u64,0) << (rs(u64,0) & 63); break;
  case DSRA: rt(s64,0) = rd(s64,0) >> decoded.sa; break;
  case DSRA32: rt(s64,0) = rd(s64,0) >> (decoded.sa + 32); break;
  case DSRAV: rt(s64,0) = rd(s64,0) >> (rs(u64,0) & 63); break;
  case DSRL: rt(u64,0) = rd(u64,0) >> decoded.sa; break;
  case DSRL32: rt(u64,0) = rd(u64,0) >> (decoded.sa + 32); break;
  case DSRLV: rt(u64,0) = rd(u64,0) >> (rs(u64,0) & 63); break;
  case SLL: rt(u64,0) = (u64)((u32)rd(u64,0) << decoded.sa); break;
  case SLLV: rt(u64,0) = (u64)((u32)rd(u64,0) << (rs(u64,0) & 31)); break;
  case SRA: rt(s64,0) = (s64)((s32)rd(s64,0) >> decoded.sa); break;
  case SRAV: rt(s64,0) = (s64)((s32)rd(s64,0) >> (rs(u64,0) & 31)); break;
  case SRL: rt(u64,0) = (u64)((u32)rd(u64,0) >> decoded.sa); break;
  case SRLV: rt(u64,0) = (u64)((u32)rd(u64,0) >> (rs(u64,0) & 31)); break;
  case DIV: mips->hi.s64[0] = rs(s64,0) % rd(s64,0); mips->lo.s64[0] = rs(s64,0) / rd(s64,0); break;
  case DIVU: mips->hi.u64[0] = (u64)rs(s64,0) % (u64)rd(s64,0); mips->lo.u64[0] = (u64)rs(s64,0) / (u64)rd(s64,0); break;
  case MFHI: rt(u64,0) = mips->hi.u64[0]; break;
  case MFLO: rt(u64,0) = mips->lo.u64[0]; break;
  case MTHI: mips->hi.u64[0] = rs(u64,0); break;
  case MTLO: mips->lo.u64[0] = rs(u64,0); break;
  case MULT: mips->lo.s64[0] = (s64)((s32)rs(s64,0)) * (s64)((s32)rd(s64,0)); mips->hi.s64[0] = (mips->lo.s64[0] >> 63); break;
  case MULTU: mips->lo.u64[0] = (u64)((u32)rs(u64,0)) * (u64)((u32)rd(u64,0)); mips->hi.u64[0] = (mips->lo.u64[0] >> 32); break;
  case J: mips->nextpc = (mips->pc & 0xF0000000) | (decoded.target << 2); break;
  case JAL: mips->ra.u64[0] = mips->pc + 8; mips->nextpc = (mips->pc & 0xF0000000) | (decoded.target << 2); break;
  case JALR: tempu64 = rs(u64,0); mips->ra.u64[0] = mips->pc + 8; mips->nextpc = tempu64; break;
  case JR: mips->nextpc = rs(u64,0); break;
  case BEQ: if (rs(u64,0) == rt(u64,0)) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BEQL: if (rs(u64,0) == rt(u64,0)) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case BGEZ: if (rs(s64,0) >= 0) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BGEZAL: if (rs(s64,0) >= 0) { mips->ra.u64[0] = mips->pc + 8; mips->nextpc = mips->pc + (simm << 2) + 4; } break;
  case BGEZALL: if (rs(s64,0) >= 0) { mips->ra.u64[0] = mips->pc + 8; mips->nextpc = mips->pc + (simm << 2) + 4; } else mips->nextpc = mips->pc + 8; break;
  case BGEZL: if (rs(s64,0) >= 0) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case BGTZ: if (rs(s64,0) > 0) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BGTZL: if (rs(s64,0) > 0) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case BLEZ: if (rs(s64,0) <= 0) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BLEZL: if (rs(s64,0) <= 0) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case BLTZ: if (rs(s64,0) < 0) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BLTZAL: if (rs(s64,0) < 0) { mips->ra.u64[0] = mips->pc + 8; mips->nextpc = mips->pc + (simm << 2) + 4; } break;
  case BLTZALL: if (rs(s64,0) < 0) { mips->ra.u64[0] = mips->pc + 8; mips->nextpc = mips->pc + (simm << 2) + 4; } else mips->nextpc = mips->pc + 8; break;
  case BLTZL: if (rs(s64,0) < 0) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case BNE: if (rs(u64,0) != rt(u64,0)) mips->nextpc = mips->pc + (simm << 2) + 4; break;
  case BNEL: if (rs(u64,0) != rt(u64,0)) mips->nextpc = mips->pc + (simm << 2) + 4; else mips->nextpc = mips->pc + 8; break;
  case SYSCALL: break;
  case BREAK: break;
  case TGE: break;
  case TGEU: break;
  case TLT: break;
  case TLTU: break;
  case TEQ: break;
  case TNE: break;
  case TGEI: break;
  case TGEIU: break;
  case TLTI: break;
  case TLTIU: break;
  case TEQI: break;
  case TNEI: break;
  case SYNC: break;
  case MOVN: if (rt(u64,0) != 0) rd(u64,0) = rs(u64,0); break;
  case MOVZ: if (rt(u64,0) == 0) rd(u64,0) = rs(u64,0); break;
  case PREF: break;
  case MADD: { s128 result = (s128)mips->lo.s64[0] + ((s128)rs(s64,0) * rd(s64,0)); mips->lo.s64[0] = (s64)result; mips->hi.s64[0] = (s64)(result >> 64); } break;
  case MADDU: { u128 result = (u128)mips->lo.u64[0] + ((u128)rs(u64,0) * rd(u64,0)); mips->lo.u64[0] = (u64)result; mips->hi.u64[0] = (u64)(result >> 64); } break;
  case MULT1: mips->lo.s64[0] = (s64)((s32)rs(s64,0)) * (s64)((s32)rd(s64,0)); mips->hi.s64[0] = (mips->lo.s64[0] >> 63); break;
  case MULTU1: mips->lo.u64[0] = (u64)((u32)rs(u64,0)) * (u64)((u32)rd(u64,0)); mips->hi.u64[0] = (mips->lo.u64[0] >> 32); break;
  case DIV1:  mips->hi.s64[0] = rs(s64,0) % rd(s64,0); mips->lo.s64[0] = rs(s64,0) / rd(s64,0); break;
  case DIVU1:  mips->hi.u64[0] = (u64)rs(s64,0) % (u64)rd(s64,0); mips->lo.u64[0] = (u64)rs(s64,0) / (u64)rd(s64,0); break;
  case MADD1:  { s128 result = (s128)mips->lo.s64[0] + ((s128)rs(s64,0) * rd(s64,0)); mips->lo.s64[0] = (s64)result; mips->hi.s64[0] = (s64)(result >> 64); } break;
  case MADDU1: { u128 result = (u128)mips->lo.u64[0] + ((u128)rs(u64,0) * rd(u64,0)); mips->lo.u64[0] = (u64)result; mips->hi.u64[0] = (u64)(result >> 64); } break;
  case MFHI1:  rt(u64,0) = mips->hi.u64[0]; break;
  case MTHI1: mips->hi.u64[0] = rs(u64,0); break;
  case MFLO1:  rt(u64,0) = mips->lo.u64[0]; break;
  case MTLO1:  mips->lo.u64[0] = rs(u64,0); break;
  case PADDB: for (int i = 0; i < 16; i++) rd(u8,i) = rs(u8,i) + rt(u8,i); break;
  case PSUBB: for (int i = 0; i < 16; i++) rd(u8,i) = rs(u8,i) - rt(u8,i); break;
  case PADDH: for (int i = 0; i < 8; i++) rd(u16,i) = rs(u16,i) + rt(u16,i); break;
  case PSUBH: for (int i = 0; i < 8; i++) rd(u16,i) = rs(u16,i) - rt(u16,i); break;
  case PADDW: for (int i = 0; i < 4; i++) rd(u32,i) = rs(u32,i) + rt(u32,i); break;
  case PSUBW: for (int i = 0; i < 4; i++) rd(u32,i) = rs(u32,i) - rt(u32,i); break;
  case PADSBH: for (int i = 0; i < 8; i++) rd(s16,i) = rs(s16,i) + ((s16)rt(s8,i*2) - (s16)rt(s8,i*2+1)); break;
  case PADDSB: for (int i = 0; i < 16; i++) { s16 temp = (s16)rs(s8,i) + (s16)rt(s8,i); rd(s8,i) = temp > 127 ? 127 : temp < -128 ? -128 : temp; } break;
  case PSUBSB: for (int i = 0; i < 16; i++) { s16 temp = (s16)rs(s8,i) - (s16)rt(s8,i); rd(s8,i) = temp > 127 ? 127 : temp < -128 ? -128 : temp; } break;
  case PADDSH: for (int i = 0; i < 8; i++) { s32 temp = (s32)rs(s16,i) + (s32)rt(s16,i); rd(s16,i) = temp > 32767 ? 32767 : temp < -32768 ? -32768 : temp; } break;
  case PSUBSH: for (int i = 0; i < 8; i++) { s32 temp = (s32)rs(s16,i) - (s32)rt(s16,i); rd(s16,i) = temp > 32767 ? 32767 : temp < -32768 ? -32768 : temp; } break;
  case PADDSW: for (int i = 0; i < 4; i++) { s64 temp = (s64)rs(s32,i) + (s64)rt(s32,i); rd(s32,i) = temp > 0x7FFFFFFF ? 0x7FFFFFFF : temp < (s64)0x80000000 ? (s64)0x80000000 : temp; } break;
  case PSUBSW: for (int i = 0; i < 4; i++) { s64 temp = (s64)rs(s32,i) - (s64)rt(s32,i); rd(s32,i) = temp > 0x7FFFFFFF ? 0x7FFFFFFF : temp < (s64)0x80000000 ? (s64)0x80000000 : temp; } break;
  case PADDUB: for (int i = 0; i < 16; i++) { u16 temp = (u16)rs(u8,i) + (u16)rt(u8,i); rd(u8,i) = temp > 255 ? 255 : temp; } break;
  case PSUBUB: for (int i = 0; i < 16; i++) { s16 temp = (s16)rs(u8,i) - (s16)rt(u8,i); rd(u8,i) = temp < 0 ? 0 : temp > 255 ? 255 : temp; } break;
  case PADDUH: for (int i = 0; i < 8; i++) { u32 temp = (u32)rs(u16,i) + (u32)rt(u16,i); rd(u16,i) = temp > 65535 ? 65535 : temp; } break;
  case PSUBUH: for (int i = 0; i < 8; i++) { s32 temp = (s32)rs(u16,i) - (s32)rt(u16,i); rd(u16,i) = temp < 0 ? 0 : temp > 65535 ? 65535 : temp; } break;
  case PADDUW: for (int i = 0; i < 4; i++) { u64 temp = (u64)rs(u32,i) + (u64)rt(u32,i); rd(u32,i) = temp > 0xFFFFFFFF ? 0xFFFFFFFF : temp; } break;
  case PSUBUW: for (int i = 0; i < 4; i++) { s64 temp = (s64)rs(u32,i) - (s64)rt(u32,i); rd(u32,i) = temp < 0 ? 0 : temp > 0xFFFFFFFF ? 0xFFFFFFFF : temp; } break;
  case PMULTW: for (int i = 0; i < 4; i++) { s64 temp = (s64)rs(s32,i) * (s64)rt(s32,i); rd(s32,i) = temp; } break;
  case PMULTUW: for (int i = 0; i < 4; i++) { u64 temp = (u64)rs(u32,i) * (u64)rt(u32,i); rd(u32,i) = temp; } break;
  case PDIVW: for (int i = 0; i < 4; i++) { rd(s32,i) = rt(s32,i) != 0 ? rs(s32,i) / rt(s32,i) : 0; } break;
  case PDIVUW: for (int i = 0; i < 4; i++) { rd(u32,i) = rt(u32,i) != 0 ? rs(u32,i) / rt(u32,i) : 0; } break;
  case PMADDW: for (int i = 0; i < 4; i++) { s64 temp = (s64)rs(s32,i) * (s64)rt(s32,i) + rd(s32,i); rd(s32,i) = temp; } break;
  case PMADDUW: for (int i = 0; i < 4; i++) { u64 temp = (u64)rs(u32,i) * (u64)rt(u32,i) + rd(u32,i); rd(u32,i) = temp; } break;
  case PMSUBW: for (int i = 0; i < 4; i++) { s64 temp = rd(s32,i) - ((s64)rs(s32,i) * (s64)rt(s32,i)); rd(s32,i) = temp; } break;
  case PMFHI: rt(u64,0) = mips->hi.u64[0]; break;
  case PMTHI: mips->hi.u64[0] = rs(u64,0); break;
  case PMFLO: rt(u64,0) = mips->lo.u64[0]; break;
  case PMTLO: mips->lo.u64[0] = rs(u64,0); break;
  case PMULTH: for (int i = 0; i < 8; i++) { s32 temp = (s32)rs(s16,i) * (s32)rt(s16,i); rd(s16,i) = temp; } break;
  case PMADDH: for (int i = 0; i < 8; i++) { s32 temp = (s32)rs(s16,i) * (s32)rt(s16,i) + rd(s16,i); rd(s16,i) = temp; } break;
  case PMSUBH: for (int i = 0; i < 8; i++) { s32 temp = rd(s16,i) - ((s32)rs(s16,i) * (s32)rt(s16,i)); rd(s16,i) = temp; } break;
  case PMFHL: break;
  case PMTHL: break;
  case PHMADH: for (int i = 0; i < 4; i++) { s32 temp = ((s32)rs(s16,i*2) * (s32)rt(s16,i*2)) + ((s32)rs(s16,i*2+1) * (s32)rt(s16,i*2+1)); rd(s32,i) = temp; } break;
  case PHMSBH: for (int i = 0; i < 4; i++) { s32 temp = ((s32)rs(s16,i*2) * (s32)rt(s16,i*2)) - ((s32)rs(s16,i*2+1) * (s32)rt(s16,i*2+1)); rd(s32,i) = temp; } break;
  case PDIVBW: for (int i = 0; i < 4; i++) { rd(s32,i) = (rs(s16,i*2) << 16) | ((rt(s16,i) != 0) ? ((s32)rs(s16,i*2+1) / rt(s16,i)) : 0); } break;
  case MFSA: break;
  case MTSA: break;
  case MTSAB: break;
  case MTSAH: break;
  case PSLLH: for (int i = 0; i < 8; i++) rd(u16,i) = rt(u16,i) << decoded.sa; break;
  case PSRLH: for (int i = 0; i < 8; i++) rd(u16,i) = rt(u16,i) >> decoded.sa; break;
  case PSRAH: for (int i = 0; i < 8; i++) rd(s16,i) = rt(s16,i) >> decoded.sa; break;
  case PSLLW: for (int i = 0; i < 4; i++) rd(u32,i) = rt(u32,i) << decoded.sa; break;
  case PSLLVW: for (int i = 0; i < 4; i++) rd(u32,i) = rt(u32,i) << (rs(u32,i) & 31); break;
  case PSRLW: for (int i = 0; i < 4; i++) rd(u32,i) = rt(u32,i) >> decoded.sa; break;
  case PSRLVW: for (int i = 0; i < 4; i++) rd(u32,i) = rt(u32,i) >> (rs(u32,i) & 31); break;
  case PSRAW: for (int i = 0; i < 4; i++) rd(s32,i) = rt(s32,i) >> decoded.sa; break;
  case PSRAVW: for (int i = 0; i < 4; i++) rd(s32,i) = rt(s32,i) >> (rs(u32,i) & 31); break;
  case QFSRV: break;
  case PABSH: for (int i = 0; i < 8; i++) rd(s16,i) = rt(s16,i) < 0 ? -rt(s16,i) : rt(s16,i); break;
  case PABSW: for (int i = 0; i < 4; i++) rd(s32,i) = rt(s32,i) < 0 ? -rt(s32,i) : rt(s32,i); break;
  case PMAXH: for (int i = 0; i < 8; i++) rd(s16,i) = rs(s16,i) > rt(s16,i) ? rs(s16,i) : rt(s16,i); break;
  case PMINH: for (int i = 0; i < 8; i++) rd(s16,i) = rs(s16,i) < rt(s16,i) ? rs(s16,i) : rt(s16,i); break;
  case PMAXW: for (int i = 0; i < 4; i++) rd(s32,i) = rs(s32,i) > rt(s32,i) ? rs(s32,i) : rt(s32,i); break;
  case PMINW: for (int i = 0; i < 4; i++) rd(s32,i) = rs(s32,i) < rt(s32,i) ? rs(s32,i) : rt(s32,i); break;
  case PAND: rd(u128,0) = rs(u128,0) & rt(u128,0); break;
  case POR: rd(u128,0) = rs(u128,0) | rt(u128,0); break;
  case PNOR: rd(u128,0) = ~(rs(u128,0) | rt(u128,0)); break;
  case PXOR: rd(u128,0) = rs(u128,0) ^ rt(u128,0); break;
  case PCGTB: for (int i = 0; i < 16; i++) rd(u8,i) = (rs(s8,i) > rt(s8,i)) ? 0xFF : 0; break;
  case PCEQB: for (int i = 0; i < 16; i++) rd(u8,i) = (rs(u8,i) == rt(u8,i)) ? 0xFF : 0; break;
  case PCGTH: for (int i = 0; i < 8; i++) rd(u16,i) = (rs(s16,i) > rt(s16,i)) ? 0xFFFF : 0; break;
  case PCEQH: for (int i = 0; i < 8; i++) rd(u16,i) = (rs(u16,i) == rt(u16,i)) ? 0xFFFF : 0; break;
  case PCGTW: for (int i = 0; i < 4; i++) rd(u32,i) = (rs(s32,i) > rt(s32,i)) ? 0xFFFFFFFF : 0; break;
  case PCEQW: for (int i = 0; i < 4; i++) rd(u32,i) = (rs(u32,i) == rt(u32,i)) ? 0xFFFFFFFF : 0; break;
  case PLZCW: for (int i = 0; i < 4; i++) rd(u32,i) = __builtin_clz(rt(u32,i)); break;
  case LQ: if (mips_read(ps2, mips, base(u32,0) + simm & ~0xf, &tempu128)) rt(u128,0) = tempu128; break;
  case SQ: mips_write(ps2, mips, base(u32,0) + simm & ~0xf, rt(u128,0)); break;
  case PPACB: for (int i = 0; i < 16; i++) rd(u8,i) = i & 1 ? rt(u8,i/2) : rs(u8,i/2); break;
  case PPACH: for (int i = 0; i < 8; i++) rd(u16,i) = i & 1 ? rt(u16,i/2) : rs(u16,i/2); break;
  case PINTEH: for (int i = 0; i < 8; i++) rd(u16,i) = i & 1 ? rt(u16,i/2) : rs(u16,i/2); break;
  case PPACW: for (int i = 0; i < 4; i++) rd(u32,i) = i & 1 ? rt(u32,i/2) : rs(u32,i/2); break;
  case PEXTUB: for (int i = 0; i < 8; i++) rd(u16,i) = (i & 1) ? rt(u8,i+8) : rs(u8,i+8); break;
  case PEXTLB: for (int i = 0; i < 8; i++) rd(u16,i) = (i & 1) ? rt(u8,i) : rs(u8,i); break;
  case PEXTUH: for (int i = 0; i < 4; i++) rd(u32,i) = (i & 1) ? rt(u16,i+4) : rs(u16,i+4); break;
  case PEXTLH: for (int i = 0; i < 4; i++) rd(u32,i) = (i & 1) ? rt(u16,i) : rs(u16,i); break;
  case PEXTUW: for (int i = 0; i < 2; i++) rd(u64,i) = (i & 1) ? rt(u32,i+2) : rs(u32,i+2); break;
  case PEXTLW: for (int i = 0; i < 2; i++) rd(u64,i) = (i & 1) ? rt(u32,i) : rs(u32,i); break;
  case PEXT5: for (int i = 0; i < 4; i++) rd(u32,i) = ((rs(u32,i) >> 3) & 0x1F) | ((rt(u32,i) << 5) & 0x3E0); break;
  case PPAC5: for (int i = 0; i < 4; i++) rd(u32,i) = ((rs(u32,i) & 0x1F) << 3) | ((rt(u32,i) & 0x3E0) >> 5); break;
  case PCPYH: for (int i = 0; i < 8; i++) rd(u16,i) = rt(u16,(i & 3) * 2); break;
  case PCPYLD: rd(u64,0) = rs(u64,0); rd(u64,1) = rt(u64,0); break;
  case PCPYUD: rd(u64,0) = rs(u64,1); rd(u64,1) = rt(u64,1); break;
  case PREVH: for (int i = 0; i < 8; i++) rd(u16,i) = rt(u16,7-i); break;
  case PINTH: for (int i = 0; i < 4; i++) { rd(u16,i*2) = rs(u16,i); rd(u16,i*2+1) = rt(u16,i); } break;
  case PEXEH: for (int i = 0; i < 4; i++) { rd(u16,i*2) = rt(u16,i*2+1); rd(u16,i*2+1) = rt(u16,i*2); } break;
  case PEXCH: for (int i = 0; i < 4; i++) { rd(u16,i*2) = rt(u16,(i^2)*2); rd(u16,i*2+1) = rt(u16,(i^2)*2+1); } break;
  case PEXEW: for (int i = 0; i < 2; i++) { rd(u32,i*2) = rt(u32,i*2+1); rd(u32,i*2+1) = rt(u32,i*2); } break;
  case PEXCW: for (int i = 0; i < 2; i++) { rd(u32,i*2) = rt(u32,(i^1)*2); rd(u32,i*2+1) = rt(u32,(i^1)*2+1); } break;
  case PROT3W: rd(u32,0) = rt(u32,1); rd(u32,1) = rt(u32,2); rd(u32,2) = rt(u32,0); rd(u32,3) = rt(u32,3); break;
  case BC0F: break;
  case BC0FL: break;
  case BC0T: break;
  case BC0TL: break;
  case CACHE: break;
  case DI: break;
  case EI: break;
  case ERET: break;
  case MFC0: break;
  case MTC0: break;
  case TLBR: break;
  case TLBWI: break;
  case TLBWR: break;
  case TLBP: break;
  case LWC1: break;
  case SWC1: break;
  case MTC1: break;
  case MFC1: break;
  case MOV_S: break;
  case CTC1: break;
  case CFC1: break;
  case CVT_S_W: break;
  case CVT_W_S: break;
  case ADD_S: break;
  case SUB_S: break;
  case MUL_S: break;
  case DIV_S: break;
  case ABS_S: break;
  case NEG_S: break;
  case SQRT_S: break;
  case ADDA_S: break;
  case SUBA_S: break;
  case MULA_S: break;
  case MADD_S: break;
  case MADDA_S: break;
  case MSUB_S: break;
  case MSUBA_S: break;
  case RSQRT_S: break;
  case MAX_S: break;
  case MIN_S: break;
  case C_LT_S: break;
  case C_EQ_S: break;
  case C_F_S: break;
  case C_LE_S: break;
  case BC1T: break;
  case BC1F: break;
  case BC1TL: break;
  case BC1FL: break;
  case BC2F: break;
  case BC2FL: break;
  case BC2T: break;
  case BC2TL: break;
  case CALLMS: break;
  case CALLMSR: break;
  case CFC2: break;
  case CTC2: break;
  case LQC2: break;
  case SQC2: break;
  case QMFC2: break;
  case QMTC2: break;
  case WAITQ: break;
  }
}
