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

typedef union inst_op {
  u32 raw;
  struct [[gnu::packed]] { u8 op: 6; u8 rs: 5; u8 rt: 5; u8 rd: 5; u8 sa: 5; u8 func: 6; };
  struct [[gnu::packed]] { u8 _op: 6; u32 target:26; };
  struct [[gnu::packed]] { u8 __op: 6; u8 __rs: 5; u8 __rt: 5; union{u16 uimm; s16 simm;};};
} inst_op [[gnu::packed]];

static_assert(sizeof(inst_op) == 4);

typedef struct instruction {
  inst_op op;
  mips_instr_e inst;
  disasm_type_e disasm;
  char* name;
} instruction;

static inline const char* mips_reg_name(int reg) {
  static const char* names[] = {
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};
  return (reg >= 0 && reg < 32) ? names[reg] : "invalid";
}