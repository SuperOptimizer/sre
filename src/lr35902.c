#include "lr35902.h"

// Function to initialize the CPU and create the coroutine
void lr35902_new(lr35902* lr35902) {
  memset(lr35902, 0, sizeof(*lr35902));

  mco_desc desc = mco_desc_init(lr35902_cpu_coro, 0);
  desc.user_data = lr35902;

  mco_result res = mco_create(&lr35902->cpu_mco_coro, &desc);
  if (res != MCO_SUCCESS) {
  }
}

void lr35902_run(lr35902* lr35902, u64 cycles) {
  lr35902->remaining_cycles = cycles;

  while (lr35902->remaining_cycles > 0 && mco_status(lr35902->cpu_mco_coro) == MCO_SUSPENDED) {
    mco_result res = mco_resume(lr35902->cpu_mco_coro);
    if (res != MCO_SUCCESS) {
      break;
    }

    lr35902->remaining_cycles--;
    lr35902->cycles++;

    // Synchronize with other components (PPU, timer, audio, etc.)
    // This is where you'd call functions to update other system components
  }
}

void lr35902_destroy(lr35902* cpu) {
  if (cpu->cpu_mco_coro) {
    mco_destroy(cpu->cpu_mco_coro);
    cpu->cpu_mco_coro = NULL;
  }
}


void lr35902_cpu_coro(mco_coro* co) {
  lr35902* cpu = mco_get_user_data(co);

  while (1) {
    u8 opcode;
    opcode = cpu->read8(cpu, cpu->pc++);
    mco_yield(co);

    switch (opcode) {
    case 0x00: break;                              // NOP
    case 0x01: break;                              // LD BC,d16
    case 0x02: break;                              // LD (BC),A
    case 0x03: break;                              // INC BC
    case 0x04: break;                              // INC B
    case 0x05: break;                              // DEC B
    case 0x06: break;                              // LD B,d8
    case 0x07: break;                              // RLCA
    case 0x08: break;                              // LD (a16),SP
    case 0x09: break;                              // ADD HL,BC
    case 0x0A: break;                              // LD A,(BC)
    case 0x0B: break;                              // DEC BC
    case 0x0C: break;                              // INC C
    case 0x0D: break;                              // DEC C
    case 0x0E: break;                              // LD C,d8
    case 0x0F: break;                              // RRCA
    case 0x10: break;                              // STOP 0
    case 0x11: break;                              // LD DE,d16
    case 0x12: break;                              // LD (DE),A
    case 0x13: break;                              // INC DE
    case 0x14: break;                              // INC D
    case 0x15: break;                              // DEC D
    case 0x16: break;                              // LD D,d8
    case 0x17: break;                              // RLA
    case 0x18: break;                              // JR r8
    case 0x19: break;                              // ADD HL,DE
    case 0x1A: break;                              // LD A,(DE)
    case 0x1B: break;                              // DEC DE
    case 0x1C: break;                              // INC E
    case 0x1D: break;                              // DEC E
    case 0x1E: break;                              // LD E,d8
    case 0x1F: break;                              // RRA
    case 0x20: break;                              // JR NZ,r8
    case 0x21: break;                              // LD HL,d16
    case 0x22: break;                              // LD (HL+),A
    case 0x23: break;                              // INC HL
    case 0x24: break;                              // INC H
    case 0x25: break;                              // DEC H
    case 0x26: break;                              // LD H,d8
    case 0x27: break;                              // DAA
    case 0x28: break;                              // JR Z,r8
    case 0x29: break;                              // ADD HL,HL
    case 0x2A: break;                              // LD A,(HL+)
    case 0x2B: break;                              // DEC HL
    case 0x2C: break;                              // INC L
    case 0x2D: break;                              // DEC L
    case 0x2E: break;                              // LD L,d8
    case 0x2F: break;                              // CPL
    case 0x30: break;                              // JR NC,r8
    case 0x31: break;                              // LD SP,d16
    case 0x32: break;                              // LD (HL-),A
    case 0x33: break;                              // INC SP
    case 0x34: break;                              // INC (HL)
    case 0x35: break;                              // DEC (HL)
    case 0x36: break;                              // LD (HL),d8
    case 0x37: break;                              // SCF
    case 0x38: break;                              // JR C,r8
    case 0x39: break;                              // ADD HL,SP
    case 0x3A: break;                              // LD A,(HL-)
    case 0x3B: break;                              // DEC SP
    case 0x3C: break;                              // INC A
    case 0x3D: break;                              // DEC A
    case 0x3E: break;                              // LD A,d8
    case 0x3F: break;                              // CCF
    case 0x40: break;                              // LD B,B
    case 0x41: break;                              // LD B,C
    case 0x42: break;                              // LD B,D
    case 0x43: break;                              // LD B,E
    case 0x44: break;                              // LD B,H
    case 0x45: break;                              // LD B,L
    case 0x46: break;                              // LD B,(HL)
    case 0x47: break;                              // LD B,A
    case 0x48: break;                              // LD C,B
    case 0x49: break;                              // LD C,C
    case 0x4A: break;                              // LD C,D
    case 0x4B: break;                              // LD C,E
    case 0x4C: break;                              // LD C,H
    case 0x4D: break;                              // LD C,L
    case 0x4E: break;                              // LD C,(HL)
    case 0x4F: break;                              // LD C,A
    case 0x50: break;                              // LD D,B
    case 0x51: break;                              // LD D,C
    case 0x52: break;                              // LD D,D
    case 0x53: break;                              // LD D,E
    case 0x54: break;                              // LD D,H
    case 0x55: break;                              // LD D,L
    case 0x56: break;                              // LD D,(HL)
    case 0x57: break;                              // LD D,A
    case 0x58: break;                              // LD E,B
    case 0x59: break;                              // LD E,C
    case 0x5A: break;                              // LD E,D
    case 0x5B: break;                              // LD E,E
    case 0x5C: break;                              // LD E,H
    case 0x5D: break;                              // LD E,L
    case 0x5E: break;                              // LD E,(HL)
    case 0x5F: break;                              // LD E,A
    case 0x60: break;                              // LD H,B
    case 0x61: break;                              // LD H,C
    case 0x62: break;                              // LD H,D
    case 0x63: break;                              // LD H,E
    case 0x64: break;                              // LD H,H
    case 0x65: break;                              // LD H,L
    case 0x66: break;                              // LD H,(HL)
    case 0x67: break;                              // LD H,A
    case 0x68: break;                              // LD L,B
    case 0x69: break;                              // LD L,C
    case 0x6A: break;                              // LD L,D
    case 0x6B: break;                              // LD L,E
    case 0x6C: break;                              // LD L,H
    case 0x6D: break;                              // LD L,L
    case 0x6E: break;                              // LD L,(HL)
    case 0x6F: break;                              // LD L,A
    case 0x70: break;                              // LD (HL),B
    case 0x71: break;                              // LD (HL),C
    case 0x72: break;                              // LD (HL),D
    case 0x73: break;                              // LD (HL),E
    case 0x74: break;                              // LD (HL),H
    case 0x75: break;                              // LD (HL),L
    case 0x76: break;                              // HALT
    case 0x77: break;                              // LD (HL),A
    case 0x78: break;                              // LD A,B
    case 0x79: break;                              // LD A,C
    case 0x7A: break;                              // LD A,D
    case 0x7B: break;                              // LD A,E
    case 0x7C: break;                              // LD A,H
    case 0x7D: break;                              // LD A,L
    case 0x7E: break;                              // LD A,(HL)
    case 0x7F: break;                              // LD A,A
    case 0x80: break;                              // ADD A,B
    case 0x81: break;                              // ADD A,C
    case 0x82: break;                              // ADD A,D
    case 0x83: break;                              // ADD A,E
    case 0x84: break;                              // ADD A,H
    case 0x85: break;                              // ADD A,L
    case 0x86: break;                              // ADD A,(HL)
    case 0x87: break;                              // ADD A,A
    case 0x88: break;                              // ADC A,B
    case 0x89: break;                              // ADC A,C
    case 0x8A: break;                              // ADC A,D
    case 0x8B: break;                              // ADC A,E
    case 0x8C: break;                              // ADC A,H
    case 0x8D: break;                              // ADC A,L
    case 0x8E: break;                              // ADC A,(HL)
    case 0x8F: break;                              // ADC A,A
    case 0x90: break;                              // SUB B
    case 0x91: break;                              // SUB C
    case 0x92: break;                              // SUB D
    case 0x93: break;                              // SUB E
    case 0x94: break;                              // SUB H
    case 0x95: break;                              // SUB L
    case 0x96: break;                              // SUB (HL)
    case 0x97: break;                              // SUB A
    case 0x98: break;                              // SBC A,B
    case 0x99: break;                              // SBC A,C
    case 0x9A: break;                              // SBC A,D
    case 0x9B: break;                              // SBC A,E
    case 0x9C: break;                              // SBC A,H
    case 0x9D: break;                              // SBC A,L
    case 0x9E: break;                              // SBC A,(HL)
    case 0x9F: break;                              // SBC A,A
    case 0xA0: break;                              // AND B
    case 0xA1: break;                              // AND C
    case 0xA2: break;                              // AND D
    case 0xA3: break;                              // AND E
    case 0xA4: break;                              // AND H
    case 0xA5: break;                              // AND L
    case 0xA6: break;                              // AND (HL)
    case 0xA7: break;                              // AND A
    case 0xA8: break;                              // XOR B
    case 0xA9: break;                              // XOR C
    case 0xAA: break;                              // XOR D
    case 0xAB: break;                              // XOR E
    case 0xAC: break;                              // XOR H
    case 0xAD: break;                              // XOR L
    case 0xAE: break;                              // XOR (HL)
    case 0xAF: break;                              // XOR A
    case 0xB0: break;                              // OR B
    case 0xB1: break;                              // OR C
    case 0xB2: break;                              // OR D
    case 0xB3: break;                              // OR E
    case 0xB4: break;                              // OR H
    case 0xB5: break;                              // OR L
    case 0xB6: break;                              // OR (HL)
    case 0xB7: break;                              // OR A
    case 0xB8: break;                              // CP B
    case 0xB9: break;                              // CP C
    case 0xBA: break;                              // CP D
    case 0xBB: break;                              // CP E
    case 0xBC: break;                              // CP H
    case 0xBD: break;                              // CP L
    case 0xBE: break;                              // CP (HL)
    case 0xBF: break;                              // CP A
    case 0xC0: break;                              // RET NZ
    case 0xC1: break;                              // POP BC
    case 0xC2: break;                              // JP NZ,a16
    case 0xC3: break;                              // JP a16
    case 0xC4: break;                              // CALL NZ,a16
    case 0xC5: break;                              // PUSH BC
    case 0xC6: break;                              // ADD A,d8
    case 0xC7: break;                              // RST 00H
    case 0xC8: break;                              // RET Z
    case 0xC9: break;                              // RET
    case 0xCA: break;                              // JP Z,a16
    case 0xCB: break;                              // PREFIX CB
    case 0xCC: break;                              // CALL Z,a16
    case 0xCD: break;                              // CALL a16
    case 0xCE: break;                              // ADC A,d8
    case 0xCF: break;                              // RST 08H
    case 0xD0: break;                              // RET NC
    case 0xD1: break;                              // POP DE
    case 0xD2: break;                              // JP NC,a16
    case 0xD3: break;                              // (Illegal)
    case 0xD4: break;                              // CALL NC,a16
    case 0xD5: break;                              // PUSH DE
    case 0xD6: break;                              // SUB d8
    case 0xD7: break;                              // RST 10H
    case 0xD8: break;                              // RET C
    case 0xD9: break;                              // RETI
    case 0xDA: break;                              // JP C,a16
    case 0xDB: break;                              // (Illegal)
    case 0xDC: break;                              // CALL C,a16
    case 0xDD: break;                              // (Illegal)
    case 0xDE: break;                              // SBC A,d8
    case 0xDF: break;                              // RST 18H
    case 0xE0: break;                              // LDH (a8),A
    case 0xE1: break;                              // POP HL
    case 0xE2: break;                              // LD (C),A
    case 0xE3: break;                              // (Illegal)
    case 0xE4: break;                              // (Illegal)
    case 0xE5: break;                              // PUSH HL
    case 0xE6: break;                              // AND d8
    case 0xE7: break;                              // RST 20H
    case 0xE8: break;                              // ADD SP,r8
    case 0xE9: break;                              // JP (HL)
    case 0xEA: break;                              // LD (a16),A
    case 0xEB: break;                              // (Illegal)
    case 0xEC: break;                              // (Illegal)
    case 0xED: break;                              // (Illegal)
    case 0xEE: break;                              // XOR d8
    case 0xEF: break;                              // RST 28H
    case 0xF0: break;                              // LDH A,(a8)
    case 0xF1: break;                              // POP AF
    case 0xF2: break;                              // LD A,(C)
    case 0xF3: break;                              // DI
    case 0xF4: break;                              // (Illegal)
    case 0xF5: break;                              // PUSH AF
    case 0xF6: break;                              // OR d8
    case 0xF7: break;                              // RST 30H
    case 0xF8: break;                              // LD HL,SP+r8
    case 0xF9: break;                              // LD SP,HL
    case 0xFA: break;                              // LD A,(a16)
    case 0xFB: break;                              // EI
    case 0xFC: break;                              // (Illegal)
    case 0xFD: break;                              // (Illegal)
    case 0xFE: break;                              // CP d8
    case 0xFF: break;                              // RST 38H
  }

    // Check if we've used all our cycles
    if (cpu->remaining_cycles == 0) {
      break;
    }
  }
}

void lr35902_main() {
  lr35902 lr35902;
  lr35902_new(&lr35902);
  //lr35902_run(&lr35902, 10);
  lr35902_destroy(&lr35902);
}