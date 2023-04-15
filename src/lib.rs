extern crate core;

use core::fmt;

use yaxpeax_arch::AddressDiff;
use yaxpeax_arch::Arch;
use yaxpeax_arch::Decoder;
use yaxpeax_arch::LengthedInstruction;
use yaxpeax_arch::Reader;
use yaxpeax_arch::StandardDecodeError;

#[cfg_attr(feature="use-serde", derive(Serialize, Deserialize))]
#[derive(Debug)]
pub struct SM83;

impl Arch for SM83 {
    type Address = u16;
    type Word = u8;
    type Instruction = Instruction;
    type DecodeError = StandardDecodeError;
    type Decoder = InstDecoder;
    type Operand = Operand;
}

#[derive(Debug, Copy, Clone)]
pub struct Instruction {
    opcode: Opcode,
    operands: [Operand; 2],
    length: u8,
}

impl Instruction {
    pub fn opcode(&self) -> Opcode {
        self.opcode
    }

    pub fn operands(&self) -> &[Operand; 2] {
        &self.operands
    }

    pub fn length(&self) -> u8 {
        self.length
    }
}

impl Default for Instruction {
    fn default() -> Instruction {
        Instruction {
            opcode: Opcode::NOP,
            operands: [Operand::Nothing, Operand::Nothing],
            length: 1,
        }
    }
}

impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.opcode())?;
        let ops = self.operands();
        if ops[0] != Operand::Nothing {
            write!(f, " {}", ops[0])?;
        }
        if ops[1] != Operand::Nothing {
            write!(f, ", {}", ops[1])?;
        }
        Ok(())
    }
}

impl LengthedInstruction for Instruction {
    type Unit = AddressDiff<<SM83 as Arch>::Address>;
    fn min_size() -> Self::Unit {
        AddressDiff::from_const(1)
    }
    fn len(&self) -> Self::Unit {
        AddressDiff::from_const(self.length as u16)
    }
}

impl yaxpeax_arch::Instruction for Instruction {
    // only decode well-formed instructions (for now???)
    fn well_defined(&self) -> bool { true }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Operand {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    AF,
    BC,
    DE,
    HL,
    SP,
    DerefHL,
    DerefBC,
    DerefDE,
    DerefDecHL,
    DerefIncHL,
    DerefHighC,
    DerefHighD8(u8),
    SPWithOffset(i8),
    Bit(u8),
    D8(u8),
    I8(i8),
    R8(i8),
    A16(u16),
    D16(u16),

    CondC,
    CondNC,
    CondZ,
    CondNZ,

    Nothing,
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Operand::A => write!(f, "a"),
            Operand::B => write!(f, "b"),
            Operand::C => write!(f, "c"),
            Operand::D => write!(f, "d"),
            Operand::E => write!(f, "e"),
            Operand::H => write!(f, "h"),
            Operand::L => write!(f, "l"),
            Operand::AF => write!(f, "af"),
            Operand::BC => write!(f, "bc"),
            Operand::DE => write!(f, "de"),
            Operand::HL => write!(f, "hl"),
            Operand::SP => write!(f, "sp"),
            Operand::DerefHL => write!(f, "[hl]"),
            Operand::DerefBC => write!(f, "[bc]"),
            Operand::DerefDE => write!(f, "[de]"),
            Operand::DerefDecHL => write!(f, "[hl-]"),
            Operand::DerefIncHL => write!(f, "[hl+]"),
            Operand::DerefHighC => write!(f, "[0xff00 + c]"),
            Operand::DerefHighD8(imm) => write!(f, "[$ff00 + ${:02x}]", imm),
            Operand::SPWithOffset(imm) => {
                if *imm == -128 {
                    write!(f, "sp - $80")
                } else if *imm >= 0 {
                    write!(f, "sp + ${:02x}", imm)
                } else {
                    write!(f, "sp - ${:02x}", -imm)
                }
            }
            Operand::Bit(imm) => write!(f, "{}", imm),
            Operand::D8(imm) => write!(f, "${:02x}", imm),
            Operand::D16(imm) => write!(f, "${:04x}", imm),
            Operand::I8(imm) => {
                if *imm == -128 {
                    write!(f, "-0x80")
                } else if *imm >= 0 {
                    write!(f, "${:02x}", imm)
                } else {
                    write!(f, "-${:02x}", -imm)
                }
            }
            Operand::R8(imm) => {
                if *imm == -128 {
                    write!(f, "$-0x80")
                } else if *imm >= 0 {
                    write!(f, "$+${:02x}", imm)
                } else {
                    write!(f, "$-${:02x}", -imm)
                }
            }
            Operand::A16(addr) => write!(f, "[${:4x}]", addr),

            Operand::CondC => write!(f, "C"),
            Operand::CondNC => write!(f, "nC"),
            Operand::CondZ => write!(f, "Z"),
            Operand::CondNZ => write!(f, "nZ"),

            Operand::Nothing => write!(f, "nothing (BUG: should not be shown)"),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Opcode {
    NOP,

    LD,

    DEC,
    INC,

    ADC,
    ADD,
    SBC,
    SUB,

    AND,
    XOR,
    OR,
    CP,

    POP,
    PUSH,

    JP,
    JR,
    CALL,
    RET,
    RETI,
    HALT,
    RST,
    STOP,

    RLCA,
    RRCA,
    RLA,
    RRA,

    DAA,
    CPL,
    SCF,
    CCF,

    DI,
    EI,

    LDH,

    RLC,
    RRC,
    RL,
    RR,
    SLA,
    SRA,
    SWAP,
    SRL,
    BIT = 0x40,
    RES = 0x80,
    SET = 0xc0,
}

impl fmt::Display for Opcode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Opcode::NOP => write!(f, "nop"),

            Opcode::LD => write!(f, "ld"),

            Opcode::DEC => write!(f, "dec"),
            Opcode::INC => write!(f, "inc"),

            Opcode::ADC => write!(f, "adc"),
            Opcode::ADD => write!(f, "add"),
            Opcode::SBC => write!(f, "sbc"),
            Opcode::SUB => write!(f, "sub"),

            Opcode::AND => write!(f, "and"),
            Opcode::XOR => write!(f, "xor"),
            Opcode::OR => write!(f, "or"),
            Opcode::CP => write!(f, "cp"),

            Opcode::POP => write!(f, "pop"),
            Opcode::PUSH => write!(f, "push"),

            Opcode::JP => write!(f, "jp"),
            Opcode::JR => write!(f, "jr"),
            Opcode::CALL => write!(f, "call"),
            Opcode::RET => write!(f, "ret"),
            Opcode::RETI => write!(f, "reti"),
            Opcode::HALT => write!(f, "halt"),
            Opcode::RST => write!(f, "rst"),
            Opcode::STOP => write!(f, "stop"),

            Opcode::RLCA => write!(f, "rlca"),
            Opcode::RRCA => write!(f, "rrca"),
            Opcode::RLA => write!(f, "rla"),
            Opcode::RRA => write!(f, "rra"),

            Opcode::DAA => write!(f, "daa"),
            Opcode::CPL => write!(f, "cpl"),
            Opcode::SCF => write!(f, "scf"),
            Opcode::CCF => write!(f, "ccf"),

            Opcode::DI => write!(f, "di"),
            Opcode::EI => write!(f, "ei"),

            Opcode::LDH => write!(f, "ldh"),

            Opcode::RLC => write!(f, "rlc"),
            Opcode::RRC => write!(f, "rrc"),
            Opcode::RL => write!(f, "rl"),
            Opcode::RR => write!(f, "rr"),
            Opcode::SLA => write!(f, "sla"),
            Opcode::SRA => write!(f, "sra"),
            Opcode::SWAP => write!(f, "swap"),
            Opcode::SRL => write!(f, "srl"),
            Opcode::BIT => write!(f, "bit"),
            Opcode::RES => write!(f, "res"),
            Opcode::SET => write!(f, "set"),
        }
    }
}

#[derive(Debug)]
pub struct InstDecoder { }

impl Default for InstDecoder {
    fn default() -> Self {
        InstDecoder { }
    }
}

pub trait DecodeHandler<T: Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>> {
    #[inline(always)]
    fn read_u8(&mut self, words: &mut T) -> Result<u8, <SM83 as Arch>::DecodeError> {
        let b = words.next()?;
        self.on_word_read(b);
        Ok(b)
    }
    #[inline(always)]
    fn read_u16(&mut self, words: &mut T) -> Result<u16, <SM83 as Arch>::DecodeError> {
        let mut buf = [0u8; 2];
        words.next_n(&mut buf).ok().ok_or(StandardDecodeError::ExhaustedInput)?;
        self.on_word_read(buf[0]);
        self.on_word_read(buf[1]);
        Ok(u16::from_le_bytes(buf))
    }
    fn on_decode_start(&mut self) {}
    fn on_decode_end(&mut self) {}
    fn on_opcode_decoded(&mut self, _opcode: Opcode) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_operand_decoded(&mut self, _which: u8, _operand: Operand) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_word_read(&mut self, _word: <SM83 as Arch>::Word) {}

    // these functions are all callbacks once we've determined an instruction fairly precisely.
    // by default they are all no-ops. users will often need only either these, or
    // `on_<instruction-item>_decoded`.
    fn on_ld_8b_mem_a(&mut self, _reg: DerefReg) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_8b_a_mem(&mut self, _reg: DerefReg) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_a16_sp(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_rr_d16(&mut self, _op: Reg16b, _v: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_r_r(&mut self, _op: Reg8b, _op1: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_sp_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_a_8b_deref_addr(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_8b_deref_addr_a(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_hl_sp_offset(&mut self, _ofs: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ldh_a_deref_high_8b(&mut self, _ofs: u8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ldh_deref_high_8b_a(&mut self, _ofs: u8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ldh_deref_high_c_a(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ldh_a_deref_high_c(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_add_16b_hl_rr(&mut self, _op: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_add_sp_i8(&mut self, _imm: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_inc_8b(&mut self, _op0: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_inc_16b_rr(&mut self, _op0: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_dec_8b(&mut self, _op0: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_dec_16b_rr(&mut self, _op0: Reg16b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ld_8b_imm(&mut self, _op0: Reg8b, _imm: u8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jr_unconditional(&mut self, _rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jr_nz(&mut self, _rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jr_z(&mut self, _rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jr_nc(&mut self, _rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jr_c(&mut self, _rel: i8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_unconditional(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_nz(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_z(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_nc(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_c(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_jp_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_call_unconditional(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_call_nz(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_call_z(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_call_nc(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_call_c(&mut self, _addr: u16) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_bit_op(&mut self, _op: BitOp, _bit: u8, _operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_op_8b_a_r(&mut self, _op: Op8bAOp, _operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_op_8b_a_d8(&mut self, _op: Op8bAOp, _operand: u8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rotate_8b_r(&mut self, _op: RotOp, _operand: Reg8b) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_push_af(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_pop_af(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_push_bc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_pop_bc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_push_de(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_pop_de(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_push_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_pop_hl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_nop(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_stop(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_halt(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rlca(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rrca(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rla(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rra(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_daa(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_cpl(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_scf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ccf(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ret_nz(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ret_z(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ret_nc(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ret_c(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_rst(&mut self, _imm: u8) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_reti(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_di(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ei(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
    fn on_ret_unconditional(&mut self) -> Result<(), <SM83 as Arch>::DecodeError> { Ok(()) }
}

#[derive(Copy, Clone, Debug)]
pub enum BitOp {
    BIT = 0x40,
    RES = 0x80,
    SET = 0xc0,
}
impl Into<Opcode> for BitOp {
    #[inline(always)]
    fn into(self) -> Opcode {
        match self {
            BitOp::BIT => Opcode::BIT,
            BitOp::RES => Opcode::RES,
            BitOp::SET => Opcode::SET,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum RotOp {
    RLC,
    RRC,
    RL,
    RR,
    SLA,
    SRA,
    SWAP,
    SRL,
}
impl Into<Opcode> for RotOp {
    #[inline(always)]
    fn into(self) -> Opcode {
        match self {
            RotOp::RLC => Opcode::RLC,
            RotOp::RRC => Opcode::RRC,
            RotOp::RL => Opcode::RL,
            RotOp::RR => Opcode::RR,
            RotOp::SLA => Opcode::SLA,
            RotOp::SRA => Opcode::SRA,
            RotOp::SWAP => Opcode::SWAP,
            RotOp::SRL => Opcode::SRL,
        }
    }
}

/// opcodes selected by `on_op_8b_a_r` or `on_op_8b_a_d8`. in that context, these execute with an
/// implicit first operand of `a`, and second operand either selected by a `Reg8b` variant or 8b
/// immediate.
#[derive(Copy, Clone, Debug)]
pub enum Op8bAOp {
    ADD,
    ADC,
    SUB,
    SBC,
    AND,
    XOR,
    OR,
    CP,
}

impl Into<Opcode> for Op8bAOp {
    #[inline(always)]
    fn into(self) -> Opcode {
        match self {
            Op8bAOp::ADD => Opcode::ADD,
            Op8bAOp::ADC => Opcode::ADC,
            Op8bAOp::SUB => Opcode::SUB,
            Op8bAOp::SBC => Opcode::SBC,
            Op8bAOp::AND => Opcode::AND,
            Op8bAOp::XOR => Opcode::XOR,
            Op8bAOp::OR => Opcode::OR,
            Op8bAOp::CP => Opcode::CP,
        }
    }
}

/// main operand set used for ld, arithmetic, and $CB-prefixed instructions.
#[derive(Copy, Clone, Debug)]
pub enum Reg8b {
    B, C, D, E,
    H, L, DerefHL, A,
}

impl TryFrom<u8> for Reg8b {
    type Error=u8;

    #[inline(always)]
    fn try_from(value: u8) -> Result<Reg8b, u8> {
        match value {
            0 => Ok(Reg8b::B),
            1 => Ok(Reg8b::C),
            2 => Ok(Reg8b::D),
            3 => Ok(Reg8b::E),
            4 => Ok(Reg8b::H),
            5 => Ok(Reg8b::L),
            6 => Ok(Reg8b::DerefHL),
            7 => Ok(Reg8b::A),
            o => Err(o),
        }
    }
}
impl Into<Operand> for Reg8b {
    #[inline(always)]
    fn into(self) -> Operand {
        match self {
            Reg8b::B => Operand::B,
            Reg8b::C => Operand::C,
            Reg8b::D => Operand::D,
            Reg8b::E => Operand::E,
            Reg8b::H => Operand::H,
            Reg8b::L => Operand::L,
            Reg8b::DerefHL => Operand::DerefHL,
            Reg8b::A => Operand::A,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Reg16b {
    BC,
    DE,
    HL,
    SP
}

impl TryFrom<u8> for Reg16b {
    type Error=u8;

    #[inline(always)]
    fn try_from(value: u8) -> Result<Reg16b, u8> {
        match value {
            0 => Ok(Reg16b::BC),
            1 => Ok(Reg16b::DE),
            2 => Ok(Reg16b::HL),
            3 => Ok(Reg16b::SP),
            o => Err(o),
        }
    }
}
impl Into<Operand> for Reg16b {
    #[inline(always)]
    fn into(self) -> Operand {
        match self {
            Reg16b::BC => Operand::BC,
            Reg16b::DE => Operand::DE,
            Reg16b::HL => Operand::HL,
            Reg16b::SP => Operand::SP,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum DerefReg {
    DerefBC = 0,
    DerefDE = 1,
    DerefIncHL = 2,
    DerefDecHL = 3,
}

impl TryFrom<u8> for DerefReg {
    type Error=u8;

    #[inline(always)]
    fn try_from(value: u8) -> Result<DerefReg, u8> {
        match value {
            0 => Ok(DerefReg::DerefBC),
            1 => Ok(DerefReg::DerefDE),
            2 => Ok(DerefReg::DerefIncHL),
            3 => Ok(DerefReg::DerefDecHL),
            o => Err(o),
        }
    }
}
impl Into<Operand> for DerefReg {
    #[inline(always)]
    fn into(self) -> Operand {
        match self {
            DerefReg::DerefBC => Operand::DerefBC,
            DerefReg::DerefDE => Operand::DerefDE,
            DerefReg::DerefIncHL => Operand::DerefIncHL,
            DerefReg::DerefDecHL => Operand::DerefDecHL,
        }
    }
}

impl<T: yaxpeax_arch::Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>> DecodeHandler<T> for &mut u8 {
    fn on_decode_start(&mut self) {
        **self = 0;
    }

    fn on_word_read(&mut self, _word: <SM83 as Arch>::Word) {
        **self = self.wrapping_add(1);
    }
}

impl<T: yaxpeax_arch::Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>> DecodeHandler<T> for Instruction {
    fn on_decode_start(&mut self) {
        self.length = 0;
        self.opcode = Opcode::STOP;
        self.operands = [Operand::Nothing, Operand::Nothing];
    }

    fn on_opcode_decoded(&mut self, opcode: Opcode) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.opcode = opcode;
        Ok(())
    }

    fn on_operand_decoded(&mut self, which: u8, operand: Operand) -> Result<(), <SM83 as Arch>::DecodeError> {
        self.operands[which as usize] = operand;
        Ok(())
    }

    fn on_word_read(&mut self, _word: <SM83 as Arch>::Word) {
        self.length += 1;
    }
}

pub fn decode_inst<
    T: Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>,
    H: DecodeHandler<T>,
>(
    _decoder: &<SM83 as Arch>::Decoder,
    handler: &mut H,
    words: &mut T,
) -> Result<(), <SM83 as Arch>::DecodeError> {
    handler.on_decode_start();

    let opc: u8 = handler.read_u8(words)?;

    let high = (opc >> 3) & 0b111;
    let low = opc & 0b111;

    // early part of the table is more varied, more special cases
    if opc < 0x40 {
        match low {
            0 => {
                // special case on high
                match high {
                    0 => {
                        handler.on_opcode_decoded(Opcode::NOP)?;
                        handler.on_nop()?;
                    },
                    1 => {
                        handler.on_opcode_decoded(Opcode::LD)?;
                        let addr = handler.read_u16(words)?;
                        handler.on_operand_decoded(0, Operand::A16(addr))?;
                        handler.on_operand_decoded(1, Operand::SP)?;
                        handler.on_ld_a16_sp(addr)?;
                    },
                    2 => {
                        handler.on_opcode_decoded(Opcode::STOP)?;
                        handler.on_stop()?;
                    },
                    3 => {
                        handler.on_opcode_decoded(Opcode::JR)?;
                        let rel = handler.read_u8(words)?;
                        handler.on_operand_decoded(0, Operand::R8(rel as i8))?;
                        handler.on_jr_unconditional(rel as i8)?;
                    },
                    4 => {
                        handler.on_opcode_decoded(Opcode::JR)?;
                        let rel = handler.read_u8(words)?;
                        handler.on_operand_decoded(0, Operand::CondNZ)?;
                        handler.on_operand_decoded(1, Operand::R8(rel as i8))?;
                        handler.on_jr_nz(rel as i8)?;
                    }
                    5 => {
                        handler.on_opcode_decoded(Opcode::JR)?;
                        let rel = handler.read_u8(words)?;
                        handler.on_operand_decoded(0, Operand::CondZ)?;
                        handler.on_operand_decoded(1, Operand::R8(rel as i8))?;
                        handler.on_jr_z(rel as i8)?;
                    }
                    6 => {
                        handler.on_opcode_decoded(Opcode::JR)?;
                        let rel = handler.read_u8(words)?;
                        handler.on_operand_decoded(0, Operand::CondNC)?;
                        handler.on_operand_decoded(1, Operand::R8(rel as i8))?;
                        handler.on_jr_nc(rel as i8)?;
                    }
                    7 => {
                        handler.on_opcode_decoded(Opcode::JR)?;
                        let rel = handler.read_u8(words)?;
                        handler.on_operand_decoded(0, Operand::CondC)?;
                        handler.on_operand_decoded(1, Operand::R8(rel as i8))?;
                        handler.on_jr_c(rel as i8)?;
                    }
                    _ => { unreachable!("impossible bit pattern"); }
                }
            }
            1 => {
                // also special case, but fewer
                if high & 1 == 0 {
                    handler.on_opcode_decoded(Opcode::LD)?;
                    let d16 = handler.read_u16(words)?;
                    let op = Reg16b::try_from(high >> 1).expect("bit pattern is possible");
                    handler.on_operand_decoded(0, op.into())?;
                    handler.on_operand_decoded(1, Operand::D16(d16))?;
                    handler.on_ld_rr_d16(op, d16)?;
                } else {
                    handler.on_opcode_decoded(Opcode::ADD)?;
                    handler.on_operand_decoded(0, Operand::HL)?;
                    let op = Reg16b::try_from(high >> 1).expect("bit pattern is possible");
                    handler.on_operand_decoded(1, op.into())?;
                    handler.on_add_16b_hl_rr(op)?;
                }
            }
            2 => {
                handler.on_opcode_decoded(Opcode::LD)?;
                let op = DerefReg::try_from(high >> 1).expect("bit pattern is possible");
                if high & 1 == 0 {
                    handler.on_ld_8b_mem_a(op)?;
                    handler.on_operand_decoded(0, op.into())?;
                    handler.on_operand_decoded(1, Operand::A)?;
                } else {
                    handler.on_ld_8b_a_mem(op)?;
                    handler.on_operand_decoded(0, Operand::A)?;
                    handler.on_operand_decoded(1, op.into())?;
                }
            }
            3 => {
                if high & 1 == 0 {
                    handler.on_opcode_decoded(Opcode::INC)?;
                    let op = Reg16b::try_from(high >> 1).expect("bit pattern is possible");
                    handler.on_operand_decoded(0, op.into())?;
                    handler.on_inc_16b_rr(op)?;
                } else {
                    handler.on_opcode_decoded(Opcode::DEC)?;
                    let op = Reg16b::try_from(high >> 1).expect("bit pattern is possible");
                    handler.on_operand_decoded(0, op.into())?;
                    handler.on_dec_16b_rr(op)?;
                }
            }
            4 => {
                handler.on_opcode_decoded(Opcode::INC)?;
                let op = Reg8b::try_from(high).expect("bit pattern is possible");
                handler.on_inc_8b(op)?;
                handler.on_operand_decoded(0, op.into())?;
            }
            5 => {
                handler.on_opcode_decoded(Opcode::DEC)?;
                let op = Reg8b::try_from(high).expect("bit pattern is possible");
                handler.on_dec_8b(op)?;
                handler.on_operand_decoded(0, op.into())?;
            }
            6 => {
                handler.on_opcode_decoded(Opcode::LD)?;
                let op = Reg8b::try_from(high).expect("bit pattern is possible");
                let imm = handler.read_u8(words)?;
                handler.on_ld_8b_imm(op, imm)?;
                handler.on_operand_decoded(0, op.into())?;
                handler.on_operand_decoded(1, Operand::D8(imm))?;
            }
            7 => {
                // special cases here too
                let (op, f) = match high {
                    0 => (Opcode::RLCA, H::on_rlca as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    1 => (Opcode::RRCA, H::on_rrca as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    2 => (Opcode::RLA, H::on_rla as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    3 => (Opcode::RRA, H::on_rra as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    4 => (Opcode::DAA, H::on_daa as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    5 => (Opcode::CPL, H::on_cpl as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    6 => (Opcode::SCF, H::on_scf as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    7 => (Opcode::CCF, H::on_ccf as fn(&mut H) -> Result<(), <SM83 as Arch>::DecodeError>),
                    _ => { unreachable!("trivially dce...") },
                };
                handler.on_opcode_decoded(op)?;
                f(handler)?;
            }
            _ => {
                unreachable!("impossible bit pattern");
            }
        };
        return Ok(());
    } else if opc < 0x80 {
        if opc == 0x76 {
            handler.on_opcode_decoded(Opcode::HALT)?;
            handler.on_halt()?;
            return Ok(());
        } else {
            let l = Reg8b::try_from(high).expect("bit pattern is possible");
            let r = Reg8b::try_from(low).expect("bit pattern is possible");
            handler.on_opcode_decoded(Opcode::LD)?;
            handler.on_operand_decoded(0, l.into())?;
            handler.on_operand_decoded(1, r.into())?;
            handler.on_ld_r_r(l, r)?;
            return Ok(());
        }
    } else if opc < 0xc0 {
        let op = match high {
            0 => Op8bAOp::ADD,
            1 => Op8bAOp::ADC,
            2 => Op8bAOp::SUB,
            3 => Op8bAOp::SBC,
            4 => Op8bAOp::AND,
            5 => Op8bAOp::XOR,
            6 => Op8bAOp::OR,
            7 => Op8bAOp::CP,
            _ => { unreachable!("dce2") }
        };
        handler.on_opcode_decoded(op.into())?;
        let op0 = Reg8b::try_from(low).expect("bit pattern is possible");
        handler.on_op_8b_a_r(op, op0)?;
        handler.on_operand_decoded(0, op0.into())?;
        return Ok(());
    } else {
        match opc {
    // 0xc0
    0xc0 => {
        handler.on_opcode_decoded(Opcode::RET)?;
        handler.on_operand_decoded(0, Operand::CondNZ)?;
        handler.on_ret_nz()
    },
    0xc1 => {
        handler.on_opcode_decoded(Opcode::POP)?;
        handler.on_operand_decoded(0, Operand::BC)?;
        handler.on_pop_bc()
    },
    0xc2 => {
        handler.on_opcode_decoded(Opcode::JP)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondNZ)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_jp_nz(addr)
    },
    0xc3 => {
        handler.on_opcode_decoded(Opcode::JP)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::D16(addr))?;
        handler.on_jp_unconditional(addr)
    },
    0xc4 => {
        handler.on_opcode_decoded(Opcode::CALL)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondNZ)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_call_nz(addr)
    },
    0xc5 => {
        handler.on_opcode_decoded(Opcode::PUSH)?;
        handler.on_operand_decoded(0, Operand::BC)?;
        handler.on_push_bc()
    },
    0xc6 => {
                    handler.on_opcode_decoded(Opcode::ADD)?;
                    let imm = handler.read_u8(words)?;
                    handler.on_operand_decoded(0, Operand::D8(imm))?;
                    handler.on_op_8b_a_d8(Op8bAOp::ADD, imm)
    },
    0xc7 => {
                    handler.on_opcode_decoded(Opcode::RST)?;
                    handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
                    handler.on_rst(opc & 0b00_111_000)
    },
    // 0xc8
    0xc8 => {
        handler.on_opcode_decoded(Opcode::RET)?;
        handler.on_operand_decoded(0, Operand::CondZ)?;
        handler.on_ret_z()
    },
    0xc9 => {
        handler.on_opcode_decoded(Opcode::RET)?;
        handler.on_ret_unconditional()
    },
    0xca => {
        handler.on_opcode_decoded(Opcode::JP)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondZ)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_jp_z(addr)
    },
    0xcb => {
            // sm83 special CB-prefixed instructions
            let opc: u8 = handler.read_u8(words)?;
            let opbits = opc & 0b11_000_000;
            if opbits == 0x00 {
                let high = (opc >> 3) & 0b111;
                let low = opc & 0b111;
                let op = match high {
                    0 => RotOp::RLC,
                    1 => RotOp::RRC,
                    2 => RotOp::RL,
                    3 => RotOp::RR,
                    4 => RotOp::SLA,
                    5 => RotOp::SRA,
                    6 => RotOp::SWAP,
                    7 => RotOp::SRL,
                    _ => { unreachable!("dce3") },
                };
                let op0 = Reg8b::try_from(low).expect("bit pattern is possible");
                handler.on_opcode_decoded(op.into())?;
                handler.on_operand_decoded(0, op0.into())?;
                handler.on_rotate_8b_r(op, op0)?;
                return Ok(());
            } else {
                let bit = (opc >> 3) & 0b111;
                let low = opc & 0b111;
                // safety: `opbits` is 0bXX_000_000 and known to be not 0, so it is 0x40, 0x80, or
                // 0xc0. these are the same values as the BitOp enum variants.
                let op: BitOp = unsafe { std::mem::transmute::<u8, BitOp>(opbits) };
                handler.on_opcode_decoded(op.into())?;
                let op1 = Reg8b::try_from(low).expect("bit pattern is possible");
                handler.on_operand_decoded(0, Operand::Bit(bit))?;
                handler.on_operand_decoded(1, op1.into())?;

                handler.on_bit_op(op, bit, op1)?;
                return Ok(());
            }
    },
    0xcc => {
        handler.on_opcode_decoded(Opcode::CALL)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondZ)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_call_z(addr)
    },
    0xcd => {
        handler.on_opcode_decoded(Opcode::CALL)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::D16(addr))?;
        handler.on_call_unconditional(addr)
    },
    0xce => {
        handler.on_opcode_decoded(Opcode::ADC)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::ADC, imm)
    },
    0xcf => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xd0
    0xd0 => {
        handler.on_opcode_decoded(Opcode::RET)?;
        handler.on_operand_decoded(0, Operand::CondNC)?;
        handler.on_ret_nc()
    },
    0xd1 => {
        handler.on_opcode_decoded(Opcode::POP)?;
        handler.on_operand_decoded(0, Operand::DE)?;
        handler.on_pop_de()
    },
    0xd2 => {
        handler.on_opcode_decoded(Opcode::JP)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondNC)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_jp_nc(addr)
    },
    0xd3 => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xd4 => {
        handler.on_opcode_decoded(Opcode::CALL)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondNC)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_call_nc(addr)
    },
    0xd5 => {
        handler.on_opcode_decoded(Opcode::PUSH)?;
        handler.on_operand_decoded(0, Operand::DE)?;
        handler.on_push_de()
    },
    0xd6 => {
        handler.on_opcode_decoded(Opcode::SUB)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::SUB, imm)
    },
    0xd7 => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xd8
    0xd8 => {
        handler.on_opcode_decoded(Opcode::RET)?;
        handler.on_operand_decoded(0, Operand::CondC)?;
        handler.on_ret_c()
    },
    0xd9 => {
        handler.on_opcode_decoded(Opcode::RETI)?;
        handler.on_reti()
    },
    0xda => {
        handler.on_opcode_decoded(Opcode::JP)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondC)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_jp_c(addr)
    },
    0xdb => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xdc => {
        handler.on_opcode_decoded(Opcode::CALL)?;
        let addr = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::CondC)?;
        handler.on_operand_decoded(1, Operand::D16(addr))?;
        handler.on_call_c(addr)
    },
    0xdd => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xde => {
        handler.on_opcode_decoded(Opcode::SBC)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::SBC, imm)
    },
    0xdf => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xe0
    0xe0 => {
        handler.on_opcode_decoded(Opcode::LDH)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::DerefHighD8(imm))?;
        handler.on_operand_decoded(1, Operand::A)?;
        handler.on_ldh_deref_high_8b_a(imm)
    },
    0xe1 => {
        handler.on_opcode_decoded(Opcode::POP)?;
        handler.on_operand_decoded(0, Operand::HL)?;
        handler.on_pop_hl()
    },
    0xe2 => {
        handler.on_opcode_decoded(Opcode::LDH)?;
        handler.on_operand_decoded(0, Operand::DerefHighC)?;
        handler.on_operand_decoded(1, Operand::A)?;
        handler.on_ldh_deref_high_c_a()
    },
    0xe3 => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xe4 => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xe5 => {
        handler.on_opcode_decoded(Opcode::PUSH)?;
        handler.on_operand_decoded(0, Operand::HL)?;
        handler.on_push_hl()
    },
    0xe6 => {
        handler.on_opcode_decoded(Opcode::AND)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::AND, imm)
    },
    0xe7 => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xe8
    0xe8 => {
        handler.on_opcode_decoded(Opcode::ADD)?;
        let imm = handler.read_u8(words)? as i8;
        handler.on_operand_decoded(0, Operand::SP)?;
        handler.on_operand_decoded(1, Operand::I8(imm))?;
        handler.on_add_sp_i8(imm)
    },
    0xe9 => {
        handler.on_opcode_decoded(Opcode::JP)?;
        handler.on_operand_decoded(0, Operand::HL)?;
        handler.on_jp_hl()
    },
    0xea => {
        handler.on_opcode_decoded(Opcode::LD)?;
        let imm = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::A16(imm))?;
        handler.on_operand_decoded(1, Operand::A)?;
        handler.on_ld_8b_deref_addr_a(imm)
    },
    0xeb => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xec => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xed => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xee => {
        handler.on_opcode_decoded(Opcode::XOR)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::XOR, imm)
    },
    0xef => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xf0
    0xf0 => {
        handler.on_opcode_decoded(Opcode::LDH)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::A)?;
        handler.on_operand_decoded(1, Operand::DerefHighD8(imm))?;
        handler.on_ldh_a_deref_high_8b(imm)
    },
    0xf1 => {
        handler.on_opcode_decoded(Opcode::POP)?;
        handler.on_operand_decoded(0, Operand::AF)?;
        handler.on_pop_af()
    },
    0xf2 => {
        handler.on_opcode_decoded(Opcode::LDH)?;
        handler.on_operand_decoded(0, Operand::A)?;
        handler.on_ldh_a_deref_high_c() },
    0xf3 => {
        handler.on_opcode_decoded(Opcode::DI)?;
        handler.on_di()
    },
    0xf4 => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xf5 => {
        handler.on_opcode_decoded(Opcode::PUSH)?;
        handler.on_operand_decoded(0, Operand::AF)?;
        handler.on_push_af()
    },
    0xf6 => {
        handler.on_opcode_decoded(Opcode::OR)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::OR, imm)
    },
    0xf7 => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
    // 0xf8
    0xf8 => {
        handler.on_opcode_decoded(Opcode::LD)?;
        let imm = handler.read_u8(words)? as i8;
        handler.on_operand_decoded(0, Operand::HL)?;
        handler.on_operand_decoded(1, Operand::SPWithOffset(imm))?;
        handler.on_ld_hl_sp_offset(imm)
    },
    0xf9 => {
        handler.on_opcode_decoded(Opcode::LD)?;
        handler.on_operand_decoded(0, Operand::SP)?;
        handler.on_operand_decoded(1, Operand::HL)?;
        handler.on_ld_sp_hl()
    },
    0xfa => {
        handler.on_opcode_decoded(Opcode::LD)?;
        let imm = handler.read_u16(words)?;
        handler.on_operand_decoded(0, Operand::A)?;
        handler.on_operand_decoded(1, Operand::A16(imm))?;
        handler.on_ld_a_8b_deref_addr(imm)
    },
    0xfb => {
        handler.on_opcode_decoded(Opcode::EI)?;
        handler.on_ei()
    },
    0xfc => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xfd => {
        Err(StandardDecodeError::InvalidOpcode)
    },
    0xfe => {
        handler.on_opcode_decoded(Opcode::CP)?;
        let imm = handler.read_u8(words)?;
        handler.on_operand_decoded(0, Operand::D8(imm))?;
        handler.on_op_8b_a_d8(Op8bAOp::CP, imm)
    },
    0xff => {
        handler.on_opcode_decoded(Opcode::RST)?;
        handler.on_operand_decoded(0, Operand::D8(opc & 0b00_111_000))?;
        handler.on_rst(opc & 0b00_111_000)
    },
            _ => { unsafe { std::hint::unreachable_unchecked() } },
        }
    }
}

impl Decoder<SM83> for InstDecoder {
    fn decode_into<T: Reader<<SM83 as Arch>::Address, <SM83 as Arch>::Word>>(&self, inst: &mut Instruction, words: &mut T) -> Result<(), <SM83 as Arch>::DecodeError> {
        decode_inst(self, inst, words)
    }
}
