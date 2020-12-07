#[cfg(feature="use-serde")]
#[macro_use] extern crate serde_derive;
#[cfg(feature="use-serde")]
extern crate serde;

extern crate yaxpeax_arch;

extern crate core;

use core::fmt;

use yaxpeax_arch::AddressDiff;
use yaxpeax_arch::Arch;
use yaxpeax_arch::Decoder;
use yaxpeax_arch::LengthedInstruction;

#[cfg(feature="use-serde")]
#[derive(Debug, Serialize, Deserialize)]
pub struct SM83;

#[cfg(not(feature="use-serde"))]
#[derive(Debug)]
pub struct SM83;

impl Arch for SM83 {
    type Address = u16;
    type Instruction = Instruction;
    type DecodeError = DecodeError;
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

#[derive(Debug, PartialEq)]
pub enum DecodeError {
    ExhaustedInput,
    InvalidOpcode,
    Incomplete,
}

impl yaxpeax_arch::DecodeError for DecodeError {
    fn data_exhausted(&self) -> bool { self == &DecodeError::ExhaustedInput }
    fn bad_opcode(&self) -> bool { self == &DecodeError::InvalidOpcode }
    fn bad_operand(&self) -> bool { false }
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            DecodeError::ExhaustedInput => { write!(f, "exhausted input") }
            DecodeError::InvalidOpcode => { write!(f, "invalid opcode") }
            DecodeError::Incomplete => { write!(f, "incomplete") }
        }
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
                    write!(f, "[sp - $80]")
                } else if *imm >= 0 {
                    write!(f, "[sp + ${:02x}]", imm)
                } else {
                    write!(f, "[sp - ${:02x}]", -imm)
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
enum OperandSpec {
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
    DerefHighD8,
    SPWithOffset,
    D8,
    R8,
    I8,
    A16,
    D16,

    CondC,
    CondNC,
    CondZ,
    CondNZ,

    Bit(u8),
    Imm(u8),

    Nothing,
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
    BIT,
    RES,
    SET,
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

// main operand map is used for ld, arithmetic, and $CB-prefixed instructions
const OPMAP: [OperandSpec; 8] = [
    OperandSpec::B,
    OperandSpec::C,
    OperandSpec::D,
    OperandSpec::E,
    OperandSpec::H,
    OperandSpec::L,
    OperandSpec::DerefHL,
    OperandSpec::A,
];

const UPPER_INSTRUCTIONS: [(Option<Opcode>, [OperandSpec; 2]); 64] = [
    // 0xc0
    (Some(Opcode::RET), [OperandSpec::CondNZ, OperandSpec::Nothing]),
    (Some(Opcode::POP), [OperandSpec::BC, OperandSpec::Nothing]),
    (Some(Opcode::JP), [OperandSpec::CondNZ, OperandSpec::D16]),
    (Some(Opcode::JP), [OperandSpec::D16, OperandSpec::Nothing]),
    (Some(Opcode::CALL), [OperandSpec::CondNZ, OperandSpec::D16]),
    (Some(Opcode::PUSH), [OperandSpec::BC, OperandSpec::Nothing]),
    (Some(Opcode::ADD), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x00), OperandSpec::Nothing]),
    // 0xc8
    (Some(Opcode::RET), [OperandSpec::CondZ, OperandSpec::Nothing]),
    (Some(Opcode::RET), [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::JP), [OperandSpec::CondNC, OperandSpec::D16]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::CALL), [OperandSpec::CondZ, OperandSpec::D16]),
    (Some(Opcode::CALL), [OperandSpec::D16, OperandSpec::Nothing]),
    (Some(Opcode::ADC), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x08), OperandSpec::Nothing]),
    // 0xd0
    (Some(Opcode::RET), [OperandSpec::CondNC, OperandSpec::Nothing]),
    (Some(Opcode::POP), [OperandSpec::DE, OperandSpec::Nothing]),
    (Some(Opcode::JP), [OperandSpec::CondNC, OperandSpec::D16]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::CALL), [OperandSpec::CondNC, OperandSpec::D16]),
    (Some(Opcode::PUSH), [OperandSpec::DE, OperandSpec::Nothing]),
    (Some(Opcode::SUB), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x10), OperandSpec::Nothing]),
    // 0xd8
    (Some(Opcode::RET), [OperandSpec::CondC, OperandSpec::Nothing]),
    (Some(Opcode::RETI), [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::JP), [OperandSpec::CondC, OperandSpec::D16]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::CALL), [OperandSpec::CondC, OperandSpec::D16]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::SBC), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x18), OperandSpec::Nothing]),
    // 0xe0
    (Some(Opcode::LDH), [OperandSpec::DerefHighD8, OperandSpec::A]),
    (Some(Opcode::POP), [OperandSpec::HL, OperandSpec::Nothing]),
    (Some(Opcode::LDH), [OperandSpec::DerefHighC, OperandSpec::A]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::PUSH), [OperandSpec::HL, OperandSpec::Nothing]),
    (Some(Opcode::AND), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x20), OperandSpec::Nothing]),
    // 0xe8
    (Some(Opcode::ADD), [OperandSpec::SP, OperandSpec::I8]),
    (Some(Opcode::JP), [OperandSpec::HL, OperandSpec::Nothing]),
    (Some(Opcode::LD), [OperandSpec::A16, OperandSpec::A]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::XOR), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x28), OperandSpec::Nothing]),
    // 0xf0
    (Some(Opcode::LDH), [OperandSpec::A, OperandSpec::DerefHighD8]),
    (Some(Opcode::POP), [OperandSpec::AF, OperandSpec::Nothing]),
    (Some(Opcode::LDH), [OperandSpec::A, OperandSpec::DerefHighC]),
    (Some(Opcode::DI), [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::PUSH), [OperandSpec::AF, OperandSpec::Nothing]),
    (Some(Opcode::OR), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x30), OperandSpec::Nothing]),
    // 0xf8
    (Some(Opcode::LD), [OperandSpec::HL, OperandSpec::SPWithOffset]),
    (Some(Opcode::LD), [OperandSpec::SP, OperandSpec::HL]),
    (Some(Opcode::LD), [OperandSpec::A, OperandSpec::A16]),
    (Some(Opcode::EI), [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (None, [OperandSpec::Nothing, OperandSpec::Nothing]),
    (Some(Opcode::CP), [OperandSpec::D8, OperandSpec::Nothing]),
    (Some(Opcode::RST), [OperandSpec::Imm(0x38), OperandSpec::Nothing]),
];

impl Decoder<Instruction> for InstDecoder {
    type Error = DecodeError;

    fn decode_into<T: IntoIterator<Item=u8>>(&self, inst: &mut Instruction, bytes: T) -> Result<(), Self::Error> {
        let mut iter = bytes.into_iter();

        let opc: u8 = iter.next().ok_or(DecodeError::ExhaustedInput)?;
        inst.length = 1;

        let high = ((opc >> 3) & 0b111) as usize;
        let low = (opc & 0b111) as usize;

        // early part of the table is more varied, more special cases
        if opc < 0x40 {
            let (opcode, operands) = match low {
                0 => {
                    // special case on high
                    match high {
                        0 => { (Opcode::NOP, [OperandSpec::Nothing, OperandSpec::Nothing]) },
                        1 => { (Opcode::LD, [OperandSpec::A16, OperandSpec::SP]) },
                        2 => { (Opcode::STOP, [OperandSpec::Nothing, OperandSpec::Nothing]) },
                        3 => { (Opcode::JR, [OperandSpec::R8, OperandSpec::Nothing]) },
                        4 => { (Opcode::JR, [OperandSpec::CondNZ, OperandSpec::R8]) },
                        5 => { (Opcode::JR, [OperandSpec::CondZ, OperandSpec::R8]) },
                        6 => { (Opcode::JR, [OperandSpec::CondNC, OperandSpec::R8]) },
                        7 => { (Opcode::JR, [OperandSpec::CondC, OperandSpec::R8]) },
                        _ => { unreachable!("impossible bit pattern"); }
                    }
                }
                1 => {
                    // also special case, but fewer
                    if high & 1 == 0 {
                        let opcode = Opcode::LD;
                        let ops = [[
                            OperandSpec::BC,
                            OperandSpec::DE,
                            OperandSpec::HL,
                            OperandSpec::SP,
                        ][high as usize >> 1], OperandSpec::D16];
                        (opcode, ops)
                    } else {
                        let opcode = Opcode::ADD;
                        let ops = [OperandSpec::HL, [
                            OperandSpec::BC,
                            OperandSpec::DE,
                            OperandSpec::HL,
                            OperandSpec::SP,
                        ][high as usize >> 1]];
                        (opcode, ops)
                    }
                }
                2 => {
                    let op = Opcode::LD;
                    if high & 1 == 0 {
                        let op0 = [
                            OperandSpec::DerefBC,
                            OperandSpec::DerefDE,
                            OperandSpec::DerefIncHL,
                            OperandSpec::DerefDecHL,
                        ][high as usize >> 1];
                        (op, [op0, OperandSpec::A])
                    } else {
                        let op1 = [
                            OperandSpec::DerefBC,
                            OperandSpec::DerefDE,
                            OperandSpec::DerefIncHL,
                            OperandSpec::DerefDecHL,
                        ][high as usize >> 1];
                        (op, [OperandSpec::A, op1])
                    }
                }
                3 => {
                    if high & 1 == 0 {
                        let op0 = [
                            OperandSpec::BC,
                            OperandSpec::DE,
                            OperandSpec::HL,
                            OperandSpec::SP,
                        ][high as usize >> 1];
                        (Opcode::INC, [op0, OperandSpec::Nothing])
                    } else {
                        let op0 = [
                            OperandSpec::BC,
                            OperandSpec::DE,
                            OperandSpec::HL,
                            OperandSpec::SP,
                        ][high as usize >> 1];
                        (Opcode::DEC, [op0, OperandSpec::Nothing])
                    }
                }
                4 => {
                    let op = Opcode::INC;
                    let op0 = [
                        OperandSpec::B,
                        OperandSpec::C,
                        OperandSpec::D,
                        OperandSpec::E,
                        OperandSpec::H,
                        OperandSpec::L,
                        OperandSpec::DerefHL,
                        OperandSpec::A,
                    ][high as usize];
                    (op, [op0, OperandSpec::Nothing])
                }
                5 => {
                    let op = Opcode::DEC;
                    let op0 = [
                        OperandSpec::B,
                        OperandSpec::C,
                        OperandSpec::D,
                        OperandSpec::E,
                        OperandSpec::H,
                        OperandSpec::L,
                        OperandSpec::DerefHL,
                        OperandSpec::A,
                    ][high as usize];
                    (op, [op0, OperandSpec::Nothing])
                }
                6 => {
                    let op = Opcode::LD;
                    let op0 = [
                        OperandSpec::B,
                        OperandSpec::C,
                        OperandSpec::D,
                        OperandSpec::E,
                        OperandSpec::H,
                        OperandSpec::L,
                        OperandSpec::DerefHL,
                        OperandSpec::A,
                    ][high as usize];
                    (op, [op0, OperandSpec::D8])
                }
                7 => {
                    // special cases here too
                    let op = [
                        Opcode::RLCA,
                        Opcode::RRCA,
                        Opcode::RLA,
                        Opcode::RRA,
                        Opcode::DAA,
                        Opcode::CPL,
                        Opcode::SCF,
                        Opcode::CCF,
                    ][high as usize];
                    (op, [OperandSpec::Nothing, OperandSpec::Nothing])
                }
                _ => {
                    unreachable!("impossible bit pattern");
                }
            };
            inst.opcode = opcode;
            interpret_operands(iter, inst, operands)?;
            return Ok(());
        } else if opc < 0x80 {
            if opc == 0x76 {
                inst.opcode = Opcode::HALT;
                inst.operands = [Operand::Nothing, Operand::Nothing];
                return Ok(());
            } else {
                inst.opcode = Opcode::LD;
                interpret_operands(iter, inst, [OPMAP[high], OPMAP[low]])?;
                return Ok(());
            }
        } else if opc < 0xc0 {
            const OPCODES: [Opcode; 8] = [
                Opcode::ADD,
                Opcode::ADC,
                Opcode::SUB,
                Opcode::SBC,
                Opcode::AND,
                Opcode::XOR,
                Opcode::OR,
                Opcode::CP,
            ];
            inst.opcode = OPCODES[high];
            let operands = [OPMAP[low], OperandSpec::Nothing];
            interpret_operands(iter, inst, operands)?;
            return Ok(());
        } else {
            if opc == 0xcb {
                // sm83 special CB-prefixed instructions
                let opc: u8 = iter.next().ok_or(DecodeError::ExhaustedInput)?;
                inst.length += 1;
                if opc < 0x40 {
                    let high = (opc >> 3) & 0b111;
                    let low = opc & 0b111;
                    const LOW_OP: [OperandSpec; 8] = [
                        OperandSpec::B,
                        OperandSpec::C,
                        OperandSpec::D,
                        OperandSpec::E,
                        OperandSpec::H,
                        OperandSpec::L,
                        OperandSpec::DerefHL,
                        OperandSpec::A,
                    ];
                    let operands = [LOW_OP[low as usize], OperandSpec::Nothing];
                    const OPCODES: [Opcode; 8] = [
                        Opcode::RLC,
                        Opcode::RRC,
                        Opcode::RL,
                        Opcode::RR,
                        Opcode::SLA,
                        Opcode::SRA,
                        Opcode::SWAP,
                        Opcode::SRL,
                    ];
                    inst.opcode = OPCODES[high as usize];
                    interpret_operands(iter, inst, operands)?;
                    return Ok(());
                } else {
                    let bit = (opc >> 3) & 0b111;
                    let low = opc & 0b111;
                    const LOW_OP: [OperandSpec; 8] = [
                        OperandSpec::B,
                        OperandSpec::C,
                        OperandSpec::D,
                        OperandSpec::E,
                        OperandSpec::H,
                        OperandSpec::L,
                        OperandSpec::DerefHL,
                        OperandSpec::A,
                    ];
                    let operands = [OperandSpec::Bit(bit), LOW_OP[low as usize]];
                    const OPCODES: [Opcode; 3] = [
                        Opcode::BIT,
                        Opcode::RES,
                        Opcode::SET,
                    ];
                    inst.opcode = OPCODES[(opc >> 6) as usize - 1];
                    interpret_operands(iter, inst, operands)?;
                    return Ok(());
                }
            } else {
                // there is no special thing we can do here. do a table lookup.
                let (maybe_opcode, operands) = UPPER_INSTRUCTIONS[opc as usize - 0xc0];
                if let Some(opcode) = maybe_opcode {
                    inst.opcode = opcode;
                    interpret_operands(iter, inst, operands)?;
                    return Ok(());
                } else {
                    return Err(DecodeError::InvalidOpcode);
                }
            }
        }
    }
}

fn interpret_operands<I: Iterator<Item=u8>>(mut iter: I, inst: &mut Instruction, operands: [OperandSpec; 2]) -> Result<(), DecodeError> {
    inst.operands[0] = interpret_operand(&mut iter, inst, operands[0])?;
    inst.operands[1] = interpret_operand(&mut iter, inst, operands[1])?;
    Ok(())
}

fn interpret_operand<I: Iterator<Item=u8>>(iter: &mut I, inst: &mut Instruction, operand: OperandSpec) -> Result<Operand, DecodeError> {
    let operand = match operand {
        OperandSpec::A => Operand::A,
        OperandSpec::B => Operand::B,
        OperandSpec::C => Operand::C,
        OperandSpec::D => Operand::D,
        OperandSpec::E => Operand::E,
        OperandSpec::H => Operand::H,
        OperandSpec::L => Operand::L,
        OperandSpec::AF => Operand::AF,
        OperandSpec::BC => Operand::BC,
        OperandSpec::DE => Operand::DE,
        OperandSpec::HL => Operand::HL,
        OperandSpec::SP => Operand::SP,
        OperandSpec::DerefHL => Operand::DerefHL,
        OperandSpec::DerefBC => Operand::DerefBC,
        OperandSpec::DerefDE => Operand::DerefDE,
        OperandSpec::DerefDecHL => Operand::DerefDecHL,
        OperandSpec::DerefIncHL => Operand::DerefIncHL,
        OperandSpec::D8 => {
            let imm = iter.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            Operand::D8(imm)
        }
        OperandSpec::DerefHighC => {
            Operand::DerefHighC
        }
        OperandSpec::DerefHighD8 => {
            let imm = iter.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            Operand::DerefHighD8(imm)
        }
        OperandSpec::R8 => {
            let imm = iter.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            Operand::R8(imm as i8)
        }
        OperandSpec::I8 => {
            let imm = iter.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            Operand::I8(imm as i8)
        }
        OperandSpec::A16 => {
            let imm =
                 (iter.next().ok_or(DecodeError::ExhaustedInput)? as u16) |
                ((iter.next().ok_or(DecodeError::ExhaustedInput)? as u16) << 8);
            inst.length += 2;
            Operand::A16(imm as u16)
        }
        OperandSpec::D16 => {
            let imm =
                 (iter.next().ok_or(DecodeError::ExhaustedInput)? as u16) |
                ((iter.next().ok_or(DecodeError::ExhaustedInput)? as u16) << 8);
            inst.length += 2;
            Operand::D16(imm as u16)
        }
        OperandSpec::SPWithOffset => {
            let imm = iter.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            Operand::SPWithOffset(imm as i8)
        }

        OperandSpec::Imm(u) => {
            Operand::D8(u)
        }
        OperandSpec::Bit(u) => {
            Operand::Bit(u)
        }

        OperandSpec::CondC => Operand::CondC,
        OperandSpec::CondNC => Operand::CondNC,
        OperandSpec::CondZ => Operand::CondZ,
        OperandSpec::CondNZ => Operand::CondNZ,

        OperandSpec::Nothing => Operand::Nothing,
    };
    Ok(operand)
}
