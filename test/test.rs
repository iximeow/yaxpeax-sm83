use std::fmt::Write;

use yaxpeax_arch::{AddressBase, Decoder, LengthedInstruction, U8Reader};
use yaxpeax_sm83::InstDecoder;

fn test_invalid(data: &[u8]) {
    test_invalid_under(&InstDecoder::default(), data);
}

fn test_invalid_under(decoder: &InstDecoder, data: &[u8]) {
    let mut reader = U8Reader::new(data);
    if let Ok(inst) = decoder.decode(&mut reader) {
        panic!("decoded {:?} from {:02x?}", inst.opcode(), data);
    } else {
        // this is fine
    }
}

fn test_display(data: &[u8], expected: &'static str) {
    test_display_under(&InstDecoder::default(), data, expected);
}

fn test_display_under(decoder: &InstDecoder, data: &[u8], expected: &'static str) {
    let mut hex = String::new();
    for b in data {
        write!(hex, "{:02x}", b).unwrap();
    }
    let mut reader = U8Reader::new(data);
    match decoder.decode(&mut reader) {
        Ok(instr) => {
            let text = format!("{}", instr);
            assert!(
                text == expected,
                "display error for {}:\n  decoded: {:?}\n displayed: {}\n expected: {}\n",
                hex,
                instr,
                text,
                expected
            );
            // while we're at it, test that the instruction is as long, and no longer, than its
            // input
            assert_eq!((0u16.wrapping_offset(instr.len()).to_linear()) as usize, data.len(), "instruction length is incorrect, wanted instruction {}", expected);
        },
        Err(e) => {
            assert!(false, "decode error ({}) for {}:\n  expected: {}\n", e, hex, expected);
        }
    }
}

#[test]
fn test_invalid_ops() {
    test_invalid(&[0xd3]);
    test_invalid(&[0xdb]);
    test_invalid(&[0xdd]);
    test_invalid(&[0xe3]);
    test_invalid(&[0xe4]);
    test_invalid(&[0xeb]);
    test_invalid(&[0xec]);
    test_invalid(&[0xed]);
    test_invalid(&[0xf4]);
    test_invalid(&[0xfc]);
    test_invalid(&[0xfd]);
}

#[test]
fn test_ld_decode() {
    test_display(&[0x6f], "ld l, a");
    test_display(&[0x6e], "ld l, [hl]");
    test_display(&[0x32], "ld [hl-], a");
    test_display(&[0x3a], "ld a, [hl-]");
    test_display(&[0x73], "ld [hl], e");
    test_display(&[0x74], "ld [hl], h");
    test_display(&[0x67], "ld h, a");
    test_display(&[0x79], "ld a, c");
    test_display(&[0x52], "ld d, d");
    test_display(&[0x64], "ld h, h");
    test_display(&[0x0e, 0x75], "ld c, $75");
    test_display(&[0x3e, 0x75], "ld a, $75");
    test_display(&[0x01, 0x99, 0x88], "ld bc, $8899");
    test_display(&[0x11, 0x99, 0x88], "ld de, $8899");
    test_display(&[0x21, 0x99, 0x88], "ld hl, $8899");
    test_display(&[0x31, 0x99, 0x88], "ld sp, $8899");
    test_display(&[0xea, 0x34, 0x12], "ld [$1234], a");
}

#[test]
fn test_push_pop() {
    test_display(&[0xf1], "pop af");
    test_display(&[0xe1], "pop hl");
    test_display(&[0xe5], "push hl");
}

#[test]
fn test_control_flow() {
    test_display(&[0xc3, 0x10, 0x20], "jp $2010");       // should this be relative?
    test_display(&[0xda, 0x10, 0x20], "jp C, $2010");    // should this be relative?
    test_display(&[0x20, 0x33], "jr nZ, $+$33");
    test_display(&[0xd9], "reti");
    test_display(&[0xc9], "ret");
    test_display(&[0x00], "nop");
    test_display(&[0xcd, 0x44, 0x66], "call $6644");       // not relative, right
    test_display(&[0xfe, 0x10], "cp $10");
    test_display(&[0xb8], "cp b");
}

#[test]
fn test_arithmetic() {
    test_display(&[0xce, 0x40], "adc $40");
    test_display(&[0x19], "add hl, de");
    test_display(&[0x81], "add c");
    test_display(&[0x83], "add e");
    test_display(&[0x88], "adc b");
    test_display(&[0x89], "adc c");
    test_display(&[0x99], "sbc c");
    test_display(&[0x9d], "sbc l");
    test_display(&[0x91], "sub c");
    test_display(&[0x0c], "inc c");
    test_display(&[0x0d], "dec c");
    test_display(&[0x2d], "dec l");
}

#[test]
fn test_bitwise() {
    test_display(&[0xb5], "or l");
    test_display(&[0xf6, 0x10], "or $10");
    test_display(&[0xcb, 0x71], "bit 6, c");
    test_display(&[0xaf], "xor a");
    test_display(&[0x2b], "dec hl");
    test_display(&[0x34], "inc [hl]");
}

#[test]
fn test_misc() {
    test_display(&[0x37], "scf");
    test_display(&[0x10], "stop");
}
