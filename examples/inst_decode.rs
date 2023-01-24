use yaxpeax_arch::U8Reader;
use yaxpeax_sm83::{InstDecoder, Instruction};

#[inline(never)]
#[no_mangle]
pub fn test_inst_decode(decoder: &InstDecoder, data: &[u8]) -> Option<Instruction> {
    let mut inst = Instruction::default();
    let mut reader = U8Reader::new(data);
    if yaxpeax_sm83::decode_inst(decoder, &mut inst, &mut reader).is_ok() {
        Some(inst)
    } else {
        None
    }
}
