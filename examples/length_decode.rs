use yaxpeax_arch::U8Reader;
use yaxpeax_sm83::InstDecoder;

#[inline(never)]
#[no_mangle]
pub fn test_length_decode(decoder: &InstDecoder, data: &[u8], mut length: &mut u8) -> bool {
    let mut reader = U8Reader::new(data);
    yaxpeax_sm83::decode_inst(decoder, &mut length, &mut reader).is_ok()
}

