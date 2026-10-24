// The generated mirror modules, compressed into the binary (mirror.bin,
// written by tools/gen_mirror.py): u32 count, then per module u16 name
// length, name, u32 compressed length, u32 source length, zlib stream.
// One module is inflated when python imports it.
use std::io::Read;

static ARCHIVE: &[u8] = include_bytes!("mirror.bin");

fn u16_at(b: &[u8], i: usize) -> usize {
    u16::from_le_bytes([b[i], b[i + 1]]) as usize
}

fn u32_at(b: &[u8], i: usize) -> usize {
    u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]) as usize
}

pub fn source(module: &str) -> Option<String> {
    let count = u32_at(ARCHIVE, 0);
    let mut i = 4;
    for _ in 0..count {
        let nlen = u16_at(ARCHIVE, i);
        let name = &ARCHIVE[i + 2..i + 2 + nlen];
        i += 2 + nlen;
        let (zlen, slen) = (u32_at(ARCHIVE, i), u32_at(ARCHIVE, i + 4));
        i += 8;
        if name == module.as_bytes() {
            let mut out = String::with_capacity(slen);
            flate2::read::ZlibDecoder::new(&ARCHIVE[i..i + zlen]).read_to_string(&mut out).ok()?;
            return Some(out);
        }
        i += zlen;
    }
    None
}
