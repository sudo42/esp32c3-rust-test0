
use crate::colour::*;

// APB clock runs at 80MHz by default?
// -> no? SYS_CLK seems to be set to XTAL_CLK by default, with SYSTEM_PRE_DIV_CNT = 1
//    ... causing APB_CLK to be set to the same frequency
//    ... XTAL_CLK is 40MHz, CPU_CLK = XTAL_CLK/(SYSTEM_PRE_DIV_CNT + 1) -> 20MHz -> 0.05µs
// -> actually, yes! (because of the bootloader setup, I guess?)
//   -> we set the rmt divider to 3 (instead of default=0), so the RMT still ticks at 20MHz
//
// Neopixel Data Timings (LS: Low Speed ("400k"), HS: High Speed ("800k"))
//          LSµ  HSµ    ×0.05µ (HS)
//      T0H 0.5 0.25    5
//      T0L 2.0 1.0     20
//      T1H 1.2 0.6     12
//      T1L 1.3 0.65    13
//      RES >50 >50     >1000
//
// pulse code frame = 2 segments: u32 -> twice: 1b level + 15b duration
// -> Neopixel 1: 1,12, 0,13  -> 0x800C000D
//    Neopixel 0: 1,5,  0,20  -> 0x80050014
//    Neopixel R: 0,512,0,512 -> 0x02000200 // TODO use a fun number instead ^^

//*
pub const NP_PULSE_1: u32 = 0x800C000D;
pub const NP_PULSE_0: u32 = 0x80050014;
//pub const NP_PULSE_R: u32 = 0x02000200;
pub const NP_PULSE_R: u32 = 0x00200020;
/*/
pub const NP_PULSE_1: u32 = 0x80300034;
pub const NP_PULSE_0: u32 = 0x80140050;
pub const NP_PULSE_R: u32 = 0x08000800;
// */
pub const NP_PULSE_01 : [u32;2] = [NP_PULSE_0, NP_PULSE_1];

pub fn byte_to_pulse(byte: &u8, pulse_seq: &mut [u32;8]) {
    for i in 0..8 {
        //pulse_seq[i] = if (byte & (0x80 >> i)) == 0 { NP_PULSE_0 } else { NP_PULSE_1 };
        pulse_seq[i] = NP_PULSE_01[((byte & (0x80 >> i)) != 0) as usize];
    }
}

pub fn colour_to_pulse(colour: &RGB8, pulse_seq: &mut [u32;24]) {
    // order is GRB
    let [ ref mut seq_g @ ..,   _,_,_,_,_,_,_,_,    _,_,_,_,_,_,_,_     ] = pulse_seq;
    let [ _,_,_,_,_,_,_,_,      ref mut seq_r @ .., _,_,_,_,_,_,_,_     ] = pulse_seq;
    let [ _,_,_,_,_,_,_,_,      _,_,_,_,_,_,_,_,    ref mut seq_b @ ..  ] = pulse_seq;
    byte_to_pulse(&colour.r, seq_r);
    byte_to_pulse(&colour.g, seq_g);
    byte_to_pulse(&colour.b, seq_b);
}

/*
#[inline]
fn next_colour_to_pulse<const N: usize>(colours: &[RGB8;N], pulse_seq: &mut [u32;N*24]) {
    match colours {
        [] => {},
        [ref colour, ref next_colours @ ..] => {
            colour_to_pulse(&colour, &pulse_seq)
            let (this_seq, next_seq) = pulse_seq.split_array_mut::<
        }
    }
}
*/

#[inline]
pub fn colours_to_pulse<const N: usize>(colours: &[RGB8;N], pulse_seq : &mut [u32;N*24 + 2]) {
    /*
    for i in 0..N {
        let start : usize = 24*i;
        let seq_view : &mut [u32;24] = pulse_seq.split_array_mut::<start>().0.rsplit_array_mut::<24>();
        colour_to_pulse(&colours[i], &mut seq_view);
    }
    */
    let mut seq_ptr = pulse_seq.as_mut_ptr();
    for ref colour in colours {
        // NOTE(UNSAFE): const generic expressions are not advanced enough for split, so we could either use the iterator's next_chunk, slices and their experimental array conversions, or this ...
        let seq_view = unsafe {
            let view = &mut *(seq_ptr as *mut [u32;24]);
            seq_ptr = seq_ptr.add(24);
            view
        };
        colour_to_pulse(colour, seq_view);
    }
    pulse_seq[N*24] = NP_PULSE_R; // Neopixel reset
    pulse_seq[N*24+1] = 0; // RMT EOT
}

#[inline]
pub fn pulse_from_colours<const N: usize>(colours: &[RGB8;N]) -> [u32;N*24 + 2] {
    //const SEQ_SIZE : usize = N*3 + 1;
    //let pulse_seq : [u32;SEQ_SIZE] = [0;SEQ_SIZE];
    let mut pulse_seq : [u32; N*24 + 2] = [0; N*24 + 2];
    colours_to_pulse(colours, &mut pulse_seq);
    pulse_seq
}
