/*
struct RmtSequence
    <const LED_COUNT: usize, const FRAME_COUNT: usize>
    (&[[u32; LED_COUNT * 24 + 2]; FRAME_COUNT]);
    */
struct RmtSequence
    <'a, const LED_COUNT: usize, const FRAME_COUNT: usize>
    (&'a [[u32; LED_COUNT * 24 + 2]; FRAME_COUNT])
    where [(); LED_COUNT * 24 + 2]:;

enum Pattern {
    Black,
    RGB,
    RGB_6,
    Trans,
}

/*
// TODO: macro-ify colour pattern gen to do it at compile time!
mod patterns {
    use crate::colour::*;
    use crate::neopixel_rmt;

    static TRANS : &[[u32; 1 * 24 + 2]; 6] = &[
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 0,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 2,  g: 2,   b: 16 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 16, g: 2,   b: 2 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 8,  g: 8,   b: 8 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 16, g: 2,   b: 2 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 2,  g: 2,   b: 16 }]),
    ];
}
*/


