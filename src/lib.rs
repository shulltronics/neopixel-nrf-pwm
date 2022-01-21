#![no_std]

use smart_leds::{SmartLedsWrite, RGB8};
use nrf52832_hal::{
    pwm::{Pwm, Instance, LoadMode, StepMode, Seq, PwmSeq, PwmEvent},
    time::Hertz,
};

// A type definition for the DMA Buffer
pub const NUM_PIXELS: usize   =  3;
const NEOPIXEL_BIT_LEN: usize = 24;
type SequenceBuffer = &'static mut [u16; NEOPIXEL_BIT_LEN*NUM_PIXELS+1];
// PWM constants for sending zeros and ones
const PWM_FREQ:     u32 = 500_000;
const PWM_MAX_DC:   u16 = 0x20;
const PWM_ZERO_DC:  u16 = 0x08;
const PWM_ONE_DC:   u16 = 0x18;

// A NeoPixel driver using the nrf52 PWM peripheral and a static mutable buffer
pub struct NeoPixel<T: Instance> {
    pwm: Option<PwmSeq<T, SequenceBuffer, SequenceBuffer>>,
}

impl<T> NeoPixel<T>
where
    T: Instance + core::fmt::Debug,
{
    // Create a new NeoPixel object from an nrf52 PWM object
    pub fn new(_pwm: Pwm<T>) -> NeoPixel<T> {//, dma_buf_0: &'static mut [u16], dma_buf_1: &'static mut [u16]) -> NeoPixel<T> {

        static mut dma_buffer_0: [u16; NEOPIXEL_BIT_LEN*NUM_PIXELS+1] = [0; NEOPIXEL_BIT_LEN*NUM_PIXELS+1];
        static mut dma_buffer_1: [u16; NEOPIXEL_BIT_LEN*NUM_PIXELS+1] = [0; NEOPIXEL_BIT_LEN*NUM_PIXELS+1];
        // Configure pwm to play sequence 0 as one shot
        _pwm.set_max_duty(PWM_MAX_DC)
            .set_period(Hertz(PWM_FREQ))
            .set_load_mode(LoadMode::Common)
            .set_step_mode(StepMode::Auto)
            .set_seq_refresh(Seq::Seq0, 0)
            .set_seq_end_delay(Seq::Seq0, 26)   // This will yield the 50us hold low we need
            .one_shot();

        Self {
            pwm: unsafe {_pwm.load(Some(&mut dma_buffer_0), Some(&mut dma_buffer_1), false).ok()},
        }
    }

}

impl<T> SmartLedsWrite for NeoPixel<T>
where
    T: Instance + core::fmt::Debug
{
    type Error = ();
    type Color = RGB8;

    fn write<U, I>(&mut self, pixels: U) -> Result<(), Self::Error>
    where
        U: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        let (b0, b1, pwm) = self.pwm.take().unwrap().split();
        let buf0 = b0.unwrap();
        let buf1 = b1.unwrap();
        // iterate through the pixels
        for (pixel_index, rgb) in pixels.enumerate() {
            if pixel_index >= NUM_PIXELS {
                break;
            }
            // extract the bits of color information
            for (color_num, color) in rgb.into().iter().enumerate() {
                for bit in 0..8 {
                    let dc = match (1 << bit) & color {
                        0 => PWM_ZERO_DC,
                        _ => PWM_ONE_DC,
                    };
                    buf0[pixel_index*NEOPIXEL_BIT_LEN + color_num*8 + bit] = 0x8000 | dc;
                    buf1[pixel_index*NEOPIXEL_BIT_LEN + color_num*8 + bit] = 0x8000 | dc;
                }
            }
        }
        buf0[buf0.len()-1] = 0x8000;
        buf1[buf1.len()-1] = 0x8000;
        self.pwm = pwm.load(Some(buf0), Some(buf1), false).ok();
        self.pwm.as_ref().unwrap().start_seq(Seq::Seq0);
        Ok(())
    }

}
