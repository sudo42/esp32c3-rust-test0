#![no_std]
#![no_main]

#![feature(ptr_to_from_bits,pointer_byte_offsets)]
#![feature(generic_const_exprs)]
#![feature(split_array)]
#![feature(strict_provenance)]

extern crate alloc;

use core::{mem, mem::MaybeUninit, ops::Deref};
//use riscv_rt::entry;

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;

//use hal::prelude::*;
use hal::{prelude::*, clock::ClockControl, peripherals::{Peripherals,SYSTEM}, timer::TimerGroup, Delay, IO, Rtc, rmt, rmt::{TxChannelCreator, TxChannel}, systimer::SystemTimer, Rng, gpio};
use esp_backtrace as _;
use esp_println::{print, println};
use log::{error, warn, info, debug, trace};

use esp_wifi::{initialize, EspWifiInitFor};

mod colour;
use colour::*;
mod neopixel_rmt;
mod onboard_led;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

//use core::panic::PanicInfo;
//use esp32c_rt::entry;

/*
#[allow(unused_imports)]
use panic_halt;
*/

/*
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
*/

enum BoardType {
    QtPyC3,
    Esp32C3DevKitC02,
}

//const BOARD_TYPE: BoardType = BoardType::QtPyC3;

#[cfg(feature = "board-QtPyC3")]
  const BOARD_TYPE: BoardType = BoardType::QtPyC3;
#[cfg(feature = "board-Esp32C3DevKitC02")]
  const BOARD_TYPE: BoardType = BoardType::Esp32C3DevKitC02;

/*
fn assign_led_rmt_pin<LedPin,RmtChannel> (pin: &mut LedPin, channel: pulse_control::OutputChannel<CC>) where LedPin: hal::gpio::OutputPin, RmtChannel: pulse_control::OutputChannel {
    //let pin_configured = get_pin_for_board_led().into_push_pull_output();
    channel.assign_pin(pin.into_push_pull_output())
}
*/

#[entry]
fn main() -> ! {
    println!("configuring logger ...\n");
    esp_println::logger::init_logger(log::LevelFilter::Trace);

    {
        debug!("reading registers via HAL/PAC ...");
        let SYSTEM_REGS = unsafe { &*SYSTEM::PTR };
        let SYSTEM_SYSCLK_CONF = SYSTEM_REGS.sysclk_conf.read().bits();
        println!("  *SYSTEM_SYSCLK_CONF_REG = {:0>32b}", SYSTEM_SYSCLK_CONF);
        println!("  -> SYSTEM_SOC_CLK_SEL = {}", (SYSTEM_SYSCLK_CONF >> 10) & 0b11);
    }

    // now add a heap ..
    debug!("initialising heap ...\n");
    init_heap();

    // fetch peripherals
    debug!("fetching peripheral helpers ...\n");
    let peripherals = Peripherals::take();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut system = peripherals.SYSTEM.split();
    let mut peripheral_clock_control = system.peripheral_clock_control;
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // pick pins based on the board in use ...
    #[cfg(feature = "board-QtPyC3")]            let pin_neopixel = io.pins.gpio2;
    #[cfg(feature = "board-Esp32C3DevKitC02")]  let pin_neopixel = io.pins.gpio8;

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, &mut peripheral_clock_control);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, &mut peripheral_clock_control);
    let mut wdt1 = timer_group1.wdt;

    debug!("disbling watch dog timers ...\n");
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    debug!("configuring RMT for Neopixel ...");
    let rmtTxChannelConfig = rmt::TxChannelConfig {
        clk_divider: 3,
        idle_output_level: false,
        idle_output: false,
        carrier_modulation: false,
        carrier_high: 1,
        carrier_low: 1,
        carrier_level: false,
    };
    let rmt = rmt::Rmt::new(peripherals.RMT, 80u32.MHz(), &mut peripheral_clock_control, &clocks).unwrap();
    let mut led_pulse_channel = rmt
      .channel0
      .configure(
          pin_neopixel.into_push_pull_output(),
          rmtTxChannelConfig,
      )
      .unwrap();

    let pulse_sequences = [
        //neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 0,   b: 0 }]),
        /*
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 4,  g: 0,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 3,  g: 2,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 3,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 2,   b: 2 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 0,   b: 4 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 3,  g: 0,   b: 2 }]),
        */
        /*
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 4,  g: 0,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 4,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 0,   b: 4 }]),
        */
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 0,  g: 0,   b: 0 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 2,  g: 2,   b: 16 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 16, g: 2,   b: 2 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 8,  g: 8,   b: 8 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 16, g: 2,   b: 2 }]),
        neopixel_rmt::pulse_from_colours(&[RGB8{ r: 2,  g: 2,   b: 16 }]),
    ];

    // */

    info!("init done!\n");

    debug!("clock info ...");
    debug!("  CPU:  {}", clocks.cpu_clock);
    debug!("  APB:  {}", clocks.apb_clock);
    debug!("  XTAL: {}", clocks.xtal_clock);
    //debug!("  I2C:  {}", clocks.i2c_clock);

    debug!("reading registers via HAL/PAC ...");
    let SYSTEM_REGS = unsafe { &*SYSTEM::PTR };
    let SYSTEM_SYSCLK_CONF = SYSTEM_REGS.sysclk_conf.read().bits();
    println!("  *SYSTEM_SYSCLK_CONF_REG = {:0>32b}", SYSTEM_SYSCLK_CONF);
    println!("  -> SYSTEM_SOC_CLK_SEL = {}", (SYSTEM_SYSCLK_CONF >> 10) & 0b11);

    read_regs_raw();

    info!("starting loop ...\n");

    // init helpers and go to loop!
    let mut delay = Delay::new(&clocks);
    let mut n: usize = 0;

    loop {
        debug!("loop pre-delay ... ");
        delay.delay_ms(2222u32);
        debug!("n={}\n", n);
        n += 1;


        let sequence = &pulse_sequences[n % 6];
        led_pulse_channel = match led_pulse_channel.transmit(sequence).wait() {
            Ok(channel) => { debug!("sending complete"); channel },
            Err((trans_err, channel)) => {
              error!("error sending pulse sequnce to LED: {:?}", trans_err);
              channel
            }
        };
        // */
    }
}

fn read_regs_raw() {
    debug!("reading raw registers ...");
    unsafe {
        const SYS_REG_ADDR: usize = 0x600C_0000;
        const CPU_PER_CONF_REG_OFFSET: usize = 0x0008;
        const SYSCLK_CONF_REG_OFFSET: usize = 0x0058;
        const DATE_REG: usize = 0x0FFC;

        let system_cpu_per_conf_reg: *const u32 = core::ptr::from_exposed_addr(SYS_REG_ADDR + CPU_PER_CONF_REG_OFFSET);
        let system_cpu_per_conf = system_cpu_per_conf_reg.read_volatile();
        println!("  *SYSTEM_CPU_PER_CONF_REG        = 0b{:0>32b}", system_cpu_per_conf);
        println!("    SYSTEM_CPU_WAITI_DELAY_NUM    = 0b{:>24}{:0>4b}", "", (system_cpu_per_conf >> 4) & 0b1111);
        println!("    SYSTEM_CPU_WAIT_MODE_FORCE_ON = 0b{:>28}{:0>1b}", "", (system_cpu_per_conf >> 3) & 0b1);
        println!("    SYSTEM_PLL_FREQ_SEL           = 0b{:>29}{:0>1b}", "", (system_cpu_per_conf >> 2) & 0b1);
        println!("    SYSTEM_CPUPERIOD_SEL          = 0b{:>30}{:0>2b}", "", (system_cpu_per_conf >> 0) & 0b11);

        let system_sysclk_conf_reg: *const u32 = core::ptr::from_exposed_addr(SYS_REG_ADDR + SYSCLK_CONF_REG_OFFSET);
        let system_sysclk_conf = system_sysclk_conf_reg.read_volatile();
        println!("  *SYSTEM_SYSCLK_CONF_REG = 0b{:0>32b}", system_sysclk_conf);
        println!("    SYSTEM_CLK_XTAL_FREQ  = 0b{:>13}{:0>7b}", "", (system_sysclk_conf >> 12) & 0b111_1111);
        println!("    SYSTEM_CLK_XTAL_FREQ  =   {:>13}{}", "", (system_sysclk_conf >> 12) & 0b111_1111);
        println!("    SYSTEM_SOC_CLK_SEL    = 0b{:>20}{:0>2b}", "", (system_sysclk_conf >> 10) & 0b11);
        println!("    SYSTEM_PRE_DIV_CNT    = 0b{:>22}{:0>10b}", "", (system_sysclk_conf >> 0) & 0b11_1111_1111);

        let system_date_reg: *const u32 = core::ptr::from_exposed_addr(SYS_REG_ADDR + DATE_REG);
        let system_date = system_date_reg.read_volatile();
        println!("  *SYSTEM_DATE_REG = 0b{:0>32b}", system_date);
        println!("    SYSTEM_DATE    = 0b{:>4}{:0>28b}", "", (system_date >> 0) & 0x0FFFFFFF);
        println!("                   = 0x{:>4}{:0>8x}", "", (system_date >> 0) & 0x0FFFFFFF);
        println!("                   =   {:>4}{}", "", (system_date >> 0) & 0x0FFFFFFF);
    }
}
