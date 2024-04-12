#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    embassy::{self},
    i2s::{asynch::*, DataFormat, I2s, Standard},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    IO,
};
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (_, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(0, 4092 * 4);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data32Channel24,
        44100u32.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(io.pins.gpio2)
        .with_din(io.pins.gpio3)
        .with_ws(io.pins.gpio4)
        .build();

    let buffer = rx_buffer;
    println!("Start");

    let mut data = [0u8; 5000];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let _avail = transaction.available().await;
        //println!("available {}", avail);
        let count = transaction.pop(&mut data).await.unwrap();
        //println!("got {} bytes}", count);

        // the DMA reads in 4 byte chunks, so 32 bits
        // however, the mic only is configured for 24 bits
        // so every 4th byte is empty
        for i in 0..count/4 {
            // we only care about every second 4 bytes, since the data is 2 channels, but we only have 1 channel
            if i % 2 == 0 {

                // the data is in two's complement, so we need to convert it to a signed integer
                let sample_u32 = (data[i*4+2] as u32) << 16 | (data[i*4+1] as u32) << 8 | data[i*4+0] as u32;
                let mut sample_i32 = sample_u32 as i32;
                if sample_i32 & (1 << 23) != 0 {
                    sample_i32 -= 2 * (1 << 23);
                }

                println!("{}", sample_i32);
                // println!("{:02X} {:02X} {:02X} {:02X}", &data[i*4], &data[i*4+1], &data[i*4+2], &data[i*4+3]);
            }
        }
    }
}