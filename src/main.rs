#![no_main]
#![no_std]

extern crate stm32f3xx_hal as hal;
//extern crate panic_semihosting;
extern crate panic_halt;
extern crate cortex_m_semihosting;

use cortex_m_semihosting::hprintln;
//use cortex_m::iprintln;
use rtfm::{app, cyccnt::{Instant, U32Ext}};

use core::{f32::consts::PI, ptr};
use hal::{prelude::*, hal::digital::v2::OutputPin,
	usb::{Peripheral, UsbBus, UsbBusType},
	gpio::{Output, PushPull}, nb::block,
	i2c::I2c, spi::Spi, timer::Timer, stm32::ITM
	};
use aligned::Aligned;
use cast::{f32, i32};
use l3gd20::{Odr, L3gd20};
use lsm303dlhc::{AccelOdr, MagOdr, Lsm303dlhc};
use madgwick::{F32x3, Marg};
use byteorder::{ByteOrder, LE};


// Number of samples to use for gyroscope calibration
const NSAMPLES: i32 = 256;

// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The producer is explained
// in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
const M_BIAS_X: f32 = -183.;
const M_SCALE_X: f32 = 435.;

const M_BIAS_Y: f32 = -172.;
const M_SCALE_Y: f32 = 507.;

const M_BIAS_Z: f32 = -136.;
const M_SCALE_Z: f32 = 632.;

// Sensitivities of the accelerometer and gyroscope, respectively
const K_G: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_AR: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s

// Madgwick filter parameters
const SAMPLE_FREQ: u32 = 220;
const BETA: f32 = 1e-3;


#[app(device = stm32f3xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
	struct Resources {
		LSM303DLHC: lsm303dlhc::Lsm303dlhc<I2c<
			hal::stm32::I2C1, 
			(hal::gpio::gpiob::PB6<hal::gpio::AF4>,
			hal::gpio::gpiob::PB7<hal::gpio::AF4>)>>,
		L3GD20: l3gd20::L3gd20<hal::spi::Spi<
			hal::stm32::SPI1,
				(hal::gpio::gpioa::PA5<hal::gpio::AF5>,
				hal::gpio::gpioa::PA6<hal::gpio::AF5>,
				hal::gpio::gpioa::PA7<hal::gpio::AF5>)>, 
			hal::gpio::gpioe::PE3<Output<PushPull>>>,
		
		Marg: madgwick::Marg,

		ArBiasX: i16,
		ArBiasY: i16,
		ArBiasZ: i16,
	}

	#[init(spawn = [mems])]
	fn init(mut cx: init::Context) -> init::LateResources {
 		
// tim4 set up
		cx.device.RCC.apb1enr.modify(|_, w| w.tim4en().set_bit());
		cx.device.TIM4.cr1.modify(|_, w| {
			w.cen().set_bit() });
		cx.device.TIM4.psc.modify(|_, w| unsafe{
			w.psc().bits(2181) });		// prescailer set
		cx.device.TIM4.arr.modify(|_, w|  {
			w.arr().bits(100) });
		cx.device.TIM4.dier.modify(|_, w| {
			w.uie().set_bit() });
		
		
		let mut flash = cx.device.FLASH.constrain();
		let mut rcc = cx.device.RCC.constrain();

		let clocks = rcc
			.cfgr
			.sysclk(48.mhz())
			.pclk1(32.mhz())
			.freeze(&mut flash.acr);


		let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
		let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
		let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);

// spi for gyro
		let mut nss = gpioe.pe3
			.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
		
		let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
		let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
		let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

		let spi = Spi::spi1(
			cx.device.SPI1,
			(sck, miso, mosi),
			l3gd20::MODE,
			1.mhz(),
			clocks,
			&mut rcc.apb2,
		);

		let mut l3gd20 = L3gd20::new(spi, nss).unwrap();

		l3gd20.set_odr(Odr::Hz380).unwrap();


// i2c for accelrometer and magnet orienter

		let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
		let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

		let i2c = I2c::i2c1(cx.device.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

		let mut lsm303dlhc = Lsm303dlhc::new(i2c).unwrap();

		lsm303dlhc.accel_odr(AccelOdr::Hz400).unwrap();
		lsm303dlhc.mag_odr(MagOdr::Hz220).unwrap();

// calibration
		let mut timer = Timer::tim2(cx.device.TIM2, 380.hz(), clocks, &mut rcc.apb1);
		
		// gyroscope calibration
		let mut ar_bias_x = 0;
		let mut ar_bias_y = 0;
		let mut ar_bias_z = 0;

		for _ in 0..NSAMPLES {
			block!(timer.wait()).unwrap();
			
			let ar = l3gd20.all().unwrap().gyro;
			ar_bias_x += i32(ar.x);
			ar_bias_y += i32(ar.y);
			ar_bias_z += i32(ar.z);
		}
    	let ar_bias_x = (ar_bias_x / NSAMPLES) as i16;
    	let ar_bias_y = (ar_bias_y / NSAMPLES) as i16;
    	let ar_bias_z = (ar_bias_z / NSAMPLES) as i16;

		let mut marg = Marg::new(BETA, 1. / f32(SAMPLE_FREQ));
    	let mut timer = Timer::tim1(cx.device.TIM1, SAMPLE_FREQ.hz(), clocks, &mut rcc.apb2);
		let mut tx_buf: Aligned<u32, [u8; 18]> = Aligned([0;18]);
		/*
		// readeing loop should be moved to timed interupt
		loop{
			block!(timer.wait()).unwrap();

			let m = lsm303dlhc.mag().unwrap();
			let ar = l3gd20.all().unwrap().gyro;
			let g = lsm303dlhc.accel().unwrap();

			let m_x = (f32(m.x) - M_BIAS_X) / M_SCALE_X;
			let m_y = (f32(m.y) - M_BIAS_Y) / M_SCALE_Y;
			let m_z = (f32(m.z) - M_BIAS_Z) / M_SCALE_Z;

			// Fix magnetomerter sow it match the gyro axes
			let m = F32x3 {
				x: m_y,
				y: -m_x,
				z: m_z,
			};

			// gyro to mag format
			let ar_x = f32(ar.x - ar_bias_x) * K_AR;
			let ar_y = f32(ar.y - ar_bias_y) * K_AR;
			let ar_z = f32(ar.z - ar_bias_z) * K_AR;
			let ar = F32x3 {
				x:ar_x,
				y: ar_y,
				z: ar_z,
			};

			// Fix accelerometer sow it match the gyro axes
			let g_x = f32(g.x) * K_G;
			let g_y = f32(g.y) * K_G;
			let g_z = f32(g.z) * K_G;
			let g = F32x3 {
				x: g_y,
				y: -g_x,
				z: g_z,
			};
			

			// Run the filter
			let quat = marg.update(m, ar, g);

			// Serialize the quaterion
			let mut start = 0;
			let mut buf = [0; 16];
			LE::write_f32(&mut buf[start..start + 4], quat.0);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.1);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.2);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.3);

//			iprintln!(stim, "{:?}", buf);
		}
		*/
		cx.core.DCB.enable_trace();
		cx.core.DWT.enable_cycle_counter();
		hprintln!("i").unwrap();
		//cx.schedule.mems(Instant::now() + 48000000_u32.cycles()).unwrap();
		cx.spawn.mems().unwrap();
		init::LateResources {
			LSM303DLHC: lsm303dlhc,
			L3GD20: l3gd20,
			ArBiasX: ar_bias_x,
			ArBiasY: ar_bias_y,
			ArBiasZ: ar_bias_z,
			Marg: marg,
		}
	}

	#[task(schedule = [mems], resources = [LSM303DLHC, L3GD20, ArBiasX, ArBiasY, ArBiasZ, Marg])]
	fn mems(cx: mems::Context) {
			cx.schedule.mems(Instant::now() + (48000000_u32 / SAMPLE_FREQ).cycles()).unwrap();
			
			let m = cx.resources.LSM303DLHC.mag().unwrap();
			let ar = cx.resources.L3GD20.all().unwrap().gyro;
			let g = cx.resources.LSM303DLHC.accel().unwrap();

			let m_x = (f32(m.x) - M_BIAS_X) / M_SCALE_X;
			let m_y = (f32(m.y) - M_BIAS_Y) / M_SCALE_Y;
			let m_z = (f32(m.z) - M_BIAS_Z) / M_SCALE_Z;

			// Fix magnetomerter sow it match the gyro axes
			let m = F32x3 {
				x: m_y,
				y: -m_x,
				z: m_z,
			};

			// gyro to mag format
			let ar_x = f32(ar.x - *cx.resources.ArBiasX) * K_AR;
			let ar_y = f32(ar.y - *cx.resources.ArBiasY) * K_AR;
			let ar_z = f32(ar.z - *cx.resources.ArBiasZ) * K_AR;
			let ar = F32x3 {
				x:ar_x,
				y: ar_y,
				z: ar_z,
			};

			// Fix accelerometer sow it match the gyro axes
			let g_x = f32(g.x) * K_G;
			let g_y = f32(g.y) * K_G;
			let g_z = f32(g.z) * K_G;
			let g = F32x3 {
				x: g_y,
				y: -g_x,
				z: g_z,
			};
			

			// Run the filter
			let quat = cx.resources.Marg.update(m, ar, g);

			// Serialize the quaterion
			let mut start = 0;
			let mut buf = [0; 16];
			LE::write_f32(&mut buf[start..start + 4], quat.0);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.1);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.2);
			start += 4;
			LE::write_f32(&mut buf[start..start + 4], quat.3);
			hprintln!("{:?}", buf).unwrap();
	}
	
	extern "C" {
		fn EXTI0();
	}

};
