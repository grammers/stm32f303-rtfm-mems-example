#![no_main]
#![no_std]

extern crate stm32f3xx_hal as hal;
extern crate panic_semihosting;
extern crate cortex_m_semihosting;

use cortex_m_semihosting::hprintln;
use cortex_m::asm::delay;
use rtfm::app;

use hal::{prelude::*, hal::digital::v2::OutputPin,
	usb::{Peripheral, UsbBus, UsbBusType},
	gpio::{Output, PushPull}};
use usb_device::{prelude::*, bus, bus::UsbBus as UB};
use usbd_serial::{SerialPort, USB_CLASS_CDC};


#[app(device = stm32f3xx_hal::stm32, peripherals = true)]
const APP: () = {

	struct Resources {
		LED_S: hal::gpio::gpioe::PE13<Output<PushPull>>,
    	USB_DEV: UsbDevice<'static, UsbBusType>,
		SERIAL: SerialPort<'static, UsbBusType>,
	}

	#[init]
	fn init(cx: init::Context) -> init::LateResources {
  
  		static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        // Cortex-M peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let clocks = rcc
			.cfgr
			.use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
			.pclk2(24.mhz())
            .freeze(&mut flash.acr);

		// LEDs
		let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
		let led_s = gpioe.pe13
			.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
		
		// USB pin set upp
		// a reset is allsow requierd
		let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
		let mut usb_dp = gpioa.pa12
			.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
		usb_dp.set_low().expect("could not set pin for synk");
		delay(clocks.sysclk().0 / 100);

		let usb_dm = gpioa.pa11
			.into_af14(&mut gpioa.moder, &mut gpioa.afrh);
		let usb_dp = usb_dp
			.into_af14(&mut gpioa.moder, &mut gpioa.afrh);

		let usb = Peripheral {
			usb: cx.device.USB,
			pin_dm: usb_dm,
			pin_dp: usb_dp,
		};

		// USB bus, serial conection and device set up
        *USB_BUS = Some(UsbBus::new(usb));
        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());
		let usb_dev = UsbDeviceBuilder::new(
	   		USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC) 	// set class, type of usb 
                .build();

		init::LateResources {
			LED_S: led_s,
			USB_DEV: usb_dev,
			SERIAL: serial,
		}
	}
	
    #[task(binds = USB_HP_CAN_TX, resources = [USB_DEV, SERIAL])]
    fn USB_HP_CAN_TX(mut cx: USB_HP_CAN_TX::Context) {
        usb_poll(&mut cx.resources.USB_DEV, &mut cx.resources.SERIAL);
    }

	// systematekly trigerd for USB comunications
	// These is time critical. Can not be bloked
    #[task(binds = USB_LP_CAN_RX0, resources = [USB_DEV, SERIAL, LED_S])]
    fn USB_LP_CAN_RX0(mut cx: USB_LP_CAN_RX0::Context) {
		cx.resources.LED_S.set_high().expect("expected LED to blink");
        usb_poll(&mut cx.resources.USB_DEV, &mut cx.resources.SERIAL);
		cx.resources.LED_S.set_low().expect("expected LED to blink");
    }
};


fn usb_poll<B: UB>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
) {
	// wery importent call. have to be coled at least once every 10ms 
	if !usb_dev.poll(&mut [serial]) {
        return;
    }

//    let mut buf = [0u8; 64];
    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
			// add your enterpeter her.

            // Echo back in upper case
			hprintln!("ok").unwrap();
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}

