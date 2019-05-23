# SDP3x
Linux device driver for the Sensirion SDP31, SDP32 digital differential pressure sensors

**Device tree bindings for SDP3x:**

Required properties:
- compatible: must be "sensirion,sdp3x"
- reg: i2c slave address of the chip

Optional properties:
- irq-gpio: interrupt gpio the chip's IRQn pin is connected to
- sampling-period-us: sampling period to poll sensor data, default 10000 microseconds

**Example:**

	i2c@00000000 {
		/* ... */

		sensirion_sdp3x@21 {
			compatible = "sensirion,sdp3x";
			reg = <0x21>;
			irq-gpio = <&msmgpio 28 0>;
			sampling-period-us = <10000>;
		};

		/* ... */
	};
