/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/aoas.h>

#include <drivers/device/i2c.h>
#include <drivers/device/device.h>

#define I2C_ADDRESS_AOAS	0x43

#define MEAS_RATE 100
#define MEAS_DRIVER_FILTER_FREQ 1.2f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)


class AOAS : public device::I2C, public I2CSPIDriver<AOAS>
{
public:
	AOAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address);
	virtual ~AOAS() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance);
	static void print_usage();

	void	RunImpl();

protected:

  uORB::PublicationMulti<aoas_s>	_aoas_pub{ORB_ID(aoas)};

};

extern "C" __EXPORT int aoas_main(int argc, char *argv[]);

AOAS::AOAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address)
  : I2C(DRV_SENS_DEVTYPE_AOAS, MODULE_NAME, bus, address, bus_frequency),
	  I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{ }


//measuring loop
void AOAS::RunImpl() {

  //read from sensor
	uint8_t val[4] = {0, 0, 0, 0};
	transfer(nullptr, 0, &val[0], 4);
  float aoa = (float) ( (int16_t) ( ((uint16_t)val[0]) | ((uint16_t)val[1])<<8 ) ) / 100.0f;
  float aos = (float) ( (int16_t) ( ((uint16_t)val[2]) | ((uint16_t)val[3])<<8 ) ) / 100.0f;

  //publish measurements
  aoas_s report{};
  report.timestamp = hrt_absolute_time();
  report.aoa = aoa;
  report.aos = aos;
  _aoas_pub.publish(report);

	// schedule a fresh cycle call
	ScheduleDelayed(CONVERSION_INTERVAL);
}

//initialize sensor
I2CSPIDriverBase *AOAS::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance) {

	AOAS *instance = new AOAS(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleNow();
	return instance;
}


void AOAS::print_usage() {
	PRINT_MODULE_USAGE_NAME("aoas", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int aoas_main(int argc, char *argv[]) {

	using ThisDriver = AOAS;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
  cli.i2c_address = I2C_ADDRESS_AOAS;

  cli.getopt(argc, argv, ":");

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_AOAS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
