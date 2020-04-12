/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "NavioRCInput.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>

using namespace time_literals;

namespace navio_rc_in
{

NavioRCInput::NavioRCInput() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_isRunning = true;
};

NavioRCInput::~NavioRCInput()
{
	ScheduleClear();

	_isRunning = false;

	for (int i = 0; i < CHANNELS; ++i) {
		::close(_channel_fd[i]);
	}

	::close(_connected_fd);

	perf_free(_publish_interval_perf);
}

int NavioRCInput::navio_rc_init()
{
	//init SBUS
	_connected_fd = ::open("/sys/kernel/rcio/rcin/connected", O_RDONLY);
	for (int i = 0; i < CHANNELS; ++i) {
		char buf[80] {};
		::snprintf(buf, sizeof(buf), "%s/ch%d", "/sys/kernel/rcio/rcin", i);
		int fd = ::open(buf, O_RDONLY);
		if (fd < 0) {	PX4_ERR("open %s (%d) failed", buf, i);	break; }
		_channel_fd[i] = fd;
	}

	//init UDP
	_src_addr.sin_family = AF_INET;
	_src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	_src_addr.sin_port = htons(_network_port);
	_src_addr_len = (socklen_t) sizeof(_src_addr);
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (_socket_fd < 0) {	PX4_WARN("create socket failed: %s", strerror(errno)); }
	int ret = bind(_socket_fd, (struct sockaddr *)&_src_addr, _src_addr_len);
	if (ret < 0) { PX4_WARN("bind failed: %s", strerror(errno)); }

	return PX4_OK;
}

int NavioRCInput::start()
{
	navio_rc_init();

	_should_exit.store(false);

	ScheduleOnInterval(10_ms); // 100 Hz

	return PX4_OK;
}

void NavioRCInput::stop()
{
	_should_exit.store(true);
}

void NavioRCInput::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	input_rc_s data{};
	uint64_t timestamp_sample = hrt_absolute_time();


	bool recv_udp_success = read_udp_channels((uint16_t*)data.values);

	if(recv_udp_success) {
		if (_network_tries >= _network_tries_treshold) { PX4_WARN("regained UDP rc input"); }
		_network_tries = 0;
	} else {
		_network_tries += 1;
		if (_network_tries < _network_tries_treshold) {	return;	}
		if (_network_tries == _network_tries_treshold) { PX4_WARN("lost UDP rc input. falling back to radio."); }

		bool recv_fd_success = read_fd_channels((uint16_t*)data.values);
		if (!recv_fd_success) { return; }
	}

	data.timestamp_last_signal = timestamp_sample;
	data.rc_lost = !_connected;
	data.channel_count = CHANNELS;
	data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
	data.timestamp = hrt_absolute_time();

	_input_rc_pub.publish(data);
	perf_count(_publish_interval_perf);
}

bool NavioRCInput::read_udp_channels(uint16_t* values) {

	//try to get rc data from udp socket
	char udp_buf[128]; int recv_len = 0; bool recv_success = false;
	do {
		recv_len = recvfrom(_socket_fd, (char *)udp_buf, 128, MSG_DONTWAIT, ( struct sockaddr *) &_src_addr, &_src_addr_len);
		if (recv_len >= CHANNELS*2) {
			for (int i = 0; i < CHANNELS; ++i) {
					uint16_t val = ((uint16_t)udp_buf[i*2]) | ((uint16_t)udp_buf[i*2+1])<<8;
					val = (uint16_t) ( ((float) val + 1400.0f)/1.6f );
					values[i] = val;
			}
			recv_success = true;
			_connected = true;
		}
	} while (recv_len > 0);

	return recv_success;
}

bool NavioRCInput::read_fd_channels(uint16_t* values) {

	char connected_buf[12] {};
	int ret_connected = ::pread(_connected_fd, connected_buf, sizeof(connected_buf) - 1, 0);
	if (ret_connected < 0) { return false; }
	connected_buf[sizeof(connected_buf) - 1] = '\0';
	_connected = (atoi(connected_buf) == 1);

	bool all_zero = true;
	for (int i = 0; i < CHANNELS; ++i) {

		char buf[12] {};
		int res = ::pread(_channel_fd[i], buf, sizeof(buf) - 1, 0);
		if (res < 0) { continue; }

		buf[sizeof(buf) - 1] = '\0';
		values[i] = atoi(buf);

		if (atoi(buf) > 900) { all_zero = false; }
	}
	if (all_zero) {	return false; }

	return true;
}

int NavioRCInput::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("connected: %d", _connected);

	perf_print_counter(_publish_interval_perf);

	return 0;
}

}; // namespace navio_rc_in
