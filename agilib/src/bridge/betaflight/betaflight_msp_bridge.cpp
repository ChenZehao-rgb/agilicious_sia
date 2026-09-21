#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace agi::hardware {
namespace {
uint8_t crc8(uint8_t crc, uint8_t value) {
  crc ^= value;
  for (int i = 0; i < 8; ++i)
    crc = (crc & 0x80) ? (crc << 1) ^ 0xd5 : crc << 1;
  return crc;
}
bool waitFd(int fd, short events, double deadline) {
  while (monotonicSeconds() < deadline) {
    pollfd p{fd, events, 0};
    // Never round up a caller's absolute deadline by more than 1 ms.
    const int ms = static_cast<int>(std::ceil(
      1000 * (deadline - monotonicSeconds())));
    if (ms <= 0) return false;
    const int n = poll(&p, 1, ms);
    if (n > 0) return !(p.revents & (POLLERR | POLLHUP | POLLNVAL)) &&
                       (p.revents & events);
    if (n < 0 && errno != EINTR) return false;
  }
  return false;
}
}
double monotonicSeconds() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts))
    throw std::runtime_error("clock_gettime failed");
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}
void MspDecoder::append(const uint8_t* data, size_t size) {
  if (size > 4096 || bytes_.size() + size > 4096) {
    bytes_.clear();
    ++errors_;
    return;
  }
  bytes_.insert(bytes_.end(), data, data + size);
}
bool MspDecoder::next(MspFrame* frame) {
  while (bytes_.size() >= 3) {
    if (bytes_[0] != '$' || (bytes_[1] != 'M' && bytes_[1] != 'X') ||
        (bytes_[2] != '>' && bytes_[2] != '!')) {
      bytes_.erase(bytes_.begin());
      ++errors_;
      continue;
    }
    const bool v2 = bytes_[1] == 'X';
    const size_t header = v2 ? 8 : 5;
    if (bytes_.size() < header) return false;
    const size_t length = v2 ? bytes_[6] + (bytes_[7] << 8) : bytes_[3];
    if (length > 1024) {
      bytes_.erase(bytes_.begin());
      ++errors_;
      continue;
    }
    if (bytes_.size() < header + length + 1) return false;
    uint8_t check = 0;
    for (size_t i = 3; i < header + length; ++i)
      check = v2 ? crc8(check, bytes_[i]) : check ^ bytes_[i];
    if (check != bytes_[header + length]) {
      bytes_.erase(bytes_.begin());
      ++errors_;
      continue;
    }
    frame->code = v2 ? bytes_[4] + (bytes_[5] << 8) : bytes_[4];
    frame->error = bytes_[2] == '!';
    frame->payload.assign(bytes_.begin() + header,
                          bytes_.begin() + header + length);
    bytes_.erase(bytes_.begin(), bytes_.begin() + header + length + 1);
    return true;
  }
  return false;
}
BetaflightMspBridge::BetaflightMspBridge(const std::string& device, int baud, NavigationPolicy policy)
        : owner_(std::this_thread::get_id()), gate_(policy) {
	speed_t speed;
	switch (baud) {
		case 115200:
			speed = B115200;
			break;
		case 460800:
			speed = B460800;
			break;
		case 921600:
			speed = B921600;
			break;
		default:
			throw std::runtime_error("baud must be 115200, 460800 or 921600");
	}
	fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (fd_ < 0) throw std::runtime_error("open " + device + ": " + strerror(errno));
	termios settings{};
	const auto fail = [&]() {
		const std::string error = strerror(errno);
		close(fd_);
		fd_ = -1;
		throw std::runtime_error("exclusive serial configuration: " + error);
	};
	if (ioctl(fd_, TIOCEXCL) || tcgetattr(fd_, &settings)) fail();
	cfmakeraw(&settings);
	settings.c_cflag |= CLOCAL | CREAD;
	settings.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
	settings.c_cc[VMIN] = 0;
	settings.c_cc[VTIME] = 0;
	if (cfsetispeed(&settings, speed) || cfsetospeed(&settings, speed) || tcsetattr(fd_, TCSANOW, &settings) || tcflush(fd_, TCIOFLUSH))
		fail();
}
BetaflightMspBridge::~BetaflightMspBridge() {
	if (fd_ >= 0) {
		tcflush(fd_, TCOFLUSH);
		ioctl(fd_, TIOCNXCL);
		close(fd_);
	}
}
void BetaflightMspBridge::checkOwner() const {
  if (std::this_thread::get_id() != owner_)
    throw std::logic_error("MSP serial accessed from a second thread");
}
bool BetaflightMspBridge::writeFrame(uint16_t code, const std::vector<uint8_t>& payload, double deadline) {
	checkOwner();
	const double start = monotonicSeconds();
	if (failed_ || !std::isfinite(deadline) || deadline <= start || deadline - start > 0.1 || payload.size() > 254) return false;
	const bool v2 = code > 254;
	std::vector<uint8_t> bytes;
	if (v2) {
		bytes = {'$',
		         'X',
		         '<',
		         0,
		         static_cast<uint8_t>(code & 255),
		         static_cast<uint8_t>(code >> 8),
		         static_cast<uint8_t>(payload.size()),
		         0};
	} else {
		bytes = {'$', 'M', '<', static_cast<uint8_t>(payload.size()), static_cast<uint8_t>(code)};
	}
	bytes.insert(bytes.end(), payload.begin(), payload.end());
	uint8_t check = 0;
	for (size_t i = 3; i < bytes.size(); ++i) check = v2 ? crc8(check, bytes[i]) : check ^ bytes[i];
	bytes.push_back(check);
	size_t offset = 0;
	while (offset < bytes.size() && monotonicSeconds() < deadline) {
		const ssize_t n = write(fd_, bytes.data() + offset, bytes.size() - offset);
		if (n > 0)
			offset += static_cast<size_t>(n);
		else if (n < 0 && errno == EINTR)
			continue;
		else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			if (!waitFd(fd_, POLLOUT, deadline)) break;
		} else
			break;
	}
	last_write_seconds_ = monotonicSeconds() - start;
	if (offset != bytes.size() || monotonicSeconds() > deadline) {
		++errors_;
		failed_ = true;  // Partial frames cannot be retried as another RC command.
		tcflush(fd_, TCOFLUSH);
		return false;
	}
	return true;  // Kernel acceptance only, NOT proof of FC receipt.
}
bool BetaflightMspBridge::request(uint8_t code, MspFrame* reply,
                                 double deadline) {
  checkOwner();
  if (!reply || !sendRequest(code, deadline)) return false;
  while (monotonicSeconds() < deadline) {
    MspFrame frame;
    while (decoder_.next(&frame)) {
      if (frame.code == code) {
        *reply = frame;
        if (frame.error) ++errors_;
        return !frame.error;
      }
    }
    if (!waitFd(fd_, POLLIN, deadline)) break;
    uint8_t bytes[512];
    const ssize_t n = read(fd_, bytes, sizeof(bytes));
    if (n > 0) decoder_.append(bytes, static_cast<size_t>(n));
    else if (n < 0 && errno != EAGAIN && errno != EINTR) break;
  }
  ++errors_;
  // A timeout leaves response identity ambiguous (MSP has no sequence ID).
  // Latch closed; restart only in manual, after investigating the link.
  failed_ = true;
  return false;
}
bool BetaflightMspBridge::sendRequest(uint8_t code, double deadline) {
  checkOwner();
  switch (code) {
	  case 1:
	  case 2:
	  case 3:
	  case 5:
	  case 34:
	  case 44:
	  case 64:
	  case 101:
	  case 105:
	  case 125:
	  case 238:
	  case 106:
	  case 108:
	  case 110:
	  case 111:
	  case 114:
	  case 119:
	  case 130:
	  case 150:
		  return writeFrame(code, {}, deadline);
	  default:
		  return false;
  }
}
bool BetaflightMspBridge::readOverrideSetting(const std::string& name, double deadline) {
	if (name != "msp_override_channels_mask" && name != "msp_override_failsafe" && name != "msp_override_timeout_ms") return false;
	// NUL padding gives the firmware room for its reply buffer. No '=' is ever
	// transmitted: MSP2_CLI_SETTING's write variant is intentionally inaccessible.
	std::vector<uint8_t> payload(96, 0);
	std::copy(name.begin(), name.end(), payload.begin());
	return writeFrame(0x3010, payload, deadline);
}

bool BetaflightMspBridge::receive(MspFrame* reply) {
  checkOwner();
  if (!reply) return false;
  if (!decoder_.next(reply)) {
    uint8_t bytes[512];
    const ssize_t n = read(fd_, bytes, sizeof(bytes));
    if (n > 0) decoder_.append(bytes, static_cast<size_t>(n));
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      ++errors_;
      failed_ = true;
      throw std::runtime_error("MSP serial read failed");
    }
    if (!decoder_.next(reply)) return false;
  }
  if (reply->error) ++errors_;
  return true;
}
bool BetaflightMspBridge::sendBenchRc(
  const std::array<uint16_t, 4>& channels, double deadline) {
  if (!std::all_of(channels.begin(), channels.end(),
      [](uint16_t x) { return x >= 1000 && x <= 2000; })) return false;
  std::vector<uint8_t> payload;
  for (auto channel : channels) {
    payload.push_back(channel & 255);
    payload.push_back(channel >> 8);
  }
  return writeFrame(200, payload, deadline);
}
bool BetaflightMspBridge::sendOverride(
  const std::array<uint16_t, 4>& channels, const Evidence& evidence,
  double deadline) {
  checkOwner();
  Evidence checked = evidence;
  checked.now = monotonicSeconds(); // Never trust a producer's frozen clock.
  checked.command_valid = checked.command_valid && std::all_of(
    channels.begin(), channels.end(), [](uint16_t x) { return x >= 1000 && x <= 2000; });
  if (!gate_.update(checked) || failed_) return false;
  std::vector<uint8_t> payload;
  for (uint16_t channel : channels) {
    payload.push_back(channel & 255);
    payload.push_back(channel >> 8);
  }
  if (!writeFrame(200, payload, deadline)) return false;
  last_send_time_ = monotonicSeconds();
  return true;
}
}
