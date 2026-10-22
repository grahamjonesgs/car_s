// A device on a Linux I2C bus (/dev/i2c-N), through the kernel's i2c-dev
// driver: register reads and writes as single combined transfers. Used by
// car_imu and car_mag. The user needs access to /dev/i2c-1 (dialout on the
// Pi's Ubuntu).
#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <string>

class I2cDevice {
public:
  I2cDevice() = default;
  I2cDevice(const I2cDevice &) = delete;
  I2cDevice &operator=(const I2cDevice &) = delete;
  ~I2cDevice() { close(); }

  // Open bus (e.g. 1 for /dev/i2c-1) for the device at a 7-bit address.
  // Returns false if the bus can't be opened (opening doesn't probe the
  // device: the first read shows whether it answers)
  bool open(int bus, uint8_t address) {
    close();
    fd_ = ::open(("/dev/i2c-" + std::to_string(bus)).c_str(), O_RDWR);
    address_ = address;
    return fd_ >= 0;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // Write one byte to a register. Returns false on failure
  bool write_byte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_msg msg = {address_, 0, 2, buf};
    return transfer(&msg, 1);
  }

  // Read one register. Returns the value, or -1 on failure
  int read_byte(uint8_t reg) {
    uint8_t value;
    return read_block(reg, &value, 1) ? value : -1;
  }

  // Read n bytes starting at reg, in one transfer (register address, then a
  // repeated start and the read), so they all come from the same sample.
  // Returns false on failure
  bool read_block(uint8_t reg, uint8_t *data, uint16_t n) {
    i2c_msg msgs[2] = {{address_, 0, 1, &reg}, {address_, I2C_M_RD, n, data}};
    return transfer(msgs, 2);
  }

private:
  bool transfer(i2c_msg *msgs, uint32_t n) {
    if (fd_ < 0) {
      return false;
    }
    i2c_rdwr_ioctl_data data = {msgs, n};
    return ioctl(fd_, I2C_RDWR, &data) == static_cast<int>(n);
  }

  int fd_ = -1;
  uint16_t address_ = 0;
};
