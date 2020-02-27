#include "navio_board.h"

namespace rosflight_firmware
{

NavioBoard::NavioBoard()
{
  clock_start_ = std::chrono::high_resolution_clock::now();
  clock_now_   = std::chrono::high_resolution_clock::now();
  for(int i = 0; i<PWM_NUM_OUTPUTS; i++){
      ch_ind_[PWM_CHANNELS[i]] = i;
  }
}

void NavioBoard::init_board()
{
  // system check
  check_apm(); // if apm is running, hardware cannot be accessed 
  if (getuid()) 
  {
    printf("Not root. Please launch with sudo \n");
  }
  // ignoring barometer for now 
  // int_i2c_.init(&i2c_config[BARO_I2C]);
  // ext_i2c_.init(&i2c_config[EXTERNAL_I2C]);
  
  // spi1_.init(&spi_config[MPU6000_SPI]);
  // spi3_.init(&spi_config[FLASH_SPI]);
  // uart1_.init(&uart_config[UART1], 115200, UART::MODE_8N1);
  // uart3_.init(&uart_config[UART3], 115200, UART::MODE_8N1);

//   backup_sram_init();

//   current_serial_ = &vcp_;    //uncomment this to switch to VCP as the main output
}

void NavioBoard::board_reset(bool bootloader)
{
//   (void)bootloader;
//   NVIC_SystemReset();
}

// clock
uint32_t NavioBoard::clock_millis()
{
  clock_now_ = std::chrono::high_resolution_clock::now();
  millis_ = clock_now_ - clock_start_;
  return uint32_t(millis_.count());
}

uint64_t NavioBoard::clock_micros()
{
  clock_now_ = std::chrono::high_resolution_clock::now();
  //  micros_ = std::chrono::duration_cast<std::chrono::milliseconds>(clock_now_ - clock_start_);
  micros_ = clock_now_ - clock_start_;
  return uint64_t(micros_.count());
}

void NavioBoard::clock_delay(uint32_t milliseconds)
{
  //  delay(milliseconds);
  usleep(1000*milliseconds);
}

// serial
void NavioBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
//   vcp_.init();
//   switch (dev)
//   {
//   case SERIAL_DEVICE_UART3:
//     uart3_.init(&uart_config[UART3], baud_rate);
//     current_serial_ = &uart3_;
//     secondary_serial_device_ = SERIAL_DEVICE_UART3;
//     break;
//   default:
//     current_serial_ = &vcp_;
//     secondary_serial_device_ = SERIAL_DEVICE_VCP;
//   }
}

void NavioBoard::serial_write(const uint8_t *src, size_t len)
{
//   current_serial_->write(src, len);
}

uint16_t NavioBoard::serial_bytes_available()
{
//   if (vcp_.connected() || secondary_serial_device_ == SERIAL_DEVICE_VCP)
//   {
//     current_serial_ = &vcp_;
//   }
//   else
//   {
//     switch (secondary_serial_device_)
//     {
//     case SERIAL_DEVICE_UART3:
//       current_serial_ = &uart3_;
//       break;
//     default:
//       // no secondary serial device
//       break;
//     }
//   }

  return 1;//current_serial_->rx_bytes_waiting();
}

uint8_t NavioBoard::serial_read()
{

  return 1;//current_serial_->read_byte();
}

void NavioBoard::serial_flush()
{
//   current_serial_->flush();
}


// sensors
void NavioBoard::sensors_init()
{
  while (clock_millis() < 50) {} // wait for sensors to boot up
  led_.initialize();
  imu_.initialize();
  baro_.initialize();

//   baro_.init(&int_i2c_);
//   mag_.init(&int_i2c_);
//   sonar_.init(&ext_i2c_);
//   airspeed_.init(&ext_i2c_);
//   // gnss_.init(&uart1_);
}

uint16_t NavioBoard::num_sensor_errors()
{
  return 1;//ext_i2c_.num_errors();
}

bool NavioBoard::new_imu_data()
{
  imu_.update();
  return true;//imu_.new_data();
}

bool NavioBoard::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
//   float read_accel[3], read_gyro[3];
//   imu_.read(read_accel, read_gyro, temperature, time_us);
  imu_.read_accelerometer(&ax, &ay, &az);
  imu_.read_gyroscope(&gx, &gy, &gz);

  accel[0] = -ax; // read_accel[1]; << verify this >>
  accel[1] = -ay; // read_accel[0];
  accel[2] = -az; // read_accel[2];

  gyro[0] = -gx; // read_gyro[1];
  gyro[1] = -gy; // read_gyro[0];
  gyro[2] = -gz; // read_gyro[2];

  return true;
}

void NavioBoard::imu_not_responding_error()
{
  sensors_init();
}

bool NavioBoard::mag_present()
{
//   mag_.update();
  return true; //mag_.present();
}

void NavioBoard::mag_update()
{
//   mag_.update();
}

void NavioBoard::mag_read(float mag[3])
{
//   mag_.update();
//   mag_.read(mag);
  imu_.read_magnetometer(&mx, &my, &mz);
  mag[0] = mx;
  mag[1] = my;
  mag[2] = mz;

}

bool NavioBoard::baro_present()
{
//   baro_.update();
  return true; //baro_.present();
}

void NavioBoard::baro_update()
{
  baro_.refreshPressure();
  // usleep(10000); // Waiting for pressure data ready
  baro_.readPressure();

  baro_.refreshTemperature();
  // usleep(10000); // Waiting for temperature data ready
  baro_.readTemperature();

  baro_.calculatePressureAndTemperature();
}

void NavioBoard::baro_read(float *pressure, float *temperature)
{
  baro_update();
//   baro_.read(pressure, temperature);
  *pressure = baro_.getPressure(); // millibars
  *temperature =  baro_.getTemperature(); // C

}

bool NavioBoard::diff_pressure_present()
{
  return false; //airspeed_.present();
}

void NavioBoard::diff_pressure_update()
{
//   airspeed_.update();
}


void NavioBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
//   (void) diff_pressure;
//   (void) temperature;
//   airspeed_.update();
//   airspeed_.read(diff_pressure, temperature);
}

bool NavioBoard::sonar_present()
{
  return false; //sonar_.present();
}

void NavioBoard::sonar_update()
{
//   sonar_.update();
}

float NavioBoard::sonar_read()
{
  return 0; //sonar_.read();
}

bool NavioBoard::gnss_present()
{
  // return gnss_.present();
  return false;
}
void NavioBoard::gnss_update() {}
bool NavioBoard::gnss_has_new_data()
{
  // return this->gnss_.new_data();
  return false;
}
//This method translates the UBLOX driver interface into the ROSFlight interface
//If not gnss_has_new_data(), then this may return 0's for ECEF position data,
//ECEF velocity data, or both
GNSSData NavioBoard::gnss_read()
{
  // UBLOX::GNSSPVT gnss_pvt= gnss_.read();
  // UBLOX::GNSSPosECEF pos_ecef = gnss_.read_pos_ecef();
  // UBLOX::GNSSVelECEF vel_ecef = gnss_.read_vel_ecef();
  GNSSData gnss = {};
  // gnss.time_of_week = gnss_pvt.time_of_week;
  // gnss.time = gnss_pvt.time;
  // gnss.nanos = gnss_pvt.nanos;
  // gnss.lat = gnss_pvt.lat;
  // gnss.lon = gnss_pvt.lon;
  // gnss.height = gnss_pvt.height;
  // gnss.vel_n = gnss_pvt.vel_n;
  // gnss.vel_e = gnss_pvt.vel_e;
  // gnss.vel_d = gnss_pvt.vel_d;
  // gnss.h_acc = gnss_pvt.h_acc;
  // gnss.v_acc = gnss_pvt.v_acc;
  // //Does not include ECEF position data if the timestamp doesn't match
  // //See UBLOX::new_data() for reasoning
  // if (gnss.time_of_week == pos_ecef.time_of_week)
  // {
  //   gnss.ecef.x = pos_ecef.x;
  //   gnss.ecef.y = pos_ecef.y;
  //   gnss.ecef.z = pos_ecef.z;
  //   gnss.ecef.p_acc = pos_ecef.p_acc;
  // }
  // //Does not include ECEF position data if the timestamp doesn't match
  // //See UBLOX::new_data() for reasoning
  // if (gnss.time_of_week == vel_ecef.time_of_week)
  // {
  //   gnss.ecef.vx = vel_ecef.vx;
  //   gnss.ecef.vy = vel_ecef.vy;
  //   gnss.ecef.vz = vel_ecef.vz;
  //   gnss.ecef.s_acc = vel_ecef.s_acc;
  // }

  return gnss;
}
GNSSRaw NavioBoard::gnss_raw_read()
{
//  UBLOX::NAV_PVT_t pvt = gnss_.read_raw();
  GNSSRaw raw = {};
  // raw.time_of_week = pvt.iTOW;
  // raw.year = pvt.time.year;
  // raw.month = pvt.time.month;
  // raw.day = pvt.time.day;
  // raw.hour = pvt.time.hour;
  // raw.min = pvt.time.min;
  // raw.sec = pvt.time.sec;
  // raw.valid = pvt.time.valid;
  // raw.t_acc = pvt.time.tAcc;
  // raw.nano = pvt.time.nano;
  // raw.fix_type = pvt.fixType;
  // raw.num_sat = pvt.numSV;
  // raw.lon = pvt.lon;
  // raw.lat = pvt.lat;
  // raw.height = pvt.height;
  // raw.height_msl = pvt.hMSL;
  // raw.h_acc = pvt.hAcc;
  // raw.v_acc = pvt.vAcc;
  // raw.vel_n = pvt.velN;
  // raw.vel_e = pvt.velE;
  // raw.vel_d = pvt.velD;
  // raw.g_speed = pvt.gSpeed;
  // raw.head_mot = pvt.headMot;
  // raw.s_acc = pvt.sAcc;
  // raw.head_acc = pvt.headAcc;
  // raw.p_dop = pvt.pDOP;
  // raw.rosflight_timestamp = gnss_.get_last_pvt_timestamp();
  return raw;
}

// PWM
void NavioBoard::rc_init(rc_type_t rc_type)
{
  rc_.initialize();
//   switch (rc_type)
//   {
//   case RC_TYPE_SBUS:
//     uart1_.init(&uart_config[UART1], 100000, UART::MODE_8E2);
//     inv_pin_.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
//     rc_sbus_.init(&inv_pin_, &uart1_);
//     rc_ = &rc_sbus_;
//     break;
//   case RC_TYPE_PPM:
//   default:
//     rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
//     rc_ = &rc_ppm_;
//     break;
//   }
}

float NavioBoard::rc_read(uint8_t channel)
{
  return rc_.read(channel);
}

void NavioBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].initialize(PWM_CHANNELS[i]);
    esc_out_[i].set_duty_cycle(idle_pwm);
  }
}

void NavioBoard::pwm_disable()
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].disarm();
  }
}

void NavioBoard::pwm_write(uint8_t channel, float value)
{
  if (channel < PWM_NUM_OUTPUTS)
  {
    esc_out_[ch_ind_[channel]].set_duty_cycle(value); // << is this microseconds or milli or just seconds
  }
}

bool NavioBoard::rc_lost()
{
  return true; //rc_->lost();
}

// non-volatile memory
void NavioBoard::memory_init()
{
//   return flash_.init(&spi3_);
}

bool NavioBoard::memory_read(void *data, size_t len)
{
  return false; //flash_.read_config(reinterpret_cast<uint8_t *>(data), len);
}

bool NavioBoard::memory_write(const void *data, size_t len)
{
  return false; //flash_.write_config(reinterpret_cast<const uint8_t *>(data), len);
}

// LED
/*
  color combination since there
  is only one color led on Navio2

  led0    led1    color
  ---------------------
   yes     no      Blue
   no      yes     Green
   no      no      Red
   yes     yes     Cyan

*/
void NavioBoard::led0_on()
{
  if (led1_)
    led_.setColor(Colors::Cyan);
  else
    led_.setColor(Colors::Blue);
  led0_ = true;
}

void NavioBoard::led0_off()
{
  if (led1_)
    led_.setColor(Colors::Green);
  else
    led_.setColor(Colors::Red);
  led0_ = false;
}

void NavioBoard::led0_toggle()
{
    if (led0_)
      NavioBoard::led0_off();
    else
      NavioBoard::led0_on();
}

void NavioBoard::led1_on()
{
  if (led0_)
    led_.setColor(Colors::Cyan);
  else
    led_.setColor(Colors::Green);
  led1_ = true;
}

void NavioBoard::led1_off()
{
  if (led0_)
    led_.setColor(Colors::Blue);
  else
    led_.setColor(Colors::Red);
  led1_ = false;
}

void NavioBoard::led1_toggle()
{
    if (led1_)
      NavioBoard::led1_off();
    else
      NavioBoard::led1_on();
}

//Backup memory
bool NavioBoard::has_backup_data()
{
//   BackupData backup_data = backup_sram_read();
  return false; //(check_backup_checksum(backup_data) && backup_data.error_code!=0);
}

rosflight_firmware::BackupData NavioBoard::get_backup_data()
{
	rosflight_firmware::BackupData tmp;
	return tmp;
//   return backup_sram_read();
}
}
