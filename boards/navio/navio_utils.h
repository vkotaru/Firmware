#ifndef ROSFLIGHT_FIRMWARE_NAVIO_UTILS_H
#define ROSFLIGHT_FIRMWARE_NAVIO_UTILS_H

#include <cmath>
#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>

namespace rosflight_firmware
{

class PWM_Navio2 
{
private:
  unsigned int CHANNEL;
  unsigned int PWM_FREQ = 50;
  RCOutput_Navio2 pwm;

  float PWM_CUTOFF_MAX = 1900;    // us (micro-second) 
  float PWM_CUTOFF_MIN = 1050;    // us (micro-second)
  const float PWM_HIGH = 2000;    // us (micro-second)
  const float PWM_LOW  = 1000;    // us (micro-second)


public:
  PWM_Navio2() 
  {

  }
  ~PWM_Navio2() {}

  void initialize(int _ch) 
  {
    printf("Brushless motor on channel %d\n",_ch);
    CHANNEL = (unsigned int) _ch;
    pwm.initialize(CHANNEL);
    pwm.set_frequency(CHANNEL, PWM_FREQ);
    if (!PWM_Navio2::enable()) {
        printf("\033[31;1m  PWM channel not enabled!!! \033[0m\n");
    }
  }

  bool enable () 
  {
    return pwm.enable(CHANNEL);
  }

  void update_frequency(int _freq) 
  {
    PWM_FREQ = _freq;
    pwm.set_frequency(CHANNEL, PWM_FREQ);
  }

  void set_pwm_cutoff_max (float _pwm) 
  {
    PWM_CUTOFF_MAX = _pwm;
  }

  void set_pwm_cutoff_min (float _pwm) 
  {
    PWM_CUTOFF_MIN = _pwm;
  }

  void arm() 
  {
    pwm.set_duty_cycle(CHANNEL, PWM_CUTOFF_MIN);
  }

  void disarm() 
  {
    pwm.set_duty_cycle(CHANNEL, PWM_LOW);
  }

  void set_duty_cycle(float pwm_in_us) 
  {
    // pwm in microseconds
    pwm.set_duty_cycle(CHANNEL, pwm_in_us);
  }

  void calibrate() 
  {
    sleep(1);
    pwm.set_duty_cycle(CHANNEL, PWM_LOW);
    sleep(1);
    // pwm.set_duty_cycle(CHANNEL, PWM_LOW);
    // sleep(3);
  }
};

class Socket
{

public:
    Socket(char * ip,char * port)
    {
        sockfd = socket(AF_INET,SOCK_DGRAM,0);
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(ip);
        servaddr.sin_port = htons(atoi(port));
    }

    Socket()
    {
        sockfd = socket(AF_INET,SOCK_DGRAM,0);
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(7000);
    }

    void output(float W, float X, float Y, float Z, int Hz)
    {
        sprintf(sendline,"%10f %10f %10f %10f %dHz\n", W, X, Y, Z, Hz);
        sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

private:
    int sockfd;
    struct sockaddr_in servaddr = {0};
    char sendline[80];

};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_NAVIO_UTILS_H
