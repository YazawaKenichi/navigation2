#ifndef CORE_HPP_
#define CORE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

int getch(void);

#endif

