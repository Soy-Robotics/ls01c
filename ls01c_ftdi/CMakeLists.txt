cmake_minimum_required(VERSION 2.8.3)
project(ls01c_ftdi)
find_package(catkin REQUIRED COMPONENTS ecl_command_line)

# pkg-config packages
find_package(PkgConfig)
pkg_search_module(REQUIRED libusb)
pkg_search_module(REQUIRED libftdi)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS ecl_command_line
   DEPENDS libusb libftdi
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/ls01c_config
                 scripts/create_udev_rules
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/ls01c_config
                 scripts/create_laptop_rules
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 57-ls01c.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# This has issues....indigo doing th same thing will result in 
# a file conflict. We could call it 57-indigo-ls01c.rules and
# 57-indigo-ls01c.rules. But that seems kind of lame and could
# break things if they are actually different (which should
# be called - udev will have no idea).

#install(FILES 57-ls01c.rules
#        DESTINATION /etc/udev/rules.d/
#)

