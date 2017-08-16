At best, build in own directory, e.g.:
mkdir build; cd build;
"cmake .." oder "cmake -DCMAKE_BUILD_TYPE=Debug .."
mkdir build-win32; cd build-win32;
"cmake -DCMAKE_TOOLCHAIN_FILE=../Toolchain-mingw32.cmake .."

Build import library
pexports -h /opt/cross/i386-mingw32msvc/include/ftd2xx.h \
         /opt/spare/bon/ftdi/ftd2xx/i386/ftd2xx.dll > /tmp/ftd2xx.def
/opt/cross/i386-mingw32msvc/bin/dlltool  -d /tmp/ftd2xx.def -l libftd2xx.a -k

Needed packages (as on a clean osboxes Leap-42):

Install gcc, gcc-c++, git, cmake, libftdi1-devel, libusb-compat-devel

Crosscompile:
Follow instructions on
https://software.opensuse.org/download.html?project=windows%3Amingw%3Awin32&package=mingw32-cross-gcc
zypper addrepo http://download.opensuse.org/repositories/windows:mingw:win32/openSUSE_Leap_42.3/windows:mingw:win32.repo
zypper refresh
zypper install mingw32-cross-gcc mingw32-cross-gcc-c++ mingw32-libftdi1-devel mingw32-cross-pkg-config
