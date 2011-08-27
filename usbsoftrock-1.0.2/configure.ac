AC_PREREQ([2.63])
AC_INIT(usbsoftrock.c)
AM_INIT_AUTOMAKE(usbsoftrock,1.0.2)
AC_CONFIG_SRCDIR([src/usbsoftrock.c])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_INSTALL

# Some common path names to search
CPPFLAGS="-I../ -I/usr/local/include -I/opt/local/include ${CPPFLAGS}"
CFLAGS="-I../ -I/usr/local/include -I/opt/local/include ${CFLAGS}"
LDFLAGS="-L/usr/lib -L/usr/local/lib -L/opt/local/lib ${LDFLAGS}"

# Checks for libraries.
AC_CHECK_LIB(ncurses, main,,echo "*** The ncurses library is required for interactive mode!")
PKG_CHECK_MODULES(libusb, libusb,
	[
	CFLAGS="$CFLAGS $libusb_CFLAGS"
	CPPFLAGS="$CPPFLAGS $libusb_CFLAGS"
	LIBS="$LIBS $libusb_LIBS"
	], 
	[AC_CHECK_LIB(usb, main,, echo "*** The libusb library is required!"; exit 1)]
)
#AC_CHECK_LIB(usb, main,, echo "*** The libusb library is required!"; exit 1)
#PKG_CHECK_MODULES(libusb, libusb )

#AC_SUBST(libusb_CFLAGS)
#AC_SUBST(libusb_LIBS)

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile src/Makefile])
AC_OUTPUT
