
rtems_tests_PROGRAMS = i2c_adxl345
i2c_adxl345_SOURCES = init.c

#dist_rtems_tests_DATA = i2c_adxl345.scn
#dist_rtems_tests_DATA += i2c_adxl345.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am


LINK_OBJS = $(i2c_adxl345_OBJECTS)
LINK_LIBS = $(i2c_adxl345_LDLIBS)

i2c_adxl345$(EXEEXT): $(i2c_adxl345_OBJECTS) $(i2c_adxl345_DEPENDENCIES)
	@rm -f i2c_adxl345$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
