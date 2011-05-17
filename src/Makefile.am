include $(top_srcdir)/build-aux/init.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

lib_LTLIBRARIES = libhpp-wholebody-step-planner.la

libhpp_wholebody_step_planner_la_SOURCES = 			\
					planner.cc 		\
					config-motion-constraint.cc \
					wholebody-constraint.cc \
					path-optimizer.cc \
					distance.cc

libhpp_wholebody_step_planner_la_LIBADD = $(TLCWHOLEBODYPLANNER_LIBS) 
libhpp_wholebody_step_planner_la_CPPFLAGS = $(AM_CPPFLAGS) $(TLCWHOLEBODYPLANNER_CFLAGS) 
