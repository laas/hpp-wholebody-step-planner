noinst_PROGRAMS = benchmark

benchmark_SOURCES = benchmark.cc

benchmark_CPPFLAGS = \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(TLCWHOLEBODYPLANNER_CFLAGS)

benchmark_LDFLAGS = \
	../src/.libs/libhpp-wholebody-step-planner.a \
	$(TLCWHOLEBODYPLANNER_LIBS) \
	$(ROBOTBUILDER_LIBS)


