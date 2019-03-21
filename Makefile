BUILDDIR = build
EXAMPLE1 = example1_report_norm
EXAMPLE2 = example2_report_rich
EXAMPLE3 = example3_report_develop
EXAMPLE4 = example4_control_tcp_motion
EXAMPLE5 = example5_control_485_motion
EXAMPLE6 = example6_test_fetch_instruction
EXAMPLE7 = example7_test_gripper
EXAMPLE8 = example8_control_tcp_motion
EXAMPLE9 = example9_gpio

SRC_DIR	= ./xarm/
SRC_IDIR = ./

EXAMPLE_DIR	= ./example/
COMMON_DIR = $(SRC_DIR)common/
DEBUG_DIR = $(SRC_DIR)debug/
INSTRUCTION_DIR = $(SRC_DIR)instruction/
LINUX_DIR	= $(SRC_DIR)linux/
PORT_DIR = $(SRC_DIR)port/


INCDIRS = -I$(SRC_IDIR)

CCC = g++
C_DEFS = -DSOFT_VERSION=$(SOFT_VERSION)
CFLAGS = -Wall -g -s $(C_DEFS) $(INCDIRS) $(LIBDIRS)
LIBS += -lm -lpthread

ALL_O  = $(SRC_DIR)connect.o $(SRC_DIR)report_data.o
ALL_O += $(COMMON_DIR)crc16.o $(COMMON_DIR)queue_memcpy.o
ALL_O += $(DEBUG_DIR)debug_print.o
ALL_O += $(LINUX_DIR)network.o $(LINUX_DIR)thread.o
ALL_O += $(PORT_DIR)serial.o $(PORT_DIR)socket.o
ALL_O += $(INSTRUCTION_DIR)uxbus_cmd.o $(INSTRUCTION_DIR)uxbus_cmd_ser.o $(INSTRUCTION_DIR)uxbus_cmd_tcp.o

EXAMPLE1_O = $(EXAMPLE_DIR)$(EXAMPLE1)
EXAMPLE2_O = $(EXAMPLE_DIR)$(EXAMPLE2)
EXAMPLE3_O = $(EXAMPLE_DIR)$(EXAMPLE3)
EXAMPLE4_O = $(EXAMPLE_DIR)$(EXAMPLE4)
EXAMPLE5_O = $(EXAMPLE_DIR)$(EXAMPLE5)
EXAMPLE6_O = $(EXAMPLE_DIR)$(EXAMPLE6)
EXAMPLE7_O = $(EXAMPLE_DIR)$(EXAMPLE7)
EXAMPLE8_O = $(EXAMPLE_DIR)$(EXAMPLE8)
EXAMPLE9_O = $(EXAMPLE_DIR)$(EXAMPLE9)

ALL_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(ALL_O))))
EXAMPLE1_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE1_O))))
EXAMPLE2_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE2_O))))
EXAMPLE3_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE3_O))))
EXAMPLE4_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE4_O))))
EXAMPLE5_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE5_O))))
EXAMPLE6_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE6_O))))
EXAMPLE7_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE7_O))))
EXAMPLE8_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE8_O))))
EXAMPLE9_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE9_O))))

all: $(EXAMPLE1) $(EXAMPLE2) $(EXAMPLE3) $(EXAMPLE4) $(EXAMPLE5) $(EXAMPLE6) $(EXAMPLE7) $(EXAMPLE8) $(EXAMPLE9)

$(EXAMPLE1): $(ALL_OBJ) $(EXAMPLE1_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE2): $(ALL_OBJ) $(EXAMPLE2_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE3): $(ALL_OBJ) $(EXAMPLE3_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE4): $(ALL_OBJ) $(EXAMPLE4_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE5): $(ALL_OBJ) $(EXAMPLE5_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE6): $(ALL_OBJ) $(EXAMPLE6_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE7): $(ALL_OBJ) $(EXAMPLE7_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE8): $(ALL_OBJ) $(EXAMPLE8_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(EXAMPLE9): $(ALL_OBJ) $(EXAMPLE9_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

$(BUILDDIR)/%.o: %.cc
	mkdir -p $(dir $@)
	$(CCC) -c $(CFLAGS) $< -o $@

clean:
	rm -rf ./build

