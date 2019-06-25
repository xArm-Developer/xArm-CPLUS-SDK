BUILDDIR = build

EXAMPLE0 = test
EXAMPLE1 = test_events
EXAMPLE2 = test_get
EXAMPLE3 = test_set
EXAMPLE4 = test_move_line
EXAMPLE5 = test_move_joint
EXAMPLE6 = test_get_tgpio_digital
EXAMPLE7 = test_get_tgpio_analog
EXAMPLE8 = test_set_tgpio_digital
EXAMPLE9 = test_set_gripper
EXAMPLE10 = test_cgpio

INC_DIR = ./include/
SRC_DIR = ./src/
EXAMPLE_DIR = ./example/

SRC_XARM_DIR = $(SRC_DIR)xarm/
SRC_XARM_CORE_DIR = $(SRC_XARM_DIR)core/
SRC_XARM_WRAPPER_DIR = $(SRC_XARM_DIR)wrapper/

SRC_XARM_CORE_COMMON_DIR = $(SRC_XARM_CORE_DIR)common/
SRC_XARM_CORE_DEBUG_DIR = $(SRC_XARM_CORE_DIR)debug/
SRC_XARM_CORE_INSTRUCTION_DIR = $(SRC_XARM_CORE_DIR)instruction/
SRC_XARM_CORE_LINUX_DIR = $(SRC_XARM_CORE_DIR)linux/
SRC_XARM_CORE_PORT_DIR = $(SRC_XARM_CORE_DIR)port/


INCDIRS = -I$(INC_DIR)

CCC = g++
C_DEFS = -DSOFT_VERSION=$(SOFT_VERSION)
CFLAGS = -std=c++0x -Wall -g -s $(C_DEFS) $(INCDIRS) $(LIBDIRS)
LIBS += -lm -lpthread

ALL_O = $(SRC_XARM_CORE_COMMON_DIR)crc16.o $(SRC_XARM_CORE_COMMON_DIR)queue_memcpy.o
ALL_O += $(SRC_XARM_CORE_DEBUG_DIR)debug_print.o
ALL_O += $(SRC_XARM_CORE_LINUX_DIR)network.o $(SRC_XARM_CORE_LINUX_DIR)thread.o
ALL_O += $(SRC_XARM_CORE_PORT_DIR)serial.o $(SRC_XARM_CORE_PORT_DIR)socket.o
ALL_O += $(SRC_XARM_CORE_INSTRUCTION_DIR)uxbus_cmd.o $(SRC_XARM_CORE_INSTRUCTION_DIR)uxbus_cmd_ser.o $(SRC_XARM_CORE_INSTRUCTION_DIR)uxbus_cmd_tcp.o
ALL_O += $(SRC_XARM_WRAPPER_DIR)xarm_api.o $(SRC_XARM_WRAPPER_DIR)xarm_api.o

EXAMPLE0_O = $(EXAMPLE_DIR)$(EXAMPLE0)
EXAMPLE1_O = $(EXAMPLE_DIR)$(EXAMPLE1)
EXAMPLE2_O = $(EXAMPLE_DIR)$(EXAMPLE2)
EXAMPLE3_O = $(EXAMPLE_DIR)$(EXAMPLE3)
EXAMPLE4_O = $(EXAMPLE_DIR)$(EXAMPLE4)
EXAMPLE5_O = $(EXAMPLE_DIR)$(EXAMPLE5)
EXAMPLE6_O = $(EXAMPLE_DIR)$(EXAMPLE6)
EXAMPLE7_O = $(EXAMPLE_DIR)$(EXAMPLE7)
EXAMPLE8_O = $(EXAMPLE_DIR)$(EXAMPLE8)
EXAMPLE9_O = $(EXAMPLE_DIR)$(EXAMPLE9)
EXAMPLE10_O = $(EXAMPLE_DIR)$(EXAMPLE10)

ALL_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(ALL_O))))

EXAMPLE0_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE0_O))))
EXAMPLE1_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE1_O))))
EXAMPLE2_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE2_O))))
EXAMPLE3_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE3_O))))
EXAMPLE4_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE4_O))))
EXAMPLE5_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE5_O))))
EXAMPLE6_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE6_O))))
EXAMPLE7_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE7_O))))
EXAMPLE8_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE8_O))))
EXAMPLE9_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE9_O))))
EXAMPLE10_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE10_O))))

all: $(EXAMPLE0) $(EXAMPLE1) $(EXAMPLE2) $(EXAMPLE3) $(EXAMPLE4) $(EXAMPLE5) $(EXAMPLE6) $(EXAMPLE7) $(EXAMPLE8) $(EXAMPLE9) $(EXAMPLE10)

$(EXAMPLE0): $(ALL_OBJ) $(EXAMPLE0_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map

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

$(EXAMPLE10): $(ALL_OBJ) $(EXAMPLE10_OBJ)
	mkdir -p $(dir $@)
	$(CCC) -o $(CFLAGS) $^ -o $(BUILDDIR)/$@ $(LIBS) -Wl,-Map,$(BUILDDIR)/$@.map


$(BUILDDIR)/%.o: %.cc
	mkdir -p $(dir $@)
	$(CCC) -c $(CFLAGS) $< -o $@

clean:
	rm -rf ./build
	rm -rf ./xarm

