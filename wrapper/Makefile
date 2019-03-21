BUILDDIR = build

EXAMPLE0 = test
EXAMPLE1 = test_events
EXAMPLE2 = test_get
EXAMPLE3 = test_set
EXAMPLE4 = test_move_line
EXAMPLE5 = test_move_joint

SRC_DIR	= ../xarm/
SRC_IDIR = ../

EXAMPLE_DIR = ./example/
COMMON_DIR = $(SRC_DIR)common/
DEBUG_DIR = $(SRC_DIR)debug/
INSTRUCTION_DIR = $(SRC_DIR)instruction/
LINUX_DIR	= $(SRC_DIR)linux/
PORT_DIR = $(SRC_DIR)port/
WRAPPER_DIR = $(SRC_DIR)wrapper/


INCDIRS = -I$(SRC_IDIR)

CCC = g++
C_DEFS = -DSOFT_VERSION=$(SOFT_VERSION)
CFLAGS = -std=c++0x -Wall -g -s $(C_DEFS) $(INCDIRS) $(LIBDIRS)
LIBS += -lm -lpthread

ALL_O = $(COMMON_DIR)crc16.o $(COMMON_DIR)queue_memcpy.o
ALL_O += $(DEBUG_DIR)debug_print.o
ALL_O += $(LINUX_DIR)network.o $(LINUX_DIR)thread.o
ALL_O += $(PORT_DIR)serial.o $(PORT_DIR)socket.o
ALL_O += $(INSTRUCTION_DIR)uxbus_cmd.o $(INSTRUCTION_DIR)uxbus_cmd_ser.o $(INSTRUCTION_DIR)uxbus_cmd_tcp.o
ALL_O += $(WRAPPER_DIR)xarm_api.o $(WRAPPER_DIR)xarm_api.o $(WRAPPER_DIR)events.o $(WRAPPER_DIR)gpio.o $(WRAPPER_DIR)config.o

EXAMPLE0_O = $(EXAMPLE_DIR)$(EXAMPLE0)
EXAMPLE1_O = $(EXAMPLE_DIR)$(EXAMPLE1)
EXAMPLE2_O = $(EXAMPLE_DIR)$(EXAMPLE2)
EXAMPLE3_O = $(EXAMPLE_DIR)$(EXAMPLE3)
EXAMPLE4_O = $(EXAMPLE_DIR)$(EXAMPLE4)
EXAMPLE5_O = $(EXAMPLE_DIR)$(EXAMPLE5)

ALL_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(ALL_O))))

EXAMPLE0_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE0_O))))
EXAMPLE1_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE1_O))))
EXAMPLE2_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE2_O))))
EXAMPLE3_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE3_O))))
EXAMPLE4_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE4_O))))
EXAMPLE5_OBJ = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(EXAMPLE5_O))))

all: $(EXAMPLE0) $(EXAMPLE1) $(EXAMPLE2) $(EXAMPLE3) $(EXAMPLE4) $(EXAMPLE5)

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


$(BUILDDIR)/%.o: %.cc
	mkdir -p $(dir $@)
	$(CCC) -c $(CFLAGS) $< -o $@

clean:
	rm -rf ./build
	rm -rf ./xarm

