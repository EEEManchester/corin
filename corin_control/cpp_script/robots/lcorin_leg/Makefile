REMOVE = rm -f

DIR_BIN  = ./bin
DIR_OBJS = $(DIR_BIN)
DIR_DEPS = $(DIR_OBJS)
OUT_DIRS = $(DIR_BIN) $(DIR_OBJS) $(DIR_DEPS)

CPPFLAGS = -I$(EIGEN_ROOT) -I$(IIT_RBD_ROOT)
CXXFLAGS = -g -Wall -O3 -march=native -mtune=native -D EIGEN_NO_DEBUG

SRCS = $(wildcard *.cpp)
OBJS = $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(SRCS))

EXE_OBJS = $(addprefix $(DIR_OBJS)/,test.o benchmarkID.o main_sine_task_ID.o test_jsim.o)
LIB_OBJS = $(filter-out $(EXE_OBJS), $(OBJS))

EXES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_BIN)/%,$(EXE_OBJS))

LIB = liblcorin_leg.a

all : $(OBJS)

# ------------ #
# OBJECT FILES #
# ------------ #
COMPILE = $(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@

$(OBJS) : | $(sort $(DIR_OBJS) $(DIR_DEPS))
$(OBJS) : $(DIR_OBJS)/%.o : %.cpp
	@echo "   * Compiling $< ..."
	$(COMPILE)

# -------- #
# BINARIES #
# -------- #
BUILD = $(CXX) $(CXXFLAGS) $^ -o $@
executables : $(EXES)
$(EXES) : | $(DIR_BIN)
$(DIR_BIN)/benchmarkID : $(addprefix $(DIR_OBJS)/,benchmarkID.o inverse_dynamics.o inertia_properties.o transforms.o)
	@echo "   * Building binary $@"
	$(BUILD)

$(DIR_BIN)/test : $(addprefix $(DIR_OBJS)/,test.o inverse_dynamics.o inertia_properties.o transforms.o)
	@echo "   * Building binary $@"
	$(BUILD)

$(DIR_BIN)/main_sine_task_ID : $(addprefix $(DIR_OBJS)/,main_sine_task_ID.o inverse_dynamics.o inertia_properties.o transforms.o)
	@echo "   * Building binary $@"
	$(BUILD)

$(DIR_BIN)/test_jsim : $(addprefix $(DIR_OBJS)/,test_jsim.o jsim.o inertia_properties.o transforms.o)
	@echo "   * Building binary $@"
	$(BUILD)

# ------- #
# LIBRARY #
# ------- #
lib: $(DIR_BIN)/$(LIB) | $(DIR_BIN)
$(DIR_BIN)/$(LIB) : $(LIB_OBJS)
	@echo "   * Building library $@ ($(notdir $^))"
	@ar cr $@ $^

DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJS))
-include $(DEPS_FILES)

$(sort $(OUT_DIRS)) :
	@mkdir -p $@

clean :
	@$(REMOVE) $(OBJS) $(DEPS_FILES) $(DIR_BIN)/$(LIB) $(EXES)

debug:
	@echo $(LIB_OBJS)

.PHONY = all lib clean debug binaries
