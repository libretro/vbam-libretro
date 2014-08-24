TILED_RENDERING = 0

ifeq ($(platform),)
platform = unix
ifeq ($(shell uname -a),)
   platform = win
else ifneq ($(findstring MINGW,$(shell uname -a)),)
   platform = win
else ifneq ($(findstring win,$(shell uname -a)),)
   platform = win
else ifneq ($(findstring Darwin,$(shell uname -a)),)
   platform = osx
   arch = intel
ifeq ($(shell uname -a),)
   arch = ppc
endif
endif
endif

TARGET_NAME = vbam

ifeq ($(platform), unix)
   TARGET := $(TARGET_NAME)_libretro.so
   fpic := -fPIC
   SHARED := -shared
	TILED_RENDERING = 1
else ifeq ($(platform), osx)
   TARGET := $(TARGET_NAME)_libretro.dylib
   fpic := -fPIC
   SHARED := -dynamiclib
   arch = intel
ifeq ($(shell uname -a),)
   arch = ppc
endif

ifneq ($(arch),ppc)
   SHARED += -DLSB_FIRST
endif
OSXVER = `sw_vers -productVersion | cut -c 4`
ifneq ($(OSXVER),9)
   fpic += -mmacosx-version-min=10.5
endif
else ifeq ($(platform), ios)
   TARGET := $(TARGET_NAME)_libretro_ios.dylib
   fpic := -fPIC
   SHARED := -dynamiclib -DLSB_FIRST

   CC = clang -arch armv7 -isysroot $(IOSSDK)
   CXX = clang++ -arch armv7 -isysroot $(IOSSDK)
OSXVER = `sw_vers -productVersion | cut -c 4`
ifneq ($(OSXVER),9)
   CC += -miphoneos-version-min=5.0
   CXX += -miphoneos-version-min=5.0
   SHARED += -miphoneos-version-min=5.0
endif
	TILED_RENDERING = 1
else ifeq ($(platform), qnx)
   TARGET := $(TARGET_NAME)_libretro_qnx.so
   fpic := -fPIC
   SHARED := -shared
	TILED_RENDERING = 1
	CC = qcc -Vgcc_ntoarmv7le
	CXX = QCC -Vgcc_ntoarmv7le_cpp
	AR = QCC -Vgcc_ntoarmv7le
else
   TARGET := $(TARGET_NAME)_libretro.dll
   LDFLAGS += -Wl,-no-undefined -Wl,--version-script=link.T
   CC = gcc
   CXX = g++
   SHARED := -shared -static-libgcc -static-libstdc++
endif

ifeq ($(TILED_RENDERING), 1)
VBA_DEFINES += -DTILED_RENDERING
endif

VBA_DIR := ../

VBA_SRC_DIRS := $(VBA_DIR)/gba $(VBA_DIR)/apu 

VBA_CXXSRCS := $(foreach dir,$(VBA_SRC_DIRS),$(wildcard $(dir)/*.cpp))
VBA_CXXOBJ := $(VBA_CXXSRCS:.cpp=.o) ../common/Patch.o
VBA_CSRCS := $(foreach dir,$(VBA_SRC_DIRS),$(wildcard $(dir)/*.c))
VBA_COBJ := $(VBA_CSRCS:.c=.o)
UTIL_SOURCES := 
UTIL_OBJS := $(UTIL_SOURCES:.c=.o)

OBJS := $(VBA_COBJ) $(VBA_CXXOBJ) $(UTIL_OBJS) libretro.o UtilRetro.o SoundRetro.o scrc32.o

VBA_DEFINES += -D__LIBRETRO__ -DFINAL_VERSION -DC_CORE -DUSE_GBA_ONLY -DNO_LINK
VBA_DEFINES += -DFRONTEND_SUPPORTS_RGB565

CFLAGS += -O3 -std=gnu99 $(fpic) $(VBA_DEFINES)
CXXFLAGS += -O3 $(fpic) $(VBA_DEFINES)


INCDIRS := -I$(VBA_DIR)
LIBS :=

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(SHARED) $(OBJS) $(LDFLAGS) $(LIBS)

%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(INCDIRS)

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS) $(INCDIRS)

clean:
	rm -f $(OBJS)
	rm -f $(TARGET)

.PHONY: clean


