# This makefile is for building libdraco.a as a fat binary
# that can be used to produce universal2 wheels.

ARMDIR = draco/build_arm
X86DIR = draco/build_x86

all: staticlib

staticlib: arm64 x86
	lipo -create -output ./libdraco.a $(ARMDIR)/libdraco.a $(X86DIR)/libdraco.a

arm64:
	mkdir -p $(ARMDIR)
	cmake -B $(ARMDIR) -S draco -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DBUILD_SHARED_LIBS=OFF
	cd $(ARMDIR) && make
x86:
	mkdir -p $(X86DIR)
	cmake -B $(X86DIR) -S draco -DCMAKE_OSX_ARCHITECTURES=x86_64 -DCMAKE_OSX_DEPLOYMENT_TARGET=10.9 -DBUILD_SHARED_LIBS=OFF
	cd $(X86DIR) && make

clean:
	rm -rf draco/build_arm draco/build_x86