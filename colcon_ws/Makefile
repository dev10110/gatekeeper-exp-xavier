
# simple colcon build instruction

# Usage:
#
# 1) to build all packages:
#    $ make
#
# 2) to build only 'my_pkg`:
#    $ pkg=my_pkg make

mode=Release
.SILENT: all clean
.PHONY: all clean

all: 
	if [ -z "$(pkg)" ]; then \
		echo "building all"; \
		colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(mode); \
	else \
		echo "building $(pkg)"; \
		colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(mode) --packages-select $(pkg);\
	fi

clean: 
	if [ -z "$(pkg)" ]; then \
		echo "cleaning all"; \
		rm -rf build install; \
	else \
		echo "cleaning $(pkg)"; \
		rm -rf build/$(pkg) install/$(pkg); \
	fi
