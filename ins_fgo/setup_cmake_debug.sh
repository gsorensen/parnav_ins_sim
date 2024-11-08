# Sets up the cmake project.
# To build, simply run build.sh
#
cd build_debug
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -GNinja
cd ..
ln -s build_debug/compile_commands.json
