# Installs packages defined in conanfile.txt
conan install . --output-folder=build_debug --build=missing -s build_type=Debug
