@echo off
"C:\\AndroidStudio\\cmake\\3.22.1\\bin\\cmake.exe" ^
  "-HC:\\Users\\Tom Portegys\\Documents\\education\\OpenWorm\\WormWorx\\WormWorx++\\app\\src\\main\\cpp" ^
  "-DCMAKE_SYSTEM_NAME=Android" ^
  "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" ^
  "-DCMAKE_SYSTEM_VERSION=23" ^
  "-DANDROID_ABI=armeabi-v7a" ^
  "-DCMAKE_ANDROID_ARCH_ABI=armeabi-v7a" ^
  "-DANDROID_NDK=C:\\AndroidStudio\\ndk\\26.1.10909125" ^
  "-DCMAKE_ANDROID_NDK=C:\\AndroidStudio\\ndk\\26.1.10909125" ^
  "-DCMAKE_TOOLCHAIN_FILE=C:\\AndroidStudio\\ndk\\26.1.10909125\\build\\cmake\\android.toolchain.cmake" ^
  "-DCMAKE_MAKE_PROGRAM=C:\\AndroidStudio\\cmake\\3.22.1\\bin\\ninja.exe" ^
  "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=C:\\Users\\Tom Portegys\\Documents\\education\\OpenWorm\\WormWorx\\WormWorx++\\app\\build\\intermediates\\cxx\\Debug\\5p192423\\obj\\armeabi-v7a" ^
  "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=C:\\Users\\Tom Portegys\\Documents\\education\\OpenWorm\\WormWorx\\WormWorx++\\app\\build\\intermediates\\cxx\\Debug\\5p192423\\obj\\armeabi-v7a" ^
  "-DCMAKE_BUILD_TYPE=Debug" ^
  "-BC:\\Users\\Tom Portegys\\Documents\\education\\OpenWorm\\WormWorx\\WormWorx++\\app\\.cxx\\Debug\\5p192423\\armeabi-v7a" ^
  -GNinja ^
  "-DANDROID_PLATFORM=android-16" ^
  "-DANDROID_TOOLCHAIN=clang" ^
  "-DANDROID_STL=c++_static"
