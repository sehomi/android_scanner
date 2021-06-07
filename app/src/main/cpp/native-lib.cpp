#include <jni.h>
#include <string>
#include "scanner.h"

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_android_1scanner_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    Scanner sc;
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}