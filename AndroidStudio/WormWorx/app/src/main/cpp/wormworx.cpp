#include <jni.h>
#include <math.h>
#include <android/log.h>
#include "wormsim.h"

#define  LOG_TAG    "libwormworx_jni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern "C" 
{
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_init(JNIEnv * env, jobject obj);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_setSteeringSynapseWeights(JNIEnv * env, jobject obj, jdoubleArray weights);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_step(JNIEnv * env, jobject obj, jdouble salt_stimulus);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getSteeringActivations(JNIEnv * env, jobject obj, jdoubleArray activations);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getDorsalMotorActivations(JNIEnv * env, jobject obj, jdoubleArray activations);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getVentralMotorActivations(JNIEnv * env, jobject obj, jdoubleArray activations);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getDorsalMuscleActivations(JNIEnv * env, jobject obj, jdoubleArray activations);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getVentralMuscleActivations(JNIEnv * env, jobject obj, jdoubleArray activations);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getBody(JNIEnv * env, jobject obj, jdoubleArray body);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getSegmentAngles(JNIEnv * env, jobject obj, jdoubleArray angles);
    JNIEXPORT void JNICALL Java_openworm_WormWorxLib_terminate(JNIEnv * env, jobject obj);
};

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_init(JNIEnv * env, jobject obj)
{
    init();
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_setSteeringSynapseWeights(JNIEnv * env, jobject obj, jdoubleArray weights)
{
    jsize len = env->GetArrayLength(weights);
    jdouble *weightsf = env->GetDoubleArrayElements(weights, 0);
    for (int i = 0; i < len; i++)
    {
        set_steering_synapse_weight(i, weightsf[i]);
    }
    env->ReleaseDoubleArrayElements(weights, weightsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_step(JNIEnv * env, jobject obj, jdouble salt_stimulus)
{
    step(salt_stimulus);
}


JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getSteeringActivations(JNIEnv * env, jobject obj, jdoubleArray activations)
{
    jsize len = env->GetArrayLength(activations);
    jdouble *activationsf = env->GetDoubleArrayElements(activations, 0);
    for (int i = 0; i < len; i++)
    {
        activationsf[i] = get_steering_activation(i);
    }
    env->ReleaseDoubleArrayElements(activations, activationsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getDorsalMotorActivations(JNIEnv * env, jobject obj, jdoubleArray activations)
{
    jsize len = env->GetArrayLength(activations);
    jdouble *activationsf = env->GetDoubleArrayElements(activations, 0);
    for (int i = 0; i < len; i++)
    {
        activationsf[i] = get_dorsal_motor_activation(i);
    }
    env->ReleaseDoubleArrayElements(activations, activationsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getVentralMotorActivations(JNIEnv * env, jobject obj, jdoubleArray activations)
{
    jsize len = env->GetArrayLength(activations);
    jdouble *activationsf = env->GetDoubleArrayElements(activations, 0);
    for (int i = 0; i < len; i++)
    {
        activationsf[i] = get_ventral_motor_activation(i);
    }
    env->ReleaseDoubleArrayElements(activations, activationsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getDorsalMuscleActivations(JNIEnv * env, jobject obj, jdoubleArray activations)
{
    jsize len = env->GetArrayLength(activations);
    jdouble *activationsf = env->GetDoubleArrayElements(activations, 0);
    for (int i = 0; i < len; i++)
    {
        activationsf[i] = get_dorsal_muscle_activation(i);
    }
    env->ReleaseDoubleArrayElements(activations, activationsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getVentralMuscleActivations(JNIEnv * env, jobject obj, jdoubleArray activations)
{
    jsize len = env->GetArrayLength(activations);
    jdouble *activationsf = env->GetDoubleArrayElements(activations, 0);
    for (int i = 0; i < len; i++)
    {
        activationsf[i] = get_ventral_muscle_activation(i);
    }
    env->ReleaseDoubleArrayElements(activations, activationsf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getBody(JNIEnv * env, jobject obj, jdoubleArray body)
{
    jdouble *bodyf = env->GetDoubleArrayElements(body, 0);
    for (int i = 0; i < NBAR; ++i)
    {
        bodyf[i * 3] = get_body_point(i * 3);
        bodyf[i * 3 + 1] = get_body_point(i * 3 + 1);
        bodyf[i * 3 + 2] = get_body_point(i * 3 + 2);
    }
    env->ReleaseDoubleArrayElements(body, bodyf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_getSegmentAngles(JNIEnv * env, jobject obj, jdoubleArray angles)
{
    jdouble *anglesf = env->GetDoubleArrayElements(angles, 0);
    for (int i = 0; i < 12; ++i)
    {
        anglesf[i] = get_segment_angle(i);
    }
    env->ReleaseDoubleArrayElements(angles, anglesf, 0);
}

JNIEXPORT void JNICALL Java_openworm_WormWorxLib_terminate(JNIEnv * env, jobject obj)
{
    terminate();
}
