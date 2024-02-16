#include <android/asset_manager_jni.h>
#include <android/log.h>
#include <jni.h>

#include "NcnnProcessor.h"

const char *TAG = "NativeNcnnProcessor";

static NcnnProcessor *ncnnProcessor;

extern "C" {
JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved) {
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "JNI_OnLoad");

    return JNI_VERSION_1_4;
}

JNIEXPORT void JNI_OnUnload(JavaVM *vm, void *reserved) {
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "JNI_OnUnload");

    if (ncnnProcessor) {
        delete ncnnProcessor;
    }
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_utils_ncnnprocessor_NcnnProcessor_CreateRawInstance(
        JNIEnv *env, jobject thiz, jobject assetManager, jstring paramPath, jstring modelPath,
        jintArray modelInputSize, jfloatArray meanValues, jfloatArray normValues,
        jfloat confidenceThreshold, jint threadNum
) {
    AAssetManager *mgr = AAssetManager_fromJava(env, assetManager);
    const char *paramAssetPath = env->GetStringUTFChars(paramPath, JNI_FALSE);
    const char *modelAssetPath = env->GetStringUTFChars(modelPath, JNI_FALSE);

    __android_log_print(ANDROID_LOG_INFO, TAG,
                        "Loading model. modelPath: %s",
                        modelAssetPath
    );

    assert(env->GetArrayLength(modelInputSize) == 2);
    assert(env->GetArrayLength(meanValues) == 3);
    assert(env->GetArrayLength(normValues) == 3);

    ncnnProcessor = new NcnnProcessor(mgr, paramAssetPath, modelAssetPath,
                                      *reinterpret_cast<std::array<int, 2> *>(
                                              env->GetIntArrayElements(modelInputSize,
                                                                       JNI_FALSE)),
                                      *reinterpret_cast<std::array<float, 3> *>(
                                              env->GetFloatArrayElements(meanValues, JNI_FALSE)),
                                      *reinterpret_cast<std::array<float, 3> *>(
                                              env->GetFloatArrayElements(normValues, JNI_FALSE)),
                                      confidenceThreshold, threadNum);
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_utils_ncnnprocessor_NcnnProcessor_RawInit(JNIEnv *env,
                                                                              jobject thiz,
                                                                              jint frameWidth,
                                                                              jint frameHeight) {
    ncnnProcessor->init(frameWidth, frameHeight);
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_utils_ncnnprocessor_NcnnProcessor_RawProcessFrame(JNIEnv *env,
                                                                                      jobject thiz,
                                                                                      long frameAddress) {
    ncnnProcessor->processFrame(*(cv::Mat *) frameAddress);
}

JNIEXPORT jobjectArray JNICALL
Java_org_firstinspires_ftc_teamcode_utils_ncnnprocessor_NcnnProcessor_RawGetDetections(JNIEnv *env,
                                                                                       jobject thiz) {
    jclass jDetectedObject = env->FindClass(
            "org/firstinspires/ftc/teamcode/utils/ncnnprocessor/DetectedObject");
    jclass jRect2d = env->FindClass(
            "org/opencv/core/Rect2d");
    jmethodID objectConstructor = env->GetMethodID(jDetectedObject, "<init>",
                                                   "(Lorg/opencv/core/Rect2d;IF)V");
    jmethodID rectConstructor = env->GetMethodID(jRect2d, "<init>", "(DDDD)V");

    jobjectArray jDetections = env->NewObjectArray(ncnnProcessor->detections.size(),
                                                   jDetectedObject, NULL);

    for (int i = 0; i < ncnnProcessor->detections.size(); ++i) {
        DetectedObject object = ncnnProcessor->detections[i];
        jobject jRect = env->NewObject(
                jRect2d, rectConstructor,
                (double) object.rect.x, (double) object.rect.y,
                (double) object.rect.width, (double) object.rect.height
        );
        jobject jDetection = env->NewObject(
                jDetectedObject, objectConstructor,
                jRect,
                object.label, object.confidence
        );
        env->SetObjectArrayElement(jDetections, i, jDetection);
    }

    return jDetections;
}
}