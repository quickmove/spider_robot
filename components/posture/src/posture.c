#include "spider_posture.h"

#include <string.h>

/*
 * 内置姿态。
 *
 * height_mm 是唯一的自由度——2DOF 构型下四膝即机身高度，
 * 髋偏移只用于外观与表达（如 play_dead 的收腿、wave 的单腿抬起）。
 *
 * SPAWN 是仰卧姿态：髋回中立、膝压到下限，腿朝天。
 */
static const posture_def_t POSTURES[] = {
    /* REST  腹部近地 */
    { "rest",  SPIDER_REST_HEIGHT_MM,  {  12.0f,  12.0f,  12.0f,  12.0f } },
    /* STAND 标准站立 */
    { "stand", SPIDER_STAND_HEIGHT_MM, {   0.0f,   0.0f,   0.0f,   0.0f } },
    /* SIT   蹲坐：机身压低、腿内收 */
    { "sit",   SPIDER_SIT_HEIGHT_MM,   {  -8.0f,  -8.0f,  -8.0f,  -8.0f } },
    /* SPAWN 翻车：腿朝天 */
    { "spawn", SPIDER_SPAWN_HEIGHT_MM, {   0.0f,   0.0f,   0.0f,   0.0f } },
};

#define POSTURE_COUNT  (sizeof(POSTURES) / sizeof(POSTURES[0]))

const posture_def_t *posture_get(posture_id_t id)
{
    if ((size_t)id >= POSTURE_COUNT) {
        return NULL;
    }
    return &POSTURES[id];
}

const char *posture_name(posture_id_t id)
{
    const posture_def_t *p = posture_get(id);
    return p ? p->name : "?";
}

bool posture_from_name(const char *name, posture_id_t *out)
{
    if (name == NULL || out == NULL) {
        return false;
    }
    for (size_t i = 0; i < POSTURE_COUNT; i++) {
        if (strcmp(name, POSTURES[i].name) == 0) {
            *out = (posture_id_t)i;
            return true;
        }
    }
    return false;
}