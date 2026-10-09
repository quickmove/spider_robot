#!/bin/sh
# 主机端单元测试：不烧板即可验证纯计算组件（kinematics 等）
# 用法: ./tools/run_tests.sh
set -e
DIR=$(cd "$(dirname "$0")/.." && pwd)
CC=${CC:-cc}
CFLAGS="-std=gnu17 -O2 -Wall -Wextra -Werror"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

status=0
run_test() {
    name=$1; shift
    src=$1; shift
    printf '\n=== %s ===\n' "$name"
    # shellcheck disable=SC2086
    # shellcheck disable=SC2086
    if $CC $CFLAGS $ALL_INC "$DIR/$src" $EXTRA_SRC -o "$TMP/$name" -lm; then
        "$TMP/$name" || status=1
    else
        echo "编译失败: $name"
        status=1
    fi
}

KIN_INC="-I $DIR/components/kinematics/include"
SERVO_INC="-I $DIR/components/servo/include"
PCA_INC="-I $DIR/components/pca9685/include"
GAIT_INC="-I $DIR/components/gait/include"
POST_INC="-I $DIR/components/posture/include"
ALL_INC="$KIN_INC $SERVO_INC $PCA_INC $GAIT_INC $POST_INC"

EXTRA_SRC="$DIR/components/kinematics/src/kinematics.c" \
    run_test test_kinematics components/kinematics/test/test_kinematics.c

EXTRA_SRC="$DIR/components/servo/src/servo_logic.c" \
    run_test test_servo components/servo/test/test_servo.c

EXTRA_SRC="$DIR/components/pca9685/src/pca9685_math.c" \
    run_test test_pca9685 components/pca9685/test/test_pca9685.c

EXTRA_SRC="$DIR/components/gait/src/gait.c $DIR/components/kinematics/src/kinematics.c" \
    run_test test_gait components/gait/test/test_gait.c

EXTRA_SRC="$DIR/components/posture/src/kf_build.c" \
    run_test test_kf_build components/posture/test/test_kf_build.c

# 协议一致性（文档 <-> ws_cmd.c）
printf '\n=== protocol_check ===\n'
python3 "$DIR/tools/protocol_check.py" || status=1

# 参数自洽校验（params.yaml -> 头文件）
printf '\n=== params_check ===\n'
python3 "$DIR/tools/gen_params.py" --check --out "$TMP/spider_params.h" || status=1

# 步态仿真验收（Python 与固件同源参数 + 同算法）
if command -v python3 >/dev/null 2>&1 && \
   python3 -c "import yaml, matplotlib" 2>/dev/null; then
    for g in crawl trot; do
        printf '\n=== gait_sim %s ===\n' "$g"
        python3 "$DIR/tools/gait_sim/preview.py" --check-only --gait "$g" || status=1
    done
else
    printf '\n[WARN] gait_sim 跳过：缺少 python3 依赖 (pyyaml, matplotlib)\n'
fi

printf '\n===============================\n'
if [ "$status" -eq 0 ]; then
    echo "全部测试通过"
else
    echo "存在失败用例"
fi
exit $status
