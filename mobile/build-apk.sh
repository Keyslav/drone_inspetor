#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
TOOLS_DIR="$PWD/.android-tools"
export ANDROID_HOME="${ANDROID_HOME:-$TOOLS_DIR/sdk}"
export ANDROID_USER_HOME="${ANDROID_USER_HOME:-$TOOLS_DIR/android-user}"
export GRADLE_USER_HOME="${GRADLE_USER_HOME:-$TOOLS_DIR/gradle-home}"
if [ -x "$TOOLS_DIR/gradle-8.7/bin/gradle" ]; then
    exec "$TOOLS_DIR/gradle-8.7/bin/gradle" --no-daemon assembleDebug lintDebug
fi
exec ./gradlew --no-daemon assembleDebug lintDebug
