#!/usr/bin/env bash
# 새 PC에서 RosettaDrone 빌드 환경을 되살린다.
# 2026-08-28 에 실제로 빌드가 성공한 조합 그대로다. 버전을 바꾸면 깨진다.
#
# 쓰는 법:  bash setup_new_pc.sh
# 걸리는 시간: 다운로드 포함 20~40분 (회선에 따라)

set -u
WS="$HOME/drone_ws"
SEED="$(cd "$(dirname "$0")" && pwd)"     # 이 스크립트가 있는 폴더 = 씨앗
SDK="$HOME/Android/Sdk"

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()  { printf '   \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '   \033[31m✗\033[0m %s\n' "$*"; }

mkdir -p "$WS" && cd "$WS"

# ---------------------------------------------------------------- 0. 준비물
say "0. 준비물 확인"
for c in git curl unzip tar python3; do
  command -v "$c" >/dev/null && ok "$c" || { bad "$c 없음 → sudo apt install $c"; MISSING=1; }
done
[ "${MISSING:-0}" = 1 ] && { bad "위 도구를 먼저 설치할 것"; exit 1; }

# sdkmanager 는 Java 17+ 를 요구하고, Gradle 6.8.3 은 Java 16+ 에서 못 돈다.
# → 자바를 두 개 두고 용도별로 나눠 쓴다. 시스템 기본은 건드리지 않는다.
SYS_JAVA_VER=$(java -version 2>&1 | head -1 | grep -oE '"[0-9]+' | tr -d '"')
if [ "${SYS_JAVA_VER:-0}" -ge 17 ] 2>/dev/null; then
  ok "시스템 자바 $SYS_JAVA_VER (SDK 도구용으로 충분)"
else
  bad "시스템 자바가 17 미만이다 → sudo apt install openjdk-21-jdk"
  bad "  (빌드용 자바 11 은 아래에서 따로 받는다. 이건 SDK 다운로더용이다)"
  exit 1
fi

# ---------------------------------------------------------------- 1. 빌드용 JDK 11
say "1. 빌드용 JDK 11 (Temurin) — 시스템 기본은 안 건드림"
if [ -x "$WS/jdk/bin/javac" ]; then
  ok "이미 있음: $("$WS/jdk/bin/java" -version 2>&1 | head -1)"
else
  curl -fL -o temurin11.tar.gz \
    "https://api.adoptium.net/v3/binary/latest/11/ga/linux/x64/jdk/hotspot/normal/eclipse" \
    && mkdir -p jdk && tar xzf temurin11.tar.gz -C jdk --strip-components=1 \
    && ok "설치됨: $("$WS/jdk/bin/java" -version 2>&1 | head -1)" \
    || { bad "JDK 11 다운로드 실패 → https://adoptium.net 에서 Temurin 11 (linux x64) 수동 다운로드"; exit 1; }
fi
export JAVA_HOME="$WS/jdk"

# ---------------------------------------------------------------- 2. Android SDK
say "2. Android SDK (정확히 이 5개만 필요)"
if [ ! -x "$SDK/cmdline-tools/latest/bin/sdkmanager" ]; then
  curl -fL -o cmdline-tools.zip \
    "https://dl.google.com/android/repository/commandlinetools-linux-15859902_latest.zip" \
    || { bad "cmdline-tools 다운로드 실패"
         bad "  → https://developer.android.com/studio#command-line-tools-only 에서"
         bad "     Linux 용 zip 을 받아 cmdline-tools.zip 이름으로 $WS 에 두고 다시 실행"; exit 1; }
  mkdir -p "$SDK/cmdline-tools"
  unzip -q cmdline-tools.zip -d "$SDK/cmdline-tools"
  mv "$SDK/cmdline-tools/cmdline-tools" "$SDK/cmdline-tools/latest"
fi
ok "sdkmanager 준비됨"

# sdkmanager 는 시스템 자바(17+)로 돌린다 — JAVA_HOME 을 잠깐 비운다
yes | env -u JAVA_HOME "$SDK/cmdline-tools/latest/bin/sdkmanager" --licenses >/dev/null 2>&1
env -u JAVA_HOME "$SDK/cmdline-tools/latest/bin/sdkmanager" \
    "platforms;android-33" "build-tools;30.0.2" "ndk;22.0.7026061" \
    "cmake;3.10.2.4988404" "platform-tools" \
  && ok "SDK 5종 설치 완료" || { bad "SDK 설치 실패"; exit 1; }

# ---------------------------------------------------------------- 3. 소스
say "3. RosettaDrone 소스 (master, 커밋 805ca9e — 이 커밋으로 고정한다)"
if [ -d "$WS/rosettadrone/.git" ]; then
  ok "이미 있음"
else
  git clone https://github.com/RosettaDrone/rosettadrone.git \
    && git -C rosettadrone checkout 805ca9e4525e3ae741877cf912883cd6e85e5478 \
    && ok "받음" || { bad "clone 실패"; exit 1; }
fi
# 대용량 파일이 실물로 왔는지 확인 (안 오면 빌드는 되는데 앱이 실행 즉시 죽는다)
BIG=$(find rosettadrone -name '*.so' -o -name '*.aar' 2>/dev/null | head -1)
[ -n "$BIG" ] && [ "$(stat -c%s "$BIG" 2>/dev/null || echo 0)" -gt 10000 ] \
  && ok "대용량 파일 실물 확인" || bad "⚠ 대용량 파일이 포인터일 수 있다 — 앱이 실행 즉시 죽으면 이걸 의심"

# ---------------------------------------------------------------- 4. 인증키
say "4. 인증키 심기"
cp "$SEED/secrets/keys.xml" "$WS/rosettadrone/app/src/main/res/values/keys.xml" \
  && ok "keys.xml 배치됨"
git -C rosettadrone check-ignore -q app/src/main/res/values/keys.xml \
  && ok "git 이 무시함 (실수로 커밋될 위험 없음)" \
  || bad "⚠ git 이 keys.xml 을 추적한다 — 커밋하지 말 것"

# ------------------------------------------------------- 4.5 패치 (필수!)
# 2026-09-10: 이 단계가 없어서 「스크립트를 돌렸는데 앱이 그대로 죽는다」가 될 뻔했다.
# 소스 원본은 영상이 동작하지 않는다 — Manifest 에서 VideoService 가 주석 처리돼 있다.
say "4.5 패치 적용 (patches/01~04 — 없으면 영상이 동작하지 않는다)"
cd "$WS/rosettadrone"
PATCH_FAIL=0
for f in 01-noparse-nullguard 02-manifest-videoservice \
         03-sys-status-voltage 04-jmethodid-nullguard; do
  if git apply --check "$SEED/patches/$f.patch" 2>/dev/null; then
    git apply "$SEED/patches/$f.patch" && ok "$f"
  elif git apply --reverse --check "$SEED/patches/$f.patch" 2>/dev/null; then
    ok "$f (이미 적용돼 있음)"
  else
    bad "✗ $f 적용 실패"; PATCH_FAIL=1
  fi
done
[ "$PATCH_FAIL" = "0" ] \
  && ok "패치 4개 준비됨" \
  || { bad "패치가 안 붙었다 — 이대로 빌드하면 영상이 안 된다. 중단한다"; exit 1; }
cd "$WS"

# ---------------------------------------------------------------- 5. 빌드
say "5. 빌드 (Gradle 6.8.3 · Java 11 · 약 2분)"
cd "$WS/rosettadrone"

# Gradle 은 SDK 위치를 local.properties 나 ANDROID_SDK_ROOT 에서만 찾는다.
# 이게 없으면  "SDK location not found"  로 멈춘다 (2026-09-10 에 실제로 걸렸다 —
# 8/28 에는 셸에 손으로 넣고 빌드해서 그 사실이 기록에 안 남아 있었다).
if [ ! -f local.properties ]; then
  echo "sdk.dir=$SDK" > local.properties
  ok "local.properties 생성 (sdk.dir=$SDK)"
fi

ANDROID_SDK_ROOT="$SDK" JAVA_HOME="$WS/jdk" ./gradlew assembleDebug 2>&1 | tail -20
APK="$WS/rosettadrone/app/build/outputs/apk/debug/app-debug.apk"
if [ -f "$APK" ]; then
  ok "APK 생성: $(du -h "$APK" | cut -f1)  ← 190MB 대가 정상"
  # 2026-09-10: 소스를 고친 것과 산출물에 들어간 것은 다르다. APK 자체를 확인한다.
  aapt_v=$(ls "$SDK"/build-tools/*/aapt 2>/dev/null | head -1)
  if [ -n "$aapt_v" ]; then
    "$aapt_v" dump xmltree "$APK" AndroidManifest.xml 2>/dev/null \
      | grep -q "video.VideoService" \
      && ok "Manifest 에 VideoService 선언 확인 (patches/02 반영됨)" \
      || bad "⚠ Manifest 에 VideoService 가 없다 — 영상이 안 된다"
  fi
  SOF="$WS/rosettadrone/app/build/intermediates/cmake/debug/obj/arm64-v8a/libdjivideojni.so"
  NDKBIN=$(ls -d "$SDK"/ndk/*/toolchains/llvm/prebuilt/linux-x86_64/bin 2>/dev/null | head -1)
  if [ -f "$SOF" ] && [ -n "$NDKBIN" ]; then
    "$NDKBIN/llvm-objdump" -d "$SOF" 2>/dev/null \
      | awk '/<invokeFrameDataCallback>:/{f=1} f&&/cbnz/{print; exit}' | grep -q cbnz \
      && ok ".so 에 dataCallbackMID 널 가드 확인 (patches/04 반영됨)" \
      || bad "⚠ .so 가 옛 것일 수 있다 — patches/README 의 검증 절 참조"
  fi
  aapt=$(ls "$SDK"/build-tools/*/aapt 2>/dev/null | head -1)
  [ -n "$aapt" ] && "$aapt" dump badging "$APK" 2>/dev/null | grep -q "sq.rogue.rosettadrone" \
    && ok "패키지명 sq.rogue.rosettadrone 확인"
else
  bad "빌드 실패 — 위 로그를 볼 것"; exit 1
fi

# ---------------------------------------------------------------- 6. QGC
say "6. QGroundControl 4.4.5 (libfuse2 없이 쓰려고 미리 압축을 푼다)"
mkdir -p "$WS/qgc" && cd "$WS/qgc"
if [ -d qgc445 ]; then ok "이미 있음"
elif curl -fL -o QGroundControl-4.4.5.AppImage \
    "https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.5/QGroundControl.AppImage"; then
  chmod +x QGroundControl-4.4.5.AppImage
  ./QGroundControl-4.4.5.AppImage --appimage-extract >/dev/null 2>&1 && mv squashfs-root qgc445 \
    && ok "압축 해제 완료 → 실행: $WS/qgc/qgc445/AppRun" \
    || bad "압축 해제 실패 — AppImage 를 직접 실행해 볼 것"
else
  bad "QGC 다운로드 실패 → https://github.com/mavlink/qgroundcontrol/releases/tag/v4.4.5 에서 수동"
fi

# ---------------------------------------------------------------- 7. 방화벽 (새 PC의 함정)
say "7. ⚠ 방화벽 — 새 PC 에서 되살아나는 함정"
cd "$WS"
if command -v ufw >/dev/null; then
  echo "   현재 상태:"; sudo ufw status verbose 2>/dev/null | head -4 || echo "   (sudo 필요)"
  cat <<'EOF'
   막혀 있으면 아래 두 줄. 증상이 고약하다 — UDP 를 말없이 버려서
   앱은 정상으로 보이는데 QGC 만 텅 빈다. 에러 메시지가 없다.
     sudo ufw allow 14550/udp comment 'MAVLink RosettaDrone'
     sudo ufw allow 5600/udp  comment 'RosettaDrone video'
EOF
else
  ok "ufw 없음 — 이 함정은 해당 없음"
fi

# ---------------------------------------------------------------- 8. 진단 도구
say "8. 진단 도구 배치"
cp "$SEED/tools/"*.py "$WS/" && chmod +x "$WS"/*.py && ok "udp_probe / decode_check / uplink_test"

say "끝났습니다"
cat <<EOF

  APK      : $APK
  QGC      : $WS/qgc/qgc445/AppRun
  기록     : $SEED/진행상황.md

  ⚠ 손으로 고쳐야 하는 것 하나:
     $SEED/tools/ 의 스크립트 안 GCS_IP 가 172.20.10.9 (옛 핫스팟)로 박혀 있습니다.
     새 PC 의 IP 로 바꾸세요 →  ip -4 addr | grep inet

  다음 할 일은 진행상황.md 의 「다음에 할 일」 을 볼 것.
EOF
