# 앱 소스 패치

RosettaDrone `master` / 커밋 `805ca9e4525e3ae741877cf912883cd6e85e5478` 기준입니다.
소스는 이 저장소에 없습니다(1.3GB). `setup_new_pc.sh` 가 같은 커밋으로 받아옵니다.

## 적용

```bash
cd ~/drone_ws/rosettadrone
git apply ~/rosettadrone_work/patches/01-noparse-nullguard.patch
ANDROID_SDK_ROOT=$HOME/Android/Sdk JAVA_HOME=~/drone_ws/jdk ./gradlew assembleDebug
```

되돌리려면 `git apply -R` 입니다.

⚠ `ANDROID_SDK_ROOT` 없이는 **"SDK location not found"** 로 멈춥니다
(`local.properties` 가 없기 때문. `setup_new_pc.sh` 가 이제 만들어 줍니다).

## ⚠ C 코드 패치는 `.so` 가 진짜 다시 빌드됐는지 확인할 것

자바만 재컴파일되고 옛 `.so` 가 그대로 포장되면 **크래시가 그대로 재현되어
「패치가 틀렸다」고 오판하게 됩니다.** 기체가 다른 자리에 있으면 헛걸음이 큽니다.

```bash
# ① 오늘 다시 링크됐나
ls -la app/build/intermediates/stripped_native_libs/debug/out/lib/arm64-v8a/libdjivideojni.so
# ② 기계어에 진짜 들어갔나 (가장 확실하다)
NDK=~/Android/Sdk/ndk/22.0.7026061/toolchains/llvm/prebuilt/linux-x86_64/bin
$NDK/llvm-objdump -d app/build/intermediates/cmake/debug/obj/arm64-v8a/libdjivideojni.so \
  | grep -A45 "<noparse>:"
```

## 목록

| 패치 | 고치는 것 | 상태 |
|---|---|---|
| `01-noparse-nullguard.patch` | **함정 5** — 영상을 켜면 Mavic 2 에서 앱이 SIGSEGV 로 죽는다 | ✅ **2026-09-10 적용·빌드 완료** → Release `apk-2026-09-10-video01` |

### 01 의 빌드 검증 결과 (2026-09-10)

```
<noparse>:
    1924: str  wzr, [sp, #32]     ← frame_num  = 0
    1930: str  wzr, [sp, #20]     ← height     = 0
    193c: cbz  x9, 0x1974         ★ m_pCodecPaser == NULL 이면 통째로 건너뛴다
    1948: ldr  w9, [x9, #328]     ★ 328 = 0x148 — logcat 의 fault addr 그 자리
```
**9/9 logcat 의 `fault addr 0x148` 이 저 `#328` 오프셋이고, 이제 `cbz` 뒤에 있습니다.**

⚠ 다만 이것은 **크래시가 막혔다는 증명일 뿐, 영상이 흐른다는 증명은 아닙니다.**
그건 `tools/video_probe.py` 로 실기에서 따로 봐야 합니다.

### 01 — `noparse()` 의 널 역참조

`app/src/main/cpp/dji_video_jni.c` 의 `noparse()` 가 `m_pCodecPaser` 를
**널 검사 없이** 역참조한다. 바로 위 `parse()` 에는 가드가 있는데 여기만 빠졌다.

터지는 경로 (2026-09-09 에 logcat 으로 확정):

| # | 어디 | 무슨 일 |
|---|---|---|
| 1 | `MainActivity.java:963` | Mavic 2 → `getVideoMode()` 가 **1** 을 반환 (Phantom·Mini·Air 계열은 2) |
| 2 | `MainActivity.java:867` | `m_videoMode != 2` → `parserMode = 1` |
| 3 | `MainActivity.java:872` | 영상 콜백(**CommonThreadPool**)에서 `NativeHelper.getInstance().parse(buf, size, 1)` |
| 4 | `NativeHelper.java` | 이 호출이 싱글턴을 처음 만든다 → 생성자의 `Log.e("NativeHelper")` |
| 5 | `dji_video_jni.c` | `mode != 0` → `noparse()` |
| 6 | `noparse()` | `m_pCodecPaser->frame_num` … → **SIGSEGV, fault addr `0x148`** |

**왜 항상 널인가.** `MainActivity.initVideoService()` 가 `startService()` 뒤에
**`safeSleep(500)` 으로 메인 스레드를 막는다.** `VideoService.onCreate()`
→ `NativeHelper.init()` 은 그 메인 스레드 큐에 걸려 있어서, 메인 스레드가 루퍼로
돌아갈 때까지 못 돈다. 그런데 그 전에 영상 리스너가 등록되어 첫 프레임이
`CommonThreadPool` 로 들어온다. **경쟁이 아니라 확정적으로 지는 순서** — 그래서 11/11 재현.

**왜 0 을 넣어도 되는가.** 그 네 값(`frame_num`·`key_frame`·`width`·`height`)은
`VideoService.onDataRecv()` 로 전달되는데 거기서 **전부 무시된다.** 버퍼만 쓴다:

```java
public void onDataRecv(byte[] data, int size, int frameNum, boolean isKeyFrame, int width, int height) {
    if (size > 0 && isRunning) {
        splitNALs(data);      // data 만 쓴다
    }
}
```

즉 **읽어봐야 아무 데도 안 쓰이는 값을 읽다가 죽고 있었다.**

⚠ 이 패치는 크래시만 없앤다. 그 뒤에 **영상이 실제로 흐르는지는 따로 확인**해야 한다
(`VideoService.isRunning` 은 `setParameters()` 가 불려야 true 가 된다).
