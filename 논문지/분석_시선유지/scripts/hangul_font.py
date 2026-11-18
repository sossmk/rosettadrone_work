"""matplotlib 에 깔려 있는 한글 폰트를 하나 골라 준다.

그림 라벨이 전부 한글이라 폰트가 없으면 네모(두부)로 나온다. 리눅스와 윈도우에
깔려 있는 이름이 달라서, 이름을 박아 두면 다른 PC 에서 깨진다. 그래서 설치된
것 중에서 고른다.

    from hangul_font import use_hangul_font
    use_hangul_font()
"""
import matplotlib
import matplotlib.pyplot as plt

# 앞에서부터 깔려 있는 것을 쓴다. 리눅스 → 윈도우 → macOS 순
CANDIDATES = [
    "Noto Sans CJK KR",     # 리눅스 (fonts-noto-cjk)
    "NanumGothic",          # 리눅스 (fonts-nanum)
    "Noto Sans KR",         # pip 로 깐 경우
    "Malgun Gothic",        # 윈도우 기본 (맑은 고딕)
    "AppleSDGothicNeo-Regular",  # macOS
    "AppleGothic",          # macOS
]


def installed():
    """matplotlib 이 아는 폰트 이름 집합."""
    fm = matplotlib.font_manager
    try:
        return set(fm.get_font_names())        # matplotlib >= 3.6
    except AttributeError:
        return {f.name for f in fm.fontManager.ttflist}


def use_hangul_font(verbose=False):
    """고른 폰트 이름을 돌려준다. 하나도 없으면 None 이고 라벨이 깨진다."""
    have = installed()
    name = next((n for n in CANDIDATES if n in have), None)
    if name:
        plt.rcParams["font.family"] = name
        if verbose:
            print(f"한글 폰트: {name}")
    else:
        print("한글 폰트를 못 찾았다 — 그림의 한글이 네모로 나온다.\n"
              "  리눅스: sudo apt install fonts-noto-cjk  (또는 fonts-nanum)\n"
              "  윈도우: 맑은 고딕이 기본인데 안 잡히면 "
              "python -c \"import matplotlib;matplotlib.font_manager._load_fontmanager(try_read_cache=False)\"\n"
              f"  직접 지정: plt.rcParams['font.family'] = '이름'  (후보: {', '.join(CANDIDATES)})")
    plt.rcParams["axes.unicode_minus"] = False   # 마이너스 기호가 폰트에 없으면 깨진다
    return name
